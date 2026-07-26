/**********************************************************************************
                SNES RA Adapter - Raspberry Pi Pico 2 Firmware (skeleton)

   This firmware targets the RP2350B (QFN-80, 48 GPIOs) for the Super Nintendo
   RetroAchievements adapter. Pinout v0.2 (see snes-ra-adapter-pcb/docs/
   netlist.md): no duplicated GPIOs -- shared signals live in the PIO overlap
   window GPIO16-31 (PIO0 sees 0-31; PIO1 sees 16-47 via pio_set_gpio_base).

     * PIO0 SM0 (snesBusWram):     A-bus WRAM writes
     * PIO0 SM1 (snesBusCart):     A-bus cart-space writes (SRAM)
     * PIO1 SM0 (snesBusBport):    B-bus $2180-83 WRAM port (DMA writes!)
     * PIO1 SM1 (snesBusCartHigh): A17-A23 for large SRAM (phase 4b, parked)

   Capture is one word per write cycle (sampled at a fixed point inside the
   data-valid window); consumers on core 1 validate the captured control bits
   and update full mirrors: 128 KB WRAM + 32 KB SRAM. Full mirroring (instead
   of the old sparse hashmap) is required for AddAddress/indirect achievements
   whose target addresses are only known at runtime.

   Date:    2026-07-04
   Version: 0.7-skeleton
   By odelot
**********************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include "snes-bus.pio.h"

// rcheevos: MD5 helper, client API, low-level types.
#include "rhash/md5.h"
#include "rc_runtime_types.h"
#include "rc_client.h"
#include "rc_api_runtime.h"
#include "rc_api_user.h"
#include "rc_version.h"

#define FIRMWARE_VERSION "0.7-skeleton"

// =============================================================================
// Pinout v0.2 (must match snes-bus.pio and snes-ra-adapter-pcb/docs/netlist.md)
// =============================================================================
//
// Sem GPIOs duplicados: os sinais compartilhados pelos dois PIOs vivem na
// janela de overlap GPIO 16..31 (PIO0 ve 0..31; PIO1, com gpio_base=16,
// ve 16..47). Taps diretos (sem buffer) sao entradas 5V-tolerant do RP2350B.

// --- Endereco A0..A16 (17 bits contiguos = offset nos 128KB de WRAM) ---
#define SNES_A0            0   // A0..A15 -> GP0..GP15
#define SNES_A15          15
#define SNES_A16          16   // dentro da janela de overlap

// --- Dados (lado cart do U4 CBT3245) ---
#define SNES_D0           17   // D0..D7 -> GP17..GP24
#define SNES_D7           24

// --- Controle (janela de overlap; visivel aos dois PIOs) ---
#define SNES_PHI2         25   // tap direto
#define SNES_WR_N         26   // lado cart do U5 (Pico dirige no cart-ID)
#define SNES_WRAM_N       27   // tap direto
#define SNES_CART_N       28   // lado cart do U5
#define SNES_RD_N         29   // lado cart do U5
#define SNES_REFRESH_N    30   // tap direto (filtro de refresh no consumer)
#define SNES_PAWR_N       31   // tap direto (strobe de escrita do B-bus)

// --- Endereco alto (bancos; cart-ID + fase 4b) ---
#define SNES_A17          32   // A17..A23 -> GP32..GP38
#define SNES_A23          38

// --- Taps do B-bus (decodificacao parcial: PA7=1 so em $2180-83) ---
#define SNES_PA0          39
#define SNES_PA1          40
#define SNES_PA7          41
#define SNES_PARD_N       42

// --- Reset ---
#define SNES_RESET_N      43   // monitor direto da linha /RESET
#define SNES_RESET_DRIVE  44   // saida -> R2 -> Q1
                               //   HIGH = Q1 satura -> /RESET low (console preso)
                               //   LOW  = Q1 corta  -> pull-up do console solta

// --- Bus isolation (/OE comum de U1..U5) ---
//   LOW  = buffers ativos (modo monitor: console ve o cart)
//   HIGH = HiZ (modo cart-ID: console isolado, Pico dirige o cart bus)
//   Default no power-on e' HIGH via pull-up externo de 10k.
#define BUFFER_OE_N       45

// --- ESP32 UART link (UART0 alternate function do RP2350B) ---
#define UART_ID         uart0
#define UART_TX_PIN     46
#define UART_RX_PIN     47
#define UART_BAUD       115200

// GPIOs de barramento/taps inicializados como input pelo core1 (0..43).
// 44/45 sao saidas do main, 46/47 UART.
#define BUS_GPIO_FIRST  0
#define BUS_GPIO_LAST   43

// =============================================================================
// PIO assignments
// =============================================================================

#define BUS_LOW_PIO     pio0
#define BUS_WRAM_SM     0      // snesBusWram  -- escritas WRAM (A-bus)
#define BUS_CART_SM     1      // snesBusCart  -- escritas no espaco do cart
#define BUS_HISRAM_SM   2      // snesBusHiSram -- SRAM de HiROM (sem /CART!)

#define BUS_HIGH_PIO    pio1   // exige pio_set_gpio_base(pio1, 16)!
#define BUS_BPORT_SM    0      // snesBusBport -- porta WRAM $2180-83 (B-bus)
#define BUS_CARTHI_SM   1      // snesBusCartHigh -- fase 4b (desabilitado)

// =============================================================================
// Bit extraction from captured PIO words (pinout v0.2)
// =============================================================================
//
// PIO0 word (in_base=0, bit = GPIO):
#define LOW_ADDR(w)      ((w) & 0x1FFFFu)          // A0..A16
#define LOW_DATA(w)      (((w) >> 17) & 0xFFu)     // D0..D7
#define LOW_PHI2(w)      (((w) >> 25) & 0x1u)
#define LOW_WR_N(w)      (((w) >> 26) & 0x1u)
#define LOW_WRAM_N(w)    (((w) >> 27) & 0x1u)
#define LOW_CART_N(w)    (((w) >> 28) & 0x1u)
#define LOW_RD_N(w)      (((w) >> 29) & 0x1u)
#define LOW_REFRESH_N(w) (((w) >> 30) & 0x1u)
#define LOW_PAWR_N(w)    (((w) >> 31) & 0x1u)
// SNIFF: o A22 (slot 47) foi re-soldado no GP29 (o tap de /RD nunca foi
// ligado). Ele resolve o mirror da WRAM: /WRAM & A22=1 => acesso direto
// $7E/$7F (o A16 escolhe a metade); A22=0 => mirror => sempre $7E baixo.
#define LOW_A22(w)       (((w) >> 29) & 0x1u)

// PIO1 word (gpio_base=16, in_base=16, bit = GPIO-16):
#define HI_A16(w)        (((w) >>  0) & 0x1u)
#define HI_DATA(w)       (((w) >>  1) & 0xFFu)     // D0..D7
#define HI_PHI2(w)       (((w) >>  9) & 0x1u)
#define HI_WR_N(w)       (((w) >> 10) & 0x1u)
#define HI_WRAM_N(w)     (((w) >> 11) & 0x1u)
#define HI_CART_N(w)     (((w) >> 12) & 0x1u)
#define HI_RD_N(w)       (((w) >> 13) & 0x1u)
#define HI_REFRESH_N(w)  (((w) >> 14) & 0x1u)
#define HI_PAWR_N(w)     (((w) >> 15) & 0x1u)
#define HI_ADDR_HIGH(w)  (((w) >> 16) & 0x7Fu)     // A17..A23
#define HI_PA0(w)        (((w) >> 23) & 0x1u)
#define HI_PA1(w)        (((w) >> 24) & 0x1u)
#define HI_PA7(w)        (((w) >> 25) & 0x1u)
#define HI_PARD_N(w)     (((w) >> 26) & 0x1u)
#define HI_RESET_N(w)    (((w) >> 27) & 0x1u)
// bits 28..31 = RESET_DRIVE / /OE / UART -- sempre mascarar

// =============================================================================
// Espelho integral de WRAM + SRAM
// =============================================================================
//
// A v0.6 usava um hashmap esparso populado por um "discovery pass" do
// rcheevos. Isso quebra com achievements de enderecamento indireto
// (AddAddress): o conjunto de enderecos consultados NAO e' estatico -- os
// ponteiros assumem valores reais em jogo e o rcheevos consulta enderecos
// nunca registrados. Solucao: espelhar tudo, como no firmware NES atual.
// O RP2350 tem 520 KB de SRAM; 128K + 32K de espelhos cabem com folga.
//
// Concorrencia: producer (core1) escreve bytes, consumer (core0/rcheevos)
// le bytes. Stores/loads de 8 bits sao atomicos; o rcheevos tolera estado
// intermediario entre frames (mesma premissa do NES pre-snapshot).
//
// rcheevos linear address map:
//   0x000000..0x01FFFF  WRAM   (128 KB)
//   0x020000..0x027FFF  SRAM   (espelho de ate 32 KB -- LoROM <=32K /
//                               HiROM <=8K; SRAM grande = fase 4b)

#define RCHEEVOS_WRAM_BASE  0x000000u
#define RCHEEVOS_SRAM_BASE  0x020000u

#define WRAM_MIRROR_SIZE    (128u * 1024u)
#define SRAM_MIRROR_SIZE    (32u * 1024u)

static volatile uint8_t wram_mirror[WRAM_MIRROR_SIZE];
static volatile uint8_t sram_mirror[SRAM_MIRROR_SIZE];

// Inter-core "something fresh happened" signal.
static volatile bool flag_bus_written = false;

static void mirrors_reset(void) {
    memset((void *)wram_mirror, 0, sizeof(wram_mirror));
    memset((void *)sram_mirror, 0, sizeof(sram_mirror));
}

// =============================================================================
// DMA ring streams (PIO RX FIFO -> ring buffer; drain incremental no core1)
// =============================================================================
//
// Mudanca vs v0.6 (ping-pong + IRQ): com captura 1-palavra-por-ciclo, um
// stream de baixo trafego (SRAM: escritas so em save points) levaria horas
// para completar um buffer ping-pong -- os dados ficariam invisiveis ate la.
// Agora cada stream usa UM canal de DMA em modo ENDLESS (RP2350) com ring
// no write_addr: o canal roda para sempre, e o consumer no core1 drena
// incrementalmente comparando seu indice de leitura com o write_addr do DMA.
// Sem IRQ, sem handshake de metades.
//
// Buffers precisam de alinhamento 2^n para o ring do DMA.

#define DMA_WRAM_LOG2   14   // 16 KB = 4096 palavras
#define DMA_CART_LOG2   12   // 4 KB  = 1024 palavras
#define DMA_HISRAM_LOG2 11   // 2 KB = 512 palavras (trafego de CPU, sem DMA)
#define DMA_BPORT_LOG2  13   // 8 KB  = 2048 palavras

#define DMA_WRAM_WORDS  (1u << (DMA_WRAM_LOG2 - 2))
#define DMA_CART_WORDS  (1u << (DMA_CART_LOG2 - 2))
#define DMA_HISRAM_WORDS (1u << (DMA_HISRAM_LOG2 - 2))
#define DMA_BPORT_WORDS (1u << (DMA_BPORT_LOG2 - 2))

static volatile uint32_t dma_wram_ring[DMA_WRAM_WORDS]
    __attribute__((aligned(1u << DMA_WRAM_LOG2)));
static volatile uint32_t dma_cart_ring[DMA_CART_WORDS]
    __attribute__((aligned(1u << DMA_CART_LOG2)));
static volatile uint32_t dma_hisram_ring[DMA_HISRAM_WORDS]
    __attribute__((aligned(1u << DMA_HISRAM_LOG2)));
static volatile uint32_t dma_bport_ring[DMA_BPORT_WORDS]
    __attribute__((aligned(1u << DMA_BPORT_LOG2)));

typedef struct {
    volatile uint32_t *ring;
    uint32_t mask;          // words - 1
    int chan;
    uint32_t rd;            // indice de leitura do consumer
    volatile uint32_t words;    // total drenado (telemetria/DBG)
    volatile uint32_t last_word; // ultima palavra vista (DBG)
} bus_stream_t;

static bus_stream_t s_wram, s_cart, s_hisram, s_bport;

/** Palavras disponiveis para consumir (posicao do DMA - posicao de leitura). */
static inline uint32_t __not_in_flash_func(stream_wr_pos)(const bus_stream_t *s) {
    uintptr_t wa = dma_channel_hw_addr(s->chan)->write_addr;
    return (uint32_t)((wa - (uintptr_t)s->ring) >> 2) & s->mask;
}

/**
 * Cria um stream: canal unico, ENDLESS + ring no write address, pacing pelo
 * DREQ do RX FIFO do PIO.
 */
static void setup_dma_stream(bus_stream_t *s, PIO pio, uint sm,
                             volatile uint32_t *ring, uint32_t log2_bytes) {
    uint32_t words = 1u << (log2_bytes - 2);
    memset((void *)ring, 0, words * sizeof(uint32_t));

    s->ring = ring;
    s->mask = words - 1u;
    s->rd   = 0;
    s->chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(s->chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, log2_bytes);      // wrap no write_addr
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    channel_config_set_high_priority(&c, true);
    channel_config_set_enable(&c, true);

    dma_channel_set_read_addr(s->chan, &pio->rxf[sm], false);
    dma_channel_set_write_addr(s->chan, ring, false);
    dma_channel_set_config(s->chan, &c, false);
    // MODE=ENDLESS (0xF nos bits 31:28): TRANS_COUNT nunca decrementa, o
    // canal roda para sempre. Campo de count preenchido com valor maximo
    // por defensividade (em ENDLESS ele e' ignorado, mas count=0 + trigger
    // seria interpretavel como transfer vazio em outros modos).
    dma_channel_hw_addr(s->chan)->al1_transfer_count_trig =
        (DMA_CH0_TRANS_COUNT_MODE_VALUE_ENDLESS << DMA_CH0_TRANS_COUNT_MODE_LSB) |
        DMA_CH0_TRANS_COUNT_COUNT_BITS;
}

static void setup_dma(void) {
    setup_dma_stream(&s_wram,  BUS_LOW_PIO,  BUS_WRAM_SM,  dma_wram_ring,  DMA_WRAM_LOG2);
    setup_dma_stream(&s_cart,  BUS_LOW_PIO,  BUS_CART_SM,  dma_cart_ring,  DMA_CART_LOG2);
    setup_dma_stream(&s_hisram, BUS_LOW_PIO, BUS_HISRAM_SM, dma_hisram_ring, DMA_HISRAM_LOG2);
    setup_dma_stream(&s_bport, BUS_HIGH_PIO, BUS_BPORT_SM, dma_bport_ring, DMA_BPORT_LOG2);
}

// =============================================================================
// Clock
// =============================================================================

// SNES fast cycle ~279 ns. At 200 MHz sys_clk we have ~55 PIO cycles per bus
// cycle, which is comfortable. Boost to 250 MHz if margin proves tight on real
// hardware.
#define SNES_SYS_CLOCK_KHZ 200000

static void setup_clock(void) {
    // RP2350 typical Vcore is 1.10 V; bump slightly for stable 200+ MHz.
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(10);
    set_sys_clock_khz(SNES_SYS_CLOCK_KHZ, true);
}

// =============================================================================
// GPIO + PIO bring-up
// =============================================================================

static void reset_bus_gpios(void) {
    // Input simples, sem pulls, funcao SIO. Os PIOs leem os sincronizadores
    // de entrada de qualquer funcao -- captura nao exige ownership do pino
    // (por isso nao ha pio_gpio_init em lugar nenhum). GP44/45 (saidas do
    // main) e GP46/47 (UART) ficam fora do range.
    for (uint i = BUS_GPIO_FIRST; i <= BUS_GPIO_LAST; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_disable_pulls(i);
    }
    // GP29 = A22 (re-soldado do slot 47). Pull-down para a auto-deteccao ler
    // 0 quando o fio nao existe (=> FALLBACK), em vez de flutuar e mentir 1.
    // Com o fio, o driver do console vence o pull (~50uA) com folga.
    gpio_pull_down(SNES_RD_N);
}

/**
 * Configure safe defaults *before anything else touches the GPIOs*:
 *  - BUFFER_OE_N driven HIGH  (SNES isolated; pull-up already does this,
 *                              but we drive actively to make it solid)
 *  - SNES_RESET_DRIVE HIGH    (Q1 saturates -> SNES held in reset while
 *                              we do cartridge ID)
 *
 * Call this as the FIRST action in main(), before stdio/uart/anything.
 */

// -----------------------------------------------------------------------------
// Abstracao do isolamento de barramento
// -----------------------------------------------------------------------------
//
// Placa final ('245/CBT3245): /OE ativo-BAIXO, push-pull do GPIO basta
// (Vih TTL = 2.0V). Pull-up de 10k a 5V da o failsafe de power-on (isolado).
//
// DEFAULT (v0.3): banco de 9x 74HC4066/CD4066 -- protoboard e PCB usam os
// mesmos chips. Controle BUS_CTRL ativo-ALTO (HIGH = fechado), push-pull de
// 3.3V direto (validado no NES; prefira 74HC4066, Vih 3.15V @ 5V), com R1
// pull-DOWN de 10k -> failsafe: power-on = switches abertos = isolado.
//
// Plano B (lote de 4066 que nao feche com 3.3V; pior caso CD4066B: 3.5V):
// habilite BUS_SWITCH_4066_OPEN_DRAIN e troque R1 para pull-UP a 5V -- o
// GPIO passa a so puxar LOW (isola) ou soltar (pull-up fecha com 5V pleno).
// Nessa variante o pull-up e OBRIGATORIO e perde-se o failsafe de power-on
// (janela de ~dezenas de ms conectado; inofensiva -- Q1 reseta o console).
//
// Comente BUS_SWITCH_4066 apenas para hardware antigo com '245/CBT3245
// (/OE ativo-baixo).

#define BUS_SWITCH_4066
// #define BUS_SWITCH_4066_OPEN_DRAIN

static inline void bus_isolation_set(bool isolated) {
#if defined(BUS_SWITCH_4066) && defined(BUS_SWITCH_4066_OPEN_DRAIN)
    if (isolated) {
        gpio_set_dir(BUFFER_OE_N, GPIO_OUT);
        gpio_put(BUFFER_OE_N, 0);          // controle LOW -> switches abertos
    } else {
        gpio_set_dir(BUFFER_OE_N, GPIO_IN); // solta: pull-up 5V fecha switches
    }
#elif defined(BUS_SWITCH_4066)
    // Push-pull 3.3V, polaridade invertida vs '245: HIGH = fechado/conectado.
    gpio_set_dir(BUFFER_OE_N, GPIO_OUT);
    gpio_put(BUFFER_OE_N, isolated ? 0 : 1);
#else
    gpio_set_dir(BUFFER_OE_N, GPIO_OUT);
    gpio_put(BUFFER_OE_N, isolated ? 1 : 0);   // /OE dos '245 (ativo-baixo)
#endif
}

static void setup_safe_defaults(void) {
    gpio_init(BUFFER_OE_N);
    gpio_disable_pulls(BUFFER_OE_N);
    bus_isolation_set(true);               // console isolado

    gpio_init(SNES_RESET_DRIVE);
    gpio_set_dir(SNES_RESET_DRIVE, GPIO_OUT);
    gpio_put(SNES_RESET_DRIVE, 1);         // Q1 ON -> /RESET asserted
}

// --- RX da UART por interrupcao -----------------------------------------
//
// O FIFO de RX tem so 32 bytes; drenar por polling no loop do core 0 perde
// bytes se qualquer coisa engasgar >2.8ms (ex.: printf no stdio USB com
// host lento trava ate 0.5s). Payloads grandes (patch de achievements,
// ~21KB continuos) corrompiam no meio. A ISR empurra para um ring de 8KB
// e o poll_uart consome com calma.

#define UART_RX_RING_SIZE 8192   // potencia de 2
static volatile uint8_t  uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint32_t uart_rx_head;   // escrito pela ISR
static volatile uint32_t uart_rx_tail;   // lido pelo core 0

static void __not_in_flash_func(uart_rx_isr)(void) {
    while (uart_is_readable(UART_ID)) {
        uint8_t c = (uint8_t)uart_get_hw(UART_ID)->dr;
        uint32_t next = (uart_rx_head + 1u) & (UART_RX_RING_SIZE - 1u);
        if (next != uart_rx_tail) {          // ring cheio: descarta (>8KB atras)
            uart_rx_ring[uart_rx_head] = c;
            uart_rx_head = next;
        }
    }
}

static void setup_uart(void) {
    uart_init(UART_ID, UART_BAUD);
    // GP46/47: a funcao F2 (GPIO_FUNC_UART) nessas posicoes e' UART0
    // CTS/RTS! O TX/RX fica na funcao ALTERNATIVA F11 (UART_AUX) do
    // RP2350 -- confirmado em io_bank0.h: GPIO46 F11 = UART0_TX,
    // GPIO47 F11 = UART0_RX.
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART_AUX);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART_AUX);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    uart_rx_head = uart_rx_tail = 0;
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_isr);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);   // RX + timeout IRQ
}

static uint pio_wram_offset, pio_cart_offset, pio_hisram_offset, pio_bport_offset, pio_carthi_offset;

static void setup_bus_pios(void) {
    // PIO1 precisa enxergar GPIO 16..47: no RP2350B cada PIO tem uma janela
    // de 32 GPIOs selecionada por GPIOBASE (0 ou 16). Sem esta chamada,
    // qualquer referencia a GPIO >= 32 no PIO1 e' invalida.
    pio_set_gpio_base(BUS_HIGH_PIO, 16);

    // PIO0: SM0 (snesBusWram) escritas WRAM; SM1 (snesBusCart) escritas cart.
    pio_wram_offset = pio_add_program(BUS_LOW_PIO, &snesBusWram_program);
    pio_cart_offset = pio_add_program(BUS_LOW_PIO, &snesBusCart_program);
    pio_hisram_offset = pio_add_program(BUS_LOW_PIO, &snesBusHiSram_program);
    snesBusWram_program_init(BUS_LOW_PIO, BUS_WRAM_SM, pio_wram_offset, 1.0f);
    snesBusCart_program_init(BUS_LOW_PIO, BUS_CART_SM, pio_cart_offset, 1.0f);
    snesBusHiSram_program_init(BUS_LOW_PIO, BUS_HISRAM_SM, pio_hisram_offset, 1.0f);

    // PIO1: SM0 (snesBusBport) porta WRAM $2180-83 via /PAWR.
    //       SM1 (snesBusCartHigh) fase 4b -- programa carregado, SM parado.
    pio_bport_offset  = pio_add_program(BUS_HIGH_PIO, &snesBusBport_program);
    pio_carthi_offset = pio_add_program(BUS_HIGH_PIO, &snesBusCartHigh_program);
    snesBusBport_program_init   (BUS_HIGH_PIO, BUS_BPORT_SM,  pio_bport_offset,  1.0f);
    snesBusCartHigh_program_init(BUS_HIGH_PIO, BUS_CARTHI_SM, pio_carthi_offset, 1.0f);

    // Nao ha pio_gpio_init: captura le os sincronizadores de entrada
    // independente da funcao do pino (que fica SIO/input via gpio_init).

    // Desabilitados ate DMA + consumer estarem no ar.
    pio_sm_set_enabled(BUS_LOW_PIO,  BUS_WRAM_SM,   false);
    pio_sm_set_enabled(BUS_LOW_PIO,  BUS_CART_SM,   false);
    pio_sm_set_enabled(BUS_HIGH_PIO, BUS_BPORT_SM,  false);
    pio_sm_set_enabled(BUS_HIGH_PIO, BUS_CARTHI_SM, false);
}

// =============================================================================
// Cartridge identification phase (runs on core 0 before monitor mode)
// =============================================================================
//
// With BUFFER_OE_N held HIGH (buffers in HiZ) and SNES_RESET_DRIVE HIGH (SNES
// stuck in reset), the cartridge bus is ours exclusively. We drive A0..A23,
// /CART, /RD as outputs, sample D0..D7 as inputs, and walk the ROM space to
// compute its MD5 hash and parse the SNES header.
//
// Timing: SNES mask ROMs typically have 120-200 ns access time. We use ~1 us
// per read which is several times more than enough.
//
// After ID:
//   1. Release all bus GPIOs back to input
//   2. Drive BUFFER_OE_N LOW -> buffers active, SNES sees cart
//   3. Drive SNES_RESET_DRIVE LOW -> Q1 cuts off -> /RESET releases
//   4. Hand off to core1 (monitor mode)

static inline void cart_drive_pin(uint pin, bool level) {
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, level);
}

static inline void cart_release_pin(uint pin) {
    gpio_set_dir(pin, GPIO_IN);
    gpio_disable_pulls(pin);
}

/**
 * Switch the cart-side GPIOs from input to output and stage them in the
 * "idle, no read active" state.
 *
 * After this call, the Pico is mastering the cart bus exclusively. The SNES
 * is still isolated (BUFFER_OE_N high) and held in reset.
 */
static void cart_drive_bus_acquire(void) {
    // Address bus: A0..A16 em GP0..GP16 (contiguos), A17..A23 em GP32..GP38.
    for (uint i = SNES_A0; i <= SNES_A16; i++) cart_drive_pin(i, 0);
    for (uint i = SNES_A17; i <= SNES_A23; i++) cart_drive_pin(i, 0);

    // Data bus (D0..D7): inputs while we read. Pico is 5V-tolerant.
    for (uint i = SNES_D0; i <= SNES_D7; i++) cart_release_pin(i);

    // Control signals: drive /CART high (inactive), /RD high (inactive),
    // /WR high (inactive). We're not writing anything to the cart.
    cart_drive_pin(SNES_CART_N, 1);
    cart_drive_pin(SNES_RD_N,   1);
    cart_drive_pin(SNES_WR_N,   1);
}

/**
 * Return all cart-side GPIOs to input/HiZ. The SNES is about to drive these.
 */
static void cart_drive_bus_release(void) {
    for (uint i = SNES_A0;  i <= SNES_A16; i++) cart_release_pin(i);
    for (uint i = SNES_A17; i <= SNES_A23; i++) cart_release_pin(i);
    for (uint i = SNES_D0;  i <= SNES_D7;  i++) cart_release_pin(i);
    cart_release_pin(SNES_CART_N);
    cart_release_pin(SNES_RD_N);
    cart_release_pin(SNES_WR_N);
}

// Bit-mask of address pins in the low SIO register (GP0..GP31).
// A0..A16 ocupam GP0..GP16 -> 17 bits a partir do bit 0.
#define ADDR_LO_MASK   ((1u << 17) - 1u)

// Bit-mask of address pins in the high SIO register (GP32..GP47).
// A17..A23 ocupam GP32..GP38 -> 7 bits a partir do bit 0 do registrador hi.
#define ADDR_HI_MASK   ((1u << 7) - 1u)

/**
 * Drive a 24-bit address onto A0..A23 using atomic register writes.
 * Four SIO stores total (clear, set on lo; clear, set on hi) -- ~4 cycles.
 * Caller must have called cart_drive_bus_acquire() first.
 */
static inline void cart_set_address(uint32_t addr) {
    uint32_t lo_bits = (addr & 0x1FFFFu);
    uint32_t hi_bits = ((addr >> 17) & 0x7Fu);

    sio_hw->gpio_clr    = ADDR_LO_MASK;
    sio_hw->gpio_set    = lo_bits;
    sio_hw->gpio_hi_clr = ADDR_HI_MASK;
    sio_hw->gpio_hi_set = hi_bits;
}

/**
 * Read one byte from the cart at the given 24-bit address.
 *
 * Sequence:
 *   1. Set address on A0..A23
 *   2. Assert /CART low (cartridge ROM enable)
 *   3. Assert /RD low (read strobe)
 *   4. Wait for cart access time
 *   5. Sample D0..D7
 *   6. De-assert /RD and /CART
 */
static uint8_t cart_read_byte(uint32_t addr) {
    cart_set_address(addr);

    gpio_put(SNES_CART_N, 0);
    gpio_put(SNES_RD_N,   0);

    // Mask ROM access time ~120-200 ns; we wait ~1 us to be conservative.
    busy_wait_us_32(1);

    uint32_t gpio_in = sio_hw->gpio_in;
    uint8_t  data    = (uint8_t)((gpio_in >> SNES_D0) & 0xFFu);

    gpio_put(SNES_RD_N,   1);
    gpio_put(SNES_CART_N, 1);

    return data;
}

// -----------------------------------------------------------------------------
// SNES cartridge header
// -----------------------------------------------------------------------------
//
// Both LoROM and HiROM carts expose their 64-byte header at bus address
// $00:FFB0-$00:FFFF. The cart's internal address decoder maps this to
// ROM file offset 0x7FB0 (LoROM) or 0xFFB0 (HiROM) -- but from our point of
// view on the bus, the read address is the same. We then look at the map_mode
// byte to learn which mapper the cart uses for the rest of the ROM.
//
// Header layout (relative to bus $00:FFB0, byte offsets 0x00..0x4F):
//   0x10..0x24  title          (21 bytes, ASCII)
//   0x25        map_mode       (bit 0: 0=LoROM, 1=HiROM ; bit 4: FastROM)
//   0x26        rom_type       (00=ROM, 01=ROM+RAM, 02=ROM+RAM+BATT, ...)
//   0x27        rom_size_code  (rom_size = 1 << (code + 10))
//   0x28        sram_size_code (sram_size = code ? 1 << (code + 10) : 0)
//   0x29        region         (00,01,0B,0D,0F = NTSC; rest = PAL)
//   0x2A        dev_id
//   0x2B        version
//   0x2C..0x2D  checksum complement (~checksum)
//   0x2E..0x2F  checksum (sum of every ROM byte, low 16 bits)

#define SNES_HEADER_BUS_ADDR    0x00FFB0u
#define SNES_HEADER_SIZE        80    // 0xFFB0..0xFFFF inclusive

#define HDR_OFF_TITLE           0x10
#define HDR_OFF_MAP_MODE        0x25
#define HDR_OFF_ROM_TYPE        0x26
#define HDR_OFF_ROM_SIZE        0x27
#define HDR_OFF_SRAM_SIZE       0x28
#define HDR_OFF_REGION          0x29
#define HDR_OFF_CSUM_COMPLEMENT 0x2C
#define HDR_OFF_CSUM            0x2E

typedef struct {
    char     title[22];          // 21 bytes + NUL
    uint8_t  map_mode;
    uint8_t  rom_type;
    uint8_t  rom_size_code;
    uint8_t  sram_size_code;
    uint8_t  region;
    uint16_t checksum;
    uint16_t checksum_complement;

    uint32_t rom_size_bytes;     // decoded from rom_size_code
    uint32_t sram_size_bytes;    // decoded from sram_size_code
    bool     is_hirom;
    bool     header_valid;
    bool     is_ntsc;            // derived from region byte

    uint8_t  md5[16];            // MD5 of full ROM (rcheevos hash)
} snes_cart_info_t;

static snes_cart_info_t cart_info;

/**
 * Sequentially read N bytes from a contiguous bus address range into dst.
 * Used for header reads only -- the streaming MD5 loop uses cart_read_byte
 * directly so it can compute the mapper-translated bus address per byte.
 */
static void cart_read_block(uint32_t bus_addr_start, uint8_t *dst, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        dst[i] = cart_read_byte(bus_addr_start + i);
    }
}

/**
 * Check the header's self-consistency: checksum XOR complement == 0xFFFF.
 * If the cart is in a slot that returns garbage (e.g. open bus), this will
 * almost always fail -- it's our smoke test for "did we actually read a cart".
 */
static bool snes_header_validate(const uint8_t *hdr) {
    uint16_t cs = (uint16_t)hdr[HDR_OFF_CSUM]            |
                  ((uint16_t)hdr[HDR_OFF_CSUM + 1] << 8);
    uint16_t cc = (uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT] |
                  ((uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT + 1] << 8);
    return (uint16_t)(cs ^ cc) == 0xFFFFu;
}

/**
 * Translate a ROM file offset (0-based, linear) to the bus address that
 * the cart's decoder maps to that byte, given the mapper type.
 *
 * LoROM: 32KB ROM banks at offsets $XX:8000..$XX:FFFF for bank $XX.
 *        ROM offset N -> bus addr ((N >> 15) << 16) | ((N & 0x7FFF) | 0x8000)
 *
 * HiROM: 64KB ROM banks at offsets $XX:0000..$XX:FFFF for bank $XX>=$C0.
 *        ROM offset N -> bus addr 0xC00000 | N
 *        (the $C0..$FF region is the canonical linear view of HiROM)
 */
static inline uint32_t rom_offset_to_bus(bool is_hirom, uint32_t rom_offset) {
    if (is_hirom) {
        return 0xC00000u | rom_offset;
    } else {
        return ((rom_offset >> 15) << 16) | ((rom_offset & 0x7FFFu) | 0x8000u);
    }
}

/**
 * Region byte -> NTSC vs PAL.
 * NTSC: 0x00 (JP), 0x01 (US), 0x0B (CA), 0x0D (KR), 0x0F (HK)
 * PAL: everything else (mostly 0x02..0x0A = Europe/Australia)
 */
static bool region_is_ntsc(uint8_t r) {
    return (r == 0x00 || r == 0x01 || r == 0x0B || r == 0x0D || r == 0x0F);
}

// -----------------------------------------------------------------------------
// Cart bus address -> linear SRAM offset
// -----------------------------------------------------------------------------
//
// Standard SNES SRAM windows:
//   LoROM: banks $70-$7D, offsets $0000-$7FFF (32 KB per bank);
//          mirror at $F0-$FD (FastROM region) accesses the same chip.
//   HiROM: banks $30-$3F, offsets $6000-$7FFF (8 KB window per bank);
//          mirror at $B0-$BF.
//
// Returns the linear SRAM offset (0..sram_size-1) for the given bus access,
// or BUS_NOT_SRAM if the access doesn't fall inside the SRAM window for the
// active mapper.
//
// IMPORTANT: this version uses ONLY A0..A16 (matching what PIO0 SM1 captures).
// That covers:
//   - LoROM SRAM up to 32 KB (single bank: offset directly = SRAM byte)
//   - HiROM SRAM up to 8 KB  (single bank window: offset-$6000 = SRAM byte)
// Carts with larger SRAM (Star Ocean, Tales of Phantasia, Dragon Quest VI)
// need PIO1's A17..A23 to disambiguate banks -- TODO in Phase 4b.

#define BUS_NOT_SRAM 0xFFFFFFFFu

static inline uint32_t bus_to_sram_offset(uint32_t bus_addr,
                                          bool is_hirom,
                                          uint32_t sram_size) {
    if (sram_size == 0) return BUS_NOT_SRAM;

    // We only have A0..A16 from PIO0 SM1's capture.
    uint16_t offset = bus_addr & 0xFFFFu;        // A0..A15
    // A16 is the bit above, but we don't need it for the small-SRAM path:
    // /CART being asserted already tells us we're in cart-mapped territory,
    // and SRAM windows are bank-offset-defined, not A16-defined.

    if (!is_hirom) {
        // LoROM: SRAM lives at offset $0000..$7FFF inside banks $70-$7D
        // (and $F0-$FD). For SRAM up to 32 KB, the byte is simply
        // `offset`. For larger SRAM the bank index matters, but we can't
        // see A17-A23 here -- caller must guard against sram_size > 32K.
        if (offset < 0x8000u) {
            if (offset < sram_size) return offset;
        }
        return BUS_NOT_SRAM;
    } else {
        // HiROM: SRAM at $6000..$7FFF (8 KB) inside banks $30-$3F / $B0-$BF.
        // For SRAM up to 8 KB, the byte is `offset - $6000`.
        if (offset >= 0x6000u && offset < 0x8000u) {
            uint32_t off = (uint32_t)(offset - 0x6000u);
            if (off < sram_size) return off;
        }
        return BUS_NOT_SRAM;
    }
}

/**
 * Stream-MD5 the full ROM image, translating each linear offset to a bus
 * address according to the mapper. Reads in chunks to amortise the MD5
 * function call overhead.
 */
static void cart_compute_md5(bool is_hirom, uint32_t rom_size_bytes,
                             uint8_t out_md5[16]) {
    md5_state_t ctx;
    md5_init(&ctx);

    enum { CHUNK = 256 };
    uint8_t chunk[CHUNK];

    uint32_t offset = 0;
    while (offset < rom_size_bytes) {
        uint32_t to_read = rom_size_bytes - offset;
        if (to_read > CHUNK) to_read = CHUNK;

        for (uint32_t i = 0; i < to_read; i++) {
            uint32_t bus_addr = rom_offset_to_bus(is_hirom, offset + i);
            chunk[i] = cart_read_byte(bus_addr);
        }
        md5_append(&ctx, chunk, (int)to_read);
        offset += to_read;
    }
    md5_finish(&ctx, out_md5);
}

// -----------------------------------------------------------------------------
// Modo SNIFF-ONLY (bring-up com fios soldados direto no console)
// -----------------------------------------------------------------------------
//
// Para testar SO o sniff de memoria, sem placa intermediaria: fios soldados
// nos pontos do slot embaixo do console, Pico alimentado pelo USB, GND comum.
// Neste modo o firmware NUNCA dirige nenhum pino do barramento (nao ha
// switches para isolar e o console esta vivo!): o cart-ID vira um stub que
// preenche cart_info a partir da TABELA de jogos abaixo. O jogo ativo e'
// trocado SEM reflash: comando GAME=<n> (USB serial ou pagina /game do
// ESP32) grava o indice e reinicia. Persistencia no ultimo setor da flash.
// Comente SNIFF_ONLY_MODE para voltar ao cart-ID real (fase 5).
//
#define SNIFF_ONLY_MODE 1

#ifdef SNIFF_ONLY_MODE

typedef struct {
    const char *name;      // vai no cart_info.title (max 21 chars, sem ';')
    const char *md5;       // hash RA da ROM, 32 hex
    uint8_t     is_hirom;
    uint16_t    sram_kb;   // janela de SRAM espelhada (LoROM $70 / HiROM $3x:6-7FFF)
    uint8_t     is_ntsc;
} sniff_game_t;

// ATENCAO: a pagina /game do snes-esp-firmware.ino duplica esta lista
// (nomes+indices) -- mantenha as duas em sincronia ao editar.
static const sniff_game_t sniff_games[] = {
    { "Super Mario World",     "cdd3c8c37322978ca8669b34bc89c804", 0,  2, 1 },
    { "Street Fighter II",     "be7fc84ac1151b99ad48329327e2aaa7", 0,  0, 1 },
    { "Final Fantasy III (J)", "544311e104805e926083acf29ec664da", 1,  8, 1 },
    { "Yoshi's Island",        "cb472164c5a71ccd3739963390ec6a50", 0, 32, 1 },
    { "Star Fox",              "9dce6a9dcbe4e304d67b9e8fd8999e7e", 0, 32, 1 },
    { "Mega Man X",            "df1cc0c8c8c4b61e3b834cc03366611c", 0,  0, 1 },
    { "Super Castlevania IV",  "094f035993e9724647b61ddcba1e9a7a", 0,  0, 1 },
    { "EarthBound",            "a864b2e5c141d2dec1c4cbed75a42a85", 1,  8, 1 },
    { "Zelda: Link to Past",   "608c22b8ff930c62dc2de54bcd6eba72", 0,  8, 1 },
    { "Super Punch-Out!!",     "97fe7d7d2a1017f8480e60a365a373f0", 0,  8, 1 },
    { "Super Mario Kart",      "7f25ce5a283d902694c52fb1152fa61a", 0,  2, 1 },
};
#define SNIFF_GAME_COUNT   (sizeof(sniff_games) / sizeof(sniff_games[0]))
#define SNIFF_GAME_DEFAULT 4u   // usado so' quando a flash ainda esta virgem

static uint8_t sniff_active_game = SNIFF_GAME_DEFAULT;

// Persistencia do indice: ultimo setor da flash (o RP2350 nao tem EEPROM).
// Escrever flash com o core1 capturando (codigo XIP) = hardfault; por isso
// GAME=<n> apenas deixa o pedido no scratch do watchdog (sobrevive ao reboot,
// nao a power-off) e reinicia -- game_slot_boot_init() persiste na flash no
// comeco do boot, quando so o core0 existe. scratch[4..7] sao do bootrom/
// watchdog_reboot; usamos [0]/[1].
#define GAME_SLOT_MAGIC     0x534E4946u   // "SNIF"
#define GAME_SLOT_FLASH_OFF (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

static void game_slot_boot_init(void) {
    if (watchdog_hw->scratch[0] == GAME_SLOT_MAGIC &&
        watchdog_hw->scratch[1] < SNIFF_GAME_COUNT) {
        uint8_t idx = (uint8_t)watchdog_hw->scratch[1];
        watchdog_hw->scratch[0] = 0;
        uint8_t page[FLASH_PAGE_SIZE];
        memset(page, 0xFF, sizeof(page));
        page[0] = 'S'; page[1] = 'N'; page[2] = 'I'; page[3] = 'F';
        page[4] = idx;
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(GAME_SLOT_FLASH_OFF, FLASH_SECTOR_SIZE);
        flash_range_program(GAME_SLOT_FLASH_OFF, page, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
    }
    const uint8_t *p = (const uint8_t *)(XIP_BASE + GAME_SLOT_FLASH_OFF);
    if (p[0] == 'S' && p[1] == 'N' && p[2] == 'I' && p[3] == 'F' &&
        p[4] < SNIFF_GAME_COUNT) {
        sniff_active_game = p[4];
    }
}

static void sniff_fill_cart_info(void) {
    const sniff_game_t *g = &sniff_games[sniff_active_game];

    memset(&cart_info, 0, sizeof(cart_info));
    strncpy(cart_info.title, g->name, 21);
    cart_info.title[21]       = '\0';
    cart_info.header_valid    = true;
    cart_info.is_hirom        = g->is_hirom != 0;
    cart_info.is_ntsc         = g->is_ntsc != 0;
    cart_info.sram_size_bytes = (uint32_t)g->sram_kb * 1024u;

    // MD5 da tabela: string hex -> bytes
    for (int i = 0; i < 16; i++) {
        char b[3] = { g->md5[i * 2], g->md5[i * 2 + 1], '\0' };
        cart_info.md5[i] = (uint8_t)strtol(b, NULL, 16);
    }
    printf("SNIFF-ONLY [%u] %s: %s SRAM=%luKB md5=%s\n",
           (unsigned)sniff_active_game, g->name,
           cart_info.is_hirom ? "HiROM" : "LoROM",
           (unsigned long)(cart_info.sram_size_bytes / 1024), g->md5);
}
#endif

/**
 * Top-level cartridge identification. Drives the bus, reads the header,
 * computes streaming MD5 over the full ROM, fills in `cart_info`, then
 * releases the bus.
 *
 * Returns true if a valid SNES header was found.
 */
static bool cart_id_phase(void) {
#ifdef SNIFF_ONLY_MODE
    // Sniff-only: NAO tocar no barramento. cart_info ja foi preenchida no
    // boot; so reafirma para o caso de um READ_CART repetido.
    sniff_fill_cart_info();
    return true;
#else
    memset(&cart_info, 0, sizeof(cart_info));

    cart_drive_bus_acquire();
    sleep_ms(1);   // let levels settle after switching to output

    // --- Read header ---
    uint8_t hdr[SNES_HEADER_SIZE];
    cart_read_block(SNES_HEADER_BUS_ADDR, hdr, SNES_HEADER_SIZE);

    cart_info.header_valid = snes_header_validate(hdr);
    if (!cart_info.header_valid) {
        printf("cart_id: header @ $00:FFB0 INVALID (cs/compl mismatch). "
               "First bytes:");
        for (int i = 0; i < 16; i++) printf(" %02X", hdr[i]);
        printf("\n");
        cart_drive_bus_release();
        return false;
    }

    // --- Parse header fields ---
    memcpy(cart_info.title, &hdr[HDR_OFF_TITLE], 21);
    cart_info.title[21] = '\0';
    // Trim trailing 0x00 / 0xFF padding so the printout looks clean.
    for (int i = 20; i >= 0; i--) {
        uint8_t c = (uint8_t)cart_info.title[i];
        if (c == 0x00 || c == 0xFF || c == ' ') cart_info.title[i] = '\0';
        else break;
    }

    cart_info.map_mode            = hdr[HDR_OFF_MAP_MODE];
    cart_info.rom_type            = hdr[HDR_OFF_ROM_TYPE];
    cart_info.rom_size_code       = hdr[HDR_OFF_ROM_SIZE];
    cart_info.sram_size_code      = hdr[HDR_OFF_SRAM_SIZE];
    cart_info.region              = hdr[HDR_OFF_REGION];
    cart_info.checksum            = (uint16_t)hdr[HDR_OFF_CSUM] |
                                    ((uint16_t)hdr[HDR_OFF_CSUM + 1] << 8);
    cart_info.checksum_complement = (uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT] |
                                    ((uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT + 1] << 8);

    cart_info.is_hirom = (cart_info.map_mode & 0x01u) != 0u;
    cart_info.is_ntsc  = region_is_ntsc(cart_info.region);

    if (cart_info.rom_size_code >= 1 && cart_info.rom_size_code <= 13) {
        cart_info.rom_size_bytes = 1u << (cart_info.rom_size_code + 10);
    } else {
        cart_info.rom_size_bytes = 0;
    }
    if (cart_info.sram_size_code > 0 && cart_info.sram_size_code <= 8) {
        cart_info.sram_size_bytes = 1u << (cart_info.sram_size_code + 10);
    } else {
        cart_info.sram_size_bytes = 0;
    }

    printf("cart_id: title='%s' %s%s ROM=%luKB SRAM=%luKB region=%02X(%s) "
           "type=%02X cs=%04X\n",
           cart_info.title,
           cart_info.is_hirom ? "HiROM" : "LoROM",
           (cart_info.map_mode & 0x10u) ? "/Fast" : "",
           (unsigned long)(cart_info.rom_size_bytes  / 1024),
           (unsigned long)(cart_info.sram_size_bytes / 1024),
           cart_info.region,
           cart_info.is_ntsc ? "NTSC" : "PAL",
           cart_info.rom_type,
           cart_info.checksum);

    // --- Streaming MD5 over the full ROM ---
    if (cart_info.rom_size_bytes > 0) {
        absolute_time_t t0 = get_absolute_time();
        cart_compute_md5(cart_info.is_hirom, cart_info.rom_size_bytes,
                         cart_info.md5);
        int64_t elapsed_ms = absolute_time_diff_us(t0, get_absolute_time()) / 1000;

        printf("cart_id: MD5=");
        for (int i = 0; i < 16; i++) printf("%02x", cart_info.md5[i]);
        printf(" (%lldms)\n", elapsed_ms);
    } else {
        printf("cart_id: rom_size_code %u out of range; skipping MD5\n",
               cart_info.rom_size_code);
    }

    cart_drive_bus_release();
    return true;
#endif  // SNIFF_ONLY_MODE
}

/**
 * Final step of the boot sequence: release the SNES.
 *   1. Lower BUFFER_OE_N (buffers ON, SNES now sees the cart bus)
 *   2. Wait a beat for the SNES address decoders to settle
 *   3. Lower SNES_RESET_DRIVE (Q1 off; /RESET pull-up takes over,
 *      SNES starts executing from the reset vector)
 */
static void release_snes(void) {
    bus_isolation_set(false);
    sleep_us(10);
    gpio_put(SNES_RESET_DRIVE, 0);
}

// =============================================================================
// ESP32 serial protocol
// =============================================================================
//
// Mirrors the NES adapter's contract so the ESP32 firmware is a minimal fork.
// All messages are CR-LF terminated ASCII.
//
//   Pico -> ESP32
//     PICO_FIRMWARE_VERSION=<v>          one-shot on boot
//     PICO_READY                         handshake complete on Pico side
//     SYNC_ACK                           reply to SYNC
//     CART_INFO=<title>;<rom_kb>;<sram_kb>;<region>;<map_hex>;<type_hex>;<cs_hex>;<md5_hex>
//     SNES_RESETED                       SNES started touching the bus
//     GAME_INFO=<id>;<title>;<image_url>
//     ACH_SUMMARY=<unlocked>;<total>
//     A=<id>;<title>;<image_url>         achievement triggered
//     P=S;<id>;<title>;<url>;<progress>  progress indicator show/update
//     P=H;<id>                           progress indicator hide
//     C=S;<id>;<title>;<url>;<progress>  challenge indicator show
//     C=H;<id>                           challenge indicator hide
//     REQ=<id>;M:<method>;U:<url>;D:<body>   HTTP request via ESP32
//     BUFFER_OVERFLOW                    serial buffer wrapped
//
//   ESP32 -> Pico
//     SYNC                               handshake ping
//     TOKEN_AND_USER=<user>,<token>      credentials
//     READ_CART                          trigger cart-ID phase
//     START_WATCH                        login + load_game, then start monitor
//     RESP=<id>;<http_code>;<body>       HTTP response
//     RESET                              force Pico reboot via watchdog

#define BAUD_RATE          115200

// Fase A: 96KB estaticos p/ o payload do patch. Orcamento (520KB total):
// espelhos 160K + serial 96K + rings ~30K = BSS ~292K, sobrando ~215K de
// heap p/ o parse do rcheevos -- protegido pelo filter_large_memaddr()
// (ideia do NES v1.3, com stub corrigido: "1=0" nunca-verdadeiro; o "0=0"
// original do NES e' SEMPRE-verdadeiro e daria award falso em hardcore!).
#define SERIAL_BUFFER_SIZE 98304

static uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
static uint8_t *serial_buffer_head = serial_buffer;

// Triggers com MemAddr acima disso sao stubados para "1=0" antes do parse
// (aparecem na lista, nunca disparam). 4KB corta os 8 "monstros" do SMW
// (4.8-9.6KB de logica cada, ~150KB+ de heap se parseados) e preserva
// todo o resto.
#define MAX_MEMADDR_LEN 4096

/**
 * Varre o JSON do patch e substitui valores de "MemAddr" maiores que
 * MAX_MEMADDR_LEN por "1=0", deslocando o restante in-place.
 * Retorna o novo tamanho (string continua NUL-terminada).
 */
static size_t filter_large_memaddr(char *json, size_t len) {
    const char  *needle = "\"MemAddr\":\"";
    const size_t needle_len = 11;
    char *pos = json;

    for (;;) {
        char *found = strstr(pos, needle);
        if (!found) break;

        char *v_start  = found + needle_len;
        char *v_end    = v_start;
        char *json_end = json + len;

        while (v_end < json_end && *v_end != '"') {
            if (*v_end == '\\') v_end++;    // pula escapes
            v_end++;
        }
        if (v_end >= json_end) break;

        size_t v_len = (size_t)(v_end - v_start);
        if (v_len > MAX_MEMADDR_LEN) {
            printf("[filter] MemAddr de %u bytes -> stub 1=0\n",
                   (unsigned)v_len);
            memmove(v_start + 3, v_end, (size_t)(json_end - v_end) + 1);
            memcpy(v_start, "1=0", 3);
            len -= (v_len - 3);
            pos = v_start + 4;
        } else {
            pos = v_end + 1;
        }
    }
    return len;
}

static inline bool prefix(const char *pre, const char *str) {
    return strncmp(pre, str, strlen(pre)) == 0;
}

// Espelha todo o trafego Pico->ESP32 no USB serial -- de graca no bring-up
// (CART_INFO, GAME_INFO, A=..., REQ=... visiveis num terminal no WeAct).
#define MIRROR_UART_TO_USB 1

static inline void uart_send(const char *s) {
    uart_puts(UART_ID, s);
#if MIRROR_UART_TO_USB
    printf("%s", s);
#endif
}

// =============================================================================
// Refresh rate (frame loop pacing) -- derivado do header do cartucho em
// runtime (byte de regiao): NTSC 60 Hz, PAL 50 Hz.
// =============================================================================

#define FRAME_TIME_NTSC_US 16667
#define FRAME_TIME_PAL_US  20000

// =============================================================================
// State machine
// =============================================================================
//
//  0 : idle, waiting for ESP32 commands
//  1 : cart-ID requested, will run cart_id_phase() in main loop
//  2 : cart-ID done, waiting for credentials + START_WATCH
//  5 : rcheevos login in flight
//  6 : login done, will call begin_load_game() in main loop
//  7 : load_game in flight
//  8 : in-game, frame loop running, core1 monitoring the bus

static volatile uint8_t state = 0;
static volatile bool    snes_running_announced = false;

// =============================================================================
// Achievement FIFO (events from rcheevos core0 -> outgoing UART)
// =============================================================================

#define ACH_FIFO_SIZE 15

typedef struct {
    uint32_t id;
    uint8_t  event;
    char     measured_progress[24];
} achievement_t;

typedef struct {
    achievement_t buffer[ACH_FIFO_SIZE];
    int head;
    int tail;
    int count;
} achievement_fifo_t;

static achievement_fifo_t ach_fifo;

static void ach_fifo_init(achievement_fifo_t *f) {
    f->head = f->tail = f->count = 0;
}

static bool ach_fifo_full(achievement_fifo_t *f)  { return f->count == ACH_FIFO_SIZE; }
static bool ach_fifo_empty(achievement_fifo_t *f) { return f->count == 0; }

static bool ach_fifo_push(achievement_fifo_t *f, achievement_t v) {
    if (ach_fifo_full(f)) return false;
    f->buffer[f->tail] = v;
    f->tail = (f->tail + 1) % ACH_FIFO_SIZE;
    f->count++;
    return true;
}

static bool ach_fifo_pop(achievement_fifo_t *f, achievement_t *v) {
    if (ach_fifo_empty(f)) return false;
    *v = f->buffer[f->head];
    f->head = (f->head + 1) % ACH_FIFO_SIZE;
    f->count--;
    return true;
}

// =============================================================================
// rcheevos client + async HTTP plumbing
// =============================================================================

typedef struct {
    rc_client_server_callback_t callback;
    void                       *callback_data;
} async_cb_t;

typedef struct {
    uint8_t    id;
    async_cb_t data;
} async_slot_t;

#define MAX_ASYNC_CALLBACKS 5
static async_slot_t async_handlers[MAX_ASYNC_CALLBACKS];
static uint8_t      async_handlers_idx = 0;
static uint8_t      request_id_seq     = 0;
static uint8_t      request_ongoing    = 0;
static uint32_t     last_request_ms    = 0;

static rc_client_t *g_client = NULL;
static void        *g_callback_userdata = &g_client;
static char         rcheevos_userdata[16];

// Credentials received from ESP32
static char ra_user[256];
static char ra_token[32];

// MD5 of the cartridge as a 32-char hex string (lowercase, no NUL slack)
static char cart_md5_hex[33];

static void md5_to_hex(const uint8_t md5[16], char out[33]) {
    static const char *hex = "0123456789abcdef";
    for (int i = 0; i < 16; i++) {
        out[i*2  ] = hex[(md5[i] >> 4) & 0xF];
        out[i*2+1] = hex[ md5[i]       & 0xF];
    }
    out[32] = '\0';
}

// --- rcheevos memory read callback ---
//
// rcheevos consulta enderecos lineares; mapeamos direto nos espelhos:
//   0x000000..0x01FFFF -> wram_mirror
//   0x020000..0x027FFF -> sram_mirror (32 KB)
//   resto              -> 0
//
// Com espelho integral nao ha mais "discovery pass": qualquer endereco que
// o rcheevos pedir (inclusive alvos de AddAddress resolvidos em runtime)
// ja esta espelhado. Leituras byte a byte sao atomicas vs o producer.

// Rastreio passivo: quais enderecos o rcheevos consulta (por regiao).
// A lista de SRAM sai no DBG com o valor vivo do espelho -- responde na
// hora "este set usa SRAM? os valores estao mudando?".
#define QRY_SRAM_MAX 32
static uint32_t qry_sram_addr[QRY_SRAM_MAX];
static uint8_t  qry_sram_n;
static uint32_t qry_calls, qry_sram_calls;

// --- Snapshot atomico dos enderecos de interesse (ideia do odelot) ----------
//
// O do_frame lia o espelho VIVO enquanto o core1 segue escrevendo: as N
// condicoes de um achievement eram avaliadas sobre instantes ligeiramente
// diferentes (tearing) -- conjuncoes multi-endereco (ex.: Peppy Special:
// 0x1503 + 0x33A + 0xA05A no MESMO frame) podiam nunca fechar. Agora o
// read_memory APRENDE quais bytes o set consulta (1o frame serve vivo e
// registra) e, no inicio de CADA do_frame, snapshot_refresh() congela uma
// copia deles -- a avaliacao inteira enxerga UM instante unico, como num
// emulador. PEEK/WATCH continuam lendo o espelho vivo (in_do_frame gate).
// Snapshot NAO filtra escrita-lixo (isso e' solda/ruido) -- elimina apenas
// a incoerencia temporal.
#define SNAP_MAX 1024
static uint32_t snap_addr[SNAP_MAX];
static uint8_t  snap_val[SNAP_MAX];
static volatile uint16_t snap_count;
static bool     in_do_frame;      // gate: snapshot so' vale p/ o rcheevos

static inline uint8_t mirror_read_byte(uint32_t a) {
    if (a < WRAM_MIRROR_SIZE) return wram_mirror[a];
    if (a >= RCHEEVOS_SRAM_BASE && a < RCHEEVOS_SRAM_BASE + SRAM_MIRROR_SIZE)
        return sram_mirror[a - RCHEEVOS_SRAM_BASE];
    return 0;
}

/** Congela os bytes aprendidos (chamado imediatamente antes do do_frame). */
static void snapshot_refresh(void) {
    uint16_t n = snap_count;
    for (uint16_t i = 0; i < n; i++)
        snap_val[i] = mirror_read_byte(snap_addr[i]);
}

static uint32_t read_memory(uint32_t address, uint8_t *buffer,
                            uint32_t num_bytes, rc_client_t *client) {
    (void)client;
    qry_calls++;
    if (address >= RCHEEVOS_SRAM_BASE &&
        address < RCHEEVOS_SRAM_BASE + SRAM_MIRROR_SIZE) {
        qry_sram_calls++;
        uint8_t i;
        for (i = 0; i < qry_sram_n; i++)
            if (qry_sram_addr[i] == address) break;
        if (i == qry_sram_n && qry_sram_n < QRY_SRAM_MAX)
            qry_sram_addr[qry_sram_n++] = address;
    }
    for (uint32_t i = 0; i < num_bytes; i++) {
        uint32_t a = address + i;
        if (in_do_frame) {
            // Serve do snapshot congelado; miss = 1a consulta deste byte:
            // serve vivo e registra p/ entrar no proximo refresh.
            uint16_t k;
            for (k = 0; k < snap_count; k++) {
                if (snap_addr[k] == a) { buffer[i] = snap_val[k]; break; }
            }
            if (k == snap_count) {
                buffer[i] = mirror_read_byte(a);
                if (snap_count < SNAP_MAX) {
                    snap_addr[snap_count] = a;
                    snap_val[snap_count]  = buffer[i];
                    snap_count++;
                }
            }
        } else {
            buffer[i] = mirror_read_byte(a);   // PEEK/WATCH: espelho vivo
        }
    }
    return num_bytes;
}

// --- HTTP plumbing: server_call() turns rc_client requests into REQ= lines,
//     and http_callback() feeds RESP= bodies back into rc_client ---

static void http_callback(int status_code, const char *content, size_t content_size,
                          void *userdata, const char *error_message) {
    rc_api_server_response_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.body             = content;
    resp.body_length      = content_size;
    resp.http_status_code = status_code;
    if (status_code == 0 && error_message) {
        resp.body             = error_message;
        resp.body_length      = strlen(error_message);
        resp.http_status_code = RC_API_SERVER_RESPONSE_RETRYABLE_CLIENT_ERROR;
    }
    async_cb_t *cb = (async_cb_t *)userdata;
    cb->callback(&resp, cb->callback_data);
}

static void server_call(const rc_api_request_t *request,
                        rc_client_server_callback_t callback,
                        void *callback_data, rc_client_t *client) {
    (void)client;
    char buffer[640];
    const char *method = request->post_data ? "POST" : "GET";

    async_handlers[async_handlers_idx].id           = request_id_seq;
    async_handlers[async_handlers_idx].data.callback      = callback;
    async_handlers[async_handlers_idx].data.callback_data = callback_data;
    async_handlers_idx = (async_handlers_idx + 1) % MAX_ASYNC_CALLBACKS;

    snprintf(buffer, sizeof(buffer), "REQ=%02hhX;M:%s;U:%s;D:%s\r\n",
             request_id_seq, method, request->url,
             request->post_data ? request->post_data : "");
    request_id_seq++;
    request_ongoing++;
    last_request_ms = to_ms_since_boot(get_absolute_time());

    uart_send(buffer);
}

static rc_clock_t get_pico_millis(const rc_client_t *client) {
    (void)client;
    return to_ms_since_boot(get_absolute_time());
}

static void log_message(const char *message, const rc_client_t *client) {
    (void)client;
    printf("[rc] %s\n", message);
}

// --- rcheevos event handler: forward unlocks/progress to the FIFO ---

static void event_handler(const rc_client_event_t *event, rc_client_t *client) {
    (void)client;
    achievement_t a;
    memset(&a, 0, sizeof(a));
    switch (event->type) {
    case RC_CLIENT_EVENT_ACHIEVEMENT_TRIGGERED:
        a.id = event->achievement->id;
        a.event = event->type;
        ach_fifo_push(&ach_fifo, a);
        break;
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_SHOW:
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_HIDE:
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_UPDATE:
    case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_SHOW:
    case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_HIDE:
        a.event = event->type;
        if (event->achievement) {
            a.id = event->achievement->id;
            const char *p = event->achievement->measured_progress;
            strncpy(a.measured_progress, p && *p ? p : "0",
                    sizeof(a.measured_progress) - 1);
        }
        ach_fifo_push(&ach_fifo, a);
        break;
    default:
        break;
    }
}

// Held by core0 until the boot flow finishes (cart-ID + login + load_game);
// core1 blocks on entry before starting the bus PIOs + DMA.
static mutex_t bus_takeover_mutex;

// Solta o core1 (arma PIOs + DMA) exatamente uma vez -- e' chamado tanto nos
// caminhos de sucesso/falha do load_game quanto no boot do modo sniff-only.
static bool capture_started = false;

static void start_capture_once(void) {
    if (capture_started) return;
    capture_started = true;
    mutex_exit(&bus_takeover_mutex);
}

// --- Login + load_game callbacks ---

static void on_login(int result, const char *error_message,
                     rc_client_t *client, void *userdata) {
    (void)client; (void)userdata;
    if (result == RC_OK) {
        printf("rc login ok\n");
        state = 6;
    } else {
        // Login failed -- the SNES is still held in reset, which would
        // strand the user with a black screen forever. Boot the console
        // anyway (no achievement tracking) so they can at least play.
        printf("rc login failed: %s\n", error_message ? error_message : "?");
        uart_send("GAME_INFO=0;Login failed;\r\n");
        start_capture_once();
        sleep_ms(20);
        release_snes();
        state = 8;
    }
}

static void on_load_game(int result, const char *error_message,
                         rc_client_t *client, void *userdata) {
    (void)client; (void)userdata;
    if (result != RC_OK) {
        printf("rc load_game failed: %s\n", error_message ? error_message : "?");
        uart_send("GAME_INFO=0;No Title;No URL\r\n");
        // Even on load_game failure we must release the SNES -- otherwise
        // it stays stuck in reset forever (Q1 ON). The console will just
        // run without achievement tracking, which is a better UX than a
        // black screen.
        start_capture_once();
        sleep_ms(20);
        release_snes();
        state = 8;   // pretend in-game so the frame loop / FIFO drains run
        return;
    }

    const rc_client_game_t *game = rc_client_get_game_info(g_client);
    if (game) {
        char url[256];
        rc_client_game_get_image_url(game, url, sizeof(url));
        char line[512];
        snprintf(line, sizeof(line), "GAME_INFO=%lu;%s;%s\r\n",
                 (unsigned long)game->id, game->title, url);
        uart_send(line);
    }

    // Espelho integral: nao ha discovery pass. Garante espelhos zerados
    // antes do console comecar a escrever (o SNES ainda esta em reset).
    mirrors_reset();

    rc_client_user_game_summary_t summary;
    rc_client_get_user_game_summary(g_client, &summary);
    char line[64];
    snprintf(line, sizeof(line), "ACH_SUMMARY=%u;%u\r\n",
             summary.num_unlocked_achievements, summary.num_core_achievements);
    uart_send(line);

    // === SNES release (was previously done right after cart-ID) ===
    //
    // We held the SNES in reset throughout login + load_game so that the
    // very first WRAM write the console emits already lands in the capture
    // pipeline (mirrors zeroed, PIOs armed). Zero early-write loss.
    //
    // Order matters:
    //   1. mutex_exit  -- wakes core1, which spends ~10-15 ms bringing
    //                     PIO0 + DMA online and enabling capture.
    //   2. sleep_ms    -- buffer so core1 finishes setup before bus
    //                     activity starts. The SNES boot ROM takes
    //                     100+ ms to reach significant WRAM activity,
    //                     so 20 ms here is generous.
    //   3. release_snes -- /OE LOW (buffers active, SNES sees cart),
    //                      then /RESET released (SNES executes reset
    //                      vector). Game starts booting from here.
    start_capture_once();
    sleep_ms(20);
    release_snes();

    state = 8;
}

// --- rcheevos client init / teardown ---

static void rcheevos_init(void) {
    g_client = rc_client_create(read_memory, server_call);
    rc_client_enable_logging(g_client, RC_CLIENT_LOG_LEVEL_INFO, log_message);
    rc_client_set_hardcore_enabled(g_client, 1);   // real hardware = hardcore
    rc_client_set_event_handler(g_client, event_handler);
    rc_client_set_get_time_millisecs_function(g_client, get_pico_millis);
    rc_client_get_user_agent_clause(g_client, rcheevos_userdata, sizeof(rcheevos_userdata));
    printf("rc user_agent=%s\n", rcheevos_userdata);
}

// =============================================================================
// Outgoing achievement events (drain FIFO -> UART)
// =============================================================================

static void drain_ach_fifo(void) {
    if (request_ongoing != 0 || ach_fifo_empty(&ach_fifo)) return;
    achievement_t a;
    if (!ach_fifo_pop(&ach_fifo, &a)) return;

    char line[512];
    const rc_client_achievement_t *info = a.id ?
        rc_client_get_achievement_info(g_client, a.id) : NULL;

    switch (a.event) {
    case RC_CLIENT_EVENT_ACHIEVEMENT_TRIGGERED: {
        if (!info) break;
        char url[128];
        rc_client_achievement_get_image_url(info, RC_CLIENT_ACHIEVEMENT_STATE_UNLOCKED,
                                            url, sizeof(url));
        snprintf(line, sizeof(line), "A=%lu;%s;%s\r\n",
                 (unsigned long)info->id, info->title, url);
        uart_send(line);
        break;
    }
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_SHOW:
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_UPDATE:
    case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_SHOW: {
        if (!info) break;
        char url[128];
        rc_client_achievement_get_image_url(info, a.event, url, sizeof(url));
        const char *kind =
            (a.event == RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_SHOW) ? "C" : "P";
        snprintf(line, sizeof(line), "%s=S;%u;%s;%s;%s\r\n",
                 kind, (unsigned)info->id, info->title, url, info->measured_progress);
        uart_send(line);
        break;
    }
    case RC_CLIENT_EVENT_ACHIEVEMENT_PROGRESS_INDICATOR_HIDE:
        snprintf(line, sizeof(line), "P=H;%lu\r\n", (unsigned long)a.id);
        uart_send(line);
        break;
    case RC_CLIENT_EVENT_ACHIEVEMENT_CHALLENGE_INDICATOR_HIDE:
        snprintf(line, sizeof(line), "C=H;%lu\r\n", (unsigned long)a.id);
        uart_send(line);
        break;
    default:
        break;
    }
}

// =============================================================================
// Cart-info announcement (sent right after cart_id_phase completes)
// =============================================================================

static void send_cart_info(void) {
    if (!cart_info.header_valid) {
        uart_send("CART_INFO=?;0;0;0;0;0;0;\r\n");
        return;
    }
    md5_to_hex(cart_info.md5, cart_md5_hex);

    char line[512];
    snprintf(line, sizeof(line),
             "CART_INFO=%s;%lu;%lu;%02X;%02X;%02X;%04X;%s\r\n",
             cart_info.title,
             (unsigned long)(cart_info.rom_size_bytes  / 1024),
             (unsigned long)(cart_info.sram_size_bytes / 1024),
             cart_info.region,
             cart_info.map_mode,
             cart_info.rom_type,
             cart_info.checksum,
             cart_md5_hex);
    uart_send(line);
}

// =============================================================================
// Command handlers (ESP32 -> Pico messages)
// =============================================================================

static void handle_resp(const char *cmd) {
    // RESP=<id_hex>;<http_code_hex>;<body>
    const char *p = cmd + 5;
    char aux[8];

    strncpy(aux, p, 2); aux[2] = '\0';
    uint8_t id = (uint8_t)strtol(aux, NULL, 16);

    p += 3;
    strncpy(aux, p, 3); aux[3] = '\0';
    uint16_t http = (uint16_t)strtol(aux, NULL, 16);
    p += 4;

    if (request_ongoing) request_ongoing--;

    size_t body_len = strlen(p);

    // Telemetria de bring-up: tamanho do corpo recebido (truncamento de
    // payload grande aparece aqui na hora).
    printf("[resp] id=%02X http=%u len=%u\n", id, http, (unsigned)body_len);

    // Payload grande = provavelmente o patch: stub-a triggers gigantes
    // antes do parse (protege a heap) e mede o custo real do rcheevos.
    bool big = body_len > 8192;
    if (big) {
        // o buffer subjacente e' o serial_buffer (mutavel); o const vem
        // so da assinatura do handler
        body_len = filter_large_memaddr((char *)p, body_len);
        struct mallinfo mi = mallinfo();
        printf("[heap] antes do parse: usado=%d livre=%d\n",
               mi.uordblks, mi.fordblks);
    }

    for (int i = 0; i < MAX_ASYNC_CALLBACKS; i++) {
        if (async_handlers[i].id == id) {
            async_cb_t copy = async_handlers[i].data;
            http_callback(http, p, body_len, &copy, NULL);
            if (big) {
                struct mallinfo mi = mallinfo();
                printf("[heap] depois do parse: usado=%d livre=%d\n",
                       mi.uordblks, mi.fordblks);
            }
            return;
        }
    }
    printf("RESP id=%02X not matched\n", id);
}

static void handle_token_and_user(const char *cmd) {
    // TOKEN_AND_USER=<token>,<user>  -- TOKEN vem PRIMEIRO (fiel ao nome do
    // comando; conferido no snes-esp-firmware.ino: sprintf com
    // ra_user_token, ra_user nessa ordem).
    const char *p = cmd + strlen("TOKEN_AND_USER=");
    const char *comma = strchr(p, ',');
    if (!comma) return;

    size_t token_len = (size_t)(comma - p);
    size_t user_len  = strlen(comma + 1);
    while (user_len && (comma[1 + user_len - 1] == '\r' ||
                        comma[1 + user_len - 1] == '\n')) user_len--;

    if (token_len >= sizeof(ra_token)) token_len = sizeof(ra_token) - 1;
    if (user_len  >= sizeof(ra_user))  user_len  = sizeof(ra_user)  - 1;

    memset(ra_user,  0, sizeof(ra_user));
    memset(ra_token, 0, sizeof(ra_token));
    memcpy(ra_token, p, token_len);
    memcpy(ra_user,  comma + 1, user_len);

    printf("creds: user=%s token=%.4s...\n", ra_user, ra_token);
}

// =============================================================================
// Frame pacing (60Hz / 50Hz timer on core 0)
// =============================================================================

static uint64_t last_frame_us = 0;

// (definido na secao do B-bus, mais abaixo -- declaracao tentativa)
static volatile uint32_t vb_count;

// Fase 4d: pacing do rc_client_do_frame pelo vblank real detectado
// (rajada OAM DMA), com fallback por timer de 25ms para telas em
// force-blank / menus sem OAM DMA. VALIDADO E2E em 2026-07-11 (SMW @
// 60.2Hz medidos, rajada=548, trofeu desbloqueado no modo) -> DEFAULT ON.
// VSYNC=OFF via serial volta ao timer livre se precisar comparar.
static bool     vsync_on = true;
static uint32_t vsync_seen;
static volatile uint32_t frames_vb, frames_timer;   // telemetria

/**
 * Avaliacao ATOMICA: congela o snapshot dos enderecos de interesse e roda o
 * do_frame servindo leituras dele (gate in_do_frame) -- o rcheevos enxerga
 * um instante unico do espelho, sem tearing do core1 escrevendo no meio.
 */
static void do_frame_atomic(void) {
    snapshot_refresh();
    in_do_frame = true;
    rc_client_do_frame(g_client);
    in_do_frame = false;
}

static void maybe_do_frame(void) {
    if (state != 8) return;
    uint64_t now = time_us_64();

    if (vsync_on) {
        uint32_t vc = vb_count;
        if (vc != vsync_seen) {
            vsync_seen = vc;
            last_frame_us = now;
            frames_vb++;
            do_frame_atomic();
        } else if (now - last_frame_us >= 25000) {
            last_frame_us = now;
            frames_timer++;
            do_frame_atomic();
        }
        return;
    }

    uint64_t frame_us = (cart_info.header_valid && !cart_info.is_ntsc)
                            ? FRAME_TIME_PAL_US : FRAME_TIME_NTSC_US;
    if (now - last_frame_us >= frame_us) {
        last_frame_us = now;
        frames_timer++;
        do_frame_atomic();
    }
}

// =============================================================================
// One-shot detector: SNES has started touching the bus
// =============================================================================

static void maybe_announce_snes_running(void) {
    if (snes_running_announced) return;
    if (flag_bus_written) {
        snes_running_announced = true;
        uart_send("SNES_RESETED\r\n");
    }
}

// =============================================================================
// Core 1 -- consumers dos streams de captura
// =============================================================================
//
// Cada palavra e' UMA escrita ja amostrada no ponto estavel (captura
// one-shot no PIO) -- nao ha mais heuristica de "endereco mudou".
// As esperas do PIO nao sao atomicas (o barramento pode mudar entre um
// `wait` e o proximo), entao cada consumer valida os bits de controle
// capturados na propria palavra e descarta falsos gatilhos.

// -----------------------------------------------------------------------------
// Conserto de fiacao por software: par de linhas de endereco trocadas
// -----------------------------------------------------------------------------
//
// O chicote soldado do bring-up (2026-07-11) tem A6 <-> A12 invertidos
// (diagnostico via PEEKs: 0xDC1/0x13BF corrompidos, 0x13/0xDBE/0x100/0x17F5E
// intactos -- unica troca consistente). Este define destroca no consumer.
// AO RESSOLDAR (slot pino 11 -> GP6/W51, pino 37 -> GP12/W54), comente.
// #define ADDR_FIX_SWAP_A 6
// #define ADDR_FIX_SWAP_B 12

static inline uint32_t __not_in_flash_func(addr_fix)(uint32_t a) {
#ifdef ADDR_FIX_SWAP_A
    uint32_t x = ((a >> ADDR_FIX_SWAP_A) ^ (a >> ADDR_FIX_SWAP_B)) & 1u;
    a ^= (x << ADDR_FIX_SWAP_A) | (x << ADDR_FIX_SWAP_B);
#endif
    return a;
}

// Telemetria p/ DBG: escritas efetivamente aplicadas em cada espelho.
static volatile uint32_t stat_wram_commit, stat_cart_commit, stat_bport_commit;

// --- Resolucao de bank da WRAM via A22 (GP29) --------------------------------
// A22 desfaz a ambiguidade mirror vs $7E/$7F direto (a colisao $7F:0000-1FFF
// vs mirror de bank impar). Por PADRAO a resolucao e' SEMPRE exata: se o fio
// do A22 estiver ruim, isso APARECE (missample>0 no DBG + captura errada nos
// testes), em vez de ser silenciosamente mascarado.
//
// WRAM_BANK_FALLBACK (opt-in, default DESLIGADO): reativa o auto-detect do fio
// com rede de double-write, para um chicote SEM o fio do A22. Fica off de
// proposito -- uma rede silenciosa esconde uma solda ruim atras de "ta quase
// funcionando", que e' PIOR do que falhar alto. So ligue se souber que aquele
// hardware nao tem o A22 e voce aceita o veneno do double-write.
// #define WRAM_BANK_FALLBACK
typedef enum { BANKPAIR_EXACT = 0, BANKPAIR_PROBING, BANKPAIR_FALLBACK } bankpair_mode_t;
#ifdef WRAM_BANK_FALLBACK
// FORCADO em double-write: NUNCA le o GP29, NUNCA se auto-promove a EXACT.
// (A 1a versao bootava em PROBING e, com o fio lendo 1 nos acessos diretos,
// se promovia sozinha a EXACT em 64 escritas -- o "teste DW" de 15:21 rodou
// EXACT sem querer e invalidou o experimento de isolamento. Licao: build de
// isolamento nao pode ter caminho de promocao.)
static volatile uint8_t  bankpair_mode = BANKPAIR_FALLBACK;
static volatile uint32_t bankpair_confirms;   // morto (probe nunca roda)
#define BANKPAIR_CONFIRM_TARGET 64u
#else
static volatile uint8_t  bankpair_mode = BANKPAIR_EXACT;    // confia no A22, falha alto
#endif
// Missample: mirror (A22=0) com offset >= 0x2000 e' impossivel -> A22 falhou
// a amostra. SEMPRE contado -- e' o velocimetro da saude do fio no DBG
// (>0 sustentado = reflow no GP29/slot 47). No modo exato tratamos como direto.
static volatile uint32_t stat_a22_missample;

// Censo de classes de escrita na WRAM (auditoria do mirror no DBG). Cada
// jogo tem uma "assinatura" esperada:
//   - mirror  > 0 SEMPRE (pagina-zero/pilha vivem nos 8KB baixos via mirror);
//   - SMW: mirror dominante, dir7F ~0;
//   - Star Fox: dir7F alto (scratch do SuperFX) e SEPARADO do mirror.
// Absurdos denunciam bug/fio: mirror=0 e' impossivel; dir7F enorme no SMW
// e' suspeito; missample>0 = fio do A22.
//
// dir7f_lo e' O DISCRIMINADOR da "mentira para cima" do A22 (ler 1 numa
// escrita de mirror -- a direcao que o missample NAO enxerga): assinatura
// (A22=1, A16=1, offset<0x2000) = escrita direta em $7F:0000-1FFF. No SMW
// isso e' ~zero (ele nao usa $7F baixo); se aparecer aos MILHARES, sao
// mirrors de bank impar sendo desviados => o fio NAO carrega o A22 real
// (A17/A18/A19? solda no pino errado / ponte com vizinho no slot 47).
static volatile uint32_t stat_wram_dir7e, stat_wram_dir7f_lo,
                         stat_wram_dir7f_hi, stat_wram_mirror_w;

// GREP=<addr>: conta capturas comitadas num endereco especifico, separadas
// por origem (A-bus vs porta $2180). Distingue "o jogo nao escreve ai do
// jeito que enxergamos" de "escreve mas o espelho/leitura erra".
#define GREP_DISABLED 0xFFFFFFFFu
static volatile uint32_t grep_addr = GREP_DISABLED;
static volatile uint32_t grep_hits_raw;    // match no addr ANTES da validacao
static volatile uint32_t grep_hits_wram, grep_hits_bport;
static volatile uint8_t  grep_last_wram, grep_last_bport;
static volatile uint32_t grep_last_raw_word;   // palavra crua do ultimo match
// Rastreador de COMMITS: ring das ultimas 8 palavras cruas que efetivamente
// ESCREVERAM no endereco do GREP -- identifica o "vandalo" que pisoteia um
// byte (endereco capturado + dado + a22 + a16 de cada escrita, no DBG).
static volatile uint32_t grep_commit_ring[8];
static volatile uint8_t  grep_commit_idx;

// Log dos ultimos eventos da porta $2180-83 (diagnostico do ponteiro-sombra):
//   set_log: escritas em $2181/82/83 -> (port<<25)|(data<<17)|ponteiro_apos
//   wr_log : escritas em $2180       -> (data<<17)|ponteiro_usado
static volatile uint32_t bport_set_log[8], bport_wr_log[8];
static volatile uint8_t  bport_set_idx, bport_wr_idx;

static void __not_in_flash_func(consume_wram_word)(uint32_t w) {
    // Valida: /WRAM e /WR ativos no instante da amostra. /REFRESH entra
    // na palavra (bit 30) -- se o bring-up mostrar ciclos de refresh se
    // passando por escrita, adicionar `|| !LOW_REFRESH_N(w)` aqui.
    if (addr_fix(LOW_ADDR(w)) == grep_addr) {
        grep_hits_raw++;                 // capturado, ainda sem validacao
        grep_last_raw_word = w;
    }
    // /WR e /WRAM validados na propria palavra: descarta leitura de WRAM
    // (/WR alto) e falsos gatilhos onde /WRAM ja subiu. So escrita passa.
    if (LOW_WRAM_N(w) || LOW_WR_N(w)) return;
    uint32_t a = addr_fix(LOW_ADDR(w));
    uint8_t  d = (uint8_t)LOW_DATA(w);

#ifdef WRAM_BANK_FALLBACK
    // Prova do fio A22 (escrita inequivocamente direta: offset >= 0x2000).
    // So existe no modo com rede -- por padrao ja entramos direto em EXACT.
    if (bankpair_mode == BANKPAIR_PROBING && (a & 0xFFFFu) >= 0x2000u) {
        if (LOW_A22(w)) {
            if (++bankpair_confirms >= BANKPAIR_CONFIRM_TARGET)
                bankpair_mode = BANKPAIR_EXACT;
        } else {
            bankpair_mode = BANKPAIR_FALLBACK;   // pull-down leu 0: fio ausente
        }
    }
#endif

    if (bankpair_mode == BANKPAIR_EXACT) {
        // Resolucao exata pelo A22 (caminho PADRAO).
        uint32_t target;
        if (LOW_A22(w)) {
            target = a;                    // $7E/$7F direto (A16 e' o bit16)
            if (a >= 0x10000u) {
                if ((a & 0xFFFFu) < 0x2000u) stat_wram_dir7f_lo++;
                else                         stat_wram_dir7f_hi++;
            } else {
                stat_wram_dir7e++;
            }
        } else if ((a & 0xFFFFu) < 0x2000u) {
            target = a & 0x1FFFu;          // mirror -> sempre $7E baixo
            stat_wram_mirror_w++;
        } else {
            stat_a22_missample++;          // mirror c/ off>=2000 e' impossivel:
            target = a;                    // fio falhou -> trata direto e DENUNCIA
        }
        wram_mirror[target] = d;
        if (target == grep_addr) {
            grep_hits_wram++; grep_last_wram = d;
            grep_commit_ring[grep_commit_idx++ & 7u] = w;
        }
    }
#ifdef WRAM_BANK_FALLBACK
    else if (a >= 0x10000u && (a & 0xFFFFu) < 0x2000u) {
        // FALLBACK (fio ausente): double-write nos dois candidatos. O "lixo"
        // do $7F fica em 0x1xxxx, so lido se um achievement ler $7F.
        uint32_t off = a & 0x1FFFu;
        wram_mirror[off]            = d;
        wram_mirror[0x10000u | off] = d;
        if (off == grep_addr || (0x10000u | off) == grep_addr) {
            grep_hits_wram++; grep_last_wram = d;
            grep_commit_ring[grep_commit_idx++ & 7u] = w;
        }
    }
#endif
    else {
        // Escrita inequivoca (a<0x10000, ou offset>=0x2000 no modo com rede).
        wram_mirror[a] = d;
        if (a == grep_addr) {
            grep_hits_wram++; grep_last_wram = d;
            grep_commit_ring[grep_commit_idx++ & 7u] = w;
        }
    }
    stat_wram_commit++;
    if (!flag_bus_written) flag_bus_written = true;
}

// Escritas de cart VALIDADAS (controles ok) que a janela de SRAM rejeitou:
// separa "nenhuma escrita real chegou" de "chegou mas o filtro descartou"
// (caso SMK/DSP boards: a SRAM pode viver numa janela fora do padrao LoROM).
// O ring guarda as ultimas 8 palavras rejeitadas -- o DBG mostra ONDE o jogo
// realmente escreve, e ai ajustamos a janela com dado, nao com palpite.
static volatile uint32_t stat_cart_reject;
static volatile uint32_t cart_reject_ring[8];
static volatile uint8_t  cart_reject_idx;

static void __not_in_flash_func(consume_cart_word)(uint32_t w) {
    if (LOW_CART_N(w) || LOW_WR_N(w)) return;
    uint32_t sram_off = bus_to_sram_offset(addr_fix(LOW_ADDR(w)),
                                           cart_info.is_hirom,
                                           cart_info.sram_size_bytes);
    if (sram_off == BUS_NOT_SRAM || sram_off >= SRAM_MIRROR_SIZE) {
        stat_cart_reject++;
        cart_reject_ring[cart_reject_idx++ & 7u] = w;
        return;
    }
    sram_mirror[sram_off] = (uint8_t)LOW_DATA(w);
    if ((RCHEEVOS_SRAM_BASE + sram_off) == grep_addr) {
        grep_hits_wram++;                 // origem A-bus (cart)
        grep_last_wram = (uint8_t)LOW_DATA(w);
    }
    stat_cart_commit++;
    if (!flag_bus_written) flag_bus_written = true;
}

// --- SRAM de HiROM (escritas SEM /CART) --------------------------------------
//
// Em HiROM o /ROMSEL nao asserta na janela de SRAM ($3x/$Bx:6000-7FFF,
// A15=0) -- confirmado em hardware com FF6 (cart commits=0 durante saves).
// O SM snesBusHiSram captura escritas com /WRAM inativo; aqui filtramos a
// janela $6000-7FFF e so comitamos em cart HiROM. Sem A17-A23 nao da p/
// conferir o bank: escritas em $00-$2F:6000-7FFF (area aberta, jogos nao
// escrevem) seriam falsos positivos aceitos -- mesmo nivel de heuristica
// do caminho LoROM.
static volatile uint32_t stat_hisram_commit;

static void __not_in_flash_func(consume_hisram_word)(uint32_t w) {
    if (!LOW_WRAM_N(w) || LOW_WR_N(w)) return;   // exige /WRAM=1 e /WR=0
    if (!cart_info.is_hirom || cart_info.sram_size_bytes == 0) return;
    uint32_t off16 = LOW_ADDR(w) & 0xFFFFu;
    if (off16 < 0x6000u || off16 >= 0x8000u) return;   // fora da janela
    uint32_t sram_off = off16 - 0x6000u;
    if (sram_off >= cart_info.sram_size_bytes || sram_off >= SRAM_MIRROR_SIZE)
        return;
    sram_mirror[sram_off] = (uint8_t)LOW_DATA(w);
    if ((RCHEEVOS_SRAM_BASE + sram_off) == grep_addr) {
        grep_hits_wram++;
        grep_last_wram = (uint8_t)LOW_DATA(w);
    }
    stat_hisram_commit++;
    if (!flag_bus_written) flag_bus_written = true;
}

// --- Porta WRAM do B-bus ($2180-$2183) --------------------------------------
//
// O chip de WRAM tem uma segunda porta no B-bus com ponteiro interno:
//   $2180 (PA=80): dado    -- WRAM[ponteiro++]
//   $2181 (PA=81): ponteiro bits 7..0
//   $2182 (PA=82): ponteiro bits 15..8
//   $2183 (PA=83): ponteiro bit 16
// Escritas via $2180 (incluindo DMA ROM->WRAM, o caso comum de bulk copy)
// NAO colocam o endereco de WRAM no A-bus -- so sao visiveis por aqui.
// Decodificacao parcial: PA7=1 no B-bus so ocorre em $2180-83 (excecao
// conhecida: registradores BS-X $2188+, fora do escopo).
//
// Simplificacao conhecida: leituras via $2180 (/PARD) tambem incrementam o
// ponteiro e NAO sao rastreadas -- jogos reais setam o ponteiro antes de
// cada rajada, entao o desvio se auto-corrige. Se o bring-up mostrar
// problema, ha SM livre no PIO1 para um tracker de /PARD.

static uint32_t wm_pointer = 0;   // ponteiro sombra de $2181-83 (17 bits)

// -----------------------------------------------------------------------------
// Fase 4d: deteccao de vblank pela rajada do OAM DMA no B-bus
// -----------------------------------------------------------------------------
//
// Todo frame, o NMI handler dispara DMA de 544 bytes para $2104 (OAM):
// ~544 strobes /PAWR consecutivos com PA7=0, espacados ~373ns (1 byte a
// cada 8 master clocks). Assinatura: corrida de escritas PA7=0 CONTIGUAS
// no tempo. HDMA tambem escreve PA7=0, mas em grupos pequenos separados
// por ~63us (1 scanline) -- o gap reseta a corrida. Uploads grandes de
// VRAM tambem cruzam o limiar, mas ocorrem dentro do MESMO vblank; a
// janela de supressao de 10ms funde tudo em 1 evento por frame.
//
// Uso: telemetria sempre ativa (DBG secao 9). `VSYNC=ON` passa o
// rc_client_do_frame a disparar no vblank detectado (fallback por timer
// se nenhum vblank chegar em 25ms -- menus/force-blank sem OAM DMA).

#define VBLANK_BURST_MIN   300u    // OAM DMA = 544 escritas; margem folgada
#define VBLANK_GAP_US      30u     // gap maior que isso reseta a corrida
#define VBLANK_SUPPRESS_US 10000u  // 1 evento por frame

static volatile uint32_t vb_count;                  // vblanks detectados
static volatile uint32_t vb_delta_us;               // ultimo intervalo
static volatile uint32_t vb_min_us = 0xFFFFFFFFu;   // menor intervalo
static volatile uint32_t vb_max_us;                 // maior intervalo
static volatile uint32_t vb_anom;                   // fora de 16.0-17.4ms
static volatile uint32_t vb_last_len;               // tamanho da ultima corrida

// estado interno do detector (so core1)
static uint32_t vb_run, vb_prev_us, vb_fired_us, vb_last_evt_us;

static inline void __not_in_flash_func(vblank_track)(void) {
    uint32_t now = time_us_32();
    if (now - vb_prev_us > VBLANK_GAP_US) {
        if (vb_run) vb_last_len = vb_run;   // fecha a corrida anterior
        vb_run = 1u;
    } else {
        vb_run++;
    }
    vb_prev_us = now;

    if (vb_run == VBLANK_BURST_MIN &&
        (now - vb_fired_us) > VBLANK_SUPPRESS_US) {
        vb_fired_us = now;
        if (vb_count) {
            uint32_t d = now - vb_last_evt_us;
            vb_delta_us = d;
            if (d < vb_min_us) vb_min_us = d;
            if (d > vb_max_us) vb_max_us = d;
            if (d < 16000u || d > 17400u) vb_anom++;
        }
        vb_last_evt_us = now;
        vb_count++;
    }
}

static void __not_in_flash_func(consume_bport_word)(uint32_t w) {
    if (HI_PAWR_N(w)) return;
    if (!HI_PA7(w)) {
        // Escrita B-bus fora de $2180-83 (registradores PPU/APU):
        // alimenta o detector de rajada do OAM DMA (fase 4d).
        vblank_track();
        return;
    }
    if (vb_run) { vb_last_len = vb_run; vb_run = 0; }   // $218x fecha corrida
    uint32_t port = (HI_PA1(w) << 1) | HI_PA0(w);
    uint8_t  data = (uint8_t)HI_DATA(w);
    switch (port) {
    case 0:   // $2180: dado
        wram_mirror[wm_pointer] = data;
        if (wm_pointer == grep_addr) { grep_hits_bport++; grep_last_bport = data; }
        bport_wr_log[bport_wr_idx++ & 7u] = ((uint32_t)data << 17) | wm_pointer;
        wm_pointer = (wm_pointer + 1u) & 0x1FFFFu;
        stat_bport_commit++;
        if (!flag_bus_written) flag_bus_written = true;
        break;
    case 1:   // $2181
        wm_pointer = (wm_pointer & 0x1FF00u) | data;
        break;
    case 2:   // $2182
        wm_pointer = (wm_pointer & 0x100FFu) | ((uint32_t)data << 8);
        break;
    case 3:   // $2183
        wm_pointer = (wm_pointer & 0x0FFFFu) | ((uint32_t)(data & 1u) << 16);
        break;
    }
    if (port != 0) {
        bport_set_log[bport_set_idx++ & 7u] =
            (port << 25) | ((uint32_t)data << 17) | wm_pointer;
    }
}

/**
 * Drena um stream ate a posicao atual do DMA. Telemetria de quase-overrun:
 * se o backlog passar de 3/4 do ring, imprime o marcador (o ENDLESS ring
 * nao para -- um lap completo do produtor sobrescreve dados nao lidos).
 */
static inline void __not_in_flash_func(drain_stream)(bus_stream_t *s,
                                                     void (*consume)(uint32_t),
                                                     const char *ovr_tag) {
    uint32_t wr = stream_wr_pos(s);
    uint32_t backlog = (wr - s->rd) & s->mask;
    if (backlog > ((s->mask + 1u) * 3u) / 4u) printf("%s", ovr_tag);
    while (s->rd != wr) {
        uint32_t w = s->ring[s->rd];
        consume(w);
        s->last_word = w;
        s->words++;
        s->rd = (s->rd + 1u) & s->mask;
    }
}

static void __not_in_flash_func(core1_main)(void) {
    // Block here until core0 hands us the bus. This lets core0 perform the
    // cartridge ID phase (which drives the address pins as outputs) without
    // the capture path fighting for the bus GPIOs mid-read.
    mutex_enter_blocking(&bus_takeover_mutex);

    reset_bus_gpios();
    sleep_ms(10);
    setup_bus_pios();
    setup_dma();

    pio_sm_set_enabled(BUS_LOW_PIO,  BUS_WRAM_SM,  true);
    pio_sm_set_enabled(BUS_LOW_PIO,  BUS_CART_SM,  true);
    pio_sm_set_enabled(BUS_LOW_PIO,  BUS_HISRAM_SM, true);
    pio_sm_set_enabled(BUS_HIGH_PIO, BUS_BPORT_SM, true);
    // BUS_CARTHI_SM fica parado ate a fase 4b.

    while (true) {
        drain_stream(&s_wram,  consume_wram_word,  "!w");
        drain_stream(&s_cart,  consume_cart_word,  "!c");
        drain_stream(&s_hisram, consume_hisram_word, "!h");
        drain_stream(&s_bport, consume_bport_word, "!b");
    }
}

// =============================================================================
// DBG: raio-X do pipeline de captura (fios -> PIO -> DMA -> consumer)
// =============================================================================
//
// Imprime, em ordem de estagio:
//  1. atividade dos GPIOs em 50 ms (quais pinos MEXERAM) + niveis atuais
//     -> valida fios/solda: PHI2//WR//WRAM/A/D tem que mexer com console on
//  2. estado dos SMs de PIO + nivel dos RX FIFOs
//     -> FIFO enchendo mas DMA parado = problema no DMA
//  3. posicao de escrita do DMA vs leitura do consumer + palavras drenadas
//  4. commits nos espelhos + ultima palavra crua capturada (decodificada)

static void debug_dump(void) {
    printf("[dbg] v%s estado=%u captura=%s uptime=%lums\n",
           FIRMWARE_VERSION, (unsigned)state, capture_started ? "ON" : "off",
           (unsigned long)to_ms_since_boot(get_absolute_time()));

    // --- 1. atividade de pinos (50 ms de amostragem) ---
    uint32_t l0 = sio_hw->gpio_in, h0 = sio_hw->gpio_hi_in;
    uint32_t mov_lo = 0, mov_hi = 0;
    absolute_time_t t_end = make_timeout_time_ms(50);
    while (!time_reached(t_end)) {
        mov_lo |= sio_hw->gpio_in ^ l0;
        mov_hi |= sio_hw->gpio_hi_in ^ h0;
    }
    printf("[dbg] mexeu(50ms): lo=%08lX hi=%04lX\n",
           (unsigned long)mov_lo, (unsigned long)(mov_hi & 0xFFFFu));
    printf("[dbg] mexeu: PHI2=%d /WR=%d /WRAM=%d /CART=%d /PAWR=%d "
           "A0-15=%04lX D0-7=%02lX PA017=%d%d%d\n",
           (int)((mov_lo >> SNES_PHI2) & 1), (int)((mov_lo >> SNES_WR_N) & 1),
           (int)((mov_lo >> SNES_WRAM_N) & 1), (int)((mov_lo >> SNES_CART_N) & 1),
           (int)((mov_lo >> SNES_PAWR_N) & 1),
           (unsigned long)(mov_lo & 0xFFFFu),
           (unsigned long)((mov_lo >> SNES_D0) & 0xFFu),
           (int)((mov_hi >> (SNES_PA0 - 32)) & 1),
           (int)((mov_hi >> (SNES_PA1 - 32)) & 1),
           (int)((mov_hi >> (SNES_PA7 - 32)) & 1));
    printf("[dbg] nivel: PHI2=%d /WR=%d /WRAM=%d /CART=%d /RD=%d /REFRESH=%d "
           "/PAWR=%d /PARD=%d /RESET=%d\n",
           gpio_get(SNES_PHI2), gpio_get(SNES_WR_N), gpio_get(SNES_WRAM_N),
           gpio_get(SNES_CART_N), gpio_get(SNES_RD_N), gpio_get(SNES_REFRESH_N),
           gpio_get(SNES_PAWR_N), gpio_get(SNES_PARD_N), gpio_get(SNES_RESET_N));

    // --- 2. PIO ---
    printf("[dbg] pio0 ctrl=%02lX sm0(wram): rx=%u  sm1(cart): rx=%u | "
           "pio1 ctrl=%02lX sm0(bport): rx=%u\n",
           (unsigned long)(BUS_LOW_PIO->ctrl & 0xFu),
           (unsigned)pio_sm_get_rx_fifo_level(BUS_LOW_PIO, BUS_WRAM_SM),
           (unsigned)pio_sm_get_rx_fifo_level(BUS_LOW_PIO, BUS_CART_SM),
           (unsigned long)(BUS_HIGH_PIO->ctrl & 0xFu),
           (unsigned)pio_sm_get_rx_fifo_level(BUS_HIGH_PIO, BUS_BPORT_SM));

    // --- 3+4. streams / espelhos ---
    printf("[dbg] wram : dma=%lu rd=%lu words=%lu commits=%lu\n",
           (unsigned long)stream_wr_pos(&s_wram), (unsigned long)s_wram.rd,
           (unsigned long)s_wram.words, (unsigned long)stat_wram_commit);
    printf("[dbg] cart : dma=%lu rd=%lu words=%lu commits=%lu rejeitados=%lu\n",
           (unsigned long)stream_wr_pos(&s_cart), (unsigned long)s_cart.rd,
           (unsigned long)s_cart.words, (unsigned long)stat_cart_commit,
           (unsigned long)stat_cart_reject);
    if (stat_cart_reject) {
        // Escritas de cart validas que a janela de SRAM descartou: mostra
        // ONDE o jogo escreve de verdade (caso SMK: janela fora do padrao?)
        printf("[dbg] cart rejeitados (addr=dado, recentes):");
        for (uint32_t k = 1; k <= 8; k++) {
            uint32_t rw = cart_reject_ring[(cart_reject_idx - k) & 7u];
            printf(" %05lX=%02lX",
                   (unsigned long)LOW_ADDR(rw), (unsigned long)LOW_DATA(rw));
        }
        printf("\n");
    }
    printf("[dbg] hisram: dma=%lu rd=%lu words=%lu commits=%lu (so HiROM)\n",
           (unsigned long)stream_wr_pos(&s_hisram), (unsigned long)s_hisram.rd,
           (unsigned long)s_hisram.words, (unsigned long)stat_hisram_commit);
    printf("[dbg] bport: dma=%lu rd=%lu words=%lu commits=%lu\n",
           (unsigned long)stream_wr_pos(&s_bport), (unsigned long)s_bport.rd,
           (unsigned long)s_bport.words, (unsigned long)stat_bport_commit);

    uint32_t w = s_wram.last_word;
    printf("[dbg] ultima wram word=%08lX -> addr=%05lX data=%02lX "
           "phi2=%lu /wr=%lu /wram=%lu /cart=%lu /refresh=%lu\n",
           (unsigned long)w, (unsigned long)LOW_ADDR(w),
           (unsigned long)LOW_DATA(w), (unsigned long)LOW_PHI2(w),
           (unsigned long)LOW_WR_N(w), (unsigned long)LOW_WRAM_N(w),
           (unsigned long)LOW_CART_N(w), (unsigned long)LOW_REFRESH_N(w));

    // --- 5. fase do PHI2 em que /WR fica ativo (detecta clock invertido!) ---
    // 65816: dado valido com PHI2 ALTO. Se /WR ativo aparecer majoritariamente
    // com PHI2=0 no pino do slot, o clock esta invertido vs a suposicao do
    // PIO e estamos amostrando a fase do BANK BYTE (dado errado).
    uint32_t wr_phi_hi = 0, wr_phi_lo = 0, wram_wr = 0;
    t_end = make_timeout_time_ms(50);
    while (!time_reached(t_end)) {
        uint32_t v = sio_hw->gpio_in;
        if (!((v >> SNES_WR_N) & 1u)) {
            if ((v >> SNES_PHI2) & 1u) wr_phi_hi++; else wr_phi_lo++;
            if (!((v >> SNES_WRAM_N) & 1u)) wram_wr++;
        }
    }
    printf("[dbg] /WR ativo em 50ms: c/ PHI2=1: %lu | c/ PHI2=0: %lu | "
           "c/ /WRAM=0: %lu\n",
           (unsigned long)wr_phi_hi, (unsigned long)wr_phi_lo,
           (unsigned long)wram_wr);

    // --- 5b. /PAWR direto no pino (fase 4d): strobes de DMA visiveis? ---
    // 50ms = 3 frames. Se o OAM DMA estivesse visivel: >= ~1700 bordas
    // (3x544 + resto). ~40-60 bordas = so escritas de CPU chegam ao pino.
    // pa7=1 alto (>5% das amostras low) = tap do PA7 mentindo (APU $2140-43
    // sendo decodificado como $218x -> corrompe o ponteiro-sombra).
    uint32_t pawr_falls = 0, pawr_low = 0, pawr_low_pa7 = 0, prev_pawr = 1;
    t_end = make_timeout_time_ms(50);
    while (!time_reached(t_end)) {
        uint32_t lo = sio_hw->gpio_in;
        uint32_t hi = sio_hw->gpio_hi_in;
        uint32_t p  = (lo >> SNES_PAWR_N) & 1u;
        if (!p) {
            pawr_low++;
            if ((hi >> (SNES_PA7 - 32)) & 1u) pawr_low_pa7++;
            if (prev_pawr) pawr_falls++;
        }
        prev_pawr = p;
    }
    printf("[dbg] /PAWR 50ms: bordas=%lu amostras_low=%lu com_pa7=1: %lu\n",
           (unsigned long)pawr_falls, (unsigned long)pawr_low,
           (unsigned long)pawr_low_pa7);

    // --- 6. espelho de WRAM: quantos bytes != 0 e onde estao ---
    uint32_t nz = 0, shown = 0;
    printf("[dbg] espelho WRAM != 0:");
    for (uint32_t i = 0; i < WRAM_MIRROR_SIZE; i++) {
        if (wram_mirror[i]) {
            nz++;
            if (shown < 10) {
                printf(" %05lX:%02X", (unsigned long)i, wram_mirror[i]);
                shown++;
            }
        }
    }
    printf(" (total %lu bytes)\n", (unsigned long)nz);

    // --- 7. ultimas 8 palavras cruas do ring de WRAM ---
    printf("[dbg] ring wram (recentes):");
    for (uint32_t k = 1; k <= 8; k++) {
        uint32_t rw = s_wram.ring[(s_wram.rd - k) & s_wram.mask];
        printf(" %05lX=%02lX", (unsigned long)LOW_ADDR(rw),
               (unsigned long)LOW_DATA(rw));
    }
    printf("\n");

    // --- 8. porta $2180-83: ponteiro atual + ultimos eventos ---
    printf("[dbg] bport ptr=%05lX | sets(p:dado->ptr):",
           (unsigned long)wm_pointer);
    for (uint32_t k = 0; k < 8; k++) {
        uint32_t e = bport_set_log[(bport_set_idx + k) & 7u];
        printf(" %lu:%02lX->%05lX", (unsigned long)((e >> 25) & 3u),
               (unsigned long)((e >> 17) & 0xFFu),
               (unsigned long)(e & 0x1FFFFu));
    }
    printf("\n[dbg] bport writes(ptr=dado):");
    for (uint32_t k = 0; k < 8; k++) {
        uint32_t e = bport_wr_log[(bport_wr_idx + k) & 7u];
        printf(" %05lX=%02lX", (unsigned long)(e & 0x1FFFFu),
               (unsigned long)((e >> 17) & 0xFFu));
    }
    printf("\n");

    // --- 10. saude dos canais de DMA e PC dos SMs ---
    // ctrl: bit0=EN, BUSY e bits de erro (AHB/READ/WRITE nos bits altos).
    // Canal morto = BUSY 0 ou erro setado; write_addr congelado = DREQ.
    // PC do SM preso na mesma instrucao = wait nunca satisfeito.
    printf("[dbg] dma wram ch%d ctrl=%08lX wr=%08lX | cart ch%d ctrl=%08lX "
           "wr=%08lX | bport ch%d ctrl=%08lX wr=%08lX\n",
           s_wram.chan,
           (unsigned long)dma_channel_hw_addr(s_wram.chan)->ctrl_trig,
           (unsigned long)dma_channel_hw_addr(s_wram.chan)->write_addr,
           s_cart.chan,
           (unsigned long)dma_channel_hw_addr(s_cart.chan)->ctrl_trig,
           (unsigned long)dma_channel_hw_addr(s_cart.chan)->write_addr,
           s_bport.chan,
           (unsigned long)dma_channel_hw_addr(s_bport.chan)->ctrl_trig,
           (unsigned long)dma_channel_hw_addr(s_bport.chan)->write_addr);
    printf("[dbg] pio pc: wram=%u(base %u) cart=%u(base %u) | "
           "bport=%u(base %u) in_base_rel=%lu (esperado 0)\n",
           (unsigned)pio_sm_get_pc(BUS_LOW_PIO, BUS_WRAM_SM), pio_wram_offset,
           (unsigned)pio_sm_get_pc(BUS_LOW_PIO, BUS_CART_SM), pio_cart_offset,
           (unsigned)pio_sm_get_pc(BUS_HIGH_PIO, BUS_BPORT_SM), pio_bport_offset,
           (unsigned long)((BUS_HIGH_PIO->sm[BUS_BPORT_SM].pinctrl &
                            PIO_SM0_PINCTRL_IN_BASE_BITS)
                           >> PIO_SM0_PINCTRL_IN_BASE_LSB));

    // --- 11b. resolucao de bank via A22 (mirror vs $7E/$7F direto) ---
    // ordem casa com o enum: EXACT=0, PROBING=1, FALLBACK=2
    static const char *bp_names[] = { "EXACT", "PROBING", "FALLBACK" };
    printf("[dbg] bankpair=%s missample=%lu A22(GP29)=%d | classes: "
           "dir7E=%lu dir7F_lo=%lu dir7F_hi=%lu mirror=%lu\n",
           bp_names[bankpair_mode],
           (unsigned long)stat_a22_missample, gpio_get(SNES_RD_N),
           (unsigned long)stat_wram_dir7e,
           (unsigned long)stat_wram_dir7f_lo,
           (unsigned long)stat_wram_dir7f_hi,
           (unsigned long)stat_wram_mirror_w);
    printf("[dbg]   (SMW ground truth: dir7F_lo~0. Aos milhares = fio do "
           "A22 mentindo p/ cima -> mirrors de bank impar desviados)\n");

    // --- 11c. rastreador de commits do GREP: quem escreveu no endereco ---
    if (grep_addr != GREP_DISABLED) {
        printf("[dbg] grep commits (addr=dado a22.a16, recentes):");
        for (uint32_t k = 1; k <= 8; k++) {
            uint32_t rw = grep_commit_ring[(grep_commit_idx - k) & 7u];
            printf(" %05lX=%02lX %lu.%lu",
                   (unsigned long)LOW_ADDR(rw),
                   (unsigned long)LOW_DATA(rw),
                   (unsigned long)LOW_A22(rw),
                   (unsigned long)((rw >> 16) & 1u));
        }
        printf("\n");
    }

    // --- 12. espelho de SRAM: bytes != 0 (enderecos ja em espaco rcheevos) ---
    uint32_t snz = 0, sshown = 0;
    printf("[dbg] espelho SRAM != 0:");
    for (uint32_t i = 0; i < SRAM_MIRROR_SIZE; i++) {
        if (sram_mirror[i]) {
            snz++;
            if (sshown < 12) {
                printf(" %05lX:%02X",
                       (unsigned long)(RCHEEVOS_SRAM_BASE + i), sram_mirror[i]);
                sshown++;
            }
        }
    }
    printf(" (total %lu bytes)\n", (unsigned long)snz);

    // --- 11. enderecos que o rcheevos consulta (rastreio passivo) ---
    printf("[dbg] rc queries: total=%lu sram=%lu snapshot=%u bytes | "
           "sram unicos=%u:",
           (unsigned long)qry_calls, (unsigned long)qry_sram_calls,
           (unsigned)snap_count, (unsigned)qry_sram_n);
    for (uint8_t k = 0; k < qry_sram_n; k++) {
        printf(" %05lX=%02X", (unsigned long)qry_sram_addr[k],
               sram_mirror[qry_sram_addr[k] - RCHEEVOS_SRAM_BASE]);
    }
    printf(qry_sram_n ? "\n" : " (este set nao usa SRAM)\n");

    // --- 9. fase 4d: vblank pela rajada do OAM DMA ---
    printf("[dbg] vblank: n=%lu delta=%luus min=%lu max=%lu anom=%lu "
           "ultima_rajada=%lu | vsync=%s frames vb=%lu timer=%lu\n",
           (unsigned long)vb_count, (unsigned long)vb_delta_us,
           (unsigned long)vb_min_us, (unsigned long)vb_max_us,
           (unsigned long)vb_anom, (unsigned long)vb_last_len,
           vsync_on ? "ON" : "off",
           (unsigned long)frames_vb, (unsigned long)frames_timer);
}

// =============================================================================
// Deteccao de RESET do console (tap /RESET no GP43)
// =============================================================================
//
// Quando o usuario aperta reset (ou liga o console depois do Pico), o jogo
// recomeca do zero mas os espelhos e o estado do rcheevos ficam PODRES --
// hit counts, deltas e valores antigos podem gerar trigger falso. Aqui:
// /RESET precisa ficar ALTO por 500ms para ARMAR (garante que o fio existe
// e o console roda -- GP43 flutuando em 0 nunca arma, entao sem o fio a
// feature fica dormente). Depois de armado, /RESET em 0 por >30ms = reset
// real -> limpa espelhos, rc_client_reset() (API do rcheevos exatamente
// para isso: zera hits/estado, preserva unlocks) e avisa o ESP32.
//
// Feature que o adaptador NES nao tem ("console RESET detection is not
// yet implemented" no README de la desde sempre).

static void maybe_detect_console_reset(void) {
    static uint32_t last_ms, high_ms, low_ms;
    static bool armed;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now == last_ms) return;                 // amostra ~1x por ms
    uint32_t dt = now - last_ms;
    last_ms = now;

    if (gpio_get(SNES_RESET_N)) {
        high_ms += dt;
        low_ms = 0;
        if (!armed && high_ms > 500) {
            armed = true;
            printf("[reset] deteccao armada (fio do /RESET presente)\n");
        }
    } else {
        low_ms += dt;
        high_ms = 0;
        if (armed && low_ms > 30) {
            armed = false;                      // re-arma quando voltar a ALTO
            printf("[reset] console resetado -> espelhos limpos + rc_client_reset\n");
            mirrors_reset();
            wm_pointer = 0;
            if (g_client != NULL && state == 8) {
                rc_client_reset(g_client);
            }
            uart_send("SNES_RESETED\r\n");      // ESP32 redesenha a UI
        }
    }
}

// =============================================================================
// Banner ao conectar terminal USB
// =============================================================================
//
// Os prints de boot saem ~250 ms apos o reset -- ANTES do host enumerar a
// porta CDC (~1-2 s), entao se perdem. Este banner e' reimpresso toda vez
// que um terminal conecta (borda de subida de stdio_usb_connected), para a
// bancada nunca ficar "muda".

static bool usb_was_connected = false;

static void maybe_print_usb_banner(void) {
    bool now = stdio_usb_connected();
    if (now && !usb_was_connected) {
        sleep_ms(50);   // settle do DTR antes de imprimir
        printf("\n=== snes-pico-firmware v%s (build %s %s) ===\n",
               FIRMWARE_VERSION, __DATE__, __TIME__);
#ifdef ADDR_FIX_SWAP_A
        printf("addr-fix ATIVO: A%d<->A%d destrocados por software\n",
               ADDR_FIX_SWAP_A, ADDR_FIX_SWAP_B);
#endif
#ifdef SNIFF_ONLY_MODE
        printf("modo: SNIFF-ONLY (tabela de jogos, captura armada, nenhum pino dirigido)\n");
        printf("jogo ativo [%u]: %s (%s, SRAM=%luKB) -- GAME? lista, GAME=<n> troca\n",
               (unsigned)sniff_active_game,
               sniff_games[sniff_active_game].name,
               cart_info.is_hirom ? "HiROM" : "LoROM",
               (unsigned long)(cart_info.sram_size_bytes / 1024));
#else
        printf("modo: normal (isolamento %s) -- aguardando READ_CART\n",
#ifdef BUS_SWITCH_4066
               "4066"
#else
               "'245"
#endif
        );
#endif
        printf("estado=%u captura=%s uptime=%lums\n",
               (unsigned)state, capture_started ? "ON" : "off",
               (unsigned long)to_ms_since_boot(get_absolute_time()));
        printf("comandos: SYNC | DBG | READ_CART | PEEK=<addr hex> | "
               "WATCH=<addr hex>|OFF | VSYNC=ON|OFF | GREP=<addr>|OFF | "
               "GAME? | GAME=<n> | RESETSNES | RESET\n\n");
    }
    usb_was_connected = now;
}

// Reporter do GREP: imprime quando os contadores de captura mudam.
static uint32_t grep_seen_wram, grep_seen_bport;

static void maybe_grep(void) {
    if (grep_addr == GREP_DISABLED) return;
    uint32_t hw = grep_hits_wram, hb = grep_hits_bport;
    if (hw != grep_seen_wram || hb != grep_seen_bport) {
        // raw = ultima palavra crua que casou o endereco (pre-validacao):
        // a22/a16 dizem a CLASSE da escrita (direto $7E/$7F vs mirror).
        uint32_t rw = grep_last_raw_word;
        printf("[grep] %05lX: wram hits=%lu last=%02X | bport hits=%lu "
               "last=%02X | raw=%08lX a22=%lu a16=%lu\n",
               (unsigned long)grep_addr,
               (unsigned long)hw, grep_last_wram,
               (unsigned long)hb, grep_last_bport,
               (unsigned long)rw,
               (unsigned long)LOW_A22(rw),
               (unsigned long)((rw >> 16) & 1u));
        grep_seen_wram = hw; grep_seen_bport = hb;
        sleep_ms(50);   // limita a ~20 prints/s se o endereco for quente
    }
}

// =============================================================================
// Watch de endereco (calibracao: "sound test do SF2")
// =============================================================================
//
// WATCH=<addr hex> pelo USB/UART: core0 vigia o byte no espelho e imprime
// cada mudanca com timestamp -- muda o item no sound test, o valor tem que
// acompanhar. Funciona em qualquer estado com a captura armada.

#define WATCH_DISABLED 0xFFFFFFFFu
static uint32_t watch_addr = WATCH_DISABLED;
static uint8_t  watch_last = 0;

static void maybe_watch(void) {
    if (watch_addr == WATCH_DISABLED) return;
    uint8_t v = 0;
    read_memory(watch_addr, &v, 1, NULL);
    if (v != watch_last) {
        printf("[watch] %06lX: %02X -> %02X (%lums)\n",
               (unsigned long)watch_addr, watch_last, v,
               (unsigned long)to_ms_since_boot(get_absolute_time()));
        watch_last = v;
    }
}

// =============================================================================
// Main (core 0)
// =============================================================================

// =============================================================================
// UART command parser
// =============================================================================

static void process_serial_command(char *command) {
    if (prefix("RESP=", command)) {
        handle_resp(command);

    } else if (prefix("TOKEN_AND_USER=", command)) {
        handle_token_and_user(command);

    } else if (prefix("READ_CART", command)) {
        // Trigger cart-ID; main loop will pick this up at state 1.
        if (state == 0) state = 1;

    } else if (prefix("START_WATCH", command)) {
        // Init rcheevos and log the user in. on_login() advances state to 6.
        if (state == 2) {
            rcheevos_init();
            rc_client_begin_login_with_token(g_client, ra_user, ra_token,
                                             on_login, g_callback_userdata);
            state = 5;
        }

    } else if (prefix("SYNC", command)) {
        uart_send("SYNC_ACK\r\n");

    } else if (prefix("PEEK=", command)) {
        // Debug de bring-up: PEEK=<addr hex> -> byte do espelho (endereco
        // linear rcheevos: 0x000000 WRAM, 0x020000 SRAM).
        uint32_t a = (uint32_t)strtoul(command + 5, NULL, 16);
        uint8_t  v = 0;
        read_memory(a, &v, 1, NULL);
        char line[48];
        snprintf(line, sizeof(line), "PEEK=%06lX;%02X\r\n",
                 (unsigned long)a, v);
        uart_send(line);

    } else if (prefix("WATCH=", command)) {
        // Debug de bring-up: WATCH=<addr hex> imprime no USB toda mudanca do
        // byte no espelho (fluxo "sound test do SF2"). WATCH=OFF desliga.
        if (prefix("WATCH=OFF", command)) {
            watch_addr = WATCH_DISABLED;
            printf("[watch] off\n");
        } else {
            watch_addr = (uint32_t)strtoul(command + 6, NULL, 16);
            read_memory(watch_addr, (uint8_t *)&watch_last, 1, NULL);
            printf("[watch] %06lX (valor atual %02X)\n",
                   (unsigned long)watch_addr, watch_last);
        }

    } else if (prefix("VSYNC=", command)) {
        vsync_on = (bool)prefix("VSYNC=ON", command);
        vsync_seen = vb_count;
        printf("[vsync] %s\n",
               vsync_on ? "ON: do_frame no vblank detectado (fallback 25ms)"
                        : "OFF: timer livre 60/50Hz");

    } else if (prefix("GREP=", command)) {
        if (prefix("GREP=OFF", command)) {
            grep_addr = GREP_DISABLED;
            printf("[grep] off\n");
        } else {
            grep_hits_raw = grep_hits_wram = grep_hits_bport = 0;
            // aceita tambem espaco de SRAM (0x020000+)
            grep_addr = (uint32_t)strtoul(command + 5, NULL, 16) & 0x3FFFFu;
            printf("[grep] contando capturas em %05lX\n",
                   (unsigned long)grep_addr);
        }

    } else if (prefix("DBG", command)) {
        debug_dump();

#ifdef SNIFF_ONLY_MODE
    } else if (prefix("GAME?", command)) {
        printf("[game] tabela (GAME=<n> troca, persiste e reinicia):\n");
        for (uint32_t i = 0; i < SNIFF_GAME_COUNT; i++) {
            printf("  %lu%c %-22s %s SRAM=%uKB\n",
                   (unsigned long)i, i == sniff_active_game ? '*' : ' ',
                   sniff_games[i].name,
                   sniff_games[i].is_hirom ? "HiROM" : "LoROM",
                   sniff_games[i].sram_kb);
        }

    } else if (prefix("GAME=", command)) {
        uint32_t n = (uint32_t)strtoul(command + 5, NULL, 10);
        if (n < SNIFF_GAME_COUNT) {
            printf("[game] trocando para [%lu] %s -- persistindo e reiniciando...\n",
                   (unsigned long)n, sniff_games[n].name);
            sleep_ms(50);   // deixa o printf sair antes do reboot
            watchdog_hw->scratch[0] = GAME_SLOT_MAGIC;
            watchdog_hw->scratch[1] = n;
            watchdog_reboot(0, 0, 0);
        } else {
            printf("[game] indice invalido (0-%lu)\n",
                   (unsigned long)(SNIFF_GAME_COUNT - 1));
        }
#endif

    } else if (prefix("RESETSNES", command)) {
        // Reset forcado do CONSOLE via Q1 (GP44 -> R2 -> base). No chicote
        // sem Q1 e' um no-op inofensivo (GP44 esta no ar).
        printf("[reset] pulso de reset no console via Q1 (150ms)\n");
        gpio_put(SNES_RESET_DRIVE, 1);
        sleep_ms(150);
        gpio_put(SNES_RESET_DRIVE, 0);

    } else if (prefix("RESET", command)) {
        watchdog_reboot(0, 0, 0);

    } else {
        // Feedback so no USB (nao ecoar pro ESP32): linha nao reconhecida.
        printf("[?] comando desconhecido: %s\n", command);
    }
}

static void poll_uart(void) {
    while (uart_rx_tail != uart_rx_head) {
        char c = (char)uart_rx_ring[uart_rx_tail];
        uart_rx_tail = (uart_rx_tail + 1u) & (UART_RX_RING_SIZE - 1u);
        *serial_buffer_head++ = (uint8_t)c;

        if (serial_buffer_head - serial_buffer >= SERIAL_BUFFER_SIZE) {
            memset(serial_buffer, 0, SERIAL_BUFFER_SIZE);
            serial_buffer_head = serial_buffer;
            uart_send("BUFFER_OVERFLOW\r\n");
            return;
        }

        // Look for \r\n terminator (last two bytes).
        if (c == '\n' && serial_buffer_head - serial_buffer >= 2 &&
            *(serial_buffer_head - 2) == '\r') {

            int len = (int)(serial_buffer_head - serial_buffer);
            if (len > 2) {
                *(serial_buffer_head - 2) = '\0';        // strip CR
                process_serial_command((char *)serial_buffer);
            }
            memset(serial_buffer, 0, (size_t)len);
            serial_buffer_head = serial_buffer;
        }
    }
}

// -----------------------------------------------------------------------------
// Comandos via USB serial (stdio) -- bring-up sem ESP32
// -----------------------------------------------------------------------------
//
// Aceita o MESMO protocolo do link UART (READ_CART, PEEK=..., SYNC, RESET...)
// digitado num terminal no USB do WeAct. Terminador: \n ou \r (terminais
// variam). Buffer proprio pequeno -- respostas grandes continuam indo pelo
// uart_send (espelhadas no USB via MIRROR_UART_TO_USB).

#define USB_CMD_BUFFER_SIZE 256
static char usb_cmd_buffer[USB_CMD_BUFFER_SIZE];
static int  usb_cmd_len = 0;

static void poll_usb_commands(void) {
    for (;;) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) return;
        if (c == '\r' || c == '\n') {
            if (usb_cmd_len > 0) {
                usb_cmd_buffer[usb_cmd_len] = '\0';
                process_serial_command(usb_cmd_buffer);
                usb_cmd_len = 0;
            }
            continue;
        }
        if (usb_cmd_len < USB_CMD_BUFFER_SIZE - 1) {
            usb_cmd_buffer[usb_cmd_len++] = (char)c;
        } else {
            usb_cmd_len = 0;   // linha absurda: descarta
        }
    }
}

// =============================================================================
// Main (core 0)
// =============================================================================

int main(void) {
    // Step 0: pin safety defaults FIRST. Locks SNES in reset + isolated
    // before anything else touches the GPIOs.
    setup_safe_defaults();

    setup_clock();
    stdio_init_all();
    setup_uart();

#ifdef SNIFF_ONLY_MODE
    // Persistencia do jogo ativo: PRECISA rodar antes do core1 nascer --
    // flash_range_erase com codigo XIP executando no outro core = hardfault.
    game_slot_boot_init();
#endif

    mirrors_reset();
    ach_fifo_init(&ach_fifo);

    mutex_init(&bus_takeover_mutex);
    mutex_enter_blocking(&bus_takeover_mutex);   // core1 will block here
    multicore_launch_core1(core1_main);
    sleep_ms(250);    // folga p/ core1 iniciar e bloquear no mutex

#ifdef SNIFF_ONLY_MODE
    // Sniff-only: cart_info vem da tabela e captura armada desde o boot -- o
    // console roda livre (sem switches, sem Q1). PEEK=/WATCH= ja funcionam
    // pelo USB; READ_CART/START_WATCH seguem o fluxo normal com o MD5
    // da tabela, caso o ESP32 esteja conectado.
    sniff_fill_cart_info();
    start_capture_once();
    printf("[BOOT] SNIFF-ONLY: captura armada, nenhum pino sera dirigido\n");
#endif

    // Tell the ESP32 we're alive and what version we are.
    {
        char line[64];
        snprintf(line, sizeof(line), "PICO_FIRMWARE_VERSION=%s\r\n", FIRMWARE_VERSION);
        uart_send(line);
        uart_send("PICO_READY\r\n");
        // Mirror boot status to USB serial for debug purposes.
        printf("[BOOT] snes-pico-firmware v%s ready\n", FIRMWARE_VERSION);
    }

    while (true) {
        poll_uart();
        poll_usb_commands();   // mesmo protocolo via USB (bring-up sem ESP32)
        maybe_print_usb_banner();
        maybe_watch();         // WATCH=<addr>: imprime mudancas no espelho
        maybe_grep();          // GREP=<addr>: contadores de captura
        maybe_detect_console_reset();   // botao de reset do console (GP43)

        // ---- Cart-ID phase (gated on ESP32's READ_CART command) ----
        //
        // We do NOT release the SNES here. It stays held in reset (Q1 ON,
        // /RESET LOW) and isolated (BUFFER_OE_N HIGH) until on_load_game
        // finishes login + load_game with os espelhos prontos.
        // That keeps the very first WRAM write the console emits inside
        // a fully-prepared monitoring pipeline.
        if (state == 1) {
            cart_id_phase();
            send_cart_info();
            state = 2;
        }

        // ---- rcheevos load-game (triggered after login success) ----
        if (state == 6) {
            rc_client_begin_load_game(g_client, cart_md5_hex,
                                      on_load_game, g_callback_userdata);
            state = 7;
        }

        // ---- In-game: frame loop, FIFO drain, reset announcement ----
        if (state == 8) {
            maybe_announce_snes_running();
            maybe_do_frame();
            drain_ach_fifo();
        }

        // ---- HTTP request timeout ----
        // 120s: o download do patch (set de achievements) envolve fetch
        // HTTPS de ~100KB no ESP32-C3 + limpeza de JSON + transferencia
        // chunked pela UART -- 30s estourava em sets grandes (SMW).
        if (request_ongoing) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_request_ms > 120000) {
                printf("HTTP request timeout\n");
                request_ongoing = 0;
            }
        }
    }
    return 0;
}
