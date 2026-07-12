#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "unity.h"

// =============================================================================
// Host-side tests -- pinout v0.2 / firmware 0.7
//
// Cobre: macros de extracao das palavras dos PIOs, consumers (validacao de
// bits de controle + espelhos), tracker da porta $2180 do B-bus, parser de
// header e traducao de enderecos. Nao da' pra incluir main.c direto (puxa
// pico-sdk), entao a logica relevante e' re-declarada aqui.
// MANTER EM SINCRONIA com src/main.c.
// =============================================================================

// --- PIO0 word (in_base=0, bit = GPIO) --------------------------------------
#define LOW_ADDR(w)      ((w) & 0x1FFFFu)
#define LOW_DATA(w)      (((w) >> 17) & 0xFFu)
#define LOW_PHI2(w)      (((w) >> 25) & 0x1u)
#define LOW_WR_N(w)      (((w) >> 26) & 0x1u)
#define LOW_WRAM_N(w)    (((w) >> 27) & 0x1u)
#define LOW_CART_N(w)    (((w) >> 28) & 0x1u)
#define LOW_RD_N(w)      (((w) >> 29) & 0x1u)
#define LOW_REFRESH_N(w) (((w) >> 30) & 0x1u)
#define LOW_PAWR_N(w)    (((w) >> 31) & 0x1u)

// --- PIO1 word (gpio_base=16, bit = GPIO-16) ---------------------------------
#define HI_A16(w)        (((w) >>  0) & 0x1u)
#define HI_DATA(w)       (((w) >>  1) & 0xFFu)
#define HI_PHI2(w)       (((w) >>  9) & 0x1u)
#define HI_WR_N(w)       (((w) >> 10) & 0x1u)
#define HI_WRAM_N(w)     (((w) >> 11) & 0x1u)
#define HI_CART_N(w)     (((w) >> 12) & 0x1u)
#define HI_RD_N(w)       (((w) >> 13) & 0x1u)
#define HI_REFRESH_N(w)  (((w) >> 14) & 0x1u)
#define HI_PAWR_N(w)     (((w) >> 15) & 0x1u)
#define HI_ADDR_HIGH(w)  (((w) >> 16) & 0x7Fu)
#define HI_PA0(w)        (((w) >> 23) & 0x1u)
#define HI_PA1(w)        (((w) >> 24) & 0x1u)
#define HI_PA7(w)        (((w) >> 25) & 0x1u)
#define HI_PARD_N(w)     (((w) >> 26) & 0x1u)
#define HI_RESET_N(w)    (((w) >> 27) & 0x1u)

// --- Espelhos + consumers (espelho de src/main.c) ----------------------------

#define RCHEEVOS_SRAM_BASE  0x020000u
#define WRAM_MIRROR_SIZE    (128u * 1024u)
#define SRAM_MIRROR_SIZE    (32u * 1024u)
#define BUS_NOT_SRAM        0xFFFFFFFFu

static uint8_t wram_mirror[WRAM_MIRROR_SIZE];
static uint8_t sram_mirror[SRAM_MIRROR_SIZE];
static int flag_bus_written;

static struct { int is_hirom; uint32_t sram_size_bytes; } cart_info;

static uint32_t bus_to_sram_offset(uint32_t bus_addr, int is_hirom, uint32_t sram_size) {
    if (sram_size == 0) return BUS_NOT_SRAM;
    uint16_t offset = bus_addr & 0xFFFFu;
    if (!is_hirom) {
        if (offset < 0x8000u) {
            if (offset < sram_size) return offset;
        }
        return BUS_NOT_SRAM;
    } else {
        if (offset >= 0x6000u && offset < 0x8000u) {
            uint32_t off = (uint32_t)(offset - 0x6000u);
            if (off < sram_size) return off;
        }
        return BUS_NOT_SRAM;
    }
}

static void consume_wram_word(uint32_t w) {
    if (LOW_WRAM_N(w) || LOW_WR_N(w)) return;
    wram_mirror[LOW_ADDR(w)] = (uint8_t)LOW_DATA(w);
    flag_bus_written = 1;
}

static void consume_cart_word(uint32_t w) {
    if (LOW_CART_N(w) || LOW_WR_N(w)) return;
    uint32_t sram_off = bus_to_sram_offset(LOW_ADDR(w), cart_info.is_hirom,
                                           cart_info.sram_size_bytes);
    if (sram_off == BUS_NOT_SRAM || sram_off >= SRAM_MIRROR_SIZE) return;
    sram_mirror[sram_off] = (uint8_t)LOW_DATA(w);
    flag_bus_written = 1;
}

static uint32_t wm_pointer = 0;

static void consume_bport_word(uint32_t w) {
    if (HI_PAWR_N(w) || !HI_PA7(w)) return;
    uint32_t port = (HI_PA1(w) << 1) | HI_PA0(w);
    uint8_t  data = (uint8_t)HI_DATA(w);
    switch (port) {
    case 0:
        wram_mirror[wm_pointer] = data;
        wm_pointer = (wm_pointer + 1u) & 0x1FFFFu;
        flag_bus_written = 1;
        break;
    case 1: wm_pointer = (wm_pointer & 0x1FF00u) | data; break;
    case 2: wm_pointer = (wm_pointer & 0x100FFu) | ((uint32_t)data << 8); break;
    case 3: wm_pointer = (wm_pointer & 0x0FFFFu) | ((uint32_t)(data & 1u) << 16); break;
    }
}

void setUp(void) {
    memset(wram_mirror, 0, sizeof(wram_mirror));
    memset(sram_mirror, 0, sizeof(sram_mirror));
    flag_bus_written = 0;
    wm_pointer = 0;
    cart_info.is_hirom = 0;
    cart_info.sram_size_bytes = 0;
}
void tearDown(void) {}

// -----------------------------------------------------------------------------
// Helpers para montar palavras capturadas
// -----------------------------------------------------------------------------

// PIO0: linhas de controle default inativas (high), exceto o que o teste ativa
static uint32_t make_low_word(uint32_t addr17, uint8_t data,
                              int wram_n, int wr_n, int cart_n) {
    uint32_t w = 0;
    w |= (addr17 & 0x1FFFFu);
    w |= ((uint32_t)data) << 17;
    w |= 1u << 25;                    // PHI2 high (janela de amostra)
    w |= ((uint32_t)(wr_n   & 1)) << 26;
    w |= ((uint32_t)(wram_n & 1)) << 27;
    w |= ((uint32_t)(cart_n & 1)) << 28;
    w |= 1u << 29;                    // /RD inativo
    w |= 1u << 30;                    // /REFRESH inativo
    w |= 1u << 31;                    // /PAWR inativo
    return w;
}

// PIO1: palavra do strobe /PAWR (B-bus)
static uint32_t make_bport_word(uint8_t pa_low2, int pa7, uint8_t data) {
    uint32_t w = 0;
    w |= ((uint32_t)data) << 1;
    w |= 1u << 9;                     // PHI2 high
    w |= 1u << 10;                    // /WR inativo (A-bus)
    w |= 1u << 11;                    // /WRAM inativo
    w |= 1u << 12;                    // /CART inativo
    w |= 1u << 13;                    // /RD inativo
    w |= 1u << 14;                    // /REFRESH inativo
    // bit 15 (/PAWR) = 0 -> strobe ativo
    w |= ((uint32_t)(pa_low2 & 1)) << 23;        // PA0
    w |= ((uint32_t)((pa_low2 >> 1) & 1)) << 24; // PA1
    w |= ((uint32_t)(pa7 & 1)) << 25;            // PA7
    w |= 1u << 26;                    // /PARD inativo
    w |= 1u << 27;                    // /RESET solto
    return w;
}

// -----------------------------------------------------------------------------
// Macros de extracao
// -----------------------------------------------------------------------------

void test_low_word_wram_write(void) {
    uint32_t w = make_low_word(0x01234, 0x42, 0, 0, 1);
    TEST_ASSERT_EQUAL_HEX32(0x1234, LOW_ADDR(w));
    TEST_ASSERT_EQUAL_HEX8(0x42, LOW_DATA(w));
    TEST_ASSERT_EQUAL(1, LOW_PHI2(w));
    TEST_ASSERT_EQUAL(0, LOW_WRAM_N(w));
    TEST_ASSERT_EQUAL(0, LOW_WR_N(w));
    TEST_ASSERT_EQUAL(1, LOW_CART_N(w));
    TEST_ASSERT_EQUAL(1, LOW_RD_N(w));
    TEST_ASSERT_EQUAL(1, LOW_REFRESH_N(w));
    TEST_ASSERT_EQUAL(1, LOW_PAWR_N(w));
}

void test_low_word_full_address_and_data(void) {
    uint32_t w = make_low_word(0x1FFFF, 0xAB, 0, 0, 1);
    TEST_ASSERT_EQUAL_HEX32(0x1FFFF, LOW_ADDR(w));
    TEST_ASSERT_EQUAL_HEX8(0xAB, LOW_DATA(w));
    // dado 0xFF nao pode vazar no PHI2 (fronteira bit 24/25)
    w = make_low_word(0, 0xFF, 0, 0, 1);
    TEST_ASSERT_EQUAL_HEX8(0xFF, LOW_DATA(w));
    TEST_ASSERT_EQUAL_HEX32(0, LOW_ADDR(w));
}

void test_hi_word_layout(void) {
    uint32_t w = make_bport_word(0x0, 1, 0x99);
    TEST_ASSERT_EQUAL_HEX8(0x99, HI_DATA(w));
    TEST_ASSERT_EQUAL(1, HI_PA7(w));
    TEST_ASSERT_EQUAL(0, HI_PA0(w));
    TEST_ASSERT_EQUAL(0, HI_PA1(w));
    TEST_ASSERT_EQUAL(0, HI_PAWR_N(w));
    TEST_ASSERT_EQUAL(1, HI_PARD_N(w));
    // A17..A23 em bits 16..22
    uint32_t w2 = ((uint32_t)0x70) << 16;
    TEST_ASSERT_EQUAL_HEX8(0x70, HI_ADDR_HIGH(w2));
}

// -----------------------------------------------------------------------------
// Consumers: WRAM
// -----------------------------------------------------------------------------

void test_wram_consumer_commits_write(void) {
    consume_wram_word(make_low_word(0x00123, 0x55, 0, 0, 1));
    TEST_ASSERT_EQUAL_HEX8(0x55, wram_mirror[0x123]);
    TEST_ASSERT_TRUE(flag_bus_written);
}

void test_wram_consumer_rejects_false_trigger(void) {
    // /WRAM inativo na amostra (race das esperas do PIO) -> descarta
    consume_wram_word(make_low_word(0x00123, 0x55, 1, 0, 1));
    TEST_ASSERT_EQUAL_HEX8(0x00, wram_mirror[0x123]);
    // /WR inativo -> descarta
    consume_wram_word(make_low_word(0x00123, 0x66, 0, 1, 1));
    TEST_ASSERT_EQUAL_HEX8(0x00, wram_mirror[0x123]);
    TEST_ASSERT_FALSE(flag_bus_written);
}

void test_wram_consumer_last_write_wins(void) {
    consume_wram_word(make_low_word(0x1FFFF, 0x01, 0, 0, 1));
    consume_wram_word(make_low_word(0x1FFFF, 0x02, 0, 0, 1));
    TEST_ASSERT_EQUAL_HEX8(0x02, wram_mirror[0x1FFFF]);
}

// -----------------------------------------------------------------------------
// Consumers: cart/SRAM
// -----------------------------------------------------------------------------

void test_cart_consumer_lorom_sram_write(void) {
    cart_info.is_hirom = 0;
    cart_info.sram_size_bytes = 0x2000;                    // 8 KB
    consume_cart_word(make_low_word(0x00100, 0x77, 1, 0, 0));
    TEST_ASSERT_EQUAL_HEX8(0x77, sram_mirror[0x100]);
}

void test_cart_consumer_ignores_rom_region(void) {
    cart_info.is_hirom = 0;
    cart_info.sram_size_bytes = 0x2000;
    // offset >= $8000 = ROM (LoROM) -> nao e' SRAM
    consume_cart_word(make_low_word(0x08000, 0x77, 1, 0, 0));
    for (uint32_t i = 0; i < 0x2000; i++)
        TEST_ASSERT_EQUAL_HEX8(0x00, sram_mirror[i]);
}

void test_cart_consumer_hirom_window(void) {
    cart_info.is_hirom = 1;
    cart_info.sram_size_bytes = 0x2000;
    consume_cart_word(make_low_word(0x06100, 0x88, 1, 0, 0));   // $6100
    TEST_ASSERT_EQUAL_HEX8(0x88, sram_mirror[0x100]);
}

void test_cart_consumer_rejects_false_trigger(void) {
    cart_info.is_hirom = 0;
    cart_info.sram_size_bytes = 0x2000;
    consume_cart_word(make_low_word(0x00100, 0x77, 1, 0, 1));   // /CART inativo
    TEST_ASSERT_EQUAL_HEX8(0x00, sram_mirror[0x100]);
}

// -----------------------------------------------------------------------------
// Consumer: porta $2180-83 do B-bus
// -----------------------------------------------------------------------------

static void bport_set_pointer(uint32_t ptr) {
    consume_bport_word(make_bport_word(1, 1, (uint8_t)(ptr & 0xFF)));         // $2181
    consume_bport_word(make_bport_word(2, 1, (uint8_t)((ptr >> 8) & 0xFF)));  // $2182
    consume_bport_word(make_bport_word(3, 1, (uint8_t)((ptr >> 16) & 1)));    // $2183
}

void test_bport_pointer_set_and_write(void) {
    bport_set_pointer(0x01234);
    consume_bport_word(make_bport_word(0, 1, 0xAA));   // $2180
    TEST_ASSERT_EQUAL_HEX8(0xAA, wram_mirror[0x1234]);
}

void test_bport_autoincrement(void) {
    bport_set_pointer(0x00100);
    consume_bport_word(make_bport_word(0, 1, 0x01));
    consume_bport_word(make_bport_word(0, 1, 0x02));
    consume_bport_word(make_bport_word(0, 1, 0x03));
    TEST_ASSERT_EQUAL_HEX8(0x01, wram_mirror[0x100]);
    TEST_ASSERT_EQUAL_HEX8(0x02, wram_mirror[0x101]);
    TEST_ASSERT_EQUAL_HEX8(0x03, wram_mirror[0x102]);
}

void test_bport_pointer_wraps_17_bits(void) {
    bport_set_pointer(0x1FFFF);
    consume_bport_word(make_bport_word(0, 1, 0x11));   // escreve em 0x1FFFF
    consume_bport_word(make_bport_word(0, 1, 0x22));   // wrap -> 0x00000
    TEST_ASSERT_EQUAL_HEX8(0x11, wram_mirror[0x1FFFF]);
    TEST_ASSERT_EQUAL_HEX8(0x22, wram_mirror[0x00000]);
}

void test_bport_ignores_non_wram_port(void) {
    // PA7=0 -> registrador PPU/APU qualquer ($21xx com PA7=0), nao e' $2180
    consume_bport_word(make_bport_word(0, 0, 0xAA));
    TEST_ASSERT_EQUAL_HEX8(0x00, wram_mirror[0]);
    TEST_ASSERT_EQUAL_HEX32(0, wm_pointer);
}

void test_bport_ignores_inactive_strobe(void) {
    uint32_t w = make_bport_word(0, 1, 0xAA) | (1u << 15);   // /PAWR inativo
    consume_bport_word(w);
    TEST_ASSERT_EQUAL_HEX8(0x00, wram_mirror[0]);
}

void test_bport_partial_pointer_update(void) {
    // Jogos frequentemente so reescrevem $2181 mantendo $2182/83
    bport_set_pointer(0x00500);
    consume_bport_word(make_bport_word(1, 1, 0x80));   // $2181 = 0x80
    consume_bport_word(make_bport_word(0, 1, 0xCC));
    TEST_ASSERT_EQUAL_HEX8(0xCC, wram_mirror[0x580]);
}

// -----------------------------------------------------------------------------
// SNES header parser + address translator (inalterados da v0.6)
// -----------------------------------------------------------------------------

#define HDR_OFF_CSUM_COMPLEMENT 0x2C
#define HDR_OFF_CSUM            0x2E

static int snes_header_validate(const uint8_t *hdr) {
    uint16_t cs = (uint16_t)hdr[HDR_OFF_CSUM]            |
                  ((uint16_t)hdr[HDR_OFF_CSUM + 1] << 8);
    uint16_t cc = (uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT] |
                  ((uint16_t)hdr[HDR_OFF_CSUM_COMPLEMENT + 1] << 8);
    return (uint16_t)(cs ^ cc) == 0xFFFFu;
}

static uint32_t rom_offset_to_bus(int is_hirom, uint32_t rom_offset) {
    if (is_hirom) return 0xC00000u | rom_offset;
    return ((rom_offset >> 15) << 16) | ((rom_offset & 0x7FFFu) | 0x8000u);
}

static int region_is_ntsc(uint8_t r) {
    return (r == 0x00 || r == 0x01 || r == 0x0B || r == 0x0D || r == 0x0F);
}

void test_header_validate_good_checksum(void) {
    uint8_t hdr[80] = {0};
    hdr[HDR_OFF_CSUM_COMPLEMENT]     = 0x5A;
    hdr[HDR_OFF_CSUM_COMPLEMENT + 1] = 0x3C;
    hdr[HDR_OFF_CSUM]                = 0xA5;
    hdr[HDR_OFF_CSUM + 1]            = 0xC3;
    TEST_ASSERT_TRUE(snes_header_validate(hdr));
}

void test_header_validate_bad_checksum(void) {
    uint8_t hdr[80] = {0};
    hdr[HDR_OFF_CSUM]     = 0xA5;
    hdr[HDR_OFF_CSUM + 1] = 0xC3;
    TEST_ASSERT_FALSE(snes_header_validate(hdr));
}

void test_header_validate_open_bus_garbage(void) {
    uint8_t hdr[80];
    memset(hdr, 0xFF, sizeof(hdr));
    TEST_ASSERT_FALSE(snes_header_validate(hdr));
}

void test_lorom_address_translation(void) {
    TEST_ASSERT_EQUAL_HEX32(0x008000u, rom_offset_to_bus(0, 0x00000));
    TEST_ASSERT_EQUAL_HEX32(0x00FFFFu, rom_offset_to_bus(0, 0x07FFF));
    TEST_ASSERT_EQUAL_HEX32(0x018000u, rom_offset_to_bus(0, 0x08000));
    TEST_ASSERT_EQUAL_HEX32(0x00FFB0u, rom_offset_to_bus(0, 0x07FB0));
    TEST_ASSERT_EQUAL_HEX32(0x208000u, rom_offset_to_bus(0, 0x100000));
}

void test_hirom_address_translation(void) {
    TEST_ASSERT_EQUAL_HEX32(0xC00000u, rom_offset_to_bus(1, 0x00000));
    TEST_ASSERT_EQUAL_HEX32(0xC0FFB0u, rom_offset_to_bus(1, 0x0FFB0));
    TEST_ASSERT_EQUAL_HEX32(0xC10000u, rom_offset_to_bus(1, 0x10000));
    TEST_ASSERT_EQUAL_HEX32(0xC3FFFFu, rom_offset_to_bus(1, 0x3FFFF));
}

void test_region_classification(void) {
    TEST_ASSERT_TRUE(region_is_ntsc(0x00));
    TEST_ASSERT_TRUE(region_is_ntsc(0x01));
    TEST_ASSERT_TRUE(region_is_ntsc(0x0B));
    TEST_ASSERT_FALSE(region_is_ntsc(0x02));
    TEST_ASSERT_FALSE(region_is_ntsc(0x09));
}

// -----------------------------------------------------------------------------
// bus_to_sram_offset (inalterado da v0.6)
// -----------------------------------------------------------------------------

void test_sram_lorom_basic(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0100u, bus_to_sram_offset(0x700100u, 0, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(0x1FFFu, bus_to_sram_offset(0x701FFFu, 0, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(0x0100u, bus_to_sram_offset(0xF00100u, 0, 0x2000u));
}

void test_sram_lorom_out_of_range(void) {
    TEST_ASSERT_EQUAL_HEX32(BUS_NOT_SRAM, bus_to_sram_offset(0x708000u, 0, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(BUS_NOT_SRAM, bus_to_sram_offset(0x705000u, 0, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(BUS_NOT_SRAM, bus_to_sram_offset(0x700100u, 0, 0));
}

void test_sram_hirom_window(void) {
    TEST_ASSERT_EQUAL_HEX32(0x0100u, bus_to_sram_offset(0x306100u, 1, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(0x0000u, bus_to_sram_offset(0x306000u, 1, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(0x1FFFu, bus_to_sram_offset(0x307FFFu, 1, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(BUS_NOT_SRAM, bus_to_sram_offset(0x305FFFu, 1, 0x2000u));
    TEST_ASSERT_EQUAL_HEX32(BUS_NOT_SRAM, bus_to_sram_offset(0x308000u, 1, 0x2000u));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_low_word_wram_write);
    RUN_TEST(test_low_word_full_address_and_data);
    RUN_TEST(test_hi_word_layout);
    RUN_TEST(test_wram_consumer_commits_write);
    RUN_TEST(test_wram_consumer_rejects_false_trigger);
    RUN_TEST(test_wram_consumer_last_write_wins);
    RUN_TEST(test_cart_consumer_lorom_sram_write);
    RUN_TEST(test_cart_consumer_ignores_rom_region);
    RUN_TEST(test_cart_consumer_hirom_window);
    RUN_TEST(test_cart_consumer_rejects_false_trigger);
    RUN_TEST(test_bport_pointer_set_and_write);
    RUN_TEST(test_bport_autoincrement);
    RUN_TEST(test_bport_pointer_wraps_17_bits);
    RUN_TEST(test_bport_ignores_non_wram_port);
    RUN_TEST(test_bport_ignores_inactive_strobe);
    RUN_TEST(test_bport_partial_pointer_update);
    RUN_TEST(test_header_validate_good_checksum);
    RUN_TEST(test_header_validate_bad_checksum);
    RUN_TEST(test_header_validate_open_bus_garbage);
    RUN_TEST(test_lorom_address_translation);
    RUN_TEST(test_hirom_address_translation);
    RUN_TEST(test_region_classification);
    RUN_TEST(test_sram_lorom_basic);
    RUN_TEST(test_sram_lorom_out_of_range);
    RUN_TEST(test_sram_hirom_window);
    return UNITY_END();
}
