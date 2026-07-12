# SNES-RA-Adapter — Pico Firmware (RP2350B)

Firmware do Raspberry Pi Pico 2 (RP2350B, QFN-80 com 48 GPIOs) para o adaptador
RetroAchievements do Super Nintendo. Baseado na arquitetura validada do
[`nes-pico-firmware`](../nes-pico-firmware), com as adaptações necessárias para
o barramento mais largo, clock mais rápido e WRAM de 128KB do SNES.

> **Status:** v0.6-skeleton — fim-a-fim funcional **em estrutura**:
> captura WRAM (PIO0 SM0 + DMA), **captura cart-writes para SRAM ≤32KB
> LoROM / ≤8KB HiROM (PIO0 SM1 + DMA)**, identificação de cartucho com MD5
> streaming + parse de header, protocolo serial com ESP32 (compatível NES),
> cliente rcheevos + frame loop + discovery pass. Falta: PIO1 para SRAM
> grande (Fase 4b) e bring-up em hardware real.

---

## Visão geral da arquitetura

| Bloco             | NES                          | SNES                                       |
|-------------------|------------------------------|--------------------------------------------|
| MCU               | Pico 1 (RP2040, 30 GPIOs)    | Pico 2 (RP2350B, **48 GPIOs**)             |
| Largura captura   | 1 PIO, 32 bits               | **2 PIOs, 32 bits cada**                   |
| Address bus       | 16 bits (A0–A15)             | 24 bits (A0–A23)                           |
| Shadow RAM        | 2KB WRAM + 8KB SRAM          | **128KB WRAM** + até 128KB SRAM            |
| Clock CPU alvo    | 1.79 MHz (NTSC)              | 3.58 MHz (fast) / 2.68 MHz (slow)          |
| Level shifter     | Necessário (5V)              | **Não usado** (RP2350 5V-tolerant nos IN)  |
| ESP32 link        | UART 115200 8N1              | UART 115200 8N1 (idêntico)                 |

A grande mudança em relação ao NES é a **divisão da captura em dois PIOs**.
No pinout v0.2, os sinais compartilhados (D0–D7, PHI2, /WR, /WRAM, /CART,
/RD, /REFRESH, /PAWR) vivem na **janela de overlap GPIO 16–31**, visível
tanto ao PIO0 (janela 0–31) quanto ao PIO1 (janela 16–47, via
`pio_set_gpio_base(pio1, 16)`). Isso elimina os GPIOs duplicados da v0.1 e
liberou pinos para os taps do B-bus (PA0/PA1/PA7, /PAWR, /PARD), que
permitem rastrear a porta de WRAM `$2180–$2183` e detectar vblank real.

---

## Pinout v0.2 (RP2350B QFN-80)

> Implementado no firmware **0.7-skeleton** (`main.c` + `snes-bus.pio`).
> Fonte de verdade: `../snes-ra-adapter-pcb/docs/netlist.md`.
> Build: `PICO_BOARD=solderparty_rp2350_stamp_xl` (RP2350B, 48 GPIOs —
> `pico2` é RP2350A/30 GPIOs e não serve).

> Endereços/dados/sinais lidos do barramento do cartucho **sem level shifter
> dedicado**: o RP2350 (stepping atual) é 5V-tolerant em entradas digitais;
> saídas (modo identificação de cartucho) ficam em 3.3V, que as ROMs
> interpretam como HIGH (premissa validada no adaptador NES).

```
GPIO  Função              Direção      Notas
─────────────────────────────────────────────────────────────────────────
 0-15 A0–A15              IN/OUT*      lado cart de U1/U2 (*OUT só no cart-ID)
 16   A16                 IN/OUT*      lado cart de U3
 17-24 D0–D7              IN           lado cart de U4 (CBT3245)
 25   PHI2                IN           tap direto (sem buffer)
 26   /WR                 IN/OUT*      lado cart de U5
 27   /WRAM               IN           tap direto
 28   /CART               IN/OUT*      lado cart de U5
 29   /RD                 IN/OUT*      lado cart de U5
 30   /REFRESH            IN           tap direto (anti-falso-write)
 31   /PAWR               IN           tap direto (trigger $2180)
 32-38 A17–A23            IN/OUT*      lado cart de U3 (fase 4b + cart-ID)
 39   PA0                 IN           tap direto ┐ decodificação parcial
 40   PA1                 IN           tap direto │ do B-bus: PA7=1 só em
 41   PA7                 IN           tap direto ┘ $2180-83 (porta WRAM)
 42   /PARD               IN           tap direto (increment do ponteiro)
 43   /RESET              IN           tap direto (monitor)
 44   RESET_DRIVE         OUT          → R2 → Q1 (segura console em reset)
 45   BUFFER_OE_N         OUT          → /OE de U1..U5 (pull-up 10k)
 46   UART0 TX            OUT          → ESP32 RX
 47   UART0 RX            IN           ← ESP32 TX
```

### Janelas dos PIOs (sem sinais duplicados)

- **PIO0**: `gpio_base=0`, janela GPIO 0–31. Word (bit = GPIO):
  0–15 `A0–A15`, 16 `A16`, 17–24 `D0–D7`, 25 `PHI2`, 26 `/WR`, 27 `/WRAM`,
  28 `/CART`, 29 `/RD`, 30 `/REFRESH`, 31 `/PAWR`.
- **PIO1**: **`pio_set_gpio_base(pio1, 16)`** (obrigatório no RP2350B!),
  janela GPIO 16–47. Word (bit = GPIO−16): 0 `A16`, 1–8 `D0–D7`, 9 `PHI2`,
  10 `/WR`, 11 `/WRAM`, 12 `/CART`, 13 `/RD`, 14 `/REFRESH`, 15 `/PAWR`,
  16–22 `A17–A23`, 23 `PA0`, 24 `PA1`, 25 `PA7`, 26 `/PARD`, 27 `/RESET`,
  28–31 mascarar (RESET_DRIVE, /OE, UART).

Os sinais compartilhados vivem na **janela de overlap 16–31**, então os dois
PIOs condicionam a captura nos mesmos pinos físicos — sem IRQ inter-PIO e
sem GPIOs duplicados (que a v0.1 usava e custavam 5 pinos).

State machines previstas:

| SM | Trigger | Função |
|---|---|---|
| PIO0 SM0 | /WRAM+/WR+PHI2 | escritas WRAM (A-bus) |
| PIO0 SM1 | /CART+/WR+PHI2 | escritas SRAM do cart |
| PIO1 SM0 | /PAWR (+PA7 no consumer) | porta WRAM $2180–83 (escritas via DMA/porta) |
| PIO1 SM1 | /CART+/WR+PHI2 | A17–A23 p/ SRAM grande (fase 4b, pareada com PIO0 SM1) |

> **Captura 1-palavra-por-ciclo**: os programas PIO devem amostrar UMA vez
> por ciclo de escrita (delay fixo após PHI2 subir, re-arma só depois de /WR
> subir), em vez do flood-sampling herdado do NES. Isso corta ~8× o tráfego
> de DMA (a causa do overflow na PoC) e permite parear PIO0 SM1 ↔ PIO1 SM1
> palavra a palavra.

### Por que UART em GP46/47

O RP2350B oferece UART0 em várias opções (GP0/1, GP12/13, GP16/17, GP28/29,
GP32/33, GP44/45, **GP46/47**). Escolhemos **GP46/47** porque:

1. Ficam **no extremo final** dos GPIOs, longe dos pinos de barramento.
2. Estão na "função padrão" da página de UART do RP2350 — roteamento limpo.
3. Mesmo caindo dentro da janela de captura do PIO1 (16–47), não causam
   problema: são os 2 bits mais significativos da palavra, fáceis de mascarar.

---

## Mapa de memória rcheevos para SNES

O backend RetroAchievements expõe o mapa de memória SNES como uma região
linear. Nosso shadow buffer segue:

```
0x000000 – 0x01FFFF   WRAM   (128KB, banks 7E/7F)
0x020000 – 0x03FFFF   SRAM   (até 128KB, varia por cartucho)
```

A captura em A0–A16 dá o offset direto dentro da WRAM (os 128KB são
endereçados linearmente pelo barramento físico — todos os mirrors de
`$00:0000-$00:1FFF`, `$80:0000-$80:1FFF` etc. são transparentes para nós).

### Estratégia de alocação: hashmap esparso (bus_track)

Em vez de espelhar 128 KB de WRAM + até 128 KB de SRAM, mantemos um único
**hashmap open-addressed** que armazena apenas os endereços que o rcheevos
pede. Um jogo típico do SNES tem 50–500 endereços únicos referenciados em
achievements, então a tabela cabe folgada em ~8 KB.

**Estrutura:**

- Capacidade fixa: 2048 slots × 4 bytes = **8 KB total** (cobre WRAM + SRAM)
- Cada slot é uma palavra de 32 bits empacotada:
  - bits 19..0 : endereço linear rcheevos (20 bits)
  - bits 27..20: byte de dado
  - bit 28     : flag "ocupado" (palavra 0 = slot vazio)
- Probing linear, hash de Knuth
- Carga típica ~25%, sem rehash em runtime (capacidade fixa)

**Concorrência sem locks:**

- O caminho quente é uma única **store atômica de 32 bits** na slot encontrada
  via probing — o RP2350 garante atomicidade de loads/stores alinhados de
  32 bits entre core 0 e core 1
- Produtor único (consumer no core 1) faz updates
- Consumidor único (rcheevos no core 0) faz leituras
- Inserção de novos endereços (`bus_track_add`) só acontece em fase de
  game-load com o PIO pausado — não há concorrência insert × update

**Custo de performance:**

- Update do produtor: 1 probe médio (≈20 ciclos a 200 MHz = 100 ns)
- Maioria das escritas SNES é em endereços **não** rastreados → terminam no
  primeiro probe com hit em slot vazio (zero cost além do hash)
- Pico estimado de tráfego WRAM ≈ 1M escritas/s → ~10% de CPU no core 1
- Comparado a array flat (~3 ciclos por update), a penalidade é absorvida
  pelo orçamento de tempo do ciclo do barramento (279 ns em fast mode)

**O que ganhamos:**

- ~250 KB de SRAM liberados para rcheevos state, buffers serial, stacks
- Sem snapshot — rcheevos lê direto do hashmap; leituras byte-a-byte
  são atômicas, e o próprio rcheevos é tolerante a estados intermediários
  entre frames
- Independente de SRAM size do cartucho (não precisa parsing do header
  para alocar)

**Trade-off aceito:** endereços não pré-registrados via `bus_track_add()`
retornam 0. Isso é OK porque o rcheevos sempre registra os endereços que
vai consultar durante a inicialização do achievement set.

---

## Roadmap de desenvolvimento

- [x] **Fase 0** — Pinout fechado, estrutura de pastas
- [x] **Fase 1** — PIO0 + DMA ping-pong + consumer no core1, hashmap esparso concurrent-safe
- [x] **Fase 2** — Identificação de cartucho: drive bus, MD5 streaming, parse de header LoROM/HiROM
- [x] **Fase 3** — Protocolo serial ESP32 (compat NES) + rcheevos client + frame loop 60/50Hz
- [x] **Fase 3.5** — `snes-esp-firmware` portado (READ_CART/CART_INFO + renames)
- [x] **Fase 4a** — PIO0 SM1 + cart-write capture + `bus_to_sram_offset` (small SRAM: ≤32KB LoROM, ≤8KB HiROM)
- [ ] **Fase 4b** — Habilitar PIO1 para recuperar A17–A23 (carts com SRAM grande: Star Ocean, Tales of Phantasia, Dragon Quest VI)
- [x] **Fase 4.5** — Firmware no pinout v0.2 (`pio_set_gpio_base`, janela de
      overlap, captura 1-palavra-por-ciclo, DMA ENDLESS+ring com drenagem
      incremental, espelho integral 128K WRAM + 32K SRAM no lugar do hashmap)
- [x] **Fase 4c** — Tracker da porta WRAM `$2180–83` via /PAWR+PA7/PA1/PA0
      (PIO1 SM0), com ponteiro-sombra de `$2181–83` e auto-incremento
- [ ] **Fase 4d** — Detecção de vblank real via B-bus (escritas `$21xx` no
      NMI) p/ frame pacing e snapshots atômicos
- [ ] **Fase 5** — Bring-up em hardware real (validar timing, ajustar busy_wait, medir DMA overrun, scope no ponto de amostragem)
- [ ] **Fase 6** — ExHiROM e enhancement chips (SuperFX, SA-1, DSP)

## Bring-up / debug (sem ESP32)

O USB serial do WeAct aceita **o mesmo protocolo do link UART** (terminador
`\n` ou `\r`), então dá para exercitar tudo num terminal, sem ESP32 montado:

- `READ_CART` — dispara a identificação do cartucho (header + MD5 no console)
- `PEEK=<addr hex>` — lê um byte do espelho (endereço linear rcheevos:
  `0x000000+` = WRAM, `0x020000+` = SRAM). Ex.: `PEEK=7E` não — use o
  offset: vidas em `$7E:0DBE` → `PEEK=0DBE`
- `SYNC` / `RESET` — handshake / reboot
- Todo o tráfego Pico→ESP32 é espelhado no USB (`MIRROR_UART_TO_USB`)
- Marcadores `!w` `!c` `!b` = backlog >3/4 no ring de captura (quase-overrun)

**Banco de 4066 (protoboard e PCB v0.3)**: `BUS_SWITCH_4066` já é o
**default** no `main.c` — GP45 push-pull 3.3V, ativo-ALTO (4066 fecha com
HIGH), R1 pull-down na PCB para failsafe de power-on. Se um lote de 4066
não fechar com 3.3V (pior caso CD4066B: 3.5V): habilite
`BUS_SWITCH_4066_OPEN_DRAIN` **e** troque R1 por pull-up de 10k ao 5V
(obrigatório nessa variante — o GPIO só puxa LOW ou solta). Prefira
74HC4066 (Vih 3.15V @ 5V). Hardware antigo com '245: comente o define.

## Build

Pré-requisito: Pico SDK 2.x (com suporte a RP2350) em `PICO_SDK_PATH`.

```sh
cmake --preset pico
cmake --build --preset build_pico
```

Saída em `build_pico/src/snes-pico-firmware.uf2`.

## Licença

GPL-3.0 — mesma do projeto NES.
