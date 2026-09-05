/*
 * 3DS Wi-Fi SDIO/BMI driver, ARM11 side.
 *
 * Runs on the ARM11 (the only core with the MMIO grant for the Wi-Fi SDIO
 * controller). Powers the Atheros AR6014 chip, enumerates it over SDIO, reads
 * its registers (CMD52/CMD53), and attempts the first BMI command. All results
 * are published to the shared WifiShared block at WIFI_SHARED_ADDR for the ARM9
 * to read. Every loop is bounded and each stage sets `phase`, so a bad MMIO
 * access stalls a phase rather than hanging.
 *
 * The TMIO register layout and command encodings are transcribed from a 3DS SD
 * driver of the same controller family (GodMode9-derived); no NWM code is used.
 * See docs/wifi.md for the full bring-up state (SDIO works; BMI is the wall).
 *
 * Host integration: the host's ARM11 command dispatcher calls wifi_probe_run()
 * when the ARM9 posts the probe request (see include/wifi_host.h).
 */
#include <stdint.h>
#include "wifi.h"

/* Generic ARM11 MMIO + data-cache primitives. */
typedef volatile uint8_t  vu8;
typedef volatile uint16_t vu16;
typedef volatile uint32_t vu32;

#define MMIO16(a) (*(vu16 *)(a))
#define MMIO32(a) (*(vu32 *)(a))

static inline void dsb(void) {
  __asm__ volatile("mcr p15, 0, %0, c7, c10, 4" ::"r"(0) : "memory");
}
static inline void dcache_clean(void) {
  __asm__ volatile("mcr p15, 0, %0, c7, c10, 0" ::"r"(0) : "memory");
  dsb();
}

/* Wi-Fi SDIO controller register access (16-bit TMIO at WIFI_SDIO_BASE). */
#define WB16(off) MMIO16(WIFI_SDIO_BASE + (off))

#define WR_CMD     0x00
#define WR_PORTSEL 0x02
#define WR_ARG0    0x04
#define WR_ARG1    0x06
#define WR_STOP    0x08
#define WR_RESP0   0x0C
#define WR_RESP1   0x0E
#define WR_STAT0   0x1C
#define WR_STAT1   0x1E
#define WR_IRM0    0x20
#define WR_IRM1    0x22
#define WR_CLKCTL  0x24
#define WR_BLKLEN  0x26
#define WR_OPT     0x28
#define WR_DATACTL 0xD8
#define WR_RESET   0xE0
#define WR_DATACTL32  0x100
#define WR_BLKLEN32   0x104
#define WR_BLKCOUNT32 0x108

#define WSTAT0_CMDRESPEND 0x0001u
#define WSTAT1_CMDBUSY    0x4000u
#define WMASK_GW          0x807Fu /* TMIO_MASK_GW; ILL_FUNC is not fatal */
#define WMASK_ALL         0x837F031Du

/* Write divider, then divider|enable(bit8) (mirrors the SD driver's setckl). */
static void wifi_setckl(uint32_t data) {
  WB16(WR_CLKCTL) = (uint16_t)(data & 0xFF);
  WB16(WR_CLKCTL) = (uint16_t)((1u << 8) | (data & 0x2FF));
}

/* Bring the controller out of reset and configure it. Transcription of an SD
 * controller_init + set_target against the Wi-Fi base; the SD-slot-only "mount
 * fix" CFG poke is deliberately omitted. */
static void wifi_ctrl_init(void) {
  WB16(WR_DATACTL32) &= 0xF7FFu;
  WB16(WR_DATACTL32) &= 0xEFFFu;
  WB16(WR_DATACTL32) |= 0x402u;
  WB16(WR_DATACTL) = (uint16_t)((WB16(WR_DATACTL) & 0xFFDD) | 2);
  WB16(WR_DATACTL32) &= 0xFFFFu;
  WB16(WR_DATACTL) &= 0xFFDFu;
  WB16(WR_BLKLEN32) = 512;
  WB16(WR_BLKCOUNT32) = 1;
  WB16(WR_RESET) &= 0xFFFEu; /* assert reset  */
  WB16(WR_RESET) |= 1u;      /* release reset */
  WB16(WR_IRM0) |= (uint16_t)WMASK_ALL;
  WB16(WR_IRM1) |= (uint16_t)(WMASK_ALL >> 16);
  WB16(0xFC) |= 0xDBu;
  WB16(0xFE) |= 0xDBu;
  WB16(WR_PORTSEL) &= 0xFFFCu;
  WB16(WR_CLKCTL) = 0x20;
  WB16(WR_OPT) = 0x40E9;
  WB16(WR_PORTSEL) &= 0xFFFCu;
  WB16(WR_BLKLEN) = 512;
  WB16(WR_STOP) = 0;

  /* set_target: port 0, ~523 KHz ID clock, 1-bit bus. */
  WB16(WR_PORTSEL) &= 0xFFFCu;
  wifi_setckl(0x20);
  WB16(WR_OPT) |= 0x8000u; /* 1-bit bus */
}

/* Command word = index | response-type field (R4/R3 = 0x700, R6 = 0x400,
 * R1b = 0x500, R5 = 0x400). */
#define WCMD5  0x0705u /* IO_SEND_OP_COND, R4 */
#define WCMD3  0x0403u /* SEND_RELATIVE_ADDR, R6 */
#define WCMD7  0x0507u /* SELECT_CARD, R1b */
#define WCMD52 0x0434u /* IO_RW_DIRECT, R5 */

/* CMD52 argument: bit31 R/W, bits30-28 func, bit27 RAW (read-after-write),
 * bits25-9 addr, bits7-0 data. */
#define WCMD52_RD(func, addr) \
  (((uint32_t)(func) << 28) | (((uint32_t)(addr) & 0x1FFFFu) << 9))
#define WCMD52_WR(func, addr, data)                                   \
  ((1u << 31) | ((uint32_t)(func) << 28) |                            \
   (((uint32_t)(addr) & 0x1FFFFu) << 9) | ((uint32_t)(data) & 0xFFu))
#define WCMD52_WRR(func, addr, data) (WCMD52_WR(func, addr, data) | (1u << 27))

/* Send a no-data command and wait (bounded) for a response, into log entry `e`. */
static void wifi_cmd(WifiShared *w, WifiCmd *e, uint16_t cmd16, uint32_t arg) {
  uint32_t g = 0;
  while ((WB16(WR_STAT1) & WSTAT1_CMDBUSY) && ++g < 100000u)
    ;
  if (g >= 100000u)
    w->timeouts++;
  WB16(WR_IRM0) = 0;
  WB16(WR_IRM1) = 0;
  WB16(WR_STAT0) = 0;
  WB16(WR_STAT1) = 0;
  WB16(WR_DATACTL32) = (uint16_t)((WB16(WR_DATACTL32) & ~0x1800u) | 0x400u);
  WB16(WR_ARG0) = (uint16_t)(arg & 0xFFFF);
  WB16(WR_ARG1) = (uint16_t)(arg >> 16);
  WB16(WR_CMD) = cmd16;

  g = 0;
  for (;;) {
    uint16_t s1 = WB16(WR_STAT1);
    if (s1 & WMASK_GW)
      break; /* hard error (timeout/CRC/etc.) */
    if (!(s1 & WSTAT1_CMDBUSY) && (WB16(WR_STAT0) & WSTAT0_CMDRESPEND))
      break; /* response complete */
    if (++g >= 300000u) {
      w->timeouts++;
      break;
    }
  }
  uint16_t s0 = WB16(WR_STAT0);
  uint16_t s1 = WB16(WR_STAT1);
  e->cmd = cmd16;
  e->arg = arg;
  e->stat0 = s0;
  e->stat1 = s1;
  e->resp = (uint32_t)WB16(WR_RESP0) | ((uint32_t)WB16(WR_RESP1) << 16);
  e->ok = (s0 & WSTAT0_CMDRESPEND) ? 1u : 2u;
}

/* Append one command to the log (bounded) and return its entry. */
static WifiCmd *wifi_logcmd(WifiShared *w, uint16_t cmd16, uint32_t arg) {
  if (w->nlog >= WIFI_LOG_MAX)
    return &w->log[WIFI_LOG_MAX - 1];
  WifiCmd *e = (WifiCmd *)&w->log[w->nlog];
  wifi_cmd(w, e, cmd16, arg);
  w->nlog++;
  dcache_clean();
  return e;
}

/* Wi-Fi chip reset line: GPIO_DATA4_DATA_OUT_WIFI (ARM11 GPIO). GBATEK: bit0 =
 * "Wifi Enable (0=Reset, need re-upload wifi firmware, 1=On)", wired to the
 * chip's RESET/SYS_RST_L. Setting bit0 brings the Atheros chip out of reset so
 * it powers up and answers on the SDIO bus. In retail firmware this is brokered
 * by the MCU (the mcu::NWM service); bare-metal it is set directly. */
#define WIFI_GPIO_WIFI 0x10147028u

#define WR_FIFO 0x30 /* SD_FIFO (16-bit data port) */
#define WSTAT1_RXRDY 0x0100u
#define WSTAT1_TXRQ  0x0200u

/* CMD53 (IO_RW_EXTENDED) byte-mode transfer of `nwords` 32-bit words to/from
 * function `func` at `addr`. `incr`=1 increments the address, 0 = fixed.
 * Returns 0 on success, negative on error/timeout. */
static int wifi_cmd53(WifiShared *w, uint32_t func, uint32_t addr,
                      uint32_t *buf, int nwords, int is_write, int incr,
                      uint16_t *dbg) {
  uint32_t bytes = (uint32_t)nwords * 4u;
  uint16_t *h = (uint16_t *)buf; /* 16-bit FIFO access */
  int nhalf = (int)(bytes / 2u);
  uint32_t g = 0;
  while ((WB16(WR_STAT1) & WSTAT1_CMDBUSY) && ++g < 100000u)
    ;
  WB16(WR_IRM0) = 0;
  WB16(WR_IRM1) = 0;
  WB16(WR_STAT0) = 0;
  WB16(WR_STAT1) = 0;

  /* Use the 16-bit FIFO path (the controller signals via STAT1 RXRDY/TXRQ for
   * these small transfers, not the 32-bit DATACTL32 flags). */
  WB16(WR_DATACTL32) &= ~0x0002u; /* disable 32-bit FIFO */
  WB16(WR_STOP) = 0;
  WB16(0x0A) = 1;                    /* SDBLKCOUNT */
  WB16(WR_BLKLEN) = (uint16_t)bytes; /* SDBLKLEN */

  uint32_t arg = ((uint32_t)(is_write & 1) << 31) | ((func & 7u) << 28) |
                 ((uint32_t)(incr & 1) << 26) | ((addr & 0x1FFFFu) << 9) |
                 (bytes & 0x1FFu); /* byte mode (bit27=0), count in bytes */
  WB16(WR_ARG0) = (uint16_t)(arg & 0xFFFF);
  WB16(WR_ARG1) = (uint16_t)(arg >> 16);
  uint16_t cmd = 0x0035u | 0x0400u | 0x0800u; /* CMD53 | R5 | data present */
  if (!is_write)
    cmd |= 0x1000u; /* read direction */
  WB16(WR_CMD) = cmd;

  int idx = 0, done_data = 0;
  g = 0;
  for (;;) {
    uint16_t s1 = WB16(WR_STAT1);
    if (!is_write && (s1 & WSTAT1_RXRDY) && idx < nhalf) {
      WB16(WR_STAT1) = (uint16_t)(s1 & ~WSTAT1_RXRDY);
      for (int i = 0; i < nhalf; i++)
        h[idx++] = WB16(WR_FIFO);
      done_data = 1;
    }
    if (is_write && (s1 & WSTAT1_TXRQ) && idx < nhalf) {
      WB16(WR_STAT1) = (uint16_t)(s1 & ~WSTAT1_TXRQ);
      for (int i = 0; i < nhalf; i++)
        WB16(WR_FIFO) = h[idx++];
      done_data = 1;
    }
    uint16_t s0 = WB16(WR_STAT0);
    if (dbg) {
      dbg[0] = s0;
      dbg[1] = s1;
      dbg[2] = (uint16_t)idx;
      dbg[3] = (uint16_t)g;
    }
    if (s1 & WMASK_GW) {
      w->timeouts++;
      return -1;
    }
    if (!(s1 & WSTAT1_CMDBUSY) && done_data && (s0 & 0x0004u)) /* DATAEND */
      return 0;
    if (++g >= 800000u) {
      w->timeouts++;
      return -2;
    }
  }
}

/* Atheros HIF mailbox 0 (SDIO function-1 address, per ath6kl). Reads use the
 * base 0x800. WRITES must END at the mailbox boundary (0xFFF) for the target to
 * treat the packet as complete, so the write address is base + (WIDTH - len) =
 * 0x1000 - len (ath6kl's mailbox write-address adjustment). */
#define WIFI_MBOX0        0x800u
#define WIFI_MBOX_WIDTH   0x800u
#define WIFI_MBOX0_WR(len) (WIFI_MBOX0 + (WIFI_MBOX_WIDTH - (len)))

/* BMI_GET_TARGET_INFO (id 8): send the id, read back version + type. Needs no
 * firmware, so it tests the BMI channel. NOTE: does not yet get a response; the
 * target never sets mbox-data-pending. See docs/wifi.md for the open work. */
static void wifi_bmi_target_info(WifiShared *w) {
  uint16_t dbg[4] = {0, 0, 0, 0};

  /* Dump the 8 COUNT registers (0x420..0x43F) low bytes: counter 1 holds the
   * BMI command credit on this chip. */
  WifiCmd tc;
  for (int i = 0; i < 8; i++) {
    wifi_cmd(w, &tc, WCMD52, WCMD52_RD(1, 0x420 + i * 4));
    w->cnt[i] = (uint8_t)(tc.resp & 0xFFu);
  }

  /* Consume the credit by reading COUNT_DEC counter 1 (0x444, auto-decrements),
   * the flow-control step before writing a BMI command. */
  WifiCmd tcr;
  wifi_cmd(w, &tcr, WCMD52, WCMD52_RD(1, 0x444));
  w->bmi_credit = tcr.resp & 0xFFu;

  uint32_t cmd = 8; /* BMI_GET_TARGET_INFO */
  w->bmi_wr = wifi_cmd53(w, 1, WIFI_MBOX0_WR(4), &cmd, 1, 1 /*write*/, 1, 0);

  /* Poll HOST_INT_STATUS (0x400) low nibble = mailbox-0 data pending. */
  WifiCmd t;
  uint32_t polls, hs = 0;
  for (polls = 0; polls < 4000; polls++) {
    wifi_cmd(w, &t, WCMD52, WCMD52_RD(1, 0x400));
    hs = t.resp & 0xFFu;
    if (hs & 0x0Fu)
      break;
    for (volatile int d = 0; d < 3000; d++)
      ;
  }
  uint32_t r403, r405;
  wifi_cmd(w, &t, WCMD52, WCMD52_RD(1, 0x403));
  r403 = t.resp & 0xFFu;
  wifi_cmd(w, &t, WCMD52, WCMD52_RD(1, 0x405));
  r405 = t.resp & 0xFFu;
  w->bmi_look = hs | (r403 << 8) | (r405 << 16) | ((polls & 0xFF) << 24);

  uint32_t ver = 0, type = 0;
  w->bmi_rd = wifi_cmd53(w, 1, WIFI_MBOX0, &ver, 1, 0 /*read*/, 1, dbg);
  w->bmi_s0 = dbg[0];
  w->bmi_s1 = dbg[1];
  w->bmi_ctl = dbg[2];
  w->bmi_idx = dbg[3];
  w->bmi_ver = ver;
  wifi_cmd53(w, 1, WIFI_MBOX0, &type, 1, 0, 1, 0);
  w->bmi_type = type;
}

void wifi_probe_run(void) {
  WifiShared *w = (WifiShared *)WIFI_SHARED_ADDR;
  w->phase = WIFI_PH_NONE;
  w->timeouts = 0;
  w->nlog = 0;
  dcache_clean();

  /* Release the chip's hardware reset (GPIO_DATA4 bit0 = 1) and let it boot. */
  w->gpio_before = MMIO16(WIFI_GPIO_WIFI);
  MMIO16(WIFI_GPIO_WIFI) = (uint16_t)(w->gpio_before | 1u);
  dcache_clean();
  for (volatile int d = 0; d < 4000000; d++)
    ;
  w->gpio_after = MMIO16(WIFI_GPIO_WIFI);

  w->phase = WIFI_PH_REGS;
  dcache_clean();
  wifi_ctrl_init();

  w->phase = WIFI_PH_CLK;
  w->clk = WB16(WR_CLKCTL);
  dcache_clean();
  for (volatile int d = 0; d < 300000; d++) /* >=74 SD clocks before CMD5 */
    ;

  /* SDIO identification: CMD5 (OCR) -> CMD3 (get RCA) -> CMD7 (select). */
  w->phase = WIFI_PH_CMD;
  dcache_clean();

  WifiCmd *e = wifi_logcmd(w, WCMD5, 0);
  uint32_t ocr = e->resp & 0x00FFFFFFu;
  if (ocr == 0)
    ocr = 0x00FF8000u;

  for (int i = 0; i < 4; i++) { /* re-send with OCR until ready bit31 set */
    e = wifi_logcmd(w, WCMD5, ocr);
    if (e->resp & 0x80000000u)
      break;
    for (volatile int d = 0; d < 200000; d++)
      ;
  }

  e = wifi_logcmd(w, WCMD3, 0);
  uint32_t rca = e->resp & 0xFFFF0000u;
  wifi_logcmd(w, WCMD7, rca);

  /* CCCR reads (I/O-level access): 0x00 SDIO rev, 0x08 capability. */
  wifi_logcmd(w, WCMD52, WCMD52_RD(0, 0x00));
  wifi_logcmd(w, WCMD52, WCMD52_RD(0, 0x08));

  /* Walk the CIS (pointer in CCCR 0x09..0x0B) for the MANFID tuple = chip ID. */
  WifiCmd tmp;
  uint32_t cis = 0;
  wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(0, 0x09));
  cis |= (tmp.resp & 0xFFu);
  wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(0, 0x0A));
  cis |= (tmp.resp & 0xFFu) << 8;
  wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(0, 0x0B));
  cis |= (tmp.resp & 0xFFu) << 16;
  w->cis_addr = cis;
  for (int i = 0; i < 48; i++) {
    wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(0, cis + i));
    w->cis[i] = (uint8_t)(tmp.resp & 0xFFu);
  }

  /* Enable I/O function 1 (Wi-Fi): CCCR 0x02 IOE bit1, then poll CCCR 0x03 IOR
   * bit1 = function core ready. */
  WifiCmd *e2 = wifi_logcmd(w, WCMD52, WCMD52_RD(0, 0x02));
  uint8_t ioe = (uint8_t)(e2->resp & 0xFFu);
  wifi_logcmd(w, WCMD52, WCMD52_WRR(0, 0x02, ioe | 0x02u));
  for (int i = 0; i < 8; i++) {
    wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(0, 0x03));
    if (tmp.resp & 0x02u)
      break;
    for (volatile int d = 0; d < 200000; d++)
      ;
  }
  e2 = wifi_logcmd(w, WCMD52, WCMD52_RD(0, 0x03));
  w->ior = (uint8_t)(e2->resp & 0xFFu);

  /* Function-1 HIF register block 0x400..0x40F (HOST_INT_STATUS etc. per ath6kl). */
  for (int i = 0; i < 16; i++) {
    wifi_cmd(w, &tmp, WCMD52, WCMD52_RD(1, 0x400 + i));
    w->hif[i] = (uint8_t)(tmp.resp & 0xFFu);
  }

  wifi_bmi_target_info(w);

  for (int i = 0; i < 32; i++)
    w->reg[i] = WB16(i * 2);
  w->ext[WIFI_EXT_D8] = WB16(WR_DATACTL);
  w->ext[WIFI_EXT_E0] = WB16(WR_RESET);
  w->ext[WIFI_EXT_FC] = WB16(0xFC);
  w->ext[WIFI_EXT_FE] = WB16(0xFE);
  w->ext[WIFI_EXT_100] = WB16(WR_DATACTL32);
  w->ext[WIFI_EXT_102] = WB16(WR_DATACTL32 + 2);

  w->phase = WIFI_PH_DONE;
  w->seq++;
  dcache_clean();
}
