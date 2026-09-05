/*
 * 3DS Wi-Fi diagnostic screen (reference / host-coupled).
 *
 * This is the on-device test screen that triggers a probe and renders the
 * results table (command log, chip ID, HIF/counter/BMI diagnostics). It is the
 * screen used to reverse-engineer the bring-up. The formatting logic is
 * self-contained, but the drawing, input, screen, and audio-core-status calls
 * are provided by the host OS. In AuroraOS these came from screen.c / os_main.c /
 * audio9.c. Adapt the "host contract" externs below to your platform.
 */
#include <stdint.h>
#include "wifi.h"

typedef uint8_t u8;
typedef uint32_t u32;
typedef uint16_t u16;

/* ---- Host contract (adapt to your OS) ------------------------------------- */
typedef struct { unsigned char r, g, b; } Color;

extern volatile u8 *const VRAM_BOT_A; /* bottom-screen framebuffer */
extern const int BOT_SCREEN_HEIGHT;
extern const u32 BOT_FB_SIZE;
extern const Color COLOR_HM_BG, COLOR_HM_TEXT2, COLOR_AURORA, COLOR_ORANGE,
    COLOR_WHITE;
extern const u32 BUTTON_A, BUTTON_B;
extern const unsigned char icon_wifi_bits[];

void draw_string(volatile u8 *fb, int x, int y, int screen_h, const char *s,
                 Color fg, Color bg);
void clear_screen(volatile u8 *fb, u32 size, Color c);
void screen_present_bottom(void);
void settings_header(const unsigned char *icon, const char *title, Color c);
u32 get_keys_down(void);
int touch_tap(int *x, int *y);
void delay(volatile u32 cycles);
int audio_alive(void);       /* is the ARM11 core running? */
u32 audio_version(void);     /* resident ARM11 core version */
/* --------------------------------------------------------------------------- */

static char *snd_cpy(char *dst, const char *src) {
  while ((*dst = *src)) {
    dst++;
    src++;
  }
  return dst;
}

static void snd_u32(char *out, u32 v) {
  char tmp[12];
  int i = 0;
  if (v == 0)
    tmp[i++] = '0';
  while (v) {
    tmp[i++] = (char)('0' + v % 10);
    v /= 10;
  }
  int p = 0;
  while (i)
    out[p++] = tmp[--i];
  out[p] = '\0';
}

static void snd_hex(char *out, u32 v) {
  static const char d[] = "0123456789ABCDEF";
  out[0] = '0';
  out[1] = 'x';
  for (int i = 0; i < 8; i++)
    out[2 + i] = d[(v >> ((7 - i) * 4)) & 0xF];
  out[10] = '\0';
}

static void snd_hex2(char *out, u32 v) {
  static const char d[] = "0123456789ABCDEF";
  out[0] = d[(v >> 4) & 0xF];
  out[1] = d[v & 0xF];
  out[2] = '\0';
}

static void wifi_hex4(char *out, u32 v) {
  static const char d[] = "0123456789ABCDEF";
  for (int i = 0; i < 4; i++)
    out[i] = d[(v >> ((3 - i) * 4)) & 0xF];
  out[4] = '\0';
}

static void wifi_run_probe(WifiShared *w) {
  wifi_get(w);
  u32 last = w->seq;
  wifi_probe();
  for (int t = 0; t < 150; t++) { /* wait for the ARM11 to finish (~3s max) */
    delay(200000);
    wifi_get(w);
    if (w->seq != last)
      break;
  }
}

static void wifitest_draw(const WifiShared *w) {
  clear_screen(VRAM_BOT_A, BOT_FB_SIZE, COLOR_HM_BG);
  draw_string(VRAM_BOT_A, 8, 6, BOT_SCREEN_HEIGHT, "Wi-Fi Test", COLOR_HM_TEXT2,
              COLOR_HM_BG);

  char line[48], num[16], *p;
  int alive = audio_alive();

  p = snd_cpy(line, alive ? "core v" : "core? v");
  snd_u32(num, audio_version());
  p = snd_cpy(p, num);
  p = snd_cpy(p, " ph ");
  snd_u32(num, w->phase);
  p = snd_cpy(p, num);
  p = snd_cpy(p, " clk ");
  wifi_hex4(num, w->clk);
  p = snd_cpy(p, num);
  p = snd_cpy(p, " t/o ");
  snd_u32(num, w->timeouts);
  snd_cpy(p, num);
  Color pc = (w->phase == WIFI_PH_DONE) ? COLOR_AURORA : COLOR_ORANGE;
  draw_string(VRAM_BOT_A, 8, 22, BOT_SCREEN_HEIGHT, line, pc, COLOR_HM_BG);

  draw_string(VRAM_BOT_A, 8, 38, BOT_SCREEN_HEIGHT,
              "idx  arg      resp      s0/s1", COLOR_HM_TEXT2, COLOR_HM_BG);

  /* One row per logged SDIO command; green when the response is non-zero. */
  u32 n = w->nlog;
  if (n > WIFI_LOG_MAX)
    n = WIFI_LOG_MAX;
  for (u32 i = 0; i < n; i++) {
    const WifiCmd *e = &w->log[i];
    p = snd_cpy(line, "C");
    snd_u32(num, (u32)(e->cmd & 0x3F));
    if ((e->cmd & 0x3F) < 10) {
      *p++ = '0';
      *p = '\0';
    }
    p = snd_cpy(p, num);
    *p++ = ' ';
    wifi_hex4(num, (u16)(e->arg >> 16));
    p = snd_cpy(p, num);
    wifi_hex4(num, (u16)e->arg);
    p = snd_cpy(p, num);
    *p++ = ' ';
    snd_hex(num, e->resp);
    p = snd_cpy(p, num);
    *p++ = ' ';
    wifi_hex4(num, e->stat0);
    p = snd_cpy(p, num);
    *p++ = '/';
    wifi_hex4(num, e->stat1);
    snd_cpy(p, num);
    int good = (e->ok == 1) && (e->resp != 0) && (e->resp != 0xFFFFFFFF);
    draw_string(VRAM_BOT_A, 8, 52 + (int)i * 13, BOT_SCREEN_HEIGHT, line,
                good ? COLOR_AURORA : (e->ok == 2 ? COLOR_ORANGE : COLOR_HM_TEXT2),
                COLOR_HM_BG);
  }

  /* Parse the CIS for the CISTPL_MANFID (0x20) tuple: manufacturer + card ID.
   * Atheros vendor = 0x0271. */
  {
    u32 manf = 0, card = 0;
    int found = 0, pos = 0;
    for (int g = 0; g < 24 && pos + 1 < 48; g++) {
      u8 code = w->cis[pos];
      if (code == 0xFF)
        break; /* end of CIS */
      if (code == 0x00) {
        pos++;
        continue;
      } /* null tuple */
      u8 link = w->cis[pos + 1];
      if (code == 0x20 && pos + 5 < 48) { /* CISTPL_MANFID */
        manf = w->cis[pos + 2] | ((u32)w->cis[pos + 3] << 8);
        card = w->cis[pos + 4] | ((u32)w->cis[pos + 5] << 8);
        found = 1;
        break;
      }
      pos += 2 + link;
    }
    if (found && manf == 0x0271)
      p = snd_cpy(line, "ATHEROS ");
    else
      p = snd_cpy(line, "id ");
    wifi_hex4(num, manf);
    p = snd_cpy(p, num);
    *p++ = ':';
    wifi_hex4(num, card);
    p = snd_cpy(p, num);
    p = snd_cpy(p, "  fn1 ");
    p = snd_cpy(p, (w->ior & 0x02) ? "RDY" : "no");
    Color mc = (found && manf == 0x0271) ? COLOR_AURORA : COLOR_WHITE;
    draw_string(VRAM_BOT_A, 8, BOT_SCREEN_HEIGHT - 58, BOT_SCREEN_HEIGHT, line,
                mc, COLOR_HM_BG);
  }

  /* COUNT registers 0x420..0x43F (credit counters, low byte each). */
  p = snd_cpy(line, "CNT ");
  for (int i = 0; i < 8; i++) {
    snd_hex2(num, w->cnt[i]);
    p = snd_cpy(p, num);
    *p++ = ' ';
  }
  *p = '\0';
  {
    int any = 0;
    for (int i = 0; i < 8; i++)
      if (w->cnt[i])
        any = 1;
    draw_string(VRAM_BOT_A, 8, BOT_SCREEN_HEIGHT - 44, BOT_SCREEN_HEIGHT, line,
                any ? COLOR_AURORA : COLOR_ORANGE, COLOR_HM_BG);
  }
  /* HOST_INT_STATUS(0x400) + consumed credit + BMI target version. */
  p = snd_cpy(line, "400 ");
  snd_hex2(num, w->bmi_look & 0xFF);
  p = snd_cpy(p, num);
  p = snd_cpy(p, " cr");
  snd_hex2(num, w->bmi_credit & 0xFF);
  p = snd_cpy(p, num);
  p = snd_cpy(p, " rs0 ");
  wifi_hex4(num, w->bmi_s0);
  p = snd_cpy(p, num);
  p = snd_cpy(p, " ver ");
  snd_hex(num, w->bmi_ver);
  snd_cpy(p, num);
  {
    int bmi_ok = w->bmi_ver != 0 && w->bmi_ver != 0xFFFFFFFF;
    draw_string(VRAM_BOT_A, 8, BOT_SCREEN_HEIGHT - 30, BOT_SCREEN_HEIGHT, line,
                bmi_ok ? COLOR_AURORA : COLOR_HM_TEXT2, COLOR_HM_BG);
  }

  draw_string(VRAM_BOT_A, 8, BOT_SCREEN_HEIGHT - 15, BOT_SCREEN_HEIGHT,
              "A / tap: Re-probe    B: Back", COLOR_HM_TEXT2, COLOR_HM_BG);
  screen_present_bottom();
}

/* Entry point: open the Wi-Fi test screen, probe, and loop on re-probe / back. */
void wifi_test_screen(void) {
  settings_header(icon_wifi_bits, "Wi-Fi Test", COLOR_WHITE);
  static WifiShared w;
  wifi_run_probe(&w);
  wifitest_draw(&w);
  while (1) {
    u32 k = get_keys_down();
    int tx, ty;
    if ((k & BUTTON_A) || touch_tap(&tx, &ty)) {
      wifi_run_probe(&w);
      wifitest_draw(&w);
    }
    if (k & BUTTON_B)
      return;
    delay(60000);
  }
}
