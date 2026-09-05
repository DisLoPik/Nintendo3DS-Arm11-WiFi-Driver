/*
 * 3DS Wi-Fi driver, ARM9 side.
 *
 * The ARM9 cannot reach the Wi-Fi SDIO controller in 3DS mode, so it triggers
 * the ARM11 probe (wifi_probe_run) through a cross-core signal and reads the
 * results back out of the shared WifiShared block. FCRAM is not cache-coherent
 * between the cores, so every access is bracketed by os_cache_sync().
 *
 * The two host hooks (os_cache_sync, wifi_host_post_probe) are described in
 * include/wifi_host.h.
 */
#include <stdint.h>
#include "wifi.h"
#include "wifi_host.h"

/* Ask the ARM11 to run the probe. */
void wifi_probe(void) {
  os_cache_sync();
  wifi_host_post_probe();
  os_cache_sync();
}

/* Copy the shared results out of FCRAM. sdmmcctl is an ARM9-side CFG9 register
 * (0x10000020) filled here for the diagnostic UI. */
void wifi_get(WifiShared *out) {
  os_cache_sync();
  volatile unsigned char *s = (volatile unsigned char *)WIFI_SHARED_ADDR;
  unsigned char *d = (unsigned char *)out;
  for (unsigned i = 0; i < sizeof(WifiShared); i++)
    d[i] = s[i];
  out->sdmmcctl = *(volatile uint16_t *)0x10000020u;
}
