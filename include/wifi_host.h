/*
 * Host contract: the small set of primitives the Wi-Fi driver needs from the
 * surrounding OS. The driver itself (src/wifi_arm11.c) is self-contained; this
 * header covers the ARM9-side glue in src/wifi_arm9.c.
 *
 * In AuroraOS (the project this was extracted from) these were:
 *   - os_cache_sync():        ARM9 clean+invalidate of the data cache, used both
 *                             to push writes to physical RAM for the ARM11 and to
 *                             re-read the ARM11's replies (FCRAM is not coherent).
 *   - wifi_host_post_probe(): signal the ARM11 to run wifi_probe_run(). AuroraOS
 *                             piggybacked its audio-core mailbox (a shared command
 *                             block the ARM11 polls); any cross-core mechanism
 *                             works. The ARM11 dispatcher must call
 *                             wifi_probe_run() when it receives the signal.
 */
#ifndef WIFI_HOST_H
#define WIFI_HOST_H

void os_cache_sync(void);
void wifi_host_post_probe(void);

#endif /* WIFI_HOST_H */
