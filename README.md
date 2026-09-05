# **Nintendo3DS** Arm11 WiFi Driver
## All code and documentation for a fully working bare metal Arm11 driver for using the 3DS Atheros AR6014 chip.

A from-scratch, bare-metal driver for the Nintendo 3DS Wi-Fi chip (Atheros
AR6014), extracted from the AuroraOS project. The chip is driven directly from
the ARM11 over its SDIO/TMIO host controller, with no reliance on Nintendo's NWM
sysmodule at runtime.

## Status

The SDIO hardware stack is functional and verified on real hardware (New 3DS):

- Chip power-on (release the reset line at GPIO `0x10147028` bit 0).
- SDIO enumeration (CMD5 / CMD3 / CMD7).
- Chip identification: Atheros AR6014 (SDIO vendor `0x0271`, device `0x0201`),
  read from the card's CIS.
- Register I/O (CMD52 / CMD53) and enabling the Wi-Fi I/O function.

Not yet implemented: the BMI firmware upload and the HTC / WMI / 802.11 / WPA2 /
DHCP / TCP/IP layers. The current blocker is the first BMI handshake. The SDIO
transfers succeed, but the chip's bootloader does not respond, because its exact
control-register map differs from the open-source ath6kl reference and must be
taken from the NWM sysmodule. A working `ping` remains a large, multi-stage
project. See `docs/wifi.md` for the full technical state and `docs/wifi-summary.md`
for a short overview.

## Layout

```
include/
  wifi.h        Shared interface: WifiShared block, command log, entry points.
  wifi_host.h   The two host primitives the ARM9 glue needs.
src/
  wifi_arm11.c  The driver. Runs on the ARM11: power, enumeration, CMD52/53, BMI.
  wifi_arm9.c   ARM9-side trigger (wifi_probe) and result read-back (wifi_get).
  wifi_ui.c     On-device diagnostic screen (host-coupled reference).
tools/
  nwm_extract.py  Extracts the Atheros firmware blocks from a dumped NWM .code.
docs/
  wifi.md          Full status, architecture, register reference, path forward.
  wifi-summary.md  Short summary suitable for a forum/Discord post.
```

`src/wifi_arm11.c` is the core and is self-contained (it includes its own MMIO and
cache primitives). It publishes all results to the `WifiShared` block at
`WIFI_SHARED_ADDR` (`0x233B0000`).

## Integrating into a host

The driver runs on the ARM11 and reports to the ARM9 through a shared,
non-cache-coherent FCRAM block. A host OS must provide:

- A way to run `wifi_probe_run()` on the ARM11 when the ARM9 asks for it. The ARM9
  side calls `wifi_host_post_probe()` (you implement the cross-core signal) and the
  ARM11 dispatcher calls `wifi_probe_run()` on receipt.
- `os_cache_sync()`: an ARM9 data-cache clean+invalidate, so writes reach RAM and
  replies are re-read (see `include/wifi_host.h`).
- For `wifi_ui.c` only: drawing, input, and screen primitives (declared as externs
  at the top of that file). The UI is optional reference code; the driver does not
  depend on it.

Fixed addresses used: Wi-Fi SDIO controller at logical `0x10122000`, chip reset
GPIO at `0x10147028`, shared block at `0x233B0000`. Adjust for your memory map.

## Firmware

The Atheros firmware is Nintendo's copyright and is **not** included. Bringing up
the chip past enumeration will require it: dump the NWM sysmodule (title
`0004013000002d02`) from your own console, then run:

```
python tools/nwm_extract.py <nwm.dec.code> fw/
```

Any driver that uploads firmware must load it from removable storage at runtime;
it must never be embedded in a binary or committed to this repository.

## Credits

Reverse-engineered from the retail NWM sysmodule (using rizin) with reference to
GBATEK, 3dbrew, and the Linux ath6kl driver (a functional reference for the
protocol layers; not register-exact for the AR6014). The TMIO/SDHC register logic
is transcribed from a GodMode9-derived 3DS SD driver.
