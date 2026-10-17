# GivEnergy Gen1 BMS emulator

ESP32 firmware (LilyGo T-CAN485) that lets a GivEnergy Gen1 hybrid inverter run a
non-GivEnergy LV battery. It answers the inverter's BMS polls on RS485 (9600 8N1,
address 0x01) and fills them from the battery's live Pylontech-LV CAN data
(DATOUBOSS LFP48-314 / JBD BMS, 500 kbps).

## Build

```
cp include/secrets.h.example include/secrets.h   # then add your WiFi details
pio run                                          # build
pio run -t upload                                # flash over USB
pio device monitor                               # serial log with timestamps
```

`platformio.ini` is a minimal one for a fresh clone; keep your own if you have it.

## Diagnostics

`http://<board-ip>/health` returns JSON. The fields to check when charging stops:

| Field | Meaning |
|---|---|
| `hr20`, `hr25` | What the inverter is being told now (alarm bits, current limit in 0.01 A) |
| `can_failsafe_trips` / `can_failsafe_full_trips` | CAN went quiet: 5 s (HR25=0) / 20 s (also full+empty) |
| `can_longest_gap_s` | Longest silence between battery CAN frames since boot |
| `can_bus_off` | CAN controller bus-off events (each now recovered automatically) |
| `block_bms_protection` | Times the BMS raised a protection flag |
| `block_threshold` | Times our own OV/UV/OC/charge-UT thresholds tripped |
| `bms_warnings` | Times the BMS raised a warning (logged only, does not stop the inverter) |
| `bms_limit_zero` | Times the BMS itself sent a 0 A current limit |
| `rs485_bad_length` | RS485 requests that piled up because the loop was blocked |

The serial log prints a timestamped line (`[uptime s] ...`) every time HR20, HR25 or
the BMS flags change, with the reason.

## Known Gen1 behaviour

If the inverter is told to stop at night, it drops into standby (IR0 = 0) and does
not resume by itself once the condition clears; it needs an inverter restart.
The firmware avoids sending that signal on transient glitches, but keep a Home
Assistant watchdog (cheap rate on, SoC below target, battery power 0 W for 10 min →
alert or reboot inverter) as a backstop.
