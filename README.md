# f446-rtos-logger

A FreeRTOS (CMSIS-RTOS v2) sensor logger on a NUCLEO-F446RE, built in three short
days to practice the parts of firmware work that are hard to see in a demo:
task/queue design, a real I2C sensor driver, fault handling, and **measuring**
the result with a logic analyzer instead of trusting the code.

Every claim below links to a raw log or capture in `logs/`.

---

## Hardware and tools

| Item | Detail |
|---|---|
| Board | NUCLEO-F446RE, SYSCLK 84 MHz from **HSE bypass** (ST-LINK MCO, 8 MHz) |
| Sensor | VL53L0X ToF, 4-pin module (VIN/GND/SCL/SDA, **no XSHUT**), I2C1 PB8/PB9 @ 100 kHz |
| Log output | USART2 via ST-LINK VCP, 115200 8N1 |
| Timing markers | PA6 (D12) = sensor measurement window, PA7 (D11) = consumer UART write |
| Logic analyzer | 8-ch FX2 clone, PulseView + fx2lafw, 1 MHz sampling |
| Toolchain | STM32CubeIDE 2.2.0, STM32CubeMX 6.18.1, STM32Cube FW_F4 V1.28.3 |
| RTOS | FreeRTOS via CMSIS-RTOS v2 only (no native API mixed in), 1 kHz tick, heap_4 with 15 KB, HAL timebase on TIM6 |

The VL53L0X ST API (`Drivers/VL53L0X`) was copied from my thesis project; only the
platform delay was changed (`HAL_Delay(2)` busy-wait → `osDelay(2)`).

---

## Architecture

```
            200 ms (osDelayUntil)            queue (8 x 16 B)
 ┌──────────────────────────┐   put    ┌──────────────┐   get    ┌───────────────────────┐
 │ SensorTask   (Normal)    │ ───────▶ │  sensorQ     │ ───────▶ │ ConsumerTask (AboveN.) │──▶ UART
 │  INIT → RUN → ERROR      │          │ DROP_OLDEST  │          │  prints [DATA] + age   │
 │  VL53L0X single-shot     │          └──────────────┘          └───────────────────────┘
 └──────────────────────────┘                                               ▲
 HeartbeatTask (BelowNormal, 1 s): state, queue depth, drop/miss/err counters, stack high-water
 LedTask       (Low, 500 ms)                     UART access is serialized by a priority-inheritance mutex
```

| Task | Priority | Period | Stack | Min free (Day 2) |
|---|---|---|---|---|
| ConsumerTask | AboveNormal | queue-driven | 2048 B | 1408 B |
| SensorTask | Normal | 200 ms | 2048 B | 1132 B |
| HeartbeatTask | BelowNormal | 1 s | 2048 B | 1388 B |
| LedTask | Low | 500 ms | 512 B | 360 B |

Log lines:

```
[DATA] seq=51 t=11235 dist=117 mm st=0 sig=4546 kcps meas=37 ms age=37 ms
[HB]   tick=12156 vl=RUN q=0/8 drop=0 miss=0 err=3 last=-7 rec=1
[RST]  csr=0x04000000 PIN            <- reset cause from RCC->CSR, printed at boot
[CLK]  sysclk=84000000 Hz sws=PLL pll_in=HSE hse_on=1 hse_byp=1
```

`st != 0` means the ST range status is not valid (e.g. `st=2 dist=8191` = signal
fail / out of range); such distances must not be used.

---

## Day 1 — tasks, queue, and why both naive overflow policies fail

Mock sensor data, 4 tasks, 8-slot queue. 250+ samples with no loss and exactly
200 ms spacing (`logs/day1_step5.log`). Then the consumer was slowed to 1 sample/s
against 5 samples/s production:

| Put timeout | Result | Log |
|---|---|---|
| `0` (drop) | 4 drops/s, logged values ~8 s stale | `logs/day1_step6_drop.log` |
| `osWaitForever` (backpressure) | no loss, but the **sampling period collapses to 1 s** | `logs/day1_step6_block.log` |

Neither is acceptable for a logger. Revisited with real data on Day 2 (below).

---

## Day 2 — real sensor, fault handling, measurement

### 1. VL53L0X on I2C1

- Single-shot ranging every 200 ms, timing budget 33 ms. Sensor init runs inside the
  task because the ST API polls with `osDelay`.
- Result: 200 ms spacing, `drop=0`, `meas=37 ms` (`logs/day2/day2_step2.log`).

### 2. Fault handling

Sensor state machine: `INIT → RUN → ERROR → (bus recovery, backoff) → INIT`.

- `RUN → ERROR` after **3 consecutive** failures (single glitches are absorbed).
- Recovery: I2C bus recovery (9 SCL pulses + STOP to release a slave holding SDA,
  then RCC reset of I2C1 for the F4 stuck-BUSY erratum), then full sensor re-init.
  Retry interval 100 → 200 → … → 2000 ms.
- After recovery the period reference is reset, and missed slots are **skipped and
  counted (`miss`)** instead of being executed back-to-back (the Day 1
  `osDelayUntil` catch-up burst).

Two problems in the vendor code were found and bounded:

| ST API behaviour | Consequence | Fix |
|---|---|---|
| `VL53L0X_ResetDevice()` polls `MODEL_ID` in a `while` loop **without timeout** | task hangs forever if the sensor is absent | own `vl_soft_reset()` with bounded polling and `MODEL_ID == 0xEE` presence check |
| `PerformSingleRangingMeasurement()` waits up to 2000 × `PollingDelay` ≈ **4 s** | a sensor that reset silently (I2C still ACKs, data-ready never set) made the task look `RUN` while producing nothing for ~12 s | own `vl_measure()` with a **100 ms** completion timeout |

Fault tests (`logs/day2/`):

| Test | Detected as | Result |
|---|---|---|
| SDA wire pulled while running | `-20` (NACK) ×3 → ERROR in ~0.6 s | recovered after reconnect, no burst (`day2_step3_t1.log`) |
| SCL wire pulled | `-20` | same (`day2_step3_t2.log`) |
| VIN pulled | `-20` | recovered, downtime measured from first failure (`day2_step3_t3_2.log`) |
| Boot with sensor absent | `NO RESPONSE`, init retries | runs once plugged in (`day2_step3_t4_2.log`) |
| **Injected sensor-only reset** every 10 s (`FAULT_INJECT_EVERY`) | `-7` (timeout) ×3 | **13/13 recovered, downtime 819 ms every time** (`day2_step3_test_4.log`) |

Hardware findings (not fixable in firmware, recorded as constraints):

- **Hot-plugging the sensor resets the MCU (POR).** Reset cause read from `RCC->CSR`:
  whole-connector plug-in caused `POR PIN BOR` in 3 of 4 events; with single wires,
  VIN connected before SDA/SCL caused POR (1/1) while VIN connected last did not (0/1).
  Likely inrush into the module's discharged capacitors; no oscilloscope to confirm.
  A production design would need an inrush-limited load switch on the sensor rail.
- On NUCLEO-64 the 3V3 header pin sits between **NRST** and **5V**; handling that jumper
  produced one `PIN`-only reset. Connect/disconnect on the sensor side only.
- **Open issue (seen once):** after hot-plug activity the sensor stayed in a state where
  every soft reset + init timed out, surviving MCU resets; only a sensor power cycle
  cleared it. Not reproduced by 13 injected resets. With XSHUT wired this would be
  recoverable in software.

### 3. Logic analyzer measurements

Captures in `logs/day2/la/` (PulseView, 1 MHz; D0 SDA, D1 SCL, D2 PA6, D3 PA7).

| # | Quantity | Measured |
|---|---|---|
| M1 | Sensor period (HSE) | **200.020 ms**, p-p jitter ≈ **1 µs** (`hse_period.png`) |
| M2 | Measurement window (PA6 high) | 37.8 ms (matches `meas=37` in UART) |
| M3 | I2C inside one measurement | start burst + **15 data-ready polls at 2 ms** + result read (`m3_measure_zoom.png`) |
| M4 | One 1-byte register write @100 kHz | ≈ 250 µs; a 1-byte read ≈ 0.5 ms (`m4_i2c_write.png`) |
| — | I2C bus busy per sample | ≈ 13.5 ms of 200 ms (~7 %) |
| M5 | Sensor task done → consumer running | **≈ 22 µs** (queue put + context switch) (`m5_queue_latency.png`) |
| M6 | One UART log line | ≈ 5.7–6.0 ms |

M5 also shows PA6 falling ~9 µs **before** the I2C STOP on the bus: HAL returns as soon
as the STOP bit is set in the peripheral, not when the bus is idle.

#### Clock source: HSI → HSE

The first captures showed a period of **199.78 ms** on every channel. `.ioc` showed the
PLL fed from **HSI** (internal RC). After switching to HSE bypass (PLLM 16 → 8,
SYSCLK unchanged at 84 MHz):

| | HSI | HSE bypass |
|---|---|---|
| Period (analyzer) | 199.78 ms (−1100 ppm) | 200.020 ms |
| Period (PC reference, `tools/clock_check.py`, 180 s) | — | 200.007 ms (**−33 ppm**) |
| Timestamp drift | ~4 s / hour | ~0.12 s / hour |
| Period jitter p-p | ~70 µs | **~1 µs** |

MCU, PC, and analyzer agree within ~100 ppm after the change, so the analyzer itself is
trustworthy. The 70× jitter reduction shows the HSI's short-term instability, not the
scheduler, was limiting timing accuracy (`hsi_period.png`, `hsi_period_after_reboot.png`,
`logs/day2/clock_check_hse.txt`).

### 4. Queue overflow policy

Overload: consumer delayed 1000 ms (≈1 sample/s) vs. 5 samples/s production.
`age` = time from measurement start to dequeue.

| Run | Policy | Len | Consumer | age (steady state) | Drops at t≈57.5 s |
|---|---|---|---|---|---|
| Q0 | DROP_OLDEST | 8 | no delay | **37 ms** (= measurement time) | 0 |
| Q1 | DROP_NEWEST (Day 1) | 8 | 1 s | **7.89–8.08 s** | 222 |
| Q2 | DROP_OLDEST | 8 | 1 s | **1.44–1.63 s** | 222 |
| Q3 | DROP_OLDEST (mailbox) | 1 | 1 s | **40–234 ms** | 229 |

- Under sustained overload the **number** of lost samples is set by the rate mismatch,
  not the policy (Q1 = Q2; Q3 is higher by exactly 7 = the difference in queue capacity).
  The policy only decides **which** samples survive, i.e. how stale the output is.
- Ages follow `37 ms + (N−1)·200 ms + phase`, and the phase ramps 6 ms per line because the
  consumer uses `osDelay(1000)` plus a ~6 ms print, not `osDelayUntil`.
- Chosen default: **DROP_OLDEST, length 8** — no loss and 37 ms age in normal operation,
  bounded staleness (≤ ~1.6 s) and 1.4 s of burst buffering under overload.
  A control loop that only needs the current value should use the mailbox (Q3).

Logs: `logs/day2/day2_q0_normal.log` … `day2_q3_mailbox.log`.

---

## Build and flash

1. Open the project in STM32CubeIDE and build (`Debug`).
2. Flash either with Run/Debug, or by copying `Debug/f446-rtos-logger.bin` to the
   `NODE_F446RE` USB drive (post-build binary output is enabled).
3. Terminal at 115200 8N1 on the ST-LINK VCP.

Compile-time switches at the top of `Core/Src/app_rtos.c`:

| Switch | Default | Purpose |
|---|---|---|
| `QUEUE_POLICY` / `QUEUE_LEN` | `QP_DROP_OLDEST` / 8 | overflow policy experiment |
| `CONSUMER_DELAY_MS` | 0 | overload experiment |
| `FAULT_INJECT_EVERY` | 0 | inject a sensor soft reset every N samples |
| `TASK_MARKERS` | 1 | PA6/PA7 timing markers |

Clock check (close the serial terminal first):

```
pip install pyserial
python tools/clock_check.py COM3 180
```

---

## Known limitations / next steps

- The 15 data-ready polls take ~7.5 ms of the ~13.5 ms bus time per sample; starting to poll
  ~30 ms after the start command would cut this by ~6 ms.
- Recovery cannot power-cycle the sensor (no XSHUT on this module); see the open issue above.
- Logging goes over a blocking UART write (~6 ms/line) inside the consumer; DMA would free the CPU.
- Day 3: TBD.
