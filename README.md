# f446-rtos-logger

A FreeRTOS (CMSIS-RTOS v2) sensor logger on a NUCLEO-F446RE with a VL53L0X ToF sensor,
built in three short days to practice the parts of firmware work that a demo does not show:
task and queue design, a real I2C driver, fault handling, and **measuring the result with a
logic analyzer instead of trusting the code**.

Every number below links to the raw UART log or analyzer capture it came from.

---

## Results at a glance

| What was checked | Result | Evidence |
|---|---|---|
| Sampling period (HSE clock) | **200.020 ms**, period jitter ≈ **1 µs** p-p; −33 ppm vs. PC clock | [`hse_period.png`](logs/day2/la/hse_period.png), [`clock_check_hse.txt`](logs/day2/clock_check_hse.txt) |
| Clock source fix | HSI → HSE cut period error from −1100 ppm to −33 ppm and jitter 70× (70 µs → 1 µs) | [`hsi_period.png`](logs/day2/la/hsi_period.png), [`day2_clk_hse_boot.log`](logs/day2/day2_clk_hse_boot.log) |
| Producer → consumer latency | **22 µs** from queue put to consumer running | [`m5_queue_latency.png`](logs/day2/la/m5_queue_latency.png) |
| Fault recovery | Wire pulls (SDA/SCL/VIN) all recovered; injected sensor resets **13/13 recovered, 819 ms downtime each** | [`day2_step3_test_4.log`](logs/day2/day2_step3_test_4.log) |
| Vendor driver hazards | 2 found and bounded: an unbounded `while` (hang) and a 4 s blocking wait | [Day 2 §2](#2-fault-handling) |
| Queue overflow policy | Under overload, **how many** samples are lost is set by the rate mismatch, not the policy. The policy only decides **which** survive: output staleness 8.0 s (drop-newest) vs. 1.5 s (drop-oldest) vs. ≤ 0.23 s (mailbox) | [Day 2 §4](#4-queue-overflow-policy) |
| I2C bus optimization | Delaying the first data-ready poll: polls 15 → 2, **bus busy 11.5 → 6.3 ms per sample (−45 %)**, measurement time unchanged within the 2 ms poll grid | [Day 3](#day-3--a-small-change-proven-by-measurement) |
| Stack headroom | Every task keeps ≥ 360 B free; three of four keep more than half | [`day3_final_boot.log`](logs/day3/day3_final_boot.log) |

Open issues are listed, not hidden: see [Known limitations](#known-limitations-and-open-issues).

---

## Hardware and tools

| Item | Detail |
|---|---|
| Board | NUCLEO-F446RE, SYSCLK 84 MHz from **HSE bypass** (ST-LINK MCO 8 MHz; PLLM 8, N 336, P 4) |
| Sensor | VL53L0X ToF, 4-pin module (VIN/GND/SCL/SDA, **no XSHUT, no GPIO1**), I2C1 PB8/PB9 @ 100 kHz, addr 0x29 |
| Log output | USART2 via ST-LINK VCP, 115200 8N1 |
| Timing markers | PA6 (D12) = sensor measurement window, PA7 (D11) = consumer UART write |
| Logic analyzer | 8-ch FX2 clone, PulseView + fx2lafw, 1 MHz sampling (D0 SDA, D1 SCL, D2 PA6, D3 PA7). No oscilloscope. |
| Toolchain | STM32CubeIDE 2.2.0, STM32CubeMX 6.18.1, STM32Cube FW_F4 V1.28.3 |
| RTOS | FreeRTOS via **CMSIS-RTOS v2 only** (no native API mixed in), 1 kHz tick, heap_4 15 KB, HAL timebase on TIM6 |

The VL53L0X ST API (`Drivers/VL53L0X/`) was reused from my thesis project; only the platform
delay was changed (`HAL_Delay(2)` busy-wait → `osDelay(2)`, so polling yields the CPU).

## Repository layout

```
Core/Src/app_rtos.c      the whole application: tasks, queue, sensor state machine, bus recovery
Core/Inc/app_rtos.h      App_RTOS_Init(), called once from main.c (RTOS_THREADS section)
Drivers/VL53L0X/         ST VL53L0X API (unmodified except platform delay)
logs/day1_*.log          Day 1 UART logs (mock data, queue experiments)
logs/day2/               Day 2 UART logs; logs/day2/la/ = analyzer captures
logs/day3/               Day 3 UART logs; logs/day3/la/ = analyzer captures
tools/clock_check.py     measures the sample period against the PC clock over the VCP
```

`app_rtos.c` is commented as a study reference: each RTOS call has a short note on *why* it was
used and what the alternative would have been.

---

## Architecture

```
            200 ms (osDelayUntil)             queue (8 x 16 B)
 ┌──────────────────────────┐   put (t/o 0)  ┌─────────────┐  get (wait)  ┌────────────────────────┐
 │ SensorTask    (Normal)   │ ─────────────▶ │  sensorQ    │ ───────────▶ │ ConsumerTask (AboveN.) │──▶ UART
 │  INIT → RUN → ERROR      │                │ DROP_OLDEST │              │  prints [DATA] + age   │
 │  VL53L0X single-shot     │                └─────────────┘              └────────────────────────┘
 └──────────────────────────┘
 HeartbeatTask (BelowNormal, 1 s): state, queue depth, drop/miss/err counters, stack high-water mark
 LedTask       (Low, 500 ms)       UART is shared by 3 tasks → serialized by a priority-inheritance mutex
```

| Task | Priority | Period | Stack | Min free |
|---|---|---|---|---|
| ConsumerTask | AboveNormal | queue-driven | 2048 B | 1408 B |
| SensorTask | Normal | 200 ms | 2048 B | 1132 B |
| HeartbeatTask | BelowNormal | 1 s | 2048 B | 1416 B |
| LedTask | Low | 500 ms | 512 B | 384 B (lowest seen: 360 B) |

Min free from [`day3_final_boot.log`](logs/day3/day3_final_boot.log); the 360 B LedTask low was
seen in [`day2_q3_mailbox.log`](logs/day2/day2_q3_mailbox.log).

Boot and runtime log lines (from [`day3_final_boot.log`](logs/day3/day3_final_boot.log)):

```
[RST] csr=0x04000000 PIN                                       reset cause read from RCC->CSR
[CLK] sysclk=84000000 Hz sws=PLL pll_in=HSE hse_on=1 hse_byp=1 clock source read from registers
[CFG] queue=DROP_OLDEST len=8 cons_delay=0 ms first_poll=30 ms fault_inject=0
[I2C] VL53L0X @0x29: ACK
[VL] state INIT -> RUN (tick=208)
[DATA] seq=0 t=212 dist=170 mm st=0 sig=578 kcps meas=38 ms age=38 ms
[HB] tick=1083 vl=RUN q=0/8 drop=0 miss=0 err=0 last=0 rec=0
[STK] min free bytes: LED=384 HB=1416 Sens=1132 Cons=1408
```

`age` = time from measurement start to dequeue (how stale the logged value is). `st != 0` means
the ST range status is invalid (e.g. `st=2 dist=8191` = signal fail); such distances must not be used.
`err` counts every failed measurement or init attempt (I2C NACK `-20` **and** timeout `-7`).

### Design decisions

| Decision | Alternative considered | Why this one | Evidence |
|---|---|---|---|
| `osDelayUntil` on a fixed 200 ms grid | `osDelay(200)` | `osDelay` is relative, so the ~38 ms measurement and any preemption add to every period (≈ 238 ms). `osDelayUntil` keeps the grid. | M1: 200.020 ms, 1 µs jitter |
| Skip and count missed slots (`miss`) | Plain `osDelayUntil` after a delay | With a past deadline the CMSIS wrapper returns immediately, so late periods execute back-to-back (catch-up burst) | Day 2 §2 |
| Single-shot ranging | Continuous mode | Continuous ranging runs on the sensor's own timer and drifts against the MCU grid; single-shot pins each sample to the period | `t=` steps of exactly 200 ms |
| Queue put timeout `0` + **DROP_OLDEST** | Drop-newest; blocking put | Drop-newest leaves 8 s stale data; blocking keeps every sample but drags the producer to the consumer rate (period 200 ms → 1 s) | Day 1, Day 2 §4 |
| Consumer priority > Sensor | Equal or lower | The consumer preempts as soon as a sample is queued, so the queue stays at 0–1 and age = measurement time | M5: 22 µs |
| Priority-inheritance mutex for UART | Binary semaphore; no lock | Three tasks print. A semaphore has no owner, so no inheritance; a low-priority holder could be preempted by a mid-priority task while the consumer waits | — |
| `RUN → ERROR` after **3** consecutive failures | React to first failure | Single I2C glitches are absorbed; a dead sensor is still detected in ~0.6 s | `day2_step3_t1.log` |
| Exponential backoff 100 → 2000 ms | Fixed retry interval | Fast recovery for brief faults, little bus/log noise when the sensor is gone for long | `day2_step3_t4_2.log` |
| Sensor init inside the task | Init in `main()` before the scheduler | The ST API polls with `osDelay`, which needs a running scheduler | — |
| Own `vl_soft_reset()` / `vl_measure()` | ST `ResetDevice()` / `PerformSingleRangingMeasurement()` | Bounded waits; see the vendor table in Day 2 §2 | `day2_step3_test_4.log` |

---

## Day 1 — tasks, queue, and why both naive overflow policies fail

Mock sensor data, 4 tasks, 8-slot queue. 250+ samples with no loss and exactly 200 ms spacing
([`day1_step5.log`](logs/day1_step5.log)). Then the consumer was slowed to 1 sample/s against
5 samples/s production:

| Put timeout | Result | Log |
|---|---|---|
| `0` (drop newest) | 4 drops/s, logged values ~8 s stale | [`day1_step6_drop.log`](logs/day1_step6_drop.log) |
| `osWaitForever` (backpressure) | no loss, but the **sampling period collapses to 1 s** | [`day1_step6_block.log`](logs/day1_step6_block.log) |

Neither is acceptable for a logger. Revisited with real data on Day 2 (§4).

---

## Day 2 — real sensor, fault handling, measurement

### 1. VL53L0X on I2C1

- Single-shot ranging every 200 ms, timing budget 33 ms.
- Result: 200 ms spacing, `drop=0`, `meas=37 ms` ([`day2_step2.log`](logs/day2/day2_step2.log)).

### 2. Fault handling

Sensor state machine: `INIT → RUN → ERROR → (backoff, bus recovery) → INIT`.

- Bus recovery: up to 9 SCL pulses + STOP to release a slave holding SDA, then an RCC reset of
  I2C1 because the F4 stuck-BUSY erratum survives `DeInit`/`Init`. Then a full sensor re-init.
- After recovery the period reference is reset, so there is no catch-up burst.
- Downtime is measured from the **first** failure after the last good sample (counting from the
  third failure under-reported it by the ~0.4 s detection time).

Two hazards in the vendor code were found and bounded:

| ST API behaviour | Consequence | Fix |
|---|---|---|
| `VL53L0X_ResetDevice()` polls `MODEL_ID` in a `while` loop **without timeout** | task hangs forever if the sensor is absent | `vl_soft_reset()`: bounded polling, presence check `MODEL_ID == 0xEE` |
| `PerformSingleRangingMeasurement()` waits up to 2000 × `PollingDelay` ≈ **4 s** | a sensor that reset silently (I2C still ACKs, data-ready never set) made the task look `RUN` while producing nothing for ~12 s | `vl_measure()`: **100 ms** completion timeout → detected as `-7` |

Fault tests:

| Test | Detected as | Result | Log |
|---|---|---|---|
| SDA wire pulled while running | `-20` (NACK) ×3 → ERROR in ~0.6 s | recovered after reconnect, no burst | [`day2_step3_t1.log`](logs/day2/day2_step3_t1.log) |
| SCL wire pulled | `-20` | same | [`day2_step3_t2.log`](logs/day2/day2_step3_t2.log) |
| VIN pulled | `-20` | recovered, downtime from first failure | [`day2_step3_t3_2.log`](logs/day2/day2_step3_t3_2.log) |
| Boot with sensor absent | `NO RESPONSE`, init retries with backoff | runs once plugged in | [`day2_step3_t4_2.log`](logs/day2/day2_step3_t4_2.log) |
| **Injected sensor-only reset** every 10 s (`FAULT_INJECT_EVERY=50`) | `-7` (timeout) ×3 | **13/13 recovered, downtime 819 ms every time** | [`day2_step3_test_4.log`](logs/day2/day2_step3_test_4.log) |

Why inject faults in software: a real power glitch that resets only the sensor is hard to produce
by hand (hot-plugging also resets the MCU, see below). Writing the sensor's soft-reset register
reproduces the same end state, I2C alive but ranging configuration lost, on demand and repeatably.

Hardware findings (constraints, not fixable in firmware):

- **Hot-plugging the sensor resets the MCU.** Reset cause from `RCC->CSR`: plugging the whole
  connector caused `POR PIN BOR` in 3 of 4 events; with single wires, VIN before SDA/SCL caused POR
  (1/1), VIN last did not (0/1). Likely inrush into the module's discharged capacitors (no scope to
  confirm). A product would need an inrush-limited load switch on the sensor rail.
- On NUCLEO-64 the 3V3 header pin sits between **NRST** and **5V**; handling that jumper produced
  one `PIN`-only reset. Connect and disconnect on the sensor side only.

### 3. Logic analyzer measurements

Captures in [`logs/day2/la/`](logs/day2/la/) (PulseView, 1 MHz).

| # | Quantity | Measured |
|---|---|---|
| M1 | Sensor period (HSE) | **200.020 ms**, p-p jitter ≈ **1 µs** ([`hse_period.png`](logs/day2/la/hse_period.png)) |
| M2 | Measurement window (PA6 high) | 37.8 ms (matches `meas=37` in UART) |
| M3 | I2C inside one measurement | start burst + **15 data-ready polls at 2 ms** + result read ([`m3_measure_zoom.png`](logs/day2/la/m3_measure_zoom.png)) |
| M4 | One 1-byte register write @100 kHz | ≈ 250 µs; a 1-byte read ≈ 0.5 ms ([`m4_i2c_write.png`](logs/day2/la/m4_i2c_write.png)) |
| M5 | Sensor task done → consumer running | **≈ 22 µs** (queue put + context switch) ([`m5_queue_latency.png`](logs/day2/la/m5_queue_latency.png)) |
| M6 | One UART log line | ≈ 5.7–6.0 ms (blocking transmit) |

M5 also shows PA6 falling ~9 µs **before** the I2C STOP appears on the bus. Lesson: a HAL call
returning is not the same as the bus being idle; HAL returns once the STOP bit is set in the
peripheral.

#### Clock source: HSI → HSE

The first captures showed **199.78 ms** on every channel. `.ioc` showed the PLL fed from **HSI**
(internal RC). After switching to HSE bypass (PLLM 16 → 8, SYSCLK unchanged at 84 MHz), the
register read-back at boot confirmed the change ([`day2_clk_hse_boot.log`](logs/day2/day2_clk_hse_boot.log)):

| | HSI | HSE bypass |
|---|---|---|
| Period (analyzer) | 199.78 ms (−1100 ppm) | 200.020 ms |
| Period (PC reference, `tools/clock_check.py`, 180 s) | — | 200.007 ms (**−33 ppm**) |
| Timestamp drift | ~4 s / hour | ~0.12 s / hour |
| Period jitter p-p | ~70 µs | **~1 µs** |

MCU, PC, and analyzer agree within ~100 ppm after the change, so the analyzer itself can be trusted.
The 70× jitter reduction shows the HSI's short-term instability, not the scheduler, was limiting
timing accuracy ([`hsi_period.png`](logs/day2/la/hsi_period.png),
[`hsi_period_after_reboot.png`](logs/day2/la/hsi_period_after_reboot.png),
[`clock_check_hse.txt`](logs/day2/clock_check_hse.txt)).

### 4. Queue overflow policy

Overload: consumer delayed 1000 ms (≈ 1 sample/s) vs. 5 samples/s production.

| Run | Policy | Len | Consumer | age (steady state) | Drops at t ≈ 57.5 s | Log |
|---|---|---|---|---|---|---|
| Q0 | DROP_OLDEST | 8 | no delay | **37 ms** (= measurement time) | 0 | [`day2_q0_normal.log`](logs/day2/day2_q0_normal.log) |
| Q1 | DROP_NEWEST (Day 1) | 8 | 1 s | **7.89–8.08 s** | 222 | [`day2_q1_newest8.log`](logs/day2/day2_q1_newest8.log) |
| Q2 | DROP_OLDEST | 8 | 1 s | **1.44–1.63 s** | 222 | [`day2_q2_oldest8.log`](logs/day2/day2_q2_oldest8.log) |
| Q3 | DROP_OLDEST (mailbox) | 1 | 1 s | **40–234 ms** | 229 | [`day2_q3_mailbox.log`](logs/day2/day2_q3_mailbox.log) |

- Under sustained overload the **number** of lost samples is set by the rate mismatch, not the
  policy (Q1 = Q2; Q3 is higher by exactly 7 = the difference in queue capacity). The policy only
  decides **which** samples survive, i.e. how stale the output is.
- Why the mailbox age still climbs to 234 ms: the consumer uses `osDelay(1000)` (relative) plus a
  ~6 ms print, so it drifts 6 ms per line against the 200 ms sample grid. In Q3 the age rises
  66 → 72 → … → 234 ms, then wraps to 40 ms when the consumer slips past the next sample
  (`seq=160 … age=234 ms` → `seq=166 … age=40 ms`).
- Chosen default: **DROP_OLDEST, length 8**: no loss and 37 ms age in normal operation, bounded
  staleness (≤ ~1.6 s) and ~1.4 s of burst buffering under overload. A control loop that only
  needs the current value should use the mailbox (Q3).

`DROP_OLDEST` is built from `Get` + `Put` because CMSIS-RTOS v2 has no overwrite-put. If the
higher-priority consumer runs between the two calls and takes a sample, that sample is consumed,
not lost, so the drop count stays correct.

---

## Day 3 — a small change, proven by measurement

**Observation (Day 2, M3):** after the start command the task polled data-ready every 2 ms, but with
a 33 ms timing budget no result can exist before ~33 ms. Fifteen polls per sample were wasted bus
traffic.

**Change:** `VL_FIRST_POLL_MS` (default 30). After `StartMeasurement` the task `osDelay`s 30 ms,
then polls every 2 ms as before. `0` restores the Day 2 behaviour. The 100 ms timeout is measured
from before the start command, so fault detection is unchanged. A compile-time check rejects a
delay that is not shorter than the budget.

Same board, same wiring, same session, only the switch changed:

| | `VL_FIRST_POLL_MS 0` | `VL_FIRST_POLL_MS 30` |
|---|---|---|
| Wasted data-ready polls | 15 | **2** |
| I2C bus busy per sample | ≈ 11.5 ms | **≈ 6.3 ms (−45 %)** |
| PA6 width (measurement time) | 37.825 ms | 38.827 ms |
| UART `meas` / `age` | 37 / 37 ms | 38 / 38 ms |
| `drop` / `miss` / `err` | 0 / 0 / 0 | 0 / 0 / 0 |
| Capture | [`poll0_measure.png`](logs/day3/la/poll0_measure.png) | [`poll30_measure.png`](logs/day3/la/poll30_measure.png) |
| Log | [`day3_poll0.log`](logs/day3/day3_poll0.log) | [`day3_poll30.log`](logs/day3/day3_poll30.log) |

**The measurement time grew by 1 ms. That was not expected, so it was explained before being
accepted.** Polls happen on a 2 ms grid, and the grid's phase moved:

- `0`: polls at …, 33.1, **35.1 ms** after start → first ready at 35.1 ms (34.1 ms was still not ready)
- `30`: start burst 2.7 ms + delay → first poll at 32.0 ms, then 34.0 (not ready), **36.0 ms** (ready)

Together the two captures bracket the sensor's actual completion between **34.0 and 35.1 ms**.
Detection always lags completion by 0–2 ms depending on where the grid falls; the delay only moved
the grid. So the result is: **bus time −45 %, measurement time unchanged within the 2 ms polling
resolution.** The capture also shows the 30 ms `osDelay` lasting 29.3 ms: a FreeRTOS delay of
*n* ticks can end up to one tick early because the first tick is partial.

Notes on the comparison:
- The busy-time values are read from the captures (≈ 0.37 ms per poll); treat them as ±0.5 ms.
  The Day 2 figure of 13.5 ms was an earlier rough estimate and is superseded by the 11.5 ms
  measured here under the same conditions as the after-case.
- The target distance differed between the two runs (≈ 73 mm vs. ≈ 180 mm). With a fixed timing
  budget the ranging time does not depend on distance, and PA6 width confirms it.
- The first lines of `day3_poll30.log` contain output from the previous session before the reflash;
  the log is kept unedited.

---

## Build and flash

1. Open the project in STM32CubeIDE and build (`Debug`).
2. Copy `Debug/f446-rtos-logger.bin` to the `NODE_F446RE` USB drive (post-build binary output is
   enabled). This is the method used for all Day 2–3 results; see the debug-probe note below.
3. Open a terminal at 115200 8N1 on the ST-LINK VCP **before** pressing reset, so the boot lines
   (`[RST]`, `[CLK]`, `[CFG]`) are captured.

Compile-time switches at the top of `Core/Src/app_rtos.c`:

| Switch | Default | Purpose |
|---|---|---|
| `QUEUE_POLICY` / `QUEUE_LEN` | `QP_DROP_OLDEST` / 8 | overflow policy experiment (Day 2 §4) |
| `CONSUMER_DELAY_MS` | 0 | overload experiment |
| `FAULT_INJECT_EVERY` | 0 | inject a sensor soft reset every N samples (Day 2 §2) |
| `VL_FIRST_POLL_MS` | 30 | first data-ready poll delay; 0 = Day 2 behaviour (Day 3) |
| `TASK_MARKERS` | 1 | PA6/PA7 timing markers |
| `STACK_REPORT` | 1 | `[STK]` stack high-water mark every 5 s |

The experiment switches are kept in the final code so that every result in this README can be
reproduced from the same source. The active values are printed in the `[CFG]` boot line.

Clock check (close the serial terminal first):

```
pip install pyserial
python tools/clock_check.py COM3 180
```

---

## Known limitations and open issues

- **Sensor stuck after hot-plug (seen once, unresolved).** After hot-plug activity the sensor stayed
  in a state where every soft reset + init timed out, and it survived MCU resets; only a sensor power
  cycle cleared it. 13 injected soft resets did not reproduce it. With XSHUT (or a switched sensor
  supply) this would be recoverable in software; this 4-pin module has neither.
- **Hot-plug resets the MCU** (Day 2 §2). Firmware cannot fix this; it needs inrush limiting.
- **Blocking UART.** Each log line holds the CPU for ~6 ms inside the consumer. UART DMA would free
  it, but would also change M6 and the meaning of `age`, so it was left for a separate,
  re-measured change.
- **Polling, not interrupts.** Data-ready is still polled (now 2–3 times per sample). The module has
  no GPIO1 pin, which would remove polling entirely.
- **Debug probe connection.** CubeIDE's GDB server currently fails to start (`Error in initializing
  ST-LINK device`); flashing by drag-and-drop works. Root cause not investigated within this project.

## What I would do next

1. UART DMA, then re-measure M6, consumer CPU time, and `age`.
2. A sensor module with XSHUT wired to a GPIO, and a power-cycle step in the ERROR path; then retry
   the hot-plug stuck state.
3. Make the consumer periodic with `osDelayUntil` in the overload test to remove the 6 ms/line drift
   seen in Q3, and compare.
