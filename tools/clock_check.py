"""MCU 틱 클럭을 PC 시계와 비교해 ppm 오차를 추정한다.

[DATA] 줄의 t(MCU 틱, ms)와 PC 수신 시각을 모아 직선 회귀로 기울기를 구한다.
USB 지연 지터(수 ms)는 긴 측정 시간으로 평균되어 사라진다 (180 s면 약 10 ppm 분해능).

사용법 (PuTTY는 먼저 닫아서 COM 포트를 비워야 함):
    pip install pyserial
    python clock_check.py COM3 180
"""
import re
import sys
import time

import serial

PAT = re.compile(rb"\[DATA\] seq=(\d+) t=(\d+)")


def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0

    pc, mcu = [], []
    with serial.Serial(port, 115200, timeout=1) as ser:
        ser.reset_input_buffer()
        t_end = time.perf_counter() + dur
        while time.perf_counter() < t_end:
            line = ser.readline()
            now = time.perf_counter()
            m = PAT.search(line)
            if m:
                pc.append(now * 1000.0)          # ms
                mcu.append(float(m.group(2)))    # ms (MCU tick)
                if len(pc) % 50 == 0:
                    print(f"  {len(pc)} samples, {now - (t_end - dur):.0f} s")

    if len(pc) < 20:
        sys.exit("샘플이 너무 적음: 포트/보레이트 확인")

    # 부팅/재시작으로 t가 되감긴 경우 마지막 연속 구간만 사용
    start = 0
    for i in range(1, len(mcu)):
        if mcu[i] < mcu[i - 1]:
            start = i
    pc, mcu = pc[start:], mcu[start:]

    n = len(pc)
    mx, my = sum(pc) / n, sum(mcu) / n
    sxx = sum((x - mx) ** 2 for x in pc)
    sxy = sum((x - mx) * (y - my) for x, y in zip(pc, mcu))
    slope = sxy / sxx                              # MCU ms per PC ms
    resid = [y - (my + slope * (x - mx)) for x, y in zip(pc, mcu)]
    rstd = (sum(r * r for r in resid) / (n - 2)) ** 0.5

    span_s = (pc[-1] - pc[0]) / 1000.0
    ppm = (slope - 1.0) * 1e6
    print()
    print(f"samples      : {n}  over {span_s:.1f} s")
    print(f"slope        : {slope:.7f}  (MCU ms / PC ms)")
    print(f"MCU vs PC    : {ppm:+.0f} ppm   (+ = MCU 틱이 빠름)")
    print(f"drift        : {ppm * 3.6:+.1f} ms per hour")
    print(f"residual std : {rstd:.2f} ms  (USB 수신 지터)")
    print(f"=> 200 ms 주기는 실제 {200.0 / slope:.3f} ms")


if __name__ == "__main__":
    main()
