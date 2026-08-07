#!/usr/bin/env python3
"""Does unplugging the PC's USB cable change the servo bus failure rate?

THE HYPOTHESIS BEING TESTED

The GH1.25 between the adapter board and the camera carries TX / RX / GND / 12V,
and that 12V powers the whole camera.  So the camera's entire supply return
current shares one thin wire with the UART's voltage reference.  If ground bounce
on that wire is what destroys transactions, then anything that changes the ground
topology should move the failure rate.

Right now there are two ground paths in parallel: adapter -> GH1.25 -> camera, and
camera -> USB cable -> laptop.  Unplugging the USB removes one of them (and breaks
the loop between them).  A shift in failure rate proves the ground path is a live
variable; no shift makes the grounding story a lot less likely.

WHY THIS SCRIPT AND NOT A STOPWATCH

Cross-time A/B on this bus is worthless -- the identical command has measured 25.2%
and 13.4% forty minutes apart.  The only defence is an interleaved ABAB design, and
interleaving here means plugging and unplugging repeatedly.  That is hard to align
with a human timer, and ssh dies the moment the cable comes out, so nothing can be
observed live.

So each round labels ITSELF: the USB state is read from sysfs before and after the
round, and a round whose state changed midway is discarded rather than mislabelled.
The operator can unplug and replug whenever they like, as often as they like.

Every round also records CPU, temperature, usb0 byte counters and the dwc3 / ttyS3
interrupt counts, because those are the confounds.  Unplugging removes the USB
controller's interrupt load and whatever traffic was flowing; if those move, the
result is not attributable to grounding alone and the numbers are there to say so.

READ-ONLY on the bus: sync_read only, no register is written.

REQUIRES /dev/ttyS3 TO BE FREE -- stop wheel_odometry_node first, or pyserial will
report "device reports readiness to read but returned no data" and every number
here will be garbage.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from tinynav.platforms.feetech_bus import FeetechBus, FeetechBusError  # noqa: E402

REGISTER = "Present_Position"
UDC_STATE = "/sys/class/udc/35100000.usb/state"
USB_CARRIER = "/sys/class/net/usb0/carrier"
USB_RX = "/sys/class/net/usb0/statistics/rx_bytes"
USB_TX = "/sys/class/net/usb0/statistics/tx_bytes"
THERMAL = "/sys/class/thermal/thermal_zone0/temp"
# Interrupt labels as they appear in the last column of /proc/interrupts on this SoC.
IRQ_LABELS = ("dwc3", "ttyS3")


def _read(path: str, default: str = "") -> str:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return default


def _int(path: str, default: int = -1) -> int:
    raw = _read(path)
    try:
        return int(raw)
    except ValueError:
        return default


def usb_label() -> str:
    """USB / NOUSB / '?' -- from two independent sysfs facts, which must agree.

    They can legitimately disagree for a moment right after a plug event (the udc
    reaches 'configured' before the netdev carrier comes up), and a disagreement is
    exactly the case this must not silently mislabel.
    """
    udc = _read(UDC_STATE)
    carrier = _read(USB_CARRIER)
    if udc == "configured" and carrier == "1":
        return "USB"
    if udc in ("not attached", "") and carrier in ("0", ""):
        return "NOUSB"
    return "?"


def read_cpu() -> tuple[int, int]:
    with open("/proc/stat") as fh:
        fields = [int(x) for x in fh.readline().split()[1:]]
    total = sum(fields)
    idle = fields[3] + (fields[4] if len(fields) > 4 else 0)
    return total - idle, total


def read_irqs() -> dict[str, int]:
    """Total count across all CPUs for each label in IRQ_LABELS."""
    out = {label: 0 for label in IRQ_LABELS}
    try:
        with open("/proc/interrupts") as fh:
            next(fh, None)
            for line in fh:
                parts = line.split()
                if not parts or not parts[0].endswith(":"):
                    continue
                tail = parts[-1]
                if tail not in out:
                    continue
                total = 0
                for token in parts[1:]:
                    if token.isdigit():
                        total += int(token)
                    else:
                        break
                out[tail] = total
    except OSError:
        pass
    return out


class Snapshot:
    def __init__(self) -> None:
        self.t = time.monotonic()
        self.label = usb_label()
        self.cpu = read_cpu()
        self.irq = read_irqs()
        self.rx = _int(USB_RX)
        self.tx = _int(USB_TX)
        self.temp = _int(THERMAL)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", default="/dev/ttyS3")
    ap.add_argument("--baudrate", type=int, default=1_000_000)
    ap.add_argument("--ids", default="7,8,9")
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--cycles-per-round", type=int, default=500)
    ap.add_argument("--rounds", type=int, default=130)
    ap.add_argument("--out", default="/userdata/x5/usb_ground_ab.tsv")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",") if x.strip()]
    period = 1.0 / args.rate_hz

    bus = FeetechBus(port=args.port, baudrate=args.baudrate, timeout=0.02)
    bus.connect()

    header = ("round\telapsed_s\tlabel\tfails\tcycles\tfail_pct\t"
              "cpu_pct\ttemp_c\tusb_rx_Bps\tusb_tx_Bps\tdwc3_irq_s\tttyS3_irq_s\n")
    out = open(args.out, "w")
    out.write(header)
    out.flush()
    os.fsync(out.fileno())
    print(header, end="", flush=True)

    t_start = time.monotonic()
    rows = []
    try:
        for r in range(args.rounds):
            before = Snapshot()
            fails = 0
            for _ in range(args.cycles_per_round):
                t0 = time.monotonic()
                try:
                    if not bus.sync_read(REGISTER, ids, num_retry=0).complete:
                        fails += 1
                except FeetechBusError:
                    fails += 1
                remaining = period - (time.monotonic() - t0)
                if remaining > 0:
                    time.sleep(remaining)
            after = Snapshot()

            # A round straddling a plug event is not evidence for either side.
            label = before.label if before.label == after.label else "TRANS"
            dt = max(after.t - before.t, 1e-6)
            d_busy = after.cpu[0] - before.cpu[0]
            d_total = after.cpu[1] - before.cpu[1]
            cpu_pct = 100.0 * d_busy / d_total if d_total > 0 else float("nan")
            row = {
                "round": r,
                "elapsed": after.t - t_start,
                "label": label,
                "fails": fails,
                "cycles": args.cycles_per_round,
                "pct": 100.0 * fails / args.cycles_per_round,
                "cpu": cpu_pct,
                "temp": after.temp / 1000.0,
                "rx": (after.rx - before.rx) / dt,
                "tx": (after.tx - before.tx) / dt,
                "dwc3": (after.irq["dwc3"] - before.irq["dwc3"]) / dt,
                "ttys3": (after.irq["ttyS3"] - before.irq["ttyS3"]) / dt,
            }
            rows.append(row)
            line = (f"{row['round']}\t{row['elapsed']:.1f}\t{row['label']}\t"
                    f"{row['fails']}\t{row['cycles']}\t{row['pct']:.2f}\t"
                    f"{row['cpu']:.1f}\t{row['temp']:.1f}\t{row['rx']:.0f}\t"
                    f"{row['tx']:.0f}\t{row['dwc3']:.0f}\t{row['ttys3']:.0f}\n")
            out.write(line)
            out.flush()
            os.fsync(out.fileno())
            print(line, end="", flush=True)
    finally:
        bus.disconnect()
        out.close()

    # ---- summary ----
    print("=" * 78, flush=True)
    print(f"servo bus vs PC USB cable, ids {ids}, {args.rate_hz} Hz single-attempt",
          flush=True)
    print("=" * 78, flush=True)
    print(f"  {'label':>6}  {'rounds':>6}  {'fails/cycles':>14}  {'fail%':>7}  "
          f"{'cpu%':>6}  {'temp':>6}  {'usbTx B/s':>9}  {'dwc3/s':>7}", flush=True)
    groups: dict[str, list] = {}
    for row in rows:
        groups.setdefault(row["label"], []).append(row)
    for label in ("USB", "NOUSB", "TRANS", "?"):
        g = groups.get(label)
        if not g:
            continue
        f = sum(x["fails"] for x in g)
        c = sum(x["cycles"] for x in g)
        mean = lambda k: sum(x[k] for x in g) / len(g)  # noqa: E731
        print(f"  {label:>6}  {len(g):>6}  {f:>6}/{c:<7}  {100.0 * f / c:>6.2f}%  "
              f"{mean('cpu'):>5.1f}  {mean('temp'):>5.1f}  {mean('tx'):>9.0f}  "
              f"{mean('dwc3'):>7.0f}", flush=True)

    a, b = groups.get("USB", []), groups.get("NOUSB", [])
    if a and b:
        fa = sum(x["fails"] for x in a) / sum(x["cycles"] for x in a)
        fb = sum(x["fails"] for x in b) / sum(x["cycles"] for x in b)
        na = sum(x["cycles"] for x in a)
        nb = sum(x["cycles"] for x in b)
        # Pooled two-proportion z. It assumes rounds are independent, which they are
        # not (lambda drifts on a seconds timescale), so read it as an upper bound on
        # significance, not a p-value. The ABAB alternation is what actually protects
        # against drift; this number only says whether the counts are even worth it.
        p = (fa * na + fb * nb) / (na + nb)
        se = (p * (1 - p) * (1 / na + 1 / nb)) ** 0.5
        z = (fa - fb) / se if se > 0 else float("nan")
        print("-" * 78, flush=True)
        print(f"  USB {100 * fa:.2f}%  vs  NOUSB {100 * fb:.2f}%   "
              f"delta {100 * (fa - fb):+.2f} points   z = {z:+.2f} (optimistic)",
              flush=True)
        print("  Judge it from the per-round column too: a real effect flips with every"
              " plug event.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
