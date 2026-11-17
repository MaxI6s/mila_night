#!/usr/bin/env python3
"""
meca_torque_delta.py - CLI monitor of joint torque delta on a Meca500.

Displays, for each joint:
    calculated torque  (GetRtTargetJointTorq, trajectory planner model)
    measured torque    (GetRtJointTorq, estimated from motor current)
    delta = measured - calculated
All values are in % of the joint's maximum allowable torque.

Requires firmware 11.1+ and mecademicpy (pip install mecademicpy).

Usage:
    python meca_torque_delta.py --ip 192.168.0.100
    python meca_torque_delta.py --ip 192.168.0.100 --period 0.1 --threshold 10
    python meca_torque_delta.py --ip 192.168.0.100 --log  # one line per sample instead of a live table
"""

import argparse
import sys
import time

import mecademicpy.robot as mdr

NB_JOINTS = 6
CLEAR = "\033[H\033[J"  # ANSI: cursor home + clear screen


def parse_args():
    p = argparse.ArgumentParser(description="Meca500 joint torque delta (measured - calculated)")
    p.add_argument("--ip", default="192.168.0.100", help="Robot IP address (default: 192.168.0.100)")
    p.add_argument("--period", type=float, default=0.2, help="Display refresh period in s (default: 0.2)")
    p.add_argument("--monitor-interval", type=float, default=0.015,
                   help="Robot monitoring interval in s (default: 0.015)")
    p.add_argument("--threshold", type=float, default=None,
                   help="Flag joints whose |delta| exceeds this value (in %%)")
    p.add_argument("--log", action="store_true",
                   help="Print one line per sample instead of refreshing a table")
    return p.parse_args()


def read_torques(robot):
    """Return (timestamp_us, calculated[6], measured[6]) or None if data not ready yet."""
    rt = robot.GetRobotRtData(synchronous_update=False)
    calc = rt.rt_target_joint_torq
    meas = rt.rt_joint_torq
    if len(calc.data) < NB_JOINTS or len(meas.data) < NB_JOINTS:
        return None
    return meas.timestamp, list(calc.data[:NB_JOINTS]), list(meas.data[:NB_JOINTS])


def print_table(ts, calc, meas, threshold):
    lines = [
        f"Meca500 joint torque delta   (t = {ts / 1e6:.3f} s, values in % of max)   Ctrl+C to quit",
        "",
        f"{'Joint':<6}{'Calculated':>12}{'Measured':>12}{'Delta':>10}",
        "-" * 40,
    ]
    for i in range(NB_JOINTS):
        delta = meas[i] - calc[i]
        flag = "  <-- over" if threshold is not None and abs(delta) > threshold else ""
        lines.append(f"J{i + 1:<5}{calc[i]:>12.2f}{meas[i]:>12.2f}{delta:>+10.2f}{flag}")
    sys.stdout.write(CLEAR + "\n".join(lines) + "\n")
    sys.stdout.flush()


def print_log_line(ts, calc, meas, threshold):
    parts = []
    for i in range(NB_JOINTS):
        delta = meas[i] - calc[i]
        flag = "!" if threshold is not None and abs(delta) > threshold else " "
        parts.append(f"J{i + 1}:{delta:+7.2f}{flag}")
    print(f"{ts / 1e6:12.3f}  " + " ".join(parts), flush=True)


def main():
    args = parse_args()
    robot = mdr.Robot()

    print(f"Connecting to {args.ip} ...")
    try:
        robot.Connect(address=args.ip)
    except Exception as e:
        print(f"Connection failed: {e}", file=sys.stderr)
        return 1

    try:
        # Stream both calculated (target) and measured torques on the monitoring port
        robot.SetMonitoringInterval(args.monitor_interval)
        robot.SetRealTimeMonitoring("TargetJointTorq", "JointTorq")
        time.sleep(0.2)  # let the first samples arrive

        if args.log:
            print(f"{'time [s]':>12}  delta = measured - calculated (% of max)")

        last_ts = None
        while True:
            sample = read_torques(robot)
            if sample is None:
                time.sleep(args.period)
                continue
            ts, calc, meas = sample
            if args.log and ts == last_ts:  # avoid duplicate lines
                time.sleep(args.period)
                continue
            last_ts = ts

            if args.log:
                print_log_line(ts, calc, meas, args.threshold)
            else:
                print_table(ts, calc, meas, args.threshold)
            time.sleep(args.period)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            robot.SetRealTimeMonitoring()  # disable extra RT streams
        except Exception:
            pass
        robot.Disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
