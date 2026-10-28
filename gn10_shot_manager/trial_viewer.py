"""Trial Viewer: Terminal viewer and statistical analyzer for recorded shot trials."""

import argparse
import json
import math
import os
import sys


def load_trials(filepath: str):
    if not os.path.exists(filepath):
        print(f"Error: Log file not found: {filepath}")
        return []

    records = []
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except Exception as e:
                print(f"Warning: skipping invalid JSON line: {e}")
    return records


def print_summary_table(records):
    if not records:
        print("No trial records found.")
        return

    print("=" * 88)
    print(" GN10 SHOT TRIAL RECORDS SUMMARY")
    print("=" * 88)
    print(f"{'Shot#':<6} {'Timestamp':<19} {'Target':<14} {'Target(X,Y)':<15} {'Error [m]':<11} {'V_max':<8} {'I_max':<8}")
    print("-" * 88)

    by_target = {}

    for r in records:
        shot_id = r.get("shot_id", 0)
        ts = r.get("timestamp", "")[:19]
        target = r.get("target", {})
        t_name = target.get("name", f"Target_{target.get('target_id', '?')}")
        t_xy = f"({target.get('x', 0):.2f}, {target.get('y', 0):.2f})"
        
        err = r.get("error")
        err_str = f"{err['distance']:.3f}m" if err else "N/A"

        summary = r.get("summary", {})
        v_max = f"{summary.get('max_velocity', 0.0):.1f}m/s"
        i_max = f"{summary.get('max_current', 0.0):.1f}A"

        print(f"{shot_id:<6} {ts:<19} {t_name:<14} {t_xy:<15} {err_str:<11} {v_max:<8} {i_max:<8}")

        # Group for statistics
        if t_name not in by_target:
            by_target[t_name] = []
        if err:
            by_target[t_name].append(err["distance"])

    print("=" * 88)
    print("\n TARGET ACCURACY STATISTICS")
    print("-" * 88)
    print(f"{'Target Name':<20} {'Trials':<8} {'Mean Error [m]':<18} {'Std Dev [m]':<18}")
    print("-" * 88)

    for t_name, errs in by_target.items():
        n = len(errs)
        if n == 0:
            print(f"{t_name:<20} {0:<8} {'N/A':<18} {'N/A':<18}")
            continue
        mean = sum(errs) / n
        variance = sum((x - mean) ** 2 for x in errs) / n
        std = math.sqrt(variance)
        print(f"{t_name:<20} {n:<8} {mean:.4f}m{'':<11} {std:.4f}m")
    print("=" * 88)


def print_detail(record):
    print("=" * 60)
    print(f" SHOT #{record.get('shot_id')} DETAIL")
    print("=" * 60)
    print(f"Timestamp:   {record.get('timestamp')}")
    target = record.get("target", {})
    print(f"Target:      {target.get('name')} (ID: {target.get('target_id')})")
    print(f"Target Pose: X={target.get('x')}m, Y={target.get('y')}m, Yaw={target.get('yaw')}rad")
    print(f"Memo:        {target.get('memo')}")
    print("-" * 60)
    summary = record.get("summary", {})
    print(f"Max Velocity: {summary.get('max_velocity')} m/s")
    print(f"Max Current:  {summary.get('max_current')} A")
    print(f"Avg Voltage:  {summary.get('avg_voltage')} V")
    
    landing = record.get("landing_position")
    err = record.get("error")
    if landing:
        print(f"Landing Pos:  X={landing.get('x'):.3f}m, Y={landing.get('y'):.3f}m, Z={landing.get('z'):.3f}m")
    if err:
        print(f"Landing Err:  Distance={err.get('distance'):.4f}m (dx={err.get('dx'):.4f}m, dy={err.get('dy'):.4f}m)")

    samples = record.get("samples", [])
    print(f"Samples Recorded: {len(samples)} (at ~50Hz)")
    if samples:
        print(f"{'t [s]':>6} {'V_act [m/s]':>12} {'V_tgt [m/s]':>12} {'Current [A]':>12} {'Volt [V]':>10}")
        print("-" * 56)
        # Show first, peak, and last few
        for s in samples[::5]:  # every 100ms
            print(f"{s['t']:>6.2f} {s['v_act']:>12.2f} {s['v_tgt']:>12.2f} {s['curr']:>12.2f} {s['volt']:>10.2f}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="View GN10 shot trial logs and statistics")
    default_log = os.path.expanduser("~/ros2_ws/data/shot-trial-log/trials.jsonl")
    fallback_log = os.path.expanduser("~/ros2_ws/data/trials/trials.jsonl")
    chosen_default = default_log if os.path.exists(default_log) or not os.path.exists(fallback_log) else fallback_log

    parser.add_argument("--file", "-f", default=chosen_default, help="Path to trials.jsonl file")
    parser.add_argument("--shot", "-s", type=int, default=None, help="Show detailed samples for a specific shot id")
    parser.add_argument("--last", "-l", action="store_true", help="Show detail for the most recent shot")
    args = parser.parse_args()

    records = load_trials(args.file)
    if not records:
        return

    if args.shot is not None:
        target_rec = next((r for r in records if r.get("shot_id") == args.shot), None)
        if target_rec:
            print_detail(target_rec)
        else:
            print(f"Shot #{args.shot} not found in log.")
    elif args.last:
        print_detail(records[-1])
    else:
        print_summary_table(records)


if __name__ == "__main__":
    main()

