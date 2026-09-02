#!/usr/bin/env python3
"""Independent empirical detector calibration and power-vs-window table."""

import argparse
import bisect
import csv
import math
import pathlib


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def empirical_quantile(values, probability):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return math.nan
    # Conservative order statistic: no interpolation below the requested tail.
    return values[min(len(values)-1, math.ceil(probability*len(values))-1)]


def rolling_sum(values, window):
    output, total = [], 0.0
    for index, value in enumerate(values):
        total += value
        if index >= window:
            total -= values[index-window]
        output.append(total if index+1 >= window else math.nan)
    return output


def fault_labels(run, timestamps):
    truth = read_csv(run / "fault_truth.csv")
    active_times = sorted(int(row["timestamp_ns"]) for row in truth
                          if row["active"] in ("1", "true", "True"))
    if not active_times:
        return [False] * len(timestamps)
    first = active_times[0]
    return [timestamp >= first for timestamp in timestamps]


def detector_values(run, field):
    integrity = read_csv(run / "integrity.csv")
    timestamps, values = [], []
    for row in integrity:
        try:
            value = float(row[field])
        except (KeyError, ValueError):
            value = math.nan
        timestamps.append(int(row["timestamp_ns"]))
        values.append(value)
    return timestamps, values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration-run", type=pathlib.Path, required=True)
    parser.add_argument("--evaluation-run", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    detectors = {
        "global": "global_statistic",
        "postfit": "postfit_statistic",
        "conditional": "conditional_statistic",
    }
    pfas = (1e-2, 1e-3, 1e-4)
    windows = (1, 10, 50, 200)
    records = []
    for detector, field in detectors.items():
        _, calibration = detector_values(args.calibration_run, field)
        timestamps, evaluation = detector_values(args.evaluation_run, field)
        labels = fault_labels(args.evaluation_run, timestamps)
        if not any(math.isfinite(value) for value in calibration):
            continue
        for window in windows:
            calibration_window = rolling_sum(calibration, window)
            evaluation_window = rolling_sum(evaluation, window)
            for pfa in pfas:
                threshold = empirical_quantile(calibration_window, 1-pfa)
                nominal = [value for value, fault in zip(evaluation_window, labels)
                           if not fault and math.isfinite(value)]
                faulted = [value for value, fault in zip(evaluation_window, labels)
                           if fault and math.isfinite(value)]
                false_alarms = sum(value > threshold for value in nominal)
                detections = sum(value > threshold for value in faulted)
                records.append({
                    "detector": detector, "history_length": window,
                    "target_p_fa": pfa, "threshold": threshold,
                    "calibration_count": sum(math.isfinite(value)
                                             for value in calibration_window),
                    "nominal_count": len(nominal),
                    "false_alarms": false_alarms,
                    "empirical_p_fa": false_alarms/len(nominal) if nominal else math.nan,
                    "fault_count": len(faulted), "detections": detections,
                    "detection_probability": detections/len(faulted) if faulted else math.nan,
                    "formal": False,
                })
        if detector == "conditional":
            integrity = read_csv(args.evaluation_run / "integrity.csv")
            valid = [(float(row["conditional_statistic"]),
                      float(row["conditional_threshold"]), label)
                     for row, label in zip(integrity, labels)
                     if math.isfinite(float(row["conditional_statistic"])) and
                     math.isfinite(float(row["conditional_threshold"]))]
            if valid:
                nominal = [stat > threshold for stat, threshold, label in valid if not label]
                faulted = [stat > threshold for stat, threshold, label in valid if label]
                records.append({
                    "detector": detector, "history_length": 1,
                    "target_p_fa": 1e-5, "threshold": "per-DOF theoretical",
                    "calibration_count": 0, "nominal_count": len(nominal),
                    "false_alarms": sum(nominal),
                    "empirical_p_fa": sum(nominal)/len(nominal) if nominal else math.nan,
                    "fault_count": len(faulted), "detections": sum(faulted),
                    "detection_probability": sum(faulted)/len(faulted) if faulted else math.nan,
                    "formal": True,
                })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = ("detector", "history_length", "target_p_fa", "threshold",
              "calibration_count", "nominal_count", "false_alarms",
              "empirical_p_fa", "fault_count", "detections",
              "detection_probability", "formal")
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)


if __name__ == "__main__":
    main()
