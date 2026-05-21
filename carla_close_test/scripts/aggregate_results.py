#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aggregate per-task performance results from closetest_main.py into a summary.

Usage:
  python3 aggregate_results.py [performance_result_dir]

  Defaults to <this script's package>/performance_result/
  Output: performance_result/summary.txt
"""

import os
import sys
import glob
import math
import re
from datetime import datetime


def _parse_result_file(fpath):
    """Parse a closetest_result_*.txt file into a dict of metrics."""
    result = {}
    with open(fpath, 'r') as f:
        for line in f:
            line = line.strip()

            if line.startswith('Task:'):
                result['task'] = line.split(':', 1)[1].strip()
            elif line.startswith('Timeout:'):
                result['timeout'] = line.split(':', 1)[1].strip() == 'True'
            elif line.startswith('Collision:'):
                result['collision'] = line.split(':', 1)[1].strip() == 'True'
            elif line.startswith('Parking attempted:'):
                result['parking_attempted'] = line.split(':', 1)[1].strip() == 'True'
            elif line.startswith('Correct space:'):
                val = line.split(':', 1)[1].strip()
                result['correct_space'] = val.startswith('True')
            elif line.startswith('Success (SR):'):
                result['success'] = line.split(':', 1)[1].strip() == 'True'
            elif line.startswith('SPL:'):
                try:
                    result['spl'] = float(line.split(':', 1)[1].strip())
                except ValueError:
                    result['spl'] = float('nan')
            elif line.startswith('APE:'):
                try:
                    val = line.split(':', 1)[1].strip().replace(' m', '')
                    result['ape'] = float(val)
                except ValueError:
                    result['ape'] = float('nan')
            elif line.startswith('AOE:'):
                try:
                    val = line.split(':', 1)[1].strip().replace(' deg', '')
                    result['aoe'] = float(val)
                except ValueError:
                    result['aoe'] = float('nan')
    return result


def _nanmean(vals):
    valid = [v for v in vals if not math.isnan(v)]
    return sum(valid) / len(valid) if valid else float('nan')


def _nanstd(vals):
    valid = [v for v in vals if not math.isnan(v)]
    if len(valid) < 2:
        return float('nan')
    m = sum(valid) / len(valid)
    return math.sqrt(sum((v - m) ** 2 for v in valid) / (len(valid) - 1))


def _fmt(v, unit='', decimals=4):
    if math.isnan(v):
        return 'N/A'
    return ('%.*f%s' % (decimals, v, (' ' + unit) if unit else ''))


def aggregate(result_root):
    """
    Scan all task subfolders under result_root, collect per-task results,
    write summary.txt.
    """
    pattern = os.path.join(result_root, '*', 'closetest_result_*.txt')
    files   = sorted(glob.glob(pattern))

    if not files:
        print("No result files found under: %s" % result_root)
        sys.exit(0)

    # For each task folder, take the latest result file (by filename timestamp)
    task_files = {}
    for fpath in files:
        task_name = os.path.basename(os.path.dirname(fpath))
        if task_name not in task_files or fpath > task_files[task_name]:
            task_files[task_name] = fpath

    rows = []
    for task_name in sorted(task_files.keys()):
        fpath = task_files[task_name]
        parsed = _parse_result_file(fpath)
        parsed.setdefault('task',           task_name)
        parsed.setdefault('correct_space',  False)
        parsed.setdefault('success',        False)
        parsed.setdefault('collision',      False)
        parsed.setdefault('spl',            0.0)
        parsed.setdefault('ape',            float('nan'))
        parsed.setdefault('aoe',            float('nan'))
        rows.append(parsed)

    n = len(rows)

    # ── Per-task table ────────────────────────────────────────────────────────
    header = ('%-26s  %-13s  %-7s  %-9s  %-7s  %-8s  %s'
              % ('task', 'correct_space', 'SR', 'collision', 'SPL', 'APE(m)', 'AOE(deg)'))
    sep    = '-' * len(header)

    table_lines = [header, sep]
    for r in rows:
        table_lines.append(
            '%-26s  %-13s  %-7s  %-9s  %-7s  %-8s  %s' % (
                r['task'][:26],
                str(r['correct_space']),
                str(r['success']),
                str(r['collision']),
                _fmt(r['spl'], decimals=3),
                _fmt(r['ape'], unit='m', decimals=3),
                _fmt(r['aoe'], unit='', decimals=2),
            )
        )

    # ── Aggregate metrics ─────────────────────────────────────────────────────
    correct_rate  = sum(1 for r in rows if r['correct_space']) / n * 100
    success_rate  = sum(1 for r in rows if r['success'])       / n * 100
    collision_rate = sum(1 for r in rows if r['collision'])    / n * 100
    mean_spl      = _nanmean([r['spl'] for r in rows])
    std_spl       = _nanstd( [r['spl'] for r in rows])
    mean_ape      = _nanmean([r['ape'] for r in rows])
    std_ape       = _nanstd( [r['ape'] for r in rows])
    mean_aoe      = _nanmean([r['aoe'] for r in rows])
    std_aoe       = _nanstd( [r['aoe'] for r in rows])

    agg_lines = [
        '',
        '=== Aggregate (N=%d) ===' % n,
        'Correct Space Rate: %.1f%%' % correct_rate,
        'Success Rate (SR):  %.1f%%' % success_rate,
        'Collision Rate:     %.1f%%' % collision_rate,
        'Mean SPL:           %s ± %s' % (_fmt(mean_spl, decimals=3), _fmt(std_spl, decimals=3)),
        'Mean APE:           %s ± %s m' % (_fmt(mean_ape, decimals=3), _fmt(std_ape, decimals=3)),
        'Mean AOE:           %s ± %s deg' % (_fmt(mean_aoe, decimals=2), _fmt(std_aoe, decimals=2)),
    ]

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(result_root, 'summary_%s.txt' % ts)

    with open(out_path, 'w') as f:
        f.write('=== Per-Task Results ===\n')
        f.write('\n'.join(table_lines) + '\n')
        f.write('\n'.join(agg_lines) + '\n')

    print('\n'.join(['=== Per-Task Results ==='] + table_lines + agg_lines))
    print('\nSummary written to: %s' % out_path)


def main():
    if len(sys.argv) > 1:
        result_root = sys.argv[1]
    else:
        result_root = os.path.join(os.path.dirname(__file__), '..', 'performance_result')
        result_root = os.path.normpath(result_root)

    if not os.path.isdir(result_root):
        print("Result directory not found: %s" % result_root)
        sys.exit(1)

    aggregate(result_root)


if __name__ == '__main__':
    main()
