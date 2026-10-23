"""Offline R-record audit; no serial port opened. python analyze_log.py run.txt
Optional --truth 100 100 100 applies ONLY to fixed-geometry stationary runs.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

def audit(text, truth=None):
    streams = [[], [], []]
    for line in text.splitlines():
        fields = line.strip().split(',')
        if len(fields) != 9 or fields[0] != 'R':
            continue
        try:
            row = list(map(int, fields[1:]))
        except ValueError:
            continue
        if row[1] in range(3):
            streams[row[1]].append(row)
    result = []
    for i, rows in enumerate(streams):
        valid = [r for r in rows if r[6] == 1 and r[4] == 0 and r[5] == 0]
        values = [r[3] for r in valid]
        times = [r[2] for r in valid]
        delta = lambda a, b: (a-b) & 0xffffffff
        gaps = [delta(b, a) for a, b in zip(times, times[1:])]
        head = delta(times[0], rows[0][2]) if times else None
        tail = delta(rows[-1][0], times[-1]) if times else None
        # Firmware's online worst gap covers dropped R telemetry between valids.
        online = max((r[7] for r in rows), default=0)
        candidates = gaps + ([head, tail] if times else []) + [online]
        span = delta(rows[-1][0], rows[0][2]) if rows else None
        item = dict(sensor=['front','left','right'][i], logged_attempts=len(rows),
                    logged_valid=len(valid), logged_valid_rate=len(valid)/len(rows) if rows else None,
                    valid_gap_max_ms=max(gaps,default=None),
                    valid_gap_p95_ms=sorted(gaps)[math.ceil(.95*len(gaps))-1] if gaps else None,
                    head_ms=head, tail_ms=tail,
                    logged_duration_ms=span,
                    worst_observed_no_new_data_ms=max(candidates) if rows and times else span,
                    api_error_counts=dict(Counter(r[4] for r in rows)),
                    range_status_counts=dict(Counter(r[5] for r in rows)),
                    mean_raw_mm=sum(values)/len(values) if values else None,
                    min_raw_mm=min(values,default=None),max_raw_mm=max(values,default=None))
        if truth and values:
            item['bias_mm'] = item['mean_raw_mm'] - truth[i]
            item['max_abs_error_mm'] = max(abs(v-truth[i]) for v in values)
        result.append(item)
    return dict(sensors=result, limitations='Logged samples only; UART can skip rows. '
                'Online gap maxima supplement logs, but cannot prove a future latency bound. '
                'Invalid/no-target ranges are not evidence of free space. '
                'Bias from a fixed stationary geometry is not a universal calibration.')

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', type=Path)
    p.add_argument('--truth',type=float,nargs=3)
    args=p.parse_args()
    print(json.dumps(audit(args.log.read_text(encoding='utf-8-sig'),args.truth),ensure_ascii=False,indent=2))
