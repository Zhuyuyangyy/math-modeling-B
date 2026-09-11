import json
import statistics

rows = json.load(open('validation_v4/paired/offline_paired.json', encoding='utf-8'))
for prob in (3, 4):
    sub = [r for r in rows if r['problem'] == prob]
    by = {}
    for r in sub:
        by.setdefault(r['variant'], []).append(r)
    print(f'=== Q{prob} (n={len(sub) // len(by)} seeds) ===')
    for v in ('v3', 'v4'):
        rs = by[v]
        T = [r['virtual_time_s'] for r in rs]
        tail = [r['tail_s'] for r in rs]
        mv = [r['movement_m'] for r in rs]
        ms = [r['measures'] for r in rs]
        ok = all(r['all_cleared'] for r in rs)
        print(f"{v:4s} mean_T={statistics.mean(T):8.0f}s median_T={statistics.median(T):8.0f}s "
              f"mean_tail={statistics.mean(tail):7.0f}s mean_move={statistics.mean(mv):7.0f}m "
              f"mean_meas={statistics.mean(ms):6.1f} all_cleared={ok}")
    # paired win counts
    m = {r['seed']: r for r in by['v3']}
    v4 = {r['seed']: r for r in by['v4']}
    wins = sum(1 for s, r in v4.items() if s in m and r['virtual_time_s'] < m[s]['virtual_time_s'])
    losses = sum(1 for s, r in v4.items() if s in m and r['virtual_time_s'] > m[s]['virtual_time_s'])
    ties = sum(1 for s, r in v4.items() if s in m and r['virtual_time_s'] == m[s]['virtual_time_s'])
    print(f"  v4 wins={wins} losses={losses} ties={ties} (of {len(m)})")
