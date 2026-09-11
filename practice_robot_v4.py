"""v4: v3 + certificate-contributing scan-site filter + remaining-cell route jobs.

Two provably-safe routing improvements on top of v3:
1. A scan site with potential==0 for EVERY unresolved channel can never
   help close any channel's certificate (Q3 whole-disk exclusion / Q4
   common-station direction-cover both require the station within 1000m of
   a still-remaining cell). Visiting it is wasted movement -> drop from route.
2. Undetected unresolved channels still have remaining cells; add their
   nearest remaining cell as a route job so the robot closes the last
   certificate gaps directly instead of only after the fixed sites run out.

Correctness is unchanged: retirement still requires the full whole-cell
certificate; the offline harness still asserts the true source is never
excluded and that the run completes.
"""
import argparse
import json
import math
from pathlib import Path
import time

from practice_robot import Robot, dist, search_sites, practice_preflight
from practice_robot_v3 import EvidenceRobot, open_route
from certificate_geometry import Certificates, enclosing_circle


class EvidenceRobotV4(EvidenceRobot):
    def __init__(self, problem, case_label, out, variant='v4'):
        super().__init__(problem, case_label, out)
        self.variant = 'v4'

    def useful_sites(self, sites):
        """Sites that can still contribute to at least one unresolved channel.
        whole_disk depends only on the point, so it is computed once per site."""
        unresolved = self.unresolved()
        if not unresolved:
            return []
        out = []
        for i, p in enumerate(sites):
            disk = self.cert.whole_disk(p)
            useful = any((disk & self.cert.remaining(ch)).any() for ch in unresolved)
            if useful:
                out.append((i, p))
        return out

    def remaining_jobs(self):
        """Nearest still-unmeasured remaining cell per undetected unresolved
        channel (Q3 only). A Q3 no-signal excludes a whole 1000m disk, so a few
        visits close the certificate fast. For Q4 a single measure rarely
        excludes any cell (needs >=3 common stations), so per-cell visits are
        both slow and unproductive: Q4 keeps v3's fixed-site + supplemental
        path, where the 31 sites provide the stations the certificate needs."""
        if self.problem != 3:
            return []
        jobs = []
        for ch in sorted(self.unresolved() - set(self.polys)):
            cells = self.cert.centers[self.cert.remaining(ch)]
            if len(cells) == 0:
                continue
            cand = [tuple(c) for c in cells
                    if (ch, float(c[0]), float(c[1])) not in self.measured]
            if not cand:
                continue
            p = min(cand, key=lambda q: dist(self.pos, q))
            jobs.append((('remain', ch), p))
        return jobs

    def run(self):
        started = time.monotonic()
        entry = self.post('/enter')
        self.deadline = time.monotonic()+entry['remaining_real_duration_s']
        sites = search_sites(self.problem)
        total_sites = len(sites)
        visited = 0
        error = None
        try:
            while self.unresolved() and len(self.cleared) < 16:
                pending = sorted(set(self.polys) - self.cleared)
                if not sites and not pending:
                    if self.supplemental_sites >= 60:
                        raise RuntimeError('Conservative certificate incomplete after supplemental scans')
                    ch = min(self.unresolved())
                    cells = self.cert.centers[self.cert.remaining(ch)]
                    p = min(map(tuple, cells), key=lambda q: dist(self.pos, q))
                    if self.problem == 3:
                        extra = [tuple(p)]
                    else:
                        extra = [(p[0]+200*math.cos(k*math.tau/3), p[1]+200*math.sin(k*math.tau/3)) for k in range(3)]
                    sites.extend(extra)
                    self.supplemental_sites += len(extra)

                # v4: certificate-filtered scan sites + remaining-cell jobs.
                jobs = [(('scan', i), p) for i, p in self.useful_sites(sites)]
                jobs += [(('target', ch), enclosing_circle(self.polys[ch])[0])
                         for ch in pending]
                jobs += [(('remain', ch), p) for (_, ch), p in self.remaining_jobs()]

                if not jobs:
                    break
                route = open_route(self.pos, jobs)
                (kind, index_or_ch), point = route[0]

                self.next_station = min(sites, key=lambda p: dist(self.pos, p)) if sites else None
                if kind == 'target':
                    self.localize(index_or_ch)
                elif kind == 'remain':
                    # Close a certificate gap for this undetected channel.
                    self.measure(point, index_or_ch)
                    self.opportunistic()
                else:
                    p = sites.pop(index_or_ch)
                    visited += 1
                    self.scan(p)
                    print(f'SITE {visited}/{total_sites} clear={len(self.cleared)} '
                          f'absent={len(self.retired)}', flush=True)
        except Exception as exc:
            error = repr(exc)
            print('ERROR '+error, flush=True)
        exit_result = self.post('/exit')
        complete = (not self.unresolved() or len(self.cleared) == 16)
        expected = self.movement/5+self.measures*5+self.switches+len(self.cleared)*5+self.failures*3
        summary = dict(mode='practice', strategy_version=self.variant, problem=self.problem,
                       label=self.label, cleared_count=len(self.cleared),
                       cleared_channels=sorted(self.cleared),
                       certified_absent_channels=sorted(self.retired),
                       total_sources=None, clearance_ratio=None,
                       virtual_time_s=self.virtual,
                       average_clear_time_s=self.virtual/len(self.cleared) if self.cleared else None,
                       local_wall_time_s=time.monotonic()-started, movement_m=self.movement,
                       measure_count=self.measures, switch_count=self.switches,
                       failed_clear_count=self.failures, request_count=self.seq,
                       visited_sites=visited, total_sites=total_sites,
                       coverage_completed=complete and error is None,
                       certificate_completed=not self.unresolved(),
                       exit_reason=exit_result['exit_reason'], error=error,
                       timing_reconciliation_error_s=self.virtual-expected,
                       last_clear_time_s=self.last_clear_time,
                       tail_confirmation_s=self.virtual-self.last_clear_time,
                       opportunistic_measure_count=self.opportunistic_measures,
                       tracking_no_signal_count=self.tracking_no_signal,
                       cached_measurements=self.cached_measurements,
                       supplemental_sites=self.supplemental_sites)
        (self.out/(self.label+'_summary.json')).write_text(
            json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
        if self.use_cert:
            certificate = self.cert.save()
            certificate['retirement_events'] = self.certificate_events
            (self.out/(self.label+'_certificate.json')).write_text(
                json.dumps(certificate, indent=2, ensure_ascii=False), encoding='utf-8')
        self.log.close()
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--problem', type=int, choices=[3, 4], required=True)
    parser.add_argument('--mode', choices=['practice'], required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--out', default=str(Path(__file__).parent/'results_v4'))
    args = parser.parse_args()
    practice_preflight(args.problem, args.label)
    EvidenceRobotV4(args.problem, args.label, args.out).run()
