"""Evidence-driven PRACTICE ONLY runner, with v2 retained unchanged for pairing."""
import argparse
import json
import math
from pathlib import Path
import time

from practice_robot import Robot, dist, center, clip, search_sites, practice_preflight
from certificate_geometry import Certificates, enclosing_circle, nearest_clear_point


def open_route(start, jobs):
    """Nearest neighbor followed by open-path 2-opt. No fictitious return edge."""
    pool = list(jobs)
    route = []
    p = start
    while pool:
        item = min(pool, key=lambda item:dist(p,item[1]))
        pool.remove(item)
        route.append(item)
        p = item[1]
    for _ in range(4):
        changed = False
        for i in range(len(route)-1):
            a = start if i == 0 else route[i-1][1]
            b = route[i][1]
            for j in range(i+1,len(route)):
                c = route[j][1]
                d = route[j+1][1] if j+1 < len(route) else None
                old = dist(a,b)+(dist(c,d) if d is not None else 0)
                new = dist(a,c)+(dist(b,d) if d is not None else 0)
                if new < old-1e-7:
                    route[i:j+1] = reversed(route[i:j+1])
                    changed = True
                    break
            if changed:
                break
        if not changed:
            break
    return route


class EvidenceRobot(Robot):
    def __init__(self, problem, case_label, out, variant='v3'):
        super().__init__(problem,case_label,out)
        self.variant = variant
        self.use_cert = variant != 'mec'
        self.use_route = variant in ('route','v3')
        self.use_scoring = variant == 'v3'
        self.cert = Certificates(problem)
        self.retired = set()
        self.failed_positions = {}
        self.measured = {}
        self.certificate_events = []
        self.last_clear_time = 0
        self.opportunistic_measures = 0
        self.cached_measurements = 0
        self.supplemental_sites = 0
        self.next_station = None
        self.tracking_no_signal = 0
        self.tracking = False

    def unresolved(self):
        return set(range(1,21))-self.cleared-self.retired

    def refresh_absence(self, ch):
        if self.use_cert and ch not in self.cleared and self.cert.absent(ch):
            if ch in self.polys:
                raise RuntimeError('Certificate contradicts known uncleared source')
            if ch not in self.retired:
                self.retired.add(ch)
                self.certificate_events.append(dict(channel=ch,virtual_time_s=self.virtual,
                    negative_station_ids=sorted(self.cert.histories[ch]),remaining_cells=0))

    def measure(self,p,ch):
        key = (ch,float(p[0]),float(p[1]))
        if key in self.measured and ch not in self.cleared:
            self.cached_measurements += 1
            return self.measured[key]
        result = super().measure(p,ch)
        self.measured[key] = result
        if result['measure_result'] == 'no_signal':
            self.cert.negative(ch,p)
            self.tracking_no_signal += self.tracking
        else:
            self.cert.positive(ch,p,result.get('svd_deg'))
        self.refresh_absence(ch)
        return result

    def clear(self,p,ch):
        ok = super().clear(p,ch)
        if ok:
            self.last_clear_time = self.virtual
        else:
            self.failed_positions.setdefault(ch,[]).append(tuple(p))
            self.cert.failed_clear(ch,p)
        return ok

    def opportunistic(self):
        if self.variant not in ('opportunity','route','v3'):
            return
        point = self.pos
        candidates = []
        for ch in sorted(self.unresolved()-set(self.polys)):
            if self.cert.novelty(ch,point) >= 100 and self.cert.can_receive(ch,point):
                candidates.append(ch)
        for ch in sorted(candidates,key=lambda ch:ch != self.channel):
            if ch in self.unresolved():
                self.opportunistic_measures += 1
                self.measure(point,ch)

    def choose_measurement(self,ch,k):
        poly = self.polys[ch]
        c = enclosing_circle(poly)[0] if self.use_scoring else center(poly)
        origin,_ = self.obs[ch][-1]
        r = dist(origin,c)
        ux,uy = ((c[0]-origin[0])/max(r,1e-9),(c[1]-origin[1])/max(r,1e-9))
        forward = min(250,r*.45)
        side = min(180,max(35,r*.25))
        sign = 1 if k%2==0 else -1
        original=(origin[0]+ux*forward-uy*side*sign,origin[1]+uy*forward+ux*side*sign)
        candidates=[original]
        if self.use_scoring:
            for f in [min(140,r*.25),min(400,r*.65)]:
                for offset in [-side,side]:
                    candidates.append((origin[0]+ux*f-uy*offset,origin[1]+uy*f+ux*offset))
            # Convex combinations of positive stations are direction-safe for
            # ALL source positions/headings. Distance safety is checked below.
            positives=[p for p,_ in self.obs[ch]]
            for old in positives[:-1]:
                candidates.append(((origin[0]+old[0])/2,(origin[1]+old[1])/2))
        candidates=[p for p in candidates if (ch,float(p[0]),float(p[1])) not in self.measured]
        if not candidates:
            return None
        if not self.use_scoring:
            return candidates[0]
        # This finite representative-point score is a heuristic, never a
        # certificate. Actual clearing uses the complete observed polygon.
        representatives=poly+[c]
        def score(p):
            worst=0
            for g in representatives:
                for error in [-1.0051,1.0051]:
                    angle=math.atan2(g[1]-p[1],g[0]-p[0])+math.radians(error)
                    sub=poly
                    eps=math.radians(1.0051)
                    for a,b in [(math.sin(angle-eps),-math.cos(angle-eps)),(-math.sin(angle+eps),math.cos(angle+eps))]:
                        sub=clip(sub,a,b,a*p[0]+b*p[1])
                    if sub:
                        cc=center(sub)
                        # Upper bound about vertex mean, cheap enough for ranking.
                        worst=max(worst,max(dist(cc,v) for v in sub))
            travel=dist(self.pos,p)
            if self.next_station is not None:
                travel+=dist(p,self.next_station)-dist(self.pos,self.next_station)
            distance_risk=max(0,max(dist(p,v) for v in poly)-1000)/20
            return travel/5+5+(self.channel!=ch)+.6*worst+distance_risk
        return min(candidates,key=score)

    def localize(self,ch):
        self.tracking=True
        try:
            for k in range(7):
                if ch in self.cleared:
                    return
                poly=self.polys[ch]
                point=nearest_clear_point(poly,self.pos)
                if point is not None:
                    if not self.clear(point,ch):
                        raise RuntimeError('Certified clear failed')
                    self.opportunistic()
                    return
                c,r=enclosing_circle(poly)
                if k==1 and len(self.obs[ch])>=2 and r<120:
                    if self.clear(c,ch):
                        self.opportunistic()
                        return
                point=self.choose_measurement(ch,k)
                if point is None:
                    break
                self.measure(point,ch)
                self.opportunistic()
            if ch in self.cleared:
                return
            # Finite optical fallback; exclude a clipped cell only if every
            # vertex is inside an earlier failed-clear disk (never just center).
            poly=self.polys[ch]
            cells=[]
            for i in range(math.floor(min(p[0] for p in poly)/25),math.floor(max(p[0] for p in poly)/25)+1):
                for j in range(math.floor(min(p[1] for p in poly)/25),math.floor(max(p[1] for p in poly)/25)+1):
                    cell=poly
                    for a,b,c in [(1,0,(i+1)*25),(-1,0,-i*25),(0,1,(j+1)*25),(0,-1,-j*25)]:
                        cell=clip(cell,a,b,c)
                        if not cell:
                            break
                    if cell and not any(all(dist(v,f)<20-1e-7 for v in cell) for f in self.failed_positions.get(ch,[])):
                        cells.append(((i+.5)*25,(j+.5)*25))
            while cells:
                p=min(cells,key=lambda p:dist(self.pos,p));cells.remove(p)
                if self.clear(p,ch):
                    self.opportunistic()
                    return
            raise RuntimeError('Optical fallback exhausted')
        finally:
            self.tracking=False

    def scan(self,p):
        channels=sorted(self.unresolved(),key=lambda ch:ch!=self.channel)
        for ch in channels:
            if not self.use_cert or self.cert.can_receive(ch,p):
                self.measure(p,ch)

    def run(self):
        started=time.monotonic()
        entry=self.post('/enter')
        self.deadline=time.monotonic()+entry['remaining_real_duration_s']
        sites=search_sites(self.problem)
        total_sites=len(sites)
        visited=0;error=None
        try:
            while self.unresolved() and len(self.cleared)<16:
                pending=sorted(set(self.polys)-self.cleared)
                if not sites and not pending:
                    if not self.use_cert:
                        break
                    if self.supplemental_sites >= 60:
                        raise RuntimeError('Conservative certificate incomplete after supplemental scans')
                    ch=min(self.unresolved())
                    cells=self.cert.centers[self.cert.remaining(ch)]
                    p=min(cells,key=lambda p:dist(self.pos,p))
                    if self.problem==3:
                        extra=[tuple(p)]
                    else:
                        extra=[(p[0]+200*math.cos(k*math.tau/3),p[1]+200*math.sin(k*math.tau/3)) for k in range(3)]
                    sites.extend(extra);self.supplemental_sites+=len(extra)
                if self.use_route:
                    jobs=[(('scan',i),p) for i,p in enumerate(sites)]
                    jobs += [(('target',ch),enclosing_circle(self.polys[ch])[0]) for ch in pending]
                    route=open_route(self.pos,jobs)
                    if not route:
                        break
                    (kind,index),point=route[0]
                else:
                    if pending:
                        kind,index='target',min(pending,key=lambda ch:dist(self.pos,center(self.polys[ch])))
                    else:
                        kind,index='scan',min(range(len(sites)),key=lambda i:dist(self.pos,sites[i]))
                self.next_station=min(sites,key=lambda p:dist(self.pos,p)) if sites else None
                if kind=='target':
                    self.localize(index)
                else:
                    p=sites.pop(index);visited+=1
                    self.scan(p)
                    print(f'SITE {visited}/{total_sites} clear={len(self.cleared)} absent={len(self.retired)}',flush=True)
        except Exception as exc:
            error=repr(exc)
            print('ERROR '+error,flush=True)
        exit_result=self.post('/exit')
        complete=(not self.unresolved() or len(self.cleared)==16) if self.use_cert else len(self.cleared)==16 or (not sites and not(set(self.polys)-self.cleared))
        expected=self.movement/5+self.measures*5+self.switches+len(self.cleared)*5+self.failures*3
        summary=dict(mode='practice',strategy_version=self.variant,problem=self.problem,label=self.label,
            cleared_count=len(self.cleared),cleared_channels=sorted(self.cleared),
            certified_absent_channels=sorted(self.retired),total_sources=None,clearance_ratio=None,
            virtual_time_s=self.virtual,average_clear_time_s=self.virtual/len(self.cleared) if self.cleared else None,
            local_wall_time_s=time.monotonic()-started,movement_m=self.movement,measure_count=self.measures,
            switch_count=self.switches,failed_clear_count=self.failures,request_count=self.seq,
            visited_sites=visited,total_sites=total_sites,coverage_completed=complete and error is None,
            certificate_completed=not self.unresolved(),exit_reason=exit_result['exit_reason'],error=error,
            timing_reconciliation_error_s=self.virtual-expected,last_clear_time_s=self.last_clear_time,
            tail_confirmation_s=self.virtual-self.last_clear_time,opportunistic_measure_count=self.opportunistic_measures,
            tracking_no_signal_count=self.tracking_no_signal,cached_measurements=self.cached_measurements,
            supplemental_sites=self.supplemental_sites)
        (self.out/(self.label+'_summary.json')).write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
        if self.use_cert:
            certificate=self.cert.save();certificate['retirement_events']=self.certificate_events
            (self.out/(self.label+'_certificate.json')).write_text(json.dumps(certificate,indent=2,ensure_ascii=False),encoding='utf-8')
        self.log.close()
        print(json.dumps(summary,ensure_ascii=False),flush=True)
        return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--problem',type=int,choices=[3,4],required=True)
    parser.add_argument('--mode',choices=['practice'],required=True)
    parser.add_argument('--label',required=True)
    parser.add_argument('--variant',choices=['mec','cert','opportunity','route','v3'],default='v3')
    parser.add_argument('--out',default=str(Path(__file__).parent/'results_v3'))
    args=parser.parse_args()
    practice_preflight(args.problem,args.label)
    EvidenceRobot(args.problem,args.label,args.out,args.variant).run()
