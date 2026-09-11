"""Practice-only runner. Start the correct practice module in the UI first.

Only the four documented robot endpoints are used. No simulator data is read.
Team ID is supplied via environment, never embedded in distributable source.
"""
import argparse
import json
import math
import os
import subprocess
from pathlib import Path
import time
import uuid
from urllib.request import Request, build_opener, ProxyHandler

EPS = math.radians(1.0051)  # includes rounding to 0.01 degrees


def practice_preflight(problem, label):
    """Fail closed unless the visible app is waiting in the matching practice case."""
    exe = os.environ.get('PEEKABOO_CLI', r'D:\GITHUB\PeekabooWin\publish\PeekabooWin.Cli.exe')
    result = subprocess.run([exe, 'inspect', '--window', '无线电干扰源环境模拟器',
                             '--max-depth', '30'], capture_output=True, encoding='utf-8', check=True)
    ui = json.loads(result.stdout)
    names = []
    def walk(items):
        for item in items:
            names.append(item.get('name', ''))
            walk(item.get('children', []))
    walk(ui['data']['root_elements'])
    case = label.split('_', 1)[1]
    if (f'问题{problem} 演练 测试' not in names or '等待机器狗进入' not in names
            or case not in names):
        raise RuntimeError('Practice UI/case guard failed. No robot request sent.')


def search_sites(problem):
    if problem == 3:
        # Origin plus hexagon. For r in [1000,1800] and angle <=30deg,
        # distance to nearest ring point is <=914.7m; origin covers r<=1000.
        return [(0.,0.)]+[(1400*math.cos(k*math.tau/6),1400*math.sin(k*math.tau/6)) for k in range(6)]
    # All vertices of 950m equilateral triangles intersecting the target disk.
    # Every source lies in one such triangle and is <=950m from each vertex.
    def vertex(i,j):
        return (950*(i+j*.5),950*math.sqrt(3)*j/2)
    def segment_distance(a,b):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0,min(1,-(a[0]*dx+a[1]*dy)/(dx*dx+dy*dy)))
        return math.hypot(a[0]+t*dx,a[1]+t*dy)
    sites=set()
    for i in range(-5,6):
        for j in range(-5,6):
            a,b,c,d=vertex(i,j),vertex(i+1,j),vertex(i,j+1),vertex(i+1,j+1)
            for triangle in [[a,b,c],[b,d,c]]:
                intersects=inside((0,0),triangle) or min(segment_distance(p,q) for p,q in zip(triangle,triangle[1:]+triangle[:1]))<=1800
                if intersects:
                    sites.update(triangle)
    return sorted(sites)


def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def clip(poly, a, b, c):
    out = []
    for p, q in zip(poly, poly[1:]+poly[:1]):
        fp, fq = a*p[0]+b*p[1]-c, a*q[0]+b*q[1]-c
        if fp <= 1e-8:
            out.append(p)
        if (fp < 0) != (fq < 0):
            t = fp/(fp-fq)
            out.append((p[0]+t*(q[0]-p[0]), p[1]+t*(q[1]-p[1])))
    return out


def constrain(poly, p, angle):
    lo, hi = math.radians(angle)-EPS, math.radians(angle)+EPS
    # left of lower ray and right of upper ray
    for a, b in [(math.sin(lo), -math.cos(lo)), (-math.sin(hi), math.cos(hi))]:
        poly = clip(poly, a, b, a*p[0]+b*p[1])
    # Circumscribed 64-gon: never excludes a point within reception radius 1500.
    for k in range(64):
        a, b = math.cos(k*math.tau/64), math.sin(k*math.tau/64)
        poly = clip(poly, a, b, a*p[0]+b*p[1]+1500)
    return poly


def center(poly):
    return (sum(p[0] for p in poly)/len(poly), sum(p[1] for p in poly)/len(poly))


def inside(p, poly):
    return all((q[0]-v[0])*(p[1]-v[1])-(q[1]-v[1])*(p[0]-v[0]) >= -1e-6
               for v, q in zip(poly, poly[1:]+poly[:1]))


class Robot:
    def __init__(self, problem, case_label, out):
        self.problem = problem
        self.team = os.environ['JAMMERS_ROBOT_ID']
        self.opener = build_opener(ProxyHandler({}))
        self.pos = (0., 0.)
        self.channel = 1
        self.cleared = set()
        self.polys = {}
        self.obs = {}
        self.seq = 0
        self.prefix = uuid.uuid4().hex[:12]
        self.virtual = 0
        self.movement = 0
        self.switches = 0
        self.measures = 0
        self.failures = 0
        self.deadline = float('inf')
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.label = case_label
        self.log = (self.out/(case_label+'.jsonl')).open('w', encoding='utf-8')

    def post(self, path, p=None, ch=None):
        if path != '/exit' and time.monotonic() > self.deadline-10:
            raise TimeoutError('Approaching simulator real-time deadline')
        self.seq += 1
        payload = dict(arena_id='default', robot_id=self.team, request_id=f'{self.prefix}-{self.seq}')
        if p is not None:
            payload.update(position=dict(x=float(p[0]), y=float(p[1])), channel=ch)
        raw = json.dumps(payload).encode('utf-8')
        # Retry only the exact same action and id on connection errors.
        for attempt in range(3):
            try:
                req = Request('http://127.0.0.1:2026'+path, data=raw,
                              headers={'Content-Type': 'application/json'}, method='POST')
                with self.opener.open(req, timeout=8) as response:
                    status = response.status
                    result = json.load(response)
                break
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(.3)
        safe_payload = dict(payload, robot_id='<TEAM_ID>')
        self.log.write(json.dumps(dict(path=path, request=safe_payload, http_status=status,
                                      response=result), ensure_ascii=False)+'\n')
        self.log.flush()
        if status != 200 or result.get('accepted') is not True:
            raise RuntimeError(f'Rejected action {path}: {result}')
        self.virtual = result['virtual_time_s']
        if p is not None:
            self.movement += dist(self.pos, p)
            self.pos = tuple(p)
        if path == '/measure':
            self.measures += 1
            self.switches += self.channel != ch
            self.channel = ch
        return result

    def clear(self, p, ch):
        result = self.post('/clear', p, ch)
        if result['clear_result'] == 'success':
            self.cleared.add(ch)
            print(f'CLEARED {ch:2d} | count={len(self.cleared)} | virtual={self.virtual:.2f}s', flush=True)
            return True
        self.failures += 1
        return False

    def measure(self, p, ch):
        result = self.post('/measure', p, ch)
        if result['measure_result'] == 'near':
            self.clear(p, ch)
        elif result['measure_result'] == 'direction':
            self.obs.setdefault(ch, []).append((tuple(p), result['svd_deg']))
            poly = self.polys.get(ch, [(-1800,-1800),(1800,-1800),(1800,1800),(-1800,1800)])
            self.polys[ch] = constrain(poly, p, result['svd_deg'])
            if not self.polys[ch]:
                raise RuntimeError(f'Empty feasible polygon for channel {ch}')
        return result

    def localize(self, ch):
        for k in range(7):
            if ch in self.cleared:
                return
            poly = self.polys[ch]
            c = center(poly)
            radius = max(dist(c, p) for p in poly)
            if radius <= 19:
                if self.clear(c, ch):
                    return
                raise RuntimeError('Guaranteed clear failed: check geometry/model')
            # At most one speculative optical attempt after two bearings.
            if k == 1 and len(self.obs[ch]) >= 2 and radius < 120:
                if self.clear(c, ch):
                    return
            origin, angle = self.obs[ch][-1]
            r = dist(origin, c)
            ux, uy = ((c[0]-origin[0])/max(r, 1e-9), (c[1]-origin[1])/max(r, 1e-9))
            forward = min(250, r*.45)
            side = min(180, max(35, r*.25))
            sign = 1 if k % 2 == 0 else -1
            target = (origin[0]+ux*forward-uy*side*sign, origin[1]+uy*forward+ux*side*sign)
            self.measure(target, ch)
        # Finite optical fallback covers every grid cell intersecting feasible polygon.
        # Its center is within 25/sqrt(2)<20m of any point in that cell.
        poly = self.polys[ch]
        cells = []
        for i in range(math.floor(min(p[0] for p in poly)/25), math.floor(max(p[0] for p in poly)/25)+1):
            for j in range(math.floor(min(p[1] for p in poly)/25), math.floor(max(p[1] for p in poly)/25)+1):
                cell = poly
                for a,b,c in [(1,0,(i+1)*25),(-1,0,-i*25),(0,1,(j+1)*25),(0,-1,-j*25)]:
                    cell = clip(cell,a,b,c)
                    if not cell:
                        break
                if cell:
                    cells.append(((i+.5)*25,(j+.5)*25))
        while cells:
            p = min(cells, key=lambda p:dist(self.pos,p))
            cells.remove(p)
            if self.clear(p,ch):
                return
        raise RuntimeError(f'Optical coverage exhausted without clear for {ch}')

    def run(self):
        started = time.monotonic()
        entry = self.post('/enter')
        self.deadline = time.monotonic()+entry['remaining_real_duration_s']
        sites = search_sites(self.problem)
        total_sites = len(sites)
        visited = 0
        error = None
        try:
            while sites and len(self.cleared) < 16:
                p = min(sites, key=lambda p:dist(self.pos,p))
                sites.remove(p)
                visited += 1
                detected = []
                channels = sorted(set(range(1,21))-self.cleared, key=lambda c:c != self.channel)
                for ch in channels:
                    result = self.measure(p,ch)
                    if result['measure_result'] == 'direction':
                        detected.append(ch)
                # Finish all channels at this site before leaving to localize.
                while detected:
                    ch = min(detected, key=lambda ch:dist(self.pos,center(self.polys[ch])))
                    detected.remove(ch)
                    self.localize(ch)
                print(f'SITE {visited}/{total_sites} | cleared={len(self.cleared)} | requests={self.seq}', flush=True)
        except Exception as exc:
            error = repr(exc)
            print('ERROR '+error, flush=True)
        exit_result = self.post('/exit')
        summary = dict(mode='practice', strategy_version='v2', problem=self.problem, label=self.label,
                       cleared_count=len(self.cleared), cleared_channels=sorted(self.cleared),
                       total_sources=None, clearance_ratio=None, virtual_time_s=self.virtual,
                       average_clear_time_s=self.virtual/len(self.cleared) if self.cleared else None,
                       local_wall_time_s=time.monotonic()-started,
                       movement_m=self.movement, measure_count=self.measures,
                       switch_count=self.switches, failed_clear_count=self.failures,
                       request_count=self.seq, visited_sites=visited, total_sites=total_sites,
                       coverage_completed=not sites or len(self.cleared)==16,
                       exit_reason=exit_result['exit_reason'], error=error)
        expected = self.movement/5+self.measures*5+self.switches+len(self.cleared)*5+self.failures*3
        summary['timing_reconciliation_error_s'] = self.virtual-expected
        (self.out/(self.label+'_summary.json')).write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding='utf-8')
        self.log.close()
        print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--problem', required=True, type=int, choices=[3,4])
    parser.add_argument('--mode', required=True, choices=['practice'])
    parser.add_argument('--label', required=True)
    parser.add_argument('--out', default=str(Path(__file__).parent/'results'))
    args = parser.parse_args()
    practice_preflight(args.problem, args.label)
    Robot(args.problem, args.label, args.out).run()
