"""OFFLINE fixtures only. Never reads simulator data or sends HTTP requests."""
import argparse
import contextlib
import csv
import io
import json
import math
import os
from pathlib import Path
import random
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np
from practice_robot import Robot, dist, search_sites, practice_preflight
from practice_robot_v3 import EvidenceRobot, open_route
from practice_robot_v4 import EvidenceRobotV4
from certificate_geometry import Certificates, enclosing_circle, nearest_clear_point


class GeometryTests(unittest.TestCase):
    def test_diameter_is_not_covering_circle(self):
        p=[(0,0),(38,0),(19,19*math.sqrt(3))]
        self.assertAlmostEqual(enclosing_circle(p)[1],38/math.sqrt(3))
        self.assertIsNone(nearest_clear_point(p,(0,0)))

    def test_closest_feasible_clear(self):
        p=[(-10,0),(10,0)]
        q=nearest_clear_point(p,(100,0),20)
        self.assertAlmostEqual(q[0],10)
        self.assertAlmostEqual(q[1],0)
        self.assertEqual(nearest_clear_point(p,(0,0),20),(0,0))

    def test_full_cover_certificates(self):
        for problem in [3,4]:
            c=Certificates(problem)
            for p in search_sites(problem):c.negative(1,p)
            self.assertTrue(c.absent(1))
            self.assertFalse(c.absent(2))  # physically visiting is not scanning

    def test_partial_cells_remain(self):
        c=Certificates(3)
        c.negative(1,(0,0))
        ids=np.flatnonzero((np.abs(c.centers[:,0]-987.5)<1e-7)&(np.abs(c.centers[:,1]-12.5)<1e-7))
        self.assertTrue(c.remaining(1)[ids].all())  # center in disk, corner outside

    def test_q4_one_sided_negatives_do_not_exclude(self):
        c=Certificates(4)
        for p in [(-500,-100),(-600,0),(-500,100)]:c.negative(1,p)
        near=np.max(np.abs(c.centers),axis=1)<15
        self.assertTrue(c.remaining(1)[near].all())

    def test_1000_not_maximum_reception(self):
        c=Certificates(3)
        # All positions but a square near x=1200 are safely excluded in fixture.
        c.refined[1][:]=True
        idx=np.argmin(((c.centers-np.array([1200,0]))**2).sum(axis=1))
        c.refined[1][idx]=False
        self.assertEqual(c.novelty(1,(0,0)),0)
        self.assertTrue(c.can_receive(1,(0,0)))

    def test_inferred_target_line_is_not_direction_safe(self):
        g=(100,1);normal=(0,-1);s=(0,0);p=(50,2)
        self.assertGreaterEqual(sum(normal[i]*(s[i]-g[i]) for i in range(2)),0)
        self.assertLess(sum(normal[i]*(p[i]-g[i]) for i in range(2)),0)

    def test_practice_guard_refuses_formal_and_wrong_case(self):
        def mock_ui(names):
            return type('Result',(),{'stdout':json.dumps({'data':{'root_elements':[{'name':n,'children':[]} for n in names]}})})()
        with patch('practice_robot.subprocess.run',return_value=mock_ui(['问题3 正式 测试','等待机器狗进入','CODE'])):
            with self.assertRaises(RuntimeError):practice_preflight(3,'q3_CODE')
        with patch('practice_robot.subprocess.run',return_value=mock_ui(['问题3 演练 测试','等待机器狗进入','OTHER'])):
            with self.assertRaises(RuntimeError):practice_preflight(3,'q3_CODE')


def scene(problem,seed):
    rng=random.Random(seed)
    sources={}
    for ch in rng.sample(range(1,21),rng.randint(10,16)):
        r=1800*math.sqrt(rng.random());a=rng.random()*math.tau
        heading=rng.random()*math.tau if problem==4 and rng.random()<.7 else None
        sources[ch]=(r*math.cos(a),r*math.sin(a),rng.uniform(1000,1500),heading)
    return sources


def boundary_scene(problem):
    return {i+1:(1800*math.cos(i*math.tau/10),1800*math.sin(i*math.tau/10),1000,
                 i*math.tau/10 if problem==4 else None) for i in range(10)}


class OfflineTransport:
    def post(self,path,p=None,ch=None):
        self.seq+=1
        if self.seq>5000:raise RuntimeError('Offline fixture action cap exceeded')
        result={'accepted':True,'virtual_time_s':self.virtual,'real_timestamp_ms':round(time.monotonic()*1000)}
        payload={'arena_id':'default','robot_id':'OFFLINE','request_id':str(self.seq)}
        if path=='/enter':result.update(remaining_real_duration_s=1200)
        elif path=='/exit':result.update(exit_reason='user_exit')
        else:
            payload.update(position={'x':p[0],'y':p[1]},channel=ch)
            self.movement+=dist(self.pos,p);self.virtual+=dist(self.pos,p)/5;self.pos=tuple(p)
            source=self.sources.get(ch)
            d=dist(p,source[:2]) if source else float('inf')
            if path=='/clear':
                ok=d<=20 and ch not in self.cleared
                self.virtual+=5 if ok else 3
                result['clear_result']='success' if ok else 'no_target_in_range'
                if ok:self.fixture_last_clear=self.virtual
            elif path=='/measure':
                self.switches+=self.channel!=ch;self.virtual+=5+(self.channel!=ch)
                self.channel=ch;self.measures+=1
                visible=source and ch not in self.cleared and d<=source[2] and (source[3] is None or (p[0]-source[0])*math.cos(source[3])+(p[1]-source[1])*math.sin(source[3])>=0)
                result['measure_result']='near' if visible and d<=5 else 'direction' if visible else 'no_signal'
                if result['measure_result']=='direction':
                    fixed_error=math.sin(p[0]*.071+p[1]*.137+ch*.29)*.999
                    result['svd_deg']=round((math.degrees(math.atan2(source[1]-p[1],source[0]-p[0]))+fixed_error)%360,2)%360
            else:raise AssertionError(path)
            result['virtual_time_s']=self.virtual
        self.log.write(json.dumps(dict(path=path,request=payload,http_status=200,response=result))+'\n')
        return result

    def measure(self,p,ch):
        result=super().measure(p,ch)
        if hasattr(self,'cert') and ch in self.sources and ch not in self.cleared:
            g=np.array(self.sources[ch][:2])
            containing=np.max(np.abs(self.cert.centers-g),axis=1)<=self.cert.half+1e-8
            assert containing.any()
            assert self.cert.remaining(ch)[containing].all(),('TRUE SOURCE EXCLUDED',ch,g,p,result)
            assert ch not in self.retired
        return result


class OfflineBase(OfflineTransport,Robot):pass
class OfflineEnhanced(OfflineTransport,EvidenceRobot):pass
class OfflineEnhancedV4(OfflineTransport,EvidenceRobotV4):pass


def paired_benchmark(seeds,variants,out):
    os.environ['JAMMERS_ROBOT_ID']='OFFLINE_FIXTURE'
    rows=[]
    for problem in [3,4]:
        fixtures=[(str(seed),scene(problem,seed)) for seed in range(seeds)]
        fixtures.append(('boundary',boundary_scene(problem)))
        for seed,sources in fixtures:
            for variant in variants:
                with tempfile.TemporaryDirectory() as tmp:
                    label=f'offline_q{problem}_{seed}_{variant}'
                    if variant == 'v2':
                        bot=OfflineBase(problem,label,tmp)
                    elif variant == 'v4':
                        bot=OfflineEnhancedV4(problem,label,tmp,variant)
                    else:
                        bot=OfflineEnhanced(problem,label,tmp,variant)
                    bot.sources=sources;bot.fixture_last_clear=0
                    started=time.monotonic()
                    with contextlib.redirect_stdout(io.StringIO()):bot.run()
                    summary=json.loads((Path(tmp)/(label+'_summary.json')).read_text(encoding='utf-8'))
                    assert summary['error'] is None,(label,summary)
                    assert bot.cleared==set(sources),(label,summary)
                    assert summary['coverage_completed'],(label,summary)
                    assert abs(summary['timing_reconciliation_error_s'])<.001
                    row=dict(problem=problem,seed=seed,variant=variant,count=len(sources),virtual_time_s=bot.virtual,
                             average_s=bot.virtual/len(sources),movement_m=bot.movement,measures=bot.measures,
                             tail_s=bot.virtual-bot.fixture_last_clear,wall_s=time.monotonic()-started,
                             tracking_no_signal=summary.get('tracking_no_signal_count'),all_cleared=True)
                    rows.append(row)
                    print(f'PASS Q{problem} {seed} {variant}: T={bot.virtual:.1f} tail={row["tail_s"]:.1f} real={row["wall_s"]:.2f}s',flush=True)
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    (out/'offline_paired.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    with (out/'offline_paired.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(rows)
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--seeds',type=int,default=4)
    parser.add_argument('--variants',nargs='+',default=['v2','mec','cert','route','v3'])
    parser.add_argument('--out',default=str(Path(__file__).parent/'validation_v3'))
    args=parser.parse_args()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(GeometryTests)
    assert unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful()
    paired_benchmark(args.seeds,args.variants,args.out)
