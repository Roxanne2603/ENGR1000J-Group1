from pathlib import Path
from datetime import datetime
import json
import re

out=Path(__file__).resolve().parent
summary=json.loads((out/'dry-run-summary.json').read_text(encoding='utf-8-sig'))
assert summary['preliminary_pass'],summary
reports=[]
for item in summary['tests']:
    log=(out/item['log']).read_text(encoding='utf-8-sig')
    assert item['preliminary_pass']
    assert not re.search(r'Guru Meditation|Brownout|STAGE,\d+,FAULT|; FAULT latched',log)
    raw=re.findall(r'^STAGE,(\d+),([^,\r\n]+),us=(\d+),duty=(-?\d+)/(-?\d+),drive_ms=(\d+)',log,re.M)
    events=[{'ms':int(t),'stage':s,'us':int(us),'duty':[int(l),int(r)],'drive_ms':int(ms)} for t,s,us,l,r,ms in raw]
    names=[e['stage'] for e in events]
    traces=[{'leg':int(i),'begin':int(b),'end':int(e),'ms':int(ms)} for i,b,e,ms in re.findall(r'^DRIVE_APPLIED,(\d+),begin=(\d+),end=(\d+),ms=(\d+)',log,re.M)]
    duration=3000 if item['test'] in ('full-3s','abort-countdown') else 1000
    assert events and all(e['drive_ms']==duration for e in events),events
    for trace in traces:
        assert trace['end']-trace['begin']==trace['ms'] and 0<=trace['ms']<=duration+15
    record={'test':item['test'],'passed':True,'drive_ms':duration,'stage_order':names,'drive_apply_intervals':traces}
    if item['test'].startswith('full-'):
        expected=['PRE_CENTER','COUNTDOWN','STRAIGHT','BRAKE_STRAIGHT','STEER_LEFT','LEFT_ARC','BRAKE_LEFT',
                  'STEER_RIGHT','RIGHT_ARC','BRAKE_RIGHT','RETURN_CENTER','DONE']
        assert names==expected,names
        assert events[0]['us']==events[1]['us']==events[-1]['us']==1375
        assert events[1]['ms']-events[0]['ms']>=1000
        assert events[2]['ms']-events[1]['ms']>=3000
        drive=[e for e in events if e['stage'] in ('STRAIGHT','LEFT_ARC','RIGHT_ARC')]
        assert [e['us'] for e in drive]==[1375,975,1775]
        assert all(e['duty']==[50,-50] for e in drive)
        assert all(e['duty']==[0,0] for e in events if e not in drive)
        assert [t['leg'] for t in traces]==[1,2,3]
        assert all(duration-25<=t['ms']<=duration+15 for t in traces),traces
        requested=[int(v) for v in re.findall(r'^LEG_END,\d+,[^,]+,request_ms=(\d+)',log,re.M)]
        assert len(requested)==3 and all(duration<=v<=duration+40 for v in requested)
        for a,b in ((3,4),(6,7),(9,10)):
            assert events[b]['ms']-events[a]['ms']>=2000
        rows=re.findall(r'^G,(\d+),([^,]+),(-?\d+),(-?\d+),(\d+),',log,re.M)
        for phase,descending in (('STEER_LEFT',True),('STEER_RIGHT',False),('RETURN_CENTER',True)):
            part=[r for r in rows if r[1]==phase]
            assert part and all(int(r[2])==int(r[3])==0 for r in part)
            values=[int(r[4]) for r in part]
            assert values==sorted(values,reverse=descending)
            assert all(975<=v<=1775 for v in values)
        home=[r for r in rows if r[1]=='RETURN_CENTER' and int(r[4])==1375]
        assert home and events[-1]['ms']-int(home[0][0])>=1350
        # Faster command sweep: same400us span used to take~3.64s, cross800~6.84s.
        left_ms=events[5]['ms']-events[4]['ms']
        right_ms=events[8]['ms']-events[7]['ms']
        assert left_ms<3000 and right_ms<5000,(left_ms,right_ms)
        assert 'START refused: busy or latched stop/fault.' in log
        record.update(pre_center_ms=events[1]['ms']-events[0]['ms'],left_steer_phase_ms=left_ms,
                      right_steer_phase_ms=right_ms,home_phase_ms=events[-1]['ms']-events[-2]['ms'])
    else:
        assert 'USER_STOP; STOPPED latched' in log and 'START refused: busy or latched stop/fault.' in log
        assert 'LEFT_ARC' not in names and 'RIGHT_ARC' not in names and 'DONE' not in names
        expected={'abort-countdown':['PRE_CENTER','COUNTDOWN'],
                  'abort-drive':['PRE_CENTER','COUNTDOWN','STRAIGHT'],
                  'abort-steer':['PRE_CENTER','COUNTDOWN','STRAIGHT','BRAKE_STRAIGHT','STEER_LEFT']}
        assert names==expected[item['test']],names
        if item['test']=='abort-countdown': assert not traces
    reports.append(record)
idle=(out/'startup-final-idle.log').read_text(encoding='utf-8-sig')
assert 'STATUS stage=IDLE' in idle and 'servo_applied=0 motor_raw=0/0/0/0' in idle
result={'analyzed_at':datetime.now().astimezone().isoformat(),'all_passed':True,'tests':reports,
        'restored_idle_outputs_off':True,'scope':'Disconnected actuator power; software timing and commands only'}
(out/'software-checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
