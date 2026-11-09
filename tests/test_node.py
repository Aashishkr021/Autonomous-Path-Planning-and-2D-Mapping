import sys, types, math, time, numpy as np
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_explore as ae
import airmouse.node as _nd; _nd.Float32=lambda data: data
print('scan_to_base:')
r,a=ae.scan_to_base([1.0,float('inf'),float('-inf'),2.0],0.0,math.pi/2,0.0,0.0,0.0); print('  centred no-op',np.round(r,2),np.round(a,2))
r,a=ae.scan_to_base([1.0],0.0,0.1,0.2,0.0,0.0); print('  lidar 0.2 m ahead, beam forward 1.0 m -> range from centre',np.round(r,2),'(want 1.2)')
r,a=ae.scan_to_base([1.0],math.pi/2,0.1,0.2,0.0,0.0); print('  beam to the left 1.0 m -> range',np.round(r,3),'angle',np.round(a,3),'(want 1.02, 1.373)')
print('alt_speed_factor:',[round(ae.alt_speed_factor(e),2) for e in (0,0.1,0.175,0.25,0.4)])

class Mav:
    def __init__(s): s.sent=[]; s.target_system=1; s.target_component=1
    def set_mode(s,m): s.sent.append(('mode',m))
def make(phase='EXPLORE'):
    n=object.__new__(ae.ExploreNode)
    n.P=dict(ae.DEFAULTS); n.phase=phase; n.done=False; n.armed=True; n.mode='GUIDED'
    n.t=0.0; n.now=lambda: n.t
    n.logs=[]; n.get_logger=lambda: types.SimpleNamespace(info=lambda m:n.logs.append(('I',m)),warn=lambda m:n.logs.append(('W',m)),error=lambda m:n.logs.append(('E',m)))
    n.m=Mav(); n.vel=[]; n.send_vel=lambda vx,vy,vz: n.vel.append((round(vx,2),round(vy,2),round(vz,2)))
    n.scan=object(); n.grid=object(); n.t_scan=0.0; n.t_map=0.0; n.t_ok=None; n.t_hb=0.0; n.t_arm0=None; n.prearm=[]
    n.t_alt_bad=None; n.t_pubchk=-1e9; n.map_pub_alive=True; n.errors=0; n.land_cmds=0; n.t_land=0.0; n.t_low=None
    n.t_try=-1e9; n.t_takeoff=0.0; n.t0_sim=0.0; n.t0_wall=time.time(); n.stage_t={}; n.last_state=''; n.t_log=-1e9
    n.map_prefix='/tmp/nidar_test_map'; n.shown_plan=-1
    n.pose=(1.0,1.0,0.7,0.0); n.alt=0.4; n.pump=lambda: None
    n.tf_pose=lambda parent: n.pose; n.rel_alt=lambda: n.alt
    n.count_publishers=lambda topic: 1; n.publish_viz=lambda: None; n.time_pub=types.SimpleNamespace(publish=lambda m: None)
    n.save=lambda: None; n.arm=lambda: n.m.sent.append(('arm',))
    n.logic=types.SimpleNamespace(update=lambda *a,**k: ae.Cmd(0.3,0.0,'EXPLORE'),n_frontiers=3,goal=None,reason='',plan_id=0,path=[])
    n.z0=0.0
    return n
def run(n,secs,step=0.1,cb=None,fresh=True):
    k=0
    while k*step<secs and not n.done and n.phase not in ('DONE',):
        n.t=k*step
        if fresh and not getattr(n,'stale_scan',False): n.t_scan=n.t
        if fresh: n.t_map=n.t
        if cb: cb(n,n.t)
        n.loop(); k+=1
    return n
# 1 healthy: commands flow
n=run(make(),1.0); print('healthy:',n.vel[-1],'phase',n.phase)
# 2 pose lost -> hold then land after data_timeout
n=make(); n.pose=None; run(n,12.0); print('pose lost: velocities',set(n.vel),'phase',n.phase,[m for k,m in n.logs if 'Landing' in m])
# 3 heartbeat lost
n=make(); n.t_hb=-20.0; run(n,12.0); print('heartbeat lost: phase',n.phase,[m[:60] for k,m in n.logs if 'Landing' in m])
# 4 slam not running
n=make(); n.count_publishers=lambda t:0; run(n,12.0); print('SLAM missing: phase',n.phase,[m[:80] for k,m in n.logs if 'Landing' in m])
# 5 altitude too high for >3 s
n=make(); n.alt=1.5; run(n,5.0); print('alt 1.5 m: phase',n.phase,[m[:70] for k,m in n.logs if 'Landing' in m])
# 6 altitude error 0.2 -> speed scaled
n=make(); n.alt=0.2; run(n,0.5); print('alt error 0.2 -> vx',n.vel[-1],'(scaled down)')
# 7 disarmed in flight
n=make(); n.armed=False; run(n,1.0); print('disarmed in flight: phase',n.phase,'done',n.done,[m[:50] for k,m in n.logs if k=='E'])
# 8 arming never works
n=make('ARM'); n.armed=False; n.prearm=['PreArm: Compass not healthy']; run(n,70.0); print('arm failure: phase',n.phase,'done',n.done,[m[:110] for k,m in n.logs if k=='E'])
# 9 stale scan
n=make(); n.stale_scan=True; n.t_scan=-5.0; run(n,1.0); print('stale scan: ',set(n.vel))
