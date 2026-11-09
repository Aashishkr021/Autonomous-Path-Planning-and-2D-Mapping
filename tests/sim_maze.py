import sys, math, time, numpy as np
from scipy import ndimage
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import auto_explore as ae, maze_world as mw
RES,OX,OY,W,H=mw.RES,mw.OX,mw.OY,mw.W,mw.H
SDF='/mnt/user-data/attachments/a4af0d9b-a86f-59ce-9548-30c562e72276/fa715600-attachment.txt'
def lidar(truth,x,y,yaw,rng,n=360,rmax=12.0):
    ang=-math.pi+np.arange(n)*(2*math.pi/n)
    ts=np.arange(0.12,rmax,0.025)
    X=x+np.outer(np.cos(ang+yaw),ts); Y=y+np.outer(np.sin(ang+yaw),ts)
    J=((X-OX)/RES).astype(int); I=((Y-OY)/RES).astype(int)
    inb=(I>=0)&(I<H)&(J>=0)&(J<W); hit=np.zeros_like(inb); hit[inb]=truth[I[inb],J[inb]]
    first=np.where(hit.any(1),hit.argmax(1),-1)
    r=np.where(first>=0,ts[np.maximum(first,0)],np.inf)
    r=r+rng.normal(0,0.01,n)
    return r,ang,first,I,J
def update_map(m,I,J,first):
    for k in range(I.shape[0]):
        f=first[k]; n=f if f>=0 else I.shape[1]
        ii,jj=I[k,:n],J[k,:n]; ok=(ii>=0)&(ii<H)&(jj>=0)&(jj<W)
        sel=m[ii[ok],jj[ok]]!=100; m[ii[ok][sel],jj[ok][sel]]=0
        if f>=0 and 0<=I[k,f]<H and 0<=J[k,f]<W: m[I[k,f],J[k,f]]=100
def run(start=(0.0,-8.0),yaw=math.pi/2,seed=0,tmax=3000,lat=0.0,speckle=0.0,dots=(),**params):
    rng=np.random.default_rng(seed); truth,_=mw.parse(SDF)
    spk=(np.random.default_rng(seed+99).random((H,W))<speckle)&~truth
    for dx,dy in dots: spk[int((dy-OY)/RES)-1:int((dy-OY)/RES)+1,int((dx-OX)/RES)-1:int((dx-OX)/RES)+1]=True
    clear=ndimage.distance_transform_edt(~truth)*RES
    m=np.full((H,W),-1,np.int8); logic=ae.Logic(**params)
    from collections import deque
    q=deque(); x,y=start; vx_a=vy_a=0.0; dt=0.1; t=0.0; traj=[]; states=[]; minclr=9.0; inside_min=9.0
    while t<tmax:
        r,ang,first,I,J=lidar(truth,x,y,yaw,rng); update_map(m,I,J,first)
        if speckle>0 or dots: m[spk&(m==0)]=100
        scan=ae.Scan(r,-math.pi,2*math.pi/360,0.12,12.0)
        cmd=logic.update(t,(x,y,yaw),scan,ae.Grid(m,RES,OX,OY))
        if cmd.state=='DONE': break
        q.append((cmd.vx,cmd.vy)); cx_,cy_=q.popleft() if len(q)>int(round(lat/dt)) else (0.0,0.0)
        a=dt/0.4; vx_a+=a*(cx_-vx_a); vy_a+=a*(cy_-vy_a)
        c,s=math.cos(yaw),math.sin(yaw)       # body -> world
        x+=(c*vx_a-s*vy_a)*dt; y+=(s*vx_a+c*vy_a)*dt; t+=dt
        ci,cj=int((y-OY)/RES),int((x-OX)/RES); minclr=min(minclr,clear[ci,cj])
        traj.append((x,y)); states.append(cmd.state)
    traj=np.array(traj); states=np.array(states)
    # coverage inside arena of space reachable with >=0.3 m clearance
    reach=ndimage.label(clear>=0.3)[0]; si,sj=int((start[1]-OY)/RES),int((start[0]-OX)/RES)
    region=reach==reach[si,sj]
    yy,xx=np.mgrid[0:H,0:W]; wx=OX+(xx+.5)*RES; wy=OY+(yy+.5)*RES
    arena=(np.abs(wx)<7.3)&(wy>-7.3)&(wy<7.3)
    cov=(m[region&arena]==0).mean()
    inside=traj[:,1]>-7.0
    cs=[-5.1,-1.7,1.7,5.1]
    rooms=[round(float(np.min(np.hypot(traj[:,0]-x,traj[:,1]-y))),2) for y in cs for x in cs]
    visited_rooms=int(sum(d<1.0 for d in rooms))
    return dict(rooms_visited=visited_rooms,room_min_dist=rooms,done=logic.state=='DONE',t=round(t),cov=round(float(cov),3),minclr=round(float(minclr),2),
        end=(round(x,2),round(y,2)),path=round(float(np.sum(np.hypot(*np.diff(traj,axis=0).T)))),
        first_inside_t=round(float(np.argmax(inside)*dt)) if inside.any() else None,
        left_arena_during_explore=bool(np.any((traj[:,1]<-7.6)&(states=='EXPLORE'))),
        traj=traj,m=m,truth=truth,states=states,reason=logic.reason)
if __name__=='__main__':
    kw={}
    for a in sys.argv[1:]:
        k,v=a.split('='); kw[k]=float(v)
    r=run(**kw)
    print({k:v for k,v in r.items() if k not in('traj','m','truth','states')})
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(13,6.5)); ext=[OX,OX+W*RES,OY,OY+H*RES]
    img=np.where(r['m']==-1,.5,np.where(r['m']==100,0,1.0))
    ax[0].imshow(img,origin='lower',extent=ext,cmap='gray'); 
    for st,col in (('ENTER','c'),('EXPLORE','r'),('SWEEP','g'),('RETURN','b'),('HOME','m')):
        mk=r['states']==st; ax[0].plot(r['traj'][mk,0],r['traj'][mk,1],'.',c=col,ms=1.5,label=st)
    ax[0].legend(); ax[0].set_title('map + trajectory'); ax[1].imshow(~r['truth'],origin='lower',extent=ext,cmap='gray')
    plt.savefig('maze_run.png',dpi=65)
