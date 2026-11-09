import sys, math, numpy as np
from scipy import ndimage
import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_maze as sm
NEW='/mnt/user-data/attachments/a4af0d9b-a86f-59ce-9548-30c562e72276/42a29bf5-attachment.txt'
OLD='/mnt/user-data/attachments/a4af0d9b-a86f-59ce-9548-30c562e72276/fa715600-attachment.txt'
# interior boxes of the 4 northern rooms (x0,x1,y0,y1), shrunk by 0.4 m from the walls
BOX=[(-7.35,-3.75),(-3.75,0.0),(0.0,3.75),(3.75,7.35)]
def go(world='new',seed=0,lat=0.3,view_m=1.5,**kw):
    sm.SDF = NEW if world=='new' else OLD
    r=sm.run(seed=seed,lat=lat,tmax=kw.pop('tmax',1800),**kw)
    tr=r['traj']; st=r['states']
    out=dict(done=r['done'],t=r['t'],path=r['path'],end=r['end'],minclr=r['minclr'],cov=r['cov'])
    if world=='new':
        ent=[]
        for x0,x1 in BOX:
            ent.append(bool(np.any((tr[:,0]>x0+0.4)&(tr[:,0]<x1-0.4)&(tr[:,1]>4.05)&(tr[:,1]<6.95))))
        out['rooms_entered']=sum(ent); out['room_flags']=ent
        # fraction of free arena space within view_m of the trail (a proxy for camera coverage)
        truth=r['truth']; RES=sm.RES
        free=~truth; yy,xx=np.mgrid[0:sm.H,0:sm.W]; wx=sm.OX+(xx+.5)*RES; wy=sm.OY+(yy+.5)*RES
        arena=(np.abs(wx)<7.3)&(wy>-7.3)&(wy<7.3)&free
        reach=ndimage.label(ndimage.distance_transform_edt(~truth)*RES>=0.3)[0]
        si,sj=int((-8.0-sm.OY)/RES),int((0.0-sm.OX)/RES)
        mark=np.zeros_like(truth)
        for x,y in tr[::3]:
            mark[int((y-sm.OY)/RES),int((x-sm.OX)/RES)]=True
        near=ndimage.distance_transform_edt(~mark)*RES<=view_m
        region=arena&(reach==reach[si,sj])
        out['view_cov']=round(float((near&region).sum()/region.sum()),3)
    else:
        out['rooms']=r['rooms_visited']
    out['reason']=r.get('reason','')
    names=[]
    for s_ in st:
        if not names or names[-1][0]!=s_: names.append([str(s_),0])
        names[-1][1]+=1
    out['stages']=[(a,round(b*0.1)) for a,b in names]
    return out
if __name__=='__main__':
    for sd in (0,1):
        print('new',sd,go('new',sd,inflate_m=0.35))
        print('old',sd,go('old',sd))
