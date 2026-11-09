import math, numpy as np
import xml.etree.ElementTree as ET
RES=0.05; OX,OY=-9.5,-11.5; W,H=int(19/RES),int(21/RES)   # x -9.5..9.5, y -11.5..9.5
def parse(sdf, z=0.7):
    truth=np.zeros((H,W),bool); rot=0
    root=ET.parse(sdf).getroot()
    def pose(e):
        p=e.find('pose')
        return [float(v) for v in p.text.split()] if p is not None and p.text else [0]*6
    for model in root.iter('model'):
        mp=pose(model)
        for link in model.findall('link'):
            lp=pose(link)
            for col in link.findall('collision'):
                b=col.find('geometry/box/size')
                if b is None: continue
                sx,sy,sz=[float(v) for v in b.text.split()]
                cp=pose(col)
                # compose translations (yaw only for model/link/col)
                def rotz(v,yaw): return (v[0]*math.cos(yaw)-v[1]*math.sin(yaw), v[0]*math.sin(yaw)+v[1]*math.cos(yaw))
                x,y=cp[0],cp[1]; yaw=cp[5]; zc=cp[2]
                dx,dy=rotz((x,y),lp[5]); x,y=dx+lp[0],dy+lp[1]; yaw+=lp[5]; zc+=lp[2]
                dx,dy=rotz((x,y),mp[5]); x,y=dx+mp[0],dy+mp[1]; yaw+=mp[5]; zc+=mp[2]
                if abs(yaw)>1e-6: rot+=1
                if not (zc-sz/2<=z<=zc+sz/2): continue
                # rasterize (axis aligned or rotated) by sampling
                xs=np.arange(-sx/2,sx/2+RES/2,RES/2); ys=np.arange(-sy/2,sy/2+RES/2,RES/2)
                gx,gy=np.meshgrid(xs,ys)
                wx=x+gx*math.cos(yaw)-gy*math.sin(yaw); wy=y+gx*math.sin(yaw)+gy*math.cos(yaw)
                j=((wx-OX)/RES).astype(int); i=((wy-OY)/RES).astype(int)
                ok=(i>=0)&(i<H)&(j>=0)&(j<W); truth[i[ok],j[ok]]=True
    return truth,rot
if __name__=='__main__':
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    t,rot=parse('/mnt/user-data/attachments/a4af0d9b-a86f-59ce-9548-30c562e72276/fa715600-attachment.txt')
    print('rotated collisions:',rot,'occupied cells:',t.sum())
    plt.figure(figsize=(8,8)); plt.imshow(~t,origin='lower',extent=[OX,OX+W*RES,OY,OY+H*RES],cmap='gray')
    plt.plot([0],[-8],'r^'); plt.grid(alpha=.3); plt.savefig('maze.png',dpi=70)
