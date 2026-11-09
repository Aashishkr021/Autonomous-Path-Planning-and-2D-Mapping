import sys, multiprocessing as mp; import os; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts')); sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import newworld as nw
def job(a):
    w,sd,lat=a; kw=dict(inflate_m=0.35) if w=='new' else dict(safe_radius_m=0.35)
    return w,sd,lat,nw.go(w,sd,lat=lat,**kw)
if __name__=='__main__':
    jobs=[(w,sd,lat) for w in ('new','old') for sd in (0,1,2) for lat in (0.3,0.5)]
    with mp.Pool(2) as pool:
        for w,sd,lat,r in pool.imap(job,jobs):
            print(w,'seed',sd,'lat',lat,{k:r[k] for k in r if k in('done','t','path','minclr','rooms_entered','rooms','end','reason','view_cov')},flush=True)
