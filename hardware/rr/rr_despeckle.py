import numpy as np, os, re
from scipy import ndimage

MAPS=os.path.expanduser('~/rr_maps')
SRC=os.path.join(MAPS,'corridor_clean')
DST=os.path.join(MAPS,'corridor_despeck')
MIN_KEEP=20   # occupied blobs with <= this many px are noise -> removed

# --- read yaml (simple parse) ---
ycfg={}
for line in open(SRC+'.yaml'):
    m=re.match(r'\s*([A-Za-z_]+)\s*:\s*(.+)',line)
    if m: ycfg[m.group(1)]=m.group(2).strip()
imgname=ycfg.get('image','corridor_clean.pgm').strip()
negate=int(ycfg.get('negate','0'))
occ_th=float(ycfg.get('occupied_thresh','0.65'))
print('yaml:',{k:ycfg[k] for k in ('resolution','origin','negate','occupied_thresh','free_thresh') if k in ycfg})

# --- read P5 pgm ---
path=os.path.join(MAPS, os.path.basename(imgname))
with open(path,'rb') as f:
    assert f.readline().strip()==b'P5'
    # skip comments
    def rdtok():
        t=b''
        while True:
            c=f.read(1)
            if c.startswith(b'#'):
                f.readline(); continue
            if c.isspace():
                if t: return t
                else: continue
            t+=c
    w=int(rdtok()); h=int(rdtok()); mx=int(rdtok())
    data=np.frombuffer(f.read(w*h),dtype=np.uint8).reshape(h,w).copy()
print(f'pgm {w}x{h} maxval={mx}')

# ROS convention (negate=0): occupancy=(255-p)/255 ; occupied if > occ_th -> p < 255*(1-occ_th)
if negate: occ_mask = data > (255*occ_th)
else:      occ_mask = data < (255*(1-occ_th))
print(f'occupied pixels: {occ_mask.sum()}')

lbl,n=ndimage.label(occ_mask, structure=np.ones((3,3)))
sizes=ndimage.sum(np.ones_like(lbl),lbl,range(1,n+1))
print(f'occupied components: {n}')
removed=(sizes<=MIN_KEEP).sum(); kept=(sizes>MIN_KEEP).sum()
big=sorted(sizes,reverse=True)[:6]
print(f'  <= {MIN_KEEP}px (noise, REMOVED): {removed}')
print(f'  >  {MIN_KEEP}px (walls, KEPT):    {kept}')
print(f'  largest kept sizes: {[int(x) for x in big]}')

# remove small blobs -> set to FREE (254)
small_labels=np.where(sizes<=MIN_KEEP)[0]+1
remove_mask=np.isin(lbl,small_labels)
data[remove_mask]=254
print(f'set {remove_mask.sum()} noise px -> free')

# --- write despeck pgm (P5) ---
with open(DST+'.pgm','wb') as f:
    f.write(b'P5\n%d %d\n255\n'%(w,h)); f.write(data.tobytes())
# --- write yaml (copy, swap image name) ---
out=[]
for line in open(SRC+'.yaml'):
    if re.match(r'\s*image\s*:',line): out.append('image: corridor_despeck.pgm\n')
    else: out.append(line)
open(DST+'.yaml','w').writelines(out)
print('WROTE', DST+'.pgm', 'and', DST+'.yaml')
print(f'component count {n} -> {kept} (walls only)')
