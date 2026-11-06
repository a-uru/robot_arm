import cv2,numpy as np,time,json,sys,argparse
sys.path.insert(0,'src/so101_aruco')
from so101_aruco.vision import dictionary,estimate,detect,CornerHistory
from so101_aruco.transforms import from_xyz_rpy,base_object
import yaml
parser=argparse.ArgumentParser(description='Compare ArUco methods on identical raw frames; no absolute accuracy ground truth.')
parser.add_argument('--input',default='log/aruco_accuracy/raw_comparison.npz')
parser.add_argument('--output',default='log/aruco_accuracy/method_comparison.json')
args=parser.parse_args()
D=np.load(args.input);K=D['K'];dist=D['D'];images=D['images']
p=yaml.safe_load(open('src/so101_aruco/config/marker_offsets.yaml'))['aruco_detector']['ros__parameters']
base=from_xyz_rpy(p['base_to_reference.xyz_m'],p['base_to_reference.rpy_deg']);obj=from_xyz_rpy(p['marker_to_object.xyz_m'],p['marker_to_object.rpy_deg'])
result={}
for method in ['legacy','window3','median5','contour','apriltag']:
 filter=CornerHistory();hist={};positions=[];errors=[];sizes=[];times=[];counts={};angles=[]
 for frame_index,im in enumerate(images):
  begin=time.perf_counter();gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
  if method in ['legacy','window3','median5']:cs,ids,_=detect(im,dictionary('DICT_4X4_50'),5 if method=='legacy' else 3)
  else:
   par=cv2.aruco.DetectorParameters_create();par.cornerRefinementMethod=getattr(cv2.aruco,'CORNER_REFINE_'+method.upper())
   cs,ids,_=cv2.aruco.detectMarkers(gray,dictionary('DICT_4X4_50'),parameters=par)
  poses={}
  if ids is not None:
   for c,i in zip(cs,ids.flatten()):
    i=int(i)
    if i not in [0,1]:continue
    if method=='median5':c=filter.update(i,c,frame_index/30.)
    r=estimate(c,.02,K,dist,hist.get(i))
    if r is None:continue
    rv,tv,e=r;hist[i]=rv;poses[i]=from_xyz_rpy(tv.flatten(),[0,0,0]);poses[i][:3,:3]=cv2.Rodrigues(rv)[0]
    errors.append(e);sizes.append(float(np.mean(np.linalg.norm(np.roll(c.reshape(4,2),-1,axis=0)-c.reshape(4,2),axis=1))));counts[i]=counts.get(i,0)+1
  if 0 in poses and 1 in poses:positions.append(base_object(base,poses[0],poses[1],obj)[:3,3])
  times.append((time.perf_counter()-begin)*1000)
 a=np.array(positions)
 result[method]={'counts':counts,'paired':len(a),'median_edge_px':float(np.median(sizes)) if sizes else None,'rms_px':float(np.mean(errors)) if errors else None,'mean_ms':float(np.mean(times)),'std_mm':(a.std(axis=0)*1000).tolist() if len(a) else None,'range_mm':(np.ptp(a,axis=0)*1000).tolist() if len(a) else None,'mean_position_m':a.mean(axis=0).tolist() if len(a) else None}
print(json.dumps(result,indent=2));open(args.output,'w').write(json.dumps(result,indent=2)+'\n')
