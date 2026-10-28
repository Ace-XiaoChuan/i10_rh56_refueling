"""为本次扫描生成等尺度深度伪彩色对照图和包含全部点的离线页面。"""
from pathlib import Path
import hashlib
import json
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from matplotlib.colors import Normalize

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'tools'))
from fuel_cap_task.pcd_point_cloud import load_pcd
from export_pcd_viewer import PAGE

folder = Path(sys.argv[1]).resolve()
paths = [folder/'cloud_sdk_native.pcd', folder/'filtering/cloud_filtered.pcd', folder/'filtering/cloud_removed.pcd']
clouds = [load_pcd(path) for path in paths]
points = [cloud.xyz_raw for cloud in clouds]
if any(not np.isfinite(p).all() for p in points):
    raise ValueError('本次预览要求有限坐标，需先检查原始扫描')
quality = {'frame_id':'camera_sdk_native','length_unit':'mm',
           'projection':'3D point cloud; Z pseudocolor, not a pixel-aligned depth image',
           'raw_point_count':len(points[0]), 'filtered_point_count':len(points[1]),
           'removed_point_count':len(points[2]), 'nonfinite_count':0,
           'zero_xyz_count':int(np.all(points[0]==0,axis=1).sum()),
           'raw_bounds_min_mm':points[0].min(0).tolist(), 'raw_bounds_max_mm':points[0].max(0).tolist(),
           'removed_bounds_min_mm':points[2].min(0).tolist(), 'removed_bounds_max_mm':points[2].max(0).tolist()}
assert len(points[0])==len(points[1])+len(points[2])

# 前后两幅图使用相同坐标范围、视角和深度色标，避免缩放变化掩盖过滤效果。
lower, upper = points[0].min(0), points[0].max(0)
norm = Normalize(lower[2], upper[2])
figure = plt.figure(figsize=(16,8),facecolor='white',layout='constrained')
for i, title in enumerate([f'Raw scan: {len(points[0]):,} points',f'Filtered scene: {len(points[1]):,} points']):
    ax=figure.add_subplot(1,2,i+1,projection='3d')
    p=points[i]
    sample=p[np.linspace(0,len(p)-1,min(len(p),60000),dtype=int)]
    artist=ax.scatter(*sample.T,c=sample[:,2],cmap='viridis',norm=norm,s=.4,alpha=.85,rasterized=True)
    if i==0:
        removed=points[2][::max(1,len(points[2])//12000)]
        ax.scatter(*removed.T,color='#e64b35',s=.6,alpha=.9,label='Points inside exclusion box',rasterized=True)
        ax.legend(loc='upper left',fontsize=9)
    ax.set(xlabel='X / mm',ylabel='Y / mm',zlabel='Z / mm',title=title,
           xlim=(lower[0],upper[0]),ylim=(lower[1],upper[1]),zlim=(lower[2],upper[2]))
    ax.set_box_aspect(upper-lower)
    ax.view_init(elev=15,azim=-75)
figure.colorbar(artist,ax=figure.axes,shrink=.6,label='Camera SDK native Z / mm')
figure.suptitle('2026-10-28 | Static scan before and after fixed gun-box filtering',fontsize=15)
figure.savefig(folder/'depth_preview.png',dpi=160)
plt.close(figure)

samples=[]
labels=['原始扫描','过滤后的场景','三维框内被滤除的点']
with np.load(folder/'filtering/point_indices.npz',allow_pickle=False) as data:
    row_indices=[np.arange(len(points[0])),data['kept_source_indices'],data['removed_source_indices']]
for path,cloud,p,indices,label in zip(paths,clouds,points,row_indices,labels):
    trace={'type':'scatter3d','mode':'markers','x':p[:,0].tolist(),'y':p[:,1].tolist(),'z':p[:,2].tolist(),
           'customdata':indices.tolist(), 'hovertemplate':'原始点行号 %{customdata}<br>X %{x:.3f} mm<br>Y %{y:.3f} mm<br>Z %{z:.3f} mm<extra></extra>',
           'marker':{'size':1.5,'color':p[:,2].tolist(),'colorscale':'Viridis','colorbar':{'title':'Z / mm'}}}
    layout={'scene':{'aspectmode':'data',**{name:{'title':axis+' / mm'} for name,axis in [('xaxis','X'),('yaxis','Y'),('zaxis','Z')]}},'margin':{'l':0,'r':0,'t':40,'b':0}}
    samples.append({'label':f'{label}（{len(p):,} 点）','length_unit':'mm','frame_id':'camera_sdk_native',
                    'point_count':len(p),'source_name':path.name,'source_sha256':cloud.sha256,'traces':[trace],'layout':layout})
payload=json.dumps(samples,ensure_ascii=False,separators=(',',':'),allow_nan=False)
for char in ('<','>','&','\u2028','\u2029'):
    payload=payload.replace(char,f'\\u{ord(char):04x}')
library=re.sub(r'</script',r'<\\/script',(folder/'inspection/plotly.min.js').read_text(),flags=re.I)
page=PAGE.replace('__POINT_CLOUD_DATA__',payload,1).replace('/*__PLOTLY__*/',library,1)
page=page.replace('油箱盖三维点云','本次扫描与油枪过滤对比')
(folder/'point_cloud_viewer.html').write_text(page)
(folder/'point_cloud_quality.json').write_text(json.dumps(quality,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(quality,ensure_ascii=False,indent=2))
