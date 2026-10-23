"""检查像素定位、体素归属和虚拟深度的独立表面位置；仅用于实验评价。"""
import base64
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from fuel_cap_task.mujoco_scene import MujocoCapScene

root = Path(sys.argv[1])
with np.load(root/"point_cloud/point_cloud.npz") as arrays:
    points, rgb, uv = arrays["world_xyz_m"], arrays["rgb"], arrays["pixel_uv"]
meta = json.loads((root/"observation/observation.json").read_text())
color = rgb.astype(np.int16)
blue = (color[:,2]-color[:,0]>50) & (color[:,2]-color[:,1]>25)
blue_ids = np.flatnonzero(blue)
clicked_point = int(blue_ids[len(blue_ids)//2])
pixel = uv[clicked_point]
rows = []
for name in ["voxels_2mm", "voxels_5mm"]:
    folder = root/name
    with np.load(folder/"voxels.npz") as arrays:
        ids, inverse, counts = arrays["indices_xyz"], arrays["point_to_voxel"], arrays["point_counts"]
        origin, size = arrays["origin_world_m"], float(arrays["voxel_size_m"])
        lower = origin + ids[inverse]*size
        inside = bool(np.all(points>=lower-1e-12) and np.all(points<lower+size+1e-12))
        tally = bool(np.array_equal(np.bincount(inverse,minlength=len(ids)),counts))
        page = (folder/"index.html").read_text()
        marker = "\nconst data = "
        start = page.rindex(marker)+len(marker)
        end = page.index(";\nconst centers = ",start)
        data = json.loads(page[start:end])
        del page
        lookup = np.frombuffer(base64.b64decode(data["pixel_voxel_base64"]),dtype="<i4").reshape(
            data["image_height"],data["image_width"])
        pixel_match = bool(np.array_equal(lookup[uv[:,1],uv[:,0]],inverse))
        invalid_match = int((lookup==-1).sum()) == lookup.size-len(points)
        row = {"case":name,"input_point_count":len(points),"voxel_count":len(ids),
            "all_points_inside_assigned_cells":inside,"membership_counts_match":tally,
            "point_count_conserved":int(counts.sum())==len(points),
            "photo_lookup_matches_all_valid_pixels":pixel_match,
            "invalid_pixels_preserved":bool(invalid_match),
            "blue_surface_voxel_count_color_diagnostic_only":len(np.unique(inverse[blue])),
            "blue_photo_click_uv":pixel.tolist(),
            "blue_photo_click_voxel_id":int(lookup[pixel[1],pixel[0]])}
        assert inside and tally and pixel_match and invalid_match and row["point_count_conserved"]
        assert row["blue_photo_click_voxel_id"] == int(inverse[clicked_point])
        (folder/"validation.json").write_text(json.dumps(row,indent=2)+"\n")
        rows.append(row)
        del data
bounds = [uv[blue].min(axis=0).tolist(),uv[blue].max(axis=0).tolist()]
width,height = meta["image_size_wh"]
interior = bool(bounds[0][0]>0 and bounds[0][1]>0 and bounds[1][0]<width-1 and bounds[1][1]<height-1)
assert interior
scene = MujocoCapScene()
opt = mujoco.MjvOption()
opt.geomgroup[0] = opt.geomgroup[3] = 0
origin = np.asarray(meta["world_from_optical"])[:3,3]
errors, misses = [], 0
for index in np.linspace(0,len(points)-1,1024,dtype=int):
    delta = points[index]-origin
    length = np.linalg.norm(delta)
    hit = mujoco.mj_ray(scene.model,scene.data,origin,delta/length,opt.geomgroup,True,-1,
                       np.zeros(1,dtype=np.int32))
    if hit<0:
        misses += 1
    else:
        errors.append(abs(hit-length))
ray = {"mode":"independent_render_geometry_ray_check","sample_count":1024,"ray_misses":misses,
       "distance_error_median_m":float(np.median(errors)),"distance_error_max_m":float(max(errors)),
       "numeric_surface_check_tolerance_m":1e-4,"offscreen_samples":meta["capture_settings"]["offscreen_samples"],
       "evaluation_only":True,"ray_geometry_used_to_generate_points":False}
(root/"point_cloud/evaluation_rays.json").write_text(json.dumps(ray,indent=2)+"\n")
assert misses==0 and max(errors)<ray["numeric_surface_check_tolerance_m"]
report = {"mode":"detail_voxel_observation_and_pixel_correspondence_check",
    "color_diagnostic_not_semantic_detection":True,"blue_surface_point_count":int(blue.sum()),
    "blue_pixel_bounds_uv":bounds,"blue_surface_does_not_touch_image_border":interior,
    "cases":rows,"independent_ray_check":ray}
(root/"validation.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report))
