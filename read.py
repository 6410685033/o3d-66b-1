
import open3d as o3d
import numpy as np

print("..1")
pcd = o3d.io.read_point_cloud("data/kota_circuit3.ply")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
print(downpcd)

o3d.io.write_point_cloud("data/kota_circuit4.ply", downpcd, write_ascii=True) 
