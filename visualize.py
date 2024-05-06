import open3d as o3d

ply = "data/kota_circuit3.ply"
# ply = "data/kota.ply"

pcd = o3d.io.read_point_cloud(ply)
print(pcd)
o3d.visualization.draw_geometries([pcd])