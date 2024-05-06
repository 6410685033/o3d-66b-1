import open3d as o3d

ply = "data/kota-3.ply"
pcd = o3d.io.read_point_cloud(ply)

o3d.visualization.draw_geometries([pcd])