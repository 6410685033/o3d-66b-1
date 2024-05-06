import open3d as o3d

# Step 1: Read point cloud data
print("Start processing...")
pcd = o3d.io.read_point_cloud("data/kota_circuit3.ply")

# Step 2: Downsample the point cloud
print("Downsampling...")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)

# Step 3: Estimate normals
print("Estimating normals...")
downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Step 4: Write preprocessed point cloud data
print("Writing preprocessed data...")
o3d.io.write_point_cloud("data/preprocessed_circuit.ply", downpcd, write_ascii=True) 

print("Preprocessing completed successfully!")
