import laspy
import open3d as o3d
import argparse
import numpy as np

def center_and_normalize_points(points):
    mean_xyz = np.mean(points, axis=0)
    points[:, :3] -= mean_xyz[:3]
    # min_z = np.min(points[:, 2])
    # points[:, :2] -= mean_xyz[:2]
    # points[:, 2] -= min_z
    return points

def convert_las_to_ply(las_file_path, ply_file_path):
    # Ler o arquivo LAS
    las = laspy.read(las_file_path)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    points = center_and_normalize_points(points)

    # Criar um PointCloud com Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Salvar o PointCloud no formato PLY
    o3d.io.write_point_cloud(ply_file_path, pcd)
    print(f"Arquivo PLY salvo em {ply_file_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converter arquivo LAS para PLY")
    parser.add_argument("las_file", type=str, help="Caminho para o arquivo LAS")
    parser.add_argument("ply_file", type=str, help="Caminho para salvar o arquivo PLY")
    args = parser.parse_args()

    convert_las_to_ply(args.las_file, args.ply_file)
