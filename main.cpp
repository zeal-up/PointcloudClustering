// Copyright
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

#include "ClusteringAlgo.hpp"

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " pointcloud.ply" << std::endl;
    return -1;
  }

  // 读取点云
  PointCloudT cloud(new PointCloud);
  if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
    std::cout << "Load pointcloud " << argv[1] << " failed." << std::endl;
    return -1;
  }

  // start pcl::EuclideanClusterExtraction
  std::cout << "Start pcl::EuclideanClusterExtraction ----------------------" << std::endl;
  pcl::console::TicToc tt;
  tt.tic();
  // 聚类
  auto clusters = ClusterEuclideanCluster(cloud, 0.1);
  tt.toc();
  std::cout << "Found " << clusters.size() << " clusters." << std::endl;
  // extract pointcloud from clusters and assign different color to each class
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int cluster_id = 1;
  for (auto &cluster : clusters) {
    for (auto &index : cluster.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud->points[index].x;
      point.y = cloud->points[index].y;
      point.z = cloud->points[index].z;
      point.r = 255 * (cluster_id % 2);
      point.g = 255 * (cluster_id % 3);
      point.b = 255 * (cluster_id % 5);
      colored_cloud->push_back(point);
    }
    cluster_id++;
  }
  // save colored pointcloud
  pcl::io::savePCDFileBinary("euclidean_cluster_colored.pcd", *colored_cloud);
  std::cout << "Saved " << colored_cloud->size()
            << " data points to euclidean_cluster_colored.ply." << std::endl;
  std::cout << "Done in " << tt.toc() << " ms." << std::endl;

  std::cout << "Done pcl::EuclideanClusterExtraction ----------------------" << std::endl;

  // start FastEuclideanClustering
  std::cout << "Start FastEuclideanClustering ----------------------" << std::endl;
  tt.tic();
  clusters.clear();
  clusters = ClusterFEC(cloud, 0.1);
  tt.toc();
  std::cout << "Found " << clusters.size() << " clusters." << std::endl;
  // extract pointcloud from clusters and assign different color to each class
  colored_cloud->clear();
  cluster_id = 1;
  for (auto &cluster : clusters) {
    for (auto &index : cluster.indices) {
      pcl::PointXYZRGB point;
      point.x = cloud->points[index].x;
      point.y = cloud->points[index].y;
      point.z = cloud->points[index].z;
      point.r = 255 * (cluster_id % 2);
      point.g = 255 * (cluster_id % 3);
      point.b = 255 * (cluster_id % 5);
      colored_cloud->push_back(point);
    }
    cluster_id++;
  }
  // save colored pointcloud
  pcl::io::savePCDFileBinary("fec_cluster_colored.pcd", *colored_cloud);
  std::cout << "Saved " << colored_cloud->size()
            << " data points to fec_cluster_colored.ply." << std::endl;
  std::cout << "Done in " << tt.toc() << " ms." << std::endl;

  std::cout << "Done FastEuclideanCluster ----------------------" << std::endl;
  return 0;
}