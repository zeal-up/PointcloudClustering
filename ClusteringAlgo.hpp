// Copyright
#pragma once

#include "FastEuclideanClustering.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <string>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;

/**
 * @brief ClusterEuclideanCluster
 * @param cloud 输入点云
 * @param clusterTolerance 聚类的距离阈值
 * @return 聚类的索引
 */
std::vector<pcl::PointIndices> ClusterEuclideanCluster(PointCloudT cloud,
                                                       float clusterTolerance) {
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(cloud->size() + 10);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}


std::vector<pcl::PointIndices> ClusterFEC(PointCloudT cloud,
                                          float clusterTolerance) {
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  float quality = 0.0;
  std::vector<pcl::PointIndices> cluster_indices;
  FastEuclideanClustering<PointT> fec;
  fec.setClusterTolerance(clusterTolerance);
  fec.setMinClusterSize(100);
  fec.setMaxClusterSize(cloud->size() + 10);
  fec.setSearchMethod(tree);
  fec.setInputCloud(cloud);
  fec.setQuality(quality);
  fec.segment(cluster_indices);

  return cluster_indices;
}