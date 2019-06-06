#pragma once
#include "header.hpp"

using namespace pcl;

//removeNan: NaN要素を点群データから除去するメソッド
//input : target(NaN要素を除去する対象の点群)
//output: cloud(除去を行った点群)
pcl::PointCloud<PointXYZ>::Ptr removeNan(pcl::PointCloud<pcl::PointXYZ>::Ptr target){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int n_point = target->points.size();
  for(int i=0;i<n_point; i++){
    pcl::PointXYZ tmp_point;
    if(std::isfinite(target->points[i].x) || std::isfinite(target->points[i].y) || std::isfinite(target->points[i].z)){
      tmp_point.x = target->points[i].x;
      tmp_point.y = target->points[i].y;
      tmp_point.z = target->points[i].z;
      cloud->points.push_back(tmp_point);
    }
  }
//  cout << "varid points:" << cloud->points.size() << endl;
  return cloud;
}

pcl::visualization::PCLVisualizer::Ptr simpleVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void
addRGBtoPointCloudWithRGB(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color){
  int n_point = cloud->points.size();
  for(int i=0; i<n_point ; i++){
    pcl::PointXYZRGB point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;

    point.r = r;
    point.g = g;
    point.b = b;
    cloud_color->points.push_back(point);
  }
}



