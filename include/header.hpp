#pragma once

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>

#include <chrono>
#include <armadillo>

std::string CFG_PARAM_PATH = "../cfg/parameter.toml";
double D_MAX = std::numeric_limits<double>::max();
double D_MIN = std::numeric_limits<double>::min();
double F_MAX = std::numeric_limits<float>::max();
double F_MIN = std::numeric_limits<float>::min();