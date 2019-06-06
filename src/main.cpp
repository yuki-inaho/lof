#include "header.hpp"
#include "utils.hpp"
#include "ParameterManager.hpp"
#include "lof.hpp"

using namespace pcl;

int
main (int argc, char** argv)
{
    ParameterManager cfg_param(CFG_PARAM_PATH);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    int sor_K = cfg_param.ReadIntData("Param", "K");
    float sor_sigma = cfg_param.ReadFloatData("Param", "sigma");
    float threshold = cfg_param.ReadFloatData("Param", "threshold");
    std::string pcd_name = cfg_param.ReadStringData("Param", "pcd_name");

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_name, *cloud) == -1) 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    cloud = removeNan(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_noise (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_noiseless (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointIndices::Ptr lof_inliers(new pcl::PointIndices);
    pcl::PointIndices::Ptr lof_outliers(new pcl::PointIndices);
    lof(cloud, threshold, lof_inliers, lof_outliers);

/*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointIndices::Ptr sor_inliers(new pcl::PointIndices);
    std::vector<int> _sor_inliers;
    sor.setInputCloud (cloud);
    sor.setMeanK (sor_K);
    sor.setStddevMulThresh (sor_sigma);
    sor.filter (_sor_inliers);
    sor_inliers->indices = _sor_inliers;
*/

    pcl::ExtractIndices<pcl::PointXYZ> eifilter; 
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (lof_inliers);
    eifilter.setNegative (false);
    eifilter.filter (*_cloud_noiseless);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_noise (new pcl::PointCloud<pcl::PointXYZ>);
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (lof_inliers);
    eifilter.setNegative (true);
    eifilter.filter (*_cloud_noise);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inlier_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outlier_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    addRGBtoPointCloudWithRGB(_cloud_noiseless, 0, 0, 255, cloud_inlier_rgb);
    addRGBtoPointCloudWithRGB(_cloud_noise, 255, 0, 0, cloud_outlier_rgb);

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis();

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> inlier_rgb(cloud_inlier_rgb);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_inlier_rgb, inlier_rgb, "inlier");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inlier");  
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> outlier_rgb(cloud_outlier_rgb);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_outlier_rgb, outlier_rgb, "outlier");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "outlier");  

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}