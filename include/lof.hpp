#include "header.hpp"

using namespace pcl;

float
calc_reachability(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int &idx_base, const int &idx_nn, const std::vector<float> &knn_dist){
    float k_dist = knn_dist[idx_base];
    pcl::PointXYZ point1 = cloud->points[idx_base];
    pcl::PointXYZ point2 = cloud->points[idx_nn]; 
    float nn_dist = std::sqrt(point1.x*point2.x + point1.y*point2.y + point1.z*point2.z);

    return nn_dist;
}

void
lof(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float threshold, pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr outliers){
//lof(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float threshold){    
    int n_points = cloud->points.size();

    std::vector<std::vector<int>> knn_vec;
    std::vector<float> knn_dist;    
    std::vector<std::vector<int>> rnn_vec;


    for(int i=0;i<n_points;i++){
        std::vector<int> _knn_indices;
        std::vector<int> _rnn_indices;

        knn_vec.push_back(_knn_indices);
        knn_dist.push_back(0);
        rnn_vec.push_back(_rnn_indices);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    int K = 30;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    int threads_ = 8;

    //pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);    
    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    #pragma omp parallel for shared (knn_vec, rnn_vec) private (pointIdxNKNSearch, pointNKNSquaredDistance) num_threads(threads_)
    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx)
    {
        if ( kdtree.nearestKSearch (cloud->points[idx], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for(const int nn_idx : pointIdxNKNSearch){
                knn_vec[idx].push_back(nn_idx);
                rnn_vec[nn_idx].push_back(idx);
            }
            knn_dist[idx] = pointNKNSquaredDistance[K-1];
        }else{
            knn_dist[idx] = F_MAX;
        }
    }

    std::vector<float> lrd_vec;    
    std::vector<float> lof_vec;    
    for(int i=0;i<n_points;i++){
        lrd_vec.push_back(0);
        lof_vec.push_back(0);
    }

    #pragma omp parallel for shared (lrd_vec) num_threads(threads_)
    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx)
    { 
        std::vector<int> neighbor_ind = knn_vec[idx];
        float sum_reachability = 0;
        for(const int nn_idx: neighbor_ind) {
            sum_reachability += calc_reachability(cloud, idx, nn_idx, knn_dist);
        }
        lrd_vec[idx] = neighbor_ind.size()/sum_reachability;
    }

    #pragma omp parallel for shared (lof_vec) num_threads(threads_)
    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx)
    { 
        std::vector<int> neighbor_ind = knn_vec[idx];
        float sum_lrd = 0;
        if(neighbor_ind.size() > 0){
            for(const int nn_idx: neighbor_ind) {
                sum_lrd += lrd_vec[nn_idx];
            }
            lof_vec[idx]  = sum_lrd/neighbor_ind.size()/lrd_vec[idx];
        }else{
            lof_vec[idx]  =  F_MAX;
        }
    }

    inliers->indices.clear();
    outliers->indices.clear();
    for (int idx = 0; idx < static_cast<int> (cloud->points.size ()); ++idx)
    { 
        //cout << lof_vec[idx] << endl;
        if(lof_vec[idx] <= threshold){
            inliers->indices.push_back(idx);
        }else{
            outliers->indices.push_back(idx);
        }
    }

    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
    cout << "elapsed:" << elapsed <<  endl;
}