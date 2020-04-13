#include "ransac.h"
#include "kdtree.h"

void Proximity(const std::vector<std::vector<float>>& points, KdTree* tree,
               float distanceTol, std::vector<int>& cluster, 
               std::vector<bool>& processed, int idx){
  if (processed[idx] == false){
    processed[idx] = true;
    cluster.push_back(idx);
    std::vector<int> nearby = tree->search(points[idx], distanceTol);
    for (int i = 0; i < nearby.size(); i++){
      Proximity(points, tree, distanceTol, cluster, processed, nearby[i]);
    }
  }
}

std::vector<std::vector<int>> euclideanCluster(
  const std::vector<std::vector<float>>& points, 
  KdTree* tree, float distanceTol, int minSize, int maxSize){

  // TODO: Fill out this function to return list of indices for each cluster

  // 1. create empty list of clusters.
  std::vector<std::vector<int>> clusters;
  // 2. mark all points as un-processed.
  std::vector<bool> processed(points.size(), false);

  for (int i = 0; i < points.size(); i++){
    if (processed[i] == false){
      std::vector<int> cluster;
      Proximity(points, tree, distanceTol, cluster, processed, i);
      if (cluster.size() >= minSize && cluster.size() <= maxSize)
        clusters.push_back(cluster);
    }
  }
 
  return clusters;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LoadData3D(){
  ProcessPointClouds<PointT>* pointProcessorI = new ProcessPointClouds<PointT>();
  std::string file = "../../../sensors/data/pcd/data_1/0000000000.pcd";
  typename pcl::PointCloud<PointT>::Ptr rawcloud = pointProcessorI->loadPcd(file);

  typename pcl::PointCloud<PointT>::Ptr cloud = pointProcessorI->FilterCloud(
    rawcloud, 0.1f, Eigen::Vector4f(-0, -20, -5, 1), Eigen::Vector4f(5, 50, 7.5, 1));
  return cloud;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LoadData3D(std::string file, 
                      ProcessPointClouds<PointT>* pointProcessorI){

  typename pcl::PointCloud<PointT>::Ptr rawcloud = pointProcessorI->loadPcd(file);

  typename pcl::PointCloud<PointT>::Ptr cloud = pointProcessorI->FilterCloud(
    rawcloud, 0.15f, Eigen::Vector4f(-0, -20, -5, 1), Eigen::Vector4f(5, 30, 7.5, 1));
  return cloud;
}

int main(){
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../../../sensors/data/pcd/data_1/");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  
  while (!viewer->wasStopped ()){
    cloud->clear();
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    // LoadData3D<pcl::PointXYZI>();
    cloud = LoadData3D<pcl::PointXYZI>((*streamIterator).string(), pointProcessorI);
    // cityBlock(viewer, pointProcessorI, inputCloudI);
  
    std::unordered_set<int> inliers = Ransac<pcl::PointXYZI>(cloud, 100, 0.2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < cloud->points.size(); index++){
      pcl::PointXYZI point = cloud->points[index];
      if(inliers.count(index))
        cloudInliers->points.push_back(point);
      else
        cloudOutliers->points.push_back(point);
    }

    // cloudInliers is ground, cloudOutliers is obstacles.
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloudOutliers->points.size(); i++){
      float x = cloudOutliers->points[i].x;
      float y = cloudOutliers->points[i].y;
      float z = cloudOutliers->points[i].z;
      std::vector<float> pt = {x, y, z};
      tree->insert(pt, i);
      points.push_back(pt);
    }

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 0.35, 20, 2500);

    int clusterId = 0;
    Color color = Color(1,1,1);

    for(std::vector<int> cluster : clusters){
      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pts(new pcl::PointCloud<pcl::PointXYZI>());
      for(int indice: cluster){
        pts->points.push_back(cloudOutliers->points[indice]);
        clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
      }
      renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),color);
      Box box = pointProcessorI->BoundingBox(pts);
      renderBox(viewer, box, clusterId);
      ++clusterId;
    }

    if(inliers.size()){
      renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
      renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,1,1));
    }
    else{
      renderPointCloud(viewer,cloud,"data");
    }

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    // sleep(2);
    viewer->spinOnce ();
  }

  return 0;
}