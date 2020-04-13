#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(){
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene(){
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  int distance = 16;
  viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
  // viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);
  return viewer;
}

std::vector<float> cross(std::vector<float> v1, std::vector<float> v2){
  std::vector<float> result;
  result.push_back(v1[1]*v2[2] - v1[2]*v2[1]);
  result.push_back(v1[2]*v2[0] - v1[0]*v2[2]);
  result.push_back(v1[0]*v2[1] - v1[1]*v2[0]);
  return result;
}

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, 
                               int maxIterations, float distanceTol){
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  int maxInliers = 0;
  while (maxIterations--){
    std::unordered_set<int> inliers;

    int ind1 = rand()%cloud->points.size();
    int ind2 = rand()%cloud->points.size();
    int ind3 = rand()%cloud->points.size();
  
    float x1 = cloud->points[ind1].x;
    float y1 = cloud->points[ind1].y;
    float z1 = cloud->points[ind1].z;
    float x2 = cloud->points[ind2].x;
    float y2 = cloud->points[ind2].y;
    float z2 = cloud->points[ind2].z;
    float x3 = cloud->points[ind3].x;
    float y3 = cloud->points[ind3].y;
    float z3 = cloud->points[ind3].z;

    std::vector<float> v1 = {x2-x1, y2-y1, z2-z1};
    std::vector<float> v2 = {x3-x1, y3-y1, z3-z1};

    std::vector<float> vec = cross(v1, v2);

    float a = vec[0];
    float b = vec[1];
    float c = vec[2];
    float d = -(a*x1 + b*y1 + c*z1);
    
    int count = 0;
    for (auto i = 0; i < cloud->points.size(); i++){
      float x = cloud->points[i].x;
      float y = cloud->points[i].y;
      float z = cloud->points[i].z;

      float dist = fabs(a*x + b*y + c*z + d) / sqrt(a*a + b*b + c*c);

      if (dist <= distanceTol){
        inliers.insert(i);
        count += 1;
      }
    }

    if (count >= maxInliers){
      inliersResult = inliers;
      maxInliers = count;
    }
  }

  return inliersResult;
}
