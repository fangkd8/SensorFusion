/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
      double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
      double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
      pcl::PointXYZ point;
      point.x = i+scatter*rx;
      point.y = i+scatter*ry;
      point.z = 0;

      cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
      double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
      double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
      pcl::PointXYZ point;
      point.x = 5*rx;
      point.y = 5*ry;
      point.z = 0;

      cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem (1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  // TODO: Fill in this function

  // For max iterations 

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers
  
  int iter = 0;
  int maxInliers = 0;
  int size = cloud->points.size();
  // std::cout << size << std::endl;
   while (iter < maxIterations){
    // 1. select two points.
    int fst_idx = rand()%(size);
    int snd_idx = rand()%(size);
    while (snd_idx == fst_idx){
      snd_idx = rand()%(size);
    }
    // std::cout << fst_idx << " " << snd_idx << std::endl;
    // 2. compute a line, Ax + By + C = 0;
    // int A = cloud->points[fst_idx].y - cloud->points[snd_idx].y;
    // int B = -cloud->points[fst_idx].x + cloud->points[snd_idx].x;
    // int C = cloud->points[fst_idx].x*cloud->points[snd_idx].y - 
    //         cloud->points[fst_idx].y*cloud->points[snd_idx].x;
    double y1 = cloud->points[fst_idx].y;
    double y2 = cloud->points[snd_idx].y;
    double x1 = cloud->points[fst_idx].x;
    double x2 = cloud->points[snd_idx].x;

    double A = y1 - y2;
    double B = x2 - x1;
    double C = x1*y2 - x2*y1;
    // 3. compute distances of all points.
    int inliers_count = 0;
    std::unordered_set<int> result_;
    for (int i = 0; i < size; i++){
      float dist = A * cloud->points[i].x + B * cloud->points[i].y + C;
      dist = fabs(dist) / sqrt(A*A + B*B);
      if (dist <= distanceTol){
        inliers_count++;
        result_.insert(i);
      }
    }
    // 4. save best model.
    if (inliers_count >= maxInliers){
      inliersResult = result_;
      maxInliers = inliers_count;
    }
    // 5. iter
    iter++;
  }

  return inliersResult;

}

int main ()
{

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
  

  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  std::unordered_set<int> inliers = Ransac(cloud, 50, 1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for(int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZ point = cloud->points[index];
    if(inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }


  // Render 2D point cloud with inliers and outliers
  if(inliers.size())
  {
    renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
    renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
  }
  else
  {
    renderPointCloud(viewer,cloud,"data");
  }
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }
    
}
