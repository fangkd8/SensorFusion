/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* velodyne = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = velodyne->scan();

    ProcessPointClouds<pcl::PointXYZ> processor = ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segPts = processor.SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer, segPts.first, "ground", Color(1, 0, 0));
    // renderPointCloud(viewer, segPts.second, "obstacles", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = processor.Clustering(segPts.second, 1, 3, 30);
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (int i = 0; i < clusters.size(); i++){
      renderPointCloud(viewer, clusters[i], std::to_string(i), colors[i]);
      Box box = processor.BoundingBox(clusters[i]);
      renderBox(viewer, box, i);
    }
}

void cityblock(pcl::visualization::PCLVisualizer::Ptr& viewer){
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(50, 7.5, 20, 1));
  std::cout << "Filtered point cloud size is " << Cloud->points.size() << std::endl;
  // renderPointCloud(viewer, Cloud, "Cloud");

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segPts = pointProcessorI->SegmentPlane(Cloud, 100, 0.2);
  renderPointCloud(viewer, segPts.first, "ground", Color(0, 1, 0));
  renderPointCloud(viewer, segPts.second, "obstacles", Color(1, 0, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segPts.second, 0.5, 50, 1500);
  // std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  srand(time(NULL));
  for (int i = 0; i < clusters.size(); i++){
    Color c(rand(), rand(), rand());
    renderPointCloud(viewer, clusters[i], std::to_string(i), c);
    Box box = pointProcessorI->BoundingBox(clusters[i]);
    renderBox(viewer, box, i);
  }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, 
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, 
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
  pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(50, 7.5, 20, 1));
  std::cout << "Filtered point cloud size is " << Cloud->points.size() << std::endl;
  // renderPointCloud(viewer, Cloud, "Cloud");

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segPts = pointProcessorI->SegmentPlane(Cloud, 100, 0.2);
  renderPointCloud(viewer, segPts.first, "ground", Color(0, 1, 0));
  renderPointCloud(viewer, segPts.second, "obstacles", Color(1, 0, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segPts.second, 0.4, 50, 2500);
  // std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  srand(time(NULL));
  for (int i = 0; i < clusters.size(); i++){
    Color c(rand(), rand(), rand());
    renderPointCloud(viewer, clusters[i], std::to_string(i), c);
    Box box = pointProcessorI->BoundingBox(clusters[i]);
    renderBox(viewer, box, i);
  }
}

int main (int argc, char** argv){
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // cityblock(viewer);

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped ()){
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce ();
  }
}