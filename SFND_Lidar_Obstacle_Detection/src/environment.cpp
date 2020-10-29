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
    bool renderScene = false; //是否显示高速和小车
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0.0);
    // TODO:: Create point processor

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    //renderRays(viewer, lidar->position, cloud);//显示Lider每条射线

    //renderPointCloud(viewer, cloud, "cloud");//显示白色点云

    ProcessPointClouds<pcl::PointXYZ> processor;
    //or heap
    //ProcessPointClouds<pcl::PointXYZ>* processor2 = new ProcessPointClouds<pcl::PointXYZ>();

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor.SegmentPlane(cloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obsCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segmentCloud.first, 1.0, 3, 30);
    std::cout<<cloudClusters.size()<<std::endl;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          processor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

          Box box = processor.BoundingBox(cluster);//箱体处理。。
          renderBox(viewer,box,clusterId);
          ++clusterId;

    }


}
void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
     ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
     //renderPointCloud(viewer,inputCloud,"inputCloud");//显示原始点云
     pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , Eigen::Vector4f (-15, -6, -3, 1), Eigen::Vector4f ( 20, 6, 1, 1));
     //renderPointCloud(viewer,filterCloud,"filterCloud");
     //划分平面与障碍
     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
     segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100000, 0.2);
     //renderPointCloud(viewer,segmentCloud.first,"obsCloud",Color(1,0,0));
     renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

     //划分点集
     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 30, 3000);
     std::cout<<"cloudClusters size"<<cloudClusters.size()<<std::endl;
     int clusterId = 0;
     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
     {
           std::cout << "cluster size ";
           pointProcessorI->numPoints(cluster);
           renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

           Box box = pointProcessorI->BoundingBox(cluster);//箱体处理。。
           renderBox(viewer,box,clusterId);
           ++clusterId;

     }

}
//课程stream
void CityBlock2(pcl::visualization::PCLVisualizer::Ptr& viewer,
                ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    //fillter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , Eigen::Vector4f (-15, -6, -3, 1), Eigen::Vector4f ( 20, 6, 1, 1));

    //划分平面与障碍
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
    segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100000, 0.2);
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));//显示平面

    //划分点集
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 30, 3000);
    std::cout<<"cloudClusters size"<<cloudClusters.size()<<std::endl;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessorI->numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

          Box box = pointProcessorI->BoundingBox(cluster);//箱体处理。。
          renderBox(viewer,box,clusterId);
          ++clusterId;

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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //simpleHighway(viewer);

    //CityBlock(viewer);
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce ();
//    }
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      CityBlock2(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }

}
