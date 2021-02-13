#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <string>
#include <map>
#include <stdexcept>

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

    Lidar* lidar = new Lidar(cars, 0);

    // Get cloud points from lidar.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

    // Render rays or points.
    // renderRays(viewer, lidar->position, cloud);
    // renderPointCloud(viewer, cloud, "Cloud");

    // Create point processor.
    ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> seg_res = point_processor->SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer, seg_res.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer, seg_res.second,"planeCloud",Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor->Clustering(seg_res.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

struct PntCldParams{
    float filterRes=0.1f; // Filter resolution.
    float clusterTolerance=0.4f; // Cluster tolerance.
    int minSize = 10; // Minimum size of cluster. 
    int maxSize = 1000;  // Maximum size of cluster.
    std::string to_stop="all";  // Stage to print. 
};

void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* point_processor,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    const PntCldParams& pnt_cld_params
    )
{
    float filterRes= pnt_cld_params.filterRes;
    float clusterTolerance=pnt_cld_params.clusterTolerance;
    int minSize =pnt_cld_params.minSize;
    int maxSize =pnt_cld_params.maxSize;
    std::string to_stop=pnt_cld_params.to_stop;


    if (to_stop == "ini"){
        renderPointCloud(viewer, input_cloud, "ini", Color(1, 1, 1));
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = point_processor->FilterCloud(input_cloud, filterRes, Eigen::Vector4f{-50, -6, -2, 1}, Eigen::Vector4f{50, 6, 10, 1});
    if (to_stop == "filter"){
        renderPointCloud(viewer, cloud, "filter", Color(0, 1, 1));
        return;
    }
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_res = point_processor->SegmentPlaneMy(cloud, 100, 0.2);
    if (to_stop == "seg"){
        renderPointCloud(viewer, seg_res.second, "plane", Color(0, 1, 0));
        renderPointCloud(viewer, seg_res.first, "seg", Color(0, 1, 1));
        return;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processor->Clustering(seg_res.first, clusterTolerance, minSize, maxSize);
    int clusterId = 0;
    std::vector<Color> colors = {Color(65/256.0f, 105/256.0f, 225/256.0f), Color(100/256.0f,149/256.0f,237/256.0f),
    Color(238/256.0f,130/256.0f,238/256.0f), Color(0,1,1)};
    renderPointCloud(viewer, seg_res.second, "plane", Color(0, 1, 0));
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        Box box = point_processor->BoundingBox(cluster);
        if (to_stop !="clust")
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

    PntCldParams param;
    if (argc >= 2)
        param.filterRes=std::stof(argv[1]); // Filter resolution.
    if (argc >= 3)
        param.clusterTolerance=std::stof(argv[2]); // Cluster tolerance.
    if (argc >= 4)
        param.minSize = std::stof(argv[3]); // Minimum size of cluster. 
    if (argc >=5 )
        param.maxSize = std::stof(argv[4]);  // Maximum size of cluster.
    if (argc >=6 )
        param.to_stop=argv[5];  // Stage to print. 

    // Create point processor.
    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();
   
    // Get cloud points from point cloud file.
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = point_processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        cloud = point_processor->loadPcd((*stream_iterator).string());
        cityBlock(viewer, point_processor, cloud, param); 

        stream_iterator++;
        if(stream_iterator == stream.end())
            stream_iterator = stream.begin();

        viewer->spinOnce ();
    }
}
