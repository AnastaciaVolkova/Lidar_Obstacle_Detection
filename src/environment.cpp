/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <string>
#include <map>
#include <stdexcept>

struct CmdLnPrms{
    enum class PointProc{kSimpleHighway, kCityBlock};
    static std::map<std::string, PointProc> point_proc_map;
    PointProc point_proc;
};

void PrintHelp(char* name){
    std::cout << "Program usage:\n" <<
    name << " [--param value]" <<
    "--point_processor simpleHighway|cityBlock" << std::endl;
}

int CommandLineParser(int argc, char** argv, CmdLnPrms& cmdLnPrms){
    int i = 1;
    if (argc == 1) return 0;
    if ((argc-1)%2 != 0) {
        std::cout << "Wrong parameters" << std::endl;
        PrintHelp(argv[0]);
        return -1;
    }
    while (i < argc){
        if (std::string("--point_processor") == std::string(argv[i]))
            try{
                cmdLnPrms.point_proc = CmdLnPrms::point_proc_map.at(argv[i+1]);
            } catch(std::out_of_range exception){
                std::cout << "Wrong parameter value" << std::endl;
                PrintHelp(argv[1]);
                return -1;
            }
            i+=2;
    }
    return 0;
}

std::map<std::string, CmdLnPrms::PointProc> CmdLnPrms::point_proc_map =
    std::map<std::string, CmdLnPrms::PointProc>{
        {"simpleHighway", CmdLnPrms::PointProc::kSimpleHighway},
        {"cityBlock", CmdLnPrms::PointProc::kCityBlock}
        };

CmdLnPrms cl_prms;

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    //bool renderScene = false;
    //std::vector<Car> cars = initHighway(renderScene, viewer);

    //Lidar* lidar = new Lidar(cars, 0);

    // Create point processor.
    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();

    // Get cloud points from point cloud file.
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = point_processor->loadPcd("./src/sensors/data/pcd/data_1/0000000000.pcd");

    cloud = point_processor->FilterCloud(cloud, 0.1f, Eigen::Vector4f{-20, -20, -2, 1}, Eigen::Vector4f{20, 20, 10, 1});
    renderPointCloud(viewer, cloud, "input cloud");
    // Render rays or points.
    // renderRays(viewer, lidar->position, cloud);

    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> seg_res = point_processor->SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer, seg_res.first,"obstCloud",Color(1,0,0));
    // renderPointCloud(viewer, seg_res.second,"planeCloud",Color(0,1,0));
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processor->Clustering(seg_res.first, 1.0, 3, 30);
    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    /*for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }*/
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
    std::map<CmdLnPrms::PointProc, void(*)(pcl::visualization::PCLVisualizer::Ptr& )> point_proc {
        {CmdLnPrms::PointProc::kSimpleHighway, simpleHighway},
        {CmdLnPrms::PointProc::kCityBlock, cityBlock}
    };
    CommandLineParser(argc, argv, cl_prms);
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    point_proc[cl_prms.point_proc](viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}
