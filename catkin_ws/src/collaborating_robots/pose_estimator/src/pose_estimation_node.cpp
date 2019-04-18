#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <inttypes.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <iostream>
#include "ransac_global_alignment.h"
#include "segment_plane.h"


bool new_cloud_from_msg = false;
bool ready_for_new_cloud = true;
pcl::PCLPointCloud2* cloud2_from_msg = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2_from_msg);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_msg(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_mesh(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_yoshi (new pcl::PointCloud<pcl::PointXYZ>);

const std::string path_object_yoshi = "src/collaborating_robots/pose_estimator/object_yoshi.pcd";
const std::string path_scene_yoshi = "src/collaborating_robots/pose_estimator/scene_joshi7.ply";

bool kbhit()							//used for checking for terminla input, without pausing the loop
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)	//callback function for camera
{	
	Eigen::Matrix4f transformation;
	transformation << 	1,  0, 	0, 0,
    					0, -1, 	0, 0,
    					0,  0, -1, 0,
						0,  0,  0, 1;
	if(ready_for_new_cloud)											//if the program is ready for a new cloud
	{
		ROS_INFO("callback");
		pcl_conversions::toPCL(*cloud_msg, *cloud2_from_msg);		//convert msg to pcl pointcloud
		pcl::fromPCLPointCloud2(*cloud2_from_msg, *cloud_from_msg);	//convert pointcloud2 to pointcloud
		pcl::transformPointCloud(*cloud_from_msg, *cloud_from_msg, transformation);
		new_cloud_from_msg = true;									//signal new cloud is ready
		ready_for_new_cloud = false;
	} 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, point_cloud_callback);
	ros::Rate loop_rate(10);	//loop rate in Hz

	pcl::visualization::PCLVisualizer viewer("Plane segmentation result");
	pcl::PolygonMesh testMesh;

	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_object_yoshi, *cloud_object_yoshi) == -1) //* load the file
	{
    PCL_ERROR ("Couldn't read file object yoshi \n");
    return (-1);
	}
	pcl::transformPointCloud(*cloud_object_yoshi, *cloud_object_yoshi, Eigen::Matrix4f::Identity()*0.001);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmentation_scene(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmentation_plane(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_yoshi(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PLYReader ply_reader;
	
	while(ros::ok)
	{
		ros::spinOnce();		//update all ROS related stuff
		if(new_cloud_from_msg)
		{
			new_cloud_from_msg = false;
			
			ROS_INFO("stamp = %"PRIu64, cloud2_from_msg->header.stamp);		//This is just for showing that the pointcloud gets updated this works, the red line under ROS_INFO is a intelisens error, made by vscode
			
			ply_reader.read(path_scene_yoshi, *cloud_scene_yoshi);
			//manepulate cloud hereloading yoshi object as pointcloud
			removeTable(cloud_from_msg, cloud_segmentation_scene, cloud_segmentation_plane);
			//removeTable(cloud_scene_yoshi, cloud_segmentation_scene, cloud_segmentation_plane);
			// get_pose_global_allignment(cloud_from_msg, cloud_object_yoshi);
			// Show result
            //viewer.removePointCloud("outliers");
			//viewer.removePointCloud("cloud_from_msg");
			//viewer.removePointCloud("cloud_scene_yoshi");
            viewer.addPointCloud<pcl::PointXYZ>(cloud_from_msg, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_from_msg, 255, 0, 0), "cloud_from_msg");
			//viewer.addPointCloud<pcl::PointXYZ>(cloud_scene_yoshi, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene_yoshi, 0, 255, 255), "cloud_scene_yoshi");
			
			viewer.addPointCloud<pcl::PointXYZ>(cloud_segmentation_scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_segmentation_scene, 0, 0, 255), "cloud_segmentation_scene");
			viewer.addPointCloud<pcl::PointXYZ>(cloud_segmentation_plane, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_segmentation_plane, 0, 255, 255), "cloud_segmentation_plane");
			//viewer.addCoordinateSystem();
			//viewer.spinOnce();
			viewer.spin();
			ready_for_new_cloud = false;		//signal that the function is ready for a new cloud from the camera
		}
		if (kbhit())		//it there is a terminal input
		{
			viewer.close();		//close the plc viewer windows.
			break;				// break and exit program
		};

		loop_rate.sleep();		//delay
	}
	return 0;
}
