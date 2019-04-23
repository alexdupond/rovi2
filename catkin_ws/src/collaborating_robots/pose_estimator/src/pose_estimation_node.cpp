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
//#include "ransac_global_alignment.h"
//#include "segment_plane.h"
#include <pcl/filters/crop_box.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include "poseEstimator.hpp"

bool new_cloud_from_msg = false;
bool ready_for_new_cloud = true;
pcl::PCLPointCloud2* cloud2_from_msg = new pcl::PCLPointCloud2;

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
	Eigen::Matrix4f T_flip;
	T_flip << 	1,  0, 	0, 0,
    			0, -1, 	0, 0,
    			0,  0, -1, 0,
				0,  0,  0, 1;
							
	if(ready_for_new_cloud)											//if the program is ready for a new cloud
	{
		pcl_conversions::toPCL(*cloud_msg, *cloud2_from_msg);				//convert msg to pcl pointcloud
		pcl::fromPCLPointCloud2(*cloud2_from_msg, *cloud_from_msg);			//convert pointcloud2 to pointcloud
		pcl::transformPointCloud(*cloud_from_msg, *cloud_from_msg, T_flip);	//flip pointcloud 180 deg around x, to get the right rotation
		ROS_INFO("new Pointcloud arrived! stamp: %"PRIu64, cloud_from_msg->header.stamp);			//This is just for showing that the pointcloud gets updated
		new_cloud_from_msg = true;											//signal new cloud is ready
		ready_for_new_cloud = false;
	} 
}

Eigen::Matrix4f RPY2H(float Rz, float Ry, float Rx, float tx, float ty, float tz)
{
	Eigen::Matrix4f T_homogenus;
	T_homogenus << 	( cos(Rz) * cos(Ry) ),	( cos(Rz) * sin(Ry) * sin(Rx) - sin(Rz) * cos(Rx) ),	( cos(Rz) * sin(Ry) * cos(Rx) + sin(Rz) * sin(Rx) ),	tx,
					( sin(Rz) * cos(Ry) ),	( sin(Rz) * sin(Ry) * sin(Rx) + cos(Rz) * cos(Rx) ),	( sin(Rz) * sin(Ry) * cos(Rx) - cos(Rz) * sin(Rx) ),	ty,
					( -sin(Ry) ),			( cos(Ry) * sin(Rx) ),									( cos(Ry) * cos(Rx) ),									tz,
					0,						0,														0,														1;
	return T_homogenus;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, point_cloud_callback);
	ros::Rate loop_rate(10);	//loop rate in Hz

	
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_rotate(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_discarded (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_boxFilter_discarded (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PLYReader ply_reader;
	ply_reader.read(path_scene_yoshi, *cloud_scene_yoshi);

	//float minX = -1, maxX = -0.02, minY = 0.02, maxY = 1, minZ = 0.03, maxZ = 0.2;
	float minX = -1, maxX = 0.0, minY = 0.0, maxY = 1, minZ = 0.03, maxZ = 0.5;
	pcl::visualization::PCLVisualizer viewer("Plane segmentation result");
	viewer.addCoordinateSystem(0.3); // 0,0,0
	viewer.addCube(minX, maxX, minY, maxY, minZ, maxZ, 1.0,1.0,0, "filter_box", 0);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "filter_box");
	
	poseEstimator PE;
	
	bool first_run = true;
	bool enable_pose_estimation = false;

	char user_input;
	cout << "You have the following options:" << endl << "p  : Pose estimation" << endl << "f  : Free view" << endl;
	cin >> user_input;
	if(user_input == 'p')
	{
		enable_pose_estimation = true;
	}
	else if(user_input == 'f')
	{
		enable_pose_estimation = false;
	}
	else
	{
		cout << endl << endl << "    Huu.. You can't even type one letter. SHAME ON YOU!" << endl << endl;
	}
	
	PE.addObjectCloud(cloud_object_yoshi,"cloud_object_yoshi");
	while(ros::ok)
	{
		ros::spinOnce();		//update all ROS related stuff
		if(new_cloud_from_msg)
		{	
			PE.printObjectCloudsNames();
			
			new_cloud_from_msg = false;
			
			pcl::transformPointCloud(*cloud_from_msg, *cloud_scene_rotate, RPY2H(0.87266, 0.13089, 0.63, 0, 0, 0.33));


			pcl::PointIndices::Ptr indices_discarded_points (new PointIndices);
			pcl::CropBox<pcl::PointXYZ> boxFilter(true);								//fintering pointcloud. remove everything outside the box
			boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
			boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
			boxFilter.setInputCloud(cloud_scene_rotate);
			boxFilter.filter(*cloud_boxFilter_output);
			boxFilter.getRemovedIndices(*indices_discarded_points);

			pcl::ExtractIndices<pcl::PointXYZ> extract;							//finding the removed points, for visualisation purposes
			extract.setInputCloud(cloud_scene_rotate);
			extract.setIndices(indices_discarded_points);
			extract.setNegative(false);
			extract.filter(*cloud_boxFilter_discarded);

			boxFilter.setMin(Eigen::Vector4f(-1, -1, -1, 1.0));					//removing points that is fare away, for visualisation purposes
			boxFilter.setMax(Eigen::Vector4f(1, 1, 1, 1.0));
			boxFilter.setInputCloud(cloud_boxFilter_discarded);
			boxFilter.filter(*cloud_boxFilter_boxFilter_discarded);

			if(first_run && enable_pose_estimation)
			{
				Eigen::Matrix4f T_pose_estimation;
				T_pose_estimation = PE.get_pose_global(cloud_boxFilter_output, "cloud_object_yoshi", 5000);
				pcl::transformPointCloud(*cloud_object_yoshi, *cloud_object_yoshi, T_pose_estimation);
				T_pose_estimation = PE.get_pose_local(cloud_boxFilter_output, "cloud_object_yoshi") * T_pose_estimation;
				cout << "Final pose:" << endl << T_pose_estimation << endl;
			}
            
			viewer.removePointCloud("cloud_boxFilter_output");
			viewer.removePointCloud("cloud_boxFilter_boxFilter_discarded");
			viewer.addPointCloud<pcl::PointXYZ>(cloud_boxFilter_output, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boxFilter_output, 0, 255, 0), "cloud_boxFilter_output");
			viewer.addPointCloud<pcl::PointXYZ>(cloud_boxFilter_boxFilter_discarded, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boxFilter_boxFilter_discarded, 150, 150, 0), "cloud_boxFilter_boxFilter_discarded");
			viewer.spinOnce();
			//viewer.spin();

			ready_for_new_cloud = true;		//signal that the function is ready for a new cloud from the camera
			first_run = false;
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
