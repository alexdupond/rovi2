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
#include <pcl/filters/voxel_grid.h>
#include "poseEstimator.hpp"
#include "cloudManipulator.hpp"
//#include "std_msgs/Float64MultiArray.h"
#include <eigen_conversions/eigen_msg.h>

bool new_cloud_from_msg = false;
bool ready_for_new_cloud = true;
pcl::PCLPointCloud2::Ptr cloud2_from_msg (new pcl::PCLPointCloud2 ());
//pcl::PCLPointCloud2* cloud2_from_msg = new pcl::PCLPointCloud2;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_msg(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_mesh(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_yoshi (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object_yoda (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_from_msg_plane (new pcl::PointCloud<pcl::PointXYZ>);

const std::string path_object_yoshi = "src/collaborating_robots/pose_estimator/object_yoshi.pcd";
const std::string path_object_yoda = "src/collaborating_robots/pose_estimator/object_yoda.pcd";
//const std::string path_scene_yoshi = "src/collaborating_robots/pose_estimator/scene_joshi7.ply";
const float leaf_size = 0.005f; // leaf size for downscaling

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
                pcl::PCLPointCloud2::Ptr input_cloud (new pcl::PCLPointCloud2 ());
                //pcl::fromROSMsg(*cloud_msg, *cloud_from_msg);                                   //convert msg to pointcloud

                pcl_conversions::toPCL(*cloud_msg, *input_cloud);//convert msg to pcl pointcloud
		// Downsample the input cloud
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (input_cloud);
		sor.setLeafSize (leaf_size, leaf_size, leaf_size);
		sor.filter (*cloud2_from_msg);
		ROS_INFO_STREAM("PointCloud before filtering: " << input_cloud->width * input_cloud->height
						<< " data points (" << pcl::getFieldsList (*input_cloud) << ")." << endl
						<< "PointCloud after filtering: " << cloud2_from_msg->width * cloud2_from_msg->height
						<< " data points (" << pcl::getFieldsList (*cloud2_from_msg) << ").");
		pcl::fromPCLPointCloud2(*cloud2_from_msg, *cloud_from_msg);			//convert pointcloud2 to pointcloud

        pcl::transformPointCloud(*cloud_from_msg, *cloud_from_msg, T_flip);	//flip pointcloud 180 deg around x, to get the right rotation // TODO comment back in after test
		ROS_INFO("new Pointcloud arrived! stamp: %PRIu64", cloud_from_msg->header.stamp);			//This is just for showing that the pointcloud gets updated
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

void test_two_objects(poseEstimator PE, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output)
{
	static int iteration = 0;
	static int rotations = 1;

	if(rotations > 0)
	{
		if(iteration < 10)
		{
			ROS_WARN_STREAM("Run: " << iteration+1 << " / " << " 10   (rotations left: " << rotations-1 << ")");
			global_pose_data global_pose;
			local_pose_data local_pose;
                        global_pose = PE.get_pose_global(cloud_boxFilter_output, "cloud_object_yoda", 3500, 0.000025, false);		// TODO change to true after test
                        local_pose = PE.get_pose_local(cloud_boxFilter_output, "cloud_object_yoda", 200, 0.0001, global_pose.pose, false);
			PE.save_pose_data("./pose_data.csv", global_pose, local_pose); //save pose
			char temp;
			ROS_WARN_STREAM("Move object and press a key, followed by 'Enter'");
			cin >> temp;
			iteration++;
		}
		else
		{
			char temp;
			ROS_WARN_STREAM("Rotate object and press a key, followed by 'Enter'");
			cin >> temp;
			iteration = 0;
			rotations--;
		}
	}
	else
	{
		ROS_WARN_STREAM("test done!");
	}
}

void test_function(poseEstimator PE, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output)
{
	static int iteration = 0;
	static int rotations = 16;

	if(rotations > 0)
	{
		if(iteration < 10)
		{
			ROS_WARN_STREAM("Run: " << iteration+1 << " / " << " 10   (rotations left: " << rotations-1 << ")");
			global_pose_data global_pose;
			local_pose_data local_pose;
                        global_pose = PE.get_pose_global(cloud_boxFilter_output, "cloud_object_yoshi", 3500, 0.000025, false);		// TODO change to true after test
                        local_pose = PE.get_pose_local(cloud_boxFilter_output, "cloud_object_yoshi", 200, 0.0001, global_pose.pose, false);
			PE.save_pose_data("./pose_data.csv", global_pose, local_pose); //save pose
			iteration++;
		}
		else
		{
			char temp;
			ROS_WARN_STREAM("Rotate object and press a key, followed by 'Enter'");
			cin >> temp;
			iteration = 0;
			rotations--;
		}
	}
	else
	{
		ROS_WARN_STREAM("test done!");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_node");
	ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, point_cloud_callback);
	ros::Publisher pose_pub = nh.advertise<std_msgs::Float64MultiArray>("objects_pose", 1);
	ros::Rate loop_rate(10);	//loop rate in Hz

        // add yoshi
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_object_yoshi, *cloud_object_yoshi) == -1) //* load the file
	{
    	PCL_ERROR ("Couldn't read file object yoshi \n");
    	return (-1);
	}
	pcl::transformPointCloud(*cloud_object_yoshi, *cloud_object_yoshi, Eigen::Matrix4f::Identity()*0.001);	//scale down yoshi

	// add yoda
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (path_object_yoda, *cloud_object_yoda) == -1) //* load the file
	{
	PCL_ERROR ("Couldn't read file object yoshi \n");
	return (-1);
	}
	pcl::transformPointCloud(*cloud_object_yoda, *cloud_object_yoda, Eigen::Matrix4f::Identity()*0.001);	//scale down yoda

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented_scene(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_discarded (new pcl::PointCloud<pcl::PointXYZ>);

	// Downsample objects - same leaf size as for scene cloud
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	grid.setInputCloud (cloud_object_yoshi);
	grid.filter (*cloud_object_yoshi);
	grid.setInputCloud (cloud_object_yoda);
	grid.filter (*cloud_object_yoda);

        float minX = -1, maxX = 0.0, minY = 0.0, maxY = 1, minZ = 0.01, maxZ = 0.3;

	pcl::visualization::PCLVisualizer viewer("Plane segmentation result");
	viewer.addCoordinateSystem(0.3); // 0,0,0
	Eigen::Matrix4f T_camTable2World;
	T_camTable2World << 	 0,  1,  0, -1.4730,				//0.45 used for transform output from pose estimation to worldframe
							-1,  0,  0,  0.054,				//0.05259 + 0.185987
							 0,  0,  1,  0,
							 0,  0,  0,  1;

	Eigen::Matrix4f T_yoshiButtom2GraspPoint;
	T_yoshiButtom2GraspPoint << 1,  0,  0,  0,		//0.12183-0.05318 //used for transform output from pose estimation to worldframe
								0,  1, 	0,  0.075,					
								0,  0,  1,  0.15,
								0,  0,  0,  1;

	Eigen::Matrix4f T_yoshiGraspPoint2RobotGripper;
	T_yoshiGraspPoint2RobotGripper << 	1,  0,  0,  0,				//used for transform output from pose estimation to worldframe
										0,  1, 	0,  0,					
										0,  0,  1,  0.205,
										0,  0,  0,  1;

	Eigen::Affine3f worldframe;
	worldframe = T_camTable2World;
	viewer.addCoordinateSystem(0.3, worldframe);
	viewer.addCube(minX, maxX, minY, maxY, minZ, maxZ, 1.0,1.0,0, "filter_box", 0);
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "filter_box");
	poseEstimator PE;
	cloudManipulator CM;
	
	bool first_run = true;
	bool enable_pose_estimation = false;
	bool enable_test = false;
        int calibrate_iterations = 100;

	char user_input;
	cout << "You have the following options:" << endl << "p  : Pose estimation" << endl << "f  : Free view" << endl  << "t  : Test" << endl;
	cin >> user_input;
	if(user_input == 'p')
	{
		enable_pose_estimation = true;
	}
	else if(user_input == 'f')
	{
		enable_pose_estimation = false;
	}
	else if(user_input == 't')
	{
		enable_test = true;
	}
	else
	{
		cout << endl << endl << "    Huu.. You can't even type one letter. SHAME ON YOU!" << endl << endl;
	}
        PE.addObjectCloud(cloud_object_yoshi,"cloud_object_yoshi", 0.01, 1.0, 0.04, 0.01);
        PE.addObjectCloud(cloud_object_yoda,"cloud_object_yoda", 0.01, 1.0, 0.073, 0.035);
	while(ros::ok)
	{
		ros::spinOnce();		//update all ROS related stuff
		if(new_cloud_from_msg)
		{	
			//PE.printObjectCloudsNames();
			new_cloud_from_msg = false;

                        //if (first_run)

                        if(calibrate_iterations > 0)
                        {
                            CM.segmentPlane(cloud_from_msg);
                            calibrate_iterations--;
                        }
                        CM.alignWithPlane(cloud_from_msg, cloud_aligned);
                        CM.cropCloud(cloud_aligned, cloud_boxFilter_output, minX, maxX, minY, maxY, minZ, maxZ);
                        CM.getDiscardedPoints(cloud_aligned, cloud_boxFilter_discarded);

			//ROS_INFO_STREAM("Plane coefficients: 0:" << coefficients->values[0] << " 1:" << coefficients->values[1]<< " 2:" << coefficients->values[2]<< " 3:" << coefficients->values[3]);

			//pcl::transformPointCloud(*cloud_from_msg, *cloud_scene_rotate, RPY2H(0.87266, 0.13089, 0.63, 0, 0, 0.33));

			//ROS_INFO_STREAM("PointCloud after segmentation+clipping: " << cloud2_from_msg->width * cloud2_from_msg->height
			//                       << " data points (" << pcl::getFieldsList (*cloud2_from_msg) << ").");

            if(!calibrate_iterations && first_run && enable_pose_estimation)		// TODO delete after test
			{	
				Eigen::Matrix4f T_pose_global;
				Eigen::Matrix4f T_pose_local;
				Eigen::Matrix4f T_pose_estimation;
				global_pose_data global_pose;
				local_pose_data local_pose;

                                global_pose = PE.get_pose_global(cloud_boxFilter_output, "cloud_object_yoda", 3500, 0.000025, true);		// TODO change to true after test
                                local_pose = PE.get_pose_local(cloud_boxFilter_output, "cloud_object_yoda", 200, 0.0001, global_pose.pose, true);
				
				T_pose_estimation = local_pose.pose * global_pose.pose;
				PE.valid_output_pose(T_pose_estimation);
				cout << "raw pose:" << endl << T_pose_estimation << endl;

				T_pose_estimation =  T_camTable2World * T_pose_estimation * T_yoshiGraspPoint2RobotGripper * T_yoshiButtom2GraspPoint;		//transform from youshi buttom to grasping point(middle of the top surface of the cube)
				cout << "Top pose:" << endl << T_pose_estimation << endl;

				PE.save_pose_data("./pose_data.csv", global_pose, local_pose); //save pose

				std_msgs::Float64MultiArray pose_msg;				//convert eigen matrix to ros msg and publish it
				tf::matrixEigenToMsg(T_pose_estimation, pose_msg);
				pose_pub.publish(pose_msg);

                                first_run = false;

                      }
        	if(!calibrate_iterations && enable_test)
			{
				//test_function(PE, cloud_boxFilter_output);
				test_two_objects(PE, cloud_boxFilter_output);
			}
			viewer.removePointCloud("cloud_boxFilter_output");
			viewer.removePointCloud("cloud_boxFilter_discarded");
                        //viewer.removePointCloud("yoshi");
                        viewer.removePointCloud("yoda");
			viewer.addPointCloud<pcl::PointXYZ>(cloud_boxFilter_output, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boxFilter_output, 0, 255, 0), "cloud_boxFilter_output");
			viewer.addPointCloud<pcl::PointXYZ>(cloud_boxFilter_discarded, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_boxFilter_discarded, 150, 150, 0), "cloud_boxFilter_discarded");
                        //viewer.addPointCloud<pcl::PointXYZ>(cloud_object_yoshi, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_object_yoshi, 255, 0, 0), "yoshi");
                        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "yoshi");
                        viewer.addPointCloud<pcl::PointXYZ>(cloud_object_yoda, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_object_yoda, 255, 0, 0), "yoda");
                        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "yoda");
			viewer.spinOnce();
			//viewer.spin();

                        ready_for_new_cloud = true;		// signal that the function is ready for a new cloud from the camera

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
