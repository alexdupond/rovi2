#ifndef POSEESTIMATOR_HPP
#define POSEESTIMATOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

#define MAX_POSE_ANGLE 0.20             //the maximum amount of rotation (rad) the estimatet pose is alowed to have around rx and ry
#define MAX_POSE_TRANSLATION_X 1    	//the maximum amound of translation (meter) in x;
#define MAX_POSE_TRANSLATION_Y 1        //the maximum amound of translation (meter) in x;
#define MAX_POSE_TRANSLATION_Z 0.03     //the maximum amound of translation (meter) in x;

struct global_pose_data
{
	Matrix4f pose;
	int time_surface_normals;
	int time_shape_features;
	int time_feature_matches;
	int time_ransac;
	int ransac_iterations;
	int ransac_inliers;
	int features;
	int object_cloud_size;
	int scene_cloud_size;
	float rms_error;
	bool pose_valid;
};

struct local_pose_data
{
	Matrix4f pose;
	int time_icp;
	int icp_inliers;
	int object_cloud_size;
	int scene_cloud_size;
	float rms_error;
	bool pose_valid;
};

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

class poseEstimator
{

public:
	poseEstimator();
	~poseEstimator();
	bool addObjectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr object, string object_name, float crop_minZ, float crop_maxZ, float offset_x, float offset_y);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getObjectCloud(string name);
	void printObjectCloudsNames();
	// Matrix4f get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 4000, float thressq = 0.000025, bool show_matches = false);
	global_pose_data get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 4000, float thressq = 0.000025, bool show_matches = false);
	// Matrix4f get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 50, float thressq = 0.0001, Eigen::Matrix4f T_pose = Eigen::Matrix4f::Identity());
	local_pose_data get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 50, float thressq = 0.0001, Eigen::Matrix4f T_pose = Eigen::Matrix4f::Identity(), bool show_output = false);
	bool valid_output_pose(Matrix4f H);
	void save_pose_data(string file_name, global_pose_data global, local_pose_data local);

private:
	void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);
	inline float dist_sq(const FeatureT& query, const FeatureT& target);
	void cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float minZ = 1, float maxZ = 1, float offset_x = 0, float offset_y = 0);
	vector<float> R2RPY(Matrix3f R);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_objects_ptr;
	vector<string> vec_objects_names;
};

#endif
