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

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

class poseEstimator
{

public:
	poseEstimator();
	~poseEstimator();
	bool addObjectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr object, string object_name, float crop_minZ, float crop_maxZ);
	pcl::PointCloud<pcl::PointXYZ>::Ptr getObjectCloud(string name);
	void printObjectCloudsNames();
	Matrix4f get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 4000, bool show_matches = true);
	Matrix4f get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter = 50, float thressq = 0.0001);

private:
	void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);
	inline float dist_sq(const FeatureT& query, const FeatureT& target);
	void cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float minZ = 1, float maxZ = 1);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_objects_ptr;
	vector<string> vec_objects_names;

};

#endif
