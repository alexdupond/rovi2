#ifndef CLOUDMANIPULATOR_HPP
#define CLOUDMANIPULATOR_HPP

#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PointIndices.h>
#include <boost/foreach.hpp>

using namespace std;

class cloudManipulator
{

public:
        cloudManipulator();
        ~cloudManipulator();
        bool segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original);
        void alignWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned);
        void alignWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned, float Rz, float Ry, float Rx, float tx, float ty, float tz);
        void removePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented);
        void cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output, float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
        void getDiscardedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_discarded);

private:
        Eigen::Matrix4f RPY2H(float Rz, float Ry, float Rx, float tx, float ty, float tz);
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        pcl::PointIndices indices_discarded_points;

};

#endif
