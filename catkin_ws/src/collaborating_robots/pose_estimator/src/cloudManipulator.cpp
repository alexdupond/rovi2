#include "cloudManipulator.hpp"
#include "ros/ros.h"

using namespace std;

cloudManipulator::cloudManipulator()
{

}

cloudManipulator::~cloudManipulator()
{

}

Eigen::Matrix4f cloudManipulator::RPY2H(float Rz, float Ry, float Rx, float tx, float ty, float tz)
{
        Eigen::Matrix4f T_homogenus;
        T_homogenus << 	( cos(Rz) * cos(Ry) ),	( cos(Rz) * sin(Ry) * sin(Rx) - sin(Rz) * cos(Rx) ),	( cos(Rz) * sin(Ry) * cos(Rx) + sin(Rz) * sin(Rx) ),	tx,
                                        ( sin(Rz) * cos(Ry) ),	( sin(Rz) * sin(Ry) * sin(Rx) + cos(Rz) * cos(Rx) ),	( sin(Rz) * sin(Ry) * cos(Rx) - cos(Rz) * sin(Rx) ),	ty,
                                        ( -sin(Ry) ),			( cos(Ry) * sin(Rx) ),									( cos(Ry) * cos(Rx) ),									tz,
                                        0,						0,														0,														1;
        return T_homogenus;
}

bool cloudManipulator::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original)
{
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud_original);
    seg.segment (inliers, coefficients);
    if (inliers.indices.size() == 0)
        return false;
    ROS_INFO_STREAM("Plane coefficients: 0:" << coefficients.values[0] << " 1:" << coefficients.values[1]<< " 2:" << coefficients.values[2]<< " 3:" << coefficients.values[3]);
    coefficients_vector.push_back(coefficients);
    averageCoefficients();
    return true;
}

void cloudManipulator::alignWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned)
{
    Eigen::Matrix4f T_homogenus = RPY2H(coefficients.values[2], -coefficients.values[0], coefficients.values[1], 0, 0, coefficients.values[3]);
    pcl::transformPointCloud(*cloud_original, *cloud_aligned, T_homogenus);
}

void cloudManipulator::alignWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned, float Rz, float Ry, float Rx, float tx, float ty, float tz)
{
    Eigen::Matrix4f T_homogenus = RPY2H(Rz, Ry, Rx, tx, ty, tz);
    pcl::transformPointCloud(*cloud_original, *cloud_aligned, T_homogenus);
}

void cloudManipulator::removePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_aligned);
    extract.setIndices(boost::make_shared<pcl::PointIndices> (inliers));
    extract.setNegative(true);
    extract.filter(*cloud_segmented);
}

void cloudManipulator::cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_output, float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    pcl::CropBox<pcl::PointXYZ> boxFilter(true);								//fintering pointcloud. remove everything outside the box
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud_segmented);
    boxFilter.filter(*cloud_boxFilter_output);
    boxFilter.getRemovedIndices(indices_discarded_points);
}

void cloudManipulator::getDiscardedPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segmented_scene, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boxFilter_discarded)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;							//finding the removed points, for visualisation purposes
    extract.setInputCloud(cloud_segmented_scene);
    extract.setIndices(boost::make_shared<pcl::PointIndices> (indices_discarded_points));
    extract.setNegative(false);
    extract.filter(*cloud_temp);

    pcl::CropBox<pcl::PointXYZ> boxFilter(true);
    boxFilter.setMin(Eigen::Vector4f(-1, -1, -1, 1.0));					//removing points that is fare away, for visualisation purposes
    boxFilter.setMax(Eigen::Vector4f(1, 1, 1, 1.0));
    boxFilter.setInputCloud(cloud_temp);
    boxFilter.filter(*cloud_boxFilter_discarded);
}

void cloudManipulator::averageCoefficients()
{
    vector<float> coefficients_temp(4, 0);
    for (int i = 0; i < coefficients_vector.size(); i++)
    {
        coefficients_temp[0] += coefficients_vector[i].values[0];
        coefficients_temp[1] += coefficients_vector[i].values[1];
        coefficients_temp[2] += coefficients_vector[i].values[2];
        coefficients_temp[3] += coefficients_vector[i].values[3];
    }
    coefficients_temp[0] /= coefficients_vector.size();
    coefficients_temp[1] /= coefficients_vector.size();
    coefficients_temp[2] /= coefficients_vector.size();
    coefficients_temp[3] /= coefficients_vector.size();
    coefficients.values[0] = coefficients_temp[0];
    coefficients.values[1] = coefficients_temp[1];
    coefficients.values[2] = coefficients_temp[2];
    coefficients.values[3] = coefficients_temp[3];

    ROS_INFO_STREAM("Plane coefficients: 0:" << coefficients.values[0] << " 1:" << coefficients.values[1]<< " 2:" << coefficients.values[2]<< " 3:" << coefficients.values[3]);
}
