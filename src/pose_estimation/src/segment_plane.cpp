#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/conversions.h>

using namespace std;

// Source: http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)

int
main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr allPointsExceptPlane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 cloudIn1;
    pcl::PCLPointCloud2 cloudIn2;
    pcl::PCLPointCloud2 cloudOut;

    pcl::PLYReader Reader;
    Reader.read("../3D_scenes/scene_joshi3.ply", *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
    Reader.read("../3D_objects/object_joshi.ply", *object);

    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    if (planeIndices->indices.size() == 0)
        cout << "Could not find a plane in the scene." << endl;
    else
    {
        // Copy the points of the plane to a new cloud.
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(planeIndices);
        extract.filter(*plane);

        // Retrieve the convex hull.
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(plane);
        // Make sure that the resulting hull is bidimensional.
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        // Redundant check.
        if (hull.getDimension() == 2)
        {
            // Prism object.
            pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
            prism.setInputCloud(cloud);
            prism.setInputPlanarHull(convexHull);
            // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
            // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
            prism.setHeightLimits(0.0f, 0.2f);
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

            prism.segment(*objectIndices);

            // Get and show all points retrieved by the hull.
            extract.setIndices(objectIndices);
            extract.filter(*objects);
            //extract.setIndices(planeIndices);
            //extract.setNegative(true);
            //extract.filter(*allPointsExceptPlane);

            //pcl::toPCLPointCloud2(*objects, cloudIn1);
            //pcl::toPCLPointCloud2(*allPointsExceptPlane, cloudIn2);
            //pcl::concatenatePointCloud(cloudIn1, cloudIn2, cloudOut);
            //pcl::fromPCLPointCloud2(cloudOut, *objects2);

            //pcl::ExtractIndices<pcl::PointXYZ> extract2;
            //extract2.setInputCloud(objects);



            // Show result
            pcl::visualization::PCLVisualizer viewer("Plane segmentation result");
            viewer.addPointCloud<pcl::PointXYZ>(objects, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(objects, 0, 255, 0), "outliers");
            viewer.addPointCloud<pcl::PointXYZ>(plane, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(plane, 0, 0, 255), "inliers");
            //viewer.addPointCloud<pcl::PointXYZ>(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 255, 0, 0), "object");
            viewer.spin();

            /*pcl::visualization::CloudViewer viewerObjects("Objects on table");
            viewerObjects.showCloud(objects);
            while (!viewerObjects.wasStopped())
            {
                // Do nothing but wait.
            }*/
        }
        else cout << "The chosen hull is not planar." << endl;
    }
}
