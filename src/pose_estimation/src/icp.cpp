#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cmath>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>


int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);

  std::string sceneName = "../3D_scenes/scene.pcd";
  std::string objectName = "../3D_objects/object-local.pcd";


  pcl::io::loadPCDFile(sceneName, *scene);
  pcl::io::loadPCDFile(objectName, *object);

  // Create a k-d tree for scene
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(scene);

  pcl::PointCloud<pcl::PointXYZ> closestPoints; 
  std::vector<int> pointsToUseTgt, pointsToUseSrc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned(new pcl::PointCloud<pcl::PointXYZ>(*object));

  float minDist = 0.005;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  for(int k = 0; k < 50; k++)
  {
    // 1) Find closest points
    std::vector<std::vector<int> > idx;
    std::vector<std::vector<float> > distsq;
    tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

    // Threshold and create indices for object/scene and compute RMSE
    std::vector<int> idxobj;
    std::vector<int> idxscn;
    for(size_t j = 0; j < idx.size(); ++j) {
        if(distsq[j][0] <= minDist) {
            idxobj.push_back(j);
            idxscn.push_back(idx[j][0]);
        }
    }

    // 2) Estimate transformation
    Eigen::Matrix4f T;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> est;
    est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);

    // 3) Apply pose
    pcl::transformPointCloud(*object_aligned, *object_aligned, T);

    // 4) Update result
    pose = T * pose;

    
  }

  // Compute inliers and RMSE
  std::vector<std::vector<int> > idx;
  std::vector<std::vector<float> > distsq;
  tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
  size_t inliers = 0;
  float rmse = 0;
  for(size_t i = 0; i < distsq.size(); ++i)
      if(distsq[i][0] <= minDist)
          ++inliers, rmse += distsq[i][0];
  rmse = std::sqrt(rmse / inliers);

  // Print pose
  std::cout << "Got the following pose:" << std::endl << pose << std::endl;
  std::cout << "Inliers: " << inliers << "/" << object->size() << std::endl;
  std::cout << "RMSE: " << rmse << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (scene, "scene");
  viewer->addPointCloud<pcl::PointXYZ> (object, "object");
  viewer->addPointCloud<pcl::PointXYZ> (object_aligned, "object aligned");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "scene");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1, "object");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_aligned");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,1, "object_aligned");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

 while (!viewer->wasStopped ()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
 
  return (0);
}
