#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::common;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;
typedef Histogram<153> FeatureT;

inline float dist_sq(const FeatureT& query, const FeatureT& target) {
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

int
  main (int argc, char** argv)
{
  PointCloud<PointNormal>::Ptr scene (new PointCloud<PointNormal>);
  PointCloud<PointNormal>::Ptr object (new PointCloud<PointNormal>);

  pcl::PLYReader Reader;
    Reader.read("../3D_scenes/scene_joshi3.ply", *scene);
    Reader.read("../3D_objects/object_joshi.ply", *object);

  // Load scene- and object file
//  string sceneName = "../scene.pcd";
//  string objectName = "../pcl_project/object-global.pcd";
//  loadPCDFile(sceneName, *scene);
//  loadPCDFile(objectName, *object);

  PCLVisualizer viewer("Before global alignment");
  viewer.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
  viewer.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
  viewer.spin();



  // Compute surface normals
  NormalEstimation<PointNormal, PointNormal> ne;
  ne.setKSearch(10);

  ne.setInputCloud(object);
  ne.compute(*object);

  ne.setInputCloud(scene);
  ne.compute(*scene);

  // Compute shape features
  PointCloud<FeatureT>::Ptr objectFeatures(new PointCloud<FeatureT>);
  PointCloud<FeatureT>::Ptr sceneFeatures(new PointCloud<FeatureT>);

  SpinImageEstimation<PointNormal,PointNormal,FeatureT> spin;
  spin.setRadiusSearch(0.05);

  spin.setInputCloud(object);
  spin.setInputNormals(object);
  spin.compute(*objectFeatures);

  spin.setInputCloud(scene);
  spin.setInputNormals(scene);
  spin.compute(*sceneFeatures);

  // Find feature matches
  Correspondences corr(objectFeatures->size());
  for(size_t i = 0; i < objectFeatures->size(); ++i)
  {
      corr[i].index_query = i;
      nearest_feature(objectFeatures->points[i], *sceneFeatures, corr[i].index_match, corr[i].distance);
  }
  {
      PCLVisualizer viewer("Matches");
      viewer.addPointCloud<PointNormal> (scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0), "scene");
      viewer.addPointCloud<PointNormal> (object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
      viewer.addCorrespondences<PointNormal>(object, scene, corr, 1);
      viewer.spin();
  }
  // Create a k-d tree for scene
  search::KdTree<PointNormal> tree;
  tree.setInputCloud(scene);
  
  // Set RANSAC parameters
  const float thressq = 0.0001;

  // Start RANSAC
  Matrix4f pose = Matrix4f::Identity();
  PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
  float penalty = FLT_MAX;
  const size_t k = 50;

  {
      cout << "Starting RANSAC..." << endl;
      UniformGenerator<int> gen(0, corr.size() - 1);
      for(size_t i = 0; i < k; ++i)
      {
          if((i + 1) % 100 == 0)
              cout << "\t" << i+1 << endl;
          // Sample 3 random correspondences
          vector<int> idxobj(3);
          vector<int> idxscn(3);
          for(int j = 0; j < 3; ++j)
          {
              const int idx = gen.run();
              idxobj[j] = corr[idx].index_query;
              idxscn[j] = corr[idx].index_match;
          }

          // Estimate transformation
          Matrix4f T;
          TransformationEstimationSVD<PointNormal,PointNormal> est;
          est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);

          // Apply pose
          transformPointCloud(*object, *object_aligned, T);

          // Validate
          vector<vector<int> > idx;
          vector<vector<float> > distsq;
          tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

          // Compute inliers and RMSE
          size_t inliers = 0;
          float rmse = 0;
          for(size_t j = 0; j < distsq.size(); ++j)
              if(distsq[j][0] <= thressq)
                  ++inliers, rmse += distsq[j][0];
          rmse = sqrtf(rmse / inliers);

          // Evaluate a penalty function
          const float outlier_rate = 1.0f - float(inliers) / object->size();
          //const float penaltyi = rmse;
          const float penaltyi = outlier_rate;

          // Update result
          if(penaltyi < penalty)
          {
              cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
              penalty = penaltyi;
              pose = T;
          }
      }

      transformPointCloud(*object, *object_aligned, pose);

      // Compute inliers and RMSE
      vector<vector<int> > idx;
      vector<vector<float> > distsq;
      tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
      size_t inliers = 0;
      float rmse = 0;
      for(size_t i = 0; i < distsq.size(); ++i)
          if(distsq[i][0] <= thressq)
              ++inliers, rmse += distsq[i][0];
      rmse = sqrtf(rmse / inliers);

      // Print pose
      cout << "Got the following pose:" << endl << pose << endl;
      cout << "Inliers: " << inliers << "/" << object->size() << endl;
      cout << "RMSE: " << rmse << endl;
  }

  // Show result
  {
      PCLVisualizer viewer("After global alignment");
      viewer.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
      viewer.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
      viewer.spin();
  }

    return 0;
}
