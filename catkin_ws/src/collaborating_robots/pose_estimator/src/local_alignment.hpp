#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace Eigen;

Matrix4f get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, PointCloud<PointXYZ>::Ptr object_in, float iter = 50, float thressq = 0.0001)
{
    
    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
	copyPointCloud(*scene_in, *scene);
	copyPointCloud(*object_in, *object);
    
    // Show
    {
        PCLVisualizer v("Before local alignment");
        v.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set ICP parameters
    // const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 50;
    // const float thressq = 0.01 * 0.01;
    
    // Start ICP
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
    {
        ScopeTime t("ICP");
        cout << "Starting ICP..." << endl;
        for(size_t i = 0; i < iter; ++i) {
            // 1) Find closest points
            vector<vector<int> > idx;
            vector<vector<float> > distsq;
            tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
            // Threshold and create indices for object/scene and compute RMSE
            vector<int> idxobj;
            vector<int> idxscn;
            for(size_t j = 0; j < idx.size(); ++j) {
                if(distsq[j][0] <= thressq) {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }
            
            // 2) Estimate transformation
            Matrix4f T;
            TransformationEstimationSVD<PointNormal,PointNormal> est;
            est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);
            
            // 3) Apply pose
            transformPointCloud(*object_aligned, *object_aligned, T);
            
            // 4) Update result
            pose = T * pose;
        }
        
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
    } // End timing
    
    // Show result
    {
        PCLVisualizer v("After local alignment");
        v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    
    return pose;
}

// int main(int argc, char**argv) {
//     if(argc < 3) {
//         cout << "Usage: " << argv[0] << " <object> <scene> [iterations]" << endl;
//         return 0;
//     }
    
//     // Load
//     PointCloud<PointNormal>::Ptr object(new PointCloud<PointNormal>);
//     PointCloud<PointNormal>::Ptr scene(new PointCloud<PointNormal>);
//     loadPCDFile(argv[1], *object);
//     loadPCDFile(argv[2], *scene);
    
//     // Show
//     {
//         PCLVisualizer v("Before local alignment");
//         v.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
//         v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
//         v.spin();
//     }
    
//     // Create a k-d tree for scene
//     search::KdTree<PointNormal> tree;
//     tree.setInputCloud(scene);
    
//     // Set ICP parameters
//     const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 50;
//     const float thressq = 0.01 * 0.01;
    
//     // Start ICP
//     Matrix4f pose = Matrix4f::Identity();
//     PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>(*object));
//     {
//         ScopeTime t("ICP");
//         cout << "Starting ICP..." << endl;
//         for(size_t i = 0; i < iter; ++i) {
//             // 1) Find closest points
//             vector<vector<int> > idx;
//             vector<vector<float> > distsq;
//             tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
//             // Threshold and create indices for object/scene and compute RMSE
//             vector<int> idxobj;
//             vector<int> idxscn;
//             for(size_t j = 0; j < idx.size(); ++j) {
//                 if(distsq[j][0] <= thressq) {
//                     idxobj.push_back(j);
//                     idxscn.push_back(idx[j][0]);
//                 }
//             }
            
//             // 2) Estimate transformation
//             Matrix4f T;
//             TransformationEstimationSVD<PointNormal,PointNormal> est;
//             est.estimateRigidTransformation(*object_aligned, idxobj, *scene, idxscn, T);
            
//             // 3) Apply pose
//             transformPointCloud(*object_aligned, *object_aligned, T);
            
//             // 4) Update result
//             pose = T * pose;
//         }
        
//         // Compute inliers and RMSE
//         vector<vector<int> > idx;
//         vector<vector<float> > distsq;
//         tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
//         size_t inliers = 0;
//         float rmse = 0;
//         for(size_t i = 0; i < distsq.size(); ++i)
//             if(distsq[i][0] <= thressq)
//                 ++inliers, rmse += distsq[i][0];
//         rmse = sqrtf(rmse / inliers);
    
//         // Print pose
//         cout << "Got the following pose:" << endl << pose << endl;
//         cout << "Inliers: " << inliers << "/" << object->size() << endl;
//         cout << "RMSE: " << rmse << endl;
//     } // End timing
    
//     // Show result
//     {
//         PCLVisualizer v("After local alignment");
//         v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
//         v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
//         v.spin();
//     }
    
//     return 0;
// }