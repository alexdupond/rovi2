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

void nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq);

Matrix4f get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, PointCloud<PointXYZ>::Ptr object_in, size_t iter = 4000)
{   
    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
	copyPointCloud(*scene_in, *scene);
	copyPointCloud(*object_in, *object);
    
    // Show
    {
        PCLVisualizer v("Before global alignment");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.spin();
    }
    
    // Compute surface normals
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(10);
        
        ne.setInputCloud(object);
        ne.compute(*object);
        
        ne.setInputCloud(scene);
        ne.compute(*scene);
    }
    
    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");
        
        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        spin.setRadiusSearch(0.05);
        
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);
        
        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
    }
    
    // Find feature matches
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }
    
    // Show matches
    {
        PCLVisualizer v("Matches");
        v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
        v.addCorrespondences<PointT>(object, scene, corr, 1);
        v.spin();
    }
    
    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);
    
    // Set RANSAC parameters
    //const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 5000;
	//const size_t iter = 5000;
    const float thressq = 0.01 * 0.01;
    
    // Start RANSAC
    Matrix4f pose = Matrix4f::Identity();
    PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
    float penalty = FLT_MAX;
    {
        ScopeTime t("RANSAC");
        cout << "Starting RANSAC..." << endl;
        UniformGenerator<int> gen(0, corr.size() - 1);
        for(size_t i = 0; i < iter; ++i) {
            if((i + 1) % 100 == 0)
                cout << "\t" << i+1 << endl;
            // Sample 3 random correspondences
            vector<int> idxobj(3);
            vector<int> idxscn(3);
            for(int j = 0; j < 3; ++j) {
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
            if(penaltyi < penalty) {
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
    } // End timing
    
    // Show result
    {
        PCLVisualizer v("After global alignment");
        v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
        v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
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
//     PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
//     PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
//     loadPCDFile(argv[1], *object);
//     loadPCDFile(argv[2], *scene);
    
//     // Show
//     {
//         PCLVisualizer v("Before global alignment");
//         v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
//         v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
//         v.spin();
//     }
    
//     // Compute surface normals
//     {
//         ScopeTime t("Surface normals");
//         NormalEstimation<PointT,PointT> ne;
//         ne.setKSearch(10);
        
//         ne.setInputCloud(object);
//         ne.compute(*object);
        
//         ne.setInputCloud(scene);
//         ne.compute(*scene);
//     }
    
//     // Compute shape features
//     PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
//     PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
//     {
//         ScopeTime t("Shape features");
        
//         SpinImageEstimation<PointT,PointT,FeatureT> spin;
//         spin.setRadiusSearch(0.05);
        
//         spin.setInputCloud(object);
//         spin.setInputNormals(object);
//         spin.compute(*object_features);
        
//         spin.setInputCloud(scene);
//         spin.setInputNormals(scene);
//         spin.compute(*scene_features);
//     }
    
//     // Find feature matches
//     Correspondences corr(object_features->size());
//     {
//         ScopeTime t("Feature matches");
//         for(size_t i = 0; i < object_features->size(); ++i) {
//             corr[i].index_query = i;
//             nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
//         }
//     }
    
//     // Show matches
//     {
//         PCLVisualizer v("Matches");
//         v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
//         v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
//         v.addCorrespondences<PointT>(object, scene, corr, 1);
//         v.spin();
//     }
    
//     // Create a k-d tree for scene
//     search::KdTree<PointNormal> tree;
//     tree.setInputCloud(scene);
    
//     // Set RANSAC parameters
//     const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 5000;
//     const float thressq = 0.01 * 0.01;
    
//     // Start RANSAC
//     Matrix4f pose = Matrix4f::Identity();
//     PointCloud<PointNormal>::Ptr object_aligned(new PointCloud<PointNormal>);
//     float penalty = FLT_MAX;
//     {
//         ScopeTime t("RANSAC");
//         cout << "Starting RANSAC..." << endl;
//         UniformGenerator<int> gen(0, corr.size() - 1);
//         for(size_t i = 0; i < iter; ++i) {
//             if((i + 1) % 100 == 0)
//                 cout << "\t" << i+1 << endl;
//             // Sample 3 random correspondences
//             vector<int> idxobj(3);
//             vector<int> idxscn(3);
//             for(int j = 0; j < 3; ++j) {
//                 const int idx = gen.run();
//                 idxobj[j] = corr[idx].index_query;
//                 idxscn[j] = corr[idx].index_match;
//             }
            
//             // Estimate transformation
//             Matrix4f T;
//             TransformationEstimationSVD<PointNormal,PointNormal> est;
//             est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T);
            
//             // Apply pose
//             transformPointCloud(*object, *object_aligned, T);
            
//             // Validate
//             vector<vector<int> > idx;
//             vector<vector<float> > distsq;
//             tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);
            
//             // Compute inliers and RMSE
//             size_t inliers = 0;
//             float rmse = 0;
//             for(size_t j = 0; j < distsq.size(); ++j)
//                 if(distsq[j][0] <= thressq)
//                     ++inliers, rmse += distsq[j][0];
//             rmse = sqrtf(rmse / inliers);
            
//             // Evaluate a penalty function
//             const float outlier_rate = 1.0f - float(inliers) / object->size();
//             //const float penaltyi = rmse;
//             const float penaltyi = outlier_rate;
            
//             // Update result
//             if(penaltyi < penalty) {
//                 cout << "\t--> Got a new model with " << inliers << " inliers!" << endl;
//                 penalty = penaltyi;
//                 pose = T;
//             }
//         }
        
//         transformPointCloud(*object, *object_aligned, pose);
        
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
//         PCLVisualizer v("After global alignment");
//         v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
//         v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
//         v.spin();
//     }
    
//     return 0;
// }

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