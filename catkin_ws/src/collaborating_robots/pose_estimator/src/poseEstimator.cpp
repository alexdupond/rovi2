#include "poseEstimator.hpp"
#include <ros/console.h>
#include <pcl/filters/crop_box.h>
#include <math.h>
#include <fstream>

typedef PointNormal PointT;
typedef Histogram<153> FeatureT;

poseEstimator::poseEstimator()
{

}

poseEstimator::~poseEstimator()
{

}

void poseEstimator::cropCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float minZ, float maxZ)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ min_point, max_point;
	pcl::getMinMax3D (*cloud_in, min_point, max_point);
	//Eigen::Vector4f centroidObject;									//move object to centroid
	//pcl::compute3DCentroid(*cloud_in, centroidObject);
        Eigen::Vector4f min_point_object(min_point.x+0.04, min_point.y+0.01, min_point.z, 1);
	pcl::demeanPointCloud(*cloud_in, min_point_object, *cloud_temp);
	*cloud_in = *cloud_temp;

	pcl::CropBox<pcl::PointXYZ> boxFilter1(true);					//fintering yoshi pointcloud. remove the bottom of the figure
	boxFilter1.setMin(Eigen::Vector4f(-1, -1, minZ, 1.0));
	boxFilter1.setMax(Eigen::Vector4f(1, 1, maxZ, 1.0));
	boxFilter1.setInputCloud(cloud_in);
	boxFilter1.filter(*cloud_temp);
	*cloud_in = *cloud_temp;
}

bool poseEstimator::addObjectCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr object, string object_name, float crop_minZ, float crop_maxZ)
{
	for (int i = 0; i < vec_objects_names.size(); i++)
	{
		if(vec_objects_names[i] == object_name)
		{
			ROS_ERROR_STREAM("poseEstimator: object cloud '" << object_name << " already exists... object cloud not added");
			return false;
		}
	}
	cropCloud(object, crop_minZ, crop_maxZ);
	vec_objects_ptr.push_back(object);
	vec_objects_names.push_back(object_name);
	return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr poseEstimator::getObjectCloud(string name)
{
	for (int i = 0; i < vec_objects_names.size(); i++)
	{
		if(vec_objects_names[i] == name)
		{
			return vec_objects_ptr[i];
		}
	}
	ROS_ERROR_STREAM("poseEstimator: cant find object cloud '" << name << "'...");
	return nullptr;
}

void poseEstimator::printObjectCloudsNames()
{
	string names = "";
	for (int i = 0; i < vec_objects_names.size(); i++)
	{
		names += "'" + vec_objects_names[i] + "'  ";
	}
	ROS_INFO_STREAM("Object cloud names:\n\t" << names);
}

// Matrix4f poseEstimator::get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter, float thressq, bool show_matches)
global_pose_data poseEstimator::get_pose_global(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter, float thressq, bool show_matches)
{
	global_pose_data data;		//data that is returned after this function
	// Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);

	//PointCloud<PointXYZ>::Ptr object_in = this->getObjectCloud(object_name);
	copyPointCloud(*scene_in, *scene);
	copyPointCloud(*(this->getObjectCloud(object_name)), *object);
    data.object_cloud_size = object->points.size();
	data.features = object->points.size();				// makes features for all points in the object file
	data.scene_cloud_size = scene->points.size();
    // Show
    // {
    //     PCLVisualizer v("Before global alignment");
    //     v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
    //     v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
    //     v.spin();
    // }

    // Compute surface normals
    {
        ScopeTime t("Surface normals");
        NormalEstimation<PointT,PointT> ne;
        ne.setKSearch(10);

        ne.setInputCloud(object);
        ne.compute(*object);

        ne.setInputCloud(scene);
        ne.compute(*scene);
		data.time_surface_normals = t.getTime();
    }

    // Compute shape features
    PointCloud<FeatureT>::Ptr object_features(new PointCloud<FeatureT>);
    PointCloud<FeatureT>::Ptr scene_features(new PointCloud<FeatureT>);
    {
        ScopeTime t("Shape features");

        SpinImageEstimation<PointT,PointT,FeatureT> spin;
        //spin.setRadiusSearch(0.05);
		// spin.setRadiusSearch(0.03);
		// spin.setMinPointCountInNeighbourhood(8);		//added by us
        spin.setRadiusSearch(0.05);
        //spin.setMinPointCountInNeighbourhood(10);		// 10 before : added by us
        spin.setInputCloud(object);
        spin.setInputNormals(object);
        spin.compute(*object_features);

        spin.setInputCloud(scene);
        spin.setInputNormals(scene);
        spin.compute(*scene_features);
		data.time_shape_features = t.getTime();
    }

    // Find feature matches
    Correspondences corr(object_features->size());
    {
        ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
		data.time_feature_matches = t.getTime();
    }

    // Show matches
    if(show_matches == true)
    {
		{
			PCLVisualizer v("Matches");
			v.addPointCloud<PointT>(object, PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
			v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
			v.addCorrespondences<PointT>(object, scene, corr, 1);
			v.spin();
        }
	}


    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);

    // Set RANSAC parameters
    //const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 5000;
	//const size_t iter = 5000;
    //const float thressq = 0.005 * 0.005;

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
				data.ransac_iterations = i;
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
		data.ransac_inliers = inliers;
		data.rms_error = rmse;
		data.time_ransac = t.getTime();
    } // End timing

    // Show result
    if(show_matches)
     {
         PCLVisualizer v("After global alignment");
         v.addPointCloud<PointT>(object_aligned, PointCloudColorHandlerCustom<PointT>(object_aligned, 0, 255, 0), "object_aligned");
         v.addPointCloud<PointT>(scene, PointCloudColorHandlerCustom<PointT>(scene, 255, 0, 0),"scene");
         v.spin();
     }
    data.pose_valid = valid_output_pose(pose);
	data.pose = pose;
    //return pose;
	return data;
}
// (this->getObjectCloud(object_name)
// Matrix4f poseEstimator::get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter, float thressq, Eigen::Matrix4f T_pose)
local_pose_data poseEstimator::get_pose_local(PointCloud<PointXYZ>::Ptr scene_in, string object_name, size_t iter, float thressq, Eigen::Matrix4f T_pose, bool show_output)
{
	local_pose_data data;

    // Load
    PointCloud<PointT>::Ptr object(new PointCloud<PointT>);
    PointCloud<PointT>::Ptr scene(new PointCloud<PointT>);
    copyPointCloud(*scene_in, *scene);
    copyPointCloud(*(this->getObjectCloud(object_name)), *object);
    pcl::transformPointCloud(*object, *object, T_pose);
	data.object_cloud_size = object->points.size();
	data.scene_cloud_size = scene->points.size();

    // Show
    // {
    //     PCLVisualizer v("Before local alignment");
    //     v.addPointCloud<PointNormal>(object, PointCloudColorHandlerCustom<PointNormal>(object, 0, 255, 0), "object");
    //     v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
    //     v.spin();
    // }

    // Create a k-d tree for scene
    search::KdTree<PointNormal> tree;
    tree.setInputCloud(scene);

    // Set ICP parameters
    //const size_t iter = argc >= 4 ? std::stoi(argv[3]) : 50;
    //const float thressq = 0.01 * 0.01;

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
		data.time_icp = t.getTime();
		data.rms_error = rmse;
		data.icp_inliers = inliers;
    } // End timing
    
	// Show result
    {
		if(show_output)
		{
			PCLVisualizer v("After local alignment");
			v.addPointCloud<PointNormal>(object_aligned, PointCloudColorHandlerCustom<PointNormal>(object_aligned, 0, 255, 0), "object_aligned");
			v.addPointCloud<PointNormal>(scene, PointCloudColorHandlerCustom<PointNormal>(scene, 255, 0, 0),"scene");
			v.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene");
			v.spin();
		}
    }
	data.pose_valid = valid_output_pose(pose);
	data.pose = pose;
    //return pose;
	return data;
}

void poseEstimator::nearest_feature(const FeatureT& query, const PointCloud<FeatureT>& target, int &idx, float &distsq)
{
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

inline float poseEstimator::dist_sq(const FeatureT& query, const FeatureT& target)
{
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    return result;
}

bool poseEstimator::valid_output_pose(Matrix4f H)
{
	//we know that the object will stand on the table, so if the pose is rotatet around x or y, it is not corretc.
	Matrix3f R = H.block<3,3>(0,0);
	Vector3f T = H.block<3,1>(0,3);
	vector<float> rpy = R2RPY(R);

	bool pose_error = false;
	//if(!(rpy[1] < MAX_POSE_ANGLE || rpy[1] > M_PI-MAX_POSE_ANGLE) || !(rpy[2] < MAX_POSE_ANGLE || rpy[2] > M_PI-MAX_POSE_ANGLE))
	if(abs(rpy[1]) > MAX_POSE_ANGLE || abs(rpy[2]) > MAX_POSE_ANGLE)
	{
			ROS_WARN_STREAM("Pose is rotatet too mutch.. Dam it! it's not trustworthy");
			ROS_WARN_STREAM("rpy: " << rpy[2] << " " << rpy[1] << " " << rpy[0]);
			pose_error = true;
	}
	if(T(0,0) > MAX_POSE_TRANSLATION_X || T(1,0) > MAX_POSE_TRANSLATION_Y || abs(T(2,0)) > MAX_POSE_TRANSLATION_Z)
	{
			ROS_WARN_STREAM("Pose is translated too mutch.. Dam it! it's not trustworthy");
			ROS_WARN_STREAM("Translation:" << endl << T);
			pose_error = true;
	}
	if(pose_error)
	{
			return false;
	}
	ROS_INFO("Pose good!");
	ROS_WARN_STREAM("rpy: " << rpy[2] << " " << rpy[1] << " " << rpy[0]);
	return true;
}

vector<float> poseEstimator::R2RPY(Matrix3f R)
{
	vector<float> rpy;
	float r11 = R(0,0), r21 = R(1,0), r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);
	float yaw = atan2(r21,r11);
	float pitch = atan2( -r31, sqrt( pow(r32,2) + pow(r33,2) ) );
	float roll = atan2(r32, r33);
	rpy.reserve(3);
	rpy.push_back(yaw);
	rpy.push_back(pitch);
	rpy.push_back(roll);
	return rpy;
}

void poseEstimator::save_pose_data(string file_name, global_pose_data global, local_pose_data local)
{
	ROS_INFO_STREAM("Saving data to file..");
	Matrix3f global_R = global.pose.block<3,3>(0,0);
	Matrix3f local_R = local.pose.block<3,3>(0,0);
	vector<float> global_rpy = R2RPY(global_R);
	vector<float> local_rpy = R2RPY(local_R);
	fstream csvfile;
	csvfile.open(file_name, ios::out | ios::app);
	csvfile	<< "global_pose, " << global.pose(0,0) << ", " << global.pose(0,1) << ", " << global.pose(0,2) <<  ", " << global.pose(0,3) << endl
			<< "global_pose, " << global.pose(1,0) << ", " << global.pose(1,1) << ", " << global.pose(1,2) <<  ", " << global.pose(1,3) << endl
			<< "global_pose, " << global.pose(2,0) << ", " << global.pose(2,1) << ", " << global.pose(2,2) <<  ", " << global.pose(2,3) << endl
			<< "global_pose, " << global.pose(3,0) << ", " << global.pose(3,1) << ", " << global.pose(3,2) <<  ", " << global.pose(3,3) << endl
			<< "global_rpy, "	<< global_rpy[0] << ", " << global_rpy[1] << ", " << global_rpy[2] << endl
			<< "global_time_surface_normals" << ", " << global.time_surface_normals << endl
			<< "global_time_shape_features" << ", " << global.time_shape_features << endl
			<< "global_time_feature_matches" << ", " << global.time_feature_matches << endl
			<< "global_time_ransac" << ", " << global.time_ransac << endl
			<< "global_ransac_iterations" << ", " << global.ransac_iterations << endl
			<< "global_ransac_inliers" << ", " << global.ransac_inliers << endl
			<< "global_features" << ", " << global.features << endl
			<< "global_object_cloud_size" << ", " << global.object_cloud_size << endl
			<< "global_scene_cloud_size" << ", " << global.scene_cloud_size << endl
			<< "global_rms_error" << ", " << global.rms_error << endl
			<< "global_pose_valid" << ", " << global.pose_valid << endl

			<< "local_pose, " << local.pose(0,0) << ", " << local.pose(0,1) << ", " << local.pose(0,2) <<  ", " << local.pose(0,3) << endl
			<< "local_pose, " << local.pose(1,0) << ", " << local.pose(1,1) << ", " << local.pose(1,2) <<  ", " << local.pose(1,3) << endl
			<< "local_pose, " << local.pose(2,0) << ", " << local.pose(2,1) << ", " << local.pose(2,2) <<  ", " << local.pose(2,3) << endl
			<< "local_pose, " << local.pose(3,0) << ", " << local.pose(3,1) << ", " << local.pose(3,2) <<  ", " << local.pose(3,3) << endl
			<< "local_rpy, "	<< local_rpy[0] << ", " << local_rpy[1] << ", " << local_rpy[2] << endl
			<< "local_time_icp" << ", " << local.time_icp << endl
			<< "local_icp_inliers" << ", " << local.icp_inliers << endl
			<< "local_object_cloud_size" << ", " << local.object_cloud_size << endl
			<< "local_scene_cloud_size" << ", " << local.scene_cloud_size << endl
			<< "local_rms_error" << ", " << local.rms_error << endl
			<< "local_pose_valid" << ", " << local.pose_valid << endl
			<< "=======================================================" << endl;
			
	csvfile.close();
	ROS_INFO_STREAM("DONE! data saved to file '" << file_name << "'");
}
