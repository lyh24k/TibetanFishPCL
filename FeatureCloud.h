#pragma once
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

class FeatureCloud
{
public:
	FeatureCloud();
	~FeatureCloud();
	void setInputCloud(PointCloud::Ptr xyz);
	void loadInputCloud(const std::string &pcd_file);
	PointCloud::Ptr getPointCloud();
	SurfaceNormals::Ptr getSurfaceNormals();
	LocalFeatures::Ptr getLocalFeatures();
	void processInput();
	void computeSurfaceNormals();
	void computeLocalFeatures();
private:
	// Point cloud data
	PointCloud::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

	// Parameters
	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment
{
public:
	// A struct for storing alignment results
	struct Result
	{
		float fitness_score;
		Eigen::Matrix4f final_transformation;
	};
	TemplateAlignment();
	~TemplateAlignment();
	void setTargetCloud(FeatureCloud &target_cloud);
	void addTemplateCloud(FeatureCloud &template_cloud);
	void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result);
	void alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results);
	int findBestAlignment(TemplateAlignment::Result &result);

private:
	// A list of template clouds and the target to which they will be aligned
	std::vector<FeatureCloud> templates_;
	FeatureCloud target_;

	// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_;
	float max_correspondence_distance_;
	int nr_iterations_;
};