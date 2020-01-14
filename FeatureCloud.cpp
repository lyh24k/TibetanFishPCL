#include "FeatureCloud.h"

FeatureCloud::FeatureCloud(): 
	search_method_xyz_ (new SearchMethod),
	normal_radius_ (2.0f),
	feature_radius_ (2.0f)
	{}

FeatureCloud::~FeatureCloud() {}

void FeatureCloud::setInputCloud(PointCloud::Ptr xyz)
{
	xyz_ = xyz;
	processInput();
}

void FeatureCloud::loadInputCloud(const std::string &pcd_file)
{
	xyz_ = PointCloud::Ptr(new PointCloud);
	pcl::io::loadPCDFile(pcd_file, *xyz_);
	processInput();
}

PointCloud::Ptr FeatureCloud::getPointCloud()
{
	return (xyz_);
}

SurfaceNormals::Ptr FeatureCloud::getSurfaceNormals()
{
	return (normals_);
}

LocalFeatures::Ptr FeatureCloud::getLocalFeatures()
{
	return (features_);
}

void FeatureCloud::processInput()
{
	computeSurfaceNormals();
	computeLocalFeatures();
}

// Compute the surface normals
void FeatureCloud::computeSurfaceNormals()
{
	normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setInputCloud(xyz_);
	norm_est.setSearchMethod(search_method_xyz_);
	norm_est.setRadiusSearch(normal_radius_);
	norm_est.compute(*normals_);
}

void FeatureCloud::computeLocalFeatures()
{
	features_ = LocalFeatures::Ptr(new LocalFeatures);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(xyz_);
	fpfh_est.setInputNormals(normals_);
	fpfh_est.setSearchMethod(search_method_xyz_);
	fpfh_est.setRadiusSearch(feature_radius_);
	fpfh_est.compute(*features_);
}

//TemplateAlignment
TemplateAlignment::TemplateAlignment():
	min_sample_distance_(2.0f),
	max_correspondence_distance_(0.1f),
	nr_iterations_(500)
{
	// Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
	sac_ia_.setMinSampleDistance(min_sample_distance_);
	sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
	sac_ia_.setMaximumIterations(nr_iterations_);
}

TemplateAlignment::~TemplateAlignment() {}

// Set the given cloud as the target to which the templates will be aligned
void TemplateAlignment::setTargetCloud(FeatureCloud &target_cloud)
{
	target_ = target_cloud;
	sac_ia_.setInputTarget(target_cloud.getPointCloud());
	sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
}

// Add the given cloud to the list of template clouds
void TemplateAlignment::addTemplateCloud(FeatureCloud &template_cloud)
{
	templates_.push_back(template_cloud);
}

// Align the given template cloud to the target specified by setTargetCloud ()
void TemplateAlignment::align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
{
	//PointCloud::Ptr temp(new PointCloud);
	//temp = template_cloud.getPointCloud();
	sac_ia_.setInputSource(template_cloud.getPointCloud());
	//sac_ia_.setInputCloud(template_cloud.getPointCloud());
	LocalFeatures::Ptr temp(new LocalFeatures);
	temp = template_cloud.getLocalFeatures();
	sac_ia_.setSourceFeatures(temp);

	pcl::PointCloud<pcl::PointXYZ> registration_output;
	sac_ia_.align(registration_output);

	result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
	result.final_transformation = sac_ia_.getFinalTransformation();
}

// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
void TemplateAlignment::alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
{
	results.resize(templates_.size());
	for (std::size_t i = 0; i < templates_.size(); ++i)
	{
		align(templates_[i], results[i]);
	}
}

// Align all of template clouds to the target cloud to find the one with best alignment score
int TemplateAlignment::findBestAlignment(TemplateAlignment::Result &result)
{
	// Align all of the templates to the target cloud
	std::vector<Result, Eigen::aligned_allocator<Result> > results;
	alignAll(results);

	// Find the template with the best (lowest) fitness score
	float lowest_score = std::numeric_limits<float>::infinity();
	int best_template = 0;
	for (std::size_t i = 0; i < results.size(); ++i)
	{
		const Result &r = results[i];
		if (r.fitness_score < lowest_score)
		{
			lowest_score = r.fitness_score;
			best_template = (int)i;
		}
	}

	// Output the best alignment
	result = results[best_template];
	return (best_template);
}