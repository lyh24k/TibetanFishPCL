#include "TibetanFish.h"

int TibetanFish::RemovePlane(string plyfile)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPLYFile<PointT>(plyfile, *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return -1;
	}
	cout << "orginal pointcloud " << cloud->points.size() << endl;
	//another segmentation
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.8);
	seg.setMaxIterations(100);
	//seg.setProbability(0.3);
	pcl::copyPointCloud(*cloud, *cloud_filtered);
	//cloud_filtered = cloud;
	int nr_points = (int)cloud_filtered->points.size();//剩余点云的数量
	cout << "while pointcloud " << cloud->points.size() << endl;
	cout << "while pointcloud " << cloud_filtered->points.size() << endl;
	while (cloud_filtered->points.size() > 0.3 * nr_points)
	{
		// 从剩余点云中再分割出最大的平面分量 （因为我们要处理的点云的数据是两个平面的存在的）
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) //如果内点的数量已经等于0，就说明没有
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// 从输入的点云中提取平面模型的内点
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);        //提取内点的索引并存储在其中
		extract.setNegative(false);

		// 得到与平面表面相关联的点云数据
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		//  // 移去平面局内点，提取剩余点云
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}
	cout << "tree pointcloud " << cloud->points.size() << endl;
	//pcl::io::savePCDFileASCII("loopfilter.pcd", *cloud_filtered);
	//cout << "sac success" << endl;
	//return 1;
	// 创建用于提取搜索方法的kdtree树对象
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_filtered);
	//欧几里得分割
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(2);                     // 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize(100);                 //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(100000);               //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                    //设置点云的搜索机制
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
	pcl::PointCloud<PointT>::Ptr cloud_fishonly(new pcl::PointCloud<PointT>());
	int fishindex = 0;
	int maxpoints = 0;
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		if (cluster_indices[i].indices.size() > maxpoints)
		{
			fishindex = i;
			maxpoints = cluster_indices[i].indices.size();
		}
	}
	cloud_fishonly->resize(cluster_indices[fishindex].indices.size());
	for (int i = 0; i < cluster_indices[fishindex].indices.size(); i++)
	{
		cloud_fishonly->points[i] = cloud_filtered->points[cluster_indices[fishindex].indices[i]];
	}
	pcl::io::savePCDFileASCII("cloud_fishonly.pcd", *cloud_fishonly);
	//cout << "sac success" << cloud->points.size() << endl;
	//区域生长分割
	pcl::search::Search<PointT>::Ptr treereg = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(treereg);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
	normal_estimator.compute(*normals);
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-500, -300);//保留或过滤z轴方向
	pass.filter(*indices);
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(20);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);//1.30 2.20 3.30 4.20 5.20 6.40
	reg.setInputCloud(cloud);
	reg.setIndices(indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(2.0);
	std::vector <pcl::PointIndices> clusters;
	if (indices->size() != 0)
	{
		reg.extract(clusters);
	}
	else
	{
		cout << "indices error 0" << endl;
		return -1;
	}

	/*pcl::PointCloud<PointT>::Ptr cloud_fishonly3(new pcl::PointCloud<PointT>());
	cloud_fishonly3 = reg.getColoredCloudRGBA();
	pcl::io::savePCDFileASCII("regtest.pcd", *cloud_fishonly3);
	cout << "regtest success" << endl;
	return 1;*/
	// 找比较大的连通点云，并且最高的，就是鱼的部分
	int reindex = -1;
	int z_avemax = -1000;
	int maxcount = 0;
	int indicessize = cloud->points.size()*0.02;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].indices.size() > indicessize)
		{
			int z_sum = 0;
			int z_ave = 0;
			for (int j = 0; j < clusters[i].indices.size(); j++)
			{
				z_sum += cloud->points[clusters[i].indices[j]].z;
			}
			z_ave = z_sum / clusters[i].indices.size();
			if (z_ave > z_avemax)
			{
				z_avemax = z_ave;
				reindex = i;
			}
			maxcount++;
		}
	}
	cout << "maxcount " << maxcount << endl;
	if (reindex == -1)
	{
		cout << "can't find fish,too big" << endl;
		return -1;
	}
	cout << reindex << endl;
	pcl::PointCloud<PointT>::Ptr cloud_fishonly2(new pcl::PointCloud<PointT>());
	cloud_fishonly2->resize(clusters[reindex].indices.size());
	for (int i = 0; i < clusters[reindex].indices.size(); i++)
	{
		cloud_fishonly2->points[i] = cloud->points[clusters[reindex].indices[i]];
	}
	pcl::io::savePCDFileASCII("cloud_fishonly2.pcd", *cloud_fishonly2);
	cout << "reg success" << endl;
	//把平面分割和区域生长分割得到的鱼点云合在一起
	pcl::PointCloud<PointT>::Ptr cloud_fishonly3(new pcl::PointCloud<PointT>());
	cout << " fishonly " << cloud_fishonly->points.size() << " fishonly2 " << cloud_fishonly2->points.size() << endl;
	*cloud_fishonly3 = *cloud_fishonly + *cloud_fishonly2;
	pcl::io::savePCDFileASCII("output/gaerxianzhuiwenyexuyu1completefish.pcd", *cloud_fishonly3);
	cout << "success fishonly3 " << cloud_fishonly3->points.size() << endl;
	return 1;

	
}

void TibetanFish::FishOnlyLCCP(string plyfile)
{
	// 输入点云
		pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	pcl::PCLPointCloud2 input_pointcloud2;
	if (pcl::io::loadPCDFile(plyfile, input_pointcloud2))
	{
		PCL_ERROR("ERROR: Could not read input point cloud ");
		return;
	}
	pcl::fromPCLPointCloud2(input_pointcloud2, *input_cloud_ptr);
	PCL_INFO("Done making cloud\n");

	float voxel_resolution = 0.75f;
	float seed_resolution = 3.0f;
	float color_importance = 0.1f;
	float spatial_importance = 0.5f;
	float normal_importance = 1.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	unsigned int k_factor = 0;
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(input_cloud_ptr);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);

	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();
	ofstream outFile1("过分割3.txt", std::ios_base::out);
	for (int i = 0; i < overseg->size(); i++) {
		outFile1 << overseg->points[i].x << "\t" << overseg->points[i].y << "\t" << overseg->points[i].z << "\t" << overseg->points[i].label << endl;
	}
	int label_max1 = 0;
	for (int i = 0; i< overseg->size(); i++) {
		if (overseg->points[i].label>label_max1)
			label_max1 = overseg->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud1->height = 1;
	ColoredCloud1->width = overseg->size();
	ColoredCloud1->resize(overseg->size());
	for (int i = 0; i < label_max1; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < overseg->size(); j++) {
			if (overseg->points[j].label == i) {
				ColoredCloud1->points[j].x = overseg->points[j].x;
				ColoredCloud1->points[j].y = overseg->points[j].y;
				ColoredCloud1->points[j].z = overseg->points[j].z;
				ColoredCloud1->points[j].r = color_R;
				ColoredCloud1->points[j].g = color_G;
				ColoredCloud1->points[j].b = color_B;
			}
		}
	}
	pcl::io::savePCDFileASCII("过分割3.pcd", *ColoredCloud1);



	//LCCP分割  
	float concavity_tolerance_threshold = 5;
	float smoothness_threshold = 0.1;
	uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	ofstream outFile2("分割后合并3.txt", std::ios_base::out);
	for (int i = 0; i < lccp_labeled_cloud->size(); i++) {
		outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << endl;
	}

	int label_max2 = 0;
	for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label>label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud2->height = 1;
	ColoredCloud2->width = lccp_labeled_cloud->size();
	ColoredCloud2->resize(lccp_labeled_cloud->size());
	for (int i = 0; i < label_max2; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {
			if (lccp_labeled_cloud->points[j].label == i) {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
			}
		}
	}
	pcl::io::savePCDFileASCII("分割后合并3.pcd", *ColoredCloud2);
	cout << "success" << endl;
}

int TibetanFish::FishOnlyRegion(string plyfile)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPLYFile <PointT>(plyfile, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}

	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
	normal_estimator.compute(*normals);
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-500, -300);//保留或过滤z轴方向
	pass.filter(*indices);
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(0);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);//1.30 2.20 3.30 4.20 5.20 6.40
	reg.setInputCloud(cloud);
	reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(10.0);
	std::vector <pcl::PointIndices> clusters;
	if (indices->size() != 0)
	{
		reg.extract(clusters);
	}
	else
	{
		cout << "indices error 0" << endl;
		return -1;
	}
	pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
	pcl::io::savePCDFileASCII("region_cloud.pcd", *colored_cloud);
	
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	cout << "success" << endl;
	//return 0;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
	std::endl << "cloud that belong to the first cluster:" << std::endl;
	int max1=0, max2=0;
	int max1index = 0, max2index = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].indices.size() > max1)
		{
			max2index = max1index;
			max1index = i;
			max1 = clusters[i].indices.size();
		}
		else if (clusters[i].indices.size() > max2)
		{
			max2index = i;
			max2 = clusters[i].indices.size();
		}
	}
	pcl::PointCloud<PointT>::Ptr max1cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr max2cloud(new pcl::PointCloud<PointT>);
	max1cloud->resize(clusters[max1index].indices.size());
	max2cloud->resize(clusters[max2index].indices.size());
	for (int i = 0; i < clusters[max1index].indices.size(); i++)
	{
		max1cloud->points[i] = cloud->points[clusters[max1index].indices[i]];
	}
	for (int i = 0; i < clusters[max2index].indices.size(); i++)
	{
		max2cloud->points[i] = cloud->points[clusters[max2index].indices[i]];
	}
	pcl::io::savePCDFileASCII("max1.pcd", *max1cloud);
	pcl::io::savePCDFileASCII("max2.pcd", *max2cloud);
	pcl::io::savePLYFileASCII("max2.ply", *max2cloud);
	//pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
	//pcl::io::savePCDFileASCII("region_7.pcd", *colored_cloud);
	cout << "success" << endl;
	return 0;
}

int TibetanFish::FishOnlyRegionRGB(string plyfile)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPLYFile <PointT>(plyfile, *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
	normal_estimator.compute(*normals);
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 10);
	pass.filter(*indices);
	//用于存放点云团的容器
	std::vector <pcl::PointIndices> clusters;
	//颜色分割器
	pcl::RegionGrowingRGB<PointT> reg;
	reg.setInputCloud(cloud);
	//点云经过了滤波器的预处理，提取了indices
	//reg.setIndices(indices);//设置输入点云的索引
	reg.setSearchMethod(tree);//设置点云的搜索机制
	reg.setDistanceThreshold(1);//距离阈值，判断两点是否相邻
	//点与点之间颜色容差
	reg.setPointColorThreshold(3);//两点颜色阈值，是否属于一类
	reg.setRegionColorThreshold(2);//设置两类区域区域颜色阈值，用于判断两类区域是否聚类合并
	reg.setMinClusterSize(600);//设置一个聚类的最少点数目为600
	reg.extract(clusters);
	pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
	pcl::io::savePCDFileASCII("regionrgb.pcd", *colored_cloud);
	cout << "rgb success" << endl;
	return 0;
}

int TibetanFish::FishDownSample(pcl::PointCloud<PointT>::Ptr cloud)
{
	cout << "cloud " << cloud->points.size() << endl;
	pcl::PointCloud<PointT>::Ptr cloud_simple(new pcl::PointCloud<PointT>());
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(1, 1, 1);
	sor.filter(*cloud_simple);
	pcl::io::savePCDFileASCII("all_down_sample.pcd", *cloud_simple);
	cout << "simple " << cloud_simple->points.size() << endl;
	return 1;
}

int TibetanFish::FishOverSegmentation(pcl::PointCloud<PointT>::Ptr cloud)
{

	//float voxel_resolution = 0.75f;
	//float seed_resolution = 3.0f;
	//float color_importance = 0.0f;
	//float spatial_importance = 1.0f;
	//float normal_importance = 4.0f;
	//cout << cloud->points.size() << endl;
	////生成结晶器
	//pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	//super.setUseSingleCameraTransform(false);
	////输入点云和设置参数
	//super.setInputCloud(cloud);
	//super.setColorImportance(color_importance);
	//super.setSpatialImportance(spatial_importance);
	//super.setNormalImportance(normal_importance);
	//std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	//super.extract(supervoxel_clusters);
	//cout << "super " << supervoxel_clusters.size() << endl;
	//std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	//super.getSupervoxelAdjacency(supervoxel_adjacency);
	////cpc
	//pcl::CPCSegmentation<PointT> cpcseg;
	//cpcseg.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	//using SegLabel2ClusterMap = std::map<std::uint32_t, pcl::PointCloud<WeightSACPointType>::Ptr>;
	//SegLabel2ClusterMap seg_to_edge_points_map;
	//std::map<uint32_t, std::set<uint32_t> > segment_adjacency_map;
	//cpcseg.getSegmentAdjacencyMap(segment_adjacency_map);
	//cpcseg.setCutting(20, 400, 0.16, true, true, true);
	//cpcseg.setRANSACIterations(1000);
	//cpcseg.segment();
	//pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	//pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared();
	//cpcseg.relabelCloud(*cpc_labeled_cloud);
	//SuperVoxelAdjacencyList sv_adjacency_list;
	//cpcseg.getSVAdjacencyList(sv_adjacency_list);
	//cout << cpc_labeled_cloud->points.size() << endl;
	//int label_max = 0;
	//for (int i = 0; i< cpc_labeled_cloud->size(); i++) {
	//	if (cpc_labeled_cloud->points[i].label>label_max)
	//		label_max = cpc_labeled_cloud->points[i].label;
	//}
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	//ColoredCloud1->height = 1;
	//ColoredCloud1->width = cpc_labeled_cloud->size();
	//ColoredCloud1->resize(cpc_labeled_cloud->size());
	//for (int i = 0; i < label_max; i++) {
	//	int color_R = Random(255);
	//	int color_G = Random(255);
	//	int color_B = Random(255);

	//	for (int j = 0; j < cpc_labeled_cloud->size(); j++) {
	//		if (cpc_labeled_cloud->points[j].label == i) {
	//			ColoredCloud1->points[j].x = cpc_labeled_cloud->points[j].x;
	//			ColoredCloud1->points[j].y = cpc_labeled_cloud->points[j].y;
	//			ColoredCloud1->points[j].z = cpc_labeled_cloud->points[j].z;
	//			ColoredCloud1->points[j].r = color_R;
	//			ColoredCloud1->points[j].g = color_G;
	//			ColoredCloud1->points[j].b = color_B;
	//		}
	//	}
	//}
	//pcl::io::savePCDFileASCII("cpc_seg.pcd", *ColoredCloud1);
	//cout << "cpc success" << endl;
	//过分割输出
	/*std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator it_clusters;
	pcl::PointCloud<PointT>::Ptr cloud_overseg(new pcl::PointCloud<PointT>());
	int oversegsize = 0;
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
		oversegsize += it_clusters->second->voxels_->points.size();
	}
	cloud_overseg->resize(oversegsize);
	int n = 0;
	for (it_clusters = supervoxel_clusters.begin(); it_clusters != supervoxel_clusters.end(); it_clusters++)
	{
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);
		for (int i = 0; i < it_clusters->second->voxels_->points.size(); i++)
		{
			cloud_overseg->points[n].x = it_clusters->second->voxels_->points[i].x;
			cloud_overseg->points[n].y = it_clusters->second->voxels_->points[i].y;
			cloud_overseg->points[n].z = it_clusters->second->voxels_->points[i].z;
			cloud_overseg->points[n].r = color_R;
			cloud_overseg->points[n].g = color_G;
			cloud_overseg->points[n].b = color_B;
			n++;
		}
	}
	pcl::io::savePCDFileASCII("cloud_overseg.pcd", *cloud_overseg);
	cout << n <<" overseg success " << oversegsize << endl;*/
	return 1;
}

int TibetanFish::Label2Color(pcl::PointCloud<PointTL>::Ptr cloud)
{
	int labelcount = 0;
	std::map<int, RGB> cloudrgb;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloudrgb.count(cloud->points[i].label) > 0)
		{
			continue;
		}
		else
		{
			RGB rgb;
			rgb.r = Random(255);
			rgb.g = Random(255);
			rgb.b = Random(255);
			cloudrgb[cloud->points[i].label] = rgb;
		}
		if (cloud->points[i].label > labelcount)
		{
			labelcount = cloud->points[i].label;
		}
	}
	PointCloudT::Ptr colorvoxelcloud(new PointCloudT);
	colorvoxelcloud->resize(cloud->points.size());
	for (int i = 0; i < cloud->points.size(); i++)
	{
		map<int, RGB>::iterator it;
		it = cloudrgb.find(cloud->points[i].label);
		if (it != cloudrgb.end())
		{
			colorvoxelcloud->points[i].x = cloud->points[i].x;
			colorvoxelcloud->points[i].y = cloud->points[i].y;
			colorvoxelcloud->points[i].z = cloud->points[i].z;
			colorvoxelcloud->points[i].r = it->second.r;
			colorvoxelcloud->points[i].g = it->second.g;
			colorvoxelcloud->points[i].b = it->second.b;

		}
		else
		{
			cout << "can't find color" << endl;
			continue;
		}

	}
	pcl::io::savePCDFileASCII("oversegcloud.pcd", *colorvoxelcloud);
	cout << "Label2color success" << endl;
	return 1;
}

int TibetanFish::Point2Voxel(PointCloudT::Ptr cloud)
{
	float voxel_resultion = 0.2f;
	float seed_resultion = 2.0f;
	float color_importance = 0.0f;
	float spatial_importance = 0.4f;
	float normal_importance = 5.0f;
	cout << cloud->points.size() << endl;
	pcl::SupervoxelClustering<PointT> super(voxel_resultion, seed_resultion);
	super.setInputCloud(cloud);
	super.setNormalImportance(normal_importance);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxl_clustering;
	super.extract(supervoxl_clustering);
	cout << "cluster " <<supervoxl_clustering.size() << endl;
	/*map<uint32_t, pcl::Supervoxel<PointT>::Ptr >::iterator it;
	int clustercount = 0;
	for (it = supervoxl_clustering.begin(); it != supervoxl_clustering.end(); it++)
	{
		for (int i = 0; i < it->second->voxels_->points.size(); i++)
		{
			clustercount++;
		}
	}
	cout << "clusterpoints " << clustercount << endl;*/
	pcl::PointCloud<PointTL>::Ptr supervoxel_centroid_cloud = super.getLabeledCloud();;
	cout << "label " << supervoxel_centroid_cloud->points.size() << endl;
	pcl::PointCloud<PointTL>::Ptr supervoxel_cloud = super.getLabeledVoxelCloud();
	cout << "voxel label " << supervoxel_cloud->points.size() << endl;
	multimap<uint32_t, uint32_t> SupervoxelAdjacency;
	super.getSupervoxelAdjacency(SupervoxelAdjacency);
	//获得体素点云的邻接单元

	Label2Color(supervoxel_centroid_cloud);
	//pcl::io::savePCDFileASCII("supervoxel_centroid_cloud.pcd", *supervoxel_centroid_cloud);
	cout << "success" << endl;
	return 1;
}

int TibetanFish::FacetSegmentation(PointCloudT::Ptr cloud)
{
	//求法线
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
	pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
	normalEstimation.setInputCloud(cloud);
	//normalEstimation.setRadiusSearch(1.5);
	normalEstimation.setKSearch(20);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	cout << "normals " << normals->points[0].normal_x << endl;
	//显示曲率大的地方 和附近的地方
	kdtree->setInputCloud(cloud);
	int K = 2500;
	vector<int> pointIdxKsearch(K);
	vector<float> pointNKNSquaredDistance(K);//储存近邻点对应距离平方
	PointCloudT::Ptr cur_cloud(new PointCloudT);
	cur_cloud->resize(cloud->points.size());
	bool unusedpoint[100000];
	for (int i = 0; i < cloud->points.size(); i++)
	{
		cur_cloud->points[i].x = cloud->points[i].x;
		cur_cloud->points[i].y = cloud->points[i].y;
		cur_cloud->points[i].z = cloud->points[i].z;
		if (unusedpoint[i])
		{
			if (normals->points[i].curvature < 0.08)
			{
				cur_cloud->points[i].r = 255;
				cur_cloud->points[i].g = 255;
				cur_cloud->points[i].b = 255;
				unusedpoint[i] = false;
			}
			else
			{
				int bigcurcount = 0;
				if (kdtree->nearestKSearch(cloud->points[i], 20, pointIdxKsearch, pointNKNSquaredDistance) > 0)
				{
					
					for (int j = 0; j < pointIdxKsearch.size(); j++)
					{
						if (normals->points[pointIdxKsearch[j]].curvature > 0.08)
						{
							bigcurcount++;
						}
					}
				}
				if (bigcurcount < 4)
				{
					cur_cloud->points[i].r = 255;
					cur_cloud->points[i].g = 255;
					cur_cloud->points[i].b = 255;
					unusedpoint[i] = false;
					continue;
				}
				else
				{
					cur_cloud->points[i].r = 255;
					cur_cloud->points[i].g = 0;
					cur_cloud->points[i].b = 0;
					unusedpoint[i] = false;
					if (kdtree->nearestKSearch(cloud->points[i], K, pointIdxKsearch, pointNKNSquaredDistance) > 0)
					{
						for (int j = 0; j < pointIdxKsearch.size(); j++)
						{
							//欧式距离差
							float Ddistance = sqrt(pow(cloud->points[i].x - cloud->points[pointIdxKsearch[j]].x, 2) +
								pow(cloud->points[i].y - cloud->points[pointIdxKsearch[j]].y, 2) +
								pow(cloud->points[i].z - cloud->points[pointIdxKsearch[j]].z, 2));
							float Dcolor = sqrt(3 * pow(cloud->points[i].r - cloud->points[pointIdxKsearch[j]].r, 2) +
												4 * pow(cloud->points[i].g - cloud->points[pointIdxKsearch[j]].g, 2) +
												2 * pow(cloud->points[i].b - cloud->points[pointIdxKsearch[j]].b, 2)); 
							float Dnormal = acos((normals->points[i].normal_x*normals->points[pointIdxKsearch[j]].normal_x + normals->points[i].normal_y*normals->points[pointIdxKsearch[j]].normal_y + normals->points[i].normal_z*normals->points[pointIdxKsearch[j]].normal_z)/
												sqrt((pow(normals->points[i].normal_x, 2) + pow(normals->points[i].normal_y, 2) + pow(normals->points[i].normal_z, 2))*(pow(normals->points[pointIdxKsearch[j]].normal_x, 2) + pow(normals->points[pointIdxKsearch[j]].normal_y, 2) + pow(normals->points[pointIdxKsearch[j]].normal_z, 2)))) * 180 / 3.1415926;
							if (Ddistance < 10&&Dnormal<30&&Dcolor<15)
							{
								cur_cloud->points[pointIdxKsearch[j]].r = 255;
								cur_cloud->points[pointIdxKsearch[j]].g = 0;
								cur_cloud->points[pointIdxKsearch[j]].b = 0;
								unusedpoint[pointIdxKsearch[j]] = false;
							}
						}

					}
				}
			}
		}
		else
		{
			continue;
		}
	}
	pcl::io::savePCDFileASCII("cur_cloud.pcd", *cur_cloud);
	cout << "cur success" << endl;
	//法线、曲率、颜色、坐标有了，开始过分割 循环到后面老是一个点一个类
	//pcl::PointCloud<PointTL>::Ptr oversegcloud(new pcl::PointCloud<PointTL>);
	//oversegcloud->resize(cloud->points.size());
	//vector<int> seedpoints;
	//vector<int> unusedpoints(cloud->points.size());
	//kdtree->setInputCloud(cloud);
	//int K = 20;
	//int K2 = 200;
	//map<int, vector<int>> facetlabel;
	//int facetlabelcount = 1;
	//for (int i = 0; i < cloud->points.size(); i++)
	//{
	//	unusedpoints[i] = i;
	//}
	//while (unusedpoints.size() > 0)
	//{
	//	int randpoint = Random(unusedpoints.size());
	//	unusedpoints.erase(unusedpoints.begin() + randpoint);
	//	facetlabel[facetlabelcount].push_back(randpoint);
	//	oversegcloud->points[randpoint].x = cloud->points[randpoint].x;
	//	oversegcloud->points[randpoint].y = cloud->points[randpoint].y;
	//	oversegcloud->points[randpoint].z = cloud->points[randpoint].z;
	//	oversegcloud->points[randpoint].label = facetlabelcount;
	//	vector<int> pointIdxKsearch(K);
	//	vector<float> pointNKNSquaredDistance(K);//储存近邻点对应距离平方
	//	int localMaxCurPoint = 0;
	//	vector<int> pointIdxKsearch2(K2);
	//	vector<float> pointNKNSquaredDistance2(K2);
	//	if (kdtree->nearestKSearch(cloud->points[randpoint], K, pointIdxKsearch,pointNKNSquaredDistance) > 0)
	//	{
	//		//找到局部曲率最大的点当种子点
	//		for (int i = 0; i < pointIdxKsearch.size() - 1; i++)
	//		{
	//			if (normals->points[pointIdxKsearch[i]].curvature < normals->points[pointIdxKsearch[i + 1]].curvature)
	//			{
	//				localMaxCurPoint = i + 1;
	//			}
	//		}
	//		seedpoints.push_back(localMaxCurPoint);
	//		facetlabel[facetlabelcount].push_back(localMaxCurPoint);
	//		//开始找种子点旁边的相似点
	//		if (kdtree->nearestKSearch(cloud->points[randpoint], K2, pointIdxKsearch2, pointNKNSquaredDistance2) > 0)
	//		{
	//			for (int i = 0; i < pointIdxKsearch2.size(); i++)
	//			{
	//				vector<int>::iterator usedit;
	//				usedit = find(unusedpoints.begin(), unusedpoints.end(), pointIdxKsearch2[i]);
	//				if (usedit != unusedpoints.end())
	//				{
	//					//欧式距离差
	//					float Ddistance = sqrt(pow(cloud->points[localMaxCurPoint].x - cloud->points[pointIdxKsearch2[i]].x, 2) +
	//						pow(cloud->points[localMaxCurPoint].y - cloud->points[pointIdxKsearch2[i]].y, 2) +
	//						pow(cloud->points[localMaxCurPoint].z - cloud->points[pointIdxKsearch2[i]].z, 2));
	//					//颜色差异
	//					//float Dcolor = sqrt(3 * pow(cloud->points[localMaxCurPoint].r - cloud->points[pointIdxKsearch2[i]].r, 2) +
	//						//4 * pow(cloud->points[localMaxCurPoint].g - cloud->points[pointIdxKsearch2[i]].g, 2) +
	//						//2 * pow(cloud->points[localMaxCurPoint].b - cloud->points[pointIdxKsearch2[i]].b, 2));
	//					//法线角度差
	//					float Dnormal = acos((normals->points[localMaxCurPoint].normal_x*normals->points[pointIdxKsearch2[i]].normal_x + normals->points[localMaxCurPoint].normal_y*normals->points[pointIdxKsearch2[i]].normal_y + normals->points[localMaxCurPoint].normal_z*normals->points[pointIdxKsearch2[i]].normal_z)) * 180 / 3.1415926; //
	//						//sqrt((pow(normals->points[localMaxCurPoint].normal_x, 2) + pow(normals->points[localMaxCurPoint].normal_y, 2) + pow(normals->points[localMaxCurPoint].normal_z, 2))*(pow(normals->points[pointIdxKsearch2[i]].normal_x, 2) + pow(normals->points[pointIdxKsearch2[i]].normal_y, 2) + pow(normals->points[pointIdxKsearch2[i]].normal_z, 2)))) * 180 / 3.1415926;
	//					//曲率差异
	//					float Dcurvature = abs(normals->points[localMaxCurPoint].curvature - normals->points[pointIdxKsearch2[i]].curvature);
	//					float wDistance = 0.1;
	//					float wColor = 0.1;
	//					float wNormal = 1;
	//					float wCurvature = 1;
	//					//总的差异
	//					//float Dall = sqrt(pow(wDistance*Ddistance, 2) + pow(wColor*Dcolor, 2) + pow(wNormal*Dnormal, 2) + pow(wCurvature*Dcurvature, 2));
	//					if (Dcurvature<0.1)
	//					{
	//						facetlabel[facetlabelcount].push_back(pointIdxKsearch2[i]);
	//						oversegcloud->points[pointIdxKsearch2[i]].x = cloud->points[pointIdxKsearch2[i]].x;
	//						oversegcloud->points[pointIdxKsearch2[i]].y = cloud->points[pointIdxKsearch2[i]].y;
	//						oversegcloud->points[pointIdxKsearch2[i]].z = cloud->points[pointIdxKsearch2[i]].z;
	//						oversegcloud->points[pointIdxKsearch2[i]].label = facetlabelcount;
	//						unusedpoints.erase(usedit);
	//					}
	//				}
	//			}
	//		}
	//	}
	//	facetlabelcount++;
	//}
	//Label2Color(oversegcloud);
	//cout << facetlabelcount << endl;
	/*for (int i = 0; i < cloud->points.size(); i++)
	{
		int randpoint = Random(unusedpoints.size());

	}*/
	//pointused.resize(cloud->points.size());
	return 1;
}

int TibetanFish::FindPrincipalDir(PointCloudT::Ptr cloud) {
	MatrixXf xyz(cloud->points.size(), 3);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		xyz(i, 0) = cloud->points[i].x;
		xyz(i, 1) = cloud->points[i].y;
		xyz(i, 2) = cloud->points[i].z;
	} 
	avg = xyz.colwise().mean();
	MatrixXf adjust(cloud->points.size(), 3);
	for (int i = 0; i < adjust.rows(); i++) {
		adjust(i, 0) = xyz(i, 0) - avg(0, 0);
		adjust(i, 1) = xyz(i, 1) - avg(0, 1);
		adjust(i, 2) = xyz(i, 2) - avg(0, 2);
	}
	MatrixXf cov = adjust.adjoint()*adjust;
	SelfAdjointEigenSolver<MatrixXf> eig(cov);
	MatrixXf vec, val;
	vec = eig.eigenvectors();
	val = eig.eigenvalues();
	rot = vec;
	PointCloudT::Ptr axis_cloud(new PointCloudT);
	int axis_num = 910;
	axis_cloud->resize(axis_num);
	int n = 0;
	for (int a = 0; a < 3; a++) {
		for (int j = -150; j <= 150; j++) {
			axis_cloud->points[n].x = avg(0, 0) + vec(0, a)*j;
			axis_cloud->points[n].y = avg(0, 1) + vec(1, a)*j;
			axis_cloud->points[n].z = avg(0, 2) + vec(2, a)*j;
			if (a == 2) {
				axis_cloud->points[n].r = 255;
				axis_cloud->points[n].g = 0;
				axis_cloud->points[n].b = 0;
			}
			else
			{
				axis_cloud->points[n].r = 255;
				axis_cloud->points[n].g = 255;
				axis_cloud->points[n].b = 255;
			}
			n++;
		}
	}
	cout << "n: " << n << endl;
	//pcl::io::savePCDFileASCII("output/axis_cloud.pcd", *axis_cloud);
	cout << avg << endl << rot << endl;
	cout << "axis success" << endl;
	return 0;
}

int TibetanFish::GetFishHead(PointCloud::Ptr cloud)
{
	PointCloud::Ptr head_template(new PointCloud);
	//读取头模板点云
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("input/tail2.pcd", *head_template) == -1) {
		PCL_ERROR("Couldnot read head file.\n");
		system("pause");
		return -1;
	}
	FeatureCloud obj_template;
	obj_template.setInputCloud(head_template);
	//预处理 删除距离较远的冗余点
	/*float depth_limit = 1.0;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, depth_limit);
	pass.filter(*cloud);*/
	//降采样
	/*const float voxel_grid_size = 0.5f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud(cloud);
	vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
	vox_grid.filter(*tempCloud);
	cloud = tempCloud;*/
	// Assign to the target FeatureCloud
	FeatureCloud target_cloud;
	target_cloud.setInputCloud(cloud);
	// Set the TemplateAlignment inputs
	TemplateAlignment template_align;
	template_align.addTemplateCloud(obj_template);
	template_align.setTargetCloud(target_cloud);
	// Find the best template alignment
	TemplateAlignment::Result best_alignment;
	template_align.findBestAlignment(best_alignment);
	//template_align.align(obj_template, best_alignment);
	//int best_index = template_align.findBestAlignment(best_alignment);
	//FeatureCloud best_template = obj_template;
	// Print the alignment fitness score (values less than 0.00002 are good)
	printf("Best fitness score: %f\n", best_alignment.fitness_score);
	Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	// Save the aligned template for visualization
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	pcl::transformPointCloud(*obj_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);


	pcl::io::savePCDFileBinary("transform.pcd", transformed_cloud);
	cout << "find head" << endl;
	return 1;
}

PointCloudT::Ptr TibetanFish::TransformCloud(PointCloudT::Ptr cloud)
{
	FindPrincipalDir(cloud);
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rotmat;
	Eigen::Matrix3f axismat;
	axismat <<  0, 0, 1,
				0, 1, 0, 
				1, 0, 0;
	rotmat = rot.inverse();
	rotmat = rotmat*axismat;
	transform(0, 3) = 0 - avg(0, 0);
	transform(1, 3) = 0 - avg(0, 1);
	transform(2, 3) = 0 - avg(0, 2);
	PointCloudT::Ptr transformaxis(new PointCloudT);
	pcl::transformPointCloud(*cloud, *transformaxis, transform);
	PointCloudT::Ptr rotaxis(new PointCloudT);
	rotaxis->resize(transformaxis->points.size());
	for (int i = 0; i < transformaxis->points.size(); i++)
	{
		rotaxis->points[i].z = transformaxis->points[i].z*rotmat(0, 0) + transformaxis->points[i].y*rotmat(0, 1) + transformaxis->points[i].x*rotmat(0, 2);
		rotaxis->points[i].y = transformaxis->points[i].z*rotmat(1, 0) + transformaxis->points[i].y*rotmat(1, 1) + transformaxis->points[i].x*rotmat(1, 2);
		rotaxis->points[i].x = transformaxis->points[i].z*rotmat(2, 0) + transformaxis->points[i].y*rotmat(2, 1) + transformaxis->points[i].x*rotmat(2, 2);
		rotaxis->points[i].rgba = transformaxis->points[i].rgba;
	}
	//pcl::io::savePCDFileASCII("output/rotfish.pcd", *rotaxis);
	cout<<"success" << endl;
	return rotaxis;
}

int TibetanFish::ProjectionFish(PointCloudT::Ptr cloud)
{

	return 0;
}

int TibetanFish::DrawaLine()
{
	PointCloudT::Ptr sortcloud(new PointCloudT);
	sortcloud->resize(100);
	for (int i = 0; i < 100; i++)
	{
		sortcloud->points[i].x = -76;
		sortcloud->points[i].y = i - 50;
		sortcloud->points[i].z = 0;
		sortcloud->points[i].r = 255;
		sortcloud->points[i].g = 255;
		sortcloud->points[i].b = 255;
	}
	pcl::io::savePCDFileASCII("weibing.pcd", *sortcloud);
	cout << "weibing" << endl;
	return 0;
}

PointCloudT::Ptr TibetanFish::SortByX(PointCloudT::Ptr cloud)
{
	for (int i = 0; i < cloud->points.size(); i++)
	{
		for (int j = 0; j < cloud->points.size() - 1 - i; ++j)
		{
			if (cloud->points[j].x > cloud->points[j + 1].x)
			{
				swap(cloud->points[j], cloud->points[j + 1]);
			}
		}
	}
	return cloud;
}

PointCloudT::Ptr TibetanFish::ReadObj(string filename)
{
	clock_t start, endt;
	start = clock();
	ifstream objfile(filename);
	if (!objfile.is_open())
	{
		cout << "open objfile error!" << endl;
		return FALSE;
	}
	string line;
	_xyz v;
	_xy vt;
	_normal vn;
	_trif f;
	Mat textureImg;
	map<int, int> vmap;//保存顶点和UV对应的索引
	//读取obj，存V VT VN F
	while (getline(objfile, line))
	{
		if (line.length() < 2)
			continue;
		istringstream ss(line);
		string type;
		ss >> type;
		if (type == "#")
		{
			continue;
		}
		else if (type == "v")
		{
			ss >> v.x >> v.y >> v.z;
			V.push_back(v);
		}
		else if (type == "vt")
		{
			ss >> vt.x >> vt.y;
			VT.push_back(vt);
		}
		else if (type == "vn")
		{
			ss >> vn.nx >> vn.ny >> vn.nz;
			VN.push_back(vn);
		}
		else if (type == "mtllib")
		{
			string mtlname;
			string folderpath = filename;
			ss >> mtlname;
			string::size_type position;
			position = mtlname.find_first_of(".");
			position = mtlname.find(".");
			if (position == 0)
			{
				mtlname = mtlname.erase(position, 2);
			}
			position = folderpath.find_last_of("/");
			if (position != folderpath.npos)
			{
				mtlname = folderpath.erase(position + 1, -1) + mtlname;
			}
			//mtlname= "input/obj/" + mtlname;
			ifstream mtlfile(mtlname);
			if (!mtlfile.is_open())
			{
				cout << "open mtlfile error!" << endl;
				return FALSE;
			}
			string mtlline;
			string texturename;
			while (getline(mtlfile, mtlline))
			{
				istringstream mtlss(mtlline);
				string map_kd;
				mtlss >> map_kd;
				if (map_kd == "map_Kd")
				{
					mtlss >> texturename;
					break;
				}
			}
			mtlfile.close();
			texturename = folderpath + texturename;

			//texturename = filename.replace(filename.find("."), 4, texturename);
			textureImg = imread(texturename, 1);
			if (textureImg.data == NULL)
			{
				cout << "can't find texture file" << endl;
				return FALSE;
			}
			TEXWIDTH = textureImg.cols;
			TEXHEIGHT = textureImg.rows;
		}
		else if (type == "f")
		{
			string s1, s2, s3;
			ss >> s1 >> s2 >> s3;
			for (int i = 0; i < s1.size(); i++)
			{
				if (s1[i] == '/')
				{
					s1[i] = ' ';
				}
			}
			istringstream temp1(s1);
			temp1 >> f.v[0] >> f.t[0] >> f.n[0];
			for (int i = 0; i < s2.size(); i++)
			{
				if (s2[i] == '/')
				{
					s2[i] = ' ';
				}
			}
			istringstream temp2(s2);
			temp2 >> f.v[1] >> f.t[1] >> f.n[1];
			for (int i = 0; i < s3.size(); i++)
			{
				if (s3[i] == '/')
				{
					s3[i] = ' ';
				}
			}
			istringstream temp3(s3);
			temp3 >> f.v[2] >> f.t[2] >> f.n[2];
			for (int i = 0; i < 3; i++)
			{
				vmap[f.v[i]] = f.t[i];
			}
			F.push_back(f);

		}
		else
		{
			continue;
		}
	}
	objfile.close();

	//转pcd
	PointCloudT::Ptr objcloud(new PointCloudT);
	objcloud->resize(V.size());
	for (int i = 0; i < V.size(); i++)
	{
		objcloud->points[i].x = V[i].x;
		objcloud->points[i].y = V[i].y;
		objcloud->points[i].z = V[i].z;
		//计算颜色
		int index = vmap[i + 1];//UV的索引
		if (index > 0 && index < VT.size()) {
			int x = VT[index - 1].x*TEXWIDTH;
			int y = TEXHEIGHT - VT[index - 1].y*TEXHEIGHT;
			if (x >= TEXWIDTH || y >= TEXHEIGHT || x < 0 || y < 0)
			{
				cerr << "out of img" << endl;
				return FALSE;
			}
			objcloud->points[i].r = textureImg.at<Vec3b>(y, x)[2];
			objcloud->points[i].g = textureImg.at<Vec3b>(y, x)[1];
			objcloud->points[i].b = textureImg.at<Vec3b>(y, x)[0];
		}

	}
	pcl::io::savePCDFileASCII("output/obj/huchunliefuyu.pcd", *objcloud);
	endt = clock();
	cout << "success: " <<(double)(endt - start) / CLOCKS_PER_SEC << endl;

	return objcloud;
}

int TibetanFish::SegFishOnly()
{
	int pcdsize = V.size();
	PointCloud::Ptr cloud(new PointCloud);
	cloud->resize(pcdsize);
	for (int i = 0; i < pcdsize; ++i)
	{
		cloud->points[i].x = V[i].x;
		cloud->points[i].y = V[i].y;
		cloud->points[i].z = V[i].z;

	}
	return 1;
}

int TibetanFish::ObjToMat(string filename)
{
	PointCloudT::Ptr objcloud(new PointCloudT);
	objcloud= ReadObj(filename);
	cout << objcloud->points.size() << endl;
	PointCloudT::Ptr transformcloud(new PointCloudT);
	transformcloud = TransformCloud(objcloud);
	string::size_type position,posdot;
	string pcdname;
	string folderpath = filename;
	//position = folderpath.find_last_of("/");
	posdot = folderpath.find(".");
	folderpath.erase(posdot, 4);
	//posdot = filename.find_last_of("/");
	//filename.erase(posdot, 1);
	if (posdot != folderpath.npos)
	{
		pcdname = folderpath + ".pcd";
	}
	pcl::io::savePCDFileASCII(pcdname, *transformcloud);
	cout << "mat success" << endl;
	return 1;
}