#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <stdlib.h>  
#include <cmath>
#include <limits.h>  
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <set>
#include <opencv2/opencv.hpp>
#include "FeatureCloud.h"

using namespace std;
using namespace Eigen;
using namespace cv;

#define Random(x) (rand() % x)

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZL PointTL;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
typedef pcl::PointXYZINormal WeightSACPointType;

struct RGB {
	int r;
	int g;
	int b;
};
//存顶点信息
struct _xyz {
	double x;
	double y;
	double z;
	int r=0;
	int g=0;
	int b=0;
};
//纹理坐标
struct _xy {
	double x;
	double y;
};
//法线
struct _normal {
	double nx;
	double ny;
	double nz;
};
//面
struct _trif {
	int v[3];
	int t[3];
	int n[3];
};

class TibetanFish {
public:
	int RemovePlane(string plyfile);
	void FishOnlyLCCP(string plyfile);
	int FishOnlyRegion(string plyfile);
	int FishOnlyRegionRGB(string plyfile);
	int FishDownSample(pcl::PointCloud<PointT>::Ptr cloud);
	int FishOverSegmentation(pcl::PointCloud<PointT>::Ptr cloud);
	int Point2Voxel(PointCloudT::Ptr cloud);
	int Label2Color(pcl::PointCloud<PointTL>::Ptr cloud);
	int FacetSegmentation(PointCloudT::Ptr cloud);
	int FindPrincipalDir(PointCloudT::Ptr cloud);
	int GetFishHead(PointCloud::Ptr cloud);
	PointCloudT::Ptr TransformCloud(PointCloudT::Ptr cloud);//把鱼放到坐标轴上
	int ProjectionFish(PointCloudT::Ptr cloud);
	PointCloudT::Ptr SortByX(PointCloudT::Ptr cloud);
	int DrawaLine();
	PointCloudT::Ptr ReadObj(string filename);
	int SegFishOnly();
	int ObjToMat(string filename);

private:
	MatrixXf avg;
	MatrixXf rot;
	vector<_xyz> V;//存顶点坐标
	vector<_xy> VT;//存纹理坐标
	vector<_normal> VN;//法向量
	vector<_trif> F;//面
	int TEXWIDTH;
	int TEXHEIGHT;
};