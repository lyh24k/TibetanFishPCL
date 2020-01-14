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
	int RemovePlane(string plyfile);//剔除平面
	void FishOnlyLCCP(string plyfile);//LCCP分割
	int FishOnlyRegion(string plyfile);//区域生长分割
	int FishOnlyRegionRGB(string plyfile);//基于RGB的区域生长分割
	int FishDownSample(pcl::PointCloud<PointT>::Ptr cloud);//降采样
	int FishOverSegmentation(pcl::PointCloud<PointT>::Ptr cloud);//过分割
	int Point2Voxel(PointCloudT::Ptr cloud);//点变体素
	int Label2Color(pcl::PointCloud<PointTL>::Ptr cloud);//为分层的点云染色
	int FacetSegmentation(PointCloudT::Ptr cloud);//小面分割
	int FindPrincipalDir(PointCloudT::Ptr cloud);//PCA找主轴
	int GetFishHead(PointCloud::Ptr cloud);//粗配准找鱼头
	PointCloudT::Ptr TransformCloud(PointCloudT::Ptr cloud);//把鱼放到坐标轴上
	int ProjectionFish(PointCloudT::Ptr cloud);//投影
	PointCloudT::Ptr SortByX(PointCloudT::Ptr cloud);//按X轴排序
	int DrawaLine();//画线
	PointCloudT::Ptr ReadObj(string filename);//读取obj
	int SegFishOnly();
	int ObjToMat(string filename);//转成pcd 后面要转成图片

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
