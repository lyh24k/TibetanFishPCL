#include "TibetanFish.h"

int main()
{
	TibetanFish fish;
	//f.RemovePlane("input/gaerxianzhuiwenyexuyu1.ply");
	//f.FishOnlyRegion("langcangjiangliefuyu2.ply");
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPLYFile<PointT>("output/langcangjiangliefuyu2.ply", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return -1;
	}
	fish.FishDownSample(cloud);
	/*
	PointCloud::Ptr xyz_cloud(new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("input/cloud_simple.pcd", *xyz_cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return -1;
	}*/
	//f.ProjectionFish(cloud);
	//fish.ObjToMat("input/obj/luofuyexuyu.obj");
	//f.TransformCloud(cloud);
	//f.GetFishHead(xyz_cloud);
	//f.FishDownSample(cloud);
	//f.FindPrincipalDir(cloud);
	//f.FacetSegmentation(cloud);
	//f.FishOnlyRegion("cloud_simple.pcd");
	getchar();
}