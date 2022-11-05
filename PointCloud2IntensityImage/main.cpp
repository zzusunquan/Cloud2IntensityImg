#include "cloud2Figure.h"

int main()
{
	// 生成强度图分辨率
	const double resolution = 0.05; 
	// 强度值过滤系数
	const double intensity_coeffocoent = 10; 
	// 点云文件路径
	const std::string las_path = "demo.txt";

	// 原始点云
	pcXYZIPtr cloud(new pcXYZI); 
	// 地面点点云
	pcXYZIPtr gcloud(new pcXYZI);
	// 地面点点云去噪后
	pcXYZIPtr gcloud_filter(new pcXYZI);

	// 读取原始点云数据  转成相对坐标

	Cloud2Figure* Cf = new Cloud2Figure();
	Bounds bound_3d;

	if (!Cf->readTxtFile2Cloud(las_path, cloud))
	{
		return 0; // 读取txt点云文件
	}
	// 获取边界 以及点云坐标转换
	Cf->getBoundsOfCloud(cloud, bound_3d);
	// 提取地面点云
	Cf->extractGroundPoints(cloud, gcloud, bound_3d);
	// 强度直通滤波
	Cf->removalOutlinesPoints(gcloud, gcloud_filter, bound_3d, intensity_coeffocoent);
	// 投影生成图像
	cv::Mat imgI = Cf->pointCloud2imgI(gcloud_filter, resolution);
	cv::imwrite("intensity.bmp", imgI);
	delete Cf;
}