#pragma once
#define BOOST_USE_WINDOWS_H
#define NOMINMAX
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
//！opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <io.h> 


typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;
using namespace std;

/*
* @brief 点云边界
*/
struct Bounds
{
	double min_x;
	double min_y;
	double min_z;
	double max_x;
	double max_y;
	double max_z;
	int min_intensity;
	int max_intensity;
	Bounds()
	{
		min_x = min_y = min_z = DBL_MAX;
		max_x = max_y = max_z = DBL_MIN;
		min_intensity = INT_MAX;
		max_intensity = INT_MIN;
	}
};

/*
* @brief 点云体素
*/
struct Voxel
{
	std::vector<int> point_id;
	std::vector<float> vec_min_z;
	float min_z;
	float max_z;
	float dertaz;
	float min_z_x;
	float min_z_y;
	float neighbormin_z;
	float mean_min_z;	// 与周围几个体素的最小z值的平均值
	float points_number;
	float mean_z;
	float cent_x;
	float cent_y;
	float ground_ratio;
	float ave_min_z;
	float sum_z;
	float real_min_z;
	Voxel()
	{
		min_z_x = min_z_y = neighbormin_z = mean_z = max_z = 0.f;
		points_number = 0;
		dertaz = 0.0;
		mean_min_z = -1000;
		cent_x = -1000;
		cent_y = -1000;
		min_z = 0.0;
		ground_ratio = 0;
		ave_min_z = 0.0;
		sum_z = 0.0;
		real_min_z = 0.0;
	}
};

/*
* @brief 点云投影照片类
*/
class Cloud2Figure
{
public:
	Cloud2Figure();
	~Cloud2Figure();
	/**
	 * @brief  读取点云
	 * @param file_name [in] 文件名 
	 * @param point_cloud  [] 
	 * @param bound_3d 
	 * @return 
	*/
	bool readTxtFile2Cloud(const string& file_name, const pcXYZIPtr& point_cloud);

	/*
	*  @brief 提取地面点函数
	*  param[in] cloud:输入点云  
	*  param[in] ground_cloud:地面点点云
	*  param[in] bounds:外边界结构体
	*/
	void extractGroundPoints(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud, const Bounds& bounds);
	/*
	*  @brief 获得点云外边界函数
	*  param[in] cloud:输入点云
	*  param[in] bounds:外边界结构体
	*/
	void getBoundsOfCloud(const pcXYZIPtr& cloud, Bounds& bound_3d_temp);
	/*
	*  @brief 强度值过滤函数
	*  param[in] cloud:输入点云
	*  param[in] bounds:外边界结构体
	*/
	void removalOutlinesPoints(const pcXYZIPtr& cloud, pcXYZIPtr& gcloud_filter, Bounds& bound_3d_temp, double intensity_k);
	/*!
	* @brief 点云转强度图像函数
	* param[in] cloud：点云指针  
	* param[in] resolution：点云转图像分辨率  
	* param[out] img: 点云生成的强度图像
	*/
	cv::Mat pointCloud2imgI(const pcXYZIPtr& cloud, double resolution);

protected:

private:
	/*
	* @brief 计算格网函数
	* param[in] cloud:输入点云
	* param[in] min_x:最小x
	* param[in] min_y:最小y
	* param[in] row:行数
	* param[in] list:列数
	* param[in] num_voxel:格网数量
	* param[in] grid:体素结构体
	*/
	void getGrid(const pcXYZIPtr& cloud, double min_x, double min_y,
		int row, int list, int num_voxel, Voxel* grid);

	/*
	* @brief 分离地面点和非地面点函数
	* param[in] cloud:输入点云
	* param[in] ground_cloud:地面点云
	* param[in] grid:体素结构体
	* param[in] row:行数
	* param[in] list:列数
	* param[in] num_voxel:格网数量
	*/
	void segGroundNogroudPts(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud,
		Voxel* grid, int row, int list, int num_voxel);

	// 提取地面点默认参数
	float m_grid_res_;    //格网大小
	float m_max_height_diff_;   // 地面点最高阈值
	float m_max_nei_height_diff_;
};