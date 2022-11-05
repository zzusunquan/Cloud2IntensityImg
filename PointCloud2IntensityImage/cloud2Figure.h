#pragma once
#define BOOST_USE_WINDOWS_H
#define NOMINMAX
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
//��opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <io.h> 


typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;
using namespace std;

/*
* @brief ���Ʊ߽�
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
* @brief ��������
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
	float mean_min_z;	// ����Χ�������ص���Сzֵ��ƽ��ֵ
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
* @brief ����ͶӰ��Ƭ��
*/
class Cloud2Figure
{
public:
	Cloud2Figure();
	~Cloud2Figure();
	/**
	 * @brief  ��ȡ����
	 * @param file_name [in] �ļ��� 
	 * @param point_cloud  [] 
	 * @param bound_3d 
	 * @return 
	*/
	bool readTxtFile2Cloud(const string& file_name, const pcXYZIPtr& point_cloud);

	/*
	*  @brief ��ȡ����㺯��
	*  param[in] cloud:�������  
	*  param[in] ground_cloud:��������
	*  param[in] bounds:��߽�ṹ��
	*/
	void extractGroundPoints(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud, const Bounds& bounds);
	/*
	*  @brief ��õ�����߽纯��
	*  param[in] cloud:�������
	*  param[in] bounds:��߽�ṹ��
	*/
	void getBoundsOfCloud(const pcXYZIPtr& cloud, Bounds& bound_3d_temp);
	/*
	*  @brief ǿ��ֵ���˺���
	*  param[in] cloud:�������
	*  param[in] bounds:��߽�ṹ��
	*/
	void removalOutlinesPoints(const pcXYZIPtr& cloud, pcXYZIPtr& gcloud_filter, Bounds& bound_3d_temp, double intensity_k);
	/*!
	* @brief ����תǿ��ͼ����
	* param[in] cloud������ָ��  
	* param[in] resolution������תͼ��ֱ���  
	* param[out] img: �������ɵ�ǿ��ͼ��
	*/
	cv::Mat pointCloud2imgI(const pcXYZIPtr& cloud, double resolution);

protected:

private:
	/*
	* @brief �����������
	* param[in] cloud:�������
	* param[in] min_x:��Сx
	* param[in] min_y:��Сy
	* param[in] row:����
	* param[in] list:����
	* param[in] num_voxel:��������
	* param[in] grid:���ؽṹ��
	*/
	void getGrid(const pcXYZIPtr& cloud, double min_x, double min_y,
		int row, int list, int num_voxel, Voxel* grid);

	/*
	* @brief ��������ͷǵ���㺯��
	* param[in] cloud:�������
	* param[in] ground_cloud:�������
	* param[in] grid:���ؽṹ��
	* param[in] row:����
	* param[in] list:����
	* param[in] num_voxel:��������
	*/
	void segGroundNogroudPts(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud,
		Voxel* grid, int row, int list, int num_voxel);

	// ��ȡ�����Ĭ�ϲ���
	float m_grid_res_;    //������С
	float m_max_height_diff_;   // ����������ֵ
	float m_max_nei_height_diff_;
};