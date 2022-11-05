#include "Cloud2Figure.h"

using namespace std;

Cloud2Figure::Cloud2Figure()
{
	m_grid_res_ = 1.5; 
	m_max_height_diff_ = 0.25;
	m_max_nei_height_diff_ = 1.0;

}

/*!
* @brief ��ȡlas��ʽ�������ݺ���
* param[in] file_name:las�ļ�  point_cloud:����ָ��   bound_3d:��С��߽�
*/
bool Cloud2Figure::readTxtFile2Cloud(const string& file_name, const pcXYZIPtr& point_cloud)
{
	Bounds bound_3d;
	std::ifstream ifs;
	ifs.open(file_name, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "Failed to load cloud File" << endl;
	}
	string line;
	pcl::PointXYZI point;
	while (getline(ifs, line))
	{
		stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		ss >> point.intensity;
		point_cloud->emplace_back(point);
	}
	for (auto p : point_cloud->points)
	{
		bound_3d.min_x = std::min(double(p.x), bound_3d.min_x);
		bound_3d.min_y = std::min(double(p.y), bound_3d.min_y);
		bound_3d.min_z = std::min(double(p.z), bound_3d.min_z);
		bound_3d.max_x = std::max(double(p.x), bound_3d.max_x);
		bound_3d.max_y = std::max(double(p.y), bound_3d.max_y);
		bound_3d.max_z = std::max(double(p.z), bound_3d.max_z);
		bound_3d.max_intensity = std::max(int(p.intensity), bound_3d.max_intensity);
	}
	// ��ȡ�������Χ�е����ĵ�����
	double center_x = 0.5 * (bound_3d.max_x + bound_3d.min_x);
	double center_y = 0.5 * (bound_3d.max_y + bound_3d.min_y);

	// �����������ĵ�ƽ���� XOYƽ��ԭ��
	for (auto &p: point_cloud->points)
	{
		p.x -= center_x;
		p.y -= center_y;
	}
	// ��� ǿ����Ϣ�Ƿ����
	if (point_cloud->points[0].intensity == 0) //check if the intensity exsit
	{
		cout << "Warning! Point cloud intensity may not be imported properly, check the scalar field's name.\n" << endl;
	}
	return true;
}


/*
*  @brief ��·������ȡ����
*  param[in] cloud:�������
*  param[in] ground_cloud:��·�����
*  param[in] bounds:��߽�ṹ��
*/
void Cloud2Figure::extractGroundPoints(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud, const Bounds& bounds)
{
	int row = ceil((bounds.max_y - bounds.min_y) / m_grid_res_);
	int list = ceil((bounds.max_x - bounds.min_x) / m_grid_res_);
	int num_voxel = row * list;
	Voxel* grid = new Voxel[num_voxel];

	for (int i = 0; i < num_voxel; i++)
	{
		grid[i].min_z = FLT_MAX;
		grid[i].mean_min_z = -1000;
		grid[i].cent_x = -1000;
		grid[i].cent_y = -1000;
		grid[i].ground_ratio = 0;
		grid[i].sum_z = 0.0;
	}
	getGrid(cloud, bounds.min_x, bounds.min_y, row, list, num_voxel, grid);
	segGroundNogroudPts(cloud, ground_cloud, grid, row, list, num_voxel);
	delete[]grid;
}

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
void Cloud2Figure::getGrid(const pcXYZIPtr& cloud, double min_x, double min_y,
	int row, int list, int num_voxel, Voxel* grid)
{
	// �������Ƹ���
	for (int i = 0; i < cloud->points.size(); i++)
	{
		int temp_row, temp_list, temp_num;
		temp_list = floor((cloud->points[i].x - min_x) / m_grid_res_);
		temp_row = floor((cloud->points[i].y - min_y) / m_grid_res_);
		temp_num = temp_row * list + temp_list;
		if (temp_num >= 0 && temp_num < num_voxel)
		{
			grid[temp_num].point_id.push_back(i);
			grid[temp_num].points_number++;
			grid[temp_num].vec_min_z.push_back(cloud->points[i].z);
			grid[temp_num].sum_z += cloud->points[i].z;
		}
	}

	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < list; j++)
		{
			if (grid[i * list + j].vec_min_z.size() == 0)
			{
				continue;
			}
			sort(grid[i * list + j].vec_min_z.begin(), grid[i * list + j].vec_min_z.end());
			grid[i * list + j].ave_min_z = grid[i * list + j].sum_z / grid[i * list + j].points_number;
			double min_z = min(grid[i * list + j].ave_min_z, grid[i * list + j].vec_min_z[grid[i * list + j].vec_min_z.size() / 10]);
			grid[i * list + j].real_min_z = grid[i * list + j].vec_min_z[0];
			grid[i * list + j].min_z = min_z;
			grid[i * list + j].neighbormin_z = min_z;
		}
	}

	// ȡ������Χ3*3������ƽ��ֵ
	for (int i = 0; i < row; i++)
	{
		int subrow[3];
		subrow[0] = max(0, i - 1);
		subrow[1] = i;
		subrow[2] = min(row - 1, i + 1);

		for (int j = 0; j < list; j++)
		{
			if (grid[i * list + j].point_id.size() == 0)
			{
				continue;
			}
			int subcol[3];
			subcol[0] = max(0, j - 1);
			subcol[1] = j;
			subcol[2] = min(list - 1, j + 1);

			vector<float> min_z_vec;
			float min_z_sum = 0.0;

			for (int m = 0; m < 3; m++)
			{
				for (int n = 0; n < 3; n++)
				{
					float min_z = grid[subrow[m] * list + subcol[n]].min_z;
					if (min_z != -1000 && min_z < 100000 && min_z > -1000)
					{
						min_z_vec.push_back(min_z);
						min_z_sum += min_z;
					}
				}
			}
			if (min_z_vec.size() > 0)
			{
				sort(min_z_vec.begin(), min_z_vec.end());
				if (min_z_vec.size() > 2)
				{
					grid[i * list + j].mean_min_z = (min_z_sum - min_z_vec[0] - min_z_vec[min_z_vec.size() - 1]) / (min_z_vec.size() - 2);
				}
				else
				{
					grid[i * list + j].mean_min_z = min_z_sum / min_z_vec.size();
				}
			}
		}
	}
}
/*
* @brief ��������ͷǵ���㺯��
* param[in] cloud:�������
* param[in] ground_cloud:�������
* param[in] grid:���ؽṹ��
* param[in] row:����
* param[in] list:����
* param[in] num_voxel:��������
*/
void Cloud2Figure::segGroundNogroudPts(const pcXYZIPtr& cloud, const pcXYZIPtr& ground_cloud,
	Voxel* grid, int row, int list, int num_voxel)
{
	int num_gcloud = 0;

	// ���е��Ƶ�������Ŀ
	int num_cloud_voxel = 0;

	// �������˵ĵ���������Ŀ
	int num_ground_initial = 0;

	// ��������ڸ����еı�����
	float sum_ground_ratio = 0;

	for (int i = 0; i < num_voxel; i++)
	{
		int grid_num_gcloud = 0;
		for (int j = 0; j < grid[i].point_id.size(); j++)
		{
			if (abs(cloud->points[grid[i].point_id[j]].z - grid[i].mean_min_z) < m_max_height_diff_ &&
				grid[i].min_z - grid[i].neighbormin_z < m_max_nei_height_diff_)
			{
				grid_num_gcloud++;
				num_ground_initial++;
			}

		}
		if (grid[i].point_id.size() > 0)
		{
			num_cloud_voxel++;
			grid[i].ground_ratio = 100.0 * grid_num_gcloud / grid[i].point_id.size();

			sum_ground_ratio += grid[i].ground_ratio;

		}
	}

	float ave_grid_num_gcloud = cloud->size() / num_cloud_voxel;
	float ave_ground_ratio = sum_ground_ratio / num_cloud_voxel;

	for (int i = 0; i < num_voxel; i++)
	{
		// ʹ�ø����е��Ƹ���������е�������ڱ������й���
		if (grid[i].point_id.size() > ave_grid_num_gcloud / 10 && grid[i].ground_ratio > ave_ground_ratio / 4)
		{
			for (int j = 0; j < grid[i].point_id.size(); j++)
			{
				if (abs(cloud->points[grid[i].point_id[j]].z - grid[i].mean_min_z) < m_max_height_diff_ &&
					grid[i].min_z - grid[i].neighbormin_z < m_max_nei_height_diff_)
				{
					ground_cloud->points.push_back(cloud->points[grid[i].point_id[j]]);
					num_gcloud++;
				}
			}
		}
	}
	ground_cloud->height = 1;
	ground_cloud->width = num_gcloud;
}

/*
*  @brief ��õ�����߽纯��
*  param[in] cloud:�������
*  param[in] bounds:��߽�ṹ��
*/
void Cloud2Figure::getBoundsOfCloud(const pcXYZIPtr& cloud, Bounds& bound_3d_temp)
{
	for (int i = 0; i < cloud->points.size(); i += 20)
	{
		// �����Сǿ��
		if (cloud->points[i].intensity > bound_3d_temp.max_intensity)
		{
			bound_3d_temp.max_intensity = cloud->points[i].intensity;
		}
		if (cloud->points[i].intensity < bound_3d_temp.min_intensity)
		{
			bound_3d_temp.min_intensity = cloud->points[i].intensity;
		}
		//�����Сxֵ
		if (cloud->points[i].x > bound_3d_temp.max_x)
		{
			bound_3d_temp.max_x = cloud->points[i].x;
		}
		if (cloud->points[i].x < bound_3d_temp.min_x)
		{
			bound_3d_temp.min_x = cloud->points[i].x;
		}
		//�����Сyֵ
		if (cloud->points[i].y > bound_3d_temp.max_y)
		{
			bound_3d_temp.max_y = cloud->points[i].y;
		}
		if (cloud->points[i].y < bound_3d_temp.min_y)
		{
			bound_3d_temp.min_y = cloud->points[i].y;
		}
	}
	// ��ȡ���ĵ�����
	double center_x = 0.5 * (bound_3d_temp.max_x + bound_3d_temp.min_x);
	double center_y = 0.5 * (bound_3d_temp.max_y + bound_3d_temp.min_y);
	// �������������
	for (int i = 0; i < cloud->points.size(); i++)
	{
		cloud->points[i].x -= center_x;
		cloud->points[i].y -= center_y;
	}
}

/*
*  @brief ǿ��ֵ���˺���
*  param[in] cloud:�������
*  param[in] bounds:��߽�ṹ��
*/
void Cloud2Figure::removalOutlinesPoints(const pcXYZIPtr& cloud, pcXYZIPtr& gcloud_filter, Bounds& bound_3d_temp, double intensity_k)
{
	double mean_intensity = 0.0;
	for (int i = 0; i < cloud->points.size(); i+=10)
	{
		mean_intensity += cloud->points[i].intensity;
	}
	mean_intensity /= cloud->size() / 10.0;

	double std_intensity = 0.0;
	for (int i = 0; i < cloud->size(); i += 10)
	{
		std_intensity += (cloud->points[i].intensity - mean_intensity) * (cloud->points[i].intensity - mean_intensity);
	}
	std_intensity /= cloud->size() / 10.0;
	std_intensity = sqrt(std_intensity);
	int intensity_max = min(bound_3d_temp.max_intensity + 1, (int)(mean_intensity + std_intensity * intensity_k));
	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("intensity");
	pass.setFilterLimits(0.0, intensity_max);
	pass.setFilterLimitsNegative(false);
	pass.filter(*gcloud_filter);
}

/*!
* @brief ����תǿ��ͼ����
* param[in] cloud������ָ��
* param[in] resolution������תͼ��ֱ���
* param[out] img: �������ɵ�ǿ��ͼ��
*/
cv::Mat Cloud2Figure::pointCloud2imgI(const pcXYZIPtr& cloud, double resolution)
{
	float max_x, max_y, min_x, min_y, max_i;
	max_x = max_y = max_i = FLT_MIN;
	min_x = min_y = FLT_MAX;

	for (int i = 0; i < cloud->points.size(); i += 20)
	{
		max_x = max(max_x, cloud->points[i].x);
		min_x = min(min_x, cloud->points[i].x);
		max_y = max(max_y, cloud->points[i].y);
		min_y = min(min_y, cloud->points[i].y);
		max_i = max(max_i, cloud->points[i].intensity);
	}

	double lx = max_x - min_x;					//���Ƴ���
	double ly = max_y - min_y;					//���ƿ��
	int rows = round(ly / resolution);			//ͼ��߶�
	int clos = round(lx / resolution);			//ͼ����

	cv::Mat img = cv::Mat::zeros(rows, clos, CV_8UC3);
	//ǿ�ȸ�������
	vector<vector<float>> vec_grid_intensity;

	//��������ռ� ����ʼ��
	vec_grid_intensity.resize(rows);
	//��ʼ������
	for (int i = 0; i < rows; i++)
	{
		vec_grid_intensity[i].resize(clos);
	}

	//���ν���ѹ�����ڸ���
	for (int i = 0; i < cloud->points.size(); i++)
	{
		int m = (max_x - cloud->points[i].y) / resolution;
		int n = (cloud->points[i].x - min_x) / resolution;

		if (m > 0 && m < rows && n > 0 && n < clos)
		{
			// �������еĵ��Ƶ����ǿ��ֵ��Ϊ������ǿ��ֵ
			vec_grid_intensity[m][n] = max(cloud->points[i].intensity, vec_grid_intensity[m][n]);
		}
	}

	//ǿ�����쵽0-255֮�丳ֵ
	for (int i = 0; i < rows; i++)
	{
		uchar* data = img.ptr<uchar>(i);
		for (int j = 0; j < clos; j++)
		{
			if (vec_grid_intensity[i][j] > 0)
			{
				int pixel = (int)(vec_grid_intensity[i][j] / max_i * 255);
				//��ͨ�����ظ�ֵ
				data[3 * j] = pixel;
				data[3 * j + 1] = pixel;
				data[3 * j + 2] = pixel;
			}
		}
	}
	return img;
}

Cloud2Figure::~Cloud2Figure()
{

}

