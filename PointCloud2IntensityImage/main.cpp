#include "cloud2Figure.h"

int main()
{
	// ����ǿ��ͼ�ֱ���
	const double resolution = 0.05; 
	// ǿ��ֵ����ϵ��
	const double intensity_coeffocoent = 10; 
	// �����ļ�·��
	const std::string las_path = "demo.txt";

	// ԭʼ����
	pcXYZIPtr cloud(new pcXYZI); 
	// ��������
	pcXYZIPtr gcloud(new pcXYZI);
	// ��������ȥ���
	pcXYZIPtr gcloud_filter(new pcXYZI);

	// ��ȡԭʼ��������  ת���������

	Cloud2Figure* Cf = new Cloud2Figure();
	Bounds bound_3d;

	if (!Cf->readTxtFile2Cloud(las_path, cloud))
	{
		return 0; // ��ȡtxt�����ļ�
	}
	// ��ȡ�߽� �Լ���������ת��
	Cf->getBoundsOfCloud(cloud, bound_3d);
	// ��ȡ�������
	Cf->extractGroundPoints(cloud, gcloud, bound_3d);
	// ǿ��ֱͨ�˲�
	Cf->removalOutlinesPoints(gcloud, gcloud_filter, bound_3d, intensity_coeffocoent);
	// ͶӰ����ͼ��
	cv::Mat imgI = Cf->pointCloud2imgI(gcloud_filter, resolution);
	cv::imwrite("intensity.bmp", imgI);
	delete Cf;
}