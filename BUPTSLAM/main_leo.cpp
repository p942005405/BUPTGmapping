#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <fstream>
#include <time.h>
#include <utility>      // std::pair
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "gridslamprocessor.h"

#include <opencv/highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>



#include "harray2d.h"


using namespace GMapping;
using namespace std;
using namespace cv;

// For test
void main_test()
{
	HierarchicalArray2D<double> *test0;

	HierarchicalArray2D<double> *test1;

	test0 = new HierarchicalArray2D<double>(1000,1000,4);

	test1 = new HierarchicalArray2D<double>(*test0);

	return;
}


void UpdateMAP()
{
	GMapping::ScanMatcher matcher;
}

void Transform(float& x, float& y, float& theta)
{
	float x1, y1, theta1;
//	double s = -1.0;
//	double c =  0.0;

//	double s = sin(-2.012385);
//	double c = cos(-2.012385);

	float s = sin(theta);
	float c = cos(theta);

	x1 =  c*x+s*y;
	y1 = -s*x+c*y;

	x = x1;
	y = y1;
}



int main(int argc, const char * const * argv)
{
//	double xmin =  -100.;
//	double ymin =  -100.;
//	double xmax =   100.;
//	double ymax =   100.;
//	double delta = 0.05 ;

	double xmin = -20;
	double ymin = -20;
	double xmax = 20.;
	double ymax = 20.;
	double delta = 0.05;

	//scan matching parameters
	double sigma = 0.05;
	double maxrange = 6; /* 控制了 registerScan 中的  m_laserMaxRange 值 */
	double maxUrange = 50.;
	double regscore = 1e4;
	double lstep = .05;
	double astep = .05;
	int kernelSize = 1;
	int iterations = 5;
	double critscore = 0.;
	double maxMove = 1.;
	double lsigma = .075;
	double ogain = 3;
	int lskip = 0;

	//motion model parameters 运动模型参数
	// double srr = 0.01, srt = 0.01, str = 0.01, stt = 0.01;
	double srr = 0, srt = 0, str = 0, stt = 0;

	//particle parameters
	int particles = 30;

	//gfs parameters
	// double angularUpdate = 0.5;
	double angularUpdate = 0; /* 角度更新的条件 */
	// double linearUpdate = 1;
	double linearUpdate = 0;    /* 线性更新的条件 */

	double resampleThreshold = 0.5;
	bool generateMap = true;

	static SensorMap m_sensorMap;
	GridSlamProcessor* processor = new GridSlamProcessor;

	// unsigned int gsp_laser_beam_count_ = 360; // 光束的数目
	// unsigned int gsp_laser_beam_count_ = 180;
	unsigned int gsp_laser_beam_count_ = 224;
	// double  gsp_laser_angle_increment_ =   1; // 每个光束的角度差别
	// double  gsp_laser_angle_increment_ = M_PI / 180;
	double  gsp_laser_angle_increment_ = M_PI / 180 / 2; /* 激光直接的角度差为 0.5 度 */
	double  maxRange_ = 6; // 激光的最大距离

	/* 定义激光雷达和机器人的位置关系，这里全部设置为零，表示两者重合 */
	/* 这个位置参数在 ScanMatcher 中会使用                            */
	GMapping::OrientedPoint gmap_pose(0.0, 0.0, 0.0);

	/* 定义激光器 */
	GMapping::RangeSensor* gsp_laser_;

	gsp_laser_ = new GMapping::RangeSensor("FLASER",
		gsp_laser_beam_count_,
		fabs(gsp_laser_angle_increment_),
		gmap_pose,
		0.0,
		maxRange_);

	m_sensorMap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));

	processor->setSensorMap(m_sensorMap); /* 在 GridSlamProcessor 中设置雷达数据 */

	/* 设置参数 */
	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	
	/* 设置运动模型参数 */
	processor->setMotionModelParameters(srr, srt, str, stt);
	
	/* 设置更新距离参数 */
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);

	/* 设置是否生成地图 */
	processor->setgenerateMap(generateMap);
	
	// OrientedPoint initialPose(xmin + xmax / 2, ymin + ymax / 2, 0);
	// OrientedPoint initialPose(-20, -20, 0);
	OrientedPoint initialPose(1.928, -3.848, 3.099356);

	//INITIALIZATION
	/* xmin, ymin, xmax, ymax 都是世界坐标系的单位 */
	processor->init(particles, xmin, ymin, xmax, ymax, delta, initialPose);

	GMapping::sampleGaussian(1, time(NULL));

	GridSlamProcessor* ap, *copy = processor->clone();
	ap = processor; processor = copy; copy = ap;

	/* 构造输入的雷达数据 */
	double time_r = 0.0;

	/*文件读写*/
	ifstream infile;
	// string fileName = "mit-killian.txt";
	string fileName = "Leo_Data.txt";
	infile.open(fileName.data());
	assert(infile.is_open());
	string s;
	
	bool running = true;
	int nCount = 0;

	//float u =  1.928416504;
	//float v = -3.848853027;
	//float theta0 = 3.099356 - M_PI/2;

	while (getline(infile, s))
	{
		printf("nCount = %d\n", nCount);
		/*读取传感器的里程计信息*/
		// getline(infile, s);
		vector<string> data;
		boost::split(data, s, boost::is_any_of(" "));
		double ranges_double[250];
		
		for (int i = 0; i < 224; i++) 
		{
			ranges_double[i] = atof(const_cast<const char *>(data.at(i + 3).c_str()));
			ranges_double[i] = ranges_double[i]/ 1000; /* 将毫米转换成米 */

			cout << ranges_double[i] << endl;
		}

		float pos[3];
		for (int i = 0; i < 3; i++) 
		{
			pos[i] = atof(const_cast<const char *>(data.at(i).c_str()));
			cout << pos[i] << endl;
		}

		float temp;

		pos[0] = pos[0] / 1000;  /* 将毫米转换成米 */
		pos[1] = pos[1] / 1000;  /* 将毫米转换成米 */

		//cout << pos[0] << endl;
		//cout << pos[1] << endl;

		// time_r = atof(const_cast<const char *>(data.at(188).c_str()))*1000;

		time_r = 0;

		GMapping::RangeReading reading(gsp_laser_beam_count_,ranges_double,	gsp_laser_,	time_r);
		
		/* 设置进行激光采集的时候，机器人的位置坐标 */
		//pos[0] = pos[0] - u;
		//pos[1] = pos[1] - v;

		//Transform(pos[0], pos[1], theta0);

		//pos[0] = pos[0] - 20 ;
		//pos[1] = pos[1] - 20 ;
		//pos[2] = pos[2] - theta0;

		gmap_pose.x     = pos[0] ;
		gmap_pose.y     = pos[1] ;
		gmap_pose.theta = pos[2];

		reading.setPose(gmap_pose);

		double duration;
		
		duration = static_cast<double>(cv::getTickCount());
		bool processed = processor->processScan(reading);
		duration = static_cast<double>(cv::getTickCount()) - duration;
		duration /= cv::getTickFrequency(); // the elapsed time in ms
		printf("Processing time = %f\n", duration);

		if (nCount == 274)
		{
			printf("nCount = %d\n", nCount);
		}

		double occ_thresh_ = 0.25;
		float *imagedata;
		int image_w, image_h;
		Size dsize = Size(1000,1000);

		Mat image2 = Mat(dsize, CV_32F);

		/* 搜索最佳粒子索引 */
		// if (0)
		if (processed)
		{
			unsigned int best_idx = processor->getBestParticleIndex();

			double theta = processor->getParticles()[best_idx].pose.theta;
			theta = atan2(sin(theta), cos(theta));

			cerr << "Estimate Pose= " << processor->getParticles()[best_idx].pose.x << " " << processor->getParticles()[best_idx].pose.y << " " << theta << endl;

			printf("!!!!!!!!!!!!!!!best_idx = %d\n", best_idx);
			{
				DoubleArray2D* mymap = processor->getParticles()[best_idx].map.toDoubleArray();

				image_h = mymap->getXSize();
				image_w = mymap->getYSize();

				printf("MapSizeX = %d\n", mymap->getXSize());
				printf("MapSizeY = %d\n", mymap->getYSize());

				imagedata = (float *)malloc(image_w*image_h*sizeof(float));

				memset(imagedata, 0x00, image_w*image_h*sizeof(float));

				for (int j = 0; j < image_h; j++)
				{
					for (int i = 0; i < image_w; i++)
					{
						double occ = mymap->cell(j, i);

						if (occ < 0)
						{
							/* 未开发区域 */
							imagedata[j*image_w + i] = 0.5;  
						}
						else if (occ > occ_thresh_)     /* occ_thresh_ = 0.25 */ 
						{
							/* 障碍物区域 */
							imagedata[j*image_w + i] = 0;
						}
						else
						{
							/* free 区域 */
							imagedata[j*image_w + i] = 1;  
						}
					}
				}

				Mat map(image_h, image_w, CV_32F, imagedata);
				resize(map, image2, dsize);
				imshow("Image", image2);

				char ch = char(cvWaitKey(33));

				if (ch == 'q')
					return 0;

				delete mymap;
				free(imagedata);
			}
		}
		nCount++;
	}




	return 0;
}