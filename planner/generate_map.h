#pragma once
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <random>

#define random_seed (std::chrono::system_clock::now().time_since_epoch().count())

class GenerateMap
{
public:
	GenerateMap()
	{
		std::cout << "start to construct GenerateMap" << std::endl;
	}
	~GenerateMap()
	{
		std::cout << "start to destroy GenerateMap" << std::endl;
	}

	void randomMap(cv::Mat &map, const int obs_num)
	{
		int maxx = map.rows;
		int maxy = map.cols;

		std::default_random_engine ex(random_seed);
		std::uniform_int_distribution<unsigned>ux(0, maxx - 1);
		std::default_random_engine ey(random_seed);
		std::uniform_int_distribution<unsigned>uy(0, maxy - 1);

		for (int i = 0; i < obs_num; i++)
		{
			int x = ux(ex);
			int y = uy(ey);
			// std::cout << x << "  " << y << std::endl;
			map.at<uchar>(x, y) = 255;
		}
	}


};