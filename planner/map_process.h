#pragma once
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

class MyImage
{
public:
	MyImage()
	{
		const int large_size = 50;

		cv::Mat map;
		map.create(10, 10, CV_8UC1);
		map.setTo(0);
		std::cout << "map size: " << map.rows << "  " << map.cols << std::endl;

		cv::namedWindow("planner", 0);
		cv::resizeWindow("planner", large_size * map.cols, large_size * map.rows);


		for (int i = 4; i < 6; i++)
		{
			for (int j = 5; j < 6; j++)
			{
				map.at<uchar>(i, j) = 255;
			}
		}
		std::cout << "before:" << std::endl;
		std::cout << map << std::endl;

		cv::Mat out;
		cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		cv::dilate(map, out, element);

		std::cout << "after:" << std::endl;
		std::cout << out << std::endl;

		//cv::Mat background;
		//background.create(100, 100, CV_8UC3);
		//background.setTo(255);

		cv::imshow("planner", map);
		cv::imshow("planner", out);
		//std::cout << "map " << map.rows << "  " << map.cols << std::endl;

		cv::waitKey(0);
		cv::destroyAllWindows();
	}

	~MyImage()
	{
		std::cout << "start to destroy MyImage" << std::endl;
	}
};