/*#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
//#include <Eigen/Eigen>
#include "astar.h"
#include "map_process.h"
#include "generate_map.h"





// windows下返回的是100纳秒
#define PLANNERMS (std::chrono::system_clock::now().time_since_epoch().count() / 10000)

int main()
{
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);

	std::cout << "planner start" <<std::endl;

	int a = 1, b = 1;
	double dist = std::sqrt(a * a + b * b);
	std::cout << "dist: " << dist << std::endl;
	
	// std::shared_ptr<MyImage> test = std::make_shared<MyImage>();
	// return 0;



	std::shared_ptr<planner::Astar> astar_ptr = std::make_shared<planner::Astar>();

	const int large_size = 2;


	planner::NaviData navi_data;
	navi_data.start = planner::Grid(0, 0);
	navi_data.goal = planner::Grid(250, 370);

	cv::Mat map;
	map.create(300, 400, CV_8UC1);
	map.setTo(0);
	std::cout << "map size: " << map.rows << "  " << map.cols << std::endl;

	cv::namedWindow("planner", 0);
	cv::resizeWindow("planner", large_size * map.cols, large_size * map.rows);


	std::shared_ptr<GenerateMap> random_map = std::make_shared<GenerateMap>();
	random_map->randomMap(map, 2000);

	for (int i = - 2; i <=  2; i++)
	{
		for (int j = - 2; j <= + 2; j++)
		{
			int x1 = navi_data.start.x + i;
			int y1 = navi_data.start.y + j;
			int x2 = navi_data.goal.x + i;
			int y2 = navi_data.goal.y + j;
			if (x1 >= 0 && x1 < map.rows && y1 >= 0 && y1 < map.cols)
			{
				map.at<uchar>(x1, y1) = 0;
			}
			if (x2 >= 0 && x2 < map.rows && y2 >= 0 && y2 < map.cols)
			{
				map.at<uchar>(x2, y2) = 0;
			}
		}
	}


	clock_t c1 = clock();
	auto t3 = PLANNERMS;
	auto time1 = std::chrono::system_clock::now();

	astar_ptr->init(navi_data, map);
	astar_ptr->run();
	std::vector<planner::Grid> path = astar_ptr->get_path();
	std::cout << "path size: " << path.size() << std::endl;

	clock_t c2 = clock();
	auto time2 = std::chrono::system_clock::now();
	auto t4 = PLANNERMS;

	std::cout << "find path cost " << 
	std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() << " ms" << std::endl;

	std::cout << "find path cost " << t4 - t3 << " ms" << std::endl;

	std::cout << "find path cost " << (c2 - c1) << " ms" << std::endl;

	cv::imwrite("E:\\cpp\\picture\\0.jpg", map);

	cv::Mat background;
	background.create(map.rows, map.cols, CV_8UC3);
	background.setTo(cv::Scalar(120, 120, 120));

	for (int i = 0; i < map.rows; i++)
	{
		for (int j = 0; j < map.cols; j++)
		{
			if (map.at<uchar>(i, j) == 255)
			{
				background.at<cv::Vec3b>(i, j)[0] = 0;
				background.at<cv::Vec3b>(i, j)[1] = 0;
				background.at<cv::Vec3b>(i, j)[2] = 0;
			}
		}
	}

	for (int i = 0; i < path.size(); i++)
	{
		cv::circle(background, cv::Point(path[i].y, path[i].x), 0, cv::Scalar(0, 255, 0), -1);
	}
	cv::circle(background, cv::Point(navi_data.start.y, navi_data.start.x), 0, cv::Scalar(0, 0, 255), -1);
	cv::circle(background, cv::Point(navi_data.goal.y, navi_data.goal.x), 0, cv::Scalar(0, 0, 255), -1);


	cv::imshow("planner", background);
	//std::cout << "map " << map.rows << "  " << map.cols << std::endl;

	cv::waitKey(0);
	cv::destroyAllWindows();

	
	return 0;
}*/