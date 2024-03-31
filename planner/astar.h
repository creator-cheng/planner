#ifndef _ASTART_H
#define _ASTART_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace planner
{

	struct Grid
	{
		int x, y;

		Grid(int x_, int y_) : x(x_), y(y_) {}

		Grid() {}

		bool operator == (const Grid& g) const
		{
			return (x == g.x) && (y == g.y);
		}

		bool operator != (const Grid& g) const
		{
			return (x != g.x) || (y != g.y);
		}

	};

	struct GridNode
	{
		signed char id;
		Grid index;

		double g;
		double f;
		GridNode* parent;

		GridNode(Grid _index)
		{
			id = 0;
			index = _index;

			g = 0;
			f = 0;
			parent = nullptr;
		}

		GridNode() {};
		~GridNode() {};
	};

	struct NaviData
	{
		Grid start;
		Grid goal;
		int dilate;

		NaviData() 
		{
			dilate = 1;
		};
	};


	class Astar
	{
	private:
		const int boundx = 1200;
		const int boundy = 1200;
		const int grid_threshold = 255;

		const std::vector<std::vector<int>> dir = { {1, 0}, {-1, 0}, {0, 1}, {0, -1 },
														{1, 1}, {1, -1}, {-1, 1}, {-1, -1 } };

		//const std::vector<std::vector<int>> dir = { {1, 0}, {-1, 0}, {0, 1}, {0, -1 } };

		NaviData navi_data;

		cv::Mat map;
		int map_x, map_y;

		std::vector<Grid> path;

		GridNode*** grid_node_pool;
		std::vector<GridNode*> neighbor_ptr_sets;
		std::vector<double> edge_cost_sets;


		GridNode* terminate_ptr;
		std::multimap<double, GridNode*> open_set;

		double getHeu(GridNode* node1, GridNode* node2);
		void AstarGetSucc(GridNode* currentPtr);
		int isValid(const int x, const int y);
		void resetUsedGrids();
		void getPath();
		std::vector<Grid> getVisitedNodes();
		bool inMap(const int x, const int y) const;
		void mapDilate();

	public:
		Astar();
		~Astar();

		void init(const NaviData& navi_data_, const cv::Mat& map_);
		void run();
		std::vector<Grid> get_path() const;

	};

}

#endif
