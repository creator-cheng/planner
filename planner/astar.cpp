#include "astar.h"


namespace planner
{
    Astar::Astar()
    {
        auto t1 = (std::chrono::system_clock::now().time_since_epoch().count() / 10000);

        std::cout << "construct Astar" << std::endl;
        
        grid_node_pool = new GridNode * *[boundx];
        for (int i = 0; i < boundx; i++)
        {
            grid_node_pool[i] = new GridNode * [boundy];
            for (int j = 0; j < boundy; j++)
            {
                Grid tmpIdx(i, j);
                grid_node_pool[i][j] = new GridNode(tmpIdx);
            }
        }

        auto t2 = (std::chrono::system_clock::now().time_since_epoch().count() / 10000);
        std::cout << "finished construct Astar, cost " << t2 - t1 << " ms" << std::endl;
    }

    Astar::~Astar()
    {
        std::cout << "destroy Astar" << std::endl;
        for (int i = 0; i < boundx; i++)
        {
            delete[] grid_node_pool[i];
        }
    }

    void Astar::init(const NaviData& navi_data_, const cv::Mat& map_)
    {
        navi_data = navi_data_;
        map = map_.clone();

        map_x = map.rows;
        map_y = map.cols;

        path.clear();
    }

    void Astar::resetUsedGrids()
    {
        for (int i = 0; i < boundx; i++)
        {
            for (int j = 0; j < boundy; j++)
            {
                GridNode* ptr = grid_node_pool[i][j];
                ptr->id = 0;
                ptr->parent = nullptr;
                ptr->g = 0;
                ptr->f = 0;
            }
        }
    }

    int Astar::isValid(const int x, const int y)
    {
        if (x < 0 || x >= map_x || y < 0 || y >= map_y)
        {
            return grid_threshold;
        }
        return static_cast<int>(map.at<uchar>(x, y));
    }

    bool Astar::inMap(const int x, const int y) const
    {
        if (x < 0 || x >= map_x || y < 0 || y >= map_y)
        {
            return false;
        }
        return true;
    }

    std::vector<Grid> Astar::getVisitedNodes()
    {
        std::vector<Grid> visited_nodes;
        for (int i = 0; i < boundx; i++)
        {
            for (int j = 0; j < boundy; j++)
            {
                if (grid_node_pool[i][j]->id != 0)
                {
                    visited_nodes.push_back(grid_node_pool[i][j]->index);
                }
            }
        }

        return visited_nodes;
    }

    double Astar::getHeu(GridNode* node1, GridNode* node2)
    {
        double h;
        Grid node1_coord = node1->index;
        Grid node2_coord = node2->index;

        double dx = std::abs(node1_coord.x - node2_coord.x);
        double dy = std::abs(node1_coord.y - node2_coord.y);

        h = 0.9999 * std::sqrt(dx * dx + dy * dy);

        return h;
    }

    inline void Astar::AstarGetSucc(GridNode* currentPtr)
    {
        neighbor_ptr_sets.clear();
        edge_cost_sets.clear();

        if (currentPtr == nullptr)
            std::cout << "Error: Current pointer is null!" << std::endl;

        GridNode* temp_ptr = nullptr;

        for (int i = 0; i < dir.size(); i++)
        {
            int n_x = currentPtr->index.x + dir[i][0];
            int n_y = currentPtr->index.y + dir[i][1];

            if (isValid(n_x, n_y) >= grid_threshold)
            {
                continue;
            }

            temp_ptr = grid_node_pool[n_x][n_y];

            if (temp_ptr->id == -1) continue;


            if (temp_ptr == currentPtr) {
                std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
            }

            double dist = std::sqrt(dir[i][0] * dir[i][0] + dir[i][1] * dir[i][1]);

            neighbor_ptr_sets.push_back(temp_ptr);
            edge_cost_sets.push_back(dist);
        }

    }

    void Astar::mapDilate()
    {
        int dilate_ = navi_data.dilate;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, 
                                                    cv::Size(2 * dilate_ + 1, 2 * dilate_ + 1));
        cv::dilate(map, map, element);
    }

    void Astar::run()
    {
        const Grid start(navi_data.start.x, navi_data.start.y);
        const Grid goal(navi_data.goal.x, navi_data.goal.y);

        if (inMap(start.x, start.y) == false || inMap(goal.x, goal.y) == false)
        {
            std::cout << "ERROR occur, start or goal is not in map." << std::endl;
            std::cout << "start: " << start.x << " " << start.y << std::endl;
            std::cout << "goal: " << goal.x << " " << goal.y << std::endl;
            std::cout << "map size  x: 0 " << map_x << "  y: " << map_y << std::endl;
            return;
        }

        mapDilate();

        GridNode* start_ptr = grid_node_pool[start.x][start.y];
        GridNode* end_ptr = grid_node_pool[goal.x][goal.y];


        open_set.clear();
        GridNode* current_ptr = nullptr;
        GridNode* neighbor_ptr = nullptr;


        start_ptr->g = 0;
        start_ptr->f = getHeu(start_ptr, end_ptr);
        start_ptr->id = 1;

        open_set.insert(std::make_pair(start_ptr->f, start_ptr));


        bool success = false;

        while (!open_set.empty())
        {

            current_ptr = open_set.begin()->second;
            open_set.erase(open_set.begin());

            Grid current_idx = current_ptr->index;
            grid_node_pool[current_idx.x][current_idx.y]->id = -1;


            if (current_ptr->index == goal)
            {
                terminate_ptr = current_ptr;
                success = true;
                break;
            }

            AstarGetSucc(current_ptr);

            for (int i = 0; i < (int)neighbor_ptr_sets.size(); i++)
            {
                neighbor_ptr = neighbor_ptr_sets[i];
                if (neighbor_ptr->id == 0)
                {
                    neighbor_ptr->g = current_ptr->g + edge_cost_sets[i];
                    neighbor_ptr->f = neighbor_ptr->g + getHeu(neighbor_ptr, end_ptr);
                    neighbor_ptr->parent = current_ptr;
                    neighbor_ptr->id = 1;

                    open_set.insert(std::make_pair(neighbor_ptr->f, neighbor_ptr));

                }
                else if (neighbor_ptr->g > (current_ptr->g + edge_cost_sets[i]))
                {
                    neighbor_ptr->g = current_ptr->g + edge_cost_sets[i];
                    neighbor_ptr->f = neighbor_ptr->g + getHeu(neighbor_ptr, end_ptr);
                    neighbor_ptr->parent = current_ptr;
                }
            }
        }

        if (success == true)
        {
            std::cout << "find path success" << std::endl;
            getPath();
        }
        else
        {
            std::cout << "find path failed" << std::endl;
        }

        resetUsedGrids();

    }

    void Astar::getPath()
    {
        std::vector<GridNode*> gridPath;

        GridNode* ptr = terminate_ptr;

        while (ptr->parent != nullptr)
        {
            gridPath.push_back(ptr);


            ptr = ptr->parent;

        }
        if (ptr != nullptr)
        {
            gridPath.push_back(ptr);
        }

        for (auto ptr : gridPath)
        {
            path.push_back(ptr->index);
        }

        reverse(path.begin(), path.end());
    }

    std::vector<Grid> Astar::get_path() const
    {
        return path;
    }

}