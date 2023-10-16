#pragma once
#include <drone_control/common.hpp>

class MapHandler
{
public:
    MapHandler();

    void load_map();
    void print_map();
    void bloat_map(int num_of_cells);
    void flood_fill();

private:
    std::vector<std::vector<std::vector<int>>> origin_map_;
    std::vector<std::vector<std::vector<int>>> work_map_;

    // flood fill
    std::vector<std::vector<std::vector<bool>>>visited_;
    Point<int> start_;
    Point<int> goal_;
};