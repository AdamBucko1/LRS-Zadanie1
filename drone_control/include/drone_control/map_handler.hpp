#pragma once
#include <drone_control/common.hpp>

class MapHandler
{
public:
    MapHandler();

    void load_map();
    void print_map();
    void flood_fill();

private:
    void bloat_map(int num_of_cells);
    void flood_fill_room(std::vector<std::vector<int>>& grid, unsigned int x, unsigned int y);
    void find_boxes(std::vector<std::vector<int>>& grid);
    void fill_empty_boxes();

private:
    std::vector<std::vector<std::vector<int>>> origin_map_;
    std::vector<std::vector<std::vector<int>>> work_map_;

    // flood fill
    std::vector<std::vector<std::vector<bool>>>visited_;
    Point<unsigned int> start_;
    Point<unsigned int> goal_;
};