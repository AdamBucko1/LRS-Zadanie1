#pragma once
#include <iostream> 
#include <fstream> 
#include <sstream>
#include <filesystem> 
#include <vector>

class MapHandler
{
public:
    MapHandler();

    void load_map();
    void print_map();
    void bloat_map();
    std::string get_ws_path();

private:
    std::vector<std::vector<std::vector<int>>> origin_map_;
    std::vector<std::vector<std::vector<int>>> work_map_;
};