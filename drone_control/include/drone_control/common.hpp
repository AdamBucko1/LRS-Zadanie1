#pragma once
#include <iostream> 
#include <fstream> 
#include <sstream>
#include <filesystem> 
#include <vector>
#include <queue>
#include <algorithm>
#include <map>

template<class T> 
struct Point {
    T x, y, z;
};

inline std::string get_ws_path(){
  std::filesystem::path folderPath = std::filesystem::current_path(); // Get the current working directory
  return folderPath;
}

inline std::string get_resource_path(){
  return "src/LRS-Zadanie1/drone_control/resources";
}