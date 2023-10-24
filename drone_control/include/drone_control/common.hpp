#pragma once
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <vector>

template <class T> struct Point { T x, y, z; };

inline std::string get_ws_path() {
  std::filesystem::path folderPath =
      std::filesystem::current_path(); // Get the current working directory
  return folderPath;
}

inline std::string get_resource_path() {
  return "src/lrs_zad_1/drone_control/resources";
}
// home/adam/nvapriltag_foxy/src/lrs_zad_1/drone_control/resources/map_025.pgm