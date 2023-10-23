#pragma once
#include <drone_control/common.hpp>

class MapHandler {
public:
  MapHandler();
  bool generate_path(Point<double> start, Point<double> goal);
  inline std::vector<Point<double>> get_waypoints() { return waypoints_; };
  inline std::vector<Point<double>> get_path() { return path_; };

private:
  void load_map();
  void print_map(std::vector<std::vector<std::vector<int>>> &map);
  std::vector<std::vector<std::vector<bool>>>
  flood_fill(Point<unsigned int> start, Point<unsigned int> goal);
  void bloat_map(int num_of_cells);
  void flood_fill_room(std::vector<std::vector<int>> &grid, unsigned int x,
                       unsigned int y);
  void find_boxes(std::vector<std::vector<int>> &grid);
  void fill_empty_boxes();

private:
  std::vector<std::vector<std::vector<int>>> origin_map_;
  std::vector<std::vector<std::vector<int>>> work_map_;
  std::vector<unsigned int> map_size_;
  std::map<double, int> height_to_layer_map_;
  std::map<int, double> layer_to_height_map_;
  static constexpr unsigned int GRID_SIZE_XY = 5;

  // Map navigation
  std::vector<Point<double>> waypoints_;
  std::vector<Point<double>> path_;
};