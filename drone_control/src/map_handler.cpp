#include <drone_control/map_handler.hpp>

MapHandler::MapHandler()
    : origin_map_(0, std::vector<std::vector<int>>(0, std::vector<int>(0))) {
  load_map();
  fill_empty_boxes();
  bloat_map(5);
}

void MapHandler::load_map() {
  std::string resources_path = get_resource_path();
  std::string ws_path = get_ws_path();
  std::vector<std::string> map_paths{
      "map_025.pgm", "map_075.pgm", "map_080.pgm", "map_100.pgm",
      "map_125.pgm", "map_150.pgm", "map_175.pgm", "map_180.pgm",
      "map_200.pgm", "map_225.pgm"};

  std::vector<double> layer_heights = {0.25, 0.75, 0.80, 1.0, 1.25,
                                       1.5,  1.75, 1.80, 2.0, 2.25};

  int layer = 0;
  for (std::string map_path : map_paths) {
    std::stringstream ss;
    std::string input_string;
    unsigned int num_rows = 0, num_cols = 0, num_layers = 10;
    int temp_val;

    // Open map part path
    std::ifstream infile(ws_path + "/" + resources_path + "/" + map_path);
    if (!infile.is_open()) {
      std::cerr << "file at path:" << ws_path << "/" + resources_path
                << "/" + map_path << "doesn't exist" << std::endl;
    }

    ss << infile.rdbuf();
    ss >> input_string;         // First line : version
    ss >> num_cols >> num_rows; // Second line : size
    ss >> input_string;         // Third line : comment

    map_size_ = {num_rows, num_cols, num_layers};
    origin_map_.resize(num_rows, std::vector<std::vector<int>>(
                                     num_cols, std::vector<int>(num_layers)));

    // Following lines : data
    for (unsigned int row = 0; row < num_rows; row++) {
      for (unsigned int col = 0; col < num_cols; col++) {
        ss >> temp_val;
        if (temp_val == 255) {
          origin_map_[row][col][layer] = 0;
        } else if (temp_val == 0) {
          origin_map_[row][col][layer] = 1;
        }
      }
    }

    // Transform map to cartesian coordinates
    std::vector<std::vector<unsigned int>> slice(
        map_size_[0], std::vector<unsigned int>(map_size_[1]));
    for (unsigned int row = 0; row < origin_map_.size(); row++) {
      for (unsigned int col = 0; col < origin_map_[0].size(); col++) {
        slice[row][col] = origin_map_[row][col][layer];
      }
    }
    slice = cv_to_cartesian(slice);
    for (unsigned int row = 0; row < origin_map_.size(); row++) {
      for (unsigned int col = 0; col < origin_map_[0].size(); col++) {
        origin_map_[row][col][layer] = slice[row][col];
      }
    }

    infile.close();
    layer++;
  }

  for (unsigned int i = 0; i < map_size_[2]; i++) {
    height_to_layer_map_[layer_heights[i]] = i;
    layer_to_height_map_[i] = layer_heights[i];
  }
}

void MapHandler::bloat_map(int num_of_cells) {
  int rows = origin_map_.size();
  int cols = origin_map_[0].size();
  int layers = origin_map_[0][0].size();
  std::vector<std::vector<std::vector<int>>> bloated_map = origin_map_;

  // Define 8 possible directions
  int dx[] = {-1, 1, -1, 1, 0, 0, -1, 1};
  int dy[] = {0, 0, -1, 1, -1, 1, 1, -1};

  for (int cell = 0; cell < num_of_cells; cell++) {
    for (int k = 0; k < layers; k++) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          if (origin_map_[i][j][k] == 1) {
            // Iterate over the 8 possible directions
            for (int l = 0; l < 8; l++) {
              int ni = i + dx[l];
              int nj = j + dy[l];

              // Check if the adjacent cell is within bounds
              if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                bloated_map[ni][nj][k] = 1;
              }
            }
          }
        }
      }
    }
    origin_map_ = bloated_map;
  }
  work_map_ = origin_map_;
}

void MapHandler::fill_empty_boxes() {
  std::vector<std::vector<int>> slice(map_size_[0],
                                      std::vector<int>(map_size_[1]));
  for (unsigned int layer = 0; layer < origin_map_[0][0].size(); layer++) {
    for (unsigned int row = 0; row < origin_map_.size(); row++) {
      for (unsigned int col = 0; col < origin_map_[0].size(); col++) {
        slice[row][col] = origin_map_[row][col][layer];
      }
    }

    // fills room with 2s
    find_boxes(slice);

    // 0 -> space in boxes, 2 -> free space
    for (unsigned int row = 0; row < slice.size(); row++) {
      for (unsigned int col = 0; col < slice[0].size(); col++) {
        if (slice[row][col] == 0)
          slice[row][col] = 1;
        else if (slice[row][col] == 2)
          slice[row][col] = 0;
      }
    }

    for (unsigned int row = 0; row < origin_map_.size(); row++) {
      for (unsigned int col = 0; col < origin_map_[0].size(); col++) {
        origin_map_[row][col][layer] = slice[row][col];
      }
    }
  }
}

void MapHandler::flood_fill_room(std::vector<std::vector<int>> &grid,
                                 unsigned int x, unsigned int y) {
  if (x >= grid.size() || y >= grid[0].size() || grid[x][y] != 0) {
    return;
  }
  grid[x][y] = 2; // Mark as visited
  flood_fill_room(grid, x + 1, y);
  flood_fill_room(grid, x - 1, y);
  flood_fill_room(grid, x, y + 1);
  flood_fill_room(grid, x, y - 1);
}

void MapHandler::find_boxes(std::vector<std::vector<int>> &grid) {
  for (unsigned int i = 200; i < grid.size(); i++) {
    for (unsigned int j = 200; j < grid[0].size(); j++) {
      if (grid[i][j] == 0) {
        std::vector<std::pair<int, int>> visitedCells;
        flood_fill_room(grid, i, j);
        bool isClosedLoop = true;

        // Check if the region has a wall in its boundary
        for (auto cell : visitedCells) {
          unsigned int x = cell.first;
          unsigned int y = cell.second;
          if (x == 200 || x == grid.size() - 1 || y == 200 ||
              y == grid[0].size() - 1) {
            isClosedLoop = false;
            break;
          }
        }

        if (isClosedLoop) {
          return;
        }
      }
    }
  }
}

std::vector<std::vector<std::vector<bool>>>
MapHandler::flood_fill(Point<unsigned int> start, Point<unsigned int> goal) {
  std::vector<std::vector<std::vector<bool>>> visited(
      map_size_[0], std::vector<std::vector<bool>>(
                        map_size_[1], std::vector<bool>(map_size_[2])));
  std::queue<Point<unsigned int>> elemet_queue;

  work_map_ = origin_map_;
  visited[start.y][start.x][start.z] = true;
  work_map_[start.y][start.x][start.z] = 2;
  elemet_queue.push(start);

  while (!elemet_queue.empty()) {
    Point<unsigned int> actual_element = elemet_queue.front();
    elemet_queue.pop();
    if (actual_element.x == goal.x && actual_element.y == goal.y &&
        actual_element.z == goal.z) {
      return visited;
    }

    std::vector<Point<unsigned int>> neighbors = {
        {actual_element.x + 1, actual_element.y, actual_element.z},
        {actual_element.x - 1, actual_element.y, actual_element.z},
        {actual_element.x, actual_element.y + 1, actual_element.z},
        {actual_element.x, actual_element.y - 1, actual_element.z},
        {actual_element.x + 1, actual_element.y + 1, actual_element.z},
        {actual_element.x - 1, actual_element.y - 1, actual_element.z},
        {actual_element.x + 1, actual_element.y - 1, actual_element.z},
        {actual_element.x - 1, actual_element.y + 1, actual_element.z},

        {actual_element.x + 1, actual_element.y, actual_element.z + 1},
        {actual_element.x - 1, actual_element.y, actual_element.z + 1},
        {actual_element.x, actual_element.y + 1, actual_element.z + 1},
        {actual_element.x, actual_element.y - 1, actual_element.z + 1},
        {actual_element.x + 1, actual_element.y + 1, actual_element.z + 1},
        {actual_element.x - 1, actual_element.y - 1, actual_element.z + 1},
        {actual_element.x + 1, actual_element.y - 1, actual_element.z + 1},
        {actual_element.x - 1, actual_element.y + 1, actual_element.z + 1},

        {actual_element.x + 1, actual_element.y, actual_element.z - 1},
        {actual_element.x - 1, actual_element.y, actual_element.z - 1},
        {actual_element.x, actual_element.y + 1, actual_element.z - 1},
        {actual_element.x, actual_element.y - 1, actual_element.z - 1},
        {actual_element.x + 1, actual_element.y + 1, actual_element.z - 1},
        {actual_element.x - 1, actual_element.y - 1, actual_element.z - 1},
        {actual_element.x + 1, actual_element.y - 1, actual_element.z - 1},
        {actual_element.x - 1, actual_element.y + 1, actual_element.z - 1}};

    for (const Point<unsigned int> &point : neighbors) {
      if (point.x < work_map_[0].size() && point.y < work_map_.size() &&
          point.z < work_map_[0][0].size() &&
          work_map_[point.y][point.x][point.z] != 1) {
        if (!visited[point.y][point.x][point.z]) {
          visited[point.y][point.x][point.z] = true;
          elemet_queue.push(point);
          work_map_[point.y][point.x][point.z] =
              work_map_[actual_element.y][actual_element.x][actual_element.z] +
              1;
        }
      }
    }
  }

  return visited;
}

bool MapHandler::generate_path(Point<double> start, Point<double> goal) {
  std::vector<std::vector<std::vector<bool>>> visited;
  Point<unsigned int> local_start, local_goal;
  std::vector<Point<unsigned int>> local_path, local_waypoints;
  Point<double> transformed_point;
  path_.clear();
  waypoints_.clear();

  // Transform [m] to vector indexes
  local_start.x =
      static_cast<unsigned int>((start.x * 100) / GRID_SIZE_XY + OFFSET_X);
  local_start.y =
      static_cast<unsigned int>((start.y * 100) / GRID_SIZE_XY + OFFSET_Y);
  local_start.z = static_cast<unsigned int>(height_to_layer_map_[start.z]);
  local_goal.x =
      static_cast<unsigned int>((goal.x * 100) / GRID_SIZE_XY + OFFSET_X);
  local_goal.y =
      static_cast<unsigned int>((goal.y * 100) / GRID_SIZE_XY + OFFSET_Y);
  local_goal.z = static_cast<unsigned int>(height_to_layer_map_[goal.z]);

  if (local_start.x >= map_size_[1] || local_start.y >= map_size_[0] ||
      local_start.z >= map_size_[2] || local_goal.x >= map_size_[1] ||
      local_goal.y >= map_size_[0] || local_goal.z >= map_size_[2]) {
    std::cout << "Input point/s out of map !!!" << std::endl;
    return false;
  }
  if (origin_map_[local_start.y][local_start.x][local_start.z] != 0 ||
      origin_map_[local_goal.y][local_goal.x][local_goal.z] != 0) {
    std::cout << "Input Point/s are inaccesable!!!" << std::endl;
    return false;
  }

  visited = flood_fill(local_start, local_goal);

  std::vector<bool> actual_diff_xyz = {false, false, false};
  std::vector<bool> last_diff_xyz = {true, true, true};
  Point<unsigned int> point = local_goal;

  int cost = work_map_[local_goal.y][local_goal.x][local_goal.z];
  while (point.x != local_start.x || point.y != local_start.y ||
         point.z != local_start.z) {
    transformed_point.x = static_cast<double>(
        (static_cast<double>((point.x - OFFSET_X)) / 100) * GRID_SIZE_XY);
    transformed_point.y = static_cast<double>(
        (static_cast<double>((point.y - OFFSET_Y)) / 100) * GRID_SIZE_XY);
    transformed_point.z = static_cast<double>(layer_to_height_map_[point.z]);

    path_.push_back(transformed_point);
    local_path.push_back(point);

    std::vector<Point<unsigned int>> neighbors = {
        {point.x + 1, point.y, point.z},
        {point.x - 1, point.y, point.z},
        {point.x, point.y + 1, point.z},
        {point.x, point.y - 1, point.z},
        {point.x + 1, point.y + 1, point.z},
        {point.x - 1, point.y - 1, point.z},
        {point.x + 1, point.y - 1, point.z},
        {point.x - 1, point.y + 1, point.z},

        {point.x + 1, point.y, point.z + 1},
        {point.x - 1, point.y, point.z + 1},
        {point.x, point.y + 1, point.z + 1},
        {point.x, point.y - 1, point.z + 1},
        {point.x + 1, point.y + 1, point.z + 1},
        {point.x - 1, point.y - 1, point.z + 1},
        {point.x + 1, point.y - 1, point.z + 1},
        {point.x - 1, point.y + 1, point.z + 1},

        {point.x + 1, point.y, point.z - 1},
        {point.x - 1, point.y, point.z - 1},
        {point.x, point.y + 1, point.z - 1},
        {point.x, point.y - 1, point.z - 1},
        {point.x + 1, point.y + 1, point.z - 1},
        {point.x - 1, point.y - 1, point.z - 1},
        {point.x + 1, point.y - 1, point.z - 1},
        {point.x - 1, point.y + 1, point.z - 1}};

    for (const Point<unsigned int> &neighbor : neighbors) {
      if (neighbor.x < work_map_[0].size() && neighbor.y < work_map_.size() &&
          neighbor.z < work_map_[0][0].size() &&
          visited[neighbor.y][neighbor.x][neighbor.z]) {
        if (work_map_[neighbor.y][neighbor.x][neighbor.z] == cost - 1) {
          cost = work_map_[neighbor.y][neighbor.x][neighbor.z];
          abs(neighbor.x - point.x) > 0 ? actual_diff_xyz[0] = true
                                        : actual_diff_xyz[0] = false;
          abs(neighbor.y - point.y) > 0 ? actual_diff_xyz[1] = true
                                        : actual_diff_xyz[1] = false;
          abs(neighbor.z - point.z) > 0 ? actual_diff_xyz[2] = true
                                        : actual_diff_xyz[2] = false;
          if (actual_diff_xyz[0] != last_diff_xyz[0] ||
              actual_diff_xyz[1] != last_diff_xyz[1] ||
              actual_diff_xyz[2] != last_diff_xyz[2]) {
            waypoints_.push_back(path_.back());
            local_waypoints.push_back(local_path.back());
          }

          last_diff_xyz = actual_diff_xyz;
          point = neighbor;
        }
      }
    }
  }

  path_.push_back(start);
  waypoints_.push_back(start);
  local_path.push_back(local_start);
  local_waypoints.push_back(local_start);
  std::reverse(local_path.begin(), local_path.end());
  std::reverse(path_.begin(), path_.end());
  std::reverse(waypoints_.begin(), waypoints_.end());

  work_map_ = origin_map_;
  std::cout << " lx: " << local_start.x << " ly: " << local_start.y
            << " lz: " << local_start.z << std ::endl;
  std::cout << " lx: " << start.x << " ly: " << start.y << " lz: " << start.z
            << std ::endl;
  for (Point<unsigned int> point : local_path) {
    work_map_[point.y][point.x][point.z] = 444;
    std::cout << " x: " << point.x << " y: " << point.y << " z: " << point.z
              << std ::endl;
  }

  for (Point<unsigned int> point : local_waypoints) {
    work_map_[point.y][point.x][point.z] = 111;
  }

  for (Point<double> point : waypoints_) {
    std::cout << " x: " << point.x << " y: " << point.y << " z: " << point.z
              << std ::endl;
  }
  std::cout
      << "**************************************\n*****************************"
         "*********\n**************************************\n******************"
         "********************\n**************************************\n"
      << std::endl;
  // print_map(work_map_);
  return true;
}

std::vector<std::vector<unsigned int>> MapHandler::cv_to_cartesian(
    const std::vector<std::vector<unsigned int>> &cv_map) {
  unsigned int height = cv_map.size();
  unsigned int width = cv_map[0].size();

  std::vector<std::vector<unsigned int>> cartesian_map(
      height, std::vector<unsigned int>(width, 0));

  for (unsigned int row = 0; row < height; ++row) {
    for (unsigned int col = 0; col < width; ++col) {
      unsigned int cv_value = cv_map[row][col];
      unsigned int cartesian_row = height - row - 1;
      cartesian_map[cartesian_row][col] = cv_value;
    }
  }

  return cartesian_map;
}

void MapHandler::print_map(std::vector<std::vector<std::vector<int>>> &map) {
  for (unsigned int layer = 0; layer < map[0][0].size(); layer++) {
    for (int row = map.size() - 1; row >= 0; row--) {
      for (unsigned int col = 0; col < map[0].size(); col++) {

        if (map[row][col][layer] < 10)
          std::cout << "\033[37m" << std::setw(3) << map[row][col][layer]
                    << " ";
        else if (map[row][col][layer] < 100)
          std::cout << "\033[32m" << std::setw(3) << map[row][col][layer]
                    << " ";
        else if (map[row][col][layer] < 200)
          std::cout << "\033[36m" << std::setw(3) << map[row][col][layer]
                    << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}