#include <drone_control/map_handler.hpp>

MapHandler::MapHandler() :
origin_map_(288, std::vector<std::vector<int>>(366, std::vector<int>(10))),
visited_(288, std::vector<std::vector<bool>>(366, std::vector<bool>(10)))
{
  load_map();
}

void MapHandler::load_map()
{
  std::string resources_path = "src/LRS-Zadanie1/drone_control/resources";
  std::string ws_path = get_ws_path();
  std::vector<std::string> map_paths{"map_025.pgm","map_075.pgm","map_080.pgm","map_100.pgm",
                                     "map_125.pgm","map_150.pgm","map_175.pgm","map_180.pgm",
                                     "map_200.pgm","map_225.pgm"};

  int layer = 0;
  for(std::string map_path : map_paths)
  {
    std::stringstream ss;
    std::string input_string;
    int num_rows = 0, num_cols = 0;
    int temp_val;

    // Open map part path
    std::ifstream infile(ws_path + "/" + resources_path + "/" + map_path);
    if(!infile.is_open())
    {
      std::cerr << "file at path:" << ws_path << "/" + resources_path << "/" + map_path << "doesn't exist" << std::endl;
    }

    ss << infile.rdbuf();
    ss >> input_string; // First line : version
    ss >> num_cols >> num_rows; // Second line : size
    ss >> input_string; // Third line : comment

    // Following lines : data
    for(int row = 0; row < num_rows; row++){
      for (int col = 0; col < num_cols; col++){
        ss >> temp_val;
        if(temp_val == 255){ 
          origin_map_[row][col][layer] = 0;
        }
        else if(temp_val == 0){ 
          origin_map_[row][col][layer] = 1;
        }
      }
    }

    infile.close();
    layer++;
  }

  work_map_ = origin_map_;

  start_.x = 30;
  start_.y = 200;
  start_.z = 9;
  goal_.x = 200;
  goal_.y = 200;
  goal_.z = 8;

  fill_empty_boxes();
  bloat_map(2);
  
  flood_fill();
  print_map();
}

void MapHandler::bloat_map(int num_of_cells)
{
  int rows = origin_map_.size();
  int cols = origin_map_[0].size();
  int layers = origin_map_[0][0].size();
  std::vector<std::vector<std::vector<int>>> bloated_map = origin_map_;

  // Define 8 possible directions
  int dx[] = {-1, 1, -1, 1, 0, 0, -1, 1};
  int dy[] = {0, 0, -1, 1, -1, 1, 1, -1};

  for(int cell = 0; cell < num_of_cells; cell++){
    for (int k = 0; k < layers; k++) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          if (work_map_[i][j][k] == 1) {
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
    work_map_ = bloated_map;
  }
}

void MapHandler::fill_empty_boxes()
{
  std::vector<std::vector<int>> slice(288 , std::vector<int> (366));
  for(unsigned int layer = 0; layer < work_map_[0][0].size(); layer++) {
    for(unsigned int row = 0; row < work_map_.size(); row++) {
      for(unsigned int col = 0; col < work_map_[0].size(); col++) {
        slice[row][col] = work_map_[row][col][layer];
      }
    }

    // fills room with 2s
    find_boxes(slice);

    // 0s are space in boxes and 2s are free space
    for(unsigned int row = 0; row < slice.size(); row++) {
      for(unsigned int col = 0; col < slice[0].size(); col++) {
        if(slice[row][col] == 0)
          slice[row][col] = 1;
        else if(slice[row][col] == 2)
          slice[row][col] = 0;
      }
    }

    for(unsigned int row = 0; row < work_map_.size(); row++) {
      for(unsigned int col = 0; col < work_map_[0].size(); col++) {
        work_map_[row][col][layer] = slice[row][col];
      }
    }
  }
}

void MapHandler::flood_fill_room(std::vector<std::vector<int>>& grid,unsigned int x,unsigned int y)
{
  if (x >= grid.size() || y >= grid[0].size() || grid[x][y] != 0) {
      return;
  }
  grid[x][y] = 2; // Mark as visited
  flood_fill_room(grid, x + 1, y);
  flood_fill_room(grid, x - 1, y);
  flood_fill_room(grid, x, y + 1);
  flood_fill_room(grid, x, y - 1);
}

void MapHandler::find_boxes(std::vector<std::vector<int>>& grid) {
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
            if (x == 200 || x == grid.size() - 1 || y == 200 || y == grid[0].size() - 1) {
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

void MapHandler::flood_fill()
{
  std::queue<Point<unsigned int>> elemet_queue;
  visited_[start_.x][start_.y][start_.z] = true;
  work_map_[start_.x][start_.y][start_.z] = 2;
  elemet_queue.push(start_);
  while(!elemet_queue.empty())
  {
    Point<unsigned int> actual_element = elemet_queue.front();
    elemet_queue.pop();
    if (actual_element.x == goal_.x && actual_element.y == goal_.y && actual_element.z == goal_.z)
    {
        return;
    }

    std::vector<Point<unsigned int>> neighbors = {
        {actual_element.x+1, actual_element.y, actual_element.z}, {actual_element.x-1, actual_element.y, actual_element.z},
        {actual_element.x, actual_element.y+1, actual_element.z}, {actual_element.x, actual_element.y-1, actual_element.z},
        {actual_element.x+1, actual_element.y+1, actual_element.z}, {actual_element.x-1, actual_element.y-1, actual_element.z},
        {actual_element.x+1, actual_element.y-1, actual_element.z}, {actual_element.x-1, actual_element.y+1, actual_element.z},

        {actual_element.x+1, actual_element.y, actual_element.z+1}, {actual_element.x-1, actual_element.y, actual_element.z+1},
        {actual_element.x, actual_element.y+1, actual_element.z+1}, {actual_element.x, actual_element.y-1, actual_element.z+1},
        {actual_element.x+1, actual_element.y+1, actual_element.z+1}, {actual_element.x-1, actual_element.y-1, actual_element.z+1},
        {actual_element.x+1, actual_element.y-1, actual_element.z+1}, {actual_element.x-1, actual_element.y+1, actual_element.z+1},

        {actual_element.x+1, actual_element.y, actual_element.z-1}, {actual_element.x-1, actual_element.y, actual_element.z-1},
        {actual_element.x, actual_element.y+1, actual_element.z-1}, {actual_element.x, actual_element.y-1, actual_element.z-1},
        {actual_element.x+1, actual_element.y+1, actual_element.z-1}, {actual_element.x-1, actual_element.y-1, actual_element.z-1},
        {actual_element.x+1, actual_element.y-1, actual_element.z-1}, {actual_element.x-1, actual_element.y+1, actual_element.z-1}
    };

    for (const Point<unsigned int>& point : neighbors)
    {
        if (point.x < work_map_.size() && point.y < work_map_[0].size() && 
            point.z < work_map_[0][0].size() && work_map_[point.x][point.y][point.z] != 1)
        {
            if (!visited_[point.x][point.y][point.z])
            {
                visited_[point.x][point.y][point.z] = true;
                elemet_queue.push(point);
                work_map_[point.x][point.y][point.z] = work_map_[actual_element.x][actual_element.y][actual_element.z] + 1;
            }
        }
    }
  }
}

void MapHandler::print_map()
{
  for(unsigned int layer = 0; layer < work_map_[0][0].size(); layer++) {
    for(unsigned int row = 0; row < work_map_.size(); row++) {
      for(unsigned int col = 0; col < work_map_[0].size(); col++) {

        if(work_map_[row][col][layer] < 10)
          std::cout << std::setw(3) << work_map_[row][col][layer] << "\033[37m" << " ";
        else if(work_map_[row][col][layer] < 100)
          std::cout << std::setw(3) << work_map_[row][col][layer] << "\033[32m" << " ";
        else if(work_map_[row][col][layer] < 200)
          std::cout << std::setw(3) << work_map_[row][col][layer] << "\033[36m" << " ";
        else 
          std::cout << std::setw(3) << work_map_[row][col][layer] << "\033[31m" << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}