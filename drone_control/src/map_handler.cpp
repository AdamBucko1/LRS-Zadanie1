#include <drone_control/map_handler.hpp>

MapHandler::MapHandler() :
origin_map_(288, std::vector<std::vector<int>>(366, std::vector<int>(10)))
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

  bloat_map();
  print_map();
}

void MapHandler::print_map()
{
  for(unsigned int layer = 0; layer < origin_map_[0][0].size(); layer++) {
    for(unsigned int row = 0; row < origin_map_.size(); row++) {
      for(unsigned int col = 0; col < origin_map_[0].size(); col++) {
        std::cout << origin_map_[row][col][layer] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

void MapHandler::bloat_map()
{
  int rows = origin_map_.size();
  int cols = origin_map_[0].size();
  int layers = origin_map_[0][0].size();

  std::vector<std::vector<std::vector<int>>> bloatedMap = origin_map_;

  // Define 8 possible directions
  int dx[] = {-1, 1, -1, 1, 0, 0, -1, 1};
  int dy[] = {0, 0, -1, 1, -1, 1, 1, -1};

  for (int k = 0; k < layers; k++) {
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < cols; j++) {
        
          if (origin_map_[i][j][k] == 1) {
              // Iterate over the 4 possible directions
              for (int l = 0; l < 8; l++) {
                  int ni = i + dx[l];
                  int nj = j + dy[l];

                  // Check if the adjacent cell is within bounds
                  if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                      bloatedMap[ni][nj][k] = 1;
                  }
              }
          }
      }
    }
  }
  origin_map_ = bloatedMap;
}

std::string MapHandler::get_ws_path(){
  std::filesystem::path folderPath = std::filesystem::current_path(); // Get the current working directory
  return folderPath;
}