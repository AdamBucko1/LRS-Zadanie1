#include <drone_control/point_cloud_handler.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

PointCloudHandler::PointCloudHandler()
    : origin_map_(0, std::vector<std::vector<int>>(0, std::vector<int>(0))) 
{
  load_map();
  fill_empty_boxes();
  bloat_map(2);
  print_map(origin_map_);

  // std::string resources_path = get_resource_path();
  // std::string ws_path = get_ws_path();
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // if (pcl::io::loadPCDFile<pcl::PointXYZ> (ws_path + "/" + resources_path + "/map.pcd", *cloud) == -1) 
  // {
  //   return;
  // }

  // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  // viewer.showCloud (cloud);
  // while (!viewer.wasStopped ())
  // {
  // }
}

int main()
{
  PointCloudHandler map_handler_;
}

bool PointCloudHandler::load_map()
{
  std::string resources_path = get_resource_path();
  std::string ws_path = get_ws_path();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ws_path + "/" + resources_path + "/map.pcd", *cloud) == -1) 
  {
    return false;
  }

  // Manual calculation of min and max points
  pcl::PointXYZ minPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  pcl::PointXYZ maxPoint(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

  for (const auto& point : cloud->points) {
      minPoint.x = std::min(minPoint.x, point.x);
      minPoint.y = std::min(minPoint.y, point.y);
      minPoint.z = std::min(minPoint.z, point.z);

      maxPoint.x = std::max(maxPoint.x, point.x);
      maxPoint.y = std::max(maxPoint.y, point.y);
      maxPoint.z = std::max(maxPoint.z, point.z);
  }

  // Calculate the size of the origin_map_
  unsigned int row_size = static_cast<unsigned int>(std::ceil((maxPoint.y - minPoint.y) / 0.05)) + 1;
  unsigned int column_size = static_cast<unsigned int>(std::ceil((maxPoint.x - minPoint.x) / 0.05)) + 1;
  unsigned int layer_size = static_cast<unsigned int>(std::ceil((maxPoint.z - minPoint.z) / 0.05)) + 1;
  map_size_ = {row_size, column_size, layer_size};

  // Resize the origin_map_
  origin_map_.resize(row_size, std::vector<std::vector<int>>(column_size, std::vector<int>(layer_size, 0)));
  std::cout << "x:" << origin_map_.size() << " y: " << origin_map_[0].size() << " z: " << origin_map_[0][0].size() << std::endl;

  // Calculate the shift values
  OFFSET_X = static_cast<int>(std::round(-minPoint.x / 0.05));
  OFFSET_Y = static_cast<int>(std::round(-minPoint.y / 0.05));
  OFFSET_Z = static_cast<int>(std::round(-minPoint.z / 0.05));

  // Load point cloud into 3D vector
  for (const auto& point : cloud->points) {
    unsigned int x = static_cast<int>(std::round((point.x + OFFSET_X * 0.05) / 0.05)); 
    unsigned int y = static_cast<int>(std::round((point.y + OFFSET_Y * 0.05) / 0.05));
    unsigned int z = static_cast<int>(std::round((point.z + OFFSET_Z * 0.05) / 0.05));

    origin_map_[y][x][z] = 1; 
  }

  return true;
}

void PointCloudHandler::bloat_map(int num_of_cells) {
  int rows = origin_map_.size();
  int cols = origin_map_[0].size();
  int layers = origin_map_[0][0].size();
  std::vector<std::vector<std::vector<int>>> bloated_map = origin_map_;

  // Define 26 possible directions in 3D
  int dx[] = {-1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, 0, 0, 0, 0, 0, 0};
  int dy[] = {-1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1, 0, 0, -1, 1, 0, 0, 0};
  int dz[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 1, 1, 1, -1, 1, 0, 0, 0, 1, 1};

  for (int cell = 0; cell < num_of_cells; cell++) {
    for (int k = 0; k < layers; k++) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          if (origin_map_[i][j][k] == 1) {
            // Iterate over the 26 possible directions in 3D
            for (int l = 0; l < 26; l++) {
              int ni = i + dx[l];
              int nj = j + dy[l];
              int nk = k + dz[l];

              // Check if the adjacent cell is within bounds
              if (ni >= 0 && ni < rows && nj >= 0 && nj < cols && nk >= 0 && nk < layers) {
                bloated_map[ni][nj][nk] = 1;
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

void PointCloudHandler::fill_empty_boxes() {
  std::vector<std::vector<int>> slice(map_size_[0], std::vector<int>(map_size_[1]));
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

void PointCloudHandler::flood_fill_room(std::vector<std::vector<int>> &grid,
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

void PointCloudHandler::find_boxes(std::vector<std::vector<int>> &grid) {
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


void PointCloudHandler::print_map(std::vector<std::vector<std::vector<int>>> &map) {
  std::cout << "x:" << map.size() << " y: " << map[0].size() << " z: " << map[0][0].size() << std::endl;
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