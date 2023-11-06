#include <drone_control/point_cloud_handler.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

PointCloudHandler::PointCloudHandler()
    : origin_map_(0, std::vector<std::vector<int>>(0, std::vector<int>(0))) 
{
  load_map();
  // fill_empty_boxes();
  // bloat_map(8);
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
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }

  return true;
}