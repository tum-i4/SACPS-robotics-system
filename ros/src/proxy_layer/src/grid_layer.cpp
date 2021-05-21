#include <grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

PLUGINLIB_EXPORT_CLASS(proxy_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using namespace std;
namespace proxy_layer_namespace
{

  unsigned int costn = 10;
  unsigned int countit = 0;

  void GridLayer::onInitialize()
  {

    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;

    // get the base map (from map_server)
    boost::shared_ptr<nav_msgs::OccupancyGrid const> occ_grid_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/robot_0/map");
    occ_grid = *occ_grid_ptr;

    grid_resolution = occ_grid.info.resolution; // m/cell (default: 0.05)
    // The problem is that without the parameter server of the new wrapper,
    // we do not know if the whole image is the map or only a section of it (with a different origin)
    // as it is the case in the default map (which we had in the practical course).
    // So, I do a dirty workaround:
    // 1. If the wrapper is active, we can get check the parameters
    // 2. If not, then we check the map from the map_server. If it has the same signature
    // as the default map (res=0.05, 640x640 and origin=-16.2/-16.2), I assume we have the
    // default map and take the hardcoded section for it. Otherwise, I assume it is a new
    // map without a specific section and take the whole image as map, what is the default:
    real_section_min_x = occ_grid.info.origin.position.x; // = origin of the section
    real_section_min_y = occ_grid.info.origin.position.y; // = origin of the section
    real_section_max_x = real_section_min_x + occ_grid.info.width * grid_resolution;
    real_section_max_y = real_section_min_y + occ_grid.info.height * grid_resolution;
    string wrapper_namespace;
    bool active_wrapper = false;
    if (nh.getParam("/wrapper_namespace", wrapper_namespace))
    {
      nh.getParam("/" + wrapper_namespace + "/active_wrapper", active_wrapper);
    }

    if (active_wrapper)
    {
      // wrapper is active with section parameters
      bool use_section;
      nh.getParam("/" + wrapper_namespace + "/use_map_section", use_section);
      if (use_section)
      {
        nh.getParam("/" + wrapper_namespace + "/map_section_x_min", real_section_min_x);
        nh.getParam("/" + wrapper_namespace + "/map_section_y_min", real_section_min_y);
        nh.getParam("/" + wrapper_namespace + "/map_section_x_max", real_section_max_x);
        nh.getParam("/" + wrapper_namespace + "/map_section_y_max", real_section_max_y);
      }
    }
    else if ((int)(grid_resolution * 10000) == (int)(0.05 * 10000) && occ_grid.info.origin.position.x == -16.2 && occ_grid.info.origin.position.y == -16.2 && occ_grid.info.width == 640 && occ_grid.info.height == 640)
    {
      // should be the default map (the *10000 is because of the general floating problem)
      // then we can use the hardcoded values (these are nowhere else stored/available!)
      real_section_min_x = -5.1;
      real_section_min_y = -5.1;
      real_section_max_x = 5.1;
      real_section_max_y = 5.1;
    }
    width_in_cells = round((real_section_max_x - real_section_min_x) / grid_resolution);
    height_in_cells = round((real_section_max_y - real_section_min_y) / grid_resolution);
    // ROS_ERROR("\n\nINIT:");
    // cout << "x min: " << real_section_min_x << endl;
    // cout << "y min: " << real_section_min_y << endl;
    // cout << "x max: " << real_section_max_x << endl;
    // cout << "y max: " << real_section_max_y << endl;
    // cout << "res: " << grid_resolution << endl;
    // cout << "width: " << width_in_cells << endl;
    // cout << "height: " << height_in_cells << endl;

    costmap_layer_ = vector<signed char>(width_in_cells * height_in_cells, 0);
    // cout << "costmap array size (init): " << costmap_layer_.size() << endl;

    matchSize();

    grid_sub = nh.subscribe("/custom_layers/combined", 100, &GridLayer::gridCallback, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &GridLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  GridLayer::GridLayer()
  {
  }

  void GridLayer::gridCallback(const nav_msgs::OccupancyGrid &msg)
  { // TODO update origin etc.
    costmap_layer_ = msg.data;
  }

  void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void GridLayer::matchSize()
  {
    Costmap2D *master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

  void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                               double *min_y, double *max_x, double *max_y)
  {
    if (!enabled_)
      return;
    *min_x = min(real_section_min_x, *min_x);
    *min_y = min(real_section_min_y, *min_y);
    *max_x = max(real_section_max_x, *max_x);
    *max_y = max(real_section_max_y, *max_y);

  } // namespace simple_layer_namespace

  void GridLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                              int max_j)
  {
    if (!enabled_)
      return;

    // cout << "costmap array size (later): " << costmap_layer_.size() << endl;
    // cout << "x in cells: " << master_grid.getSizeInCellsX() << endl;
    // cout << "y in cells: " << master_grid.getSizeInCellsY() << endl;
    // cout << "x in m: " << master_grid.getSizeInMetersX() << endl;
    // cout << "y in m: " << master_grid.getSizeInMetersY() << endl;
    // cout << "res: " << master_grid.getResolution() << endl;
    // cout << "x origin: " << master_grid.getOriginX() << endl;
    // cout << "y origin: " << master_grid.getOriginY() << endl;

    uint i_0, j_0;
    worldToMap(real_section_min_x, real_section_min_y, i_0, j_0);

    uint i_N = i_0 + width_in_cells;
    uint j_N = j_0 + height_in_cells;

    for (int j = j_0; j < j_N; j++)
    {
      for (int i = i_0; i < i_N; i++)
      {
        int index = (int)(width_in_cells / 4) * (int)((j - j_0) / 4) + (int)((i - i_0) / 4);
        int c = master_grid.getCost(i, j);
        if ((c > 90))
        {
          int new_c = c;
          if (c < 140)
          {
            new_c = 100;
          }
          else if (c < 252)
          {
            new_c += 20;

            master_grid.setCost(i, j, min(252, new_c));
          }
          continue;
        }

        master_grid.setCost(i, j, costmap_layer_[index]);
      }
    }
  }

} // namespace proxy_layer_namespace