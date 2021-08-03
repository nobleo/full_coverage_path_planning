//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
/** include the libraries you need in your planner here */
/** for global path planner interface */
#pragma once

#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/common.hpp"
#include "rclcpp/rclcpp.hpp"
#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::string;

// #define DEBUG_PLOT

#ifndef dabs
#define dabs(a) ((a) >= 0 ? (a) : -(a))
#endif
#ifndef dmin
#define dmin(a, b) ((a) <= (b) ? (a) : (b))
#endif
#ifndef dmax
#define dmax(a, b) ((a) >= (b) ? (a) : (b))
#endif
#ifndef clamp
#define clamp(a, lower, upper) dmax(dmin(a, upper), lower)
#endif


namespace full_coverage_path_planner
{

  enum dir
  {
    none = 0,
    right = 1,
    up = 2,
    left = -1,
    down = -2,
  };

  class FullCoveragePathPlanner
  {
  public:
    /**
     * @brief  Default constructor for the NavFnROS object
     */
    FullCoveragePathPlanner();
    // FullCoveragePathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief  Publish a path for visualization purposes
     */
    void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path);

    ~FullCoveragePathPlanner()
    {
    }
  static const unsigned char COVERAGE_COST = 65;  // Cost for checking coverage. Perhaps define this in coverage costmap plugin?

  protected:
    /**
     * @brief Convert internal representation of a to a ROS path
     * @param start Start pose of robot
     * @param goalpoints Goal points from Spiral Algorithm
     * @param plan  Output plan variable
     */
    void parsePointlist2Plan(const geometry_msgs::msg::PoseStamped &start, std::list<Point_t> const &goalpoints,
                             std::vector<geometry_msgs::msg::PoseStamped> &plan);

    /**
     * @brief Convert ROS Occupancy grid to internal grid representation, given the size of a single tile
     * @param cpp_grid_ ROS occupancy grid representation. Cells higher that 65 are considered occupied
     * @param grid internal map representation
     * @param tileSize size (in meters) of a cell. This can be the robot's size
     * @param realStart Start position of the robot (in meters)
     * @param scaledStart Start position of the robot on the grid
     * @return success
     */
    bool parseGrid(nav2_costmap_2d::Costmap2D const * cpp_costmap,
                   std::vector<std::vector<bool>> &grid,
                   double robotRadius,
                   double toolRadius,
                   geometry_msgs::msg::PoseStamped const &realStart,
                   Point_t &scaledStart);

    /**
     * @brief Create Quaternion from Yaw
     * @param yaw orientation
     * @return Quaternion with desired yaw orientation
     */

    auto createQuaternionMsgFromYaw(double yaw)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      return tf2::toMsg(q);
    }
    nav2_util::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    double robot_radius_;
    double tool_radius_;
    double plan_resolution_;
    double tile_size_;
    dPoint_t grid_origin_;
    bool initialized_;
    geometry_msgs::msg::PoseStamped previous_goal_;
    std::string name_, global_frame_;
    nav2_costmap_2d::Costmap2D * costmap_;

    struct spiral_cpp_metrics_type
    {
      int visited_counter;
      int multiple_pass_counter;
      int accessible_counter;
      double total_area_covered;
    };
    spiral_cpp_metrics_type spiral_cpp_metrics_;
  };

  /**
   * Sort function for sorting Points on distance to a POI
   */
  struct ComparatorForPointSort
  {
    explicit ComparatorForPointSort(Point_t poi) : _poi(poi)
    {
    }

    bool operator()(const Point_t &first, const Point_t &second) const
    {
      return distanceSquared(first, _poi) < distanceSquared(second, _poi);
    }

  private:
    Point_t _poi;
  };
} // namespace full_coverage_path_planner
