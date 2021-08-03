//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/spiral_stc.hpp"

using nav2_util::declare_parameter_if_not_declared;
namespace full_coverage_path_planner
{
  SpiralSTC::SpiralSTC()
  {
  }

  SpiralSTC::~SpiralSTC()
  {
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Destroying plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void SpiralSTC::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    if (!initialized_)
    {
      // Get node from parent
      node_ = parent.lock();
      name_ = name;

      // Currently this plugin does not use the costmap, instead request a map from a server
      // This will change in the future
      costmap_ = costmap_ros->getCostmap();
      global_frame_ = costmap_ros->getGlobalFrameID();

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),
                  "Configuring plugin %s of type NavfnPlanner", name_.c_str());

      // Create a publisher to visualize the plan
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);

      // Define  robot radius (radius) parameter
      double robot_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".robot_radius", rclcpp::ParameterValue(robot_radius_default));
      node_->get_parameter(name_ + ".robot_radius", robot_radius_);
      // Define  tool radius (radius) parameter
      double tool_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".tool_radius", rclcpp::ParameterValue(tool_radius_default));
      node_->get_parameter(name_ + ".tool_radius", tool_radius_);
      initialized_ = true;
    }
  }

  void SpiralSTC::activate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Activating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void SpiralSTC::deactivate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Deactivating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void SpiralSTC::cleanup()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Cleaning up plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    // TODO(clopez) Add proper cleanup
  }

  nav_msgs::msg::Path SpiralSTC::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path global_path;
    SpiralSTC::makePlan(start, goal, global_path.poses);

    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    return global_path;
  }
  std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool>> const &grid, std::list<gridNode_t> &init,
                                          std::vector<std::vector<bool>> &visited)
  {
    int nRows = grid.size();
    int nCols = grid[0].size();
    // Spiral filling of the open space
    // Copy incoming list to 'end'
    std::list<gridNode_t> pathNodes(init);
    // Create iterator for gridNode_t list and let it point to the last element of end
    std::list<gridNode_t>::iterator it = --(pathNodes.end());
    if (pathNodes.size() > 1) // if list is length 1, keep iterator at end
      it--;                   // Let iterator point to second to last element

    gridNode_t prev = *(it);
    bool done = false;
    while (!done)
    {
      // Initialize spiral direction towards y-axis
      int dx = 0;
      int dy = 1;
      int dx_prev;
      if (it != pathNodes.begin())
      {
        // turn ccw
        dx = pathNodes.back().pos.x - prev.pos.x;
        dy = pathNodes.back().pos.y - prev.pos.y;
        dx_prev = dx;
        dx = -dy;
        dy = dx_prev;
      }
      // This condition might change in the loop bellow
      done = true;
      // loop over the four possible directions: up, right, down, left
      for (size_t i = 0; i < 4; ++i)
      {
        int x2 = pathNodes.back().pos.x + dx;
        int y2 = pathNodes.back().pos.y + dy;
        if (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)
        {
          if (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen)
          {
            Point_t new_point = {x2, y2};
            gridNode_t new_node =
                {
                    new_point, // Point: x,y
                    0,         // Cost
                    0,         // Heuristic
                };
            prev = pathNodes.back();
            pathNodes.push_back(new_node);
            it = --(pathNodes.end());
            visited[y2][x2] = eNodeVisited; // Close node
            done = false;
            // brake for loop
            break;
          }
        }
        // try next direction cw
        dx_prev = dx;
        dx = dy;
        dy = -dx_prev;
      }
    }
    return pathNodes;
  }

  std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool>> const &grid,
                                           Point_t &init,
                                           int &multiple_pass_counter,
                                           int &visited_counter)
  {
    // Initial node is initially set as visited so it does not count
    multiple_pass_counter = 0;
    visited_counter = 0;

    std::vector<std::vector<bool>> visited;
    visited = grid; // Copy grid matrix
    int x = init.x;
    int y = init.y;

    Point_t new_point = {x, y};
    gridNode_t new_node =
        {
            new_point, // Point: x,y
            0,         // Cost
            0,         // Heuristic
        };
    std::list<gridNode_t> pathNodes;
    std::list<Point_t> fullPath;
    pathNodes.push_back(new_node);
    visited[y][x] = eNodeVisited;

#ifdef DEBUG_PLOT
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Grid before walking is: ");
    printGrid(grid, visited, fullPath);
#endif

    pathNodes = SpiralSTC::spiral(grid, pathNodes, visited);    // First spiral fill
    std::list<Point_t> goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints
    // Add points to full path
    std::list<gridNode_t>::iterator it;
    for (const auto pathNode : pathNodes)
    {
      Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
      visited_counter++;
      fullPath.push_back(newPoint);
    }
    // Remove all elements from pathNodes list except last element
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));

#ifdef DEBUG_PLOT
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Current grid after first spiral is");
    printGrid(grid, visited, fullPath);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "There are %d goals remaining", goals.size());
#endif
    while (goals.size() != 0)
    {
      // Remove all elements from pathNodes list except last element.
      // The last point is the starting point for a new search and A* extends the path from there on
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      visited_counter--; // First point is already counted as visited
      // Plan to closest open Node using A*
      // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
      //    to the nearest free space
      bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
      if (resign)
      {
#ifdef DEBUG_PLOT
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "A_star_to_open_space is resigning", goals.size());
#endif
        break;
      }

      // Update visited grid
      for (const auto pathNode : pathNodes)
      {
        if (visited[pathNode.pos.y][pathNode.pos.x])
        {
          multiple_pass_counter++;
        }
        visited[pathNode.pos.y][pathNode.pos.x] = eNodeVisited;
      }
      if (pathNodes.size() > 0)
      {
        multiple_pass_counter--; // First point is already counted as visited
      }

#ifdef DEBUG_PLOT
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Grid with path marked as visited is:");
      gridNode_t SpiralStart = pathNodes.back();
      printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif

      // Spiral fill from current position
      pathNodes = spiral(grid, pathNodes, visited);

#ifdef DEBUG_PLOT
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Visited grid updated after spiral:");
      printGrid(grid, visited, pathNodes, SpiralStart, pathNodes.back());
#endif

      goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints

      for (const auto pathNode : pathNodes)
      {
        Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
        visited_counter++;
        fullPath.push_back(newPoint);
      }
    }

    return fullPath;
  }

  bool SpiralSTC::makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
                           std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Initialized!");
    }

    clock_t begin = clock();
    Point_t startPoint;

    std::vector<std::vector<bool>> grid;
    if (!parseGrid(costmap_, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Could not parse retrieved grid");
      return false;
    }

#ifdef DEBUG_PLOT
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Start grid is:");
    std::list<Point_t> printPath;
    printPath.push_back(startPoint);
    printGrid(grid, grid, printPath);
#endif

    std::list<Point_t> goalPoints = spiral_stc(grid,
                                               startPoint,
                                               spiral_cpp_metrics_.multiple_pass_counter,
                                               spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "naive cpp completed!");
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Converting path to plan");

    parsePointlist2Plan(start, goalPoints, plan);
    // Print some metrics:
    spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter - spiral_cpp_metrics_.multiple_pass_counter;
    spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total visited: %d", spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

    // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
    // (also controlled by planner_frequency parameter in move_base namespace)

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Publishing plan!");
    publishPlan(plan);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published!");
    RCLCPP_DEBUG(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published");

    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Elapsed time: %f", elapsed_secs);

    return true;
  }
} // namespace full_coverage_path_planner

// register this planner as a nav2_core::GlobalPlanner plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav2_core::GlobalPlanner)