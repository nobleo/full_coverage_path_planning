//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <list>
#include <vector>

#include "full_coverage_path_planner/full_coverage_path_planner.hpp"

/*  *** Note the coordinate system ***
 *  grid[][] is a 2D-vector:
 *  where ix is column-index and x-coordinate in map,
 *  iy is row-index and y-coordinate in map.
 *
 *            Cols  [ix]
 *        _______________________
 *       |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 * Rows  |__|__|__|__|__|__|__|__|
 * [iy]  |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 *y-axis |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *
 *   O   --->> x-axis
 */

// #define DEBUG_PLOT

// Default Constructor
namespace full_coverage_path_planner
{
  FullCoveragePathPlanner::FullCoveragePathPlanner() : initialized_(false)
  {
  }

  void FullCoveragePathPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> &path)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    // create a message for the plan
    nav_msgs::msg::Path gui_path;
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
    for (const auto pose : path)
    {
      gui_path.poses.push_back(pose);
    }

    plan_pub_->publish(gui_path);
  }

  void FullCoveragePathPlanner::parsePointlist2Plan(const geometry_msgs::msg::PoseStamped &start,
                                                    std::list<Point_t> const &goalpoints,
                                                    std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    geometry_msgs::msg::PoseStamped new_goal;
    std::list<Point_t>::const_iterator it, it_next, it_prev;
    bool do_publish = false;
    double orientation = dir::none;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Received goalpoints with length: %lu", goalpoints.size());
    if(goalpoints.size()<1)
    {
      RCLCPP_WARN(rclcpp::get_logger("FullCoveragePathPlanner"), "Empty point list");
      return;
    }
    else if (goalpoints.size() == 1)
    {
      new_goal.header.frame_id = "map";
      new_goal.pose.position.x = (goalpoints.begin()->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
      new_goal.pose.position.y = (goalpoints.begin()->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
      new_goal.pose.orientation = createQuaternionMsgFromYaw(0);
      plan.push_back(new_goal);
    }
    else
    {
      for (it = goalpoints.begin(); it != goalpoints.end(); ++it)
      {
        it_next = it;
        it_next++;
        it_prev = it;
        it_prev--;

        int dx_now;
        int dy_now;
        int dx_next;
        int dy_next;
        // Check for the direction of movement
        if (it == goalpoints.begin())
        {
          dx_now = it_next->x - it->x;
          dy_now = it_next->y - it->y;
        }
        else
        {
          dx_now = it->x - it_prev->x;
          dy_now = it->y - it_prev->y;
          dx_next = it_next->x - it->x;
          dy_next = it_next->y - it->y;
        }

        // Calculate direction enum: dx + dy*2 will give a unique number for each of the four possible directions because
        // of their signs:
        //  1 +  0*2 =  1
        //  0 +  1*2 =  2
        // -1 +  0*2 = -1
        //  0 + -1*2 = -2
        int move_dir_now = dx_now + dy_now * 2;
        int move_dir_next = dx_next + dy_next * 2;

        // Check if this points needs to be published (i.e. a change of direction or first or last point in list)
        do_publish = move_dir_next != move_dir_now || it == goalpoints.begin() ||
                     (it != goalpoints.end() && it == --goalpoints.end());

        // Add to vector if required
        if (do_publish)
        {
          new_goal.header.frame_id = "map";
          new_goal.pose.position.x = (it->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
          new_goal.pose.position.y = (it->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
          // Calculate desired orientation to be in line with movement direction
          switch (move_dir_now)
          {
          case dir::none:
            // Keep orientation
            break;
          case dir::right:
            orientation = 0;
            break;
          case dir::up:
            orientation = M_PI / 2;
            break;
          case dir::left:
            orientation = M_PI;
            break;
          case dir::down:
            orientation = M_PI * 1.5;
            break;
          }
          new_goal.pose.orientation = createQuaternionMsgFromYaw(orientation);
          if (it != goalpoints.begin())
          {
            previous_goal_.pose.orientation = new_goal.pose.orientation;
            // republish previous goal but with new orientation to indicate change of direction
            // useful when the plan is strictly followed with base_link
            plan.push_back(previous_goal_);
          }
          RCLCPP_DEBUG(rclcpp::get_logger("FullCoveragePathPlanner"), "Voila new point: x=%f, y=%f, o=%f,%f,%f,%f", new_goal.pose.position.x, new_goal.pose.position.y,
                       new_goal.pose.orientation.x, new_goal.pose.orientation.y, new_goal.pose.orientation.z,
                       new_goal.pose.orientation.w);
          plan.push_back(new_goal);
          previous_goal_ = new_goal;
        }
      }
    }

    /* Add poses from current position to start of plan */

    // Compute angle between current pose and first plan point
    double dy = plan.begin()->pose.position.y - start.pose.position.y;
    double dx = plan.begin()->pose.position.x - start.pose.position.x;
    // Arbitrary choice of 100.0*FLT_EPSILON to determine minimum angle precision of 1%
    if (!(fabs(dy) < 100.0 * FLT_EPSILON && fabs(dx) < 100.0 * FLT_EPSILON))
    {
      // Add extra translation waypoint
      double yaw = std::atan2(dy, dx);
      geometry_msgs::msg::Quaternion quat_temp = createQuaternionMsgFromYaw(yaw);
      geometry_msgs::msg::PoseStamped extra_pose;
      extra_pose = *plan.begin();
      extra_pose.pose.orientation = quat_temp;
      plan.insert(plan.begin(), extra_pose);
      extra_pose = start;
      extra_pose.pose.orientation = quat_temp;
      plan.insert(plan.begin(), extra_pose);
    }

    // Insert current pose
    plan.insert(plan.begin(), start);

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan ready containing %lu goals!", plan.size());
  }

  bool FullCoveragePathPlanner::parseGrid(nav2_costmap_2d::Costmap2D const * cpp_costmap,
                                          std::vector<std::vector<bool>> &grid,
                                          double robotRadius,
                                          double toolRadius,
                                          geometry_msgs::msg::PoseStamped const &realStart,
                                          Point_t &scaledStart)
  {
    size_t nodeRow;
    size_t nodeColl;
    size_t nodeSize = dmax(floor(toolRadius / cpp_costmap->getResolution()), 1);       // Size of node in pixels/units
    size_t robotNodeSize = dmax(floor(robotRadius / cpp_costmap->getResolution()), 1); // RobotRadius in pixels/units
    size_t nRows = cpp_costmap->getSizeInCellsY();
    size_t nCols = cpp_costmap->getSizeInCellsX();
    unsigned char * cpp_costmap_data = cpp_costmap->getCharMap();
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "nRows: %lu nCols: %lu nodeSize: %lu", nRows, nCols, nodeSize);

    if (nRows == 0 || nCols == 0)
    {
      return false;
    }

    // Save map origin and scaling
    cpp_costmap->mapToWorld(0, 0, grid_origin_.x, grid_origin_.y);
    tile_size_ = nodeSize * cpp_costmap->getResolution(); // Size of a tile in meters
    // Scale starting point
    scaledStart.x = static_cast<unsigned int>(clamp((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
                                                    floor(cpp_costmap->getSizeInCellsX() / tile_size_)));
    scaledStart.y = static_cast<unsigned int>(clamp((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
                                                    floor(cpp_costmap->getSizeInCellsY() / tile_size_)));

    // Scale grid
    for (size_t iy = 0; iy < nRows; iy = iy + nodeSize)
    {
      std::vector<bool> gridRow;
      for (size_t ix = 0; ix < nCols; ix = ix + nodeSize)
      {
        bool nodeOccupied = false;
        for (nodeRow = 0; (nodeRow < robotNodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
        {
          for (nodeColl = 0; (nodeColl < robotNodeSize) && ((ix + nodeColl) < nCols); ++nodeColl)
          {
            int index_grid = dmax((iy + nodeRow - ceil(static_cast<double>(robotNodeSize - nodeSize) / 2.0)) * nCols + (ix + nodeColl - ceil(static_cast<double>(robotNodeSize - nodeSize) / 2.0)), 0);
            if (cpp_costmap_data[index_grid] > COVERAGE_COST)
            {
              nodeOccupied = true;
              break;
            }
          }
        }
        gridRow.push_back(nodeOccupied);
      }
      grid.push_back(gridRow);
    }
    return true;
  }
} // namespace full_coverage_path_planner
