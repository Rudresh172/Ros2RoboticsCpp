#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <random>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <limits>

// ROS2 shorthand
using Marker      = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

/*******************************************************
 *  1) Data Structures
 ******************************************************/
struct Point2D {
  double x;
  double y;
};

/**
 * RRT Node:
 *  - (x, y): location
 *  - path_x, path_y: intermediate points from parent to this node
 *  - parent_idx: index of the parent in the tree
 */
struct RRTNode {
  std::vector<double> path_x;
  std::vector<double> path_y;
  double x;
  double y;
  int parent_idx;
};

/*******************************************************
 *  2) RRT Planner Node
 ******************************************************/
class RRTPlannerNode : public rclcpp::Node
{
public:
  RRTPlannerNode()
  : Node("rrt_planner_node"),
    rng_(std::random_device{}()), 
    iteration_(0),
    found_path_(false)
  {
    RCLCPP_INFO(this->get_logger(), "RRT Planner Node Started!");

    // Publisher for visual markers
    marker_pub_ = this->create_publisher<MarkerArray>("rrt_markers", 10);

    // Initialize parameters
    initParameters();
    
    // Create some obstacles, for example:
    createObstacles();

    // Initialize start & goal
    start_ = {0.0, 0.0};
    goal_  = {10.0, 8.0};

    // Prepare
    initRRT();

    // Timer: expand RRT at ~1 Hz (adjust as desired)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RRTPlannerNode::timerCallback, this)
    );
  }

private:
  /*****************************************************
   * initParameters
   *****************************************************/
  void initParameters()
  {
    // Hardcode or load from param server:
    min_rand_       = -2.0;
    max_rand_       =  15.0;
    goal_sample_rate_=  5;   // percent of times we sample the goal
    max_iterations_ = 500;
    expand_dis_     = 3.0;
    path_res_       = 0.5;
    robot_radius_   = 0.8;  // collision radius
  }

  /*****************************************************
   * createObstacles
   *****************************************************/
  void createObstacles()
  {
    // Example set: (x, y, radius)
    // You can add more obstacles
    obstacles_.push_back({5.0, 5.0, 1.0});
    obstacles_.push_back({3.0, 6.0, 2.0});
    obstacles_.push_back({7.0, 6.0, 2.0});
    obstacles_.push_back({9.0, 5.0, 1.0});
  }

  /*****************************************************
   * initRRT
   *****************************************************/
  void initRRT()
  {
    // Clear any old data
    rrt_tree_.clear();
    iteration_ = 0;
    found_path_ = false;

    // Create first node from start
    RRTNode start_node;
    start_node.x = start_.x;
    start_node.y = start_.y;
    start_node.parent_idx = -1;
    start_node.path_x.push_back(start_.x);
    start_node.path_y.push_back(start_.y);
    rrt_tree_.push_back(start_node);
  }

  /*****************************************************
   * timerCallback
   * - Called every second for demonstration
   *****************************************************/
  void timerCallback()
  {
    if (found_path_) {
      // Already found path; just keep re-publishing so Rviz sees it
      publishMarkers();
      return;
    }

    if (iteration_ > max_iterations_) {
      RCLCPP_WARN(this->get_logger(), "RRT: Reached max iterations, no path found!");
      publishMarkers();
      return;
    }

    iteration_++;
    RCLCPP_INFO(this->get_logger(), "RRT iteration: %d", iteration_);

    // 1) Get random node
    RRTNode rand_node = getRandomNode();

    // 2) Find nearest node index
    int nearest_index = getNearestNodeIndex(rand_node);

    // 3) Steer from that node's index to rand_node
    RRTNode new_node = steer(nearest_index, rand_node, expand_dis_);

    // 4) Collision check, etc...
    if (collisionFree(new_node)) {
        rrt_tree_.push_back(new_node);
    
        // 5) Check if near goal
        double dist_to_goal = distance2D(new_node.x, new_node.y, goal_.x, goal_.y);
        if (dist_to_goal <= expand_dis_) {
          // Convert 'goal_' to an RRTNode
          RRTNode gnode = makeNode(goal_);
          // Steer from the newly added node index (rrt_tree_.size()-1) to the goal
          RRTNode final_node = steer(rrt_tree_.size() - 1, gnode, expand_dis_);
          if (collisionFree(final_node)) {
            // We made it
            rrt_tree_.push_back(final_node);
            found_path_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached in iteration %d!", iteration_);
            buildFinalPath(rrt_tree_.size() - 1);
          }
        }
    }

    // 6) Publish intermediate markers
    publishMarkers();
  }

  /*****************************************************
   * getRandomNode
   *****************************************************/
  RRTNode getRandomNode()
  {
    std::uniform_int_distribution<> dist100(0, 100);
    std::uniform_real_distribution<> distReal(min_rand_, max_rand_);

    RRTNode rnd;
    // With goal_sample_rate_% probability, sample the goal
    if (dist100(rng_) > goal_sample_rate_) {
      rnd.x = distReal(rng_);
      rnd.y = distReal(rng_);
    } else {
      rnd.x = goal_.x;
      rnd.y = goal_.y;
    }
    rnd.parent_idx = -1;
    return rnd;
  }

  /*****************************************************
   * makeNode from point
   *****************************************************/
  RRTNode makeNode(const Point2D& pt)
  {
    RRTNode n;
    n.x = pt.x;
    n.y = pt.y;
    n.parent_idx = -1;
    return n;
  }

  /*****************************************************
   * getNearestNodeIndex
   *****************************************************/
  int getNearestNodeIndex(const RRTNode &rnd)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    int nearest_idx = -1;
    for (size_t i = 0; i < rrt_tree_.size(); ++i) {
      double d = distance2D(rrt_tree_[i].x, rrt_tree_[i].y, rnd.x, rnd.y);
      if (d < min_dist) {
        min_dist = d;
        nearest_idx = (int)i;
      }
    }
    return nearest_idx;
  }

  /*****************************************************
   * steer
   *****************************************************/
  RRTNode steer(int from_idx, const RRTNode &to_node, double extend_length)
{
    RRTNode new_node;
    // set new_node's parent
    new_node.parent_idx = from_idx;

    // convenience
    const RRTNode &from_node = rrt_tree_[from_idx];

    // direction
    double dx = to_node.x - from_node.x;
    double dy = to_node.y - from_node.y;
    double d  = std::hypot(dx, dy);
    double theta = std::atan2(dy, dx);

    double actual_extend = std::min(extend_length, d);

    int steps = (int)std::floor(actual_extend / path_res_);
    double x = from_node.x;
    double y = from_node.y;
    for (int i = 0; i < steps; ++i) {
        x += path_res_ * std::cos(theta);
        y += path_res_ * std::sin(theta);
        new_node.path_x.push_back(x);
        new_node.path_y.push_back(y);
    }

    double remaining = d - steps * path_res_;
    if (remaining < path_res_) {
        // snap exactly to 'to_node'
        new_node.path_x.push_back(to_node.x);
        new_node.path_y.push_back(to_node.y);
        new_node.x = to_node.x;
        new_node.y = to_node.y;
    } else {
        // just end at last step
        new_node.x = x;
        new_node.y = y;
    }

    return new_node;
}

  /*****************************************************
   * collisionFree
   *****************************************************/
  bool collisionFree(const RRTNode &node)
  {
    // For each sub-point in node.path_x / path_y, check distance to obstacles
    if (node.path_x.empty()) {
      // That means we didn't move anywhere, interpret as safe or not
      // Typically won't happen except from start node or a 0-length steer
      return true;
    }

    for (size_t ip = 0; ip < node.path_x.size(); ++ip) {
      double px = node.path_x[ip];
      double py = node.path_y[ip];
      // check each obstacle
      for (auto &obs : obstacles_) {
        double dx = obs.x - px;
        double dy = obs.y - py;
        double r  = obs.radius + robot_radius_;
        if (dx*dx + dy*dy <= r*r) {
          return false; // collision
        }
      }
    }
    return true;
  }

  /*****************************************************
   * buildFinalPath
   *****************************************************/
  void buildFinalPath(int goal_idx)
  {
    final_path_.clear();
    int idx = goal_idx;
    while (idx != -1) {
      final_path_.push_back({rrt_tree_[idx].x, rrt_tree_[idx].y});
      idx = rrt_tree_[idx].parent_idx;
    }
    // Reverse
    std::reverse(final_path_.begin(), final_path_.end());
  }

  /*****************************************************
   * publishMarkers
   *****************************************************/
  void publishMarkers()
  {
    MarkerArray msg_arr;
    msg_arr.markers.reserve(4);

    rclcpp::Time now = this->now();

    // A) Obstacles
    {
      Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "rrt_obstacles";
      m.id = 0;
      m.type = Marker::POINTS;
      m.action = Marker::ADD;
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      for (auto &o : obstacles_) {
        geometry_msgs::msg::Point p;
        p.x = o.x; p.y = o.y; p.z = 0.0;
        m.points.push_back(p);
      }
      msg_arr.markers.push_back(m);
    }

    // B) RRT Edges
    {
      Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "rrt_tree";
      m.id = 1;
      m.type = Marker::LINE_LIST;
      m.action = Marker::ADD;
      m.scale.x = 0.03;
      m.color.a = 1.0;
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;  // green lines

      for (auto &node : rrt_tree_) {
        if (node.parent_idx < 0) continue;
        // draw from parent's final position to each path point
        geometry_msgs::msg::Point prev_pt;
        prev_pt.x = rrt_tree_[node.parent_idx].x;
        prev_pt.y = rrt_tree_[node.parent_idx].y;

        for (size_t i = 0; i < node.path_x.size(); ++i) {
          geometry_msgs::msg::Point this_pt;
          this_pt.x = node.path_x[i];
          this_pt.y = node.path_y[i];

          m.points.push_back(prev_pt);
          m.points.push_back(this_pt);

          prev_pt = this_pt;
        }
      }
      msg_arr.markers.push_back(m);
    }

    // C) Start & Goal Markers
    {
      Marker mg;
      mg.header.frame_id = "map";
      mg.header.stamp = now;
      mg.ns = "rrt_start_goal";
      mg.id = 2;
      mg.type = Marker::POINTS;
      mg.action = Marker::ADD;
      mg.scale.x = 0.4;
      mg.scale.y = 0.4;
      mg.color.a = 1.0;
      mg.color.r = 1.0;
      mg.color.g = 0.0;
      mg.color.b = 0.0;

      // start in blue?
      geometry_msgs::msg::Point ps;
      ps.x = start_.x; ps.y = start_.y; ps.z = 0.0;
      mg.points.push_back(ps);

      // goal in red?
      geometry_msgs::msg::Point pg;
      pg.x = goal_.x; pg.y = goal_.y; pg.z = 0.0;
      mg.points.push_back(pg);

      msg_arr.markers.push_back(mg);
    }

    // D) Final Path
    if (!final_path_.empty()) {
      Marker mp;
      mp.header.frame_id = "map";
      mp.header.stamp    = now;
      mp.ns = "rrt_final_path";
      mp.id = 3;
      mp.type = Marker::LINE_STRIP;
      mp.action = Marker::ADD;
      mp.scale.x = 0.1;
      mp.color.a = 1.0;
      mp.color.r = 1.0;
      mp.color.g = 0.0;
      mp.color.b = 0.0; // red

      for (auto &p : final_path_) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x; pt.y = p.y; pt.z = 0.0;
        mp.points.push_back(pt);
      }
      msg_arr.markers.push_back(mp);
    }

    marker_pub_->publish(msg_arr);
  }

  /*****************************************************
   * distance2D
   *****************************************************/
  double distance2D(double x1, double y1, double x2, double y2)
  {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::hypot(dx, dy);
  }

private:
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Publisher
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  // RNG
  std::mt19937 rng_;

  // RRT config
  double min_rand_;
  double max_rand_;
  int    goal_sample_rate_;
  int    max_iterations_;
  double expand_dis_;
  double path_res_;
  double robot_radius_;

  // Obstacles
  struct Obstacle {
    double x;
    double y;
    double radius;
  };
  std::vector<Obstacle> obstacles_;

  // Start and goal
  Point2D start_;
  Point2D goal_;

  // RRT tree
  std::vector<RRTNode> rrt_tree_;

  // Final path
  std::vector<Point2D> final_path_;

  // Status
  int  iteration_;
  bool found_path_;
};

/*****************************************************
 * main
 *****************************************************/
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RRTPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
