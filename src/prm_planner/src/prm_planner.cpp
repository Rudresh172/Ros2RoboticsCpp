#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <random>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

struct Point2D {
  double x;
  double y;
};

/// Graph node used in Dijkstra
struct GraphNode {
  Point2D pt;
  double cost;
  int parent_index;
};

class PRMPublisher : public rclcpp::Node
{
public:
  PRMPublisher()
  : Node("prm_publisher"),
    current_step_done_(false),
    goal_found_(false),
    dijkstra_initialized_(false)
  {
    RCLCPP_INFO(this->get_logger(), "PRM Publisher node started.");

    // 1) Publisher for markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("prm_planner_markers", 10);

    // 2) Start a timer that ticks at ~10 Hz (or slower if you want slower animation)
    timer_ = this->create_wall_timer(10ms, std::bind(&PRMPublisher::timerCallback, this));

    // 3) Initialize parameters
    N_SAMPLE_     = 500;
    N_KNN_        = 10;
    MAX_EDGE_LEN_ = 30.0;
    robot_radius_ = 2.0;

    start_.x = 10.0;  start_.y = 10.0;
    goal_.x  = 50.0;  goal_.y  = 50.0;

    // Create obstacles, sample, build adjacency
    createObstacles();
    samplePoints();
    buildRoadmap();
    initDijkstra();  // get Dijkstra data structures ready

    RCLCPP_INFO(this->get_logger(), "Initialization complete. Animation begins!");
  }

private:
  /*******************************
   * Timer callback
   *******************************/
  void timerCallback()
  {
    if (!dijkstra_initialized_) {
      return; // Wait until everything is ready
    }

    // If the goal is already found or we failed to find a path, just keep re-publishing final.
    if (goal_found_) {
      publishMarkers();
      return;
    }

    // 1) Do exactly ONE step of Dijkstra
    goal_found_ = dijkstraOneStep();

    // 2) Publish the intermediate or final markers
    publishMarkers();
  }

  /*******************************
   *  Create obstacles
   *******************************/
  void createObstacles()
  {
    obstacles_.clear();

    // Example: boundary walls
    for (int i = 0; i <= 60; ++i) {
      obstacles_.push_back({(double)i, 0.0});
      obstacles_.push_back({(double)i, 60.0});
    }
    for (int j = 0; j <= 60; ++j) {
      obstacles_.push_back({0.0, (double)j});
      obstacles_.push_back({60.0, (double)j});
    }

    // Some extra lines
    for (int j = 0; j < 40; ++j) {
      obstacles_.push_back({20.0, (double)j});
    }
    for (int j = 0; j < 40; ++j) {
      obstacles_.push_back({40.0, 60.0 - (double)j});
    }
  }

  /*******************************
   *  Sample free points
   *******************************/
  void samplePoints()
  {
    // bounding box
    double min_x = 1e9, max_x = -1e9;
    double min_y = 1e9, max_y = -1e9;
    for (auto &obs : obstacles_) {
      min_x = std::min(min_x, obs.x);
      max_x = std::max(max_x, obs.x);
      min_y = std::min(min_y, obs.y);
      max_y = std::max(max_y, obs.y);
    }
    min_x = std::max(0.0, min_x);
    min_y = std::max(0.0, min_y);
    max_x = std::max(max_x, 60.0);
    max_y = std::max(max_y, 60.0);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> distX(min_x, max_x);
    std::uniform_real_distribution<> distY(min_y, max_y);

    // gather samples
    samples_.clear();
    int count = 0;
    while ((int)samples_.size() < N_SAMPLE_ && count < 5*N_SAMPLE_) {
      double rx = distX(gen);
      double ry = distY(gen);
      if (!pointInCollision({rx, ry}, robot_radius_)) {
        samples_.push_back({rx, ry});
      }
      count++;
    }

    // add start & goal
    samples_.push_back(start_);
    samples_.push_back(goal_);
  }

  /*******************************
   *  Build adjacency
   *******************************/
  void buildRoadmap()
  {
    adjacency_.clear();
    adjacency_.resize(samples_.size());

    int n = (int)samples_.size();
    for (int i = 0; i < n; ++i) {
      // gather distance to others
      std::vector<std::pair<double,int>> dists;
      dists.reserve(n);
      for (int j = 0; j < n; ++j) {
        if (j == i) continue;
        double dx = samples_[j].x - samples_[i].x;
        double dy = samples_[j].y - samples_[i].y;
        double dd = std::hypot(dx, dy);
        dists.push_back({dd, j});
      }
      // sort
      std::sort(dists.begin(), dists.end(),
                [](auto &a, auto &b){return a.first < b.first;});

      // pick up to N_KNN feasible edges
      int edges = 0;
      for (auto &kv : dists) {
        if (kv.first > MAX_EDGE_LEN_) break;
        if (!segmentInCollision(samples_[i], samples_[kv.second], robot_radius_)) {
          adjacency_[i].push_back(kv.second);
          edges++;
          if (edges >= N_KNN_) break;
        }
      }
    }
  }

  /*******************************
   *  Prepare Dijkstra data
   *******************************/
  void initDijkstra()
  {
    int n = (int)samples_.size();
    open_list_.resize(n);
    visited_.resize(n, false);

    // fill node info
    for (int i = 0; i < n; ++i) {
      open_list_[i].pt = samples_[i];
      open_list_[i].cost = std::numeric_limits<double>::infinity();
      open_list_[i].parent_index = -1;
    }

    start_idx_ = n - 2;  // last two are start, goal
    goal_idx_  = n - 1;
    open_list_[start_idx_].cost = 0.0;

    dijkstra_initialized_ = true;
    goal_found_ = false;
  }

  /*******************************
   *  Perform ONE step of Dijkstra
   *******************************/
  bool dijkstraOneStep()
  {
    // 1) pick the node with smallest cost not visited
    double min_cost = std::numeric_limits<double>::infinity();
    int c_id = -1;
    int n = (int)open_list_.size();
    for (int i = 0; i < n; ++i) {
      if (!visited_[i] && open_list_[i].cost < min_cost) {
        min_cost = open_list_[i].cost;
        c_id     = i;
      }
    }
    if (c_id == -1) {
      // no node left
      RCLCPP_WARN(this->get_logger(), "No more nodes to expand; no path found!");
      path_.clear();
      return true;  // effectively "done"
    }

    // if c_id is goal -> done
    if (c_id == goal_idx_) {
      RCLCPP_INFO(this->get_logger(), "Goal found at cost %f!", open_list_[c_id].cost);
      backtrackPath();
      return true;
    }

    // 2) mark visited
    visited_[c_id] = true;

    // 3) relax edges
    for (auto &nbr : adjacency_[c_id]) {
      if (visited_[nbr]) continue;
      double dx = samples_[nbr].x - open_list_[c_id].pt.x;
      double dy = samples_[nbr].y - open_list_[c_id].pt.y;
      double dd = std::hypot(dx, dy);

      double cost_new = open_list_[c_id].cost + dd;
      if (cost_new < open_list_[nbr].cost) {
        open_list_[nbr].cost = cost_new;
        open_list_[nbr].parent_index = c_id;
      }
    }

    // If we haven't found goal yet, keep going
    return false;
  }

  /*******************************
   *  Reconstruct path
   *******************************/
  void backtrackPath()
  {
    path_.clear();
    int cur = goal_idx_;
    while (cur != -1) {
      path_.push_back(open_list_[cur].pt);
      cur = open_list_[cur].parent_index;
    }
    std::reverse(path_.begin(), path_.end());
  }

  /*******************************
   *  Collision checks
   *******************************/
  bool pointInCollision(const Point2D &p, double radius)
  {
    for (auto &obs : obstacles_) {
      double dx = obs.x - p.x;
      double dy = obs.y - p.y;
      if (std::hypot(dx, dy) <= radius) {
        return true;
      }
    }
    return false;
  }

  bool segmentInCollision(const Point2D &p1, const Point2D &p2, double radius)
  {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dist = std::hypot(dx, dy);
    if (dist > MAX_EDGE_LEN_) return true;
    int steps = std::max(1, (int)std::round(dist / radius));
    double stepx = dx / steps;
    double stepy = dy / steps;

    double xx = p1.x;
    double yy = p1.y;
    for (int i = 0; i <= steps; ++i) {
      if (pointInCollision({xx, yy}, radius)) {
        return true;
      }
      xx += stepx;
      yy += stepy;
    }
    return false;
  }

  /*******************************
   *  Publish Markers
   *******************************/
  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray m_array;
    m_array.markers.reserve(5);

    rclcpp::Time now = this->now();

    // 1) Obstacles
    {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp = now;
      mk.ns = "prm_obstacles";
      mk.id = 0;
      mk.type = mk.POINTS;
      mk.action = mk.ADD;
      mk.scale.x = 0.3;
      mk.scale.y = 0.3;
      mk.color.a = 1.0;
      mk.color.r = 0.0;
      mk.color.g = 0.0;
      mk.color.b = 0.0;

      for (auto &obs : obstacles_) {
        geometry_msgs::msg::Point pp;
        pp.x = obs.x;
        pp.y = obs.y;
        mk.points.push_back(pp);
      }
      m_array.markers.push_back(mk);
    }

    // 2) Samples
    {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp = now;
      mk.ns = "prm_samples";
      mk.id = 1;
      mk.type = mk.POINTS;
      mk.action = mk.ADD;
      mk.scale.x = 0.3;
      mk.scale.y = 0.3;
      mk.color.a = 1.0;
      mk.color.r = 0.0;
      mk.color.g = 0.0;
      mk.color.b = 1.0; // blue

      for (auto &s : samples_) {
        geometry_msgs::msg::Point pp;
        pp.x = s.x;
        pp.y = s.y;
        mk.points.push_back(pp);
      }
      m_array.markers.push_back(mk);
    }

    // 3) Roadmap edges
    {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp = now;
      mk.ns = "prm_edges";
      mk.id = 2;
      mk.type = mk.LINE_LIST;
      mk.action = mk.ADD;
      mk.scale.x = 0.05;
      mk.color.a = 0.3;
      mk.color.r = 0.0;
      mk.color.g = 0.0;
      mk.color.b = 0.0;

      int n = (int)samples_.size();
      for (int i = 0; i < n; ++i) {
        // If you want partial expansions:
        // you could only show edges from visited or from open_list
        for (auto &j : adjacency_[i]) {
          geometry_msgs::msg::Point p1, p2;
          p1.x = samples_[i].x; 
          p1.y = samples_[i].y;
          p2.x = samples_[j].x; 
          p2.y = samples_[j].y;
          mk.points.push_back(p1);
          mk.points.push_back(p2);
        }
      }
      m_array.markers.push_back(mk);
    }

    // 4) Visited / Expanded nodes (optional)
    {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp = now;
      mk.ns = "expanded_nodes";
      mk.id = 3;
      mk.type = mk.POINTS;
      mk.action = mk.ADD;
      mk.scale.x = 0.4;
      mk.scale.y = 0.4;
      mk.color.a = 0.8;
      mk.color.r = 0.0;
      mk.color.g = 1.0;
      mk.color.b = 0.0;  // green

      for (size_t i = 0; i < visited_.size(); i++) {
        if (visited_[i]) {
          geometry_msgs::msg::Point pp;
          pp.x = samples_[i].x;
          pp.y = samples_[i].y;
          mk.points.push_back(pp);
        }
      }
      m_array.markers.push_back(mk);
    }

    // 5) Final path (if found)
    if (!path_.empty()) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = "map";
      mk.header.stamp = now;
      mk.ns = "prm_path";
      mk.id = 4;
      mk.type = mk.LINE_STRIP;
      mk.action = mk.ADD;
      mk.scale.x = 0.3;
      mk.color.a = 1.0;
      mk.color.r = 1.0;
      mk.color.g = 0.0;
      mk.color.b = 0.0; // red line

      for (auto &p : path_) {
        geometry_msgs::msg::Point pp;
        pp.x = p.x;
        pp.y = p.y;
        mk.points.push_back(pp);
      }
      m_array.markers.push_back(mk);
    }

    marker_pub_->publish(m_array);
  }

private:
  // ROS things
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // PRM parameters
  int    N_SAMPLE_;
  int    N_KNN_;
  double MAX_EDGE_LEN_;
  double robot_radius_;

  // Start / goal
  Point2D start_, goal_;

  // Obstacles
  std::vector<Point2D> obstacles_;
  
  // Sampled free points
  std::vector<Point2D> samples_;

  // adjacency list
  std::vector<std::vector<int>> adjacency_;

  // Dijkstra data
  std::vector<GraphNode> open_list_;
  std::vector<bool> visited_;
  bool dijkstra_initialized_;
  bool goal_found_;
  int  start_idx_;
  int  goal_idx_;

  // final path
  std::vector<Point2D> path_;

  bool current_step_done_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PRMPublisher>();
  
  // This will keep calling timerCallback() at ~2 Hz 
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
