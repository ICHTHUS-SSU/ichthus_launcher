/*
 * Copyright 2019 Hyoeun Lee, Chaewon Hong, Youngjoon Choi, Kanghee Kim. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ICHTHUS_LAUNCHER_H
#define ICHTHUS_LAUNCHER_H

///// ros related headers
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

///// autoware related headers
#include "autoware_config_msgs/ConfigWaypointFollower.h"

///// boost related headers
#include <regex>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list_io.hpp>
#include <boost/graph/iteration_macros.hpp>

namespace ichthus_launcher
{
  /////////////////////////////////////////////////////
  // Road graph related data structures
  /////////////////////////////////////////////////////
  struct POI 
  {
    POI() {}
    std::string name;
  };

  struct Road 
  {
    Road() {}
    std::string name;
    float speed_limit;
    float lookahead_ratio;
    std::vector<std::vector<double> > points;
  };
  
  typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, POI, Road > RoadMap;
  typedef typename boost::graph_traits<RoadMap>::vertex_iterator   r_vrtx_iter;
  typedef typename boost::graph_traits<RoadMap>::vertex_descriptor r_vrtx_desc;
  typedef typename boost::graph_traits<RoadMap>::edge_descriptor   r_edge_desc;

  /////////////////////////////////////////////////////
  // Task graph related data structures
  /////////////////////////////////////////////////////
  struct Task 
  {
    Task() {}
    int stage;
    bool persist;
    bool alive;
    std::string name;
    std::string real_name;
    std::string topic;
    int affinity;
    int priority;
    int rate;
    std::string command;
  };

  struct Channel 
  {
    Channel() {}
    int weight;
    std::string name;
    std::string topic;
  };
  
  typedef boost::adjacency_list< boost::listS, boost::vecS, boost::directedS, Task, Channel > TaskMap;
  typedef typename boost::graph_traits<TaskMap>::vertex_iterator   t_vrtx_iter;
  typedef typename boost::graph_traits<TaskMap>::vertex_descriptor t_vrtx_desc;
  typedef typename boost::graph_traits<TaskMap>::edge_descriptor   t_edge_desc;
  
  /////////////////////////////////////////////////////
  // Main class definition
  /////////////////////////////////////////////////////
  class IchthusLauncher
  {
  private:
    ///// ros related variables
    ros::NodeHandle nh_;
    ros::Publisher pub_speed_limit_;
    ros::Publisher pub_config_wf_;
    ros::Subscriber sub_curr_pose_;

    ///// Road graph related variables and functions
    RoadMap *prmap_;
    bool curr_road_valid_;
    bool curr_road_changed_;
    r_edge_desc curr_road_desc_;

    void load_road_graph();
    void callback_get_curr_pose(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    
    void traverse_road_vertices();
    bool find_first_road_with_pose(double cx, double cy);
    bool find_next_road_with_pose(double cx, double cy);

    ///// Task graph related variables and functions
    TaskMap *ptmap_;
    int num_tasks_;
    int task_poll_interval_; // millisecond
    int stage_to_rerun_;

    void load_task_graph();
    void callback_get_status(const std_msgs::BoolConstPtr& msg, t_vrtx_desc v);
    void launch_stage(t_vrtx_iter vi0, t_vrtx_iter vi9);
    void relaunch_task_graph(int stage0);
    void shutdown_task_graph(int stage0);
    void traverse_task_vertices();
    t_vrtx_desc find_task_by_real_name(std::string name);
    
    bool find_pid(const std::string& task_name, std::string& pid);
    
  public:
    IchthusLauncher();
    ~IchthusLauncher();
    void doLoop();
  };
}

#endif // ICHTHUS_LAUNCHER_H
