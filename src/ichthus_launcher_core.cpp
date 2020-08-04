/*
 * Copyright 2019 Hyoeun Lee, Chaewon Hong, Youngjoon Choi, Kanghee Kim at Soongsil University. All rights reserved.
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

#include "ichthus_launcher/ichthus_launcher.h"
#include "ichthus_launcher/pipe_open.h"

#define MAX_POINTS_IN_POLYGON  10

namespace ichthus_launcher
{
  /////////////////////////////////////////////////////
  // Road graph related functions
  /////////////////////////////////////////////////////  

  std::ostream& operator<<(std::ostream& out, const POI& p)
  {
    out << p.name << ' '; 
    out << -1;
    return out;
  }

  bool operator==(const POI& p1, const POI& p2)
  {
    return (p1.name == p2.name);
  }

  std::ostream& operator<<(std::ostream& out, const Road& road)
  {
    out << road.name << ' ' << road.speed_limit << ' ' << road.lookahead_ratio << ' ';
    for (int i = 0; i < road.points.size(); i++) 
    {
      std::cout << "[ ";
      copy(road.points[i].begin(), road.points[i].end(), std::ostream_iterator<double>(out, " "));
      std::cout << "] ";
    }
    out << -1;
    return out;
  }

  bool operator==(const Road& c1, const Road& c2)
  {
    return (c1.name == c2.name);
  }

  bool in_polygon(double cx, double cy, struct Road& road)
  {
    int i, j;
    bool c = false;
    double road_x[MAX_POINTS_IN_POLYGON];
    double road_y[MAX_POINTS_IN_POLYGON];

    if (road.points.size() > MAX_POINTS_IN_POLYGON) 
    {
      std::cerr << "road.points.size() exceeded " << MAX_POINTS_IN_POLYGON << std::endl;
      exit(1);
    }
    
    for (int i = 0; i < road.points.size(); i++) 
    {
      road_x[i] = road.points[i][0];
      road_y[i] = road.points[i][1];
    }
    
    for(i=0, j=road.points.size()-1; i < road.points.size(); j = i++) 
    {
      if( ((road_y[i] > cy) != (road_y[j] > cy)) &&
	        (cx < (road_x[j]-road_x[i])*(cy-road_y[i])/(road_y[j]-road_y[i])+road_x[i]) ) 
      {
	      c = !c;
      }
    }

    return c;
  }

  void IchthusLauncher::traverse_road_vertices()
  {
    BGL_FORALL_VERTICES_T(v, *prmap_, RoadMap) 
    {
      std::cout << "vertex" << v << ": " << std::endl;
      BGL_FORALL_OUTEDGES_T(v, e, *prmap_, RoadMap) 
      {
	      std::cout << "edge" << e << ": " << (*prmap_)[e] << std::endl;
      }
      std::cout << std::endl;
    }
  }

  bool IchthusLauncher::find_first_road_with_pose(double cx, double cy)
  {
    BGL_FORALL_VERTICES_T(v, *prmap_, RoadMap) 
    {
      BGL_FORALL_OUTEDGES_T(v, e, *prmap_, RoadMap) 
      {
        if (in_polygon(cx, cy, (*prmap_)[e])) 
        { // found!!
        // std::cout << "### current road: " << (*prmap_)[e].name << "   " << std::endl;
          curr_road_desc_ = e;
          curr_road_changed_ = true;
          return true;
        }
      }
    }
    
    // std::cout << "### find_first_road_with_pose: no matching road" << std::endl;
    return false;
  }

  bool IchthusLauncher::find_next_road_with_pose(double cx, double cy)
  {
    r_vrtx_desc v = target(curr_road_desc_, *prmap_);
    
    BGL_FORALL_OUTEDGES_T(v, e, *prmap_, RoadMap) 
    {
      if (in_polygon(cx, cy, (*prmap_)[e])) 
      { // found!!
//	std::cout << "### current road: " << (*prmap_)[e].name << std::endl;
        curr_road_desc_ = e;
        curr_road_changed_ = true;
        return true;
      }
    }
    
    if (in_polygon(cx, cy, (*prmap_)[curr_road_desc_])) 
    { 
 //     std::cout << "### current road: " << (*prmap_)[curr_road_desc_].name << std::endl;
      return true;
    }
    
    std::cout << "### find_next_road_with_pose: no matching road" << std::endl;
    return false;
  }
  
  void IchthusLauncher::callback_get_curr_pose(const geometry_msgs::PoseStampedConstPtr& pose_msg)
  {
    if (!curr_road_valid_)
      curr_road_valid_ = find_first_road_with_pose(pose_msg->pose.position.x, pose_msg->pose.position.y);
    else
      curr_road_valid_ = find_next_road_with_pose(pose_msg->pose.position.x, pose_msg->pose.position.y);

#if 1
    if (curr_road_changed_) 
    {
      curr_road_changed_ = false;
      std_msgs::Float32 msg_speed_limit;
      autoware_config_msgs::ConfigWaypointFollower msg_config_wf;

      msg_speed_limit.data = (*prmap_)[curr_road_desc_].speed_limit;
      pub_speed_limit_.publish(msg_speed_limit);

      msg_config_wf.param_flag = 0;
      msg_config_wf.lookahead_ratio = (*prmap_)[curr_road_desc_].lookahead_ratio;
      msg_config_wf.minimum_lookahead_distance = 6.0;
      pub_config_wf_.publish(msg_config_wf);
    }
#endif
  }

  void IchthusLauncher::load_road_graph() 
  {
    int num_pois;
    int num_vtx = 0;
    int num_edges = 0;
    
    if (!nh_.getParam("num_pois", num_pois)) 
    {
      ROS_ERROR("rosparam num_pois should be defined!");
      return;
    }

    prmap_ = new RoadMap(num_pois);
    r_vrtx_iter vi = vertices(*prmap_).first;
    r_vrtx_desc v = *vi++;
    std::string s;
    
    while (1) 
    { 
      std::string param = "poi" + std::to_string(num_vtx) + "/";
      if (!nh_.getParam(param + "name", s))
        break;
      (*prmap_)[v].name = s;
      v = *vi++;
      num_vtx++;
    }
    //std::cout << write(*prmap_);
    
    int ep1, ep2;
    r_edge_desc e;
    std::vector<double> vec;
    while (1) 
    { 
      std::string param = "road" + std::to_string(num_edges) + "/";
      if (!nh_.getParam(param + "ep1", ep1) || !nh_.getParam(param + "ep2", ep2))
	      break;

      // ep1 and ep2 are type-cast into vertex_descriptor
      e = add_edge(ep1, ep2, *prmap_).first;
      
      nh_.getParam(param + "name", (*prmap_)[e].name);
      nh_.getParam(param + "speed_limit", (*prmap_)[e].speed_limit);
      nh_.getParam(param + "lookahead_ratio", (*prmap_)[e].lookahead_ratio);

      for (int num_pts=0; num_pts < MAX_POINTS_IN_POLYGON; num_pts++) 
      {
	      std::string subparam = param + "p" + std::to_string(num_pts);
	      if (!nh_.getParam(subparam, vec))
	        break;
	      (*prmap_)[e].points.push_back(vec);
      }

      num_edges++;
    }

    std::cout << write(*prmap_);
  }
  
  /////////////////////////////////////////////////////
  // Task graph related functions
  /////////////////////////////////////////////////////  

  std::ostream& operator<<(std::ostream& out, const Task& task)
  {
    out << task.stage << ' '
        << task.persist << ' '
        << task.alive << ' '
        << task.name << ' '
        << task.real_name << ' '
        << task.topic << ' '
        << task.command << ' ';
    out << -1;
    return out;
  }

  bool operator==(const Task& t1, const Task& t2)
  {
    return (t1.real_name == t2.real_name);
  }

  std::ostream& operator<<(std::ostream& out, const Channel& chan)
  {
    out << chan.weight << ' '
        << chan.name << ' '
        << chan.topic << ' ';
    out << -1;
    return out;
  }

  bool operator==(const Channel& c1, const Channel& c2)
  {
    return (c1.name == c2.name);
  }
  
  void IchthusLauncher::load_task_graph()
  {
    //int num_tasks_;
    int num_vtx = 0;
    int num_edges = 0;

    if (!nh_.getParam("num_tasks", num_tasks_)) 
    {
      ROS_ERROR("rosparam num_tasks should be defined!");
      return;
    }

    if (!nh_.getParam("task_poll_interval", task_poll_interval_)) 
    {
      ROS_ERROR("rosparam task_poll_interval should be defined!");
      return;
    }
    task_poll_interval_ *= 1000;

    if (!nh_.getParam("stage_to_rerun", stage_to_rerun_)) 
    {
      ROS_ERROR("rosparam stage_to_rerun should be defined!");
      return;
    }

    ptmap_ = new TaskMap(num_tasks_);
    t_vrtx_iter vi = vertices(*ptmap_).first;
    t_vrtx_desc v = *vi++;
    std::string s;
    
    while (1) 
    {
      std::string param = "task" + std::to_string(num_vtx) + "/";
      if (!nh_.getParam(param + "name", s))
	      break;
      (*ptmap_)[v].name = s.c_str();
      (*ptmap_)[v].real_name = s.c_str();
      (*ptmap_)[v].alive = false;
      nh_.getParam(param + "stage",   (*ptmap_)[v].stage);
      nh_.getParam(param + "persist", (*ptmap_)[v].persist);
      nh_.getParam(param + "topic",   (*ptmap_)[v].topic);
      nh_.getParam(param + "affinity",(*ptmap_)[v].affinity);
      nh_.getParam(param + "priority",(*ptmap_)[v].priority);
      nh_.getParam(param + "rate",    (*ptmap_)[v].rate);
      nh_.getParam(param + "launch",  (*ptmap_)[v].command);
      v = *vi++;
      num_vtx++;
    }

    int ptask, stask;
    t_vrtx_desc ep1, ep2;
    t_edge_desc e;
    while (1) 
    { 
      std::string param = "chan" + std::to_string(num_edges) + "/";
      if (!nh_.getParam(param + "ptask", ptask) || !nh_.getParam(param + "stask", stask))
	      break;

      // ptask and stask are type-cast into vertex_descriptor
      ep1 = ptask;
      ep2 = stask;
      e = add_edge(ep1, ep2, *ptmap_).first;

      (*ptmap_)[e].name = (*ptmap_)[ep1].name + "->" + (*ptmap_)[ep2].name;
      nh_.getParam(param + "topic", (*ptmap_)[e].topic);
      num_edges++;
    }

    std::cout << write(*ptmap_);
  }

  void IchthusLauncher::traverse_task_vertices()
  {
    BGL_FORALL_VERTICES_T(v, *ptmap_, TaskMap) 
    {
      std::cout << "vertex" << v << ": " << std::endl;
      BGL_FORALL_OUTEDGES_T(v, e, *ptmap_, TaskMap) 
      {
	      std::cout << "edge" << e << ": " << (*ptmap_)[e] << std::endl;
      }
      std::cout << std::endl;
    }
  }

  t_vrtx_desc IchthusLauncher::find_task_by_real_name(std::string name)
  {
    BGL_FORALL_VERTICES_T(v, *ptmap_, TaskMap) 
    {
      if ((*ptmap_)[v].real_name == name)
	      return v;
    }

    return -1;
  }

  void pipe_exec(std::string& cmd, std::stringstream& log)
  {
    pipe_open::PipeOpenRead pipe;
    pipe.open(cmd);
    pipe.read(log);
    pipe.close();
    //std::cout << log.str() << std::endl;
  }

  bool search_word(std::string& lines, std::string& prefix, std::string& word)
  {
    std::smatch match;
    std::regex expr("\\b(" + prefix + ")([_0-9]*)");

    // search for any word beginning with prefix
    if (std::regex_search(lines, match, expr)) {
      word = match[0].str();
      word.erase(remove(word.begin(), word.end(), '\n'), word.end());
      //std::cout << "^^^" << word << "$$$" << std::endl;
      lines = match.suffix().str();
      return true;
    }

    return false;
  }

  void split(const std::string& input, char delimiter, std::vector<std::string>& tokens)
  {
    std::stringstream ss(input);
    std::string temp;

    while (std::getline(ss, temp, delimiter))
    {
      tokens.push_back(temp);
    }
  }

  void IchthusLauncher::launch_stage(t_vrtx_iter vi0, t_vrtx_iter vi9)
  {
    if (vi0 == vi9) return;

    int stage = (*ptmap_)[*vi0].stage;
    int num_persisting = 0;
    t_vrtx_iter vi = vi0;

    ////////// step1: launch all the tasks of current stage.
    std::cout << "--------------------------------" << std::endl;
    std::cout << "### launching tasks [" << *vi0 << ".." << *vi9 << ")" << std::endl;
    while (vi != vi9) 
    {
      //std::cout << (*ptmap_)[*vi] << std::endl;
      if ((*ptmap_)[*vi].command.length() == 0) 
      {
        std::cout << "### launching stage " << stage << ": null command" << std::endl;
        (*ptmap_)[*vi].alive = true; // assume any task with null command is alive!
      }
      else 
      {
	      std::cout << "### launching stage " << stage << ": " << (*ptmap_)[*vi].command << std::endl;
	      if (std::system((*ptmap_)[*vi].command.c_str()) < 0)
        {
          ROS_ERROR("std::system(\"%s\")", (*ptmap_)[*vi].command.c_str());
          exit(1);
        }
        if ((*ptmap_)[*vi].persist)
          num_persisting++;
        else
          (*ptmap_)[*vi].alive = true; // assume any non-persisting task is alive!
      }
      ++vi;
    }

    int num_confirmed = 0;
    vi = vi0;

    ////////// step2: wait until the tasks are all launched.
    std::string cmd = "rosnode list &";
    while (num_confirmed != num_persisting) 
    {
      bool confirmed = false;
      std::stringstream log;
      pipe_exec(cmd, log);
      std::string lines = log.str();
      std::string full_name;

      for (vi = vi0; vi != vi9; ++vi) 
      {
	      if ((*ptmap_)[*vi].alive) continue;
	
        while (search_word(lines, (*ptmap_)[*vi].real_name, full_name)) 
        {
          t_vrtx_desc v = find_task_by_real_name(full_name);

          if (v == -1)
            // a task appears whose name includes a prefix of (*ptmap_)[*vi].name
            (*ptmap_)[*vi].real_name = full_name.c_str();

          if ((*ptmap_)[*vi].real_name == full_name) 
          {
            // a task appears whose name exactly matches 'full_name'
            confirmed = true;
            num_confirmed++;
            std::cout << "### task of stage " << stage << " launched ["
                  << num_confirmed << "/" << num_persisting << "]: "
                  << full_name << std::endl;
            (*ptmap_)[*vi].alive = true;
            break;
          }
        }

        if (confirmed == false)
          break; // go to usleep
      }

      usleep(task_poll_interval_); // sleep for a while
    }

    ////////// step3: wait until some critical task of this stage publishes to /*_stat (e.g. /pmap_stat, /vmap_stat, /ndt_stat, /gplan_stat, /lplan_stat)
    int num_subs = 0;
    ros::Subscriber sub_list[num_persisting];

    for (vi = vi0; vi != vi9; ++vi) 
    {
      if ((*ptmap_)[*vi].topic == "") continue;

      (*ptmap_)[*vi].alive = false; // turn off the flag for reuse
      sub_list[num_subs++]= nh_.subscribe<std_msgs::Bool>((*ptmap_)[*vi].topic, 10, bind(&IchthusLauncher::callback_get_status, this, _1, *vi));
    }

    for (vi = vi0; vi != vi9; ++vi) 
    {
      t_vrtx_iter vi2;
      //for (vi2 = vi0; vi2 != vi9; ++vi2) std::cout << "$$$ " << (*ptmap_)[*vi2] << std::endl;
      while ((*ptmap_)[*vi].alive == false) 
      {
        std::cout << "### waiting to receive a stat msg from " << (*ptmap_)[*vi].real_name << "..." << std::endl;
        ros::spinOnce();
        usleep(task_poll_interval_); // sleep for a while
      }
    }

    for (int i = 0; i < num_subs; i++)
    {
      sub_list[i].shutdown();
    }
  }
  
  void IchthusLauncher::relaunch_task_graph(int stage0)
  {
    int stage = stage0;
    int ntasks = 0;
    t_vrtx_iter vi0 = vertices(*ptmap_).first;
    t_vrtx_iter vi9 = vertices(*ptmap_).first;

    BGL_FORALL_VERTICES_T(v, *ptmap_, TaskMap) 
    {
      if ((*ptmap_)[v].stage < stage) 
      { // skip all the tasks of stage < stage0
        ++vi0;
        ++vi9;
      }
      else if ((*ptmap_)[v].stage == stage) // identify all tasks of stage
	      ++vi9;
      else if ((*ptmap_)[v].stage == stage+1) 
      {
        launch_stage(vi0, vi9);
        vi0 = vi9;
        ++vi9;
        stage++;
      }
      // Bug fixed by khkim on 20191226:
      // This for-loop was supposed to exit when all vertices are visited, but it didn't.
      // So, the following code is needed! 
      if (++ntasks == num_tasks_) 
	      break;
    }

    // launch the final stage that has not been triggered above
    launch_stage(vi0, vi9);
  }

  void IchthusLauncher::shutdown_task_graph(int stage0)
  {
    std::string cmd;
    curr_road_valid_ = false; // disable current road tracking!!

    BGL_FORALL_VERTICES_T(v, *ptmap_, TaskMap) 
    {
      if ((*ptmap_)[v].stage < stage0) 
	      continue; // skip all the tasks of stage < stage0

      if ((*ptmap_)[v].persist) 
      { // if ((*ptmap_)[v].stage >= stage0) 
        std::cout << "### killing node of stage " << (*ptmap_)[v].stage << ": " << (*ptmap_)[v].real_name << std::endl;
        cmd = "rosnode list | grep " + (*ptmap_)[v].real_name + " | xargs rosnode kill";
        if (std::system(cmd.c_str()) < 0)
        {
          ROS_ERROR("std::system(\"%s\")", cmd.c_str());
          exit(1);
        }
      }
      (*ptmap_)[v].alive = false;
      (*ptmap_)[v].real_name = (*ptmap_)[v].name.c_str();
    }

    if (std::system("echo 'y' > yes; rosnode cleanup < yes") < 0)
    {
      ROS_ERROR("std::system(\"echo 'y' > yes; rosnode cleanup < yes\")");
      exit(1);
    }
  }

  void IchthusLauncher::callback_get_status(const std_msgs::BoolConstPtr& msg, t_vrtx_desc v)
  {
    (*ptmap_)[v].alive = msg->data;
    std::cout << "### received a stat msg(" << (*ptmap_)[v].alive << ") from " << (*ptmap_)[v].real_name << "!!!" << std::endl;
  }

  bool IchthusLauncher::find_pid(const std::string& task_name, std::string& pid)
  {
    std::string cmd = "rosnode info " + task_name + " | grep Pid | awk -F' ' '{print $2}' &";
    std::stringstream log;

    pipe_exec(cmd, log);
    log >> pid;
    if (pid.empty())
      return false;

    return true;
  }

  /////////////////////////////////////////////////////
  // Public functions
  /////////////////////////////////////////////////////  

  void IchthusLauncher::doLoop()
  {
    load_road_graph();
    //traverse_road_vertices(); // debugging purposes
    load_task_graph();
    
    //traverse_task_vertices(); // debugging purposes
    relaunch_task_graph(0);

    ros::spin();
  }

  IchthusLauncher::IchthusLauncher() 
  : prmap_(NULL)
  , curr_road_valid_(false)
  , curr_road_changed_(false)
  , ptmap_(NULL)
  , task_poll_interval_(100)
  , stage_to_rerun_(0)
  {
    pub_speed_limit_ = nh_.advertise<std_msgs::Float32>("/speed_limit", 10);
    pub_config_wf_ = nh_.advertise<autoware_config_msgs::ConfigWaypointFollower>("/config/waypoint_follower", 10);
    sub_curr_pose_ = nh_.subscribe("/current_pose", 10, &IchthusLauncher::callback_get_curr_pose, this);
  }
  IchthusLauncher::~IchthusLauncher() {}
}

