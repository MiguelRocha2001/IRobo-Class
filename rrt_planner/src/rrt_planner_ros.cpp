
#include <rrt_planner/rrt_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlannerROS, nav_core::BaseGlobalPlanner)

namespace rrt_planner {

    RRTPlannerROS::RRTPlannerROS(): 
        costmap_(NULL), planner_(), initialized_(false) {}

    RRTPlannerROS::RRTPlannerROS(std::string name, 
        costmap_2d::Costmap2DROS *costmap): costmap_(NULL), 
                              planner_(), initialized_(false) {
            
            initialize(name, costmap);
    }

    RRTPlannerROS::RRTPlannerROS(std::string name, 
        costmap_2d::Costmap2DROS *costmap, std::string global_frame): 
                      costmap_(NULL), planner_(), initialized_(false) {
            
            initialize(name, costmap, global_frame);
    }

    void RRTPlannerROS::initialize(std::string name, 
            costmap_2d::Costmap2DROS *costmap, std::string global_frame) {
        
        if(!initialized_) {
            
            costmap_ = costmap->getCostmap();
            global_frame_ = global_frame;

            ros::NodeHandle nh("~/" + name);         
            nh.param("goal_tolerance", params_.goal_tolerance, 0.15);
            nh.param("rrt/step", params_.step, 0.15);
            nh.param("rrt/min_num_nodes", params_.min_num_nodes, 5000);
            nh.param("rrt/max_num_nodes", params_.max_num_nodes, 30000);

            plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);

            planner_ = std::shared_ptr<RRTPlanner>(new RRTPlanner(costmap, params_));
            initialized_ = true;
        }
        else {

            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }    
    }
    
    void RRTPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap) {

        initialize(name, costmap, costmap->getGlobalFrameID());
    }

    bool RRTPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, 
                                    const geometry_msgs::PoseStamped& goal, 
                                        std::vector<geometry_msgs::PoseStamped>& plan) {

      if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
      }

      // clear the plan, just in case
      plan.clear();

      if(start.header.frame_id != global_frame_){
        ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
      }

      if(goal.header.frame_id != global_frame_){
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
      }

      // ---------------------- To Get The Costmap Configs -----------------------
      /*    
      // Get the size of the costmap in cells (width and height)
      unsigned int costmapWidth = costmap_->getSizeInCellsX();
      unsigned int costmapHeight = costmap_->getSizeInCellsY();

      // Get the resolution of the costmap (meters per cell)
      double costmapResolution = costmap_->getResolution();

      // Get the origin of the costmap (world coordinates of the cell (0,0))
      double costmapOriginX = costmap_->getOriginX();
      double costmapOriginY = costmap_->getOriginY();

      // Calculate the world coordinates of the boundaries
      double costmapMinX = costmapOriginX;
      double costmapMaxX = costmapOriginX + costmapWidth * costmapResolution;
      double costmapMinY = costmapOriginY;
      double costmapMaxY = costmapOriginY + costmapHeight * costmapResolution;

      // Print or log the boundaries
      ROS_INFO("Costmap Bounds: MinX=%.2lf, MaxX=%.2lf, MinY=%.2lf, MaxY=%.2lf",
         costmapMinX, costmapMaxX, costmapMinY, costmapMaxY);
      */

      // ---------------------- To Get The Costmap Configs -----------------------


      double world_start[2];
      world_start[0] = start.pose.position.x;
      world_start[1] = start.pose.position.y;

      //ROS_WARN_THROTTLE(1.0, "world_start[0]: %f", world_start[0]);
      //ROS_WARN_THROTTLE(1.0, "world_start[1]: %f", world_start[1]);

      unsigned int map_x, map_y;
      if(!costmap_->worldToMap(world_start[0], world_start[1], map_x, map_y)) {

        ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
      }

      double world_goal[2];
      world_goal[0] = goal.pose.position.x;
      world_goal[1] = goal.pose.position.y;

      //ROS_WARN_THROTTLE(1.0, "world_goal[0]: %f", world_goal[0]);
      //ROS_WARN_THROTTLE(1.0, "world_goal[1]: %f", world_goal[1]);

      if(!costmap_->worldToMap(world_goal[0], world_goal[1], map_x, map_y)) {

          ROS_WARN_THROTTLE(1.0, "The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
          return false;
      }

      //ROS_INFO("[RRTPlanner] Current Position: (%.2lf, %.2lf)", world_start[0], world_start[1]);
      //ROS_INFO("[RRTPlanner] GOAL Position: (%.2lf, %.2lf)", world_goal[0], world_goal[1]);

      planner_->setStart(world_start);
      planner_->setGoal(world_goal);

      /*
      ROS_INFO("costmap_2d::FREE_SPACE: %u", costmap_2d::FREE_SPACE);
      ROS_INFO("costmap_2d::LETHAL_OBSTACLE: %u", costmap_2d::LETHAL_OBSTACLE);
      ROS_INFO("costmap_2d::NO_INFORMATION: %u", costmap_2d::NO_INFORMATION);
      */
      
      planner_->setBestCost(computeDistance(world_start, world_goal));

      do {
        if(planner_->planPath() ) {
          ROS_INFO("[RRTPlanner] Found a path.");

          double* best_node_pos = planner_->getBestNodePos();
          double best_node_pos_x = best_node_pos[0];
          double best_node_pos_y = best_node_pos[1];
          printf("best_node_pos_x: %f\n", best_node_pos_x);
          printf("best_node_pos_y: %f\n", best_node_pos_y);
          int best_node_id = planner_->getBestNodeId();
          printf("best node id: %d\n", best_node_id);
          double best_node[2];
          //best_node[0] = rrt_tree_[best_node_id].pos[0];
          //best_node[1] = rrt_tree_[best_node_id].pos[1];
          //printf("best_node[0]: %f\n", best_node[0]);
          //printf("best_node[1]: %f\n", best_node[1]);
          printf("distance best pos to goal: %f\n", computeDistance(best_node_pos, world_goal));


          followPath(start, goal, plan, 1);

          //follow(start, goal, plan, positions);

          //planner_->restoreObstacleCost();

          return true;

        } else {
          ROS_WARN("[RRTPlanner] Failed to find a path.");

          if (planner_->newBestNodeFound() == 0)
          {
            printf("aaaa");
            printf("best_cost_: %f\n", planner_->getBestCost());
            planner_->setBestCost(10);
            //printf("distnace strat to goal: %f\n", computeDistance(world_start, world_goal));
            if (computeDistance(world_start, world_goal) < params_.goal_tolerance) {
              return true;
            }
            //planner_->increaseObstacleCost();
            //return false;
          }
        

          double* best_node_pos = planner_->getBestNodePos();
          int best_node_id = planner_->getBestNodeId();

          //printf("distance best pos to goal: %f\n", computeDistance(best_node_pos, world_goal));
          //printf("best node id: %d\n", planner_->getBestNodeId());
          
          //ROS_WARN("best node id outside path: %d", planner_->getBestNodeId());
          //ROS_WARN("best node pos outside path: (%f, %f)", best_node_pos[0], best_node_pos[1]);

          followPath(start, goal, plan, 0);

          planner_->setStart(best_node_pos);       

          //ROS_WARN("best_cost_ outside path: %f", planner_->getBestCost());

          //return false;
        }
      } while (true);
    
  }


  void RRTPlannerROS::followPath(const geometry_msgs::PoseStamped& start, 
                                    const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan,
                                    int x) {

    plan_time_ = ros::Time::now();

    rrt_tree_ = planner_->getTree();
    //current_id_ = rrt_tree_.size() - 1; // Add last vertex (closest to goal)
    double* best_node_pos = planner_->getBestNodePos();
    double best_node_pos_x = best_node_pos[0];
    double best_node_pos_y = best_node_pos[1];
    //printf("best_node_pos_x: %f\n", best_node_pos_x);
    //printf("best_node_pos_y: %f\n", best_node_pos_y);
    current_id_ = planner_->getBestNodeId(); // Add last vertex (closest to goal)
    //printf("current_id_: %d\n", current_id_);
    //ROS_INFO("[RRTPlanner] Best Node ID: %d", current_id_);
    double best_node[2];
    best_node[0] = rrt_tree_[current_id_].pos[0];
    best_node[1] = rrt_tree_[current_id_].pos[1];
    //printf("best_node[0]: %f\n", best_node[0]);
    //printf("best_node[1]: %f\n", best_node[1]);

    pose_stamped_.header.stamp = plan_time_;
    pose_stamped_.header.frame_id = global_frame_;

    pose_stamped_.pose.orientation = goal.pose.orientation; // Set goal orientation for last node
    
    double world_start[2];
    world_start[0] = start.pose.position.x;
    world_start[1] = start.pose.position.y;

    double world_goal[2];
    world_goal[0] = goal.pose.position.x;
    world_goal[1] = goal.pose.position.y;

    std::vector<geometry_msgs::PoseStamped> temp_plan;

    //printf("distance from best goal node: %f\n", computeDistance(best_node_pos, world_goal));
    /*
    printf("BEGGINING");
    for (int i = 0; i < plan.size(); ++i) {
      printf("plan[%d]: (%f, %f)\n", i, plan[i].pose.position.x, plan[i].pose.position.y);
    }*/

    // Work our way back to start waypoint, building plan
    while (current_id_ != 0) {
      /* 
      if (computeDistance(rrt_tree_[current_id_].pos, world_start) < computeDistance(rrt_tree_[prev_id_].pos, world_start)) {
        prev_id_ = current_id_; // Identify next vertex in path (parent node), store previous ID
        current_id_ = rrt_tree_[current_id_].parent_id;
        continue;
      }
      */
      // Retrieve pose of current ID
      pose_stamped_.pose.position.x = rrt_tree_[current_id_].pos[0];
      pose_stamped_.pose.position.y = rrt_tree_[current_id_].pos[1];
      pose_stamped_.pose.position.z = 0.;
      //plan.push_back(pose_stamped_);           // Add pose to plan
      temp_plan.push_back(pose_stamped_);      // Add pose to temp plan

      prev_id_ = current_id_; // Identify next vertex in path (parent node), store previous ID
      current_id_ = rrt_tree_[current_id_].parent_id;

      // Set orientation for next iteration
      dy_ = rrt_tree_[prev_id_].pos[1] - rrt_tree_[current_id_].pos[1];
      dx_ = rrt_tree_[prev_id_].pos[0] - rrt_tree_[current_id_].pos[0];
      yaw_ = atan2(dy_, dx_);  // Get yaw from atan2 using current point and previous point
      quat_tf_.setRPY(0., 0., yaw_); // Convert RPY to quat
      quat_msg_ = tf2::toMsg(quat_tf_); // Convert Quat TF to msg
      pose_stamped_.pose.orientation = quat_msg_; // set orientation

    }

    if (plan.size() == 1) {
      /* */
    }

    // Add start waypoint
    pose_stamped_.pose.position.x = rrt_tree_[0].pos[0];
    pose_stamped_.pose.position.y = rrt_tree_[0].pos[1];
    pose_stamped_.pose.position.z = 0.;
    pose_stamped_.pose.orientation = start.pose.orientation;
    //plan.push_back(pose_stamped_);
    temp_plan.push_back(pose_stamped_);      // Add pose to temp plan

    std::reverse(temp_plan.begin(), temp_plan.end());
    
    for (int i = 0; i < temp_plan.size(); ++i) {
      //printf("temp_plan[%d]: (%f, %f)\n", i, temp_plan[i].pose.position.x, temp_plan[i].pose.position.y);
    }

    //printf("\n");

    plan.insert(plan.end(), temp_plan.begin(), temp_plan.end());
    // Reverse order of plan
    //std::reverse(plan.begin(), plan.end());

    if (x == 1) {
      std::vector<int> revised_plan;
      std::vector<geometry_msgs::PoseStamped> clean_path;

      revised_plan = planner_->trimPath(plan);
      double position[2];
      

      for (int i = 0; i < plan.size(); ++i) {
        position[0] = plan[i].pose.position.x;
        position[1] = plan[i].pose.position.y;
        ROS_INFO("plan[%d]: (%f, %f)", i, plan[i].pose.position.x, plan[i].pose.position.y);
        ROS_INFO("distance: %f", computeDistance(position, world_goal));
      }

    
      for(size_t i = 0; i < plan.size(); ++i) {
        // Use std::find to check if i is an exclusion index
        if(std::find(revised_plan.begin(), revised_plan.end(), i) == revised_plan.end()) {
            // i is not an exclusion index, so include my_vector[i] in result_vector
            clean_path.push_back(plan[i]);
        }
      }
      

      publishPlan(clean_path);
    }


    /*
    if (x == 1) {
      
      std::vector<geometry_msgs::PoseStamped> revised_plan;
      //revised_plan.push_back(start);
      revised_plan.insert(revised_plan.begin(), start);
      double distance_start_to_goal = computeDistance(world_start, world_goal);
      double position[2];

      double closestDistance = distance_start_to_goal;
      for (int i = 0; i < plan.size(); ++i) {
        position[0] = plan[i].pose.position.x;
        position[1] = plan[i].pose.position.y;
        double distance = computeDistance(position, world_goal);
        //printf("Position %d: (%f, %f)\n", i, position[0], position[1]);
        //printf("distance: %f\n", distance);
        //printf("distance_start_to_goal: %f\n", distance_start_to_goal);
        //printf("closesDistance: %f\n", closestDistance);
        if (distance >= distance_start_to_goal) {
          continue;
        } 
        if (distance >= closestDistance) {
          if (i != 0) {
            revised_plan[revised_plan.size()-1].pose.orientation = plan[i].pose.orientation;
          }
          continue;
        }
        closestDistance = distance;
        revised_plan.push_back(plan[i]);
      }

      
      for (int i = 0; i < revised_plan.size(); ++i) {
        position[0] = revised_plan[i].pose.position.x;
        position[1] = revised_plan[i].pose.position.y;
        ROS_INFO("revised_plan[%d]: (%f, %f)", i, revised_plan[i].pose.position.x, revised_plan[i].pose.position.y);
        ROS_INFO("distance: %f", computeDistance(position, world_goal));
      }
      
      

      publishPlan(revised_plan);
    }  
    */
    
  
    /* 
    for (int i = 0; i < plan.size(); ++i) {
      printf("plan[%d]: (%f, %f)\n", i, plan[i].pose.position.x, plan[i].pose.position.y);
    }
    */
  }

  void RRTPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {

    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(path.empty()) {
      //still set a valid frame so visualization won't hit transform issues
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } else { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    
    //for (int i = 0; i < gui_path.poses.size(); ++i) {
      //ROS_INFO("gui_path[%d]: (%f, %f)", i, gui_path.poses[i].pose.position.x, gui_path.poses[i].pose.position.y);
    //}

    plan_pub_.publish(gui_path);
  }

};   