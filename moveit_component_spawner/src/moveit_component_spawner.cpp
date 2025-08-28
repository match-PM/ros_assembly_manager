#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <tf2_ros/transform_listener.h>
#include "assembly_manager_interfaces/msg/object_scene.hpp"
#include "assembly_manager_interfaces/msg/object.hpp"
#include "assembly_manager_interfaces/srv/spawn_object.hpp"
#include "assembly_manager_interfaces/srv/destroy_object.hpp"
#include "assembly_manager_interfaces/srv/set_collision_checking.hpp"
#include <iostream>
#include <fstream>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Make sure this is included!

// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
//  #include "spawn_object_interfaces/srv/spawn_object.hpp"
//  #include "spawn_object_interfaces/srv/destroy_object.hpp"
//  #include "spawn_object_interfaces/srv/disable_obj_collision.hpp"

using std::placeholders::_1;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;
std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

struct CollisionRule
{
  std::string link_1;
  std::string link_2;
  bool should_check;
};

class MoveitObjectSpawnerNode : public rclcpp::Node
{
public:
  std::vector<std::string> component_names_list;
  std::vector<std::string> component_stl_paths_list;
  std::vector<std::string> component_parents_list;
  std::vector<geometry_msgs::msg::Pose> component_pose_list;
  rclcpp::Service<assembly_manager_interfaces::srv::SpawnObject>::SharedPtr spawn_object_service;
  rclcpp::Service<assembly_manager_interfaces::srv::DestroyObject>::SharedPtr destroy_object_service;
  rclcpp::Service<assembly_manager_interfaces::srv::SetCollisionChecking>::SharedPtr disable_collision_service;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_subscriber;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<assembly_manager_interfaces::msg::ObjectScene>::SharedPtr assembly_scene_subscriber;
  std::string robot_description_string;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::vector<assembly_manager_interfaces::msg::Object> object_list;
  tf2_msgs::msg::TFMessage::SharedPtr current_tf_msg;
  std::vector<std::string> disabled_collision_list;
  std::vector<CollisionRule> saved_collision_rules;

  MoveitObjectSpawnerNode() : Node("my_node")
  {
    RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");

    auto tf_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = tf_callback_group;

    spawn_object_service = this->create_service<assembly_manager_interfaces::srv::SpawnObject>("moveit_component_spawner/spawn_object", std::bind(&MoveitObjectSpawnerNode::spawn_object, this, std::placeholders::_1, std::placeholders::_2));
    destroy_object_service = this->create_service<assembly_manager_interfaces::srv::DestroyObject>("moveit_component_spawner/destroy_object", std::bind(&MoveitObjectSpawnerNode::destroy_object, this, std::placeholders::_1, std::placeholders::_2));
    disable_collision_service = this->create_service<assembly_manager_interfaces::srv::SetCollisionChecking>("moveit_component_spawner/set_collision_checking", std::bind(&MoveitObjectSpawnerNode::disable_collision, this, std::placeholders::_1, std::placeholders::_2));

    // tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1),options);
    planning_scene_diff_publisher = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

    // robot_description_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    //   "robot_description", 10, std::bind(&MoveitObjectSpawnerNode::robot_description_callback, this, _1));

    assembly_scene_subscriber = this->create_subscription<assembly_manager_interfaces::msg::ObjectScene>("/assembly_manager/scene", 10, std::bind(&MoveitObjectSpawnerNode::object_scene_callback, this, _1));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MoveitObjectSpawnerNode::check_node_presence, this));

    while (planning_scene_diff_publisher->get_subscription_count() < 1)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pm_robot_present = true;
  }

  void test()
  {
    // // collision_detection::AllowedCollisionMatrix& acm = planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrixNonConst();
    // // RCLCPP_ERROR(this->get_logger(), "CHECK1");

    // // acm.setEntry("*", "*", true);  // Allow all collisions
    // // planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    // // const auto& acm = scene->getAllowedCollisionMatrix();

    // // scene.getAllowedCollisionMatrixNonConst().setEntry("Calibration_Qube", "PM_Robot_Vacuum_Tool", true);

    // // // Publish the changes
    // // moveit_msgs::msg::PlanningScene ps_msg;
    // // RCLCPP_ERROR(this->get_logger(), "CHECK2");
    // // planning_scene_monitor_->getPlanningScene()->getPlanningSceneMsg(ps_msg);
    // // ps_msg.is_diff = true;
    // // RCLCPP_ERROR(this->get_logger(), "CHECK3");
    // // planning_scene_diff_publisher->publish(ps_msg);

    //   auto planning_scene = planning_scene_monitor_->getPlanningScene();
    // collision_detection::AllowedCollisionMatrix& acm =
    //     planning_scene->getAllowedCollisionMatrixNonConst();

    // RCLCPP_ERROR(this->get_logger(), "CHECK1");

    // // OR just disable specific ones:
    // acm.setEntry("Calibration_Qube", "PM_Robot_Vacuum_Tool", false);
    // acm.setEntry("Calibration_Qube", "PM_Robot_Vacuum_Tool_Tip", true);

    // // Publish the changes to MoveGroup
    // moveit_msgs::msg::PlanningScene ps_msg;
    // planning_scene->getPlanningSceneMsg(ps_msg);
    // ps_msg.is_diff = true;

    // RCLCPP_ERROR(this->get_logger(), "CHECK2");

    // planning_scene_diff_publisher->publish(ps_msg);

    // RCLCPP_ERROR(this->get_logger(), "CHECK3");
  }

private:
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
  std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
  moveit_msgs::msg::PlanningScene planning_scene;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pm_robot_present;

  bool clear_scene()
  {
    // std::vector<std::string> object_names = getSceneObjects();

    try
    {
      planning_scene.robot_state.attached_collision_objects.clear();
      planning_scene.world.collision_objects.clear();
      planning_scene.robot_state.is_diff = true;
      planning_scene.is_diff = true;

      for (size_t i = 0; i < object_list.size(); i++)
      {
        std::string object_id = object_list[i].obj_name;

        // Detach the object if attached
        moveit_msgs::msg::AttachedCollisionObject detach_object;
        detach_object.object.id = object_id;
        detach_object.object.operation = detach_object.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.push_back(detach_object);

        // Remove the object from the world
        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.id = object_id;
        remove_object.operation = remove_object.REMOVE;
        planning_scene.world.collision_objects.push_back(remove_object);
        // Verify if the object is indeed removed
      }
      planning_scene_diff_publisher->publish(planning_scene);
      RCLCPP_ERROR(this->get_logger(), "DEBUG - Publishing object Destructions");

      return true;
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during object removal: %s", ex.what());
      return false;
    }
  }

  void object_scene_callback(const assembly_manager_interfaces::msg::ObjectScene::SharedPtr msg)
  {
    // Check if the scene has changed
    if (check_scene_has_changed(msg->objects_in_scene))
    {
      RCLCPP_WARN(this->get_logger(), "Scene has changed!!!");
      clear_scene();
      object_list = msg->objects_in_scene;
      // Update the scene
      apply_objects_to_moveit();
    }
    else
    {
      // RCLCPP_INFO(this->get_logger(), "Scene has not changed");
    }

    // object_list = msg->objects_in_scene;
  }

  bool is_object_in_scene(const std::string &object_name)
  {
    for (const auto &object : object_list)
    {
      if (object.obj_name == object_name)
      {
        return true;
      }
    }
    return false;
  }

  bool check_scene_has_changed(const std::vector<assembly_manager_interfaces::msg::Object> msg_object_list)
  {
    // compare object_list with msg_object_list

    if (object_list.size() != msg_object_list.size())
    {
      return true;
    }
    else
    {
      for (size_t i = 0; i < object_list.size(); i++)
      {
        if (object_list[i].obj_name != msg_object_list[i].obj_name)
        {
          return true;
        }
        if (object_list[i].parent_frame != msg_object_list[i].parent_frame)
        {
          return true;
        }
        if (object_list[i].obj_pose.position.x != msg_object_list[i].obj_pose.position.x)
        {
          return true;
        }
        if (object_list[i].obj_pose.position.y != msg_object_list[i].obj_pose.position.y)
        {
          return true;
        }
        if (object_list[i].obj_pose.position.z != msg_object_list[i].obj_pose.position.z)
        {
          return true;
        }
        if (object_list[i].obj_pose.orientation.x != msg_object_list[i].obj_pose.orientation.x)
        {
          return true;
        }
        if (object_list[i].obj_pose.orientation.y != msg_object_list[i].obj_pose.orientation.y)
        {
          return true;
        }
        if (object_list[i].obj_pose.orientation.z != msg_object_list[i].obj_pose.orientation.z)
        {
          return true;
        }
        if (object_list[i].obj_pose.orientation.w != msg_object_list[i].obj_pose.orientation.w)
        {
          return true;
        }
      }
      return false;
    }
  }

  bool apply_objects_to_moveit()
  {

    try
    {

      planning_scene.world.collision_objects.clear();
      planning_scene.robot_state.is_diff = true;
      planning_scene.is_diff = true;
      planning_scene.robot_state.attached_collision_objects.clear();

      for (size_t i = 0; i < object_list.size(); i++)
      {
        std::string object_id = object_list[i].obj_name;
        std::string parent_frame = object_list[i].parent_frame;
        geometry_msgs::msg::Pose object_pose = object_list[i].obj_pose;
        std::string stl_path = object_list[i].cad_data_collision;

        geometry_msgs::msg::Pose adapted_pose;

        // Check if file found
        std::ifstream file(stl_path);

        if (!file.good())
        {
          RCLCPP_ERROR(this->get_logger(), "CAD-File for component %s not found!", object_id.c_str());
          RCLCPP_ERROR(this->get_logger(), "Path: '%s'!", stl_path.c_str());
          return false;
        }

        std::string new_parent_frame = get_robot_parent_frame(parent_frame);

        if (new_parent_frame == parent_frame)
        {
          adapted_pose = object_pose;
        }
        else
        {
          adapted_pose = get_adapted_pose(parent_frame, new_parent_frame, object_pose);
        }

        if (new_parent_frame == "unused_frame")
        {
          RCLCPP_ERROR(this->get_logger(), "Parent frame %s not found!", parent_frame.c_str());
          return false;
        }

        moveit_msgs::msg::AttachedCollisionObject spawining_object;

        if (parent_frame == "unused_frame")
        {
          spawining_object.object.header.frame_id = "world";
        }
        else
        {
          spawining_object.object.header.frame_id = new_parent_frame;
        }
        spawining_object.object.id = object_id;
        spawining_object.link_name = new_parent_frame;
        std::string MeshFilePath = "file://" + stl_path;
        shapes::Mesh *m = shapes::createMeshFromResource(MeshFilePath);
        shape_msgs::msg::Mesh shelf_mesh;
        shapes::ShapeMsg shelf_mesh_msg;
        // bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
        shapes::constructMsgFromShape(m, shelf_mesh_msg);
        shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

        spawining_object.object.meshes.push_back(shelf_mesh);
        spawining_object.object.mesh_poses.push_back(adapted_pose);
        spawining_object.object.operation = spawining_object.object.ADD;
        delete m; // Cleanup the mesh object
        planning_scene.robot_state.attached_collision_objects.push_back(spawining_object);
        RCLCPP_ERROR(this->get_logger(), "Spawned STL for component %s in parent frame %s", object_id.c_str(), new_parent_frame.c_str());
      }

      planning_scene_diff_publisher->publish(planning_scene);
      // sleep_for(std::chrono::milliseconds(1000));
      rclcpp::sleep_for(std::chrono::milliseconds(200));
      apply_all_saved_collisions();
      return true; // returns True/False
    }

    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose get_adapted_pose(std::string initial_parent,
                                            std::string new_parent,
                                            geometry_msgs::msg::Pose initial_pose)
  {
    geometry_msgs::msg::Pose adapted_pose;
    geometry_msgs::msg::TransformStamped transform_stamped_parent;
    geometry_msgs::msg::TransformStamped transform_stamped_parent_2 = convert_to_transform_stamped(initial_pose);

    geometry_msgs::msg::TransformStamped result_transform;

    try
    {
      // now
      transform_stamped_parent = tf_buffer_->lookupTransform(new_parent, initial_parent, tf2::get_now());
      // From p_frame to moveit_parent_frame
      // RCLCPP_WARN(this->get_logger(), "Debug-Info: p_frame: %s, moveit_parent_frame: %s", initial_parent.c_str(), new_parent.c_str());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
      return adapted_pose;
    }

    tf2::doTransform(transform_stamped_parent_2, result_transform, transform_stamped_parent);

    // adapted_pose.position.x = transform_stamped_parent.transform.translation.x; //+ transform_stamped_parent_2.transform.translation.x;
    // adapted_pose.position.y = transform_stamped_parent.transform.translation.y; //+ transform_stamped_parent_2.transform.translation.y;
    // adapted_pose.position.z = transform_stamped_parent.transform.translation.z; //+ transform_stamped_parent_2.transform.translation.z;
    // //adapted_pose.orientation = (transform_stamped_parent_2.transform.rotation,transform_stamped_parent.transform.rotation);
    // adapted_pose.orientation = transform_stamped_parent.transform.rotation;

    return convert_to_pose(result_transform);
  }

  geometry_msgs::msg::TransformStamped convert_to_transform_stamped(geometry_msgs::msg::Pose pose)
  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform.translation.x = pose.position.x;
    transform_stamped.transform.translation.y = pose.position.y;
    transform_stamped.transform.translation.z = pose.position.z;
    transform_stamped.transform.rotation = pose.orientation;
    return transform_stamped;
  }

  geometry_msgs::msg::Pose convert_to_pose(geometry_msgs::msg::TransformStamped transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return pose;
  }

  std::string get_robot_parent_frame(std::string initial_parent)
  {
    // Check if parent_frame is a robot link. if not find the next robot link.
    bool parent_frame_is_robot_link = false;

    const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();
    std::vector<std::string> list_of_robot_links = kinematic_model->getLinkModelNames();
    std::string moveit_parent_frame = initial_parent;

    // Check if parent_frame is a robot link. Search for only 10 times
    int search_count = 0;

    while (search_count < 20)
    {

      for (const std::string &link : list_of_robot_links)
      {
        if (moveit_parent_frame == link)
        {
          return moveit_parent_frame;
        }
      }

      // if parent_frame is not a robot link, find the next link and check
      if (parent_frame_is_robot_link == false)
      {
        std::string new_moveit_parent_frame;
        tf_buffer_->_getParent(moveit_parent_frame, tf2::TimePointZero, new_moveit_parent_frame);
        moveit_parent_frame = new_moveit_parent_frame;
        // RCLCPP_WARN(this->get_logger(), "Debug-Info: Parent frame is not a robot link. New parent frame: %s", moveit_parent_frame.c_str());
      }
      search_count++;
    }
    return "unused_frame";
  }

  void logRegisteredObjects()
  {
    for (size_t i = 0; i < object_list.size(); i++)
    {
      std::string object_id = object_list[i].obj_name;
      RCLCPP_INFO(this->get_logger(), "Object ID: %s", object_id.c_str());
    }
  }

  void check_node_presence()
  {
    std::vector<std::string> node_names = this->get_node_names();
    std::string target_node_name = "/pm_robot_xyz_axis_controller"; // Adjust this name

    bool found = std::find(node_names.begin(), node_names.end(), target_node_name) != node_names.end();

    if (found)
    {

      if (!pm_robot_present)
      {
        pm_robot_present = true;
        // wait 10 seconds
        std::this_thread::sleep_for(std::chrono::seconds(15));
        // spawn component
        apply_objects_to_moveit();
        RCLCPP_INFO(this->get_logger(), "Node '%s' is now present. Spawning components...", target_node_name.c_str());
      }
    }
    else
    {
      if (pm_robot_present)
      {
        RCLCPP_WARN(this->get_logger(), "PM Robot is NOT present anymore!", target_node_name.c_str());
        pm_robot_present = false;
      }
    }
  }

  // bool spawn_scene_in_moviet()
  // {
  //   clear_scene();

  //   for (size_t i = 0; i < object_list.size(); i++)
  //   {
  //     std::string object_id = object_list[i].obj_name;
  //     std::string parent_frame = object_list[i].parent_frame;
  //     geometry_msgs::msg::Pose object_pose = object_list[i].obj_pose;
  //     std::string stl_path = object_list[i].cad_data;
  //     // Check if file found
  //     std::ifstream file(stl_path);

  //     if (!file.good()){
  //       RCLCPP_ERROR(this->get_logger(),"CAD-File for component %s not found!", object_id.c_str());
  //       return false;
  //     }

  //     RCLCPP_ERROR(this->get_logger(),"Test1");
  //     std::string new_parent_frame = get_robot_parent_frame(parent_frame);
  //     RCLCPP_ERROR(this->get_logger(),"Test2");

  //     if (new_parent_frame == "unused_frame"){
  //       RCLCPP_ERROR(this->get_logger(),"Parent frame %s not found!", parent_frame.c_str());
  //       return false;
  //     }

  //     RCLCPP_ERROR(this->get_logger(),"Spawning component %s found!", object_id.c_str());

  //     bool object_applied_success = apply_object_to_moveit(object_id, new_parent_frame, object_pose, stl_path);
  //   }
  // }

  bool collision_pair_valid(CollisionRule rule)
  {
    // check if link_1 and link_2 are not empty
    if (rule.link_1.empty() || rule.link_2.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Collision pair is invalid: %s, %s", rule.link_1.c_str(), rule.link_2.c_str());
      return false;
    }
    // check if links are robot links
    const moveit::core::RobotModelPtr &kinematic_model = PM_Robot_Model_Loader->getModel();
    std::vector<std::string> list_of_robot_links = kinematic_model->getLinkModelNames();
    bool link_1_is_robot_link = false;
    bool link_2_is_robot_link = false;
    for (const std::string &link : list_of_robot_links)
    {
      if (rule.link_1 == link)
      {
        link_1_is_robot_link = true;
      }
      if (rule.link_2 == link)
      {
        link_2_is_robot_link = true;
      }
    }
    // check if links are objects in the scene
    bool link_1_is_component = is_object_in_scene(rule.link_1);
    bool link_2_is_component = is_object_in_scene(rule.link_2);

    if (link_1_is_robot_link == false && link_1_is_component == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Link 1: %s, Link 2: %s", rule.link_1.c_str(), rule.link_2.c_str());
      RCLCPP_ERROR(this->get_logger(), "Link 1 is component: %s, Link 2 is component: %s", link_1_is_component ? "true" : "false", link_2_is_component ? "true" : "false");
      RCLCPP_ERROR(this->get_logger(), "Link 1 is robot link: %s, Link 2 is robot link: %s", link_1_is_robot_link ? "true" : "false", link_2_is_robot_link ? "true" : "false");
      return false;
    }

    if (link_2_is_robot_link == false && link_2_is_component == false)
    {
      RCLCPP_ERROR(this->get_logger(), "Link 1: %s, Link 2: %s", rule.link_1.c_str(), rule.link_2.c_str());
      RCLCPP_ERROR(this->get_logger(), "Link 1 is component: %s, Link 2 is component: %s", link_1_is_component ? "true" : "false", link_2_is_component ? "true" : "false");
      RCLCPP_ERROR(this->get_logger(), "Link 1 is robot link: %s, Link 2 is robot link: %s", link_1_is_robot_link ? "true" : "false", link_2_is_robot_link ? "true" : "false");
      return false;
    }

    // This is not necessary
    // if (link_1_is_component == true)
    // {
    //   // check if link_1 is already in the disabled collision list
    //   auto it = std::find(disabled_collision_list.begin(), disabled_collision_list.end(), request->link_1);

    //   if (request->should_check == false)
    //   {
    //     if (it == disabled_collision_list.end())
    //     {
    //       // add to disabled list
    //       disabled_collision_list.push_back(request->link_1);
    //     }
    //   }
    //   else
    //   {
    //     // remove from disabled list
    //     auto it = std::remove(disabled_collision_list.begin(), disabled_collision_list.end(), request->link_1);
    //     disabled_collision_list.erase(it, disabled_collision_list.end());
    //   }
    // }

    // if (link_2_is_component == true)
    // {
    //   if (request->should_check == false)
    //   {
    //     disabled_collision_list.push_back(request->link_2);
    //   }
    //   else
    //   {
    //     // remove from disabled list
    //     auto it = std::remove(disabled_collision_list.begin(), disabled_collision_list.end(), request->link_2);
    //     disabled_collision_list.erase(it, disabled_collision_list.end());
    //   }
    //   response->success = true;
    //   apply_objects_to_moveit();
    //   return;
    // }

    // If both links are valid, return true
    RCLCPP_INFO(this->get_logger(), "Collision pair is valid: %s, %s", rule.link_1.c_str(), rule.link_2.c_str());

    return true;
  }

  void disable_collision(const std::shared_ptr<assembly_manager_interfaces::srv::SetCollisionChecking::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::SetCollisionChecking::Response> response)
  {

    // create rule
    CollisionRule rule;
    rule.link_1 = request->link_1;
    rule.link_2 = request->link_2;
    rule.should_check = request->should_check;

    bool check_valid = collision_pair_valid(rule);
    if (!check_valid)
    {
      response->success = false;
      return;
    }
    else
    {
      // add to the saved collision rules
      saved_collision_rules.push_back(rule);
      apply_all_saved_collisions();
      response->success = true;
      return;
    }
  }

  void apply_all_saved_collisions()
  {
    auto planning_scene = planning_scene_monitor_->getPlanningScene();
    collision_detection::AllowedCollisionMatrix &acm = planning_scene->getAllowedCollisionMatrixNonConst();

    for (const auto &rule : saved_collision_rules)
    {
      acm.setEntry(rule.link_1, rule.link_2, !rule.should_check);
    }

    moveit_msgs::msg::PlanningScene ps_msg;
    planning_scene->getPlanningSceneMsg(ps_msg);
    ps_msg.is_diff = true;
    planning_scene_diff_publisher->publish(ps_msg);

    RCLCPP_INFO(this->get_logger(), "Applied %lu saved collision rules", saved_collision_rules.size());
  }
  // geometry_msgs::msg::Pose translation_rotation_to_pose(geometry_msgs::msg::Vector3 translation, geometry_msgs::msg::Quaternion rotation){
  //   geometry_msgs::msg::Pose pose;
  //   pose.position.x = translation.x;
  //   pose.position.y = translation.y;
  //   pose.position.z = translation.z;
  //   pose.orientation = rotation;

  //   return pose;
  // }

  void spawn_object(const std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Response> response)
  {
    // bool obj_exists = false;
    // int index_of_comp;

    // // Check if file found
    // std::ifstream file(request->cad_data);
    // if (!file.good()){
    //   RCLCPP_ERROR(this->get_logger(),"CAD-File not found!");
    //   response->success=false;
    //   return;
    // }

    // // Check if component exists
    // for (const auto& obj_name : component_names_list) {
    //   if (obj_name == request->obj_name){
    //     //Deleting stl from stl_path_list
    //     obj_exists = true;
    //     index_of_comp = get_index_component_in_list(obj_name);
    //   }
    // }

    // // create object
    // if (obj_exists == false){
    //   component_names_list.push_back(request->obj_name);
    //   component_stl_paths_list.push_back(request->cad_data);
    //   component_parents_list.push_back(request->parent_frame);
    //   component_pose_list.push_back(translation_rotation_to_pose(request->translation,request->rotation));
    //   RCLCPP_INFO(this->get_logger(), "Spawning %s in Moveit", request->obj_name.c_str());
    //   geometry_msgs::msg::Pose object_pose = translation_rotation_to_pose(request->translation,request->rotation);
    //   bool object_applied_success = apply_object_to_moveit(request->obj_name, request->parent_frame, object_pose, request->cad_data);
    // }
    // // update frame
    // else
    // {
    //   component_parents_list[index_of_comp]=request->parent_frame;
    //   component_pose_list[index_of_comp] = translation_rotation_to_pose(request->translation,request->rotation);
    // }

    response->success = true;
  }

  // int get_index_component_in_list(std::string component_name)
  // {
  //   int index = -1;
  //   //check if object exists in object list.
  //   int obj_name_length = component_names_list.size();
  //   for (int i = 0; i < obj_name_length; i++) {
  //     if (component_names_list[i] == component_name) {
  //       index = i;
  //       return index;
  //     }
  //   }
  // }

  void destroy_object(const std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Response> response)
  {
    // response->success = false;
    // for (const auto& obj_name : component_names_list) {
    //   if (obj_name == request->obj_name){
    //     //Deleting stl from stl_path_list
    //     int index;
    //     index = get_index_component_in_list(obj_name);

    //     if (index != -1) {
    //       component_stl_paths_list.erase(component_stl_paths_list.begin() + index);
    //       component_names_list.erase(component_names_list.begin()+index);
    //       component_parents_list.erase(component_parents_list.begin()+index);
    //       component_pose_list.erase(component_pose_list.begin()+index);
    //     }
    //     else {
    //       RCLCPP_ERROR(this->get_logger(),"Error in Destroy Object. Stl not found in list!");
    //     }
    //     response->success = remove_object_from_moveit(request->obj_name);
    //     RCLCPP_INFO(this->get_logger(), "Destroying Object %s", request->obj_name.c_str());
    //     break;
    //   }
    // }
    response->success = true;
  }

  // std::vector<std::string> getSceneObjects()
  // {
  //     std::vector<std::string> object_names;

  //     // Lock the planning scene for reading
  //     planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);

  //     if (!scene)
  //     {
  //         RCLCPP_ERROR(this->get_logger(), "Failed to lock planning scene.");
  //         return object_names;
  //     }

  //     // Get Collision Objects
  //     const auto &collision_objects = scene->getWorld()->getObjectIds();
  //     RCLCPP_ERROR(this->get_logger(), "Collision Objects in Scene:");
  //     for (const auto &obj : collision_objects)
  //     {
  //         RCLCPP_ERROR(this->get_logger(), "- %s", obj.c_str());
  //         object_names.push_back(obj);
  //     }
  //     // Get all attached objects
  //     std::vector<const moveit::core::AttachedBody *> attached_bodies;
  //     scene->getCurrentState().getAttachedBodies(attached_bodies);

  //     RCLCPP_INFO(this->get_logger(), "Attached Objects in Scene: %zu", attached_bodies.size());

  //     for (const auto &body : attached_bodies)
  //     {
  //         RCLCPP_INFO(this->get_logger(), "- %s", body->getName().c_str());
  //         object_names.push_back(body->getName());
  //     }
  //     // // Get Attached Objects
  //     // const auto &attached_objects = scene->getAttachedObjects();
  //     // RCLCPP_ERROR(this->get_logger(), "Attached Objects:");
  //     // for (const auto &obj : attached_objects)
  //     // {
  //     //     RCLCPP_ERROR(this->get_logger(), "- %s", obj.first.c_str());
  //     // }
  //     return object_names;
  // }

  // geometry_msgs::msg::Pose get_pose_of_object(std::string obj_name){
  //   int index = get_index_component_in_list(obj_name);
  //   if (index != -1) {
  //     return component_pose_list[index];
  //   }
  //   else{
  //     RCLCPP_ERROR(this->get_logger(),"Error in get_pose_of_object. Object not found in list!");
  //   }
  // }

  // void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  // {

  //   // Process the received TFMessage
  //   for (const auto& transform : msg->transforms)
  //   {

  //     for (const auto& obj_name : component_names_list) {
  //       // if frame name = object name

  //       if (obj_name == transform.child_frame_id){
  //         geometry_msgs::msg::Pose object_pose;
  //         object_pose.position.x      = transform.transform.translation.x;
  //         object_pose.position.y      = transform.transform.translation.y;
  //         object_pose.position.z      = transform.transform.translation.z;
  //         object_pose.orientation.x   = transform.transform.rotation.x;
  //         object_pose.orientation.y   = transform.transform.rotation.y;
  //         object_pose.orientation.z   = transform.transform.rotation.z;
  //         object_pose.orientation.w   = transform.transform.rotation.w;
  //         std::string p_frame = transform.header.frame_id;
  //         std::string c_frame = transform.child_frame_id;

  //         //check if object exists in object list.
  //         int index = get_index_component_in_list(obj_name);

  //         // if object exists in list
  //         if (index != -1) {
  //           std::string stl_path = component_stl_paths_list[index];
  //           std::string old_parent_frame = component_parents_list[index];
  //           geometry_msgs::msg::Pose old_component_pose = component_pose_list[index];
  //           std::string moveit_parent_frame = p_frame;

  //           // if no changes occured
  //           if (old_parent_frame == moveit_parent_frame && pose_is_equal(old_component_pose, object_pose))
  //           {
  //             RCLCPP_DEBUG(this->get_logger(), "Debug-Info: no change %s", obj_name.c_str());
  //             return;
  //           }
  //           // update changes
  //           else{
  //             RCLCPP_WARN(this->get_logger(), "Debug-Info: Change occured %s", obj_name.c_str());
  //             component_parents_list[index] = p_frame;
  //             component_pose_list[index] = object_pose;
  //           }

  //           //Check if parent_frame is a robot link. if not find the next robot link.
  //           bool parent_frame_is_robot_link = false;

  //           const moveit::core::RobotModelPtr& kinematic_model = PM_Robot_Model_Loader->getModel();
  //           std::vector<std::string> list_of_robot_links=kinematic_model->getLinkModelNames();

  //           // Check if parent_frame is a robot link. Search for only 10 times
  //           int search_count = 0;
  //           while (!parent_frame_is_robot_link){
  //             for (const std::string& link : list_of_robot_links) {
  //               if(moveit_parent_frame == link){
  //                 parent_frame_is_robot_link = true;
  //                 break;
  //               }
  //             }

  //             // if parent_frame is not a robot link, find the next link and check
  //             if (parent_frame_is_robot_link == false){
  //               std::string new_movei_parent_frame;
  //               tf_buffer_->_getParent(moveit_parent_frame, tf2::TimePointZero, new_movei_parent_frame);
  //               moveit_parent_frame=new_movei_parent_frame;
  //               RCLCPP_WARN(this->get_logger(), "Debug-Info: Parent frame is not a robot link. New parent frame: %s", moveit_parent_frame.c_str());

  //             }
  //             if (search_count > 10)
  //             {
  //               return;
  //             }

  //             search_count++;
  //           }

  //           geometry_msgs::msg::TransformStamped transform_stamped_parent;

  //           try{
  //             // now
  //             transform_stamped_parent = tf_buffer_->lookupTransform(moveit_parent_frame, c_frame , tf2::get_now());
  //             // From p_frame to moveit_parent_frame
  //             RCLCPP_WARN(this->get_logger(), "Debug-Info: p_frame: %s, moveit_parent_frame: %s", p_frame.c_str(), moveit_parent_frame.c_str());
  //           }
  //           catch (tf2::TransformException &ex) {
  //             RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
  //             return;
  //           }

  //           geometry_msgs::msg::TransformStamped transform_stamped_parent_2;

  //           try{
  //             // now
  //             transform_stamped_parent_2 = tf_buffer_->lookupTransform(moveit_parent_frame, p_frame , tf2::get_now());
  //             //transform_stamped_parent_2 = tf_buffer_->lookupTransform(moveit_parent_frame, p_frame , tf2::TimePointZero);

  //             // From p_frame to moveit_parent_frame
  //             RCLCPP_WARN(this->get_logger(), "Debug-Info: p_frame: %s, moveit_parent_frame: %s", p_frame.c_str(), moveit_parent_frame.c_str());
  //           }
  //           catch (tf2::TransformException &ex) {
  //             RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
  //             return;
  //           }

  //           object_pose.position.x = transform_stamped_parent.transform.translation.x; //+ transform_stamped_parent_2.transform.translation.x;
  //           object_pose.position.y = transform_stamped_parent.transform.translation.y; //+ transform_stamped_parent_2.transform.translation.y;
  //           object_pose.position.z = transform_stamped_parent.transform.translation.z; //+ transform_stamped_parent_2.transform.translation.z;
  //           object_pose.orientation = (transform_stamped_parent_2.transform.rotation,transform_stamped_parent.transform.rotation);

  //           // RCLCPP_INFO(this->get_logger(), "Moveit_Parent_frame %s", moveit_parent_frame.c_str());
  //           bool object_applied_success = apply_object_to_moveit(c_frame, moveit_parent_frame, object_pose, stl_path);
  //         }
  //         else {
  //           RCLCPP_ERROR(this->get_logger(),"Error in tfCallback. Stl not found!");
  //         }
  //       }
  //     }
  //   }
  // }

  // void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  // {
  //   current_tf_msg = msg;
  //   RCLCPP_WARN(this->get_logger(), "RECIEVED TF MESSAGE");
  //   // // Process the received TFMessage
  //   // for (const auto& transform : msg->transforms)
  //   // {

  //   //   for (const auto& obj_name : component_names_list) {
  //   //     // if frame name = object name

  //   //     if (obj_name == transform.child_frame_id){
  //   //       geometry_msgs::msg::Pose object_pose;
  //   //       object_pose.position.x      = transform.transform.translation.x;
  //   //       object_pose.position.y      = transform.transform.translation.y;
  //   //       object_pose.position.z      = transform.transform.translation.z;
  //   //       object_pose.orientation.x   = transform.transform.rotation.x;
  //   //       object_pose.orientation.y   = transform.transform.rotation.y;
  //   //       object_pose.orientation.z   = transform.transform.rotation.z;
  //   //       object_pose.orientation.w   = transform.transform.rotation.w;
  //   //       std::string p_frame = transform.header.frame_id;
  //   //       std::string c_frame = transform.child_frame_id;

  //   //       //check if object exists in object list.
  //   //       int index = get_index_component_in_list(obj_name);

  //   //       // if object exists in list
  //   //       if (index != -1) {
  //   //         std::string stl_path = component_stl_paths_list[index];
  //   //         std::string old_parent_frame = component_parents_list[index];
  //   //         geometry_msgs::msg::Pose old_component_pose = component_pose_list[index];
  //   //         std::string moveit_parent_frame = p_frame;

  //   //         // if no changes occured
  //   //         if (old_parent_frame == moveit_parent_frame && pose_is_equal(old_component_pose, object_pose))
  //   //         {
  //   //           RCLCPP_DEBUG(this->get_logger(), "Debug-Info: no change %s", obj_name.c_str());
  //   //           return;
  //   //         }
  //   //         // update changes
  //   //         else{
  //   //           RCLCPP_WARN(this->get_logger(), "Debug-Info: Change occured %s", obj_name.c_str());
  //   //           component_parents_list[index] = p_frame;
  //   //           component_pose_list[index] = object_pose;
  //   //         }

  //   //         //Check if parent_frame is a robot link. if not find the next robot link.
  //   //         bool parent_frame_is_robot_link = false;

  //   //         const moveit::core::RobotModelPtr& kinematic_model = PM_Robot_Model_Loader->getModel();
  //   //         std::vector<std::string> list_of_robot_links=kinematic_model->getLinkModelNames();

  //   //         // Check if parent_frame is a robot link. Search for only 10 times
  //   //         int search_count = 0;
  //   //         while (!parent_frame_is_robot_link){
  //   //           for (const std::string& link : list_of_robot_links) {
  //   //             if(moveit_parent_frame == link){
  //   //               parent_frame_is_robot_link = true;
  //   //               break;
  //   //             }
  //   //           }

  //   //           // if parent_frame is not a robot link, find the next link and check
  //   //           if (parent_frame_is_robot_link == false){
  //   //             std::string new_movei_parent_frame;
  //   //             tf_buffer_->_getParent(moveit_parent_frame, tf2::TimePointZero, new_movei_parent_frame);
  //   //             moveit_parent_frame=new_movei_parent_frame;
  //   //             RCLCPP_WARN(this->get_logger(), "Debug-Info: Parent frame is not a robot link. New parent frame: %s", moveit_parent_frame.c_str());

  //   //           }
  //   //           if (search_count > 10)
  //   //           {
  //   //             return;
  //   //           }

  //   //           search_count++;
  //   //         }

  //   //         geometry_msgs::msg::TransformStamped transform_stamped_parent;

  //   //         try{
  //   //           // now
  //   //           transform_stamped_parent = tf_buffer_->lookupTransform(moveit_parent_frame, c_frame , tf2::get_now());
  //   //           // From p_frame to moveit_parent_frame
  //   //           RCLCPP_WARN(this->get_logger(), "Debug-Info: p_frame: %s, moveit_parent_frame: %s", p_frame.c_str(), moveit_parent_frame.c_str());
  //   //         }
  //   //         catch (tf2::TransformException &ex) {
  //   //           RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
  //   //           return;
  //   //         }

  //   //         geometry_msgs::msg::TransformStamped transform_stamped_parent_2;

  //   //         try{
  //   //           // now
  //   //           transform_stamped_parent_2 = tf_buffer_->lookupTransform(moveit_parent_frame, p_frame , tf2::get_now());
  //   //           //transform_stamped_parent_2 = tf_buffer_->lookupTransform(moveit_parent_frame, p_frame , tf2::TimePointZero);

  //   //           // From p_frame to moveit_parent_frame
  //   //           RCLCPP_WARN(this->get_logger(), "Debug-Info: p_frame: %s, moveit_parent_frame: %s", p_frame.c_str(), moveit_parent_frame.c_str());
  //   //         }
  //   //         catch (tf2::TransformException &ex) {
  //   //           RCLCPP_ERROR(this->get_logger(), "Exception: %s", ex.what());
  //   //           return;
  //   //         }

  //   //         object_pose.position.x = transform_stamped_parent.transform.translation.x; //+ transform_stamped_parent_2.transform.translation.x;
  //   //         object_pose.position.y = transform_stamped_parent.transform.translation.y; //+ transform_stamped_parent_2.transform.translation.y;
  //   //         object_pose.position.z = transform_stamped_parent.transform.translation.z; //+ transform_stamped_parent_2.transform.translation.z;
  //   //         object_pose.orientation = (transform_stamped_parent_2.transform.rotation,transform_stamped_parent.transform.rotation);

  //   //         // RCLCPP_INFO(this->get_logger(), "Moveit_Parent_frame %s", moveit_parent_frame.c_str());
  //   //         bool object_applied_success = apply_object_to_moveit(c_frame, moveit_parent_frame, object_pose, stl_path);
  //   //       }
  //   //       else {
  //   //         RCLCPP_ERROR(this->get_logger(),"Error in tfCallback. Stl not found!");
  //   //       }
  //   //     }
  //   //   }
  //   // }
  // }

  // bool apply_object_to_moveit(std::string c_frame, std::string p_frame,geometry_msgs::msg::Pose obj_pose,std::string stl_path)
  //   {
  //     // robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
  //     // const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  //     RCLCPP_INFO(this->get_logger(), "Spawn Object in Moveit: '%s'. Partent frame '%s'", c_frame.c_str(), p_frame.c_str());

  //     try{
  //       moveit_msgs::msg::CollisionObject spawining_object;

  //       if (p_frame == "unused_frame"){
  //         spawining_object.header.frame_id = "world";
  //       }
  //       else{
  //         spawining_object.header.frame_id = p_frame;
  //       }

  //       spawining_object.id = c_frame;
  //       std::string MeshFilePath = "file://" + stl_path;
  //       shapes::Mesh * m = shapes::createMeshFromResource(MeshFilePath);
  //       shape_msgs::msg::Mesh shelf_mesh;
  //       shapes::ShapeMsg shelf_mesh_msg;
  //       //bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
  //       shapes::constructMsgFromShape(m,shelf_mesh_msg);
  //       shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

  //       spawining_object.meshes.push_back(shelf_mesh);
  //       spawining_object.mesh_poses.push_back(obj_pose);
  //       spawining_object.operation = spawining_object.ADD;

  //       moveit_msgs::msg::AttachedCollisionObject attached_object;

  //       attached_object.object = spawining_object;
  //       attached_object.link_name = p_frame;
  //       attached_object.object.header.frame_id = p_frame;
  //       //attached_object.transform = transform_stamped.transform;
  //       delete m;  // Cleanup the mesh object

  //       planning_scene.robot_state.attached_collision_objects.clear();
  //       planning_scene.world.collision_objects.clear();
  //       planning_scene.robot_state.is_diff = true;
  //       planning_scene.is_diff = true;

  //       // planning_scene.world.collision_objects.clear();
  //       // planning_scene.world.collision_objects.push_back(attached_object.object);
  //       // planning_scene_diff_publisher->publish(planning_scene);

  //       moveit_msgs::msg::CollisionObject remove_object;
  //       remove_object.id = spawining_object.id;
  //       remove_object.header.frame_id = spawining_object.header.frame_id;
  //       remove_object.operation = remove_object.REMOVE;

  //       planning_scene.robot_state.attached_collision_objects.clear();
  //       planning_scene.robot_state.attached_collision_objects.push_back(remove_object);
  //       planning_scene_diff_publisher->publish(planning_scene);

  //       planning_scene.robot_state.attached_collision_objects.clear();
  //       planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  //       planning_scene_diff_publisher->publish(planning_scene);

  //       return true; //returns True/False

  //     } catch(const std::exception& ex){
  //         RCLCPP_ERROR(this->get_logger(), "Part '%s'could not be spawned in Moveit. Check STL_Path ", c_frame.c_str());
  //         RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
  //         return false;
  //     }
  //   }

  // bool apply_object_to_moveit(std::string object_id, std::string p_frame,geometry_msgs::msg::Pose obj_pose,std::string stl_path)
  //   {
  //     // robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
  //     // const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  //     RCLCPP_INFO(this->get_logger(), "Spawn Object in Moveit: '%s'. Partent frame '%s'", object_id.c_str(), p_frame.c_str());

  //     try{
  //       remove_object_from_moveit(object_id);

  //       moveit_msgs::msg::AttachedCollisionObject spawining_object;

  //       if (p_frame == "unused_frame"){
  //         spawining_object.object.header.frame_id = "world";
  //       }
  //       else{
  //         spawining_object.object.header.frame_id = p_frame;
  //       }

  //       spawining_object.object.id = object_id;
  //       spawining_object.link_name = p_frame;
  //       std::string MeshFilePath = "file://" + stl_path;
  //       shapes::Mesh * m = shapes::createMeshFromResource(MeshFilePath);
  //       shape_msgs::msg::Mesh shelf_mesh;
  //       shapes::ShapeMsg shelf_mesh_msg;
  //       //bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
  //       shapes::constructMsgFromShape(m,shelf_mesh_msg);
  //       shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

  //       spawining_object.object.meshes.push_back(shelf_mesh);
  //       spawining_object.object.mesh_poses.push_back(obj_pose);
  //       spawining_object.object.operation = spawining_object.object.ADD;

  //       delete m;  // Cleanup the mesh object

  //       planning_scene.world.collision_objects.clear();
  //       planning_scene.robot_state.is_diff = true;
  //       planning_scene.is_diff = true;

  //       planning_scene.robot_state.attached_collision_objects.clear();
  //       planning_scene.robot_state.attached_collision_objects.push_back(spawining_object);
  //       planning_scene_diff_publisher->publish(planning_scene);

  //       return true; //returns True/False

  //     } catch(const std::exception& ex){
  //         RCLCPP_ERROR(this->get_logger(), "Part '%s'could not be spawned in Moveit. Check STL_Path ", object_id.c_str());
  //         RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
  //         return false;
  //     }
  //   }

  // bool pose_is_equal(geometry_msgs::msg::Pose pose_1, geometry_msgs::msg::Pose pose_2){
  //   int round_trans = 9;
  //   int round_rot = 6;
  //   if (  (round_to_digits(pose_1.position.x, round_trans) == round_to_digits(pose_2.position.x, round_trans))&&
  //         (round_to_digits(pose_1.position.y, round_trans) == round_to_digits(pose_2.position.y, round_trans))&&
  //         (round_to_digits(pose_1.position.z, round_trans) == round_to_digits(pose_2.position.z, round_trans))&&
  //         (round_to_digits(pose_1.orientation.w, round_rot) == round_to_digits(pose_2.orientation.w, round_rot))&&
  //         (round_to_digits(pose_1.orientation.x, round_rot) == round_to_digits(pose_2.orientation.x, round_rot))&&
  //         (round_to_digits(pose_1.orientation.y, round_rot) == round_to_digits(pose_2.orientation.y, round_rot))&&
  //         (round_to_digits(pose_1.orientation.z, round_rot) == round_to_digits(pose_2.orientation.z, round_rot)))
  //   {
  //     return true;
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Pose not equal");
  //     return false;
  //   }
  // }

  // double round_to_digits(double value, int digits)
  // {
  //     double rounded_number = std::round(value * std::pow(10, digits)) / std::pow(10, digits);
  //     return rounded_number;
  // }

  // bool remove_object_from_moveit(std::string obj_name){
  //   try{

  //     moveit_msgs::msg::AttachedCollisionObject detach_object;
  //     detach_object.object.id = obj_name;
  //     detach_object.object.operation = detach_object.object.REMOVE;

  //     moveit_msgs::msg::AttachedCollisionObject attached_object;
  //     attached_object.object.id = obj_name;
  //     attached_object.object.operation=attached_object.object.ADD;

  //     planning_scene.robot_state.attached_collision_objects.clear();
  //     planning_scene.world.collision_objects.clear();
  //     planning_scene.robot_state.is_diff = true;
  //     planning_scene.is_diff = true;

  //     planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  //     planning_scene.world.collision_objects.push_back(attached_object.object);

  //     planning_scene_diff_publisher->publish(planning_scene);

  //     moveit_msgs::msg::CollisionObject remove_object;
  //     remove_object.id = obj_name;
  //     remove_object.operation = remove_object.REMOVE;

  //     planning_scene.robot_state.attached_collision_objects.clear();
  //     planning_scene.world.collision_objects.clear();
  //     planning_scene.robot_state.is_diff = true;
  //     planning_scene.is_diff = true;

  //     planning_scene.world.collision_objects.push_back(remove_object);
  //     planning_scene_diff_publisher->publish(planning_scene);

  //     RCLCPP_WARN(this->get_logger(), "INFO");
  //     std::vector<std::string> myVector = planning_scene_interface_.getKnownObjectNames();
  //     for (const auto& element : myVector) {
  //       RCLCPP_ERROR(this->get_logger(), "Exception : %s", element.c_str());
  //     }
  //     return true;

  //   }catch(const std::exception& ex){
  //     return false;
  //   }
  // }

  // void robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
  // {
  //   robot_description_string = msg->data;
  //   RCLCPP_WARN(this->get_logger(), "Detected publishing of robot_description. Respawn all objects in Moveit in 10 seconds.");
  //   rclcpp::sleep_for(std::chrono::seconds(10));
  //   for (const auto& obj_name : component_names_list) {
  //     geometry_msgs::msg::Pose object_pose = get_pose_of_object(obj_name);
  //     std::string stl_path;
  //     int index = get_index_component_in_list(obj_name);
  //     stl_path = component_stl_paths_list[index];
  //     std::string parent_frame = component_parents_list[index];
  //     bool object_applied_success = apply_object_to_moveit(obj_name, parent_frame, object_pose, stl_path);
  //     RCLCPP_WARN(this->get_logger(), "Respawned %s in Moveit", obj_name.c_str());
  //   }

  // }

  // bool remove_object_from_moveit(const std::string &obj_name)
  // {
  //   try {
  //       // Detach the object if attached
  //       moveit_msgs::msg::AttachedCollisionObject detach_object;
  //       detach_object.object.id = obj_name;
  //       detach_object.object.operation = detach_object.object.REMOVE;
  //       planning_scene.robot_state.attached_collision_objects.clear();
  //       planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  //       planning_scene.robot_state.is_diff = true;
  //       planning_scene.is_diff = true;
  //       planning_scene_diff_publisher->publish(planning_scene);
  //       // Remove the object from the world
  //       moveit_msgs::msg::CollisionObject remove_object;
  //       remove_object.id = obj_name;
  //       remove_object.operation = remove_object.REMOVE;
  //       planning_scene.world.collision_objects.clear();
  //       planning_scene.world.collision_objects.push_back(remove_object);
  //       planning_scene.robot_state.is_diff = true;
  //       planning_scene.is_diff = true;
  //       planning_scene_diff_publisher->publish(planning_scene);
  //       // Verify if the object is indeed removed
  //       auto known_objects = planning_scene_interface_.getKnownObjectNames();
  //       if (std::find(known_objects.begin(), known_objects.end(), obj_name) != known_objects.end()) {
  //           RCLCPP_WARN(this->get_logger(), "Object %s still exists in the planning scene.", obj_name.c_str());
  //           return false;
  //       }

  //       return true;
  //     }
  //   catch (const std::exception &ex) {
  //       RCLCPP_ERROR(this->get_logger(), "Exception during object removal: %s", ex.what());
  //       return false;
  //   }
  // }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Create MultiThreadedExecutor

  auto node = std::make_shared<MoveitObjectSpawnerNode>();
  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  // Update scene from monitored topics
  planning_scene_monitor_->startSceneMonitor();
  // planning_scene_monitor_->startStateMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();
  rclcpp::executors::MultiThreadedExecutor executor;
  // node->test();
  executor.add_node(node);

  // Spin the executor in a separate thread
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
