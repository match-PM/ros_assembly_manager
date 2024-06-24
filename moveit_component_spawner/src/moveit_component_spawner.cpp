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
#include "assembly_manager_interfaces/srv/spawn_object.hpp"
#include "assembly_manager_interfaces/srv/destroy_object.hpp"
#include "assembly_manager_interfaces/srv/disable_obj_collision.hpp"
#include <iostream>
#include <fstream>

//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
// #include "spawn_object_interfaces/srv/spawn_object.hpp"
// #include "spawn_object_interfaces/srv/destroy_object.hpp"
// #include "spawn_object_interfaces/srv/disable_obj_collision.hpp"

using std::placeholders::_1;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;

class MoveitObjectSpawnerNode : public rclcpp::Node
  {
  public:

    std::vector<std::string> component_names_list;
    std::vector<std::string> component_stl_paths_list;
    std::vector<std::string> component_parents_list;
    std::vector<geometry_msgs::msg::Pose> component_pose_list;
    rclcpp::Service<assembly_manager_interfaces::srv::SpawnObject>::SharedPtr spawn_object_service;
    rclcpp::Service<assembly_manager_interfaces::srv::DestroyObject>::SharedPtr destroy_object_service;
    rclcpp::Service<assembly_manager_interfaces::srv::DisableObjCollision>::SharedPtr disable_collision_service;
    std::string robot_description_string;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    

    MoveitObjectSpawnerNode() : Node("my_node")
    {
      RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");

      auto tf_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions options;
      options.callback_group = tf_callback_group;

      spawn_object_service = this->create_service<assembly_manager_interfaces::srv::SpawnObject>("moveit_component_spawner/spawn_object", std::bind(&MoveitObjectSpawnerNode::spawn_object, this,std::placeholders::_1, std::placeholders::_2));
      destroy_object_service = this->create_service<assembly_manager_interfaces::srv::DestroyObject>("moveit_component_spawner/destroy_object", std::bind(&MoveitObjectSpawnerNode::destroy_object, this,std::placeholders::_1, std::placeholders::_2));
      disable_collision_service = this->create_service<assembly_manager_interfaces::srv::DisableObjCollision>("moveit_component_spawner/disable_collision_of_object", std::bind(&MoveitObjectSpawnerNode::disable_collision, this,std::placeholders::_1, std::placeholders::_2));

      tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1),options);
      planning_scene_diff_publisher =this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

      while (planning_scene_diff_publisher->get_subscription_count() < 1)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_WARN(this->get_logger(), "Waiting for planing scene...");
      }

      //tf_buffer_ = tf2_ros::Buffer(this->get_clock(), tf2::Duration(10), this->shared_from_this());
      tf_buffer_ =      std::make_unique<tf2_ros::Buffer>(this->get_clock());

      tf_listener_ =      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
    }

  private:
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
    std::vector<moveit_msgs::msg::AttachedCollisionObject> collision_objects_list;
    moveit_msgs::msg::PlanningScene planning_scene;

    void disable_collision(const std::shared_ptr<assembly_manager_interfaces::srv::DisableObjCollision::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::DisableObjCollision::Response>response){
      return;
    }

    geometry_msgs::msg::Pose translation_rotation_to_pose(geometry_msgs::msg::Vector3 translation, geometry_msgs::msg::Quaternion rotation){
      geometry_msgs::msg::Pose pose;
      pose.position.x = translation.x;
      pose.position.y = translation.y;
      pose.position.z = translation.z;
      pose.orientation = rotation;

      return pose;
    }

    void spawn_object(const std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Response>      response)
    {
      bool obj_exists = false;
      int index_of_comp;
      
      // Check if file found
      std::ifstream file(request->cad_data);
      if (!file.good()){
        RCLCPP_ERROR(this->get_logger(),"CAD-File not found!");
        response->success=false;
        return;
      }

      // Check if component exists
      for (const auto& obj_name : component_names_list) {
        if (obj_name == request->obj_name){
          //Deleting stl from stl_path_list
          obj_exists = true; 
          index_of_comp = get_index_component_in_list(obj_name);
        }
      }

      // create object
      if (obj_exists == false){
        component_names_list.push_back(request->obj_name);
        component_stl_paths_list.push_back(request->cad_data);
        component_parents_list.push_back(request->parent_frame);
        component_pose_list.push_back(translation_rotation_to_pose(request->translation,request->rotation));
        RCLCPP_INFO(this->get_logger(), "Spawning %s in Moveit", request->obj_name.c_str());
        geometry_msgs::msg::Pose object_pose = translation_rotation_to_pose(request->translation,request->rotation);
        bool object_applied_success = apply_object_to_moveit(request->obj_name, request->parent_frame, object_pose, request->cad_data);
      }
      // update frame
      else
      {
        component_parents_list[index_of_comp]=request->parent_frame;
        component_pose_list[index_of_comp] = translation_rotation_to_pose(request->translation,request->rotation);
      }

      response->success=true;
    }

    int get_index_component_in_list(std::string component_name)
    {
      int index = -1;  
      //check if object exists in object list.
      int obj_name_length = component_names_list.size();
      for (int i = 0; i < obj_name_length; i++) {
        if (component_names_list[i] == component_name) {
          index = i;
          return index;
        }
      }
    }

    void destroy_object(const std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Response>      response)
    {
      response->success = false;
      for (const auto& obj_name : component_names_list) {
        if (obj_name == request->obj_name){
          //Deleting stl from stl_path_list
          int index;
          index = get_index_component_in_list(obj_name);
          
          if (index != -1) {
            component_stl_paths_list.erase(component_stl_paths_list.begin() + index);
            component_names_list.erase(component_names_list.begin()+index);
            component_parents_list.erase(component_parents_list.begin()+index);
            component_pose_list.erase(component_pose_list.begin()+index);
          } 
          else {
            RCLCPP_ERROR(this->get_logger(),"Error in Destroy Object. Stl not found in list!");
          }
          response->success = remove_object_from_moveit(request->obj_name);
          RCLCPP_INFO(this->get_logger(), "Destroying Object %s", request->obj_name.c_str());
          break;
        }
      }      
    }

    void logRegisteredObjects()
    {
      // List all registered objects in terminal
      RCLCPP_INFO(this->get_logger(), "Objects in Moveit:");
      int i = 1;
      for (const auto& value : component_names_list) {
           RCLCPP_INFO(this->get_logger(), "%i. %s", i, value.c_str());
           i++;
      }
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {

      // Process the received TFMessage
      for (const auto& transform : msg->transforms)
      {
        
        for (const auto& obj_name : component_names_list) {
          // if frame name = object name
        
          if (obj_name == transform.child_frame_id){
            geometry_msgs::msg::Pose object_pose;
            object_pose.position.x      = transform.transform.translation.x;
            object_pose.position.y      = transform.transform.translation.y;
            object_pose.position.z      = transform.transform.translation.z;
            object_pose.orientation.x   = transform.transform.rotation.x;
            object_pose.orientation.y   = transform.transform.rotation.y;
            object_pose.orientation.z   = transform.transform.rotation.z;
            object_pose.orientation.w   = transform.transform.rotation.w;
            std::string p_frame = transform.header.frame_id;
            std::string c_frame = transform.child_frame_id;

            //check if object exists in object list.
            int index = get_index_component_in_list(obj_name);

            // if object exists in list
            if (index != -1) {
              std::string stl_path = component_stl_paths_list[index];
              std::string old_parent_frame = component_parents_list[index];
              geometry_msgs::msg::Pose old_component_pose = component_pose_list[index];
              std::string moveit_parent_frame = p_frame;

              // if no changes occured
              if (old_parent_frame == moveit_parent_frame && pose_is_equal(old_component_pose, object_pose))
              {
                RCLCPP_DEBUG(this->get_logger(), "Debug-Info: no change %s", obj_name.c_str());
                return;
              }
              // update changes
              else{
                component_parents_list[index] = p_frame;
                component_pose_list[index] = object_pose;
              }

              //Check if parent_frame is a robot link. if not find the next robot link.
              bool parent_frame_is_robot_link = false;

              const moveit::core::RobotModelPtr& kinematic_model = PM_Robot_Model_Loader->getModel();
              std::vector<std::string> list_of_robot_links=kinematic_model->getLinkModelNames();
              
              while (!parent_frame_is_robot_link){
                for (const std::string& link : list_of_robot_links) {
                  if(moveit_parent_frame == link){
                    parent_frame_is_robot_link = true;
                    break;
                  }
                }

                // if parent_frame is not a robot link, find the next link and check
                if (parent_frame_is_robot_link == false){
                  std::string new_movei_parent_frame;
                  tf_buffer_->_getParent(moveit_parent_frame, tf2::TimePointZero, new_movei_parent_frame);
                  moveit_parent_frame=new_movei_parent_frame;
                }
              } 
              // RCLCPP_INFO(this->get_logger(), "Moveit_Parent_frame %s", moveit_parent_frame.c_str());
              bool object_applied_success = apply_object_to_moveit(c_frame, moveit_parent_frame, object_pose, stl_path);
            }
            else {
              RCLCPP_ERROR(this->get_logger(),"Error in tfCallback. Stl not found!");
            }
          }
        }
      }
    }

    bool apply_object_to_moveit(std::string c_frame, std::string p_frame,geometry_msgs::msg::Pose obj_pose,std::string stl_path) 
      {
        // robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
        // const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
        RCLCPP_INFO(this->get_logger(), "Spawn Object in Moveit: '%s'", c_frame.c_str());

        try{
          moveit_msgs::msg::CollisionObject spawining_object;

          if (p_frame == "unused_frame"){
            spawining_object.header.frame_id = "world";
          }
          else{
            spawining_object.header.frame_id = p_frame;
          }

          spawining_object.id = c_frame;
          std::string MeshFilePath = "file://" + stl_path;
          shapes::Mesh * m = shapes::createMeshFromResource(MeshFilePath);
          shape_msgs::msg::Mesh shelf_mesh;
          shapes::ShapeMsg shelf_mesh_msg;
          //bool success = shapes::constructMsgFromShape(m,shelf_mesh_msg);
          shapes::constructMsgFromShape(m,shelf_mesh_msg);
          shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);

          spawining_object.meshes.push_back(shelf_mesh);
          spawining_object.mesh_poses.push_back(obj_pose);
          spawining_object.operation = spawining_object.ADD;

          moveit_msgs::msg::AttachedCollisionObject attached_object;

          attached_object.object = spawining_object;
          attached_object.link_name = p_frame;
          attached_object.object.header.frame_id = p_frame;
          //attached_object.transform = transform_stamped.transform;
          delete m;  // Cleanup the mesh object

          planning_scene.robot_state.attached_collision_objects.clear();
          planning_scene.world.collision_objects.clear(); 
          planning_scene.robot_state.is_diff = true;
          planning_scene.is_diff = true;

          // planning_scene.world.collision_objects.clear();
          // planning_scene.world.collision_objects.push_back(attached_object.object);
          // planning_scene_diff_publisher->publish(planning_scene);

          moveit_msgs::msg::CollisionObject remove_object;
          remove_object.id = spawining_object.id;
          remove_object.header.frame_id = spawining_object.header.frame_id;
          remove_object.operation = remove_object.REMOVE;

          // planning_scene.world.collision_objects.clear();
          // planning_scene.world.collision_objects.push_back(remove_object);
          // planning_scene.is_diff = true;
          //planning_scene_diff_publisher->publish(planning_scene);

          planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
          planning_scene_diff_publisher->publish(planning_scene);

          return true; //returns True/False

        } catch(const std::exception& ex){
            RCLCPP_ERROR(this->get_logger(), "Part '%s'could not be spawned in Moveit. Check STL_Path ", c_frame.c_str());
            RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", ex.what());
            return false;
        }
      }

    bool pose_is_equal(geometry_msgs::msg::Pose pose_1, geometry_msgs::msg::Pose pose_2){
      int round_trans = 9;
      int round_rot = 6;
      if (  (round_to_digits(pose_1.position.x, round_trans) == round_to_digits(pose_2.position.x, round_trans))&&
            (round_to_digits(pose_1.position.y, round_trans) == round_to_digits(pose_2.position.y, round_trans))&&
            (round_to_digits(pose_1.position.z, round_trans) == round_to_digits(pose_2.position.z, round_trans))&&
            (round_to_digits(pose_1.orientation.w, round_rot) == round_to_digits(pose_2.orientation.w, round_rot))&&
            (round_to_digits(pose_1.orientation.x, round_rot) == round_to_digits(pose_2.orientation.x, round_rot))&&
            (round_to_digits(pose_1.orientation.y, round_rot) == round_to_digits(pose_2.orientation.y, round_rot))&&
            (round_to_digits(pose_1.orientation.z, round_rot) == round_to_digits(pose_2.orientation.z, round_rot)))
      {
        return true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Pose not equal");
        return false;
      }
    }

    double round_to_digits(double value, int digits)
    {
        double rounded_number = std::round(value * std::pow(10, digits)) / std::pow(10, digits);
        return rounded_number;
    }
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

    bool remove_object_from_moveit(const std::string &obj_name)
    {
    try {
        // Detach the object if attached
        moveit_msgs::msg::AttachedCollisionObject detach_object;
        detach_object.object.id = obj_name;
        detach_object.object.operation = detach_object.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.clear();
        planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
        planning_scene.robot_state.is_diff = true;
        planning_scene.is_diff = true;
        planning_scene_diff_publisher->publish(planning_scene);
        // Remove the object from the world
        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.id = obj_name;
        remove_object.operation = remove_object.REMOVE;
        planning_scene.world.collision_objects.clear();
        planning_scene.world.collision_objects.push_back(remove_object);
        planning_scene.robot_state.is_diff = true;
        planning_scene.is_diff = true;
        planning_scene_diff_publisher->publish(planning_scene);
        // Verify if the object is indeed removed
        auto known_objects = planning_scene_interface_.getKnownObjectNames();
        if (std::find(known_objects.begin(), known_objects.end(), obj_name) != known_objects.end()) {
            RCLCPP_WARN(this->get_logger(), "Object %s still exists in the planning scene.", obj_name.c_str());
            return false;
        }

        return true;
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception during object removal: %s", ex.what());
        return false;
    }
}

  };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Create MultiThreadedExecutor
  
  auto node = std::make_shared<MoveitObjectSpawnerNode>();
  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(node,"robot_description");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor in a separate thread
  
  executor.spin();

  rclcpp::shutdown();

  //rclcpp::spin(node);
  //rclcpp::shutdown();
  return 0;
}

