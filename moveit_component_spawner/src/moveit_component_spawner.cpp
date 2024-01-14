#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <tf2_ros/transform_listener.h>
// #include "spawn_object_interfaces/srv/spawn_object.hpp"
// #include "spawn_object_interfaces/srv/destroy_object.hpp"
// #include "spawn_object_interfaces/srv/disable_obj_collision.hpp"
#include "assembly_manager_interfaces/srv/spawn_object.hpp"
#include "assembly_manager_interfaces/srv/destroy_object.hpp"
#include "assembly_manager_interfaces/srv/disable_obj_collision.hpp"
#include <iostream>
#include <fstream>

using std::placeholders::_1;

std::shared_ptr<robot_model_loader::RobotModelLoader> PM_Robot_Model_Loader;


class MoveitObjectSpawnerNode : public rclcpp::Node
  {
  public:

    std::vector<std::string> obj_names_list;
    std::vector<std::string> obj_stl_paths_list;
    rclcpp::Service<assembly_manager_interfaces::srv::SpawnObject>::SharedPtr spawn_object_service;
    rclcpp::Service<assembly_manager_interfaces::srv::DestroyObject>::SharedPtr destroy_object_service;
    rclcpp::Service<assembly_manager_interfaces::srv::DisableObjCollision>::SharedPtr disable_collision_service;
    std::string robot_description_string;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    

    MoveitObjectSpawnerNode() : Node("my_node")
    {
      RCLCPP_INFO(this->get_logger(), "Moveit Object Spawner started!");

      spawn_object_service = this->create_service<assembly_manager_interfaces::srv::SpawnObject>("object_spawner_moveit/spawn_object", std::bind(&MoveitObjectSpawnerNode::spawn_object, this,std::placeholders::_1, std::placeholders::_2));
      destroy_object_service = this->create_service<assembly_manager_interfaces::srv::DestroyObject>("object_spawner_moveit/destroy_object", std::bind(&MoveitObjectSpawnerNode::destroy_object, this,std::placeholders::_1, std::placeholders::_2));
      disable_collision_service = this->create_service<assembly_manager_interfaces::srv::DisableObjCollision>("object_spawner_moveit/disable_collision_of_object", std::bind(&MoveitObjectSpawnerNode::disable_collision, this,std::placeholders::_1, std::placeholders::_2));

      tf_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MoveitObjectSpawnerNode::tfCallback, this, _1));
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

    void spawn_object(const std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::SpawnObject::Response>      response)
    {
      bool obj_exists = false;
      //robot_description_string=kinematic_model->getURDF();
      //RCLCPP_INFO(this->get_logger(), "TEST   %s", robot_description_string.c_str());

      
      std::ifstream file(request->cad_data);
      if (!file.good()){
        RCLCPP_ERROR(this->get_logger(),"CAD-File not found!");
        response->success=false;
        return;
      }

      for (const auto& obj_name : obj_names_list) {
        if (obj_name == request->obj_name){
          //Deleting stl from stl_path_list
          obj_exists = true; 
        }
      }

      if (obj_exists == false){
        obj_names_list.push_back(request->obj_name);
        obj_stl_paths_list.push_back(request->cad_data);
        RCLCPP_INFO(this->get_logger(), "Spawning %s in Moveit", request->obj_name.c_str());
      }

      for (const auto& value : obj_names_list) {
        std::cout << value << std::endl;
      }

      for (const auto& value : obj_stl_paths_list) {
        std::cout << value << std::endl;
      }
      response->success=true;
    }

    void destroy_object(const std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Request> request, std::shared_ptr<assembly_manager_interfaces::srv::DestroyObject::Response>      response)
    {
      response->success = false;

      for (const auto& obj_name : obj_names_list) {
        if (obj_name == request->obj_name){
          //Deleting stl from stl_path_list
          int index = -1;  
          for (long unsigned int i = 0; i < obj_names_list.size(); ++i) {
            if (obj_names_list[i] == obj_name) {
              index = (int)i;
              break;
            }
          }
          if (index != -1) {
            obj_stl_paths_list.erase(obj_stl_paths_list.begin() + index);
            obj_names_list.erase(obj_names_list.begin()+index);
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

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {

      // List all registered objects in terminal
      RCLCPP_INFO(this->get_logger(), "Objects in Moveit:");
      int i = 1;
      for (const auto& value : obj_names_list) {
          RCLCPP_INFO(this->get_logger(), "%i. %s", i, value.c_str());
          i++;
      }

      // Process the received TFMessage
      for (const auto& transform : msg->transforms)
      {
        
        for (const auto& obj_name : obj_names_list) {
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

            int index = -1;  

            //check if object exists in object list.
            for (long unsigned int i = 0; i < obj_names_list.size(); ++i) {
              if (obj_names_list[i] == obj_name) {
                index = (int)i;
                break;
              }
            }

            // if object exists in list
            if (index != -1) {
              std::string stl_path = obj_stl_paths_list[index];

              std::string moveit_parent_frame = p_frame;

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
        //RCLCPP_INFO(this->get_logger(), "Spawn Object in Moveit: '%s'", c_frame.c_str());

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

    bool remove_object_from_moveit(std::string obj_name){
      try{

        moveit_msgs::msg::AttachedCollisionObject detach_object;
        detach_object.object.id = obj_name;
        detach_object.object.operation = detach_object.object.REMOVE;
        
        moveit_msgs::msg::AttachedCollisionObject attached_object;
        attached_object.object.id = obj_name;
        attached_object.object.operation=attached_object.object.ADD;


        planning_scene.robot_state.attached_collision_objects.clear();
        planning_scene.world.collision_objects.clear(); 
        planning_scene.robot_state.is_diff = true;
        planning_scene.is_diff = true;

        planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
        planning_scene.world.collision_objects.push_back(attached_object.object);
        
        planning_scene_diff_publisher->publish(planning_scene);

        moveit_msgs::msg::CollisionObject remove_object;
        remove_object.id = obj_name;
        remove_object.operation = remove_object.REMOVE;
        
        planning_scene.robot_state.attached_collision_objects.clear();
        planning_scene.world.collision_objects.clear(); 
        planning_scene.robot_state.is_diff = true;
        planning_scene.is_diff = true;

        planning_scene.world.collision_objects.push_back(remove_object);
        planning_scene_diff_publisher->publish(planning_scene);

        RCLCPP_WARN(this->get_logger(), "INFO");
        std::vector<std::string> myVector = planning_scene_interface_.getKnownObjectNames();
        for (const auto& element : myVector) {
          RCLCPP_ERROR(this->get_logger(), "Exception : %s", element.c_str());
        }
        return true;

      }catch(const std::exception& ex){
        return false;
      }
    }

  };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveitObjectSpawnerNode>();
  PM_Robot_Model_Loader = std::make_shared<robot_model_loader::RobotModelLoader>(node,"robot_description");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

