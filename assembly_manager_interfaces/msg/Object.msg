string obj_name
string parent_frame
string uuid
string component_type_uuid
geometry_msgs/Pose obj_pose
string cad_data
string cad_data_collision
string parent_file_path
RefFrame[] ref_frames
Axis[] ref_axis
Plane[] ref_planes
std_msgs/ColorRGBA apperance_color
ComponentProperties properties
# Chuck the component's spawn data is made for (name from the robot's
# bringup configuration, e.g. use_chuck of the station); empty: not declared.
string chuck_config
# Station the component spawns on: "" (not declared), "gonio_left",
# "gonio_right", "smarpod_center" or "smarpod".
string spawning_location
