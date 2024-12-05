import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger
from geometry_msgs.msg import Pose
from copy import deepcopy, copy

def check_frames_exist_in_scene(frame_names: list[list], 
                                scene: ami_msg.ObjectScene, 
                                logger: RcutilsLogger = None):
    # delete from the list the frames that do not exist
    frame_names_copy = copy(frame_names)
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for frame in obj.ref_frames:
            frame: ami_msg.RefFrame
            if frame.frame_name in frame_names:
                frame_names_copy.remove(frame.frame_name)
    
    for frame in scene.ref_frames_in_scene:
        frame: ami_msg.RefFrame
        if frame.frame_name in frame_names:
            frame_names_copy.remove(frame.frame_name)                       

    if len(frame_names_copy) > 0:
        return False
    else:
        return True

def check_for_duplicate_frames(frame_names: list[str]):
    if len(frame_names) != len(set(frame_names)):
        return True
    else:
        return False

def get_ref_frame_by_name(frame_name:str, 
                          scene: ami_msg.ObjectScene)->ami_msg.RefFrame:
    """
    Returns the ref frame from the ref frames list by the given frame name.
    """
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for ref_frame in obj.ref_frames:
            ref_frame:ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame
    for ref_frame in scene.ref_frames_in_scene:
        ref_frame:ami_msg.RefFrame
        if ref_frame.frame_name == frame_name:
            return ref_frame
        
    return None
            
          
def get_parent_frame_for_ref_frame(frame_name:str, 
                                    scene: ami_msg.ObjectScene)->str:
    """
    This function returns the parent frame for the given ref frame. 
    If the ref frame does not exist the function returns None.
    """
    for obj in scene.objects_in_scene:
        obj: ami_msg.Object
        for ref_frame in obj.ref_frames:
            ref_frame: ami_msg.RefFrame
            if ref_frame.frame_name == frame_name:
                return ref_frame.parent_frame
            
    for ref_frame in scene.ref_frames_in_scene:
        ref_frame: ami_msg.RefFrame
        if ref_frame.frame_name == frame_name:
            return ref_frame.parent_frame    

    return None 

def check_ref_frames_for_same_parent_frame(frame_names:list[str], 
                                            scene: ami_msg.ObjectScene)->bool:
    """
    This function checks if all given ref frames have the same parent frame.
    If this is the case the function returns True. If not the function returns False.
    Parameters:
    - frame_names: list of strings with the names of the ref frames to check
    """
    parent_frame = get_parent_frame_for_ref_frame(frame_names[0],scene=scene)
    for frame in frame_names:
        if parent_frame != get_parent_frame_for_ref_frame(frame,scene=scene) or parent_frame is None:
            return False
    return True
    
            
            
            





            
class FrameConstraintsHandler(ami_msg.FrConstraints):
    
    CALCULATION_ORDER = ['centroid',                 
                         'in_plane']
    
    def __init__(self, logger: RcutilsLogger = None):
        super(FrameConstraintsHandler, self).__init__()
        self.logger = logger

    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                    component_name: str = None,
                                    scene: ami_msg.ObjectScene = None, 
                                    logger: RcutilsLogger = None):
        
        frame_constraints = FrameConstraintsHandler(logger=logger)
        
        
        # init centroid constraint handler
        centroid_handler = CentroidConstraintHandler.return_handler_from_dict(dictionary.get('centroid',{}), 
                                                                                compoenent_name=component_name,
                                                                                logger=logger)
        centroid_handler.set_is_active(scene=scene,
                                       component_name=component_name)
        
        frame_constraints.centroid = centroid_handler.return_as_msg()

        return frame_constraints
    
    @staticmethod
    def return_handler_from_msg(msg: ami_msg.FrConstraints, 
                                scene: ami_msg.ObjectScene = None, 
                                component_name: str = None,
                                logger: RcutilsLogger = None):
        
        handler = FrameConstraintsHandler(logger)
        msg_copy = deepcopy(msg)
        handler.unit = msg_copy.unit
        
        centroid_handler = CentroidConstraintHandler.return_handler_from_msg(msg_copy.centroid, logger)
        centroid_handler.set_is_active(scene,component_name=component_name)
        handler.centroid = centroid_handler.return_as_msg()

        return handler
    
    def return_as_msg(self):
        msg = ami_msg.FrConstraints()
        msg.unit = self.unit
        msg.centroid = self.centroid
        
        return msg
    
    def set_from_msg(self, msg: ami_msg.FrConstraints):
        msg_copy = deepcopy(msg)
        self.unit = msg_copy.unit
        self.centroid = msg_copy.centroid
    
    def calculate_frame_constraints(self, 
                                    initial_pose: Pose,
                                    scene: ami_msg.ObjectScene, 
                                    component_name: str = None,
                                    logger:RcutilsLogger = None)->Pose:
        """
        This function calculates the frame constraints and returns the pose of the frame.
        """
        result_pose = Pose()
        
        for constraint in self.CALCULATION_ORDER:
            if constraint == 'centroid':
                # calculate the centroid constraint
                centroid_handler = CentroidConstraintHandler(logger=self.logger)
                centroid_handler.set_from_msg(self.centroid)
                result_pose = centroid_handler.calculate_constraint(initial_frame_pose=initial_pose,
                                                                  scene=scene,
                                                                  component_name=component_name,
                                                                  logger=logger)
            if constraint == 'in_plane':
                pass
                # result_pose = self.calculateconstraint(initial_pose=initial_pose,
                #                                                   scene=scene,
                #                                                   logger=logger)
        
        return result_pose
    
    
    
    
    
    
    
    
    
    
    
    
    
class CentroidConstraintHandler(ami_msg.FrConstraintCentroid):
    def __init__(self, logger: RcutilsLogger = None):
        super(CentroidConstraintHandler, self).__init__()
        self.logger = logger
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintCentroid()
        msg.is_active = self.is_active
        msg.dim = self.dim
        msg.offset_values = self.offset_values
        msg.ref_frame_names = self.ref_frame_names
        return msg

    def set_from_msg(self, msg: ami_msg.FrConstraintCentroid):
        msg_copy = deepcopy(msg)
        self.is_active = msg_copy.is_active
        self.dim = msg_copy.dim
        self.offset_values = msg_copy.offset_values
        self.ref_frame_names = msg_copy.ref_frame_names

    def set_is_active(self, scene: ami_msg.ObjectScene = None, component_name = None):
        
        # check if enough reference frames are provided
        if len(self.ref_frame_names)>1:
            self.is_active = True
            if self.logger is not None:
                self.logger.debug(f'Enough reference frames provided for centroid constraint {self.ref_frame_names}.')
        else:
            self.is_active = False
            if self.logger is not None:
                pass
                #self.logger.error(f'Not enough reference frames provided for centroid constraint{self.ref_frame_names}. ')
            return
        
        # check if the dimension is valid
        if self.dim == '':
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Dimension not provided for centroid constraint')
            return
        
        if not ((self.dim == 'xyz' or self.dim == 'xzy' or self.dim == 'yxz' or self.dim == 'yzx' or self.dim == 'zxy' or self.dim == 'zyx')
                or (self.dim == 'x' or self.dim == 'y' or self.dim == 'z')
                or (self.dim == 'xy' or self.dim == 'yx' or self.dim == 'yz' or self.dim == 'zy' or self.dim == 'xz' or self.dim == 'zx')):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Invalid dimension provided for centroid constraint')
            return
        else:
            self.is_active = True
        
        if check_for_duplicate_frames(self.ref_frame_names):
            self.is_active = False
            
            if self.logger is not None:
                self.logger.error('Duplicate reference frames provided for centroid constraint')
                
            return
        
        if scene is not None and not check_frames_exist_in_scene(frame_names=self.ref_frame_names, 
                                                                 scene=scene,
                                                                 logger=self.logger):
            self.is_active = False
            if self.logger is not None:
                self.logger.error(f'Reference frames "{self.ref_frame_names}" not found in the scene!')
            return
        
        if scene is not None and component_name is not None and not check_ref_frames_for_same_parent_frame(frame_names=self.ref_frame_names, 
                                                                            scene=scene):
            self.is_active = False
            if self.logger is not None:
                self.logger.error('Reference frames do not have the same parent frame!')
            return
        

    
    def calculate_constraint(self, 
                             initial_frame_pose:Pose,
                             scene: ami_msg.ObjectScene,
                             unit: str = 'mm',
                             component_name: str = None,
                             logger: RcutilsLogger = None) -> Pose:
        
        #def caluclate_frame_centroid(self, ref_frames: list[str], dim: str, offset_values: list[float], initial_pose: Pose) -> Pose:
        """
        This function calculates the centroid of the given ref frames and returns the pose of the centroid.
        """
        # Check if constraint is valid
        self.set_is_active(scene=scene, component_name=component_name)   
        
        if self.is_active == False:
            #self.logger.error('Centroid constraint is not active')
            return initial_frame_pose
        
        else:
            self.logger.debug('Centroid constraint is active')
        
        centroid_pose = Pose()
        for index, frame in enumerate(self.ref_frame_names):
            fr: ami_msg.RefFrame = get_ref_frame_by_name(frame_name=frame,
                                                         scene=scene)
            
            # This should normaly not happen as set_is_active should have already checked this
            if fr is None:
                self.logger.error(f"Frame {frame} not found in the scene")
                return initial_frame_pose
            
            frame_pose = fr.pose
            if 'x' in self.dim:
                centroid_pose.position.x += frame_pose.position.x
            if 'y' in self.dim:
                centroid_pose.position.y += frame_pose.position.y
            if 'z' in self.dim:
                centroid_pose.position.z += frame_pose.position.z
        centroid_pose.position.x = centroid_pose.position.x/len(self.ref_frame_names)
        centroid_pose.position.y = centroid_pose.position.y/len(self.ref_frame_names)
        centroid_pose.position.z = centroid_pose.position.z/len(self.ref_frame_names)
        
        multiplier = 1
        
        if unit == 'mm':
            multiplier = 1000
            
        if unit == 'um':
            multiplier = 1000000
        
        if unit == 'm':
            multiplier = 1
            
        centroid_pose.position.x += self.offset_values.x / multiplier
        centroid_pose.position.y += self.offset_values.y / multiplier
        centroid_pose.position.z += self.offset_values.z / multiplier
        
        if 'x' in self.dim:
            initial_frame_pose.position.x = centroid_pose.position.x
        else:
            initial_frame_pose.position.x = initial_frame_pose.position.x
        if 'y' in self.dim:
            initial_frame_pose.position.y = centroid_pose.position.y
        else:
            initial_frame_pose.position.y = initial_frame_pose.position.y
        if 'z' in self.dim:
            initial_frame_pose.position.z = centroid_pose.position.z
        else:
            initial_frame_pose.position.z = initial_frame_pose.position.z

        self.logger.debug(f"Centroid pose: {str(initial_frame_pose)}")
        return initial_frame_pose
        
    @staticmethod
    def return_handler_from_msg( msg: ami_msg.FrConstraintCentroid, logger: RcutilsLogger = None):
        handler = CentroidConstraintHandler(logger)
        msg_copy = deepcopy(msg)
        handler.is_active = msg_copy.is_active
        handler.dim = msg_copy.dim
        handler.offset_values = msg_copy.offset_values
        handler.ref_frame_names = msg_copy.ref_frame_names
        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, 
                                 compoenent_name: str = None, 
                                 logger: RcutilsLogger = None):

        if dictionary == {}:
            return CentroidConstraintHandler(logger)

        constraint_handler = CentroidConstraintHandler(logger)
        constraint_handler.dim = dictionary.get('dim','')
        constraint_handler.ref_frame_names = dictionary.get('ref_frame_names',[])
        
        if compoenent_name is not None:
            for index, frame_name in enumerate(constraint_handler.ref_frame_names):
                constraint_handler.ref_frame_names[index] = compoenent_name + '_' + frame_name
            
        offset_value_vector = dictionary.get('offset_values',[])
        
        if len(offset_value_vector) < 3:
            offset_value_vector = [0,0,0]
        constraint_handler.offset_values.x = float(offset_value_vector[0])
        constraint_handler.offset_values.y = float(offset_value_vector[1])
        constraint_handler.offset_values.z = float(offset_value_vector[2])
        return constraint_handler
    

