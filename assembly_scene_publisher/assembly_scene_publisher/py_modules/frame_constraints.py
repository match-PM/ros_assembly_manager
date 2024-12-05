import assembly_manager_interfaces.srv as ami_srv
import assembly_manager_interfaces.msg as ami_msg
# import ros logger type
# RUtilsLogger
from rclpy.impl.rcutils_logger import RcutilsLogger

class FrameConstraintsHandler(ami_msg.FrConstraints):
    def __init__(self, logger: RcutilsLogger = None):
        super(FrameConstraintsHandler, self).__init__()

    @staticmethod
    def return_handler_from_dict(dictionary: dict, logger: RcutilsLogger = None):
        
        frame_constraints = FrameConstraintsHandler(logger=logger)

        # init centroid constraint handler
        centroid_handler = CentroidConstraintHandler.return_handler_from_dict(dictionary.get('centroid',{}), 
                                                                              logger=logger)
        #logger.warn(f"centroid_handler: {str(centroid_handler)}")
        frame_constraints.centroid = centroid_handler.return_as_msg()

        return frame_constraints
    
    def return_as_msg(self):
        msg = ami_msg.FrConstraints()
        for field in dir(msg):
            # Skip attributes that contain capital letters, start with '_', or are constants (e.g., ending with '__DEFAULT')
            if not any(char.isupper() for char in field) and not field.startswith('_') and not field.endswith('__DEFAULT') and hasattr(self, field):
                try:
                    setattr(msg, field, getattr(self, field))  # Set writable attributes
                except AttributeError:
                    # Ignore read-only attributes
                    pass
        return msg
    
    def set_from_msg(self, msg: ami_msg.FrConstraints):
        for field in vars(msg):
            if hasattr(self, field):
                setattr(self, field, getattr(msg, field))
        return self
    
class CentroidConstraintHandler(ami_msg.FrConstraintCentroid):
    def __init__(self, logger: RcutilsLogger = None):
        super(CentroidConstraintHandler, self).__init__()
    
    def return_as_msg(self, logger: RcutilsLogger = None):
        msg = ami_msg.FrConstraintCentroid()
        for field in dir(msg):
            # Skip attributes that contain capital letters, start with '_', or are constants (e.g., ending with '__DEFAULT')
            if not any(char.isupper() for char in field) and not field.startswith('_') and not field.endswith('__DEFAULT') and hasattr(self, field):
                try:
                    setattr(msg, field, getattr(self, field))  # Set writable attributes
                except AttributeError:
                    # Ignore read-only attributes
                    pass
        return msg

    def set_from_msg(self, msg: ami_msg.FrConstraintCentroid):
    
        for field in vars(msg):
            if hasattr(self, field):
                setattr(self, field, getattr(msg, field))
        return self    

    @staticmethod
    def return_handler_from_msg(self, msg: ami_msg.FrConstraintCentroid, logger: RcutilsLogger = None):
        handler = CentroidConstraintHandler(logger)
        error_setting = False
        for field in vars(msg):
            if hasattr(handler, field):
                setattr(handler, field, getattr(msg, field))
            else:
                error_setting = True

        if error_setting:
            if logger is not None:
                logger.error('Error setting centroid constraint handler from msg')
            handler.is_active = False
        else:
            handler.is_active = True

        return handler
    
    @staticmethod
    def return_handler_from_dict(dictionary: dict, logger: RcutilsLogger = None):

        if dictionary == {}:
            return CentroidConstraintHandler()
        
        if logger is not None:
            logger.error(f"dictionary: {str(dictionary)}")

        constraint = CentroidConstraintHandler()
        constraint.dim = dictionary.get('dim','')
        constraint.ref_frame_names = dictionary.get('refFrameNames',[])
        offset_value_vector = dictionary.get('offsetValues',[])
        constraint.offset_values.x = offset_value_vector[0]
        constraint.offset_values.y = offset_value_vector[1]
        constraint.offset_values.z = offset_value_vector[2]
        return constraint
    

