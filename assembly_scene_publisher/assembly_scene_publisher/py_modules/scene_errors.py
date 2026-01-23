

class RefFrameNotFoundError(Exception):
    def __init__(self, frame_name:str):
        self.message = f"Reference frame '{frame_name}' not found in the scene."
        super().__init__(self.message)

class RefPlaneNotFoundError(Exception):
    def __init__(self, plane_name:str):
        self.message = f"Reference plane '{plane_name}' not found in the scene."
        super().__init__(self.message)

class RefAxisNotFoundError(Exception):
    def __init__(self, axis_name:str):
        self.message = f"Reference axis '{axis_name}' not found in the scene."
        super().__init__(self.message)

class ComponentNotFoundError(Exception):
    def __init__(self, obj_name:str):
        self.message = f"Object '{obj_name}' not found in the scene."
        super().__init__(self.message)

class GrippingFrameNotFoundError(Exception):
    def __init__(self, obj_name:str):
        self.message = f"Gripping frame for object '{obj_name}' not found in the scene."
        super().__init__(self.message)

class AssemblyFrameNotFoundError(Exception):
    def __init__(self, obj_name:str):
        self.message = f"Assembly frame for object '{obj_name}' not found in the scene."
        super().__init__(self.message)

class TargetFrameNotFoundError(Exception):
    def __init__(self, obj_name:str):
        self.message = f"Target frame for object '{obj_name}' not found in the scene."
        super().__init__(self.message)


class AssemblyInstructionNotFoundError(Exception):
    def __init__(self, assembly_component:str, target_component:str):
        self.message = f"Assembly instruction for components '{assembly_component}' and '{target_component}' not found in the scene."
        super().__init__(self.message)

class AssemblyInstructionCalculationError(Exception):
    def __init__(self, assembly_component:str, target_component:str):
        self.message = f"Assembly instruction calculation error for components '{assembly_component}' and '{target_component}'."
        super().__init__(self.message)

