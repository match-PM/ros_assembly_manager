import math

x_length = 14
y_length = 9

hypotenuse = (x_length**2 + y_length**2)**0.5
angle = math.degrees(math.atan2(y_length, x_length))

print(f"Hypotenuse: {hypotenuse}")
print(f"Angle: {angle} degrees")


class lever:
    def __init__(self, 
                 multiplicator,
                 hypotenuse,
                 angle,
                 center_x,
                 center_y):
        
        self.multiplicator = multiplicator
        self.hypotenuse = hypotenuse * multiplicator
        self.angle = angle
        self.center_x = center_x
        self.center_y = center_y

    def calculate_length(self):
        x = self.hypotenuse * math.cos(math.radians(self.angle))
        y = self.hypotenuse * math.sin(math.radians(self.angle))
        return x, y
    
    def print_points(self):
        # round to 4 decimal places
        print(f"Printing points for lever with multiplicator {self.multiplicator}:")
        x, y = self.calculate_length()
        p1_x = round(self.center_x - x, 4)
        p1_y = round(self.center_y + y, 4)
        print(f"Point 1: ({p1_x}, {p1_y})")
        p2_x = round(self.center_x + x, 4)
        p2_y = round(self.center_y + y, 4)
        print(f"Point 2: ({p2_x}, {p2_y})")
        p3_x = round(self.center_x + x, 4)
        p3_y = round(self.center_y - y, 4)
        print(f"Point 3: ({p3_x}, {p3_y})")
        p4_x = round(self.center_x - x, 4)
        p4_y = round(self.center_y - y , 4)
        print(f"Point 4: ({p4_x}, {p4_y})")           

    
# Example usage

lever_instance_4 = lever(multiplicator=(4/4),
                       hypotenuse=hypotenuse,
                       angle=angle,
                       center_x=-17,
                       center_y=-12)

lever_instance_4.print_points()

lever_instance_1 = lever(multiplicator=(3/4), 
                       hypotenuse=hypotenuse, 
                       angle=angle,
                       center_x=-17,
                       center_y=-12)

lever_instance_1.print_points()

lever_instance_2 = lever(multiplicator=(2/4),
                       hypotenuse=hypotenuse,
                       angle=angle,
                       center_x=-17,
                       center_y=-12)

lever_instance_2.print_points()

lever_instance_3 = lever(multiplicator=(1/4),
                       hypotenuse=hypotenuse,
                       angle=angle,
                       center_x=-17,
                       center_y=-12)

lever_instance_3.print_points()

