
import sympy as sp

import CGAL.CGAL_Kernel as CGAL_K

def point_from_dict(point_dict):
    return sp.Point3D(point_dict['position']['x']*1e0, point_dict['position']['y']*1e0,point_dict['position']['z']*1e0, evaluate=False)

def point_from_dict_2(point_dict)->CGAL_K.Point_3:
    return CGAL_K.Point_3(point_dict['position']['x'], point_dict['position']['y'],point_dict['position']['z'])

def get_point_of_plane_intersection(plane1: sp.Plane, plane2: sp.Plane, plane3: sp.Plane) -> sp.Point3D:
    line = plane1.intersection(plane2)
    # Get the first point of intersection, should also be the only one
    #inter:sp.Point3D = plane3.intersection(line[0])

    try:
        plane3.intersection(line[0])[0]
    except Exception as e:
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")

    inter:sp.Point3D = plane3.intersection(line[0])[0]

    if not isinstance(inter, sp.Point3D):
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")
    
    # Value Error if not a point

    return inter
def get_point_of_plane_intersection_2(plane1: CGAL_K.Plane_3, plane2: CGAL_K.Plane_3, plane3: CGAL_K.Plane_3) -> CGAL_K.Point_3:
    print(type(plane3))

    print(CGAL_K.do_intersect(plane1, plane2))

    if not isinstance(plane1, CGAL_K.Plane_3) or not isinstance(plane2, CGAL_K.Plane_3) or not isinstance(plane3, CGAL_K.Plane_3):
        raise ValueError("All inputs must be instances of CGAL.Kernel.Plane_3.")
    
    line:CGAL_K.Line_3 = CGAL_K.intersection(plane1, plane2)
    # Get the first point of intersection, should also be the only one
    #inter:sp.Point3D = plane3.intersection(line[0])

    print(f"Is line {CGAL_K.Object.is_Line_3(line)}")

    print(type(plane2))

    print(f'intersect: {CGAL_K.do_intersect(plane3, plane2)}')

    inter = CGAL_K.intersection(line, plane3)
    print(inter)

    
    if not isinstance(inter, CGAL_K.Point_3):
        raise ValueError(f"Given planes (1.{plane1}, 2.{plane2}, 3.{plane3}) do not have a single point of intersection. Invalid plane selection!")
    
    # Value Error if not a point

    return inter

def add_Points_3(p1:CGAL_K.Point_3, p2:CGAL_K.Point_3)->CGAL_K.Point_3:
    return CGAL_K.Point_3(p1.x()+p2.x(), p1.y()+p2.y(), p1.z()+p2.z())

Glas_Platelet_Spawn_mod = sp.Point3D(0.875, 0.3265, 1.769, evaluate=False)
#Glas_Platelet_Spawn = sp.Point3D(0.875, 0.3265, 1.769, evaluate=False) 
Glas_Platelet_Spawn = sp.Point3D(0.0, 0.0, 0.0, evaluate=False) 

#Glas_Platelet_Spawn2= CGAL_K.Point_3(0.875, 0.3265, 1.769)
Glas_Platelet_Spawn2= CGAL_K.Point_3(0.0, 0.0, 1.0)

#Glas_Platelet_Spawn = sp.Point3D(0.0, 0.0, 0.0, evaluate=False) 
#UFC_Spawn = sp.Point3D(0.5325, 0.40528, 1.2755, evaluate=False)
UFC_Spawn = sp.Point3D(0.0, 0.0, 0.0, evaluate=False)
UFC_Spawn2 = CGAL_K.Point_3(0.5325, 0.40528, 1.2755)    

Glas_Platelet_Vision_Point_1_dict = {
                                            "position": {
                                                "x": -0.00038314608905751994,
                                                "y": 0.0019963970365923112,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }

Glas_Platelet_Vision_Point_1 = point_from_dict(Glas_Platelet_Vision_Point_1_dict) + Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_1_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_1_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_1_helper_dict = {
                                            "position": {
                                                "x": -0.00038365665824348203,
                                                "y": 0.0019968580724469523,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }

Glas_Platelet_Vision_Point_1_helper = point_from_dict(Glas_Platelet_Vision_Point_1_helper_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_1_helper_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_1_helper_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_2_dict = {
                                            "position": {
                                                "x": -3.272492340464703e-07,
                                                "y": -0.020000390169755067,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
Glas_Platelet_Vision_Point_2 = point_from_dict(Glas_Platelet_Vision_Point_2_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_2_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_2_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_2_helper_dict = {
                                            "position": {
                                                "x": -3.9902564719952457e-07,
                                                "y": -0.019998852709652157,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
Glas_Platelet_Vision_Point_2_helper = point_from_dict(Glas_Platelet_Vision_Point_2_helper_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_2_helper_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_2_helper_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_3_dict = {
                                            "position": {
                                                "x": -0.03000146785490526,
                                                "y": -0.019999542459995807,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
Glas_Platelet_Vision_Point_3 = point_from_dict(Glas_Platelet_Vision_Point_3_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_3_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_3_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_3_helper_dict = {
                                            "position": {
                                                "x": -0.029999670716108134,
                                                "y": -0.0200002225909939,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
Glas_Platelet_Vision_Point_3_helper = point_from_dict(Glas_Platelet_Vision_Point_3_helper_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_3_helper_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_3_helper_dict), Glas_Platelet_Spawn2)

Glas_Platelet_Vision_Point_4_dict = {
                                            "position": {
                                                "x": -0.028537095437947583,
                                                "y": 0.000948097956374134,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
Glas_Platelet_Vision_Point_4 = point_from_dict(Glas_Platelet_Vision_Point_4_dict)+ Glas_Platelet_Spawn 
Glas_Platelet_Vision_Point_4_2 = add_Points_3(point_from_dict_2(Glas_Platelet_Vision_Point_4_dict), Glas_Platelet_Spawn2)

UFC_Paper_Vision_Point_1_dict = {
                                            "position": {
                                                "x": -7.86135867663709e-08,
                                                "y": -7.001522873804288e-07,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }

UFC_Paper_Vision_Point_1 = point_from_dict(UFC_Paper_Vision_Point_1_dict) + UFC_Spawn
UFC_Paper_Vision_Point_1_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_1_dict), UFC_Spawn2)

UFC_Paper_Vision_Point_1_helper_dict = {
                                            "position": {
                                                "x": 5.786104960131874e-07,
                                                "y": -8.287128007246455e-07,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
UFC_Paper_Vision_Point_1_helper = point_from_dict(UFC_Paper_Vision_Point_1_helper_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_1_helper_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_1_helper_dict), UFC_Spawn2)

UFC_Paper_Vision_Point_2_dict = {
                                            "position": {
                                                "x": -1.1175142117196187e-06,
                                                "y": -0.02000140276388171,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
UFC_Paper_Vision_Point_2 = point_from_dict(UFC_Paper_Vision_Point_2_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_2_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_2_dict), UFC_Spawn2)

UFC_Paper_Vision_Point_2_helper_dict = {
                                            "position": {
                                                "x": -8.288473119493826e-07,
                                                "y": -0.020000763627041715,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
UFC_Paper_Vision_Point_2_helper = point_from_dict(UFC_Paper_Vision_Point_2_helper_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_2_helper_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_2_helper_dict), UFC_Spawn2)

UFC_Paper_Vision_Point_3_dict = {
                                            "position": {
                                                "x": -0.029998581950640986,
                                                "y": -0.019998508787187525,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
UFC_Paper_Vision_Point_3 = point_from_dict(UFC_Paper_Vision_Point_3_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_3_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_3_dict), UFC_Spawn2) 

UFC_Paper_Vision_Point_3_helper_dict = {
                                            "position": {
                                                "x": -0.029999292706559162,
                                                "y": -0.01999835106128331,
                                                "z": 0.0
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }
UFC_Paper_Vision_Point_3_helper = point_from_dict(UFC_Paper_Vision_Point_3_helper_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_3_helper_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_3_helper_dict), UFC_Spawn2)

UFC_Paper_Vision_Point_4_dict = {
                                            "position": {
                                                "x": -0.029998501083740137,
                                                "y": 0.0010013505956337572,
                                                "z": 0.002
                                            },
                                            "orientation": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0,
                                                "w": 1.0
                                            }
                                        }

UFC_Paper_Vision_Point_4 = point_from_dict(UFC_Paper_Vision_Point_4_dict)+ UFC_Spawn
UFC_Paper_Vision_Point_4_2 = add_Points_3(point_from_dict_2(UFC_Paper_Vision_Point_4_dict), UFC_Spawn2)


UFC_Center_Point = sp.Point3D(((UFC_Paper_Vision_Point_2_helper.x+UFC_Paper_Vision_Point_3_helper.x)/2),
                              ((UFC_Paper_Vision_Point_2_helper.y+UFC_Paper_Vision_Point_3_helper.y)/2), 
                              ((UFC_Paper_Vision_Point_3_helper.z+UFC_Paper_Vision_Point_3_helper.z)/2), evaluate=False)


UFC_Center_Point_2 = CGAL_K.Point_3(((UFC_Paper_Vision_Point_2_helper_2.x()+UFC_Paper_Vision_Point_3_helper_2.x())/2),
                                    (UFC_Paper_Vision_Point_2_helper_2.y()+UFC_Paper_Vision_Point_3_helper_2.y())/2, 
                                    (UFC_Paper_Vision_Point_2_helper_2.z()+UFC_Paper_Vision_Point_3_helper_2.z())/2)

Glas_Platelet_Center_Point = sp.Point3D(((Glas_Platelet_Vision_Point_2_helper.x+Glas_Platelet_Vision_Point_3_helper.x)/2),
                                        ((Glas_Platelet_Vision_Point_2_helper.y+Glas_Platelet_Vision_Point_3_helper.y)/2), 
                                        ((Glas_Platelet_Vision_Point_2_helper.z+Glas_Platelet_Vision_Point_3_helper.z)/2),  evaluate=False)

Glas_Platelet_Center_Point_2 = CGAL_K.Point_3(((Glas_Platelet_Vision_Point_2_helper_2.x()+Glas_Platelet_Vision_Point_3_helper_2.x())/2),
                                              ((Glas_Platelet_Vision_Point_2_helper_2.y()+Glas_Platelet_Vision_Point_3_helper_2.y())/2),
                                              ((Glas_Platelet_Vision_Point_2_helper_2.z()+Glas_Platelet_Vision_Point_3_helper_2.z())/2))


UFC_Paper_Alignment_Plane_1 = sp.Plane(     UFC_Paper_Vision_Point_2_helper,         UFC_Paper_Vision_Point_2,       UFC_Paper_Vision_Point_3)
Glas_Platelet_Alignment_Plane_1 = sp.Plane( Glas_Platelet_Vision_Point_2_helper,     Glas_Platelet_Vision_Point_2,   Glas_Platelet_Vision_Point_3)

UFC_Paper_Alignment_Plane_1_2 = CGAL_K.Plane_3(UFC_Paper_Vision_Point_2_helper_2, UFC_Paper_Vision_Point_2_2, UFC_Paper_Vision_Point_3_2)
Glas_Platelet_Alignment_Plane_1_2 = CGAL_K.Plane_3(Glas_Platelet_Vision_Point_2_helper_2, Glas_Platelet_Vision_Point_2_2, Glas_Platelet_Vision_Point_3_2)


UFC_Paper_Alignment_Plane_2 = sp.Plane(     UFC_Paper_Vision_Point_3_helper,       UFC_Paper_Vision_Point_2_helper,         UFC_Paper_Vision_Point_1_helper)
Glas_Platelet_Alignment_Plane_2 = sp.Plane( Glas_Platelet_Vision_Point_2, Glas_Platelet_Vision_Point_3,          Glas_Platelet_Vision_Point_1)

UFC_Paper_Alignment_Plane_2_2 = CGAL_K.Plane_3(UFC_Paper_Vision_Point_3_helper_2, UFC_Paper_Vision_Point_2_helper_2, UFC_Paper_Vision_Point_1_helper_2)
Glas_Platelet_Alignment_Plane_2_2 = CGAL_K.Plane_3(Glas_Platelet_Vision_Point_2_2, Glas_Platelet_Vision_Point_3_2, Glas_Platelet_Vision_Point_1_2)


UFC_Paper_Axis = sp.Line3D(     UFC_Paper_Vision_Point_2_helper,            UFC_Paper_Vision_Point_3_helper)
Glas_Platelet_Axis = sp.Line3D( Glas_Platelet_Vision_Point_2_helper,        Glas_Platelet_Vision_Point_3_helper)

UFC_Paper_Axis_2 = CGAL_K.Line_3(UFC_Paper_Vision_Point_2_helper_2, UFC_Paper_Vision_Point_3_helper_2)
Glas_Platelet_Axis_2 = CGAL_K.Line_3(Glas_Platelet_Vision_Point_2_helper_2, Glas_Platelet_Vision_Point_3_helper_2)

UFC_Middle_Pane = sp.Plane(UFC_Center_Point, normal_vector = UFC_Paper_Axis.direction)

UFC_Middle_Pane_2 = CGAL_K.Plane_3(UFC_Center_Point_2, UFC_Paper_Axis_2.to_vector())
UFC_test_Plane = CGAL_K.Plane_3(UFC_Paper_Vision_Point_2_helper_2, UFC_Paper_Vision_Point_1_2, UFC_Paper_Vision_Point_1_helper_2)

Glas_Platelet_Middle_Pane = sp.Plane(Glas_Platelet_Center_Point, normal_vector = Glas_Platelet_Axis.direction)
Glas_Platelet_Middle_Pane_2 = CGAL_K.Plane_3(Glas_Platelet_Center_Point_2, Glas_Platelet_Axis_2.to_vector())

# Print all planes
#print(f"UFC_Paper_Alignment_Plane_1: {UFC_Paper_Alignment_Plane_1}")
#print(f"UFC_Paper_Alignment_Plane_2: {UFC_Paper_Alignment_Plane_2}")
#print(f"UFC_Middle_Pane: {UFC_Middle_Pane}")

#print(f"Glas_Platelet_Alignment_Plane_1: {Glas_Platelet_Alignment_Plane_1}")
#print(f"Glas_Platelet_Alignment_Plane_2: {Glas_Platelet_Alignment_Plane_2}")

#UFC_Paper_Alignment_Point = get_point_of_plane_intersection(UFC_Paper_Alignment_Plane_1, UFC_Paper_Alignment_Plane_2, UFC_Middle_Pane)
UFC_Paper_Alignment_Point = get_point_of_plane_intersection(UFC_Middle_Pane, UFC_Paper_Alignment_Plane_2, UFC_Paper_Alignment_Plane_1 )

Glas_Platelet_Alignment_Point = get_point_of_plane_intersection(Glas_Platelet_Alignment_Plane_1, Glas_Platelet_Alignment_Plane_2, Glas_Platelet_Middle_Pane)

print(f"Glas_Alignment Point: {Glas_Platelet_Alignment_Point.evalf()}")
print(f"UFC_Alignment Point: {UFC_Paper_Alignment_Point.evalf()}")

print(f"Diff Center Glas {(Glas_Platelet_Center_Point.evalf()-Glas_Platelet_Alignment_Point.evalf())*1e6}")
print(f"Diff Center UFC {(UFC_Center_Point.evalf()-UFC_Paper_Alignment_Point.evalf())*1e6}")

UFC_Target = sp.Point3D(0.517499874535133, 0.37847341143272, 1.27463888543338, evaluate=False)
Glas_Platelet_Target = sp.Point3D(0.85999540737248, 0.502189263868329, 1.75480317063825, evaluate=False)

#print(f"Diff UFC: {(UFC_Paper_Alignment_Point.evalf()-UFC_Target.evalf())*1e6}")

#print(f"Diff Glas: {(Glas_Platelet_Alignment_Point.evalf()-Glas_Platelet_Target.evalf())*1e6}")

Glas_Platelet_Alignment_Point_2 = get_point_of_plane_intersection_2(Glas_Platelet_Alignment_Plane_1_2, Glas_Platelet_Alignment_Plane_2_2, Glas_Platelet_Middle_Pane_2)
#UFC_Paper_Alignment_Point_2 = get_point_of_plane_intersection_2(UFC_Paper_Alignment_Plane_1_2, UFC_Paper_Alignment_Plane_2_2, UFC_Middle_Pane_2)

#print(f"Glas_Alignment Point_2: {Glas_Platelet_Alignment_Point_2}")
#print(f"UFC_Alignment Point_2: {UFC_Paper_Alignment_Point_2}")