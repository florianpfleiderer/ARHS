#!/usr/bin/env python

import math
from geometry_msgs.msg import Vector3
from player.msg import *
import rospy
from math_utils.math_function_utils import *
from enum import Enum
from testing.testing import *

class Coordinate(Enum):
    CARTESIAN = 0
    CYLINDRICAL = 1
    SPHERICAL = 2

class TupleVector3:
    def __init__(self, value=(0, 0, 0), coordinates=Coordinate.CARTESIAN):
        self.tuple = convert_vector(value, coordinates, Coordinate.CARTESIAN)
        self.coordinates = coordinates

    def value(self):
        return self.tuple
    
    def convert(self, coordinates=None):
        return convert_vector(self.tuple, Coordinate.CARTESIAN, self.coordinates if coordinates is None else coordinates)

    def __mul__(self, factor):
        if type(factor) is TupleVector3:
            vec = TupleVector3((self.tuple[0] * factor.tuple[0],
                                self.tuple[1] * factor.tuple[1],
                                self.tuple[2] * factor.tuple[2]))
            vec.coordinates = self.coordinates
            return vec
        
        else:
            vec = TupleVector3((self.tuple[0] * factor,
                                self.tuple[1] * factor,
                                self.tuple[2] * factor))
            vec.coordinates = self.coordinates
            return vec
        
    def __rmul__(self, factor):
        return self.__mul__(factor)
    
    def __truediv__(self, factor):
        if type(factor) is TupleVector3:
            vec = TupleVector3((self.tuple[0] / factor.tuple[0],
                                self.tuple[1] / factor.tuple[1],
                                self.tuple[2] / factor.tuple[2]))
            vec.coordinates = self.coordinates
            return vec
        
        else:
            vec = TupleVector3((self.tuple[0] / factor,
                                self.tuple[1] / factor,
                                self.tuple[2] / factor))
            vec.coordinates = self.coordinates
            return vec
        
    def __rtruediv__(self, factor):
        return self.__truediv__(factor)
    
    def __floordiv__(self, factor):
        if type(factor) is TupleVector3:
            vec = TupleVector3((self.tuple[0] // factor.tuple[0],
                                self.tuple[1] // factor.tuple[1],
                                self.tuple[2] // factor.tuple[2]))
            vec.coordinates = self.coordinates
            return vec
        
        else:
            vec = TupleVector3((self.tuple[0] // factor,
                                self.tuple[1] // factor,
                                self.tuple[2] // factor))
            vec.coordinates = self.coordinates
            return vec
        
    def __rfloordiv__(self, factor):
        return self.__floordiv__(factor)

    def __add__(self, summand):
        if type(summand) is TupleVector3:
            vec = TupleVector3((self.tuple[0] + summand.tuple[0],
                                self.tuple[1] + summand.tuple[1],
                                self.tuple[2] + summand.tuple[2]))
            vec.coordinates = self.coordinates
        elif type(summand) is TupleRotator3:
            a, b, c = summand.value()
            ca = cosd(a)
            sa = sind(a)
            cb = cosd(b)
            sb = sind(b)
            cc = cosd(c)
            sc = sind(c)
            rot_matrix = ((ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc),
                            (sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc),
                            (-sb, cb * sc, cb * cc))
            x = self.tuple[0] * rot_matrix[0][0] + self.tuple[1] * rot_matrix[0][1] + self.tuple[2] * rot_matrix[0][2]
            y = self.tuple[0] * rot_matrix[1][0] + self.tuple[1] * rot_matrix[1][1] + self.tuple[2] * rot_matrix[1][2]
            z = self.tuple[0] * rot_matrix[2][0] + self.tuple[1] * rot_matrix[2][1] + self.tuple[2] * rot_matrix[2][2]
            vec = TupleVector3((x, y, z))
            vec.coordinates = self.coordinates
        return vec
    
    
    def __radd__(self, tuple_vector):
        return self.__add__(tuple_vector)

    def __sub__(self, summand):
        if type(summand) is TupleVector3:
            vec = TupleVector3((self.tuple[0] - summand.tuple[0],
                                self.tuple[1] - summand.tuple[1],
                                self.tuple[2] - summand.tuple[2]))
            vec.coordinates = self.coordinates
        elif type(summand) is TupleRotator3:
            vec = self + summand * -1
        return vec

    def __rsub__(self, tuple_vector):
        return self.__sub__(tuple_vector)

    def __lt__(self, tuple_vector):
        return (self.tuple[0] < tuple_vector.tuple[0] and
                self.tuple[1] < tuple_vector.tuple[1] and
                self.tuple[2] < tuple_vector.tuple[2] )
    
    def __gt__(self, tuple_vector):
        return (self.tuple[0] > tuple_vector.tuple[0] and
                self.tuple[1] > tuple_vector.tuple[1] and
                self.tuple[2] > tuple_vector.tuple[2] )
    
    def __eq__(self, tuple_vector):
        return (self.tuple[0] == tuple_vector.tuple[0] and
                self.tuple[1] == tuple_vector.tuple[1] and
                self.tuple[2] == tuple_vector.tuple[2] )
    
    def __ne__(self, tuple_vector):
        return (self.tuple[0] != tuple_vector.tuple[0] or
                self.tuple[1] != tuple_vector.tuple[1] or
                self.tuple[2] != tuple_vector.tuple[2] )
    
    def __le__(self, tuple_vector):
        return (self.tuple[0] <= tuple_vector.tuple[0] and
                self.tuple[1] <= tuple_vector.tuple[1] and
                self.tuple[2] <= tuple_vector.tuple[2] )
    
    def __ge__(self, tuple_vector):
        return (self.tuple[0] >= tuple_vector.tuple[0] and
                self.tuple[1] >= tuple_vector.tuple[1] and
                self.tuple[2] >= tuple_vector.tuple[2] )
    
    def __str__(self):
        pref_x = "x" if self.coordinates == Coordinate.CARTESIAN else "r"
        pref_y = "y" if self.coordinates == Coordinate.CARTESIAN else "phi" if self.coordinates == Coordinate.CYLINDRICAL else "theta"
        pref_z = "z" if self.coordinates != Coordinate.SPHERICAL else "alpha"
        value = self.convert()
        return f"{chr(10)}" + \
               f"{pref_x: >10} {value[0]: >6.2f}{chr(10)}" + \
               f"{pref_y: >10} {value[1]: >6.2f}{chr(10)}" + \
               f"{pref_z: >10} {value[2]: >6.2f}"

    def make_vector3(self):
        return Vector3(*self.tuple)
    
    
    @classmethod
    def from_vector3(cls, vector3, coordinates=Coordinate.CARTESIAN):
        return cls((vector3.x, vector3.y, vector3.z), coordinates)

class TupleRotator3:
    def __init__(self, value=(0, 0, 0)):
        self.tuple = value

    def value(self):
        return self.tuple

    def __str__(self):
        return f"{chr(10)}" + \
               f"{'yaw': >10} {self.tuple[0]: >6.2f}{chr(10)}" + \
               f"{'pitch': >10} {self.tuple[1]: >6.2f}{chr(10)}" + \
               f"{'roll': >10} {self.tuple[2]: >6.2f}"
    
    def __add__(self, summand):
        if type(summand) is TupleRotator3:
            rot = TupleRotator3((self.tuple[0] + summand.tuple[0],
                                 self.tuple[1] + summand.tuple[1],
                                 self.tuple[2] + summand.tuple[2]))
            return rot
        elif type(summand) is tuple:
            rot = TupleRotator3((self.tuple[0] + summand[0],
                                 self.tuple[1] + summand[1],
                                 self.tuple[2] + summand[2]))
            return rot
        
    def __radd__(self, tuple_rotator):
        return self.__add__(tuple_rotator)
    
    def __sub__(self, summand):
        if type(summand) is TupleRotator3:
            rot = TupleRotator3((self.tuple[0] - summand.tuple[0],
                                 self.tuple[1] - summand.tuple[1],
                                 self.tuple[2] - summand.tuple[2]))
            return rot
        elif type(summand) is tuple:
            rot = TupleRotator3((self.tuple[0] - summand[0],
                                 self.tuple[1] - summand[1],
                                 self.tuple[2] - summand[2]))
            return rot
        
    def __rsub__(self, tuple_rotator):
        return self.__sub__(tuple_rotator)
    
    def __mul__(self, factor):
        if type(factor) is float or type(factor) is int:
            rot = TupleRotator3((self.tuple[0] * factor,
                                 self.tuple[1] * factor,
                                 self.tuple[2] * factor))
            return rot
        elif type(factor) is TupleRotator3:
            rot = TupleRotator3((self.tuple[0] * factor.tuple[0],
                                 self.tuple[1] * factor.tuple[1],
                                 self.tuple[2] * factor.tuple[2]))
            return rot
        
    def __rmul__(self, tuple_rotator):
        return self.__mul__(tuple_rotator)
    

def convert_vector(vector, from_coordinates, to_coordinates):
    from_vector = ()

    if from_coordinates == Coordinate.CARTESIAN:
        from_vector = vector
    elif from_coordinates == Coordinate.CYLINDRICAL:
        from_vector = cylindrical_to_cartesian(vector)
    elif from_coordinates == Coordinate.SPHERICAL:
        from_vector = spherical_to_cartesian(vector)

    if to_coordinates == Coordinate.CARTESIAN:
        return from_vector
    elif to_coordinates == Coordinate.CYLINDRICAL:
        return cartesian_to_cylindrical(from_vector)
    elif to_coordinates == Coordinate.SPHERICAL:
        return cartesian_to_spherical(from_vector)

def cylindrical_to_cartesian(polar_vector):
    r, theta, z = polar_vector
    return (r * cosd(theta), r * sind(theta), z)

def cartesian_to_cylindrical(cartesian_vector):
    x, y, z = cartesian_vector
    return (math.sqrt(x ** 2 + y ** 2), math.atan2(y, x), z)

def spherical_to_cartesian(spherical_vector):
    r, theta, phi = spherical_vector
    return (r * sind(theta) * cosd(phi), r * sind(theta) * sind(phi), r * cosd(theta))

def cartesian_to_spherical(cartesian_vector):
    x, y, z = cartesian_vector
    r = math.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = 0 if r == 0 else acosd(z / r)
    phi = atan2d(y, x)
    return (r, theta, phi)

# def sum_vectors(*vectors):
#     return (sum([vector[0] for vector in vectors]), sum([vector[1] for vector in vectors]), sum([vector[2] for vector in vectors]))

# def subtract_vectors(vector1, vector2):
#     return (vector1[0] - vector2[0], vector1[1] - vector2[1], vector1[2] - vector2[2])

def test_vector():
    v = TupleVector3((1, 2, 3))
    v2 = TupleVector3((2, 3, -1))
    print(v < v2)
    print(v + v2)
    print(v - v2)
    print(v * v2)
    print(v * 10.5)

    print("-- Test Conversion --")
    v = (1, 0, 0)
    print(convert_vector(v, Coordinate.CARTESIAN, Coordinate.SPHERICAL))
    v = (1, 90, 0)
    print(convert_vector(v, Coordinate.SPHERICAL, Coordinate.CARTESIAN))

    print("--")
    v = TupleVector3((1, 90, 0), Coordinate.SPHERICAL)
    print(v.tuple)
    print(v.value())
    print(v.convert())
    print(v)
    v2 = TupleVector3((1, 90, 0), Coordinate.SPHERICAL)
    print(v + v2)
    print(v * 2)


def test_rotator():
    r = TupleRotator3((90, 90, 0))
    print(r)
    print(r.value())
    r2 = TupleRotator3((20, 20, 0))
    test((r + r2).value(), (110, 110, 0))
    test((r - r2).value(), (70, 70, 0))

    v = TupleVector3((1, 0, 0))
    print(v)
    test((v.convert(Coordinate.SPHERICAL)), (1, 90, 0))
    test((v + r).value(), (0, 0, -1))
    test((v - r).value(), (0, 0, 1))

if __name__ == "__main__":
    # test_vector()
    test_rotator()