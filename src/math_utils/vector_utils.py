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

    def convert(self, convert_coordinates=Coordinate.CARTESIAN):
        return convert_vector(self.tuple, Coordinate.CARTESIAN, convert_coordinates)
    
    def length(self):
        return math.sqrt(self.tuple[0] ** 2 + self.tuple[1] ** 2 + self.tuple[2] ** 2)
    
    def distance(self, vector):
        return (self - vector).length()
    
    def angle(self, vector):
        return acosd(self * vector / (self.length() * vector.length()))
    
    def unit_vector(self):
        return self / self.length()

    def __add__(self, value):
        if type(value) is TupleRotator3:
            a, b, c = value.value()
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
        
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)

        vec = TupleVector3((self.tuple[0] + tup[0],
                            self.tuple[1] + tup[1],
                            self.tuple[2] + tup[2]))
        vec.coordinates = self.coordinates
        return vec
    
    
    def __radd__(self, tuple_vector):
        return self.__add__(tuple_vector)

    def __sub__(self, value):
        if type(value) is TupleRotator3:
            vec = self + (-value)
            return vec
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)
        else:
            raise TypeError(f"bad type for vector subtraction: {type(value)}")

        vec = TupleVector3((self.tuple[0] - tup[0],
                            self.tuple[1] - tup[1],
                            self.tuple[2] - tup[2]))
        vec.coordinates = self.coordinates
        return vec

    def __rsub__(self, value):
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)

        vec = TupleVector3((tup[0] - self.tuple[0],
                            tup[1] - self.tuple[1],
                            tup[2] - self.tuple[2]))
        vec.coordinates = self.coordinates
        return vec
    
    def __mul__(self, value):
        if type(value) is int or type(value) is float:
            tup = (value, value, value)

            vec = TupleVector3((self.tuple[0] * tup[0],
                                self.tuple[1] * tup[1],
                                self.tuple[2] * tup[2]))
            vec.coordinates = self.coordinates
            return vec
        
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value

        return (self.tuple[0] * tup[0] +
                self.tuple[1] * tup[1] +
                self.tuple[2] * tup[2])
        
    def __rmul__(self, value):
        return self.__mul__(value)
    
    def __truediv__(self, value):
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)

        vec = TupleVector3((self.tuple[0] / tup[0],
                            self.tuple[1] / tup[1],
                            self.tuple[2] / tup[2]))
        vec.coordinates = self.coordinates
        return vec
    
    def __rtruediv__(self, value):
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)

        vec = TupleVector3((tup[0] / self.tuple[0] if self.tuple[0] != 0 else 0,
                            tup[1] / self.tuple[1] if self.tuple[1] != 0 else 0,
                            tup[2] / self.tuple[2] if self.tuple[2] != 0 else 0))
        vec.coordinates = self.coordinates
        return vec    

    def __lt__(self, value):
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() < len
    
    def __gt__(self, value):
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() > len
    
    def __eq__(self, value):
        if type(value) == TupleVector3:
            tup = value.tuple
        elif type(value) == tuple:
            tup = value
        elif type(value) == int or type(value) == float:
            tup = (value, value, value)
        
        return (self.tuple[0] == tup[0] and
                self.tuple[1] == tup[1] and
                self.tuple[2] == tup[2] )
    
    def __ne__(self, value):
        if type(value) == TupleVector3:
            tup = value.tuple
        elif type(value) == tuple:
            tup = value
        elif type(value) == int or type(value) == float:
            tup = (value, value, value)
        
        return (self.tuple[0] != tup[0] and
                self.tuple[1] != tup[1] and
                self.tuple[2] != tup[2] )
    
    def __le__(self, value):
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() <= len
    
    def __ge__(self, value):
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() >= len

    def __neg__(self):
        vec = TupleVector3((-self.tuple[0], -self.tuple[1], -self.tuple[2]))
        vec.coordinates = self.coordinates
        return vec

    def __str__(self):
        pref_x = "x" if self.coordinates == Coordinate.CARTESIAN else "r"
        pref_y = "y" if self.coordinates == Coordinate.CARTESIAN else "phi" if self.coordinates == Coordinate.CYLINDRICAL else "theta"
        pref_z = "z" if self.coordinates != Coordinate.SPHERICAL else "alpha"
        value = self.convert(self.coordinates)
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
    
    def __add__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((self.tuple[0] + tup[0],
                              self.tuple[1] + tup[1],
                              self.tuple[2] + tup[2]))
        
    def __radd__(self, value):
        return self.__add__(value)
    
    def __sub__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((self.tuple[0] - tup[0],
                              self.tuple[1] - tup[1],
                              self.tuple[2] - tup[2]))
        
    def __rsub__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((tup[0] - self.tuple[0],
                              tup[1] - self.tuple[1],
                              tup[2] - self.tuple[2]))
    
    def __mul__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((self.tuple[0] * tup[0],
                              self.tuple[1] * tup[1],
                              self.tuple[2] * tup[2]))
        
    def __rmul__(self, value):
        return self.__mul__(value)
    
    def __div__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((self.tuple[0] / tup[0],
                              self.tuple[1] / tup[1],
                              self.tuple[2] / tup[2]))

    def __rdiv__(self, value):
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((tup[0] / self.tuple[0] if self.tuple[0] != 0 else 0,
                              tup[1] / self.tuple[1] if self.tuple[1] != 0 else 0,
                              tup[2] / self.tuple[2] if self.tuple[1] != 0 else 0))


    def __neg__(self):
        rot = TupleRotator3((-self.tuple[0], -self.tuple[1], -self.tuple[2]))
        return rot

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
    return (math.sqrt(x ** 2 + y ** 2), atan2d(y, x), z)

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
    test_vector()
    test_rotator()