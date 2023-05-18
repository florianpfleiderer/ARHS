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
    '''This class stores a cartesian 3D vector tuple and overrides arithmetic functions to handle those tuples.
    Other useful functions like length and distance are also implemented.
    '''
    def __init__(self, value=(0, 0, 0), coordinates=Coordinate.CARTESIAN):
        '''Value: tuple of 3 values, can be given in cartesian (x, y, z), cylindrical (r, phi, z) or spherical (r, theta, alpha) coordinates.
        Coordinates: Coordinate enum, defines the coordinate system of the given value, which is converted to cartesian coordinates.
        The coordinates variable is stored in the class and used for the __str__ function.'''
        self.tuple = convert_vector(value, coordinates, Coordinate.CARTESIAN)
        self.coordinates = coordinates

    def value(self):
        '''Returns the cartesian (x, y, z) tuple'''
        return self.tuple

    def convert(self, convert_coordinates=Coordinate.CARTESIAN):
        '''Returns a tuple of the vector in the given coordinate system'''
        return convert_vector(self.tuple, Coordinate.CARTESIAN, convert_coordinates)
    
    def length(self):
        '''Returns the length of the vector'''
        return math.sqrt(self.tuple[0] ** 2 + self.tuple[1] ** 2 + self.tuple[2] ** 2)
    
    def distance(self, vector):
        '''Returns the distance between the vector and the given vector'''
        return (self - vector).length()
    
    def angle(self, vector):
        '''Returns the angle between the vector and the given vector'''
        return acosd(self.dot(vector) / (self.length() * vector.length()))
    
    def unit_vector(self):
        '''Returns the unit vector of the vector'''
        return self / self.length()

    def approx(self, vector, tolerance):
        '''Returns true if the distance between the vector and the given vector is smaller than the tolerance'''
        return self.distance(vector) <= tolerance

    def dot(self, value):
        '''Returns the dot product of the vector and the given vector.
        Value can be a TupleVector3 or a (x, y, z) tuple'''
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value

        return (self.tuple[0] * tup[0] +
                self.tuple[1] * tup[1] +
                self.tuple[2] * tup[2])

    def __add__(self, value):
        '''Element-wise addition.
        Value can be TupleVector3, tuple or int/float.
        If a TupleRotator3 is given, the vector is rotated by the rotator.'''
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
        '''Element-wise subtraction.
        Value can be TupleVector3, tuple or int/float.
        If a TupleRotator3 is given, the vector is rotated by the rotator in the reverse direction.'''
        if type(value) is TupleRotator3:
            a, b, c = value.value()
            ca = cosd(a)
            sa = sind(a)
            cb = cosd(b)
            sb = sind(b)
            cc = cosd(c)
            sc = sind(c)            
            inv_rot_matrix = ((ca * cb, sa * cb, -sb),
                              (ca * sb * sc - sa * cc, sa * sb * sc + ca * cc, cb * sc),
                              (ca * sb * cc + sa * sc, sa * sb * cc - ca * sc, cb * cc))
            x = self.tuple[0] * inv_rot_matrix[0][0] + self.tuple[1] * inv_rot_matrix[0][1] + self.tuple[2] * inv_rot_matrix[0][2]
            y = self.tuple[0] * inv_rot_matrix[1][0] + self.tuple[1] * inv_rot_matrix[1][1] + self.tuple[2] * inv_rot_matrix[1][2]
            z = self.tuple[0] * inv_rot_matrix[2][0] + self.tuple[1] * inv_rot_matrix[2][1] + self.tuple[2] * inv_rot_matrix[2][2]
            vec = TupleVector3((x, y, z))
            vec.coordinates = self.coordinates
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
        '''Element-wise multiplication.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) is TupleVector3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is int or type(value) is float:
            tup = (value, value, value)

        vec = TupleVector3((self.tuple[0] * tup[0],
                            self.tuple[1] * tup[1],
                            self.tuple[2] * tup[2]))
        vec.coordinates = self.coordinates
        return vec
        
    def __rmul__(self, value):
        return self.__mul__(value)
    
    def __truediv__(self, value):
        '''Element-wise division.
        Value can be TupleVector3, tuple or int/float.'''
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
        '''Element-wise division.
        Value can be TupleVector3, tuple or int/float.
        Sets the coordinate value to 0 if the divisor is 0.'''
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
        '''Compares the lengths of the vector and the given value.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() < len
    
    def __gt__(self, value):
        '''Compares the lengths of the vector and the given value.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() > len
    
    def __eq__(self, value):
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
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
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) == TupleVector3:
            tup = value.tuple
        elif type(value) == tuple:
            tup = value
        elif type(value) == int or type(value) == float:
            tup = (value, value, value)
        
        return (self.tuple[0] != tup[0] or
                self.tuple[1] != tup[1] or
                self.tuple[2] != tup[2] )
    
    def __le__(self, value):
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() <= len
    
    def __ge__(self, value):
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
        if type(value) == TupleVector3:
            len = value.length()
        elif type(value) == tuple:
            len = math.sqrt(value[0] ** 2 + value[1] ** 2 + value[2] ** 2)
        elif type(value) == int or type(value) == float:
            len = value
        
        return self.length() >= len

    def __neg__(self):
        '''Element-wise negation.'''
        vec = TupleVector3((-self.tuple[0], -self.tuple[1], -self.tuple[2]))
        vec.coordinates = self.coordinates
        return vec

    def __str__(self):
        '''Converts the vector into a string based on the coordinates that have been given on initialization.'''
        pref_x = "x" if self.coordinates == Coordinate.CARTESIAN else "r"
        pref_y = "y" if self.coordinates == Coordinate.CARTESIAN else "phi" if self.coordinates == Coordinate.CYLINDRICAL else "theta"
        pref_z = "z" if self.coordinates != Coordinate.SPHERICAL else "alpha"
        value = self.convert(self.coordinates)
        return f"{chr(10)}" + \
               f"{pref_x: >10} {value[0]: >6.2f}{chr(10)}" + \
               f"{pref_y: >10} {value[1]: >6.2f}{chr(10)}" + \
               f"{pref_z: >10} {value[2]: >6.2f}"

    def make_vector3(self):
        '''Converts the vector into a geometry_msg Vector3'''
        return Vector3(*self.tuple)
    
    @classmethod

    def from_vector3(cls, vector3, coordinates=Coordinate.CARTESIAN):
        '''Creates a TupleVector3 from a geometry_msg Vector3.
        The display coordinates are set to the given coordinates.'''
        vec = cls((vector3.x, vector3.y, vector3.z))
        vec.coordinates = coordinates
        return vec

class TupleRotator3:
    '''This class stores a tuple of 3 rotation values (yaw, pitch, roll) and overrides arithmetic functions to handle those tuples.'''
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
        '''Element-wise addition.
        Value can be TupleRotator3, tuple or int/float.'''
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
        '''Element-wise subtraction.
        Value can be TupleRotator3, tuple or int/float.'''
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
        '''Element-wise multiplication.
        Value can be TupleRotator3, tuple or int/float.'''
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
    
    def __truediv__(self, value):
        '''Element-wise division.
        Value can be TupleRotator3, tuple or int/float.'''
        if type(value) is TupleRotator3:
            tup = value.tuple
        elif type(value) is tuple:
            tup = value
        elif type(value) is float or type(value) is int:
            tup = (value, value, value)

        return TupleRotator3((self.tuple[0] / tup[0],
                              self.tuple[1] / tup[1],
                              self.tuple[2] / tup[2]))

    def __rtruediv__(self, value):
        '''Element-wise division.
        Value can be TupleRotator3, tuple or int/float.
        Sets coordinate value to 0 if the divisor is 0.'''
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
        '''Element-wise negation.'''
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
    v = TupleVector3((1, 2, 5))
    v2 = TupleVector3((2, 3, -1))
    test(v < v2, False)
    test(v > v2, True)
    test(v == v2, False)
    test(v != v2, True)
    test(round(v.length(), 2), 5.48)
    test(round(v.distance(v2), 2), 6.16)
    test(round(v.angle(v2), 2), 81.58)
    v3 = TupleVector3((1, 2, 4))
    test(v.approx(v3, 0.5), False)
    test(v.approx(v3, 2), True)

    test((v + v2).value(), (3, 5, 4))
    test((v + (1, 2, 3)).value(), (2, 4, 8))
    test((v + 3).value(), (4, 5, 8))

    test((v - v2).value(), (-1, -1, 6))
    test((v - (1, 2, 3)).value(), (0, 0, 2))
    test((v - 3).value(), (-2, -1, 2))

    test((v * v2).value(), (2, 6, -5))
    test((v * (1, 2, 3)).value(), (1, 4, 15))
    test((v * 3).value(), (3, 6, 15))
    test((v * 0).value(), (0, 0, 0))

    test(v.dot(v2), 3)
    test(v.dot((1, 2, 3)), 20)

    test((v / v2).value(), (0.5, 2/3, -5))
    test((v / (1, 2, 3)).value(), (1, 1, 5/3))
    test((v / 3).value(), (1/3, 2/3, 5/3))

    print("-- Test Conversion --")
    v = (1, 0, 0)
    print(convert_vector(v, Coordinate.CARTESIAN, Coordinate.SPHERICAL))
    v = (1, 90, 0)
    print(convert_vector(v, Coordinate.SPHERICAL, Coordinate.CARTESIAN))

    print("--")
    v = TupleVector3((1, 90, 0), Coordinate.SPHERICAL)
    test(v.tuple, (1, 0, 0))
    test(v.value(), (1, 0, 0))
    test(v.convert(Coordinate.SPHERICAL), (1, 90, 0))
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
    val = (v + r).value()
    test((round(val[0]), round(val[1]), round(val[2])), (0, 0, -1))
    val = (v - r).value()
    test((round(val[0]), round(val[1]), round(val[2])), (0, -1, 0))

    test((v - r + r).value(), v.value())
    test((v + r - r).value(), v.value())

if __name__ == "__main__":
    test_vector()
    test_rotator()