#!/usr/bin/env python

import math
from geometry_msgs.msg import Vector3
from player.msg import *
from math_utils.math_function_utils import *
from enum import Enum
from testing.testing import *
import random
import numpy as np

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
        self.tuple = np.array(convert_vector(value, coordinates, Coordinate.CARTESIAN))
        self.coordinates = coordinates

    def __get_tup(self, value):
        if type(value) is TupleVector3:
            tup = np.array(value.tuple)
        elif type(value) is tuple:
            tup = np.array(value)
        elif type(value) is int or type(value) is float:
            tup = np.array((value, value, value))
        else:
            raise TypeError(f"Bad type for vector operation: {type(value)}")
        return tup
    
    def __get_len(self, value):
        if type(value) is TupleVector3:
            len = value.length()
        elif type(value) is tuple:
            len = math.sqrt(np.sum(np.power(value, np.array((2, 2, 2)))))
        elif type(value) is int or type(value) is float:
            len = value
        return len

    def value(self):
        '''Returns the cartesian (x, y, z) tuple'''
        return tuple(self.tuple)
    
    def value_rounded(self, ndigits):
        '''Returns the cartesian (x, y, z) tuple with values rounded to n digits.
        This function is mainly used for testing purposes'''
        return (round(self.tuple[0], ndigits),
                round(self.tuple[1], ndigits),
                round(self.tuple[2], ndigits))

    def convert(self, convert_coordinates=Coordinate.CARTESIAN):
        '''Returns a tuple of the vector in the given coordinate system'''
        return convert_vector(self.tuple, Coordinate.CARTESIAN, convert_coordinates)
    
    def length(self):
        '''Returns the length of the vector'''
        return math.sqrt(np.sum(self.tuple ** 2))

    def length_xy(self):
        '''Returns the length of the vector in the xy plane'''
        return math.sqrt(np.sum(np.power(self.tuple[0:2], 2)))
    
    def distance(self, vector):
        '''Returns the distance between the vector and the given vector'''
        return (self - vector).length()
    
    def distance_xy(self, vector):
        '''Returns the distance between the vector and the given vector in the xy plane'''
        return (self - vector).length_xy()

    def angle(self, vector):
        '''Returns the angle between the vector and the given vector'''
        l = self.length() * vector.length()
        return acosd(self.dot(vector) / l) if l != 0 else 0
    
    def angle_xy(self, vector):
        '''Returns the angle between the vector and the given vector in the xy plane'''
        a1 = self.convert(Coordinate.CYLINDRICAL)[1]
        a2 = vector.convert(Coordinate.CYLINDRICAL)[1]
        ang = a1 - a2
        if ang > 180:
            ang -= 360
        elif ang < -180:
            ang += 360
        return ang
    
    def unit_vector(self):
        '''Returns the unit vector of the vector'''
        return self / self.length()

    def approx(self, vector, tolerance):
        '''Returns true if the distance between the vector and the given vector is smaller than the tolerance'''
        return self.distance(vector) <= tolerance

    def dot(self, value):
        '''Returns the dot product of the vector and the given vector.
        Value can be a TupleVector3 or a (x, y, z) tuple'''
        tup = self.__get_tup(value)
        return np.dot(self.tuple, tup)
    
    def cross(self, value):
        '''Returns the cross product of the vector and the given vector.
        Value can be a TupleVector3 or a (x, y, z) tuple'''
        tup = self.__get_tup(value)
        vec = TupleVector3(np.cross(self.tuple, tup))
        vec.coordinates = self.coordinates
        return vec

    def __add__(self, value):
        '''Element-wise addition.
        Value can be TupleVector3, tuple or int/float.
        If a TupleRotator3 is given, the vector is rotated by the rotator.'''
        if type(value) is TupleRotator3:
            ca, cb, cc = np.cos(value.tuple * math.pi / 180)
            sa, sb, sc = np.sin(value.tuple * math.pi / 180)
            rot_matrix = np.array([(ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc),
                                   (sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc),
                                   (-sb, cb * sc, cb * cc)])
            vec = TupleVector3(np.matmul(rot_matrix, self.tuple))
            vec.coordinates = self.coordinates
            return vec
        
        tup = self.__get_tup(value)

        vec = TupleVector3(np.add(self.tuple, tup))
        vec.coordinates = self.coordinates
        return vec
    
    
    def __radd__(self, tuple_vector):
        return self.__add__(tuple_vector)

    def __sub__(self, value):
        '''Element-wise subtraction.
        Value can be TupleVector3, tuple or int/float.
        If a TupleRotator3 is given, the vector is rotated by the rotator in the reverse direction.'''
        if type(value) is TupleRotator3:
            ca, cb, cc = np.cos(value.tuple * math.pi / 180)
            sa, sb, sc = np.sin(value.tuple * math.pi / 180)
            inv_rot_matrix = np.array([(ca * cb, sa * cb, -sb),
                              (ca * sb * sc - sa * cc, sa * sb * sc + ca * cc, cb * sc),
                              (ca * sb * cc + sa * sc, sa * sb * cc - ca * sc, cb * cc)])
            vec = TupleVector3(np.matmul(inv_rot_matrix, self.tuple))
            vec.coordinates = self.coordinates
            return vec
        
        tup = self.__get_tup(value)
        vec = TupleVector3(self.tuple - tup)
        vec.coordinates = self.coordinates
        return vec

    def __rsub__(self, value):
        tup = self.__get_tup(value)
        vec = TupleVector3(tup - self.tuple)
        vec.coordinates = self.coordinates
        return vec
    
    def __mul__(self, value):
        '''Element-wise multiplication.
        Value can be TupleVector3, tuple or int/float.'''
        tup = self.__get_tup(value)
        vec = TupleVector3(self.tuple * tup)
        vec.coordinates = self.coordinates
        return vec
        
    def __rmul__(self, value):
        return self.__mul__(value)
    
    def __truediv__(self, value):
        '''Element-wise safe division.
        Value can be TupleVector3, tuple or int/float.'''
        tup = self.__get_tup(value)
        safediv = np.frompyfunc(safe_div, 2, 1)
        vec = TupleVector3(safediv(self.tuple, tup))
        vec.coordinates = self.coordinates
        return vec
    
    def __rtruediv__(self, value):
        '''Element-wise division.
        Value can be TupleVector3, tuple or int/float.
        Sets the coordinate value to 0 if the divisor is 0.'''
        tup = self.__get_tup(value)
        safediv = np.frompyfunc(safe_div, 2, 1)
        vec = TupleVector3(safediv(tup, self.tuple))
        vec.coordinates = self.coordinates
        return vec    

    def __lt__(self, value):
        '''Compares the lengths of the vector and the given value.
        Value can be TupleVector3, tuple or int/float.'''
        len = self.__get_len(value)
        return self.length() < len
    
    def __gt__(self, value):
        '''Compares the lengths of the vector and the given value.
        Value can be TupleVector3, tuple or int/float.'''
        len = self.__get_len(value)
        return self.length() > len
    
    def __eq__(self, value):
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
        tup = self.__get_tup(value)
        
        return np.array_equal(self.tuple, tup)
    
    def __ne__(self, value):
        '''Element-wise comparison.
        Value can be TupleVector3, tuple or int/float.'''
        tup = self.__get_tup(value)
        
        return not np.array_equal(self.tuple, tup)
    
    def __le__(self, value):
        '''Comparison by length.
        Value can be TupleVector3, tuple or int/float.'''
        len = self.__get_len(value)
        return self.length() <= len
    
    def __ge__(self, value):
        '''Comparison by length.
        Value can be TupleVector3, tuple or int/float.'''
        len = self.__get_len(value)
        return self.length() >= len

    def __neg__(self):
        '''Element-wise negation.'''
        vec = TupleVector3(self.tuple * -1)
        vec.coordinates = self.coordinates
        return vec

    def __getitem__(self, index):
        return self.tuple[index]
    
    def __setitem__(self, index, value):
        self.tuple[index] = value

    def __str__(self):
        '''Converts the vector into a string based on the coordinates that have been given on initialization.'''
        pref_x = "x" if self.coordinates == Coordinate.CARTESIAN else "r"
        pref_y = "y" if self.coordinates == Coordinate.CARTESIAN else "phi" if self.coordinates == Coordinate.CYLINDRICAL else "theta"
        pref_z = "z" if self.coordinates != Coordinate.SPHERICAL else "alpha"
        value = self.convert(self.coordinates)
        return f"{chr(10)}" + \
               f"{pref_x:>10} {value[0]:>6.2f}{chr(10)}" + \
               f"{pref_y:>10} {value[1]:>6.2f}{chr(10)}" + \
               f"{pref_z:>10} {value[2]:>6.2f}"

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
    
    @classmethod
    def random(cls, length=10):
        return cls(np.random.uniform(-length, length, 3))
    
    @classmethod
    def random_xy(cls, length=10):
        return cls(((random.random() - 0.5) * 2 * length, (random.random() - 0.5) * 2 *  length, 0))

class TupleRotator3:
    '''This class stores a tuple of 3 rotation values (yaw, pitch, roll) and overrides arithmetic functions to handle those tuples.'''
    def __init__(self, value=(0, 0, 0)):
        self.tuple = np.array(value)

    def __get_tup(self, value):
        if type(value) is TupleRotator3:
            tup = np.array(value.tuple)
        elif type(value) is tuple:
            tup = np.array(value)
        elif type(value) is float or type(value) is int:
            tup = np.array((value, value, value))
        return tup

    def value(self):
        '''Returns the rotator (yaw, pitch, roll) tuple'''
        return tuple(self.tuple)
    
    def value_rounded(self, ndigits):
        '''Returns the rotator (yaw, pitch, roll) tuple with values rounded to n digits.
        This function is mainly used for testing purposes'''
        return (round(self.tuple[0], ndigits),
                round(self.tuple[1], ndigits),
                round(self.tuple[2], ndigits))

    def __str__(self):
        return f"{chr(10)}" + \
               f"{'yaw':>10} {self.tuple[0]:>6.2f}{chr(10)}" + \
               f"{'pitch':>10} {self.tuple[1]:>6.2f}{chr(10)}" + \
               f"{'roll':>10} {self.tuple[2]:>6.2f}"
    
    def __add__(self, value):
        '''Element-wise addition.
        Value can be TupleRotator3, tuple or int/float.'''
        tup = self.__get_tup(value)
        return TupleRotator3(self.tuple + tup)
        
    def __radd__(self, value):
        return self.__add__(value)
    
    def __sub__(self, value):
        '''Element-wise subtraction.
        Value can be TupleRotator3, tuple or int/float.'''
        tup = self.__get_tup(value)

        return TupleRotator3(self.tuple - tup)
        
    def __rsub__(self, value):
        tup = self.__get_tup(value)

        return TupleRotator3(tup - self.tuple)
    
    def __mul__(self, value):
        '''Element-wise multiplication.
        Value can be TupleRotator3, tuple or int/float.'''
        tup = self.__get_tup(value)

        return TupleRotator3(self.tuple * tup)
        
    def __rmul__(self, value):
        return self.__mul__(value)
    
    def __truediv__(self, value):
        '''Element-wise division.
        Value can be TupleRotator3, tuple or int/float.'''
        tup = self.__get_tup(value)
        safediv = np.frompyfunc(safe_div, 2, 1)
        return TupleRotator3(safediv(self.tuple, tup))

    def __rtruediv__(self, value):
        '''Element-wise division.
        Value can be TupleRotator3, tuple or int/float.
        Sets coordinate value to 0 if the divisor is 0.'''
        tup = self.__get_tup(value)
        safediv = np.frompyfunc(safe_div, 2, 1)
        return TupleRotator3(safediv(tup, self.tuple))


    def __neg__(self):
        '''Element-wise negation.'''
        rot = TupleRotator3(self.tuple * -1)
        return rot

    def __getitem__(self, index):
        return self.tuple[index]
    
    def __setitem__(self, index, value):    
        self.tuple[index] = value

    @classmethod
    def random(cls, angle=180):
        return cls(np.random.uniform(-angle, angle, 3))

    @classmethod
    def random_xy(cls, angle=180):
        return cls(((random.random() - 0.5) * 2 * angle, 0, 0))


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
    test(v.value_rounded(5), (1, 0, 0))
    test(v.convert(Coordinate.SPHERICAL), (1, 90, 0))
    print(v)
    v2 = TupleVector3((1, 90, 0), Coordinate.SPHERICAL)
    print(v + v2)
    print(v * 2)

    v[0] = 2
    test(v.value_rounded(5), (2, 0, 0))


def test_rotator():
    print("test rotator")
    r = TupleRotator3((90, 90, 0))
    print(r)
    print(r.value())
    r2 = TupleRotator3((20, 20, 0))
    test((r + r2).value(), (110, 110, 0))
    test((r - r2).value(), (70, 70, 0))

    v = TupleVector3((1, 0, 0))
    print(v)
    test((v.convert(Coordinate.SPHERICAL)), (1, 90, 0))
    test((v + r).value_rounded(5), (0, 0, -1))
    test((v - r).value_rounded(5), (0, -1, 0))

    test((v - r + r).value_rounded(5), v.value())
    test((v + r - r).value_rounded(5), v.value())

if __name__ == "__main__":
    test_vector()
    test_rotator()