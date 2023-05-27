#!/usr/bin/env python

from math_utils.vector_utils import TupleVector3, TupleRotator3, Coordinate
from typing import List
from itertools import permutations, combinations
from testing.testing import *
from field_components.field_components import *
import random

class PointCloud:
    def __init__(self, points: List[TupleVector3], origin=None):
        assert len(points) > 0, "You must provide at least one point"
        self.points = points
        self.origin = sum(points) / len(points) if origin is None else origin
    
    def rotate(self, rotation: TupleRotator3):
        self.points = [p + rotation for p in self.points]

    def offset(self, offset: TupleVector3):
        self.points = [p + offset for p in self.points]

    def distance(self, other_point_cloud: 'PointCloud') -> TupleVector3:
        return other_point_cloud.origin - self.origin
    
    def get_triangles(self) -> List['Triangle']:
        return sorted([Triangle(tri) for tri in combinations(self.points, 3)], key=lambda tri: tri.area)

    def get_twist(self, other_point_cloud: 'PointCloud', iterations=0):
        tris = self.get_triangles()
        other_tris = other_point_cloud.get_triangles()

        for i in range(iterations if iterations > 0 else len(tris)):
            for other_tri in other_tris:
                if tris[i].matches(other_tri):
                    return tris[i].distance(other_tri), TupleRotator3((tris[i].angle_xy(other_tri), 0, 0))

        return None, None
    
    @classmethod
    def random(cls, num_points, length=10):
        points = [TupleVector3.random(length) for i in range(num_points)]
        return cls(points)

class Triangle(PointCloud):
    '''Class that represents a triangle in 3D space. Points are oriented counterclockwise around the z axis.'''
    def __init__(self, points: List[TupleVector3]):
        assert len(points) == 3
        super().__init__(points)
        self.points = sorted(self.points, key=lambda p: (p-self.origin).convert(Coordinate.CYLINDRICAL)[1])
        A = self.points[0]
        B = self.points[1]
        C = self.points[2]
        self.a = C - B
        self.b = C - A
        self.c = B - A
        self.area = 0.5 * self.c.cross(self.b).length()
        alpha = self.c.angle(self.b)
        beta = self.a.angle(self.c)
        gamma = self.b.angle(self.a)
        self.angles = [alpha, beta, gamma]
    
    def matches(self, other_triangle: 'Triangle', tolerance=0.1):
        # check if areas match
        if abs(self.area - other_triangle.area) > tolerance ** 2:
            return False
        
        # check if all angles match
        angs_other = permutations(other_triangle.angles, 3)
        for perm in angs_other:
            if sum([abs(self.angles[i] - perm[i]) for i in range(3)]) / 3 > tolerance * math.pi:
                return False
        
        return True
    
    def angle_xy(self, other_triangle: 'Triangle'):
        if self.matches(other_triangle):
            max_dist = max(p1 - p2 for p1, p2 in combinations(self.points, 2))
            max_other = max(p1 - p2 for p1, p2 in combinations(other_triangle.points, 2))
            return max_other.angle_xy(max_dist)
        else:
            return False
    
    def __str__(self) -> str:
        return f'Triangle({self.points[0].value_rounded(2)}, {self.points[1].value_rounded(2)}, {self.points[2].value_rounded(2)})'

def test_identical_point_cloud():
    pc = PointCloud.random(10)
    pc2 = PointCloud(pc.points.copy())

    offset, offset_rotation = pc.get_twist(pc2)
    assert offset is not None
    assert offset == TupleVector3((0, 0, 0))
    assert offset_rotation == TupleRotator3((0, 0, 0))

    random.shuffle(pc2.points)

    offset, offset_rotation = pc.get_twist(pc2)
    assert offset is not None
    assert offset == TupleVector3((0, 0, 0))
    assert offset_rotation == TupleRotator3((0, 0, 0))

def test_point_cloud(base: PointCloud, compare_slice_size, tolerance=0.1, iterations=0):
    points = base.points
    random.shuffle(points)
    points = points[:compare_slice_size]
    compare = PointCloud(points)

    random_intensity = 1
    random_deviation = 0

    random_offset = TupleVector3.random_xy(random_intensity)
    random_rotation = TupleRotator3.random_xy(10 * random_intensity)

    compare.offset(random_offset)
    compare.rotate(random_rotation)
    compare.points = [p + TupleVector3.random_xy(random_deviation) for p in compare.points]

    offset, rotation = base.get_twist(compare)

    assert offset is not None
    assert (offset - random_offset).length() < tolerance
    assert abs(rotation[0] - random_rotation[0]) < tolerance


def test_poles():
    fc = Field()
    poles = fc.generate_poles(5, 3)
    pole_pc = PointCloud([pole.distance for pole in poles])

    test_point_cloud(pole_pc, 4)


if __name__ == "__main__":
    test_identical_point_cloud()

    # test_poles()