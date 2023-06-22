#!/usr/bin/env python

import cv2
import math
import numpy as np
from typing import List, Tuple
from itertools import combinations
import math_utils.math_function_utils as mf
from visualization.imgops import empty_image
from math_utils.vector_utils import TupleVector3, TupleRotator3
from testing.testing import *

class PointCloud:
    def __init__(self, points: np.ndarray, local_points=False, origin=np.array([0, 0, 0]), average_origin=False):
        '''Class that represents a cloud of points in 3D space.
        Args:
            points: List of points
            local_points: Leaves point coordinates as-is if True or recalculates their position relative to the given origin
            origin: optional origin point for the point cloud
            average_origin: whether the origin should be calculated from the average of the given points'''
        
        assert len(points) > 0, "You must provide at least one point"

        self.points: np.ndarray = points
        self.origin: np.ndarray

        if average_origin:
            self.origin = np.sum(self.points, 0) / len(self.points)
        else:
            self.origin = origin

        if not local_points:
            self.points = self.points - self.origin

    @classmethod
    def from_tuple_vectors(cls, points: List[TupleVector3], local_points=False, origin=TupleVector3(), average_origin=False):
        nppoints = np.vstack([p.tuple for p in points])
        nporigin = origin.tuple
        return cls(nppoints, local_points, nporigin, average_origin)

    def points_global(self):
        return self.points + self.origin
    
    def rotate(self, rotation: np.ndarray, global_rotation=False):
        ca, cb, cc = np.cos(rotation * math.pi / 180)
        sa, sb, sc = np.sin(rotation * math.pi / 180)
        rot_matrix = np.array([(ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc),
                                (sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc),
                                (-sb, cb * sc, cb * cc)])
        self.points = np.apply_along_axis(lambda p: np.matmul(rot_matrix, p), 1, self.points)
        self.origin = np.matmul(rot_matrix, self.origin)

    def offset(self, offset: np.ndarray):
        self.origin = self.origin + offset

    def distance(self, other_point_cloud: 'PointCloud'):
        return other_point_cloud.origin - self.origin

    def get_triangles(self):
        tris = []
        areas = []
        for comb in combinations(self.points, 3):
            tri = Triangle(np.array(comb))
            if tri.area > 0.01:
                tri.origin += self.origin
                tris.append(tri)
                areas.append(tri.area)

        tris = np.array(tris)
        areas = np.array(areas)
        areas = areas.argsort()[::-1]
        tris = tris[areas]
        return tris

    def get_twist(self, base_point_cloud: 'PointCloud', tolerance=0.05, iterations=0):
        DEBUG_DRAW = False

        compare_tris = self.get_triangles()

        for i, compare_tri in enumerate(compare_tris):
            if i + 1 > iterations > 0:
                break

            for base_edge_pts in combinations(base_point_cloud.points, 2):
                base_edge = base_edge_pts[1] - base_edge_pts[0]
                norms = np.linalg.norm(compare_tri.edges, axis=1)
                base_norm = np.linalg.norm(base_edge)
                edges_relative_difference = np.vectorize(lambda x, y: mf.relative_difference(x, y))(norms, base_norm)
                find_similar_edge_indices = np.where(edges_relative_difference <= tolerance)[0]
                if len(find_similar_edge_indices) == 0:
                    continue

                matching_compare_edge_index = find_similar_edge_indices[0]
                compare_edge = compare_tri.edges[matching_compare_edge_index]

                offset_phi = mf.atan2d(compare_edge[1], compare_edge[0]) - mf.atan2d(base_edge[1], base_edge[0]) 
                while offset_phi > 180:
                    offset_phi -= 360
                while offset_phi < -180:
                    offset_phi += 360

                if offset_phi > 90:
                    offset_phi -= 180
                    base_edge_pts = (base_edge_pts[1], base_edge_pts[0])
                    base_edge = -base_edge
                elif offset_phi < -90:
                    offset_phi += 180
                    base_edge_pts = (base_edge_pts[1], base_edge_pts[0])
                    base_edge = -base_edge

                offset_rotation = TupleRotator3((offset_phi, 0, 0))

                next_compare_edge = TupleVector3(compare_tri.edges[(matching_compare_edge_index + 1) % 3])
                try_next_base_edge = (next_compare_edge - offset_rotation).tuple

                if DEBUG_DRAW:
                    img = empty_image((400, 400))
                    compare_tri.draw(img, (255, 0, 0))
                    base_point_cloud.draw(img, (0, 255, 0))
                    draw_vector(img, base_edge, base_edge_pts[0], (0, 255, 255))
                    draw_vector(img, try_next_base_edge, base_edge_pts[1], color=(255, 255, 255))
                    draw_vector(img, -try_next_base_edge, base_edge_pts[0], color=(255, 255, 255))


                theoretical_third_point = base_edge_pts[1] + try_next_base_edge
                for pt in base_point_cloud.points:
                    third_point_difference = np.linalg.norm(theoretical_third_point - pt)
                    find_points = np.where(third_point_difference <= tolerance)[0]
                    if len(find_points) > 0:
                        found_third_point = pt
                        break
                else:
                    continue

                base_tri = Triangle(np.vstack([base_edge_pts[0], base_edge_pts[1], found_third_point]))
                base_tri.offset(base_point_cloud.origin)
                compare_origin = TupleVector3(compare_tri.origin) #- offset_rotation
                offset = compare_origin - TupleVector3(base_tri.origin)

                if DEBUG_DRAW:
                    rot_compare_tri = Triangle(compare_tri.points_global())
                    rot_compare_tri.rotate(-offset_rotation.tuple, True)
                    rot_compare_tri.draw(img, (255, 150, 100))
                    draw_vector(img, offset.tuple, color=(0, 0, 255))
                    cv2.imshow("tri", img)
                    cv2.waitKey(10)
                return offset, offset_rotation

        return None, None
    
    def draw(self, img, color=(0, 255, 0), scale=20):
        for p in self.points_global():
            draw_vector(img, p, np.array([0, 0, 0]), color, scale)

    def copy(self):
        return self.__class__(self.points.copy(), True, self.origin)

    @classmethod
    def random(cls, num_points, length=8):
        points = np.random.uniform(-length, length, (num_points, 3))
        return cls(points)

    @classmethod
    def random_xy(cls, num_points, length=8):
        points = np.random.uniform(-length, length, (num_points, 3))
        points[:, 2] = 0
        return (cls(points))
    
def draw_vector(image, vector: np.ndarray, point: np.ndarray=np.array([0, 0, 0]), color: Tuple[float, float, float] = (0, 0, 255), scale=20, draw_arrowhead=True):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    if draw_arrowhead:
        cv2.arrowedLine(image, (center[0] + int(point[0] * scale), center[1] - int(point[1] * scale)),
                               (center[0] + int((point + vector)[0] * scale), center[1] - int((point + vector)[1] * scale)), color, 1)
    else:
        cv2.line(image, (center[0] + int(point[0] * scale), center[1] - int(point[1] * scale)),
                        (center[0] + int((point + vector)[0] * scale), center[1] - int((point + vector)[1] * scale)), color, 1)
        
def draw_text(image, text, point: np.ndarray, color: Tuple[float, float, float]=(255, 255, 255), scale=20, text_size=0.5):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    cv2.putText(image, text, (center[0] + int(point[0] * scale), center[1] - int(point[1] * scale)), cv2.FONT_HERSHEY_SIMPLEX, text_size, color)

class Triangle(PointCloud):
    '''Class that represents a triangle in 3D space.
    The origin of the triangle is always the average of the three corner points.'''
    def __init__(self, points: np.ndarray, local_points=False, origin=np.array((0, 0, 0)), average_origin=True):
        assert len(points) == 3, "You must provide exactly three points"
        super().__init__(points, local_points, origin, average_origin)
        A, B, C = self.points
        a = C - B
        b = A - C
        c = B - A
        try:
            alpha = mf.np_angle(b, -c)
            beta = mf.np_angle(a, -c)
            gamma = mf.np_angle(a, -b)
        except:
            pass
        self.angles = np.array([alpha, beta, gamma])
        try:
            assert round(np.sum(self.angles), 5) == 180, "Angles do not add up to 180"
        except:
            img = empty_image((400, 400))
            draw_vector(img, a, color=(0, 0, 255))
            draw_vector(img, b, color=(0, 255, 0))
            draw_vector(img, c, color=(255, 0, 0))
            cv2.putText(img, f"{mf.np_angle(b, -c)}, {mf.np_angle(a, -c)}, {mf.np_angle(a, -b)}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
            cv2.imshow("tri angs", img)
            cv2.waitKey(100)
            pass
    
        self.edges = np.vstack([a, b, c])
        self.area = 0.5 * mf.np_area(a, b)

    def rotate(self, rotation: np.ndarray):
        ca, cb, cc = np.cos(rotation * math.pi / 180)
        sa, sb, sc = np.sin(rotation * math.pi / 180)
        rot_matrix = np.array([(ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc),
                                (sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc),
                                (-sb, cb * sc, cb * cc)])
        self.points = np.apply_along_axis(lambda p: np.matmul(rot_matrix, p), 1, self.points)
        self.edges = np.apply_along_axis(lambda e: np.matmul(rot_matrix, e), 1, self.edges)

    def matches(self, other_triangle: 'Triangle', tolerance=0.05):
        # check whether areas match
        if mf.relative_difference(self.area, other_triangle.area) > tolerance:
            return False
        
        # check whether all angles match
        angs_other = [other_triangle.angles[[0, 1, 2]],
                      other_triangle.angles[[1, 2, 0]],
                      other_triangle.angles[[2, 0, 1]]]
        for perm in angs_other:
            errors = np.vectorize(lambda ang1, ang2: mf.relative_difference(ang1, ang2))(self.angles, perm)
            if np.sum(errors) / 3 <= tolerance:
                break
        else:
            return False
        
        # check whether edge lengths match if area is near zero
        if self.area < 0.001:
            edges_other = [other_triangle.edges[[0, 1, 2]],
                           other_triangle.edges[[1, 2, 0]],
                           other_triangle.edges[[2, 0, 1]]]
            for perm in edges_other:
                errors = np.vectorize(lambda edge1, edge2: mf.relative_difference(mf.np_length(edge1), mf.np_length(edge2)))(self.edges, perm)
                if np.sum(errors) / 3 <= tolerance:
                    break
            else:
                return False
        
        return True
    
    def angle_xy(self, other_triangle: 'Triangle'):
        if self.matches(other_triangle):
            max_dist = max(self.edges, key=lambda e: mf.np_length(e))
            max_other = max(other_triangle.edges, key=lambda e: mf.np_length(e))
            return mf.np_angle_xy(max_dist, max_other)
        else:
            return False
    
    def draw(self, img, color=(0, 255, 0), scale=20, draw_labels=False):
        draw_vector(img, self.origin, color=(0, 150, 200), scale=scale)
        for i, p in enumerate(self.points):
            new_color = color # np.array(color) * (1 - 0.3 * i)
            draw_vector(img, p, self.origin, color=new_color, scale=scale, draw_arrowhead=False)
            draw_vector(img, self.edges[i], self.origin + self.points[(i + 1) % 3], color=new_color, scale=scale, draw_arrowhead=False)
            if draw_labels:
                draw_text(img, "ABC"[i], p, scale=scale)
                draw_text(img, "abc"[i], self.origin + self.points[(i + 1) % 3] + self.edges[i] / 2, scale=scale)

    def __str__(self) -> str:
        return f'Triangle({np.round(self.points[0], 2)}, {np.round(self.points[1], 2)}, {np.round(self.points[2])})'

def test_simple_triangle(printout=False):
    points = np.array([(0, 0 ,0), (1, 0, 0), (0, 1, 0)])
    tri1 = Triangle(points[[0, 1, 2]])
    tri2 = Triangle(points[[1, 2, 0]])
    tri3 = Triangle(points[[2, 0, 1]])
    cmp_origin = np.array((0.33333, 0.33333, 0))

    cmp_angles = np.array([90, 45, 45])
    cmp_edges = np.array([(-1, 1, 0), (0, -1, 0), (1, 0, 0)])
    assert np.array_equal(tri1.angles, cmp_angles)
    assert np.array_equal(tri1.edges, cmp_edges)
    assert tri1.area == 0.5
    assert np.array_equal(np.round(tri1.origin, 5), cmp_origin)

    cmp_angles = np.array([45, 45, 90])
    cmp_edges = np.array([(0, -1, 0), (1, 0, 0), (-1, 1, 0)])
    assert np.array_equal(tri2.angles, cmp_angles)
    assert np.array_equal(tri2.edges, cmp_edges)
    assert tri2.area == 0.5
    assert np.array_equal(np.round(tri2.origin, 5), cmp_origin)

    cmp_angles = np.array([45, 90, 45])
    cmp_edges = np.array([(1, 0, 0), (-1, 1, 0), (0, -1, 0)])
    assert np.array_equal(tri3.angles, cmp_angles)
    assert np.array_equal(tri3.edges, cmp_edges)
    assert tri3.area == 0.5
    assert np.array_equal(np.round(tri3.origin, 5), cmp_origin)

def test_triangle(printout=False):
    tri = Triangle.random(3)
    points = tri.points_global()
    tri1 = Triangle(points.copy()[[0, 1, 2]])
    tri2 = Triangle(points.copy()[[1, 2, 0]])
    tri3 = Triangle(points.copy()[[2, 0, 1]])

    assert tri.matches(tri1)
    assert tri.matches(tri2)
    assert tri.matches(tri3)

    if printout:
        img = empty_image((400, 400))
        tri.draw(img, (0, 255, 0))
        tri1.draw(img, (0, 0, 255))
        tri2.draw(img, (255, 0, 0))
        tri3.draw(img, (255, 255, 0))
        cv2.imshow("test", img)
        cv2.waitKey(10)

    rnd_origin = np.round(tri.origin, 5)
    assert np.array_equal(rnd_origin, np.round(tri1.origin, 5))
    assert np.array_equal(rnd_origin, np.round(tri2.origin, 5))
    assert np.array_equal(rnd_origin, np.round(tri3.origin, 5))

    offset, offset_rotation = tri.get_twist(tri1)
    assert offset is not None
    assert offset.value_rounded(5) == (0, 0, 0)
    assert offset_rotation.value_rounded(5) == (0, 0, 0)


def test_triangle_offset(printout=False):
    tri = Triangle.random_xy(3)
    test_point_cloud(tri, 3, printout=printout)

def test_narrow_triangle(printout=False):
    tri = Triangle(np.array([(-1, 0, 0), (1, 0, 0), (0, 0, 0)]))
    points = tri.points_global()
    perms = np.array([points[[0, 1, 2]],
                      points[[1, 2, 0]],
                      points[[2, 0, 1]]])

    random_deviation = 0.05
    # tri1 = Triangle([p + TupleVector3.random_xy(1) * random_deviation for p in perms[0]])
    # tri2 = Triangle([p + TupleVector3.random_xy(1) * random_deviation for p in perms[1]])
    # tri3 = Triangle([p + TupleVector3.random_xy(1) * random_deviation for p in perms[2]])

    tri1, tri2, tri3 = perms + np.append(np.random.uniform(-1, 1, (2,)) * random_deviation, 0)

    assert tri.matches(tri1)
    assert tri.matches(tri2)
    assert tri.matches(tri3)

def test_identical_point_cloud(printout=False):
    pc = PointCloud.random(10)
    pc2 = PointCloud(pc.points.copy())

    offset, offset_rotation = pc.get_twist(pc2)
    assert offset is not None
    assert offset.value_rounded(5) == (0, 0, 0)
    assert offset_rotation.value_rounded(5) == (0, 0, 0)

    np.random.shuffle(pc2.points)

    offset, offset_rotation = pc.get_twist(pc2)
    assert offset is not None
    assert offset.value_rounded(5) == (0, 0, 0)
    assert offset_rotation.value_rounded(5) == (0, 0, 0)

def test_point_cloud(base: PointCloud, compare_slice_size, tolerance=0.05, iterations=0, printout=False):
    points: np.ndarray = base.points.copy()
    np.random.shuffle(points)
    points = points[:compare_slice_size]
    compare = PointCloud(points, True, base.origin)

    # assert set([tuple(p) for p in points.tolist()]).issubset(set([tuple(p) for p in base.points.copy().tolist()]))

    random_intensity = 1
    random_deviation = 0.2

    random_offset = TupleVector3.random_xy(3 * random_intensity)
    random_rotation = TupleRotator3.random_xy(70 * random_intensity)
    random_error = TupleVector3.random_xy(random_deviation)

    compare.offset(random_offset.tuple)
    compare.rotate(random_rotation.tuple)
    compare.points = compare.points + random_error.tuple

    if printout:
        img = empty_image((400, 400))
        base.draw(img, (0, 255, 0))
        compare.draw(img, (255, 0, 0))
        cv2.imshow("test", img)
        cv2.waitKey(50)

    offset, rotation = compare.get_twist(base, tolerance, iterations)
    assert offset is not None, "no offset found"

    if printout:
        result = compare.copy()
        compare.rotate(-random_rotation.tuple, True)
        compare.offset(-random_offset.tuple)
        compare.draw(img, (255, 0, 255))

        result.rotate(-rotation.tuple, True)
        result.offset(-offset.tuple)
        result.draw(img, (0, 0, 255))
        cv2.imshow("test", img)
        cv2.waitKey(50)

    assert mf.relative_difference(rotation[0], random_rotation[0]) < tolerance, "rotation off by " + str(rotation[0] - random_rotation[0])
    assert mf.relative_difference(offset.length(), random_offset.length()) < tolerance, "offset off by " + str((offset - random_offset).length())


def test_random_point_cloud(printout=False):
    pc = PointCloud.random(20)
    test_point_cloud(pc, 7, printout=printout)

def test_generated_poles(printout=False):
    import field_components.field_components as fc
    f = fc.Field()
    poles = f.generate_poles(5, 3)
    pole_pc = PointCloud.from_tuple_vectors([pole.distance for pole in poles])

    if printout:
        img = empty_image((400, 400))
        pole_pc.draw(img)
        cv2.imshow("poles", img)
        cv2.waitKey(10)

def test_poles_offset(printout=False):
    import field_components.field_components as fc
    f = fc.Field()
    poles = f.generate_poles(5, 3)
    pole_pc = PointCloud.from_tuple_vectors([pole.distance for pole in poles])

    test_point_cloud(pole_pc, 6, printout=printout)

def numpy_test():
    arr1 = np.array([[1, 1, 1], [2, 2, 2], [3, 3, 3]])
    arr2 = np.array([1, 2, 3])
    print(arr1 + arr2)

    pc = PointCloud(arr1, True)
    rot = np.array([90, 0, 0])
    pc.rotate(rot)
    print(pc.points)

    arr = np.array([1, 2, 3])
    arr2 = np.where(arr < 2)
    print(arr2)

def full_stress_test():
    stress_test(test_triangle, 2000)
    stress_test(test_identical_point_cloud, 100)
    stress_test(test_random_point_cloud, 500)
    stress_test(test_poles_offset, 1000)

if __name__ == "__main__":
    # numpy_test()
    # test_simple_triangle()
    # test_triangle()
    # test_identical_point_cloud()
    # test_random_point_cloud()
    # test_generated_poles()
    # test_poles_offset

    # interval_test(lambda: test_triangle(True), 1)
    # interval_test(lambda: test_triangle_offset(True), 1)
    # interval_test(lambda: test_random_point_cloud(True), 1)
    # interval_test(lambda: test_poles_offset(True), 1)

    # for i in range(100):
    #     test_generated_poles()

    # for i in range(100):
    #     test_poles_offset(True)

    # test_narrow_triangle()

    # stress_test(lambda: PointCloud.random(20).get_triangles_idx(), 50)

    full_stress_test()
