#!/usr/bin/env python

import rospy

from math import sqrt
from math_utils.math_function_utils import cosd, sind, asind
import field_components.field_components as fc
from typing import NamedTuple, Tuple, List, Any
from math_utils.vector_utils import *
import random
import cv2
from visualization.imgops import *
from itertools import combinations
import time
from testing.testing import *

class PointCloud:
    def __init__(self, points: List[TupleVector3], origin=None):
        assert len(points) > 0, "You must provide at least one point"
        self.points = points
        self.origin = sum(points) / len(points) if origin is None else origin
    
    def rotate(self, rotation: TupleRotator3):
        self.points = [p + rotation for p in self.points]

    def offset(self, offset: TupleVector3):
        self.points = [p + offset for p in self.points]

    def distance(self, other_point_cloud: 'PointCloud'):
        return other_point_cloud.origin - self.origin
    
    def get_triangles(self):
        return sorted([Triangle(*tri) for tri in combinations(self.points, 3)], lambda tri: tri.area)

    def angle(self, other_point_cloud: 'PointCloud', iterations=0):
        tris = self.get_triangles()
        other_tris = other_point_cloud.get_triangles()

        for i in range(iterations if iterations > 0 else len(tris)):
            for other_tri in other_tris:
                if tris[i].matches(other_tri):
                    return tris[i].angle(other_tri)

        return False

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

    def point_permutations(self):
        A = self.points[0]
        B = self.points[1]
        C = self.points[2]
        return [(A, B, C), (B, C, A), (C, A, B)]
    
    def angle_permutations(self):
        alpha = self.angles[0]
        beta = self.angles[1]
        gamma = self.angles[2]
        return [(alpha, beta, gamma), (beta, gamma, alpha), (gamma, alpha, beta)]
    
    def matches(self, other_triangle: 'Triangle', tolerance=0.1):
        # check if areas match
        if abs(self.area / other_triangle.area - 1) > tolerance:
            return False
        
        # check if all angles match
        angs_other = other_triangle.angle_permutations()
        for perm in angs_other:
            if sum([abs(self.angles[i] / perm[i] - 1) for i in range(3)]) / 3 > tolerance:
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

class Result(NamedTuple):
    value: Any
    error: float

def cosine_theorem(pole_a: fc.Pole, pole_b: fc.Pole):
    # '''Calculates the distance between two poles using the cosine theorem.
    
    # Args:
    #     pole_a: first pole
    #     pole_b: second pole
    
    # Returns:
    #     distance between the two poles
    # '''
    # a = pole_a.distance.tuple[0]
    # b = pole_b.distance.tuple[0]
    # gamma = abs(pole_a.distance.tuple[2] - pole_b.distance.tuple[2])

    # return sqrt(a**2 + b**2 - 2 * a * b * cosd(gamma))

    return pole_a.distance.distance(pole_b.distance)

def get_position(pole_a: fc.Pole, pole_b: fc.Pole, pole_c: fc.Pole) -> Tuple:
    '''This function calculates the angle from pole to Robot.
    Poles given to this function should be ordered from left to right.
    The function check_poles() does this.
    '''
    if pole_c:
        if pole_a is None or pole_b is None:
            rospy.logwarn('Pole is None')
            return None
        if pole_a.position is None or pole_b.position is None:
            rospy.logwarn('Pole position is None')
            return None
        
        # case 1: pole a and b
        dist = cosine_theorem(pole_a, pole_b)
        gamma = abs(pole_a.distance[2] - pole_b.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x = pole_b.position[0] + pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] + pole_b.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x = pole_b.position[0] - pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] - pole_b.distance[0]*sind(rho)

        # case 2: pole a and c
        dist = cosine_theorem(pole_a, pole_c)
        gamma = abs(pole_a.distance[2] - pole_c.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x1 = pole_c.position[0] + pole_c.distance[0]*cosd(rho)
            pos_y1 = pole_c.position[1] + pole_c.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x1 = pole_c.position[0] - pole_c.distance[0]*cosd(rho)
            pos_y1 = pole_c.position[1] - pole_c.distance[0]*sind(rho)

        return (pos_x + pos_x1 / 2, pos_y + pos_y1 / 2)
    else:
        if pole_a is None or pole_b is None:
            rospy.logwarn('Pole is None')
            return None
        if pole_a.position is None or pole_b.position is None:
            rospy.logwarn('Pole position is None')
            return None
        
        # case 1: pole a and b
        dist = cosine_theorem(pole_a, pole_b)
        gamma = abs(pole_a.distance[2] - pole_b.distance[2]) 

        rho = asind(pole_a.distance[0]/dist *sind(gamma))
        if pole_a.position[1] == 0:
            pos_x = pole_b.position[0] + pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] + pole_b.distance[0]*sind(rho)
        elif pole_a.position[1] == 3:
            pos_x = pole_b.position[0] - pole_b.distance[0]*cosd(rho)
            pos_y = pole_b.position[1] - pole_b.distance[0]*sind(rho)

        return (pos_x, pos_y)

def find_large_distance(vectors: List[TupleVector3]):
    '''Finds two vectors with a large distance in a list of vectors.
    It's not really the maximum distance, but a good enough estimate.'''
    v1 = max(vectors)
    v2 = max(vectors, key=lambda v: v1-v)
    return v2, v1

def find_large_triangle(vectors: List[TupleVector3]):
    '''Finds three vectors that form a large triangle in the given list of vectors'''
    v1, v2 = find_large_distance(vectors)
    v3 = max(vectors, key=lambda v: v.distance(v1) + v.distance(v2))
    return v1, v2, v3

def find_max_distance(vectors: List[TupleVector3]):
    '''Finds the maximum distance between two vectors in the given list'''
    return max(combinations(vectors, 2), key=lambda cmb: (cmb[1]-cmb[0]).length())

def find_max_triangle(vectors: List[TupleVector3]):
    '''Finds the triangle with the largest area in the given list of vectors'''
    triangles = combinations(vectors, 3)
    return max(triangles, key=lambda t: (t[0]-t[1]).cross(t[2]-t[1]).length())

def find_distance(dist, vectors: List[TupleVector3], tolerance=0.01):
    '''Finds the given distance between two vectors in the given list'''
    distances = []
    for i in range(len(vectors)):
        for j in range(i + 1, len(vectors)):
            delta = abs(vectors[i].distance(vectors[j]) - dist)
            distances.append((delta, i, j))
            if delta < tolerance:
                return vectors[i], vectors[j]
            
    d_min, i_min, j_min = min(distances, key=lambda e: e[0])
    print(sorted(distances, key=lambda e: e[0])[:5])
    rospy.logwarn(f"Find distance: tolerance exceeded by {d_min - tolerance}!")
    print(f"Find distance: tolerance exceeded by {d_min - tolerance}!")
    return vectors[i_min], vectors[j_min]

def find_triangle(triangle: Triangle, sorted_triangles: List[Triangle], max_tolerance=0.1, delta_tolerance=0.1) -> List[Triangle]:
    '''Finds triangles in the list of triangles sorted by area that approximately match the given triangle.
    Args:
    max_tolerance: The maximum relative difference in area and angle between the given triangle and the found triangles.
    delta_tolerance: The maximum difference of the differences of the results relative to the best result. An accurate best result yields fewer secondary results.'''
    results: List[Result] = []

    for tri in sorted_triangles:
        area_percent = tri.area / triangle.area

        if area_percent < 1 - max_tolerance:
            continue
        elif area_percent > 1 + max_tolerance:
            break

        angle_percent = sum([abs(ang1 / ang2) for ang1, ang2 in zip(sorted(tri.angles()), sorted(triangle.angles()))]) / 3

        if 1 - max_tolerance <= angle_percent <= 1 + max_tolerance:
            results.append(Result(tri, abs(area_percent - 1) + abs(angle_percent - 1)))

    if len(results) == 0:
        return None
    
    # sort results by error
    results.sort(key=lambda e: e.error)
    best_result = results[0]

    # return all results with an error within delta_tolerance of the best result
    if best_result.error == 0:
        return [best_result[0]]
    else:
        return [best_result[0], *[r[0] for r in results[1:] if abs(r.error - best_result.error) / best_result.error < delta_tolerance]]

def get_triangles(vectors: List[TupleVector3]):
    '''Returns a list of all triangles in the given list of vectors, sorted by area.
    Triangles with area ~ 0 are omitted.'''
    triangles: List[Triangle] = []
    for tri in combinations(vectors, 3):
        new_tri = Triangle(*tri)
        if new_tri.area > 0.05:
            triangles.append(new_tri)
    return sorted(triangles, key=lambda t: t.area)

def calculate_distances(vectors: List[TupleVector3]):
    '''Calculates the distances between all vectors in the given list'''
    return [(comb[0].distance(comb[1]), comb[0], comb[1]) for comb in combinations(vectors, 2)]

def matching_clouds(base: List[TupleVector3], compare: List[TupleVector3], tolerance=0.1):
    '''Checks if the given vectors in compare match any vector in base within a given tolerance.'''
    for v in compare:
        for b in base:
            if v.approx(b, tolerance):
                # unchecked_base.remove(b)
                break
        else:
            return False
    return True

def get_vector_cloud_offset_2D_max(base: List[TupleVector3], compare: List[TupleVector3], tolerance=0.1, iterations=0):
    '''Calculates the offset distance and rotation of two clouds of vectors based on the maximum
    distance found in the compare list. The compare list must be a matching subset of the objects from the base list.'''
    DRAW_TEST = True

    v1_max, v2_max = find_max_distance(compare)
    distances = calculate_distances(base)
    compare_dist = v1_max.distance(v2_max)
    distances.sort(key=lambda d: abs(d[0]-compare_dist))

    offset = None
    offset_rotation = None

    for i in range(iterations if iterations > 0 else len(distances)):
        base_dist, v1, v2 = distances[i]

        cases = [([v1, v2], [v1_max, v2_max], (0, 0, 255)),
                 ([v1, v2], [v2_max, v1_max], (0, 255, 255)),
                 ([v2, v1], [v1_max, v2_max], (255, 0, 255)),
                 ([v2, v1], [v2_max, v1_max], (255, 255, 0))]

        for c in cases:
            try:
                offset, offset_rotation = get_vector_offset_2D(c[0], c[1])
            except AssertionError:
                continue

            result = [v - offset_rotation - offset for v in compare]

            if matching_clouds(base, result, tolerance):
                return offset, offset_rotation
        
    return None, None

def get_vector_cloud_offset_2D_tri(base: List[TupleVector3], compare: List[TupleVector3], tolerance=0.1, iterations=0):
    base_tris = get_triangles(base)
    compare_tris = get_triangles(compare)

    for i in range(iterations if iterations > 0 else len(compare_tris)):
        compare_tri = compare_tris[-1-i]
        results = find_triangle(compare_tri, base_tris, tolerance, tolerance)

        if results is not None:
            try:
                offset, offset_rotation = get_triangle_offset_2D(results[0], compare_tri)
            except AssertionError:
                continue

            result = [v - offset_rotation - offset for v in compare]

            if matching_clouds(base, result, tolerance):
                return offset, offset_rotation
            else:
                print("result not matching")
        
    return None, None

def get_vector_cloud_offset_2D(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the average offset distance and rotation of two clouds of vectors.
    the base and compare lists are assumed to contain the same objects in the corresponding order.
    
    Args:
        base: base cloud of vectors
        compare: cloud of vectors to compare
    
    Returns:
        offset of the two clouds
    '''
    offsets = []
    offset_rotations = []

    for i in range(len(compare)-1):
        try:
            offset, offset_rotation = get_vector_offset_2D(base[i:i+2], compare[i:i+2])
        except AssertionError:
            print(f"vectors {i} and {i+1} do not match")
            raise
        
        offsets.append(offset)
        offset_rotations.append(offset_rotation)

    offset = sum(offsets) / len(offsets)
    offset_rotation = sum(offset_rotations) / len(offset_rotations)

    return offset, offset_rotation

def get_vector_offset_2D(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the offset distance and angle of two sets of two points.'''
    assert len(base) == 2, "base must contain exactly two vectors"
    assert len(compare) == 2, "compare must contain exactly two vectors"

    base_vec = base[1] - base[0]
    compare_vec = compare[1] - compare[0]

    assert base_vec.length() - compare_vec.length() < 0.05, "vectors must have the same length"

    offset_angle = compare_vec.angle_xy(base_vec)
    offset_rotation = TupleRotator3((offset_angle, 0, 0))

    offset = compare[0] - offset_rotation - base[0]

    test_vec = compare[1] - offset_rotation - offset

    assert test_vec.approx(base[1], 0.05), f"offsets do not match by {(test_vec - base[1]).length()}"
    return offset, offset_rotation

def get_triangle_offset_2D(base: Triangle, compare: Triangle):
    '''Calculates the offset distance and rotation of two triangles.'''
    compare_perm = [compare, 
                    Triangle(compare.A, compare.C, compare.B),
                    Triangle(compare.B, compare.A, compare.C),
                    Triangle(compare.B, compare.C, compare.A),
                    Triangle(compare.C, compare.A, compare.B),
                    Triangle(compare.C, compare.B, compare.A)]

    for tri in compare_perm:
        try:
            offset, offset_rotation = get_vector_offset_2D(base.points()[:2], tri.points()[:2])
            assert (tri.points()[2] - offset_rotation - offset).approx(base.points()[2], 0.01)
            return offset, offset_rotation
        except AssertionError:
            pass

    raise AssertionError("triangles do not match")

def get_random_vector_cloud(count=10):
    return [TupleVector3.random_xy(10) for i in range(count)]

def draw_vector_cloud(image, cloud: List[TupleVector3], color: Tuple[float, float, float] = (0, 0, 255), scale=20):
    for v in cloud:
        draw_vector(image, v, TupleVector3(), color, scale)

def draw_vector(image, vector: TupleVector3, point: TupleVector3, color: Tuple[float, float, float] = (0, 0, 255), scale=20):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    cv2.arrowedLine(image, (center[0] + int(point.tuple[0] * scale), center[1] - int(point.tuple[1] * scale)),
                           (center[0] + int((point + vector).tuple[0] * scale), center[1] - int((point + vector).tuple[1] * scale)), color, 1)

def test_random_cloud_offset(printout = False, offset_calculation_method=get_vector_cloud_offset_2D_max, *offset_calculation_args):
    base = get_random_vector_cloud(10)
    test_cloud_offset(base, 4, printout, offset_calculation_method, *offset_calculation_args)

def test_pole_cloud_offset(printout = False):
    field = fc.Field()
    poles = field.generate_poles(5, 3)
    base = [p.distance for p in poles]
    test_cloud_offset(base, 3, printout, get_vector_cloud_offset_2D_tri)

def test_cloud_offset(base, compare_slice_count, printout = False, offset_calculation_method=get_vector_cloud_offset_2D_max, *offset_calculation_args):
    random_intensity = 1
    random_deviation = 0

    if printout:
        winname = "test"
        random_intensity_trackbar = TrackbarParameter(1, "random_intensity", winname, value_factor=0.1)
        random_intensity = random_intensity_trackbar.get_value(True)

    if printout:
        img = empty_image((500, 500))

    random_rotator = TupleRotator3.random_xy(10) * random_intensity
    random_offset = TupleVector3.random_xy() * random_intensity
    
    compare = base.copy()
    random.shuffle(compare)
    compare = compare[0:compare_slice_count]

    if printout:
        test(matching_clouds(base, compare, 0.1), True)
    assert matching_clouds(base, compare, 0.1)

    compare = [v + random_offset + random_rotator + TupleVector3.random_xy() * random_deviation for v in compare]

    offset, offset_rotation = offset_calculation_method(base, compare, *offset_calculation_args)

    if printout:
        if not test(offset is not None, True):
            pass # breakpoint here
    else:
        assert offset is not None, "no offset found"

    if printout:
        test(offset.value_rounded(5), random_offset.value_rounded(5))
        test(offset_rotation.value_rounded(5), random_rotator.value_rounded(5))
    else:
        assert offset.approx(random_offset, 0.01), f"offsets do not match by {(offset - random_offset).length()}"
        assert offset_rotation.tuple[0] - random_rotator.tuple[0] < 2, f"rotations do not match by {offset_rotation.tuple[0] - random_rotator.tuple[0]}"

    if printout:
        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))

    result = [v - offset_rotation - offset for v in compare]

    if printout:
        test(matching_clouds(base, result, 0.1), True)
    else:
        assert matching_clouds(base, result, 0.1)

    if printout:
        draw_vector_cloud(img, result, (0, 0, 255))
        draw_vector_cloud(img, [offset], (0, 255, 255))

    if printout:
        v1, v2 = find_max_distance(compare)
        draw_vector(img, v2 - v1, v1, (255, 255, 255))

    if printout:
        cv2.putText(img, "base", (0, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
        cv2.putText(img, "compare", (0, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
        cv2.putText(img, "result", (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
        cv2.imshow(winname, img)
        cv2.waitKey(5)

def test_matching_function():
    random_deviation = TrackbarParameter(0, "random", "test", value_factor=0.01)
    base = get_random_vector_cloud(10)

    while True:
        compare = [v + TupleVector3.random_xy() * random_deviation.get_value(True) for v in base]

        result = matching_clouds(base, compare, 2*random_deviation.get_value(True))
        print(result)

        img = empty_image((400, 400))
        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))
        cv2.putText(img, str(random_deviation.get_value(True)) + (" ok" if result else " nok"), (0, 390), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))

        cv2.imshow("test", img)
        cv2.waitKey(1)
        time.sleep(1)

def random_triangle_xy(length=1):
    return Triangle(TupleVector3.random_xy(length), TupleVector3.random_xy(length), TupleVector3.random_xy(length))

def test_triangle_functions(printout=False):
    points = [TupleVector3.random_xy(), TupleVector3.random_xy(), TupleVector3.random_xy()]
    tri_base = Triangle(*points)

    random.shuffle(points)
    tri_compare = Triangle(*points)

    if printout:
        print(tri_base)
        print(tri_compare)

    offset, offset_rotation = get_triangle_offset_2D(tri_base, tri_compare)

    assert offset is not None, "no offset found"
    assert offset.value_rounded(5) == (0, 0, 0), f"offsets do not match by {offset.value_rounded(5)}"
    assert offset_rotation.value_rounded(5) == (0, 0, 0), f"rotations do not match by {offset_rotation.value_rounded(5)}"

    rand_offset = TupleVector3.random(1) * (1, 1, 0)
    rand_offset_rotation = TupleRotator3.random(10) * (1, 0, 0)
    tri_compare = Triangle(*[v + rand_offset + rand_offset_rotation for v in tri_compare.points()])

    offset, offset_rotation = get_triangle_offset_2D(tri_base, tri_compare)

    assert offset.value_rounded(5) == rand_offset.value_rounded(5), f"offsets do not match by {offset.value_rounded(5) - rand_offset.value_rounded(5)}"
    assert offset_rotation.value_rounded(5) == rand_offset_rotation.value_rounded(5), f"rotations do not match by {offset_rotation.value_rounded(5)[1] - rand_offset_rotation.value_rounded(5)[1]}"
    
    tri_result = Triangle(*[v - rand_offset_rotation - rand_offset for v in tri_compare.points()])
    
    if printout:
        print(tri_result)
        img = empty_image((400, 400))
        draw_vector_cloud(img, tri_base.points(), (0, 255, 0))
        draw_vector_cloud(img, tri_compare.points(), (255, 0, 0))
        draw_vector_cloud(img, tri_result.points(), (0, 0, 255))
        cv2.imshow("test", img)
        cv2.waitKey(10)

def test_find_triangle(printout=False):
    tris_base = [random_triangle_xy(5) for i in range(10)]
    tris_base.sort(key=lambda tri: tri.area)

    find_tri_index = random.randint(0, len(tris_base)-1)
    find_tri = tris_base[find_tri_index]

    results = find_triangle(find_tri, tris_base)

    if printout:
        test((results is not None), True)
    else:
        assert results is not None, "no triangle found"

    if printout:
        if not test(tris_base.index(results[0]), find_tri_index):
            print("to find: ", find_tri)
            print("in:")
            for tri in tris_base:
                print("       ", tri)
            print("found:   ", *results)
            raise AssertionError("found triangle does not match base triangle")
    else:
        assert tris_base.index(results[0]) == find_tri_index, f"found triangle does not match base triangle"


def test_vector_matching(printout = False):
    base1 = TupleVector3.random(1)
    base2 = TupleVector3.random(1)
    rand_offset = TupleVector3.random(1) * (1, 1, 0)
    rand_offset_rotation = TupleRotator3.random(10) * (1, 0, 0)

    compare1 = base1 + rand_offset + rand_offset_rotation
    compare2 = base2 + rand_offset + rand_offset_rotation

    offset, offset_rotation = get_vector_offset_2D([base1, base2], [compare1, compare2])
    if printout:
        test(offset.value_rounded(5), rand_offset.value_rounded(5))
        test(offset_rotation.value_rounded(5), rand_offset_rotation.value_rounded(5))
    else:
        assert offset.value_rounded(5) == rand_offset.value_rounded(5)
        assert offset_rotation.value_rounded(5) == rand_offset_rotation.value_rounded(5)

if __name__ == "__main__":
    # test_vector_matching(True)
    # stress_test(test_vector_matching, 10000)

    # test_cloud_offset(True)
    # interval_test(lambda: test_cloud_offset(True), 3)
    # stress_test(test_random_cloud_offset, 1000)

    # test_triangle_functions(True)
    # stress_test(test_triangle_functions, 10000)
    # test_find_triangle(True)
    # stress_test(test_find_triangle, 10000)

    # test_cloud_offset(True, get_vector_cloud_offset_2D_tri)
    # interval_test(lambda: test_random_cloud_offset(True, get_vector_cloud_offset_2D_tri), 2)
    stress_test(lambda: test_random_cloud_offset(False, get_vector_cloud_offset_2D_tri), 1000)

    # interval_test(lambda: test_pole_cloud_offset(True), 1)
    # stress_test(test_pole_cloud_offset, 100)
