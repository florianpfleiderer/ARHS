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

class Triangle(NamedTuple):
    '''This class represents a triangle in 3D space.
    It is automatically oriented so that the shortest side is side a (BC).'''
    A: TupleVector3
    B: TupleVector3
    C: TupleVector3
    area: float
    gamma: float

    def __str__(self):
        return f'Triangle({self.A}, {self.B}, {self.C})'
    
    def __init__(self, A: TupleVector3, B: TupleVector3, C: TupleVector3):
        self.A = A
        self.B = B
        self.C = C
        a = C - B
        b = C - A
        c = B - A

        min_side = min(a.length(), b.length(), c.length())
        if min_side == b.length():
            self.A, self.B, self.C = self.B, self.C, self.A
            a, b, c = b, c, a
        elif min_side == c.length():
            self.A, self.B, self.C = self.C, self.A, self.B
            a, b, c = c, a, b

        self.area = 0.5 * c.cross(b).length()
        self.gamma = c.angle(b)

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
    return max(vectors, key=lambda v: max(v.distance(v2) for v2 in vectors if v2 != v))

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

def find_triangle(triangle: Triangle, sorted_triangles: List[Triangle], max_tolerance=0.1, delta_tolerance=0.1):
    '''Finds triangles in the list of triangles sorted by area that approximately match the given triangle.
    Args:
    max_tolerance: The maximum relative difference in area and angle between the given triangle and the found triangles.
    delta_tolerance: The maximum difference of the differences of the results relative to the best result. An accurate best result yields fewer secondary results.'''
    results: List[Result] = []
    for tri in sorted_triangles:
        area_percent = tri.area / triangle.area
        if area_percent < 1 - max_tolerance:
            continue
        if area_percent > 1 + max_tolerance:
            break

        angle_percent = tri.gamma / triangle.gamma
        if 1 - max_tolerance < angle_percent < 1 + max_tolerance:
            results.append(Result(tri, abs(area_percent - 1) + abs(angle_percent - 1)))
    
    if len(results) == 0:
        return None
    
    results.sort(key=lambda e: e.error, reverse=True)
    best_result = results[0]
    return [best_result, *[r[0] for r in results[1:] if abs(r.error - best_result.error) / best_result.error < delta_tolerance]]

def get_triangles(vectors: List[TupleVector3]):
    '''Returns a list of all triangles in the given list of vectors, sorted by area'''
    triangles: List[Triangle] = []
    for tri in combinations(vectors, 3):
        triangles.append(Triangle(*tri))
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
            offset, offset_rotation = get_vector_offset_2D(c[0], c[1])
            result = [v - offset_rotation - offset for v in compare]

            if matching_clouds(base, result, tolerance):
                print(f"found angle {offset_rotation.value()[0]}")
                return offset, offset_rotation
            
                # if offset_rotation.value()[0] <= 90:
                #     return offset, offset_rotation
                # else:
                #     return offset, offset_rotation - (180, 0, 0)
        
    return None, None

def get_vector_cloud_offset_2D_tri(base: List[TupleVector3], compare: List[TupleVector3], tolerance=0.1, iterations=0):
    base_tris = get_triangles(base)
    compare_tris = get_triangles(compare)
    compare_tris.sort(key=lambda t: t.area, reverse=True)

    for i in range(iterations if iterations > 0 else len(compare_tris)):
        compare_tri = compare_tris[i]
        results = find_triangle(compare_tri, base_tris, tolerance, tolerance)
        if results is not None:
            return get_triangle_offset_2D(results[0], compare_tri)

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
        offset, offset_rotation = get_vector_offset_2D(base[i:i+2], compare[i:i+2])
        offsets.append(offset)
        offset_rotations.append(offset_rotation)

    offset = sum(offsets) / len(offsets)
    offset_rotation = sum(offset_rotations) / len(offset_rotations)

    return offset, offset_rotation

def get_vector_offset_2D(base: List[TupleVector3], compare: List[TupleVector3]):
    '''Calculates the offset distance and angle of two sets of two vectors.'''

    offset_angle = (base[1] - base[0]).angle(compare[1] - compare[0])
    offset_rotation = TupleRotator3((offset_angle, 0, 0))

    v = compare[0] - offset_rotation
    offset = v - base[0]

    return offset, offset_rotation

def get_triangle_offset_2D(base: Triangle, compare: Triangle):
    '''Calculates the offset distance and rotation of two triangles.'''
    offset, offset_rotation = get_vector_cloud_offset_2D([base.a, base.b, base.c], [compare.a, compare.b, compare.c])
    return offset, offset_rotation

def get_random_vector_cloud(count=10):
    return [TupleVector3(((random.random() - 0.5) * 20, (random.random() - 0.5) * 20, 0)) for i in range(count)]

def draw_vector_cloud(image, cloud: List[TupleVector3], color: Tuple[float, float, float] = (0, 0, 255), scale=20):
    for v in cloud:
        draw_vector(image, v, TupleVector3(), color, scale)

def draw_vector(image, vector: TupleVector3, point: TupleVector3, color: Tuple[float, float, float] = (0, 0, 255), scale=20):
    dim = image.shape
    center = (int(dim[0]/2), int(dim[1]/2))
    cv2.arrowedLine(image, (center[0] + int(point.tuple[0] * scale), center[1] - int(point.tuple[1] * scale)),
                           (center[0] + int((point + vector).tuple[0] * scale), center[1] - int((point + vector).tuple[1] * scale)), color, 1)

def test_cloud_matching():
    base = get_random_vector_cloud(10)

    winname = "test"
    random_intensity = TrackbarParameter(1, "random_intensity", winname, value_factor=0.1)
    random_deviation = 0

    while True:
        img = empty_image((500, 500))
        random_rotator = TupleRotator3((random.random() * 10 * random_intensity.get_value(True), 0, 0))
        random_offset = TupleVector3((random.random() - 0.5, random.random() - 0.5, 0)) * 2 * random_intensity.get_value(True)
        
        compare = [v + TupleVector3((random.random()-0.5, random.random()-0.5, 0))*random_deviation for v in base]
        random.shuffle(compare)
        compare.pop(0)
        compare.pop(1)
        compare.pop(2)

        assert matching_clouds(base, compare, 0.1)

        compare = [v + random_offset + random_rotator for v in compare]

        offset, offset_rotation = get_vector_cloud_offset_2D_max(base, compare, 0.1, 0)

        print(offset, random_offset)
        print(offset_rotation, random_rotator)

        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))

        if offset is not None:
            result = [v - offset_rotation - offset for v in compare]
            draw_vector_cloud(img, result, (0, 0, 255))
            draw_vector_cloud(img, [offset], (0, 255, 255))

        v1, v2 = find_max_distance(compare)
        draw_vector(img, v2 - v1, v1, (255, 255, 255))


        cv2.putText(img, "base", (0, 490), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
        cv2.putText(img, "compare", (0, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0))
        cv2.putText(img, "result", (0, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))
        cv2.imshow(winname, img)

        cv2.waitKey(10)
        time.sleep(3)

def stress_test_cloud_matching():
    random_intensity = 1
    random_deviation = 0.2
    test_tolerance = 0.4
    test_max_iterations = 2
    successes = 0
    n = 100
    base_count = 15

    draw_time = 0
    start_time = time.time()

    for i in range(n):
        img = empty_image((400, 400))
        base = get_random_vector_cloud(base_count)
        random_rotator = TupleRotator3((random.random() * 10 * random_intensity, 0, 0))
        random_offset = TupleVector3((random.random() - 0.5, random.random() - 0.5, 0)) * 2 * random_intensity

        compare = [v + random_offset + random_rotator + (TupleVector3((random.random()-0.5, random.random()-0.5, 0))*2*random_deviation) for v in base[random.randint(0, int(base_count/2)):]]
        random.shuffle(compare)

        offset, offset_rotation = get_vector_cloud_offset_2D_max(base, compare, test_tolerance, test_max_iterations)
        if offset is not None:
            successes += 1
    
    print(f"Stress test completed. Average time: {(time.time()-start_time)/n*1000: .2f}ms success rate: {successes*100/n:.2f}%")

def test_matching_function():
    random_deviation = TrackbarParameter(0, "random", "test", value_factor=0.01)
    base = get_random_vector_cloud(10)

    while True:
        compare = [v + TupleVector3((random.random()-0.5, random.random()-0.5, 0))*2*random_deviation.get_value(True) for v in base]

        result = matching_clouds(base, compare, 2*random_deviation.get_value(True))
        print(result)

        img = empty_image((400, 400))
        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))
        cv2.putText(img, str(random_deviation.get_value(True)) + (" ok" if result else " nok"), (0, 390), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))

        cv2.imshow("test", img)
        cv2.waitKey(1)
        time.sleep(1)

def test_matching_poles():
    field = fc.Field()
    poles = field.generate_poles(5, 3)
    base = [p.distance for p in poles]

    while True:
        random_offset = TupleVector3((random.random() - 0.5, random.random() - 0.5, 0)) * 2
        random_rotation = TupleRotator3((random.random() * 100, 0, 0))
        random_deviation = 0.1

        rand_pos = base.copy()
        random.shuffle(rand_pos)
        compare = rand_pos[0:3]
        compare = [v + random_offset + random_rotation + TupleVector3((random.random() - 0.5, random.random() - 0.5, 0)) * 2 * random_deviation for v in compare]

        offset, offset_rotation = get_vector_cloud_offset_2D_tri(base, compare, 0.1, 0)

        img = empty_image((400, 400))
        draw_vector_cloud(img, base, (0, 255, 0))
        draw_vector_cloud(img, compare, (255, 0, 0))

        if offset is None:
            print("no offset found")
        else:
            print(offset, offset_rotation)
            result = [v - offset_rotation - offset for v in compare]
            draw_vector_cloud(img, result, (0, 0, 255))

        cv2.imshow("test", img)
        cv2.waitKey(1)
        time.sleep(1)


if __name__ == "__main__":
    # test_cloud_matching()
    # stress_test_cloud_matching()
    # test_matching_function()
    test_matching_poles()