import math
from math import cos, sin
from shapely.geometry import LineString, Point
import shapely.ops

from .. import utils as u

class NormalizedBranch:

    def __init__(self, branch):
        self.branch = branch
        self.score = -1
        self.ratio_parallel_axis = 0.01
        self._initial_angle = self.branch.get_middle_way_orientation()
        self.middle_line = branch.middle_line
        self._parallel_branches = []
        self.new_middle_line = None
        self._orientation_id = None
        self.__matrix = None

        self.proximity_threshold = 10


    def initial_angle(self):
        return self._initial_angle

    def get_new_middle_line(self):
        if self.new_middle_line:
            return self.new_middle_line
        else:
            return self.middle_line


    def is_aligned(self, branch, initial = True):
        if initial:
            sml = self.middle_line
            bml = branch.middle_line
        else:
            sml = self.get_new_middle_line()
            bml = branch.get_new_middle_line()
        # extends the middle line
        middle_axis = u.Utils.extends_edge(sml.coords)
        width = self.branch.get_mean_width()

        d1 = middle_axis.distance(Point(bml.coords[0]))
        d2 = middle_axis.distance(Point(bml.coords[1])) 

        return d1 < width and d2 < width



    def compute_similarity_score(self, branches, angular_discretization):

        if angular_discretization > 0:
            self.angular_discretization = angular_discretization
            self.range = 2 * math.pi / angular_discretization
        
            self.score = 0.0

        for b in branches:
            if b.branch.id != self.branch.id:
                a = b.initial_angle()

                if angular_discretization > 0:
                    # for this branch, compute the relative angle with the best 
                    # aligned angle
                    difference = a - self._initial_angle
                    diff_modulo = u.Utils.modulo(difference, self.range)
                    if diff_modulo > self.range / 2:
                        diff_modulo -= self.range

                    width_axis = self.branch.get_mean_width() * b.branch.get_mean_width()

                    self.score += abs(diff_modulo) / width_axis

                if self.is_aligned(b):
                    self._parallel_branches.append((b, a))

        if  angular_discretization > 0 and len(self._parallel_branches) >= 1:
            self.score *= self.ratio_parallel_axis


    def compute_main_orientation(self):
        result = self._initial_angle
        # compute mean
        for b, a in self._parallel_branches:
            if abs(self._initial_angle - a) < math.pi / 2:
                result += a
            else:
                a2 = u.Utils.angle_modulo(a + math.pi)
                if a2 > math.pi:
                    a2 -= 2 * math.pi
                result += a2

        return result / (len(self._parallel_branches) + 1)


    def reorient_middle_line(self, angle):
        p1 = self.middle_line.coords[0]
        p2 = self.middle_line.coords[1]

        distance = u.Utils.edge_length(p1, p2)
        v = [distance * math.cos(angle), -distance * math.sin(angle)]
        new_p2 = [p1[0] + v[0], p1[1] + v[1]]
        
        return LineString([p1, new_p2])


    def get_possible_angles(self, main_orientation):
        return [u.Utils.angle_modulo(main_orientation + nb * self.range) + 2 * math.pi for nb in range(0, self.angular_discretization)]


    def is_compatible_with(self, id, b):
        return id != b._orientation_id or b.middle_line.distance(self.middle_line) > self.proximity_threshold


    def is_available_orientation(self, id, other_branches):
        return id not in [b._orientation_id for b in other_branches if not self.is_compatible_with(id, b)]

    
    def set_new_orientation(self, id, angle):
        self.new_middle_line = self.reorient_middle_line(angle)
        self._orientation_id = id
        self.branch.middle_line = self.new_middle_line


    def set_best_orientation(self, first, second, angle, center):
        d1 = u.Utils.angle_distance(first[1], angle)
        d2 = u.Utils.angle_distance(second[1], angle)
        threshold = self.range / 4

        if d1 > threshold and d2 > threshold:
            # choose the best wrt the center of the crossing
            p1 = self.middle_line.coords[0]
            angle_center = u.Utils.get_bearing_radian(center.coords[0], p1)

            if u.Utils.angle_distance(angle_center, angle) > self.range / 2:
                angle_center_norm = u.Utils.angle_modulo(angle_center) + 2 * math.pi
                angle_norm = u.Utils.angle_modulo(angle_center) + 2 * math.pi
                if angle_center_norm > angle_norm:
                    self.set_new_orientation(first[0], first[1])
                    return
                elif d2 < threshold:
                    self.set_new_orientation(second[0], second[1])
                    return

        # not a strong angle
        if d1 < d2:
            self.set_new_orientation(first[0], first[1])
        else:
            self.set_new_orientation(second[0], second[1])


    def adjust_angle(self, already_adjusted, main_orientation, center):
        angle = u.Utils.angle_modulo(self._initial_angle) + 2 * math.pi
        possible_angles = self.get_possible_angles(main_orientation)

        elements = [(i, a) for i, a in enumerate(possible_angles) if u.Utils.angle_distance(a, angle) < self.range and u.Utils.angle_distance(a + self.range, angle) <= self.range]
        if len(elements) == 0:
            print("angle distance", [(i, u.Utils.angle_distance(a, angle)) for i, a in enumerate(possible_angles)])
            print("Error: no matching angle.", "possible_angles", possible_angles, "angle", angle, "range", self.range)
            return False

        id_first, first = elements[0]
        
        id_second = (id_first + 1) % len(possible_angles)
        second = possible_angles[id_second]

        available_first = self.is_available_orientation(id_first, already_adjusted)
        available_second = self.is_available_orientation(id_second, already_adjusted)

        if available_first:
            if available_second:
                # choose the best one
                self.set_best_orientation((id_first, first), (id_second, second), angle, center)
            else:
                self.set_new_orientation(id_first, first)
        else:
            if available_second:
                self.set_new_orientation(id_second, second)
            else:
                return False

        return True

    def adjust_aligned(self):
        nb = len(self._parallel_branches)
        if nb > 1:
            print("Cannot align with multiple branches")
        elif nb == 1 and self.is_aligned(self._parallel_branches[0][0], False):
            ml = self.get_new_middle_line()
            other_axis = u.Utils.extends_edge(self._parallel_branches[0][0].get_new_middle_line().coords)
            point = Point(ml.coords[0])
            projected = shapely.ops.nearest_points(other_axis, point)[0]
            vector = [c / 2 for c in u.Utils.vector(point, projected)]

            self.new_middle_line = LineString([u.Utils.translate(ml.coords[0], vector),
                                              u.Utils.translate(ml.coords[1], vector)])
            self.branch.middle_line = self.new_middle_line


    def get_matrix_rotation_angle(self):
        edge1 = self.middle_line.coords
        edge2 = self.get_new_middle_line().coords
        
        bearing1 = u.Utils.get_bearing_radian(edge1[0], edge1[1])
        bearing2 = u.Utils.get_bearing_radian(edge2[0], edge2[1])
        return bearing2 - bearing1


    def get_matrix_translation_offset(self):
        edge1 = self.middle_line.coords
        edge2 = self.get_new_middle_line().coords
        return u.Utils.vector(edge1[0], edge2[0])

    # return rigid transformation matrix that rotate and translate wrt the new middle line
    def get_transformation_matrix(self):
        if self.__matrix is None:

            # REMARK: latitude coordinate is inverted (-> angle is inverted)
            angle = -self.get_matrix_rotation_angle()
            center = self.middle_line.coords[0]
            offset = self.get_matrix_translation_offset()
            
            # see rotation matrix in https://shapely.readthedocs.io/en/stable/manual.html#spatial-analysis-methods
            xoff = center[0] - center[0] * cos(angle) + center[1] * sin(angle)
            yoff = center[1] - center[0] * sin(angle) - center[1] * cos(angle)

            self.__matrix = [cos(angle), -sin(angle), sin(angle), cos(angle), xoff + offset[0], yoff + offset[1]]

        return self.__matrix
