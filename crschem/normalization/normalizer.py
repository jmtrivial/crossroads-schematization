import shapely.affinity
from shapely.geometry import Point

from .normalized_branch import NormalizedBranch

import numpy as np
from scipy.spatial import Delaunay
import crschem.utils as u

class Normalizer:

    def __init__(self, 
                 angular_discretization = 8, 
                 snap_aligned_streets = True,
                 stop_when_missing_direction = True):
        self.angular_discretization = angular_discretization
        self.stop_when_missing_direction = stop_when_missing_direction
        self.snap_aligned_streets = snap_aligned_streets


    def compute_similarity_scores(self):
        

        for i, b in enumerate(self.branches):
            b.compute_similarity_score(self.branches, self.angular_discretization)


    def compute_main_orientation(self):
        self.main_orientation = self.branches[0].compute_main_orientation()

    
    def sort_branches_by_similarity(self):
        self.branches = sorted(self.branches, key=lambda b: b.score)
    
    
    def normalize_branches(self, branches, center):
        self.branches = [NormalizedBranch(branches[b]) for b in branches]

        # compute similarity score between each node and the others
        self.compute_similarity_scores()

        # sort branches by similarity
        self.sort_branches_by_similarity()

        # compute main orientation
        self.compute_main_orientation()

        result = True

        # adjust each branch angle
        if self.angular_discretization != 0:
            previous = []
            for b in self.branches:
                a = b.adjust_angle(previous, self.main_orientation, center)
                if a is None:
                    print("WARNING: cannot adjust angles (an orientation is missing)")
                    if self.stop_when_missing_direction:
                        return False
                    result = False
                previous.append(b)

        # align streets
        if self.snap_aligned_streets:
            for b in self.branches:
                b.adjust_aligned()

        return result


    def adjust_nodes(self, osm_input):
        new_coords = {}
        branch_nodes = []

        # TODO: 
        # - ajouter des noeuds dans le Delaunay correspondant à l'extrémité des arêtes des branches (carrefour rue Blatin)
        # - prolonger plus loin les polybranches des arêtes (carrefour carré)
        # - agrandir un peu la zone de bounding box

        # for each point
        # if it's inside a branch
        # apply the corresponding transformation
        for b in self.branches:
            matrix = b.get_transformation_matrix()
            for n in b.branch.get_all_nodes():
                node = osm_input.nodes[n]
                new_loc = shapely.affinity.affine_transform(Point(node['x'], node['y']), matrix)
                new_coords[n] = [new_loc.x, new_loc.y]
                branch_nodes.append(n)

        # create bounding box nodes
        bound_nodes = u.Utils.bounding_box_nodes(osm_input)

        # then for all other points, we apply a continuous deformation

        # build a Delaunay triangulation of points inside branches (with original coordinates)
        points = np.array([[osm_input.nodes[n]["x"], osm_input.nodes[n]["y"]] for n in branch_nodes] + bound_nodes)
        tri = Delaunay(points)
        
        # for all the other points, compute their coordinate (triangle ID, and barycentric coordinates) in the original triangulation
        # then compute the new coordinate wrt the deformed triangulation
        other_points = [id for id in osm_input.nodes if id not in branch_nodes]
        n_other_points = np.array([(osm_input.nodes[n]['x'], osm_input.nodes[n]['y']) for n in other_points])                
        simplices = tri.find_simplex(n_other_points)

        other_points_simplices = [ (op, s, tri.simplices[s]) for op, s in zip(other_points, simplices) if s >= 0]

        for op, tr, ids in other_points_simplices:
            point = np.array([(osm_input.nodes[op]["x"], osm_input.nodes[op]["y"])])
            b = tri.transform[tr, :2].dot(np.transpose(point - tri.transform[tr, 2]))
            coords = np.c_[np.transpose(b), 1 - b.sum(axis=0)][0]
            new_tri = [new_coords[branch_nodes[n]] if n < len(branch_nodes) else bound_nodes[n - len(branch_nodes)] for n in ids]

            new_point = [sum([c * t[0] for c, t in zip(coords, new_tri)]), \
                         sum([c * t[1] for c, t in zip(coords, new_tri)])]

            osm_input.nodes[op]["x"] = new_point[0]
            osm_input.nodes[op]["y"] = new_point[1]


        # finally adjust branch points to their new coordinates
        for n in branch_nodes:
            osm_input.nodes[n]["x"] = new_coords[n][0]
            osm_input.nodes[n]["y"] = new_coords[n][1]

