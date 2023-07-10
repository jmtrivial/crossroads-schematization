import math

class SimpleWay:
    
    # parameters (and attributes)
    # - n1 is the interior node, n2 is the exterior node  (centripetal orientation)
    # - edge_tags: tags from crdesc
    # - same_osm_orientation: boolean (is the OSM orientation similar)
    def __init__(self, n1, n2, edge_tags, same_osm_orientation):
        self.n1 = n1
        self.n2 = n2
        self.same_osm_orientation = same_osm_orientation
        self.edge_tags = edge_tags


    def has_sidewalk(self):
        return SimpleWay.is_number(self.edge_tags["left_sidewalk"]) or SimpleWay.is_number(self.edge_tags["right_sidewalk"])


    def is_number(value):
        if isinstance(value, str):
            return value != ""
        else:
            return not math.isnan(value)

    def get_sidewalk_id(self):
        if SimpleWay.is_number(self.edge_tags["left_sidewalk"]):
            return self.edge_tags["left_sidewalk"]
        elif SimpleWay.is_number(self.edge_tags["right_sidewalk"]):
            return self.edge_tags["right_sidewalk"]
        else:
            return ""

    def has_sidewalks_both_sides(self):
        return SimpleWay.is_number(self.edge_tags["left_sidewalk"]) and SimpleWay.is_number(self.edge_tags["right_sidewalk"])


    def get_initial_edge_id(self):
        if self.same_osm_orientation:
            return self.get_edge_id()
        else:
            return self.get_edge_id_reverse()


    def get_edge_id(self):
        return str(self.n1) + ";" + str(self.n2)


    def get_edge_id_reverse(self):
        return str(self.n2) + ";" + str(self.n1)
