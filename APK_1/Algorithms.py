from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from math import pi, atan2


class Algorithms:
    EPSILON = 1e-10 # tolerance for floating point comparisons

    def __init__(self): #
        pass

    def ray_crossing_pols(self, q: QPointF, polygons):
        '''
        q: QPointF
        polygons: list of tuples (QPolygonF, attributes)
        Returns list of results for each polygon, list of containing polygons and results corresponding to those polygons
        '''
        if not polygons:
            return [], []
            
        results = [] # list of results for each polygon
        containing_polygons = [] # list of containing polygons
        on_edge_polygons = []   # list of polygons where point is on edge
        
        for i, (pol, attributes) in enumerate(polygons): # iterate through each polygon
            status = self.ray_crossing(q, pol)
            results.append(status)
            
            if status == 1:
                containing_polygons.append(attributes)
            elif status == -1:
                on_edge_polygons.append(attributes)
                
        combined_result = 0
        if 1 in results:
            combined_result = 1
        elif -1 in results:
            combined_result = -1
            
        final_results = [combined_result] + results
        
        if combined_result == 1:
            return final_results, containing_polygons
        elif combined_result == -1:
            return final_results, on_edge_polygons
        else:
            return final_results, []

    def winding_number_pols(self, q: QPointF, polygons):
        if not polygons:
            return [], []
            
        results = []
        containing_polygons = []
        on_edge_polygons = []
        
        for i, (pol, attributes) in enumerate(polygons):
            status = self.winding_number(q, pol)
            results.append(status)
            
            if status == 1:
                containing_polygons.append(attributes)
            elif status == -1:
                on_edge_polygons.append(attributes)
                
        combined_result = 0
        if 1 in results:
            combined_result = 1
        elif -1 in results:
            combined_result = -1
            
        final_results = [combined_result] + results
        
        if combined_result == 1:
            return final_results, containing_polygons
        elif combined_result == -1:
            return final_results, on_edge_polygons
        else:
            return final_results, []

    def get_bounding_box(self, polygon):
        if len(polygon) == 0:
            return (0, 0, 0, 0)

        x_min = x_max = polygon[0].x()
        y_min = y_max = polygon[0].y()

        for i in range(1, len(polygon)):
            x_min = min(x_min, polygon[i].x())
            x_max = max(x_max, polygon[i].x())
            y_min = min(y_min, polygon[i].y())
            y_max = max(y_max, polygon[i].y())

        return (x_min, y_min, x_max, y_max)

    def ray_crossing(self, q: QPointF, pol: QPolygonF):
        """
        Ray crossing algorithm with even-odd rule and determinant based edge detection.
        Returns: 1 if inside, -1 if on edge/vertex, 0 if outside
        """
        n = len(pol)
        bbox = self.get_bounding_box(pol)
        if (q.x() < bbox[0] - self.EPSILON or
                q.x() > bbox[2] + self.EPSILON or
                q.y() < bbox[1] - self.EPSILON or
                q.y() > bbox[3] + self.EPSILON):
            return 0
            
        count = 0  # counter of intersections
                
        for i in range(n):
            p1 = pol[i]
            p2 = pol[(i+1)%n]
            if abs(p1.x() - q.x()) < self.EPSILON and abs(p1.y() - q.y()) < self.EPSILON:
                return -1
            v1 = (p1.x() - q.x(), p1.y() - q.y())
            v2 = (p2.x() - q.x(), p2.y() - q.y())
            
            # Compute determinant (cross product) and dot product
            cross_product = v1[0]*v2[1] - v1[1]*v2[0]
            dot_product = v1[0]*v2[0] + v1[1]*v2[1]
            
            # Check if point is on edge (collinear and between endpoints)
            if abs(cross_product) < self.EPSILON and dot_product <= 0:
                return -1
            
            # Check if ray (from q horizontally to the right) crosses this edge
            if ((p1.y() > q.y()) != (p2.y() > q.y())) and \
               (q.x() < p1.x() + (p2.x() - p1.x()) * (q.y() - p1.y()) / (p2.y() - p1.y())):
                count += 1  # intersection counter
        
        # even-odd rule: If the number of crossings is odd, the point is inside
        return 1 if count % 2 == 1 else 0
           
    def winding_number(self, q: QPointF, pol: QPolygonF):
        """
        Winding number algorithm using angle-based calculations.
        Returns: 1 if inside, -1 if on edge/vertex, 0 if outside
        """
        n = len(pol)
        bbox = self.get_bounding_box(pol)
        if (q.x() < bbox[0] - self.EPSILON or
                q.x() > bbox[2] + self.EPSILON or
                q.y() < bbox[1] - self.EPSILON or
                q.y() > bbox[3] + self.EPSILON):
            return 0
        
        angle_sum = 0.0
        for i in range(n):
            p1 = pol[i]
            if abs(p1.x() - q.x()) < self.EPSILON and abs(p1.y() - q.y()) < self.EPSILON:
                return -1
            p2 = pol[(i+1)%n]
            
            v1 = (p1.x() - q.x(), p1.y() - q.y())
            v2 = (p2.x() - q.x(), p2.y() - q.y())
            
            # Compute determinant (cross product) and dot product
            det = v1[0]*v2[1] - v1[1]*v2[0]
            dot_product = v1[0]*v2[0] + v1[1]*v2[1]
            
            # Check if point is on edge (collinear and between endpoints)
            if abs(det) < self.EPSILON and dot_product <= 0:
                return -1
                
            angle = atan2(det, dot_product)
            angle_sum += angle
            
        if abs(abs(angle_sum) - 2*pi) < self.EPSILON:
            return 1
        
        return 0









