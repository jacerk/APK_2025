from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from math import *
# Import specific numpy functions instead of everything
import numpy as np
from numpy.linalg import svd



# Disclaimers:
# In developing this code Ai was used to assist in generating code code snippets from pseudocodes, explaining syntax and debugging. 
# The code is a result of collaborative efforts and should be treated as such.
class Algorithms:
    def __init__(self):
        pass
    
    def generalize_pols(self, polygons, method='MBR'):
        """
        Apply generalization to a list of polygons and calculates accuracy.

        Args:
            polygons: List of QPolygonF objects (original)
            method: 'MBR', 'PCA', 'LongestEdge', 'WallAverage', or 'WeightedBisector' generalization method

        Returns:
            Tuple: (List of generalized QPolygonF objects, average accuracy as float 0-1)
            

        """
        generalized_polygons = []
        accuracy_scores_1 = [] # For delta_sigma_1
        accuracy_scores_2 = [] # For delta_sigma_2
        percentage_scores_1 = [] # For percentage based on delta_sigma_1
        percentage_scores_2 = [] # For percentage based on delta_sigma_2


        for original_polygon in polygons:
            generalized = original_polygon # Default to original if error occurs
            sigma = 0.0 # Default sigma

            try:
                # Call generalization method and get polygon + sigma
                if method.upper() == 'MBR':
                    generalized, sigma = self.createMBR(original_polygon)
                elif method.upper() == 'PCA':
                    generalized, sigma = self.createBRPCA(original_polygon)
                elif method.upper() == 'LONGESTEDGE':
                    generalized, sigma = self.longestEdge(original_polygon)
                elif method.upper() == 'WALLAVERAGE':
                    generalized, sigma = self.wallAverage(original_polygon)
                elif method.upper() == 'WEIGHTEDBISECTOR':
                    generalized, sigma = self.weighted_bisector(original_polygon)
                else:
                    # If method not recognized, keep original and sigma=0
                    generalized = original_polygon
                    sigma = 0.0

                # calculate accuracy metrics
                # ensure valid polygons for accuracy check
                if not generalized.isEmpty() and generalized.size() >= 3 and not original_polygon.isEmpty() and original_polygon.size() >= 3:
                    # Get all four metrics
                    delta_sigma_1, delta_sigma_2, perc_1, perc_2 = self.calculate_accuracy(original_polygon, sigma)
                    accuracy_scores_1.append(delta_sigma_1)
                    accuracy_scores_2.append(delta_sigma_2)
                    percentage_scores_1.append(perc_1)
                    percentage_scores_2.append(perc_2)

            except Exception as e:
                print(f"Error generalizing polygon or calculating accuracy: {str(e)}")
                # Assign 0 accuracy on error
            generalized_polygons.append(generalized)

        # Calculate average accuracy metrics
        total_scores = len(accuracy_scores_1) # Should be same for all lists
        average_accuracy_1 = sum(accuracy_scores_1) / total_scores if total_scores > 0 else 0.0
        average_accuracy_2 = sum(accuracy_scores_2) / total_scores if total_scores > 0 else 0.0
        average_percentage_1 = sum(percentage_scores_1) / total_scores if total_scores > 0 else 0.0
        average_percentage_2 = sum(percentage_scores_2) / total_scores if total_scores > 0 else 0.0

        return generalized_polygons, average_accuracy_1, average_accuracy_2, average_percentage_1, average_percentage_2

    def calculate_accuracy(self, pol: QPolygonF, sigma: float):
        """
        Calculates accuracy metrics Δσ₁, Δσ₂, and corresponding percentages
        based on angular deviation relative to a given main direction (sigma).

        Args:
            pol: Original QPolygonF.
            sigma: Main direction angle in radians.

        Returns:
            Tuple: (delta_sigma_1, delta_sigma_2, percentage_1, percentage_2) as floats.
        """

        r_list = []
        n_pol = len(pol)

        for i in range(n_pol):
            # get segment points
            p1 = pol[i]
            p2 = pol[(i + 1) % n_pol]
            x1, y1 = p1.x(), p1.y()
            x2, y2 = p2.x(), p2.y()

            # Skip zero-length segments
            dx = x2 - x1
            dy = y2 - y1
            if abs(dx) < 1e-10 and abs(dy) < 1e-10:
                continue

            # calculate segment direction
            sigma_i = atan2(dy, dx)

            # calculate relative angle
            sigma_rel = sigma_i - sigma

            # Normalize 
            # Use fmod for better precision/handling ofe dge cases
            sigma_rel = fmod(sigma_rel + pi, 2 * pi) - pi
            if abs(sigma_rel + pi) < 1e-12: 
                sigma_rel = pi

            # calculate k_i and r_i
            # avoid division by zero if pi is exactly zero (highly unlikely)
            if abs(pi) < 1e-10:
                k_i = 0.0
            else:
                # ensure correct rounding behavior 
                k_i = round((2 * sigma_rel) / pi) # Round to nearest integer

            r_i = sigma_rel - k_i * (pi / 2)


            r_list.append(r_i)
        # Calculate mean r
        num_deviations = len(r_list)
        if num_deviations == 0:
             print("Warning: r_list empty despite passing initial check.")
             return 0.0, 0.0, 0.0, 0.0
        r_mean = sum(r_list) / num_deviations

        # Calculate Δσ₁ (average absolute deviation)
        sum_abs_diff = sum(abs(r_i - r_mean) for r_i in r_list)
        delta_sigma_1 = (pi / (2 * num_deviations)) * sum_abs_diff

        # Calculate Δσ₂ (root mean square deviation)
        sum_sq_diff = sum((r_i - r_mean)**2 for r_i in r_list)
        mean_sq_diff = sum_sq_diff / num_deviations
        delta_sigma_2 = (pi / 2) * sqrt(mean_sq_diff)

        # Define maximum possible deviation (heuristic based on r_i range of [-pi/4, pi/4])
        max_dev = pi / 4.0
        percentage_1 = 0.0
        percentage_2 = 0.0

        if max_dev > 1e-10: # Avoid division by zero
            # Percentage = 100 * (1 - deviation / max_deviation), clamped at 0
            percentage_1 = max(0.0, (1.0 - (delta_sigma_1 / max_dev))) * 100.0
            percentage_2 = max(0.0, (1.0 - (delta_sigma_2 / max_dev))) * 100.0

        return delta_sigma_1, delta_sigma_2, percentage_1, percentage_2

    def get2VectorsAngle(self, p1: QPointF, p2: QPointF, p3: QPointF, p4:QPointF):
        # Compute angle between two vectors
        ux = p2.x() - p1.x()
        uy = p2.y() - p1.y()
        
        vx = p4.x() - p3.x()
        vy = p4.y() - p3.y()
        
        # Dot product
        uv = ux*vx + uy*vy
        
        # Norms u, v
        norm_u = sqrt(ux**2 + uy**2)
        norm_v = sqrt(vx**2 + vy**2)
        
        # Check for zero norm to avoid division by zero
        if norm_u < 1e-10 or norm_v < 1e-10:
            return 0
            
        #Compute argument
        arg = uv/(norm_u*norm_v)
        
        #Correct argument to interval <-1, 1>
        # Use Python's built-in min/max, not numpy's
        if arg < -1:
            arg = -1
        if arg > 1:
            arg = 1
        
        return acos(arg)
    
    
    
    def createCH(self, polygon: QPolygonF):
        """
        Create convex hull using Jarvis Scan method
        """
        ch = QPolygonF()
        
        # Create pivot
        q = min(polygon, key = lambda k: k.y())
        pj = q
        
        # Create point ph1
        px = min(polygon, key = lambda k: k.x())
        pj1 = QPointF(px.x(), pj.y())
        
        # Add pivot to ch
        ch.append(pj)
        
        # Process all points
        while True:
            #Initialize maximum and its index
            phi_max = 0
            idx_max = -1
            
            for i in range(len(polygon)):
                #Different points
                if (pj != polygon[i]):
                    #Compute angle
                    phi = self.get2VectorsAngle(pj, pj1, pj, polygon[i])
            
                    #Update maximum
                    if phi > phi_max:
                        phi_max = phi
                        idx_max = i
            
            #Add point to ch
            ch.append(polygon[idx_max])
            
            #Update indices
            pj1 = pj
            pj = polygon[idx_max]
            
            #Stop
            if pj == q:
                break
        
        return ch  
    
    
    def rotate(self, pol: QPolygonF, sigma):
        #Rotate polygon by angle sigma
        
        # Handle empty polygon
        if pol.isEmpty():
            return pol
            
        try:
            pol_r = QPolygonF()
            
            #Process points one by one
            for p in pol:
                
                #Rotate polygon point
                x_r = p.x()*cos(sigma) - p.y()*sin(sigma)
                y_r = p.x()*sin(sigma) + p.y()*cos(sigma)
                
                #Create point
                p_r = QPointF(x_r, y_r)
                
                #Add point to polygon
                pol_r.append(p_r)
            
            return pol_r
            
        except Exception as e:
            print(f"Error in rotate: {str(e)}")
            return pol
    
    def createMMB(self, pol:QPolygonF):
        # Create min-max box
        mmb = QPolygonF()
        
        # Handle empty polygon
        if pol.isEmpty() or pol.size() < 3:
            return pol, 0.0
        
        #Find extreme coordinates
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        
        for point in pol:
            if point.x() < min_x:
                min_x = point.x()
            if point.x() > max_x:
                max_x = point.x()
            if point.y() < min_y:
                min_y = point.y()
            if point.y() > max_y:
                max_y = point.y()
        
        # Check if we have valid bounds
        if min_x == float('inf') or max_x == float('-inf') or min_y == float('inf') or max_y == float('-inf'):
            return pol, 0.0
            
        # Check if the bounding box is too small (degenerate)
        if abs(max_x - min_x) < 1e-10 or abs(max_y - min_y) < 1e-10:
            return pol, 0.0
        
        #Compute area
        area = (max_x - min_x) * (max_y - min_y)
        
        #Create min-max box vertices
        v1 = QPointF(min_x, min_y)
        v2 = QPointF(max_x, min_y)
        v3 = QPointF(max_x, max_y)
        v4 = QPointF(min_x, max_y)
        
        #Create min-max box polygon
        mmb.append(v1)
        mmb.append(v2)
        mmb.append(v3)
        mmb.append(v4)
        
        return mmb, area
    
    
    def getArea(self, pol: QPolygonF):
        # Compute area of a polygon
        
        # Handle empty or degenerate polygon
        if pol.isEmpty() or pol.size() < 3:
            return 0.0
            
        try:
            area = 0
            n = len(pol)
            
            #Process vertices one by one
            for i in range(n):
                area += pol[i].x()*(pol[(i+1)%n].y()-pol[(i-1+n)%n].y())
                
            return abs(area)/2
            
        except Exception as e:
            print(f"Error in getArea: {str(e)}")
            return 0.0
    
    def distance_points(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        x1, y1 = point1
        x2, y2 = point2
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
    def resizeRectangle(self, building:QPolygonF, mbr:QPolygonF):
        # Resizing rectangle to match the building area
        
        # Check for valid inputs
        if building.isEmpty() or mbr.isEmpty() or mbr.size() != 4:
            return mbr
        
        try:
            mbr_res = QPolygonF()
                        
            #Compute k
            Ab = self.getArea(building)
            A = self.getArea(mbr)
            
            # Prevent division by zero or negative values
            if A <= 0 or Ab <= 0:
                return mbr
                
            k = Ab / A
            
            # Compute centroid
            x_t = 0.25*(mbr[0].x()+mbr[1].x()+mbr[2].x()+mbr[3].x())
            y_t = 0.25*(mbr[0].y()+mbr[1].y()+mbr[2].y()+mbr[3].y())
            
            #Compute vectors
            v1_x = mbr[0].x() - x_t
            v1_y = mbr[0].y() - y_t
            
            v2_x = mbr[1].x() - x_t
            v2_y = mbr[1].y() - y_t
            
            v3_x = mbr[2].x() - x_t
            v3_y = mbr[2].y() - y_t
            
            v4_x = mbr[3].x() - x_t
            v4_y = mbr[3].y() - y_t
            
            # Make sure k is a positive number and prevent NaN results
            if k < 0:
                k = abs(k)
            if k != k:  # Check for NaN
                k = 1.0
                
            k_sqrt = sqrt(max(k, 0))
            
            #Compute coordinates of resized points
            v1_xr = x_t + v1_x * k_sqrt
            v1_yr = y_t + v1_y * k_sqrt
            
            v2_xr = x_t + v2_x * k_sqrt
            v2_yr = y_t + v2_y * k_sqrt
            
            v3_xr = x_t + v3_x * k_sqrt
            v3_yr = y_t + v3_y * k_sqrt
            
            v4_xr = x_t + v4_x * k_sqrt
            v4_yr = y_t + v4_y * k_sqrt
            
            #Create new vertices
            v1_res = QPointF(v1_xr, v1_yr)
            v2_res = QPointF(v2_xr, v2_yr)
            v3_res = QPointF(v3_xr, v3_yr)
            v4_res = QPointF(v4_xr, v4_yr)
            
            #Add vertices to the resized mbr
            mbr_res.append(v1_res)
            mbr_res.append(v2_res)
            mbr_res.append(v3_res)
            mbr_res.append(v4_res)
            
            return mbr_res
            
        except Exception as e:
            print(f"Error in resizeRectangle: {str(e)}")
            return mbr
    

    def createMBR(self, building: QPolygonF):
        """
        Create a minimum bounding rectangle (MBR) for a given polygon.
        Args:
            building: QPolygonF object representing the building polygon.
        Returns:
            Tuple: (QPolygonF object representing the MBR, angle in radians).
        """
        
        sigma = 0.0 # Default sigma
        #Create convex hull
        ch = self.createCH(building)

        #Initilize MBR a its area
        mmb_min, area_min = self.createMMB(ch)

        #Browse all edges
        n = len(ch)

        for i in range(n): 
            
            #coordinate differences
            dx = ch[(i+1)%n].x() - ch[i].x()
            dy = ch[(i+1)%n].y() - ch[i].y()
            
            # Skip segments with zero length
            if abs(dx) < 1e-10 and abs(dy) < 1e-10:
                continue
            
            #Compute direction
            sigma = atan2(dy, dx)
            
            #Rotate polygon
            ch_r = self.rotate(ch, -sigma)
            
            #Compute min-max box 
            mmb, area = self.createMMB(ch_r)
            
            # Skip invalid min-max boxes
            if mmb.isEmpty() or area <= 0:
                continue
            
            #Update minimum
            if area < area_min:
                area_min = area 
                mmb_min = mmb
                sigma_min = sigma
                
        #Resize rectangle
        mmb_min_res = self.resizeRectangle(building, mmb_min)

        #Convert min-max box with the minimum area to MBR
        mbr_final = self.rotate(mmb_min_res, sigma_min)
        return mbr_final, sigma_min # Return polygon and main direction

    def longestEdge(self, pol: QPolygonF):
        '''
        Simplify building using longest edge method, finds longest edge, finds the angle of the edge, and uses it to find the main angle
        '''
        
        n = len(pol)
        longest_edge_sq = 0 # Intialize for squared length
        angle = 0  # default angle

        for i in range(n):
            p1 = pol[i] # current point
            p2 = pol[(i+1) % n] # next point 
            dx = p2.x() - p1.x()
            dy = p2.y() - p1.y()
            edge_sq = dx**2 + dy**2 # squared length of the edge

            #find longest edge 
            if edge_sq > longest_edge_sq:
                longest_edge_sq = edge_sq
                # find slope of the longest edge
                angle = atan2(dy, dx) 

        try:
            # rotate building to align longest edge with x-axis
            pol_r = self.rotate(pol, -angle)

            # dind min max box of the rotated polygon
            mmb, area = self.createMMB(pol_r)

            # rotate min max box back
            er = self.rotate(mmb, angle) # er = enclosing rectangle 
       
            # esize the rectangle 'er' to match the area of the original polygon 
            res = self.resizeRectangle(pol, er) # res = resized rectangle


            return res, angle # Return polygon and main direction (angle)
        except Exception as e: # except made by Ai, modified by the author
            print(f"Error during longestEdge processing: {e}")
            # Return original polygon and the calculated angle (even if error occurred later)
            # Or return 0.0 if angle calculation itself might have failed? Let's return calculated angle.
            return pol, angle # Return original polygon and potentially valid angle
    def wallAverage(self, pol: QPolygonF):
        '''
        Simplify building using wall average method, computes the average direction of the walls and creates a min-max box
        '''
        
        # compute slope of the intial edge edge 
        dx = pol[1].x() - pol[0].x()
        dy = pol[1].y() - pol[0].y()
        angle = atan2(dy, dx)
        

        n = len(pol) # polygon lenght
        
        # innitialize wall sum
        avg_rem= 0
        
        for i in range(1,n): # start from the second edge
            # compute angle for current edge
            dx_i = pol[(i+1) % n].x() - pol[i].x() # change in x
            dy_i = pol[(i+1) % n].y() - pol[i].y() # change in y
            angle_i = atan2(dy_i, dx_i) 
            
            # compute angle difference
            angle_diff = abs(angle_i - angle)
            
            
            if angle_diff < 0:
                angle_diff += 2*pi 
                
            # fraction by pi/2
            ki = round(2*angle_diff/pi) 
            
            # compute remainder for this edge to the nearest pi/2
            re_i = angle_diff - (ki*(pi/2))
            
            # update average remainder
            avg_rem += re_i
            
        # compute average direction
        angle_ave = angle + (avg_rem/n)
        
        # rotate building
        pol_r = self.rotate(pol, -angle_ave)
        # find min max box
        mmb, area = self.createMMB(pol_r)
        # rotate min max box
        er = self.rotate(mmb, angle_ave)
        # resize
        res = self.resizeRectangle(pol, er)
        return res, angle_ave # Return polygon and average angle

    # get simplified building using Weighted Bisector algorithm (Using 2 longest diagonals, ignoring intersections)
    def weighted_bisector(self, pol: QPolygonF):
        """
        Weighted bisector with all diagonals
        Simplify building using a weighted bisector approach based on the
        two longest diagonals, regardless of whether they intersect.
        """


        avg_angle = 0.0 # Default angle

        try:
            # Initialize list for all diagonals 
            all_diagonals_data = []
            n = len(pol)

            # find diagonals
            for i in range(n):
                for j in range(i + 2, n):
                    # Skip adjacent vertices and the wrap-around case i=0, j=n-1
                    if j == (i + 1) % n or i == (j + 1) % n:
                        continue
                    if i == 0 and j == n - 1:
                        continue

                    p1 = pol[i]
                    p2 = pol[j]

                    # Calculate length and add directly
                    dx = p2.x() - p1.x()
                    dy = p2.y() - p1.y()
                    length_sq = dx * dx + dy * dy
                    all_diagonals_data.append((p1, p2, length_sq)) # Add diagonal data

            # Sort by length squared (descending)
            all_diagonals_data.sort(key=lambda item: item[2], reverse=True)

            # Select top 1 or 2 diagonals
            diagonals_to_use = all_diagonals_data[:2] # Take up to the first two

            # itialize sum vectors
            sum_vector_x = 0.0
            sum_vector_y = 0.0

            for d_p1, d_p2, d_length_sq in diagonals_to_use:
                dx = d_p2.x() - d_p1.x()
                dy = d_p2.y() - d_p1.y()
                length = sqrt(d_length_sq)
                if length < 1e-9: continue

                angle = atan2(dy, dx)
                sum_vector_x += length * cos(angle) # Weight by length
                sum_vector_y += length * sin(angle) # Weight by length

        
            avg_angle = atan2(sum_vector_y, sum_vector_x)

            # Create and resize bounding box
            pol_r = self.rotate(pol, -avg_angle)
            mmb, area = self.createMMB(pol_r)
            er = self.rotate(mmb, avg_angle)
            res_pol = self.resizeRectangle(pol, er)

            return res_pol, avg_angle

        except Exception as e:
            print(f"Error in weighted_bisector: {str(e)}")


    def createBRPCA(self, building: QPolygonF):
        """
        Create a bounding rectangle using PCA (Principal Component Analysis).
        Args:
            building: QPolygonF object representing the building polygon.
        Returns:
            Tuple: (QPolygonF object representing the bounding rectangle, angle in radians).
        """ 
        sigma = 0.0 # Default sigma

        x, y = [], []

        #Convert points to coordinates
        for p in building:
            x.append(p.x())
            y.append(p.y())
            
        #Create A
        A = np.array([x, y])
            
        #Covariance matrix
        C = np.cov(A)

        #Singular value decomposition
        U, S, V = svd(C, full_matrices=False)
        
        #Direction of the principal vector
        sigma = atan2(V[0, 1], V[0, 0])

        #Rotate polygon
        building_r = self.rotate(building, -sigma)

        #Compute min-max box 
        mmb, area = self.createMMB(building_r)
        #Resize rectangle
        mmb_res = self.resizeRectangle(building, mmb)

        #Convert min-max box with the minimum area to MBR
        pca_res_pol = self.rotate(mmb_res, sigma)
        return pca_res_pol, sigma # Return polygon and calculated sigma
