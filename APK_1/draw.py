from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtGui import QMouseEvent, QPaintEvent
from PyQt6.QtWidgets import *
import geopandas as gpd
import numpy as np
import os
import traceback

class Draw(QWidget):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__q = QPointF(100.0, 1000.0)
        self.__result = -2  # -2: not analyzed yet, -1: on edge/vertex, 0: outside, 1: inside
        self.__polygons = []  # list of tuples (QPolygonF, attributes_dict) that will contain the imported polygons
        self.__gdf = None  # GeoPandas -   GeoDataFrame
        self.__min_max = [0, 0, 10, 10]
        self.__containing_polygon = []  # list to store attributes of polygon that will contain the point
        self.__highlighted_polygons = []  # polygons to highlight

    def loadData(self, filename=None):   
        '''this function loads the data from a shapefile using GeoPandas and returns a GeoDataFrame'''
        if not filename:
            filename, _ = QFileDialog.getOpenFileName(self, "Open file", "", "Shapefile (*.shp);;GeoJSON (*.geojson);;All files (*.*)")
        
        if not filename or not os.path.isfile(filename):
            return False
        
        try:

            self.__gdf = gpd.read_file(filename) #reads asoc. files as dbf, shx as well
            if self.__gdf is None or len(self.__gdf) == 0:
                print("No data loaded from file")
                return False
            print(f"Successfully loaded {len(self.__gdf)} features")
            
            # get the bounds (min_x, min_y, max_x, max_y) of the imported features aggregate
            total_bounds = self.__gdf.total_bounds
            self.__min_max = [
                total_bounds[0],  # min_x
                total_bounds[1],  # min_y
                total_bounds[2],  # max_x
                total_bounds[3]   # max_y
            ]
            # process the data and update display
            self.resizeData()
            return True
        except Exception as e:
            traceback.print_exc()
            QMessageBox.critical(self, "Error", f"Failed to load file: {str(e)}")
            return False
        
    # function for rescaling data to fit canvas  
    def resizeData(self):
        # get widget dimensions
        width = self.width() or 800 # in case of no size, set to 800x600
        height = self.height() or 600
        # clear existing polygons to prevent mixing
        self.__polygons = []
        
        # extract the x_scale and y_scale with the use of the min_max values. threat of division by zero averted in case values are too small
        x_scale = 1.0 if abs(self.__min_max[2] - self.__min_max[0]) < 1e-10 else self.__min_max[2] - self.__min_max[0]
        y_scale = 1.0 if abs(self.__min_max[3] - self.__min_max[1]) < 1e-10 else self.__min_max[3] - self.__min_max[1]
        
        # process each geometry in the GeoDataFrame
        try:
            for _, row in self.__gdf.iterrows():
                geom = row.geometry
                if geom is None:
                    continue
                
                # extract attributes except geometry
                attributes = row.drop('geometry').to_dict()
                    
                # handle different geometry types
                if geom.geom_type == 'Polygon':
                    self.__add_polygon(geom, width, height, x_scale, y_scale, attributes)
                elif geom.geom_type == 'MultiPolygon':
                    for poly in geom.geoms:
                        self.__add_polygon(poly, width, height, x_scale, y_scale, attributes)
                elif geom.geom_type in ['LineString', 'MultiLineString', 'Point', 'MultiPoint']: # skip non polygons 
                    print(f"Skipping {geom.geom_type} geometry ")
                
            print(f"Created {len(self.__polygons)} polygons")
            if len(self.__polygons) == 0:
                print("No polygons loaded")
            
            # show the loaded polygons
            self.repaint()
        except Exception:
            pass
    
    def __add_polygon(self, geom, width, height, x_scale, y_scale, attributes=None):
        """Helper method to convert a geometry to QPolygonF and add it to the polygons list"""
        try:
            polygon = QPolygonF() # intiaal
            
            # for Polygon geometries, get the exterior coordinates
            if not geom.exterior:
                print(f"Warning: Polygon has no exterior ring")
                return
                
            coords = np.array(geom.exterior.coords)
            
            for point in coords:
                # shift the coordinates to the center of the canvas, scale to fit canvas, and add a small margin just in case
                x = ((point[0] - self.__min_max[0]) / x_scale * (width * 0.9)) + width * 0.05
                y = (height - (point[1] - self.__min_max[1]) / y_scale * (height * 0.9)) - height * 0.05 # invert axis
                
                polygon.append(QPointF(x, y))
            
            if not polygon.isEmpty() and polygon.size() >= 3:
                # store the polygon with its attributes as a tuple
                self.__polygons.append((polygon, attributes or {}))
            else:
                print(f"Warning: Skipped polygon with {polygon.size()} points")
                
        except Exception:
            pass

    def mousePressEvent(self, e: QMouseEvent):
        ''' 
        e: QMouseEvent, effectively does the "clear" function, analysis resets once the user clicks anywhere on the canvas
        '''
        x = e.position().x()
        y = e.position().y()

        # set point position
        self.__q.setX(x)
        self.__q.setY(y)
        self.__result = -2  # not analyzed yet
        # clear containing polygons
        self.__containing_polygon = []
        # clear highlighted polygons
        self.__highlighted_polygons = []

        self.repaint()

    def paintEvent(self, e: QPaintEvent):
        # create new graphic object
        qp = QPainter(self)

        # draw polygons from shapefile
        if self.__polygons:
            # draw all polygons with default style
            qp.setPen(Qt.GlobalColor.darkRed)
            qp.setBrush(Qt.GlobalColor.yellow)
            for i, (polygon, _) in enumerate(self.__polygons):
                if i not in self.__highlighted_polygons:  # Skip highlighted polygons
                    if polygon and not polygon.isEmpty():
                        qp.drawPolygon(polygon)
            
            # draw highlighted polygons
            if self.__highlighted_polygons:
                qp.setPen(Qt.GlobalColor.darkBlue)
                qp.setBrush(QColor(100, 150, 255, 180))  # light blue
                for i in self.__highlighted_polygons:
                    if 0 <= i < len(self.__polygons):
                        polygon, _ = self.__polygons[i]
                        if polygon and not polygon.isEmpty():
                            qp.drawPolygon(polygon)


        # Color point based on result
        if self.__result == 1:  # Inside
            qp.setBrush(Qt.GlobalColor.green)
        elif self.__result == 0:  # Outside
            qp.setBrush(Qt.GlobalColor.red)
        elif self.__result == -1:  # On edge/vertex
            qp.setBrush(Qt.GlobalColor.blue)
        else:  # Not analyzed yet
            qp.setBrush(Qt.GlobalColor.white)

        # draw point
        r = 5
        qp.drawEllipse(int(self.__q.x() - r), int(self.__q.y() - r), 2 * r, 2 * r)
        # End drawing
        qp.end()

    def getQ(self):
        """Get the current point"""
        return self.__q

    def setResult(self, result):
        """Set analysis result"""
        self.__result = result
        self.repaint()
        
    def setContainingPolygons(self, polygons):
        """Set the list of polygon attributes that contain the point"""
        self.__containing_polygon = polygons
        
        # Find and highlight the containing polygons
        self.__highlighted_polygons = []
        for i, (_, attrs) in enumerate(self.__polygons):
            for containing_attrs in self.__containing_polygon:
                # Check if this polygon matches the containing polygon attributes
                if self.__attrs_match(attrs, containing_attrs):
                    self.__highlighted_polygons.append(i)
                    break
        
        self.repaint()
        
    def __attrs_match(self, attrs1, attrs2):
        """Helper method to check if two attribute dictionaries represent the same polygon"""
        # Simple check: if the dictionaries have some matching key-value pairs
        if not attrs1 or not attrs2:
            return False
            
        # Look for important identifiers
        keys_to_check = ['NAME', 'Name', 'name', 'ID', 'Id', 'id', 'LABEL', 'Label', 'label', 'FID', 'OBJECTID']
        for key in keys_to_check:
            if key in attrs1 and key in attrs2 and attrs1[key] == attrs2[key]:
                return True
                
        # If we find at least 3 matching key-value pairs, consider it a match
        matches = 0
        for key, value in attrs1.items():
            if key in attrs2 and attrs2[key] == value:
                matches += 1
                if matches >= 3:
                    return True
                    
        return False

    def getContainingPolygons(self):
        """Get the list of polygon attributes that contain the point"""
        return self.__containing_polygon

    def clearResults(self):
        """Clear analysis results"""
        self.__result = -2  # Not analyzed
        self.__status_message = "Click to place point"
        self.__containing_polygon = []
        self.__highlighted_polygons = []
        self.repaint()

    def clearAll(self):
        """Clear all data - point and geodata"""
        self.__q = QPointF(0.0, 0.0)
        self.__result = -2  # Not analyzed
        self.__status_message = "Click to place point"
        self.__containing_polygon = []
        self.__highlighted_polygons = []
        # Clear geodata
        self.__gdf = None
        self.__polygons = []
        self.repaint()

    def getPols(self):
        """Get all polygons"""
        return self.__polygons

