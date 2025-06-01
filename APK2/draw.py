from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtGui import QMouseEvent, QPaintEvent, QWheelEvent
from PyQt6.QtWidgets import *
import geopandas as gpd 
import os
import traceback
import numpy as np

# This class is responsible for drawing the polygons on the canvas and navigation 
# (zooming and panning) using mouse events.
# Ai was used in helping to create navigation and scrolling, but the code was modified and improved by the author.

class Draw(QWidget):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        self.building = QPolygonF()
        self.building_simp = QPolygonF()
        self.__polygons = []  # list of tuples (QPolygonF, attributes_dict) that will contain the imported polygons
        self.__simplified_polygons = []  # list of simplified polygons
        self.__gdf = None  # GeoPandas -   GeoDataFrame
        self.__min_max = [0, 0, 10, 10]
        self.__display_loaded = True  # Flag to control display of loaded polygons
        self.__scale = 1.0 # Uniform scale factor
        self.__offset_x = 0.0 # X offset for centering
        self.__offset_y = 0.0 # Y offset for centering
        
        # Zoom/Pan attributes
        self.__zoom_factor = 1.0
        self.__pan_offset = QPointF(0.0, 0.0)
        self.__last_pan_pos = None # Store the last position during panning
        
        # Enable mouse tracking for panning even when button is not pressed initially
        self.setMouseTracking(True) 
        
    def loadData(self, filename=None):   
        '''this function loads the data from a shapefile using GeoPandas and returns them in GeoDataFrame'''
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
        W = self.width() or 800 # in case of no size, set to 800x600
        H = self.height() or 600
        # clear existing polygons to prevent mixing
        self.__polygons = []
        
        # Check if data exists
        if self.__gdf is None or self.__gdf.empty:
            self.repaint()
            return

        # Data dimensions
        min_x, min_y, max_x, max_y = self.__min_max
        data_width = max_x - min_x
        data_height = max_y - min_y

        # Handle zero dimensions
        if data_width < 1e-9 or data_height < 1e-9: # disclaimer: this check was made by AI
            print("Warning: Data has zero width or height.")
            # Optionally set default scale/offset or return
            self.__scale = 1.0
            self.__offset_x = W * 0.05
            self.__offset_y = H * 0.05
            self.repaint()
            return

        # drawing area dimensions (with 10% margin)
        draw_width = W * 0.9
        draw_height = H * 0.9
        margin_x = W * 0.05
        margin_y = H * 0.05

        # Calculate scale factors
        scale_x = draw_width / data_width
        scale_y = draw_height / data_height

        # Determine uniform scale
        self.__scale = min(scale_x, scale_y)

        # Calculate scaled data dimensions
        scaled_data_width = data_width * self.__scale
        scaled_data_height = data_height * self.__scale

        # Calculate centering offsets
        self.__offset_x = margin_x + (draw_width - scaled_data_width) / 2
        self.__offset_y = margin_y + (draw_height - scaled_data_height) / 2
        
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
                    self.__add_polygon(geom, attributes)
                elif geom.geom_type == 'MultiPolygon':
                    for poly in geom.geoms:
                        self.__add_polygon(poly, attributes)
                elif geom.geom_type in ['LineString', 'MultiLineString', 'Point', 'MultiPoint']: # skip non polygons 
                    print(f"Skipping {geom.geom_type} geometry ")
                
            print(f"Created {len(self.__polygons)} polygons")
            if len(self.__polygons) == 0:
                print("No polygons loaded")
            
            # Reset view when new data is loaded and resized
            self.resetView() 
            
            # show the loaded polygons
            self.repaint()
        except Exception:
            pass

    def __add_polygon(self, geom, attributes=None):
        """helper method to convert a geometry to QPolygonF and add it to the polygons list"""
        try:
            polygon = QPolygonF() # initial
            
            # for Polygon geometries, get the exterior coordinates
            if not geom.exterior:
                print(f"Warning: Polygon has no exterior ring")
                return
                
            coords = np.array(geom.exterior.coords)
            min_x, min_y, max_x, max_y = self.__min_max # Unpack min_max
            
            for point in coords:
                # Apply uniform scale and centering offsets
                # Transform X: scale relative to min_x and add offset
                x = self.__offset_x + (point[0] - min_x) * self.__scale
                # Transform Y: scale relative to max_y (to invert), then add offset
                y = self.__offset_y + (max_y - point[1]) * self.__scale
                
                polygon.append(QPointF(x, y))
            
            if not polygon.isEmpty() and polygon.size() >= 3:
                # store the polygon with its attributes as a tuple
                self.__polygons.append((polygon, attributes or {}))
            else:
                print(f"Warning: Skipped polygon with {polygon.size()} points")
                
        except Exception:
            pass

    def __handle_panning(self, current_pos):
        """
        Helper method to handle panning logic
        """
        if self.__last_pan_pos is not None:
            delta = current_pos - self.__last_pan_pos
            self.__pan_offset += delta
            self.__last_pan_pos = current_pos
            self.repaint()
            return True
        return False

    def mousePressEvent(self, e: QMouseEvent):
        '''
        Handle mouse press events for panning and zooming.
        Uses left button for panning instead of middle button.
        '''
        if e.button() == Qt.MouseButton.LeftButton:
            # Store initial position for panning
            self.__last_pan_pos = e.position()
            e.accept()  # Indicate the event was handled
        else:
            e.ignore()  # Ignore other buttons

    def mouseMoveEvent(self, e: QMouseEvent):
        '''
        Handle mouse move events for panning.
        Uses left button for panning instead of middle button.
        '''
        if e.buttons() == Qt.MouseButton.LeftButton:
            if self.__handle_panning(e.position()):
                e.accept()
                return
        e.ignore()

    def mouseReleaseEvent(self, e: QMouseEvent):
        '''
        Handle mouse release events for panning.
        Uses left button for panning instead of middle button.
        '''
        # reset last pan position when left button is released
        if e.button() == Qt.MouseButton.LeftButton and self.__last_pan_pos is not None:
            self.__last_pan_pos = None
            e.accept()
        else:
            e.ignore()

    def wheelEvent(self, e: QWheelEvent):
        '''
        Handle mouse wheel events for zooming in and out.
        The zoom factor is adjusted based on the wheel rotation.
        '''
        angle_delta = e.angleDelta().y()
        if angle_delta > 0:
            zoom_factor_change = 1.1  # Zoom in
        else:
            zoom_factor_change = 1 / 1.1  # Zoom out

        # store old zoom factor
        old_zoom_factor = self.__zoom_factor
        
        # update zoom factor
        self.__zoom_factor *= zoom_factor_change
        
        # get mouse position relative to the widget
        mouse_pos = e.position()
        
        # adjust pan offset to keep the point under the mouse cursor stationary
        # Formula: new_offset = mouse_pos - (mouse_pos - old_offset) * (new_zoom / old_zoom)
        self.__pan_offset = mouse_pos - (mouse_pos - self.__pan_offset) * (self.__zoom_factor / old_zoom_factor)

        self.repaint()
        e.accept() # Indicate the event was handled

    def resizeEvent(self, event: QResizeEvent):
        """Handle widget resize events."""
        # Recalculate polygon positions when the widget is resized
        if self.__gdf is not None:
            self.resizeData()
        super().resizeEvent(event) # Call base class implementation
        
        
    def paintEvent(self, e: QPaintEvent):
        #draw situation
        
        #create new graphic object
        qp = QPainter(self)
        
        # Save painter state
        qp.save()
        
        # apply pan and zoom transformations
        qp.translate(self.__pan_offset)
        qp.scale(self.__zoom_factor, self.__zoom_factor)
        # draw loaded polygons if any and if display flag is set
        if self.__display_loaded and self.__polygons:
            # Set graphical attributes for loaded buildings
            # define brown color
            
            qp.setPen(Qt.GlobalColor.black)
            qp.setBrush(Qt.GlobalColor.yellow)
            
            # Draw all polygons
            for polygon, _ in self.__polygons:
                qp.drawPolygon(polygon)
                
        # draw simplified polygons if any
        if self.__simplified_polygons:
            # set graphical attributes for simplified buildings
            pen = QPen(Qt.GlobalColor.red) # Create a pen object
            pen.setWidth(2)  # Increase the pen width to make it thicker
            qp.setPen(pen) # Apply the thicker pen
            # qp.setBrush(Qt.GlobalColor.cyan) # Original solid fill
            qp.setBrush(Qt.BrushStyle.NoBrush) # Make fill transparent
            
            # draw all simplified polygons
            for polygon in self.__simplified_polygons:
                qp.drawPolygon(polygon)
        
        #set graphical attributes: building
        qp.setPen(Qt.GlobalColor.black)
        qp.setBrush(Qt.GlobalColor.yellow)
        
        #draw building
        qp.drawPolygon(self.building)
        
        #set graphical attributes: building_simplify
        qp.setPen(Qt.GlobalColor.gray)
        qp.setBrush(Qt.GlobalColor.blue)
        
        #draw building simplify
        qp.drawPolygon(self.building_simp)
        
        # restore painter state
        qp.restore()
        
        #end drawing
        qp.end()
        
    
    def getBuilding(self):
        # return analyzed building
        return self.building
    
    
    def setSimplifBuilding(self, building_simp_):
        self.building_simp = building_simp_
    
    
    def clearData(self):
        #clear polygon
        self.building.clear()
        
        #Repaint screen
        self.repaint()
        
    def getPolygons(self):
        """Return the list of loaded polygons"""
        return [poly for poly, _ in self.__polygons]
    
    def setSimplifiedPolygons(self, simplified_polygons):
        """Set the list of simplified polygons"""
        self.__simplified_polygons = simplified_polygons
        self.repaint()
        
    def toggleLoadedDisplay(self):
        """Toggle the display of loaded polygons"""
        self.__display_loaded = not self.__display_loaded
        self.repaint()
    
    def clearAllData(self):
        """Clear all data: building, simplified building, loaded polygons, and simplified polygons"""
        self.building.clear()
        self.building_simp.clear()
        self.__polygons.clear()
        self.__simplified_polygons.clear()
        self.__gdf = None
        # Reset zoom and pan
        self.__zoom_factor = 1.0
        self.__pan_offset = QPointF(0.0, 0.0)
        self.__last_pan_pos = None
        self.repaint()
        
    def clearResults(self):
        """Clear simplified results only"""
        self.building_simp.clear()
        self.__simplified_polygons.clear()
        self.repaint()

    def resetView(self):
        """Reset the view (zoom/pan)"""
        self.__zoom_factor = 1.0
        self.__pan_offset = QPointF(0.0, 0.0)
        self.__last_pan_pos = None
        self.repaint()
