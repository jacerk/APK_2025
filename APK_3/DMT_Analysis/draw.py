from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *
from qpoint3df import *
from edge import *
from triangle import *
from math import *

class Draw(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.view_points = True
        self.points = []
        self.dt = []
        self.contour_lines = []
        self.triangles = []
        self.view_dt = True
        self.view_contour_lines = True
        self.view_slope = True
        self.view_aspect = True
        self.offset = QPointF(0, 0)
        self.scale = 1.0

    def calculateTransform(self):
        if not self.points:
            return
            
        min_x = min(p.x() for p in self.points)
        max_x = max(p.x() for p in self.points)
        min_y = min(p.y() for p in self.points)
        max_y = max(p.y() for p in self.points)
        
        w = self.width()
        h = self.height()
        
        data_width = max_x - min_x
        data_height = max_y - min_y
        self.scale = min(w/data_width, h/data_height) * 0.9 if data_width * data_height != 0 else 1.0
        
        self.offset = QPointF(
            (w - data_width*self.scale)/2 - min_x*self.scale,
            (h - data_height*self.scale)/2 - min_y*self.scale
        )

    def transformPoint(self, p):
        return QPointF(
            self.width() - (p.x() * self.scale + self.offset.x()),
            (p.y() * self.scale + self.offset.y())
        )

    def paintEvent(self, e: QPaintEvent):
        self.calculateTransform()
        qp = QPainter(self)
        qp.begin(self)
        
        # Draw slope-colored triangles
        if self.view_slope and self.triangles:
            for triangle in self.triangles:
                slope = triangle.getSlope()
                slope = min(slope, 90)  # Cap slope at 45° for color scaling
                intensity = int(255 * (slope / 90))  # Scale slope 0-45° to 0-255
                qp.setPen(QPen(Qt.GlobalColor.black))
                qp.setBrush(QColor(255 - intensity, intensity - 90, 0))  # Proper red-green gradient
                poly = QPolygonF([self.transformPoint(v) for v in triangle.getVertices()])
                qp.drawPolygon(poly)

        # Draw aspect directions
        if self.view_aspect and hasattr(self, 'north'):
            # North - blue
            qp.setPen(QPen(Qt.GlobalColor.black))
            qp.setBrush(QColor(0, 0, 255))
            for poly in self.north:
                points = [self.transformPoint(e.getStart()) for e in poly]
                qp.drawPolygon(QPolygonF(points))
            
            # South - green
            qp.setPen(QPen(Qt.GlobalColor.black))
            qp.setBrush(QColor(0, 255, 0))
            for poly in self.south:
                points = [self.transformPoint(e.getStart()) for e in poly]
                qp.drawPolygon(QPolygonF(points))
            
            # East - red
            qp.setPen(QPen(Qt.GlobalColor.black))
            qp.setBrush(QColor(255, 0, 0))
            for poly in self.east:
                points = [self.transformPoint(e.getStart()) for e in poly]
                qp.drawPolygon(QPolygonF(points))
            
            # West - yellow
            qp.setPen(QPen(Qt.GlobalColor.black))
            qp.setBrush(QColor(255, 255, 0))
            for poly in self.west:
                points = [self.transformPoint(e.getStart()) for e in poly]
                qp.drawPolygon(QPolygonF(points))

        # Draw Delaunay triangulation
        if self.view_dt and self.dt:
            qp.setPen(QPen(Qt.GlobalColor.blue, 1))
            for edge in self.dt:
                p1 = self.transformPoint(edge.getStart())
                p2 = self.transformPoint(edge.getEnd())
                qp.drawLine(p1, p2)

        # Draw contour lines
        if self.view_contour_lines and self.contour_lines:
            qp.setPen(QPen(Qt.GlobalColor.darkGray, 2))
            for contour in self.contour_lines:
                path = QPainterPath()
                start = contour.getStart()
                end = contour.getEnd()
                path.moveTo(self.transformPoint(start))
                path.lineTo(self.transformPoint(end))
                qp.drawPath(path)

        # Draw points
        if self.view_points:
            qp.setPen(Qt.GlobalColor.black)
            qp.setBrush(Qt.GlobalColor.yellow)
            r = 2
            for p in self.points:
                tp = self.transformPoint(p)
                qp.drawEllipse(int((tp.x()-r)), int((tp.y()-r)), 2*r, 2*r)

        qp.end()

    # Rest of Draw class methods remain the same...
    def setPoints(self, points_):
        self.points = points_
        self.calculateTransform()
        self.repaint()

    def setViewPoints(self, view_points_):
        self.view_points = view_points_
        
    def setViewDT(self, view_dt_):
        self.view_dt = view_dt_
        
    def setViewContourLines(self, view_contour_lines_):
        self.view_contour_lines = view_contour_lines_
        
    def setViewSlope(self, view_slope_):
        self.view_slope = view_slope_
        
    def setViewAspect(self, view_aspect_):
        self.view_aspect = view_aspect_
        self.repaint()
        
    def getPoints(self):
        return self.points
        
    def getDT(self):
        return self.dt
        
    def getTriangles(self):
        return self.triangles
        
    def setDT(self, dt_):
        self.dt = dt_
        
    def setTriangles(self, triangles_):
        self.triangles = triangles_
        
    def setContourLines(self, contour_lines_):
        self.contour_lines = contour_lines_

    def setAspectData(self, north, south, west, east):
        self.north = north
        self.south = south
        self.west = west
        self.east = east
        self.repaint()
