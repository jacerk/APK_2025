from qpoint3df import*
from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

class Triangle:
    def __init__ (self, p1_:QPoint3DF, p2_:QPoint3DF, p3_:QPoint3DF, slope_:float, aspect_:float):
        #Triangle of DTM
        self.vertices = QPolygonF()
        self.p1 = p1_
        self.p2 = p2_
        self.p3 = p3_
        self.slope = slope_
        self.aspect = aspect_
        
    def getVertices(self):
        #Get triangle vertices
        return self.p1, self.p2, self.p3
    
    def getSlope(self):
        #Get triangle slope
        return self.slope
    
    def getAspect(self):
        #Get triangle aspect
        return self.aspect
    
    def setSlope(self, slope_):
        #Set triangle slope
        self.slope = slope_
        
    def setAspect(self, aspect_):
        #Set triangle aspect
        self.aspect = aspect_