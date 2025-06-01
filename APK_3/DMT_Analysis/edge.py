from qpoint3df import *

class Edge:
    def __init__(self, p1:QPoint3DF, p2:QPoint3DF):
        self.start = p1
        self.end = p2 
        
    def getStart(self):
        return self.start
    
    def getEnd(self):
        return self.end
    
    def switchOrientation(self):
        return Edge(self.end, self.start)
    
    def __eq__(self, other):
        return self.start == other.start and self.end == other.end