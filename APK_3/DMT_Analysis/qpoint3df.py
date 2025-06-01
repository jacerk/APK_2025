from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

class QPoint3DF(QPointF):
    def __init__(self, x:float, y:float, z:float):
        super().__init__(x, y)
        self.z = z
    
    def getZ(self):
        return self.z    