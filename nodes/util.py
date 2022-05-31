#!/usr/bin/env python

import numpy as np


# A class that describes a line
class Line:
    def __init__(self):
        self.vertical = False  # True if it's a vertical line
        self.pnt1 = None
        self.pnt2 = None
        self.a = None
        self.b = -1
        self.c = None

    # Construct line from two points and convert to: ax + by + c = 0
    def from2pnts(self, pnt1, pnt2):
        self.pnt1 = pnt1
        self.pnt2 = pnt2
        self.x = 0
        # Line is vertical
        if (self.pnt2[0] - self.pnt1[0]) == 0: 
            self.a = float('inf')
            self.vertical = True
            self.x = pnt1[0] # Eq: x = {}

        # Line is not vertical
        else: self.a = (self.pnt2[1]-self.pnt1[1])/(self.pnt2[0]-self.pnt1[0])
            
        self.c = pnt1[1]-self.a*pnt1[0]

    # Construct line from slope and one point and convert to: ax + by + c = 0
    def from1pnt(self, m, pnt):
        self.pnt1 = pnt
        self.a = m
        if m == float('inf'):
            self.vertical = True 
            self.x = pnt[0]
        else:
            self.c = pnt[1]-self.a*pnt[0]

    def slope(self):
        if (self.pnt2[0] - self.pnt1[0]) == 0: return float('inf')
        return (self.pnt2[1]-self.pnt1[1])/(self.pnt2[0]-self.pnt1[0])

    # Shortest distance between th given point to the line
    def distance(self, pnt):
        if self.vertical: return abs(pnt[0] - self.x)
        x,y = pnt
        return abs(self.a*x+self.b*y+self.c)/(self.a**2+self.b**2)**(0.5)
        

    # True if the point is above the line
    def larger(self, pnt):
        x,y = pnt
        if self.vertical: return x > self.x
        return (y-self.pnt1[1]) > self.a*(x-self.pnt1[0])

    # True if the point is below the line
    def smaller(self, pnt):
        x,y = pnt
        if self.vertical: return x < self.x
        return (y-self.pnt1[1]) < self.a*(x-self.pnt1[0])
        
# A class that describes a circle
class Circle:
    def __init__(self, cen, r):
        self.cen = cen
        self.r = r
    # True if the pnt is in the circle (with clearance)
    def inside(self, pnt, clearance=0):
        return self.distance(pnt) < clearance

    # Distance between given circle (with radius) to the origin of the circle
    def distance(self, pnt):
        x,y = pnt
        return ((x-self.cen[0])**2 + (y-self.cen[1])**2)**0.5 - self.r
        

# A class that describes a polygon
class Polygon_Obstacle:
    def __init__(self):
        # List contains Line objects that enclose the polygon
        self.lines = []
        # List contains information to enclose the clearance area
        self.clearance = []
    def construct(self, l):
        """
         Construct the polygon given list:
         The list looks like: [(pnt1,pnt2,larger),...]
         pnt1,pnt2 are the points that define a line 
         larger={True/False} determine where the point should be (above/below) 
         to be considered inside the polygon
        """
        for pnt1,pnt2,larger in l:
            # Define line
            line = Line()
            line.from2pnts(pnt1,pnt2)
            self.lines.append([line, larger])

            # Define clearance area
            c_lines = []
            c_lines.append([line])

            # line1 and line2 are orthogonal to the line defined above
            line1 = Line()
            line2 = Line()
            if line.a == 0:
                line1.from1pnt(float('inf'), pnt1)
                line2.from1pnt(float('inf'), pnt2)
            else:
                line1.from1pnt(-1/line.a, pnt1)
                line2.from1pnt(-1/line.a, pnt2)
            if pnt1[1] > pnt2[1]:
                c_lines.append([line1, False])
                c_lines.append([line2, True])
            elif pnt1[1] < pnt2[1]:
                c_lines.append([line1, True])
                c_lines.append([line2, False])
            else:
                if pnt1[0] < pnt2[0]:
                    c_lines.append([line1, True])
                    c_lines.append([line2, False])
                else:
                    c_lines.append([line1, False])
                    c_lines.append([line2, True])
            
            self.clearance.append(c_lines)

    # Determine whether a point is inside the polygon
    def inside(self, pnt, clearance=0):

        isInside = True
        for line,larger in self.lines:
            # The point should be above the line and is above the line
            if larger: isInside = isInside and line.larger(pnt) 
            # The point should be below the line and is below the line
            else: isInside = isInside and line.smaller(pnt)

        # If the point is inside the polygon, return 
        if isInside: return True

        # The point is not inside the polygon, check if the point is 
        # in the clearance area
        for c_lines in self.clearance:
            line = c_lines[0][0]
            # Check if the point is close enough to the line
            inThis = (line.distance(pnt) < clearance) 
            # If not, ignore
            if not inThis: continue

            # Circle clearance areas with the vertexs as origin and clearance as radius
            circle1 = Circle(line.pnt1, clearance)
            circle2 = Circle(line.pnt2, clearance)

            # Check if the point falls into the clearance area
            for line in c_lines[1:]:
                if line[1]: inThis = inThis and line[0].larger(pnt)
                else: inThis = inThis and line[0].smaller(pnt)
            if inThis or circle1.inside(pnt) or circle2.inside(pnt): return True

        return False
