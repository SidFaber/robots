# Prereqs:
#  pip install numpy
#  pip install svgwrite
#  pip install svgpathtools

#  read an svg file into paths


from svgpathtools import svg2paths, wsvg

class SvgDrawing:
    """A drawing parsed from an svg file"""

    def __init__(self):
        self.paths = []
        self.minX = None
        self.minY = None
        self.maxX = None
        self.maxY = None
        self.file = None

    def loadFile (self, file):
        """Load an svg file and transform into point paths

        The path is not meant to be modified once it is loaded.
        
        Args:
            file (string): The location of the file
        """
        self.file = file

        svgpaths, svgattributes = svg2paths(file)

        for p in svgpaths:
            self.addPath (p)


    def addPath (self, svgPath):
        """Add an SVG path

        Args:
            svgPath (Path): A path object from the svgpathtools library
        """
        #  get the starting point for this path
        x = svgPath.start.real
        y = svgPath.start.imag
        if self.minX is None:
            self.minX = x
        if self.maxX is None:
            self.maxX = x
        if self.minY is None:
            self.minY = y
        if self.maxY is None:
            self.maxY = y

        mypath = SvgPath(x, y)

        for segment in svgPath:
            mypath.addPoint (segment.end.real, segment.end.imag)

        self.paths.append(mypath)

        self.minX = min (self.minX, mypath.minX)
        self.minY = min (self.minY, mypath.minY)
        self.maxX = max (self.maxX, mypath.maxY)
        self.maxY = max (self.maxY, mypath.maxY)

    def resize (self, canvasMinX, canvasMinY, canvasMaxX, canvasMaxY):
        """Resize a drawing to fit a canvas

        Args:
            canvasMinX (int):  Minimum X coordinate of the canvas
            canvasMinY (int):  Minimum Y coordinate of the canvas
            canvasMaxX (int):  Maximum X coordinate of the canvas
            canvasMaxY (int):  Maximum Y coordinate of the canvas
        """
        if (
            (self.minY == self.maxY) or
            (self.minX == self.maxX) or
            (canvasMinX == canvasMaxX) or
            (canvasMinY == canvasMaxY)):
            return

        scale = min (
            (canvasMaxX - canvasMinY) / (self.maxX - self.minX),
            (canvasMaxY - canvasMinY) / (self.maxY - self.minY))

        self.minX *= scale
        self.minY *= scale
        self.maxX *= scale
        self.maxY *= scale

        shiftX = canvasMinX - self.minX
        shiftY = canvasMinY - self.minY

        self.minX += shiftX
        self.minY += shiftY
        self.maxX += shiftX
        self.maxY += shiftY

        for path in self.paths:
            path.minX = (path.minX * scale) + shiftX
            path.minY = (path.minY * scale) + shiftY
            path.maxX = (path.maxX * scale) + shiftX
            path.maxY = (path.maxY * scale) + shiftY
            for point in path.points:
                point[0] = (point[0] * scale) + shiftX
                point[1] = (point[1] * scale) + shiftY
                





class SvgPath:
    """A collection of x,y points connected together to make a path"""
    
    def __init__(self, x, y):
        """
        Args:
            x (int): The x coordinate of the path start
            y (int): The y coordinate of the path start
        """
        x = float(x)
        y = float(y)

        self.minX = x
        self.maxX = x
        self.minY = y
        self.maxY = y
        self.startX = x
        self.startY = y
        self.endX = x
        self.endY = y
        self.points = []

        self.addPoint (x, y)

    def addPoint (self, x, y):
        """
        Args:
            x (int): The x coordinate of a point
            y (int): The y coordinate of a point
        """

        x = float(x)
        y = float(y)

        self.points.append([x, y])
        self.endX = x
        self.endY = y
        self.minX = min (self.minX, x)
        self.minY = min (self.minY, x)
        self.maxX = max (self.maxX, x)
        self.maxY = max (self.maxY, x)



def main():
    dwg = SvgDrawing()
    dwg.loadFile ("/home/sid/robots_ws/src/robots/gamestop/res/svg/duck.svg")
    dwg.resize (0.1500, -0.0700, 0.2500, 0.0700)
    for path in dwg.paths:
        for (x,y) in path.points:
            print (
        """
ros2 service call \
    '/open_manipulator_x/goal_task_space_path' \
    'open_manipulator_msgs/srv/SetKinematicsPose' \
    '{end_effector_name: "gripper", path_time: 0.3,
    kinematics_pose: {pose: {position: {x: %f, y: %f, z: 0.1}}}}'
sleep 0.3
        """ % (x,y))


if __name__ == '__main__':
    main()
    
