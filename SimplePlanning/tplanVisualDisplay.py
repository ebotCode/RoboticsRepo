import numpy as np 
import matplotlib.pyplot as plt 

class Point:
    def __init__(self,x,y):
        self.x = x 
        self.y = y 
    def __str__(self):
        return "({},{})".format(self.x,self.y)

class Box: 
    def __init__(self,pointlist):
        self.pointlist = pointlist 

class DriveState:
    def __init__(self,x,y,orientation,steering):
        self.x = x 
        self.y = y
        self.orientation = orientation 
        self.steering = steering 

class Vehicle:
    def __init__(self, width, length, max_steering): 
        self.width = width 
        self.length = length 
        self.maxSteering = max_steering 

class Obstacle: 
    def __init__(self,topleft_point,bottomright_point):
        self.topLeftPoint = topleft_point 
        self.bottomRightPoint = bottomright_point 

class Map: 
    def __init__(self,width,height,obstacles ):
        self.width = width 
        self.height = height 
        self.obstacles = obstacles 

class Planner: 
    def __init__(self,grid_resolution, orientation_resolution,plan):
        self.gridResolution = grid_resolution 
        self.orientationResolution = orientation_resolution
        self.plan = plan  
        

def parseVehicleFromFileObj(fileobj):
    line = fileobj.readline().strip()
    assert(line.find("vehicle") == True)
    arr = [ float(item) for item in fileobj.readline().strip().split(" ")]
    vehicle = Vehicle(arr[0],arr[1],arr[2])
    return vehicle 

def parseMapFromFileObj(fileobj): 
    line = fileobj.readline() 
    assert(line.find("map") == True)  
    size = [ float(item) for item in fileobj.readline().strip().split(" ")]
    nobstacles = int(fileobj.readline())
    obstacles = [] 
    for i in range(nobstacles): 
        bbox = [float(item) for item in fileobj.readline().strip().split(" ")]
        p1 = Point(bbox[0],bbox[1])
        p2 = Point(bbox[2],bbox[3]) 
        obstacles.append(Obstacle(p1,p2))
    mapobj = Map(size[0],size[1],obstacles) 
    return mapobj 

def parsePlannerFromFileObj(fileobj): 
    line = fileobj.readline() 
    assert(line.find("planner")== True) 
    # read in the resolutions for the planner 
    resolutions = [float(item) for item in fileobj.readline().strip().split(" ")]
    # read in the plan 
    line = fileobj.readline()
    assert(line.find("plan")==True) 
    nplanpoints = int(fileobj.readline()) 
    plan = [] 
    # read the plans 
    for i in range(nplanpoints): 
        line_list = [ float(item) for item in fileobj.readline().strip().split(" ")]
        x = line_list[0]
        y = line_list[1]            
        orientation = line_list[2] 
        steering = line_list[3] 
        state = DriveState(x,y,orientation,steering)
        plan.append(state) 
    # ::
    planner = Planner(resolutions[0],resolutions[1],plan) 
    return planner 

def parsePlanFile(filepath): 
    with open(filepath,'r') as fileobj:
        # read all lines 
        vehicle = parseVehicleFromFileObj(fileobj) 
        # pars map 
        mapobj  = parseMapFromFileObj(fileobj) 
        # 
        planner = parsePlannerFromFileObj(fileobj) 

    return {'vehicle':vehicle,'map':mapobj,'planner':planner} 


def createBox(point,angle,length,box_height=0.2): 
    box_height = box_height / 2.0
    center = np.array([[point.x],[point.y]])
    vectors = [
            np.array([[point.x ], [point.y + box_height]]),
            np.array([[point.x ], [point.y - box_height]]),
            np.array([[point.x + length] , [point.y - box_height]]),
            np.array([[point.x + length] , [point.y + box_height]]) ]

    RotationMatrix = np.array([[np.cos(angle),-np.sin(angle)],
                               [np.sin(angle),np.cos(angle)]])
    pointlist = []            
    for vec in vectors: 
        res = np.matmul(RotationMatrix,vec - center)
        actual = res + center; 
        p1 = Point(actual[0,0],actual[1,0])
        pointlist.append(p1)  

    box  = Box(pointlist) 
    return box 

def degToRad(angle):
    return np.pi * angle / 180.0 

def plotPointList(pointlist):
    x = [] 
    y = [] 
    for item in pointlist: 
        x.append(item.x)
        y.append(item.y) 
    x.append(pointlist[0].x) 
    y.append(pointlist[0].y) 
    plt.plot(x,y) 



def plotBoundingBox(topleft,bottomright): 
    x = [topleft.x,topleft.x,bottomright.x,bottomright.x,topleft.x] 
    y = [topleft.y,bottomright.y,bottomright.y,topleft.y,topleft.y] 
    plt.plot(x,y) 

def plotMap(mapobj):
    """ plots the map obstacles on the screen """ 
    for obstacle in mapobj.obstacles: 
        plotBoundingBox(obstacle.topLeftPoint,obstacle.bottomRightPoint) 

def plotPlan(planner,vehicle,pause=False):
    for drivestate in planner.plan: 
        point = Point(drivestate.x,drivestate.y)
        orientation = drivestate.orientation 
        box = createBox(point,orientation,vehicle.length,vehicle.width) 
        plotPointList(box.pointlist) 
        if pause:
            input('<enter to continue>') 

def showPlan(planfilepath,pause=False):
    info = parsePlanFile(planfilepath) 
    vehicle = info['vehicle']
    planner = info['planner'] 
    mapobj = info['map']

    plt.ion() 
    plt.ylim((-1,mapobj.height))
    plt.xlim((-1,mapobj.width))
    #:: Plot the map obstacles 
    plotMap(mapobj)
    #:: show the plan
    plotPlan(planner,vehicle,pause) 
    

###########################
GENERATED_PATHS_DIR = './src/generated_paths/'
# showPlan("./src/generated_paths/simpleplan.tplan")
showPlan(GENERATED_PATHS_DIR + 'hybridpressure.tplan',pause=True)
# showPlan(GENERATED_PATHS_DIR + 'hybrid.tplan',pause=False)

while 1: 
    a = input("Done") 
    if ( a == 'q'):
        break 