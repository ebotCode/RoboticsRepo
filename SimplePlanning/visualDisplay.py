import numpy as np 
import matplotlib.pyplot as plt 

CAR_LENGTH = 0.2 
CAR_WIDTH =  0.08

class DriveState:
    def __init__(self,x,y,orientation,steering):
        self.x = x 
        self.y = y
        self.orientation = orientation 
        self.steering = steering 

def parsePlanFile(filepath): 
    plan = [] 
    with open(filepath,'r') as fileobj:
        all_lines = fileobj.readlines() 
        for line in all_lines: 
            line_list = [ float(item) for item in line.strip().split(" ")]
            x = line_list[0]
            y = line_list[1]            
            orientation = line_list[2] 
            steering = line_list[3] 
            state = DriveState(x,y,orientation,steering)
            plan.append(state) 

    return plan 

class Point:
    def __init__(self,x,y):
        self.x = x 
        self.y = y 
    def __str__(self):
        return "({},{})".format(self.x,self.y)

class Box: 
    def __init__(self,pointlist):
        self.pointlist = pointlist 


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

def showPlan(planfilepath):
    plan = parsePlanFile(planfilepath) 
    boxes = [] 
    for drivestate in plan: 
        point = Point(drivestate.x,drivestate.y)
        orientation = drivestate.orientation 
        box = createBox(point,orientation,CAR_LENGTH,CAR_WIDTH) 
        boxes.append(box) 
        

    # plot the box 
    for box in boxes: 
        plotPointList(box.pointlist) 
        input('<enter to continue>') 
    

box = createBox(Point(1.5,0),degToRad(30),1) 
for item in box.pointlist:
    print(item) 

# plt.ylim((0,5))
# plt.xlim((0,10))
# plotPointList(box.pointlist)
# plt.show() 

###########################
GENERATED_PATHS_DIR = './src/generated_paths/'
box1 = createBox(Point(2.5,3.5 - 0.5*1.5),0,3,1.5)
box2 = createBox(Point(7.5,3.5 - 0.5*2.5),0,2,2.5)
plt.ion() 
plt.ylim((-1,6))
plt.xlim((-1,12))
plotPointList(box1.pointlist) 
plotPointList(box2.pointlist) 
# showPlan("./src/generated_paths/simpleplan.tplan")
showPlan(GENERATED_PATHS_DIR + 'hybridpressure.tplan')

while 1: 
    a = input("Done") 
    if ( a == 'q'):
        break 