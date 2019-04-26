
CPPFLAGS= -O
CXX= g++ 

#------------------------------
# This is short, (kindly review). 
#-------------------------------
# SRC = Map.cc Geometry.cc VehicleModel.cc MetricMath.cc workspace.cc 
# OBJ = $(SRC:.cpp = .o)

# all: output 

# output : $(OBJ) 
# 		$(CXX) $(CPPFLAGS) -o main.out $(OBJ) 
#------------------------------
#
#-------------------------------

# s

all: ws_planner

Map.o: Map.cc 
	$(CXX) $(CPPFLAGS) -c Map.cc 

Geometry.o : Geometry.cc 
	$(CXX) $(CPPFLAGS) -c Geometry.cc 

VehicleModel.o : VehicleModel.cc 
	$(CXX) $(CPPFLAGS) -c VehicleModel.cc 

MetricMath.o : MetricMath.cc 
	$(CXX) $(CPPFLAGS) -c MetricMath.cc 

Display.o : Display.cc 
	$(CXX) $(CPPFLAGS) -c Display.cc 

Planner.o : Planner.cc 
	$(CXX) $(CPPFLAGS) -c Planner.cc 

PlannerVariant.o : PlannerVariant.cc 
	$(CXX) $(CPPFLAGS) -c PlannerVariant.cc 

workspace.o: workspace.cc 
	$(CXX) $(CPPFLAGS) -c workspace.cc

ws_planner.o: ws_planner.cc 
	$(CXX) $(CPPFLAGS) -c ws_planner.cc 

ws_map.o: ws_map.cc 
	$(CXX) $(CPPFLAGS) -c ws_map.cc 

ws_occupancy.o: ws_occupancy.cc 
	$(CXX) $(CPPFLAGS) -c ws_occupancy.cc 

ws_vehicle.o: ws_vehicle.cc 
	$(CXX) $(CPPFLAGS) -c ws_vehicle.cc

ws_paths.o: ws_paths.cc 
	$(CXX) $(CPPFLAGS) -c ws_paths.cc 

# list of object files needed for the workspace output 
OBJ = \
Map.o \
Geometry.o \
VehicleModel.o \
MetricMath.o \
Display.o \
Planner.o \
PlannerVariant.o 

ws_map: $(OBJ)  ws_map.o 
	$(CXX) $(CPPFLAGS) -o ws_map.out $(OBJ) ws_map.o 

ws_planner:  $(OBJ) ws_planner.o
	$(CXX) $(CPPFLAGS) -o ws_planner.out $(OBJ) ws_planner.o 

ws_occupancy: $(OBJ) ws_occupancy.o 
	$(CXX) $(CPPFLAGS) -o ws_occupancy.out $(OBJ) ws_occupancy.o 

ws_vehicle: $(OBJ) ws_vehicle.o 
	$(CXX) $(CPPFLAGS) -o ws_vehicle.out $(OBJ) ws_vehicle.o 

ws_paths: $(OBJ) ws_paths.o 
	$(CXX) $(CPPFLAGS) -o ws_paths.out $(OBJ) ws_paths.o 


clean: 
	rm -f core *.o 



