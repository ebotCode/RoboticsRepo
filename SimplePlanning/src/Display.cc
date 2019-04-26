#include "Display.h"


namespace display {

    void print(geometry::BoundingBox box){    
        std::cout << " Rect: { "<< "left( "<< box.topLeft.x.value << " , "<< box.topLeft.y.value  <<"), " <<
                                "right( "<<box.bottomRight.x.value  << " , "<<box.bottomRight.y.value <<") }"<<std::endl;
    }


    void print(geometry::RectangleGeometry rect){
        print(rect.getBoundingBox());
    }


    void print(map::Obstacle obstacle){
        print(obstacle.getBoundingBox()); 
    }

    void print(map::Map& map){
        // print the obstacles contained in the map 
        geometry::Dimension dimension = map.getDimension(); 
        std::cout << "(map-description-start)################"<<std::endl;
        std::cout << "Map:{"<< std::endl; 
        std::cout << "--> dimension (width x height): " << dimension.width.value  << " x " << dimension.height.value  << std::endl; 
        std::cout << "--> Obstacles: "<< std::endl; 
        std::vector<map::Obstacle>obstacles = map.getObstacles(); 
        for ( map::Obstacle& obstacle : obstacles ){
            std::cout << "---> ";
            print(obstacle.getBoundingBox()); 
        }
        std::cout << "}"<< std::endl; 
        std::cout <<"(map-description-start)##################"<<std::endl; 
    }

    void print(map::OccupancyGrid& grid){        
        map::GridSize grid_size = grid.getSize(); 
        std::cout << "Grid size = "<< grid_size.rows << " x " <<grid_size.columns << std::endl; 
        for (int i = 0; i < grid_size.columns + 2; i++){
            std::cout << "#";    
        }
        std::cout<< std::endl; 

        for(int row = 0; row < grid_size.rows ; row++){
            std::cout <<"#"; 
            for ( int column = 0; column < grid_size.columns ; column++ ){
                map::Occupancy occupancy = grid.getOccupancy(map::GridPoint{row,column});
                std::string value = (occupancy == map::Occupancy::OCCUPIED)? "#" : " ";
                std::cout << value; // << " "; 
            }
            std::cout <<"#" << std::endl; 
        }

        for (int i = 0; i < grid_size.columns + 2; i++){
            std::cout << "#";    
        }        
        std::cout << std::endl; 
    }

    void print(map::OccupancyMap& occupancy_map){
        map::OccupancyGrid grid = occupancy_map.getGrid(); 
        print(grid); 
    }

    void print(units::Radian angle){
        units::Degree degree = metricmath::radianToDegree(angle); 
        std::cout <<"theta: "<<angle.value<<"[rad] ("<<degree.value<<" [deg])"<<std::endl;
    }

    void print(vehicle::Pose pose){
        std::cout <<"Pose: "; 
        std::cout << "x: "<<pose.x.value<<", y: "<<pose.y.value<<", ";
        print(pose.orientation); 
        
    }

    void print(vehicle::DriveState state){
        std::cout <<"DriveState {"<<std::endl; 
        std::cout <<"---> ";
        print(state.pose); 
        std::cout <<"---> steeringAngle: ";
        print(state.steeringAngle);
        std::cout <<"}"<<std::endl;             
    }

    void print(planner::NodeState state){
        std::cout <<"NodeState {"; 
        std::cout <<"---> { "<< " xId: "<< state.xId 
                            << " yId: "<< state.yId 
                            << " thetaId: " << state.orientationId 
                            << " steeringId: "<< state.steeringId << std::endl; 
        // print(state.info.driveState); 
        std::cout << "}"<<std::endl;
    }


}
