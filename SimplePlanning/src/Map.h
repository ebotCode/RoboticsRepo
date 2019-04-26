#include <iostream> 
#include <vector> 
#include "Geometry.h" 
#include "Units.h"


#ifndef MAP_H
#define MAP_H 


namespace map{

class Obstacle{
    public: 
        explicit Obstacle (geometry::RectangleGeometry& geometry_in); 
        Obstacle(std::string& name_in, geometry::RectangleGeometry& geometry_in); 
        geometry::BoundingBox getBoundingBox() const ; 

    private: 
        std::string  _name = ""; 
        geometry::RectangleGeometry _geometry; 
};    


class Map {
    public: 
        virtual void addObstacle(Obstacle obstacle)  = 0;
        virtual std::vector<Obstacle> getObstacles() const  = 0;
        virtual geometry::Dimension getDimension() const = 0;          
};

class StaticObstaclesMap : public Map {
    public: 
        StaticObstaclesMap();
        explicit StaticObstaclesMap(geometry::Dimension dimension);  
        void addObstacle(Obstacle obstacle) ;
        std::vector<Obstacle> getObstacles() const  ; 
        geometry::Dimension getDimension() const;                
        
    private: 
        std::vector<Obstacle> _obstacles; 
        geometry::Dimension _dimension;     
};

struct GridPoint{
    int row = 0; 
    int column = 0; 
};

struct GridSize{
    int rows = 0; 
    int columns = 0; 
};

enum class Occupancy {FREE = 0, OCCUPIED = 1};

class OccupancyGrid { 
    public: 
        OccupancyGrid();
        OccupancyGrid(units::Meter resolution, geometry::Dimension dimension); 
        void markAsOccupied(GridPoint gridpoint); 
        void markAsFree(GridPoint gridpoint);         
        bool isFree(GridPoint gridpoint) const; 
        GridPoint convertToGridPoint(geometry::Point point) const ;
        Occupancy getOccupancy(GridPoint gridpoint) const;
        GridSize getSize() const; 
        bool isWithinGrid(GridPoint gridpoint) const; 
        units::Meter getResolution() const; 
    private: 
        void setOccupancy(Occupancy value, GridPoint gridpoint);
        GridSize computeGridSizeFromResolutionAndDimension(units::Meter resolution, geometry::Dimension dimension) const ; 

        std::vector<std::vector<Occupancy>> _grid; 
        GridSize _grid_size;
        units::Meter _resolution; 
        geometry::Dimension _dimension; 
};


class OccupancyMap {
    public: 
        explicit OccupancyMap(OccupancyGrid grid);
        void setMap(const Map& map_in);
        void computeOccupancy();
        void markRegionAsOccupied(GridPoint topleft, GridPoint bottom_right); 
        OccupancyGrid getGrid() const; 
        GridPoint convertToGridPoint(geometry::Point point) const; 
        bool isFree(GridPoint point) const; 
        bool isWithin(GridPoint point) const;
        units::Meter getResolution() const;  

    private: 
        const Map* _map; 
        OccupancyGrid _grid; 
};



} // namespace map


#endif