#include "Map.h" 


namespace map{

/**
 * Obstacle
 * 
 * */

Obstacle::Obstacle(geometry::RectangleGeometry& geometry){
    _geometry = geometry; 
}
Obstacle::Obstacle(std::string& name_in, geometry::RectangleGeometry& geometry){
    _name = name_in; 
    _geometry = geometry; 
}

geometry::BoundingBox Obstacle::getBoundingBox() const { 
    return _geometry.getBoundingBox();
}

/**
 * StaticObstaclesMap : Map 
 * 
 * 
 * */
StaticObstaclesMap::StaticObstaclesMap(){}

StaticObstaclesMap::StaticObstaclesMap(geometry::Dimension dimension):
_dimension(dimension){}

void StaticObstaclesMap::addObstacle(Obstacle obstacle){
    _obstacles.push_back(obstacle); 
}

std::vector<Obstacle> StaticObstaclesMap::getObstacles() const {
    return _obstacles; 
}

geometry::Dimension StaticObstaclesMap::getDimension() const {
    return _dimension; 
}


/**
 * Occupancy Grid
 * */

OccupancyGrid::OccupancyGrid(){}

OccupancyGrid::OccupancyGrid(units::Meter resolution, geometry::Dimension dimension):
_resolution(resolution), _dimension(dimension){
    // compute the grid_size based on resolution 
    _grid_size = computeGridSizeFromResolutionAndDimension(resolution,dimension); 

    _grid.resize(_grid_size.rows); 
    for (size_t i = 0; i < _grid.size(); i++){
        _grid.at(i).resize(_grid_size.columns,Occupancy::FREE); 
    }

}

GridSize OccupancyGrid::computeGridSizeFromResolutionAndDimension(units::Meter resolution, geometry::Dimension dimension) const{
    int nrow = (dimension.height.value / resolution.value) + 1; 
    int ncol = (dimension.width.value  / resolution.value) + 1; 
    map::GridSize size; 
    size.columns = ncol; 
    size.rows = nrow; 
    return size; 
}

GridSize OccupancyGrid::getSize() const{
    return _grid_size; 
}

void OccupancyGrid::setOccupancy(Occupancy value, GridPoint gridpoint){
    if (isWithinGrid(gridpoint) ){
        _grid.at(gridpoint.row).at(gridpoint.column) = value; 
    }
    
}

Occupancy OccupancyGrid::getOccupancy(GridPoint gridpoint) const {
    if ( isWithinGrid(gridpoint) ){
        return _grid.at(gridpoint.row).at(gridpoint.column); 
    }
    return Occupancy::OCCUPIED;
    
}

void OccupancyGrid::markAsOccupied(GridPoint gridpoint){
    if ( isWithinGrid(gridpoint) ){
        setOccupancy(Occupancy::OCCUPIED,gridpoint); 
    }
    
}

void OccupancyGrid::markAsFree(GridPoint gridpoint){    
    if ( isWithinGrid(gridpoint) ){
        setOccupancy(Occupancy::FREE,gridpoint); 
    }
    
}

bool OccupancyGrid::isFree(GridPoint gridpoint) const {  
    if ( isWithinGrid(gridpoint) ){
        return getOccupancy(gridpoint) == Occupancy::FREE;
    }  
    return false; 
    
}

GridPoint OccupancyGrid::convertToGridPoint(geometry::Point point) const {
    double dx = _dimension.width.value / _grid_size.columns; 
    double dy = _dimension.height.value / _grid_size.rows; 

    int column_id = ( point.x.value / dx) ;  
    int row_id    = ( _dimension.height.value - point.y.value ) / dy; 
    // 
    GridPoint gridpoint; 
    gridpoint.row = row_id; 
    gridpoint.column = column_id; 
    return gridpoint;  

}

bool OccupancyGrid::isWithinGrid(GridPoint gridpoint) const{
    return (gridpoint.row >= 0 && gridpoint.row < _grid_size.rows && 
            gridpoint.column >= 0 && gridpoint.column < _grid_size.columns); 
}

units::Meter OccupancyGrid::getResolution() const{
    return _resolution; 
}
/**
 * Occupancy Map 
*/
void OccupancyMap::setMap(const Map& map_in){
    _map = &map_in; 
}

// void printBox(geometry::BoundingBox box){
//     std::cout << "box = {" << std::endl; 
//     std::cout << "topleft = " << box.topLeft.x << " " << box.topLeft.y << std::endl; 
//     std::cout << "bottomright = " << box.bottomRight.x << " " << box.bottomRight.y << std::endl; 
//     std::cout << "}"<<std::endl; 
// }

// void printGridPoint(GridPoint point){
//     std::cout << "GridPoint: " << point.row << " "<< point.column << std::endl; 
// }

void OccupancyMap::computeOccupancy(){
    std::vector<Obstacle> obstacles = _map->getObstacles();     

    for (Obstacle& obstacle : obstacles ){                        
        geometry::BoundingBox box = obstacle.getBoundingBox();         
        GridPoint topleft = _grid.convertToGridPoint(box.topLeft);
        GridPoint bottomright  = _grid.convertToGridPoint(box.bottomRight); 
        markRegionAsOccupied(topleft,bottomright); 
    }
}

void OccupancyMap::markRegionAsOccupied(GridPoint topleft, GridPoint bottom_right){
    GridPoint loc; 
    for ( int row = topleft.row; row <= bottom_right.row; row++){
        for ( int column = topleft.column; column <= bottom_right.column; column++ ){            
            loc.row = row;
            loc.column = column; 
            _grid.markAsOccupied(loc); 
        }
    }
}

OccupancyMap::OccupancyMap(OccupancyGrid grid){
    _grid = grid; 
}

OccupancyGrid OccupancyMap::getGrid() const{
    return _grid; 
}

GridPoint OccupancyMap::convertToGridPoint(geometry::Point point) const {
    return _grid.convertToGridPoint(point); 
}

bool OccupancyMap::isFree(GridPoint point) const{
    return _grid.isFree(point); 
}

//isWithin 
bool OccupancyMap::isWithin(GridPoint point) const{
    return _grid.isWithinGrid(point); 
}

// getResolution 
units::Meter OccupancyMap::getResolution() const{
    return _grid.getResolution(); 

}



} // namespace map