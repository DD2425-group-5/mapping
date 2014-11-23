#include "mapRepresentation.hpp"

// Shorten the grid utils namespace
namespace ocutil = occupancy_grid_utils;

MapRepresentation::MapRepresentation(int argc, char *argv[]){
    ros::init(argc, argv, "map_representation");
    ros::NodeHandle handle;
    std::string lineTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/stitched_line_topic",
                      lineTopic);
    
    ROSUtil::getParam(handle, "/map_representation/grid_resolution", grid.info.resolution);
    line_sub = handle.subscribe(lineTopic, 1, &MapRepresentation::lineCallback, this);

    runNode();
}

void MapRepresentation::runNode(){
    
}

void MapRepresentation::lineCallback(const mapping_msgs::LineVector& msg){
    lines = msg;
}

/**
 * Find the size of the bounding box of the set of lines given, in metres.
 */
std::pair<float, float> MapRepresentation::findLineBoundSize(mapping_msgs::LineVector lineVec) {
    std::vector<mapping_msgs::Line> lines=lineVec.lines;

    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    for(size_t lineIterator = 0; lineIterator<lines.size(); lineIterator++){
        MinMaxXY minmax = lineMinMax(lines[lineIterator]);
        if (minmax.minX < minX){
            minX = minmax.minX;
        }
        if (minmax.maxX > maxX){
            maxX = minmax.maxX;
        }
        if (minmax.minY < minY){
            minY = minmax.minY;
        }
        if (minmax.maxY > maxY){
            maxY = minmax.maxY;
        }
    }

    float L1normX = maxX - minX;
    float L1normY = maxY - minY;

    return std::pair<float, float> (L1normX, L1normY);
}

/**
 * Find the minimum and maximum values of x and y on the specified line.
 */
MinMaxXY MapRepresentation::lineMinMax(mapping_msgs::Line l){
    float minX = l.start.x < l.end.x ? l.start.x : l.end.x;
    float maxX = l.start.x > l.end.x ? l.start.x : l.end.x;
    float minY = l.start.y < l.end.y ? l.start.y : l.end.y;
    float maxY = l.start.y > l.end.y ? l.start.y : l.end.y;

    return MinMaxXY(minX, maxX, minY, maxY);
}

/**
 * Gets the bounding box of the two lines provided. Takes into account the fact
 * that the lines may be of different length and orientation.
 */
geometry_msgs::Polygon MapRepresentation::boundsFromLines(mapping_msgs::Line l1, mapping_msgs::Line l2){
    geometry_msgs::Polygon poly;
    
    // First find the bounding box of the lines - this is not just the end
    // points of the lines. Need to actually extract the min and max values,
    // because lines can be of different lengths.
    MinMaxXY l1mm = lineMinMax(l1);
    MinMaxXY l2mm = lineMinMax(l2);
    
    // min and max values when looking at both lines
    float minX = l1mm.minX < l2mm.minX ? l1mm.minX : l2mm.minX;
    float maxX = l1mm.maxX > l2mm.maxX ? l1mm.maxX : l2mm.maxX;
    float minY = l1mm.minY < l2mm.minY ? l1mm.minY : l2mm.minY;
    float maxY = l1mm.maxY > l2mm.maxY ? l1mm.maxY : l2mm.maxY;

    geometry_msgs::Point32 botLeft;
    botLeft.x = minX;
    botLeft.y = minY;
    botLeft.z = 0;
    geometry_msgs::Point32 topLeft;
    botLeft.x = minX;
    botLeft.y = maxY;
    botLeft.z = 0;
    geometry_msgs::Point32 topRight;
    botLeft.x = maxX;
    botLeft.y = maxY;
    botLeft.z = 0;
    geometry_msgs::Point32 botRight;
    botLeft.x = maxX;
    botLeft.y = minY;
    botLeft.z = 0;
    
    // push points onto the polygon in an order where they make a rectangle.
    poly.points.push_back(botLeft);
    poly.points.push_back(topLeft);
    poly.points.push_back(topRight);
    poly.points.push_back(botRight);

    return poly;
}

/**
 * Set cells in the given polygon on the grid to the specified value. It is
 * assumed that the polygon lies inside the bounds of the grid
 */
void MapRepresentation::setCellsInBounds(geometry_msgs::Polygon bounds, signed char value){
    // get cells within the polygon
    std::set<ocutil::Cell> cells = occupancy_grid_utils::cellsInConvexPolygon(grid.info, bounds);
    for (std::set<ocutil::Cell>::iterator cell = cells.begin(); cell != cells.end(); cell++) {
        // Get the index corresponding to the cell
        ocutil::index_t ind = ocutil::cellIndex(grid.info, *cell);
        // set the cell to the specified value in the data array
        grid.data[ind] = value;
    }
}

/**
 * Projects a line onto the grid, setting cells which fall on the line to the
 * specified value.
 */
void MapRepresentation::projectLineOntoGrid(mapping_msgs::Line lineToProject, signed char value){
    ocutil::RayTraceIterRange iterators = ocutil::rayTrace( grid.info,
                                                            lineToProject.start,
                                                            lineToProject.end,
                                                            false,
                                                            false);
    ocutil::RayTraceIterator iterator = iterators.first;
    ocutil::RayTraceIterator iteratorEnd = iterators.second;
    
    // Run the first iterator until it hits the position of the second iterator.
    for(; !(iterator == iteratorEnd); iterator++ ){
        // get the index corresponding to the cell pointed to by the iterator
        ocutil::index_t ind = ocutil::cellIndex(grid.info, *iterator);
        // set the index to the specified value
        grid.data[ind] = value;
    }
}

int main(int argc, char *argv[])
{
    MapRepresentation(argc, argv);
}
