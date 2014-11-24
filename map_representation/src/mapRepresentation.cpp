#include "mapRepresentation.hpp"

// Shorten the grid utils namespace
namespace ocutil = occupancy_grid_utils;

MapRepresentation::MapRepresentation(int argc, char *argv[]){
    ros::init(argc, argv, "map_representation");
    ros::NodeHandle handle;
    
    std::string lineTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/stitched_line_topic",
                      lineTopic);
    line_sub = handle.subscribe(lineTopic, 1, &MapRepresentation::lineCallback, this);
    
    ROSUtil::getParam(handle, "/map_representation/grid_resolution", grid.info.resolution);

    /*tf::Transform transform;
    tf::StampedTransform stampedtransform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    //stampedtransform.something = transform;
    stampedtransform.stamp_ = ros::Time::now();
    stampedtransform.frame_id_ = "map_frame";
    br.sendTransform(stampedtransform);*/
    
    runNode();
}

void MapRepresentation::runNode(){
    ros::Rate loopRate(10);
    // wait to receive lines from the topic
    while (!receivedLines && ros::ok()){
        loopRate.sleep();
        ros::spinOnce();
    }

    // received lines, so process them and populate the occupancy grid
    populateGrid();
}

void MapRepresentation::lineCallback(const mapping_msgs::SegmentLineVector& msg){
    segLineVec = msg;
    receivedLines = true; // start processing the lines to create the map
}

void MapRepresentation::populateGrid(){
    std::vector<mapping_msgs::LineVector> segmentLines = segLineVec.segments;
    MinMaxXY gridBounds = findSegmentBounds(segmentLines);
    
    // Set the width and height of the grid in cells based on the bounds of the
    // lines provided, and the resolution of the grid.
    float widthMetres = gridBounds.maxX - gridBounds.minX;
    float heightMetres = gridBounds.maxY - gridBounds.minY;
    grid.info.width = (int)(std::ceil(widthMetres/grid.info.resolution));
    grid.info.height = (int)(std::ceil(heightMetres/grid.info.resolution));
    // initialise the grid data, with each cell undefined.
    grid.data = std::vector<signed char>(grid.info.width * grid.info.height, -1);
    
    // iterate over all the lines received 
    std::vector<mapping_msgs::LineVector>::iterator it = segmentLines.begin();
    for (; it != segmentLines.end(); it++) {
        mapping_msgs::LineVector segment = *it;
        // Find the bounds of a single segment
        MinMaxXY segmentBounds = findSegmentBounds(segment);
        // get the polygon representing the bound, will be a rectangle
        geometry_msgs::Polygon bound = boundsToPolygon(segmentBounds);
        // set the value of all cells in the polygon to empty
        setCellsInBounds(bound, 0);
        // for each individual line (wall) in the segment, set cells which fall under
        // the line to occupied.
        std::vector<mapping_msgs::Line> linesInSegment = segment.lines;
        for (size_t i = 0; i < linesInSegment.size(); i++){
            setCellsOnLine(linesInSegment[i], 100);
        }
    }
}

/**
 * Find min and max of the set of linevectors given, in metres. Basically the
 * bounding box.
 */
MinMaxXY MapRepresentation::findSegmentBounds(std::vector<mapping_msgs::LineVector> segmentLines) {
    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();

    for(size_t segmentInd = 0; segmentInd < segmentLines.size(); segmentInd++){
        mapping_msgs::LineVector lines = segmentLines[segmentInd];
        MinMaxXY minmax = findSegmentBounds(lines);
        setMinMaxFromStruct(minmax, minX, maxX, minY, maxY);
    }

    return MinMaxXY(minX, maxX, minY, maxY);
}

/**
 * Find the min and max of a single vector of lines, returning the bounding box
 * of those lines.
 */
MinMaxXY MapRepresentation::findSegmentBounds(mapping_msgs::LineVector segmentLine){
    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    
    std::vector<mapping_msgs::Line> lines = segmentLine.lines;

    for (size_t lineInd = 0; lineInd < lines.size(); lineInd++){
        MinMaxXY minmax = lineMinMax(lines[lineInd]);
        setMinMaxFromStruct(minmax, minX, maxX, minY, maxY);
    }
        
    return MinMaxXY(minX, maxX, minY, maxY);
}

/**
 * Sets the values of minX, maxX, minY, maxY by comparing to the given MinMaxXY
 * struct. if any of the values are less than or equal to their counterparts
 * given in the parameters, their values are modified to correspond to values in
 * the struct.
 */
void MapRepresentation::setMinMaxFromStruct(MinMaxXY comp, float& minX, float& maxX, float& minY, float& maxY){
    if (comp.minX < minX){
        minX = comp.minX;
    }
    if (comp.maxX > maxX){
        maxX = comp.maxX;
    }
    if (comp.minY < minY){
        minY = comp.minY;
    }
    if (comp.maxY > maxY){
        maxY = comp.maxY;
    }
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
 * Gets the bounding box of the lines provided. Takes into account the fact
 * that the lines may be of different length and orientation.
 */
geometry_msgs::Polygon MapRepresentation::boundsToPolygon(MinMaxXY bounds){
    geometry_msgs::Polygon poly;
    
    geometry_msgs::Point32 botLeft;
    botLeft.x = bounds.minX;
    botLeft.y = bounds.minY;
    botLeft.z = 0;
    geometry_msgs::Point32 topLeft;
    botLeft.x = bounds.minX;
    botLeft.y = bounds.maxY;
    botLeft.z = 0;
    geometry_msgs::Point32 topRight;
    botLeft.x = bounds.maxX;
    botLeft.y = bounds.maxY;
    botLeft.z = 0;
    geometry_msgs::Point32 botRight;
    botLeft.x = bounds.maxX;
    botLeft.y = bounds.minY;
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
void MapRepresentation::setCellsOnLine(mapping_msgs::Line lineToProject, signed char value){
    ocutil::RayTraceIterRange iterators = ocutil::rayTrace(grid.info,
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
