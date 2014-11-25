#include "mapRepresentation.hpp"

MapRepresentation::MapRepresentation(int argc, char *argv[]){
    ros::init(argc, argv, "map_representation");
    ros::NodeHandle handle;
    
    std::string resultTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/results_topic",
                      resultTopic);
    result_sub = handle.subscribe(resultTopic, 1, &MapRepresentation::stitchingCallback, this);

    std::string mapTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/map_representation/published/map_topic",
                      mapTopic);
    map_pub = handle.advertise<nav_msgs::OccupancyGrid>(mapTopic, 1);
    
    ROSUtil::getParam(handle, "/map_representation/grid_resolution", grid.info.resolution);
    
    runNode();
}

void MapRepresentation::runNode(){
    ros::Rate checkRate(10);
    // wait to receive lines from the topic
    while (!receivedLines && ros::ok()){
        checkRate.sleep();
        ros::spinOnce();
    }

    // received lines, so process them and populate the occupancy grid
    populateGrid();

    // publish the map at 1hz
    ros::Rate mapPubRate(1);
    while (ros::ok()){
        mapPubRate.sleep();
        map_pub.publish(grid);
    }
}

void MapRepresentation::stitchingCallback(const mapping_msgs::StitchingResults& msg){
    ROS_INFO("Received segment line vector representing map");
    stitchResults = msg;
    receivedLines = true; // start processing the lines to create the map
}

void MapRepresentation::populateGrid(){
    std::vector<mapping_msgs::LineVector>& segmentLines = stitchResults.lines.segments;

    MinMaxXY gridBounds = findSegmentBounds(segmentLines);
    translateToOrigin(segmentLines, gridBounds);
    alignToAxes(segmentLines);
    ROS_INFO_STREAM("Grid bounds: " << gridBounds);

    // Set the width and height of the grid in cells based on the bounds of the
    // lines provided, and the resolution of the grid.
    float widthMetres = gridBounds.maxX - gridBounds.minX;
    float heightMetres = gridBounds.maxY - gridBounds.minY;
    ROS_INFO("Grid height: %fm, width: %fm", widthMetres, heightMetres);
    grid.info.width = (int)(std::ceil(widthMetres/grid.info.resolution));
    grid.info.height = (int)(std::ceil(heightMetres/grid.info.resolution));
    // initialise the grid data, with each cell undefined.
    grid.data = std::vector<signed char>(grid.info.width * grid.info.height, MapUtil::UNKNOWN);

    // iterate over all the lines received 
    std::vector<mapping_msgs::LineVector>::iterator it = segmentLines.begin();
    for (; it != segmentLines.end(); it++) {
        mapping_msgs::LineVector segment = *it;
        // Find the bounds of a single segment
        MinMaxXY segmentBounds = findSegmentBounds(segment);
        // get the polygon representing the bound, will be a rectangle
        ROS_INFO_STREAM("Bounds of segment " << it - segmentLines.begin() << ": " << segmentBounds);
        geometry_msgs::Polygon bound = boundsToPolygon(segmentBounds);
        ROS_INFO_STREAM("Polygon created from bounds: " << bound);
        // set the value of all cells in the polygon to empty
        setCellsInBounds(bound, MapUtil::UNOCCUPIED);

        // // for each individual line (wall) in the segment, set cells which fall under
        // // the line to occupied.
        std::vector<mapping_msgs::Line> linesInSegment = segment.lines;
        for (size_t i = 0; i < linesInSegment.size(); i++){
            setCellsOnLine(linesInSegment[i], MapUtil::OCCUPIED);
        }
    }
}

/**
 * Translate the lines in segmentLines to the origin. Modifies the segmentLines in place.
 */
void MapRepresentation::translateToOrigin(std::vector<mapping_msgs::LineVector>& segmentLines,
                                          const MinMaxXY& gridBounds){
    for (size_t segment = 0; segment < segmentLines.size(); segment++){
        mapping_msgs::LineVector& currentSegment = segmentLines[segment];
        for (size_t line = 0; line < currentSegment.lines.size(); line++){
            mapping_msgs::Line& currentLine = currentSegment.lines[line];
            currentLine.start.x -= gridBounds.minX;
            currentLine.start.y -= gridBounds.minY;
            currentLine.end.x   -= gridBounds.minX;
            currentLine.end.y   -= gridBounds.minY;
        }
    }
}

/**
 * Align the given segment lines to the x and y axes, modifying in place.
 */
void MapRepresentation::alignToAxes(std::vector<mapping_msgs::LineVector>& segmentLines){
    for (size_t segment = 0; segment < segmentLines.size(); segment++){
        mapping_msgs::LineVector& currentSegment = segmentLines[segment];
        for (size_t line = 0; line < currentSegment.lines.size(); line++){
            MapUtil::alignLineToAxisInPlace(currentSegment.lines[line]);
            
        }
    }
}

/**
 * Find min and max of the set of linevectors given, in metres. Basically the
 * bounding box.
 */
MinMaxXY MapRepresentation::findSegmentBounds(const std::vector<mapping_msgs::LineVector>& segmentLines) {
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
MinMaxXY MapRepresentation::findSegmentBounds(const mapping_msgs::LineVector& segmentLine){
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
void MapRepresentation::setMinMaxFromStruct(const MinMaxXY& comp,
                                            float& minX, float& maxX,
                                            float& minY, float& maxY){
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
MinMaxXY MapRepresentation::lineMinMax(const mapping_msgs::Line& l){
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
geometry_msgs::Polygon MapRepresentation::boundsToPolygon(const MinMaxXY& bounds){
    geometry_msgs::Polygon poly;
    
    ROS_INFO_STREAM("boundsToPolygon received bounds: " << bounds);
    geometry_msgs::Point32 botLeft;
    botLeft.x = bounds.minX;
    botLeft.y = bounds.minY;
    ROS_INFO_STREAM("Bottom left point: " << botLeft);

    geometry_msgs::Point32 topLeft;
    topLeft.x = bounds.minX;
    topLeft.y = bounds.maxY;
    ROS_INFO_STREAM("Top left point: " << topLeft);

    geometry_msgs::Point32 topRight;
    topRight.x = bounds.maxX;
    topRight.y = bounds.maxY;
    ROS_INFO_STREAM("Top right point: " << topRight);

    geometry_msgs::Point32 botRight;
    botRight.x = bounds.maxX;
    botRight.y = bounds.minY;
    ROS_INFO_STREAM("Bottom right point: " << botRight);
    
    // push points onto the polygon in an order where they make a rectangle.
    poly.points.push_back(botLeft);
    poly.points.push_back(topLeft);
    poly.points.push_back(topRight);
    poly.points.push_back(botRight);

    return poly;
}

/**
 * Set cells in the given polygon on the grid to the specified value. It is
 * assumed that the polygon lies inside the bounds of the grid, and that it is
 * aligned to the x and y axes.
 */
void MapRepresentation::setCellsInBounds(const geometry_msgs::Polygon& bounds, signed char value){
    // get cells within the polygon
    std::vector<int> inds = MapUtil::indicesInAlignedRectangle(grid.info, bounds);
    for (size_t i = 0; i < inds.size(); i++){
        grid.data[inds[i]] = value;
    }
}

/**
 * Projects a line onto the grid, setting cells which fall on the line to the
 * specified value. Assumes the line given is aligned to either the x or y axis
 */
void MapRepresentation::setCellsOnLine(const mapping_msgs::Line& line, signed char value){
    //ROS_INFO_STREAM("Line to project: " << line);
        
    std::vector<int> inds = MapUtil::indicesOnAlignedLine(grid.info, line);

    for(size_t i = 0; i < inds.size(); i++ ){
        grid.data[inds[i]] = value;
    }
}

int main(int argc, char *argv[])
{
    MapRepresentation(argc, argv);
}
