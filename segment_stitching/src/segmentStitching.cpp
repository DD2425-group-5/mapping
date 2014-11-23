#include "segmentStitching.hpp"

SegmentStitching::SegmentStitching(int argc, char *argv[]) {
    ros::init(argc, argv, "segment_stitching");
    ros::NodeHandle handle;
    
    std::string segmentTopic;
    ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_storage/bag_segment_topic",
                      segmentTopic);

    std::string segmentFile;
    ROSUtil::getParam(handle, "/segment_stitching/input_file",
                      segmentFile);

    ROSUtil::getParam(handle, "/segment_stitching/ransac_threshold", ransacThreshold);

    segcloud_pub = handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/segment_stitching/segcloud", 1);
    linemarker_pub = handle.advertise<visualization_msgs::Marker>("/segment_stitching/linemarkers", 1);
	
	std::string lineTopic;
	ROSUtil::getParam(handle, "/topic_list/mapping_topics/segment_stitching/published/stitched_line_topic",
						lineTopic);
	stitched_pub = handle.advertise<mapping_msgs::LineVector>(lineTopic, 1);
	
    // Initialise the locations of the IR sensors relative to the centre of the robot.
    populateSensorPositions(handle);

    // open the bag to read it
    segmentBag.open(segmentFile, rosbag::bagmode::Read);
    
    // define the topics to read
    std::vector<std::string> topics;
    topics.push_back(segmentTopic);
    
    // define a view onto the bag file - only interested in one topic
    rosbag::View view(segmentBag, rosbag::TopicQuery(topics));

    ROS_INFO("Reading segments from %s", segmentFile.c_str());
    // Iterate over messages in the bag
    for (rosbag::View::iterator it = view.begin(); it != view.end(); it++){
        // extract the messageInstance that the iterator points to
        rosbag::MessageInstance mi = *it;
        // Extract the segment from the bag
        mapping_msgs::MapSegment segment = *(mi.instantiate<mapping_msgs::MapSegment>());
        // Put it into the vector of segments
        mapSegments.push_back(segment);
    }

    ROS_INFO("Number of segments: %d", (int)mapSegments.size());

    runNode();
}

void SegmentStitching::runNode(){

    std::vector<std::vector<Line> > segmentLines;
    // Go through all the segments, extract measurements, and convert these to lines.
    for (size_t segment = 0; segment < mapSegments.size(); segment++) {
        ROS_INFO("Processing segment %d of %d", (int)(segment + 1), (int)(mapSegments.size()));
        // first, get the set of points which represent the measurements taken
        // in the segment. 
        pcl::PointCloud<pcl::PointXYZ>::Ptr measurements(new pcl::PointCloud<pcl::PointXYZ>);
        segmentToMeasurements(mapSegments[segment], measurements);
        
        // Extract the lines from this segment using ransac. This operation
        // destroys the measurement pointcloud
        std::vector<Line> lines = extractLinesFromMeasurements(measurements, ransacThreshold);
        segmentLines.push_back(lines);
        stitchSegmentLines(segmentLines);
        tmpPublish(measurements, lines);
    }
}

void SegmentStitching::tmpPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr cl, std::vector<Line> lines){
    // define the frame for the map
    // NEED TO PUBLISH THE MAP FRAME!
    cl->header.frame_id = "camera_link";
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.ns = "RANSACline";
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    for (size_t i = 0; i < lines.size(); i++){
        geometry_msgs::Point start;
        start.x = lines[i].start.x;
        start.y = lines[i].start.y;
        start.z = lines[i].start.z;
        
        geometry_msgs::Point end;
        end.x = lines[i].end.x;
        end.y = lines[i].end.y;
        end.z = lines[i].end.z;
        marker.points.push_back(start);
        marker.points.push_back(end);
        ROS_INFO_STREAM("Line start: " << start << ", line end: " << end);
    }

    ros::Rate loop(10);
    while(ros::ok()){
        segcloud_pub.publish(cl);
        linemarker_pub.publish(marker);
        ros::spinOnce();
        loop.sleep();
    }
}

void SegmentStitching::publishFinalLines(std::vector<std::vector<Line> > lines){
	mapping_msgs::LineVector lv;
	
	for (size_t i = 0; i < lines.size(); i++) {
		for (size_t j = 0; j < lines[i].size(); j++){
			mapping_msgs::Line l = lines[i][j];
			l.id = i; // id corresponds to the segment the line came from
		}
	}
	stitched_pub.publish(lv);
}

/**
 * Read data from the robot_info parameter and construct the offset coordinates
 * of the sensors
 */
void SegmentStitching::populateSensorPositions(ros::NodeHandle handle){
    int numSensors;
    ROSUtil::getParam(handle, "/robot_info/num_sensors", numSensors);
    std::string xSuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_x_suffix", xSuffix);
    std::string ySuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_y_suffix", ySuffix);
    std::string zSuffix;
    ROSUtil::getParam(handle, "/robot_info/sensor_z_suffix", zSuffix);
    std::vector<std::string> sensor_names;
    ROSUtil::getParam(handle, "/robot_info/sensor_names", sensor_names);
    
    for (size_t i = 0; i < sensor_names.size(); i++) {
        std::string xOffParam = std::string("/robot_info/" + sensor_names[i] + xSuffix);
        std::string yOffParam = std::string("/robot_info/" + sensor_names[i] + ySuffix);
        std::string zOffParam = std::string("/robot_info/" + sensor_names[i] + zSuffix);
        std::string rotParam = std::string("/robot_info/" + sensor_names[i] + "_rotation");

        float xoff;
        ROSUtil::getParam(handle, xOffParam, xoff);
        float yoff;
        ROSUtil::getParam(handle, yOffParam, yoff);
        float zoff;
        ROSUtil::getParam(handle, zOffParam, zoff);
        float rot;
        ROSUtil::getParam(handle, rotParam, rot);
        
        sensors.push_back(IRSensor(sensor_names[i], xoff, yoff, zoff, rot));
        ROS_INFO_STREAM(sensors[i]);
        std::cout << sensor_names[i] << " pcl point: " << sensors[i].asPCLPoint() << std::endl;
    }
}

/**
 * Convert a whole segment into a set of points corresponding to measurements
 * taken by IR sensors during the motion in the segment. The pointcloud passed
 * to this function will be populated with the points.
 */
void SegmentStitching::segmentToMeasurements(mapping_msgs::MapSegment segment,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr measurements){
    ROS_INFO("Points in segment: %d", (int)segment.pointList.size());
    for (size_t point = 0; point < segment.pointList.size(); point++){
        // convert the IR distances at the point to a useful coordinate space
        segmentPointToMeasurements(segment.pointList[point], measurements);
    }
    ROS_INFO("Measurements in segment: %d", (int)measurements->size());
}

/**
 * Convert a segment point into a set of points corresponding to measurements of
 * walls in the maze. No need to take into account any rotation at this point -
 * assume that the robot is moving in a perfectly straight line. Assume (0,0) is
 * the robot location at the start of the segment, and then compute the x,y
 * coordinates of where the IR reading is measured according to the sensor
 * positions and distances received. Only consider the side facing sensors.
 * The pointcloud passed to this function will be populated with the points.
 *
 */
void SegmentStitching::segmentPointToMeasurements(mapping_msgs::SegmentPoint pt,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr measurements){
    hardware_msgs::IRDists dists = pt.distances;
    hardware_msgs::Odometry odom = pt.odometry;
    
    float distances[6] = {dists.s0, dists.s1, dists.s2, dists.s3, dists.s4, dists.s5};
    // The robot base has moved relative to the start of the segment
    pcl::PointXYZ odompt(0, odom.distanceTotal, 0);
    // only look at first 4 sensors, last 2 are front facing, assume either -90 or 90 rotation
    for (size_t i = 0; i < sensors.size() - 2; i++){
        IRSensor s = sensors[i];
        // naive way of getting point measurement - if sensor rotated -90,
        // subtract from x, otherwise add. The generated points will be rotated
        // later to match segment rotation if necessary
        if (s.rotation == -90) { // might be dangerous, rotation is a float
            distances[i] = -distances[i];
        }
        pcl::PointXYZ p = s.asPCLPoint() + odompt + pcl::PointXYZ(distances[i], 0, 0);
        // std::cout << "adding " << s.asPCLPoint() << ", " << pcl::PointXYZ(distances[i], 0, 0) << std::endl;
        // std::cout << "result: " << p << std::endl;
        measurements->push_back(p);
    }
}

/**
 * Extract lines from a set of points using ransac
 */
std::vector<Line> SegmentStitching::extractLinesFromMeasurements(pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                                                                 float ransacThreshold){
    std::vector<Line> lines;
    // will pass this vector to ransac to get the line model - we then need to
    // use it to remove the inliers of the model so that we can process other
    // lines in the data
    std::vector<int>* inliers = new std::vector<int>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr processing(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed(new pcl::PointCloud<pcl::PointXYZ>);
    *processing = *measurements; // copy stuff from the measurement cloud to be processed

    // naive: extract two lines from each segment. TODO: extraction of lines
    // based on proportion of points remaining in the cloud once the first line
    // has been removed.
    for (int i = 0; i < 2; i++) {
        // ROS_INFO("Trimmed size: %d", (int)trimmed->size());
        // ROS_INFO("Processing size: %d", (int)processing->size());

        // Extract a line from the measurements, and put the inlier references into inliers.
        lines.push_back(extractLineFromMeasurements(processing, ransacThreshold, inliers));
        // ExtractIndices expects a different type than vector, so create that
        pcl::PointIndices::Ptr inl(new pcl::PointIndices);
        inl->indices = *inliers;

        // Trim the cloud down to those points which are not the inliers of the
        // model that was discovered by ransac.
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(processing);
        extract.setIndices(inl);
        extract.setNegative(true); // return points which are NOT the indices
        extract.filter(*trimmed);

        // Update the points to be processed for the next loop
        *processing = *trimmed;
    
        std::cout << lines[i] << std::endl;
    }

    return lines;
}

Line SegmentStitching::extractLineFromMeasurements(pcl::PointCloud<pcl::PointXYZ>::Ptr measurements,
                                                   float ransacThreshold,
                                                   std::vector<int>* inliers){
    // Create the sample consensus object for the received cloud
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
        modelLine(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(measurements));
            
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelLine);
    ransac.setDistanceThreshold(ransacThreshold);
    ransac.computeModel();
    ransac.getInliers(*inliers);

    // Vector of 6 points
    // see http://docs.pointclouds.org/trunk/classpcl_1_1_sample_consensus_model_line.html
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    
    for (int i = 0; i < coeff.size(); i++){
        ROS_INFO("Coeff %d: %f", i, coeff[i]);
    }
    ROS_INFO("#Inliers: %d", (int)inliers->size());

    // Project inliers onto the line so that we end up with a cloud where
    // min+max can be found in order to define a line segment
    pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
    // project the inliers onto the model, without copying non-inliers over
    modelLine->projectPoints(*inliers, coeff, *projected, false);
    ROS_INFO("Projected size: %d", (int)projected->size());
    pcl::PointCloud<pcl::PointXYZ>::iterator it = projected->begin();

    // Segments have start and end points. A point is at the start or end of the
    // line if one of its coordinates corresponds to the minimum or maximum
    // value of either of the coordinate axes
    int xmaxInd = -1;
    int xminInd = -1;
    int ymaxInd = -1;
    int yminInd = -1;
    
    float xmax = -std::numeric_limits<float>::max();
    float ymax = -std::numeric_limits<float>::max();
    float xmin = std::numeric_limits<float>::max();
    float ymin = std::numeric_limits<float>::max();
    for (; it != projected->end(); it++) {
        // ROS_INFO_STREAM("Point " <<  (int)(it - projected->begin()) << ": " << *it);
        // std::cout << "x: " << it->x << ", max: " << xmax << " x bigger? " << (it->x > xmax) << std::endl;
        if (it->x > xmax){
            xmax = it->x;
            // index of this point is 
            xmaxInd = it - projected->begin();
            //ROS_INFO("X max ind: %d with val %f", xmaxInd, xmax);
        }
        if (it->y > ymax){
            ymax = it->y;
            // index of this point is 
            ymaxInd = it - projected->begin();
            //ROS_INFO("Y max ind: %d with val %f", ymaxInd, ymax);
        }
        if (it->x < xmin){
            xmin = it->x;
            // index of this point is 
            xminInd = it - projected->begin();
            //ROS_INFO("X min ind: %d with val %f", xminInd, xmin);
        }
        if (it->y < ymin){
            ymin = it->y;
            // index of this point is 
            yminInd = it - projected->begin();
            //ROS_INFO("Y min ind: %d with val %f", yminInd, ymin);
        }
    }
    ROS_INFO("xmin ind %d, xmax ind %d, ymin ind %d, ymax ind %d", 
             xminInd, xmaxInd, yminInd, ymaxInd);
    ROS_INFO("xmin %f, xmax %f, ymin %f, ymax %f", 
             xmin, xmax, ymin, ymax);

    // If the x or y coordinates are identical, then you have a vertical or
    // horizontal line - define start and end points of the line with the value
    // of xmin or max, and then assign values for the other coordinate to the
    // start and end point arbitrarily. You should be able to do this min/max
    // comparison based on the indices of xmin and xmax - they should be the same
    // if lines are horizontal or vertical
    if (xminInd == xmaxInd){
        return Line(pcl::PointXYZ(xmin, ymin, 0), pcl::PointXYZ(xmin, ymax, 0));
    } else if (yminInd == ymaxInd){
        return Line(pcl::PointXYZ(xmin, ymin, 0), pcl::PointXYZ(xmax, ymin, 0));
    } else {
        // TODO: Make sure this works correctly
        // This is a line which is not horizontal or vertical - the indices of
        // xmin,ymin or xmax, ymax should match. 
        if (xminInd == yminInd && xmaxInd == ymaxInd){
            return Line(pcl::PointXYZ(xmin, ymin, 0), pcl::PointXYZ(xmax, ymax, 0));
        } else if (xmaxInd == yminInd && xminInd == ymaxInd){
            return Line(pcl::PointXYZ(xmax, ymin, 0), pcl::PointXYZ(xmin, ymax, 0));
        }
    }
}


/* rotate a single Line line*/
Line SegmentStitching::rotateLine(Line lineToRotate, float angle){
    float angleInRadians = (M_PI*angle)/180.0;
    
    
    //create empty cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr lineToRotateAsCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr lineRotatedAsCloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    //push line into the cloud
    lineToRotateAsCloud->push_back(lineToRotate.start);
    lineToRotateAsCloud->push_back(lineToRotate.end);
        
    //call rotateCloud
    lineRotatedAsCloud = PCLUtil::rotateCloud(lineToRotateAsCloud, angle);
    
    //extract rotated line from rotated cloud
    pcl::PointCloud<pcl::PointXYZ>::iterator startElement = lineRotatedAsCloud->begin();
    pcl::PointCloud<pcl::PointXYZ>::iterator endElement = lineRotatedAsCloud->end();
    Line rotatedLine(*startElement, *endElement);
    ROS_INFO_STREAM("Extracted rotated point " <<  (int)(startElement -\
                                                    lineRotatedAsCloud->begin()) << ": " << *startElement);
    ROS_INFO_STREAM("Extracted rotated point " <<  (int)(endElement - \
                                                    lineRotatedAsCloud->begin()) << ": " << *endElement);
    return rotatedLine;
}


/**
 * Combine the lines extracted from segments into a single set of lines which
 * hopefully represent the walls in the maze. The first segment starts at 0,0
 * and other segments need to be rotated to correspond to the relative rotation
 * of the segment that precedes them, based on the turn made by the robot. Just
 * rotating the points that correspond to the line start and end and then
 * rotating around the point that ends the previous segment should work?
 * Transform the point we are rotating around to the origin, rotate the points,
 * then translate back.
 *
 * Should return the lines for the segments rotated and translated to the
 * correct position relative to the map.
 */
std::vector<std::vector<Line> > SegmentStitching::stitchSegmentLines(std::vector<std::vector<Line> > linesInSegments){
    // Keep a list of the positions of the robot at the beginning and end of
    // each segment, relative to the map, as opposed to each segment.
    std::vector<pcl::PointXYZ> segmentPointChain;
    segmentPointChain.push_back(pcl::PointXYZ(0,0,0)); // chain starts at the origin
    std::vector<std::vector<Line> > stitchedLines;
    
	float xStart=0;	// start x position of line
	float yStart=0;	// start y poistion of line
	int xDir=1;		// direction x
	int yDir=0;		// direction y
	/*float xEnd=0;
	float yEnd=0;*/
	
    for (size_t i = 0; i < linesInSegments.size(); i++) {
        // End point of the segment is at the end of the list. Don't need to
        // extract the first point because the values are all zero.
        std::vector<mapping_msgs::SegmentPoint> segList = mapSegments[i].pointList;
        mapping_msgs::SegmentPoint segmentEnd = segList[segList.size() - 1];
        
        // Extract the points from the segmentpoint. Start point for the segment
        // should always be (0,0,0). The end point corresponds to the position
        // of the robot at the end of the segment. We assume no motion in the x
        // direction.
        pcl::PointXYZ segmentStartPoint(xStart, yStart, 0);
        pcl::PointXYZ segmentEndPoint(0, segmentEnd.odometry.distanceTotal, 0);

        if (i == 0) {
            // for the first segment, there is no need to modify the end point
            segmentPointChain.push_back(segmentEndPoint);
        } else {
            // need to add or subtract from x or y depending on the turn
            // direction given in the previous mapsegment.
            if(mapSegments[i].turnDirection == mapSegments[i-1].LEFT_TURN){
				if(1==xDir && 0==yDir){
					xDir=0;
					yDir=1;
				}
				else if(0==xDir && 1==yDir){
					xDir=-1;
					yDir=0;
				}
				else if(-1==xDir && 0==yDir){
					xDir=0;
					yDir=-1;
				}
				else{
					xDir=1;
					yDir=0;
				}
			}
			else if(mapSegments[i].turnDirection == mapSegments[i-1].RIGHT_TURN){
				if(1==xDir && 0==yDir){
					xDir=0;
					yDir=-1;
				}
				else if(0==xDir && -1==yDir){
					xDir=-1;
					yDir=0;
				}
				else if(-1==xDir && 0==yDir){
					xDir=0;
					yDir=1;
				}
				else{
					xDir=1;
					yDir=0;
				}
			}
			else if(mapSegments[i].turnDirection == mapSegments[i-1].U_TURN){
				if(xDir!=0){
					xDir=-xDir;
				}
				else if(yDir!=0){
					yDir=-yDir;
				}
			}
			
			//Just add the difference and change the segmentEndPoint x and y
			float newX = xDir * segmentEnd.odometry.distanceTotal;
			float newY = yDir * segmentEnd.odometry.distanceTotal;
			newX = newX + xStart;
			newY = newY + yStart;
			segmentEndPoint.x = newX;
			segmentEndPoint.y = newY;
			
			segmentPointChain.push_back(segmentEndPoint);
        }
		xStart=segmentEndPoint.x;
		yStart=segmentEndPoint.y;
    }
	return stitchedLines;
}

/**
 * Create an occupancy grid from the stitched lines. Once these are extracted it
 * should be possible to simply travel along the line with a small step, and
 * populate all the points in the occupancy grid that the line passes through.
 */
void SegmentStitching::createOccupancyGrid(){
}


int main(int argc, char *argv[])
{
    SegmentStitching stitch(argc, argv);
}
