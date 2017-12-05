#ifndef FEATURE_H
#define FEATURE_H

#include <ros/ros.h>

#include <vector>
#include <math.h>

#include "laser_sensor.h"
#include "laser_line_extraction/LineSegmentList.h"

#define LINE_TOLERANCE 0.8
#define ANGLE_TOLERANCE 0.2


struct line
{
    line(float x_start, float x_end, float y_start, float y_end)
        : x_start(x_start), x_end(x_end), y_start(y_start), y_end(y_end)
    {
        angle = atan2(y_end - y_start, x_end - x_start);
    };

    float x_start, x_end, y_start, y_end, angle;

    // Identify if the line was already used to extract some feature, it's used to avoid duplicates
    bool already_used;
};


// struct corridor
// {
//     // A corridor is defined by two lines (walls)
//     corridor(line *wall_1, line *wall_2)
//         : wall_1(wall_1), wall_2(wall_2)
//     {
//         // Calculate the width between the walls center
//         float distance_x = ((wall_1->x_start + wall_1->x_end) / 2) - ((wall_2->x_start + wall_2->x_end) / 2);

//         float distance_y = ((wall_1->y_start + wall_1->y_end) / 2) - ((wall_2->y_start + wall_2->y_end) / 2);

//         width = sqrt(distance_x * distance_x + distance_y * distance_y);
//     };

//     line *wall_1;
//     line *wall_2;
//     float width;
// };


class Feature
{
private:
    void process_lines();
    void reset_used_lines();
    void extract_corridors();
    void extract_corners();
    float smallest_distance(const line & line1, const line & line2);
    float calculate_distance(float x1, float x2, float y1, float y2);

	ros::Subscriber subscriber;
    laser_line_extraction::LineSegmentList::ConstPtr msg;
    const LaserSensor& laser_sensor;
    
    // Stores if the data in the msg was processed
    bool data_already_processed;

    // Stores processed lines
    std::vector<line> lines;

    // Store number of each feature being seen by the Pionner
    double corridor_width;
    uint internal_corners_count;
    uint external_corners_count;

public:

    Feature(ros::NodeHandle& nodeHandle, const LaserSensor& _laser_sensor);
    void callback(const laser_line_extraction::LineSegmentList::ConstPtr& msg);
    void process();

    double get_corridor_width() { return corridor_width; };
    uint get_internal_corners_count() { return internal_corners_count; };
    uint get_external_corners_count() { return external_corners_count; };

    bool has_changed;
};

#endif
