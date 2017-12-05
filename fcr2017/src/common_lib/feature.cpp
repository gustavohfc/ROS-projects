
#include "common.h"
#include "feature.h"

Feature::Feature(ros::NodeHandle& nodeHandle, const LaserSensor& _laser_sensor)
    : data_already_processed(true), laser_sensor(_laser_sensor), has_changed(false)
{
    subscriber = nodeHandle.subscribe("line_segments", 1, &Feature::callback, this);
}


void Feature::callback(const laser_line_extraction::LineSegmentList::ConstPtr& msg)
{
    data_already_processed = false;
    this->msg = msg;
}


void Feature::process()
{
    // Check if the current data is already processed
    if (data_already_processed)
        return;

    process_lines();
    extract_corridors();
    extract_corners();

    has_changed = true;
}


// Process the received message and extract the useful lines
void Feature::process_lines()
{
    lines = std::vector<line>();

    for (int i = 0; i < msg->line_segments.size(); i++)
    {
        bool is_new_line = true;
        line new_line(msg->line_segments[i].start[0], msg->line_segments[i].end[0],
                      msg->line_segments[i].start[1], msg->line_segments[i].end[1]);

        // Check if there is any NAN
        if (isnan(new_line.x_start) || isnan(new_line.y_start) || isnan(new_line.x_end) || isnan(new_line.y_end))
            continue;

        // Aparently this isn't needed, the laser_line_extraction already does that
        // Try to connect with another line with near points and same inclination
        // for (std::vector<line>::iterator it = lines.begin() ; it != lines.end(); ++it)
        // {
        //     if (fabs(new_line.x_start - it->x_end) < LINE_TOLERANCE &&
        //         fabs(new_line.y_start - it->y_end) < LINE_TOLERANCE &&
        //         fabs(new_line.angle - it->angle) < LINE_TOLERANCE)
        //     {
        //         break;
        //     }
        // }

        if (is_new_line)
            lines.push_back(new_line);
    }
}


// Set all lines as not used
void Feature::reset_used_lines()
{
    for (int i = 0; i < lines.size(); i++)
    {
        lines[i].already_used = false;
    }
}


// Extract the corridors from the vector lines
void Feature::extract_corridors()
{
    reset_used_lines();
    corridor_width = 0;

    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = 0; j < lines.size(); j++)
        {
            if (i != j && !lines[i].already_used && !lines[j].already_used)
            {
                lines[i].already_used = true;
                lines[j].already_used = true;
                corridor_width = laser_sensor.getWidth();
            }
        }
    }
}


// Extract the corners features from the vector lines
void Feature::extract_corners()
{
    reset_used_lines();
    internal_corners_count = 0;
    external_corners_count = 0;

    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = 0; j < lines.size(); j++)
        {
            if (i != j && !lines[i].already_used && !lines[j].already_used &&
                smallest_distance(lines[i], lines[j]) < LINE_TOLERANCE &&
                fabs(fabs(fabs(lines[i].angle) - fabs(lines[j].angle)) - degreesToRadians(90)) < ANGLE_TOLERANCE)
            {
                lines[i].already_used = true;
                lines[j].already_used = true;

                // Check if the corner is internal or external
                if (fabs(fabs(lines[i].x_end) - fabs(lines[j].x_start)) < LINE_TOLERANCE)
                    internal_corners_count++;
                else
                    external_corners_count++;
            }
        }
    }
}


// Calculate the smallest distance between the extremities of two lines
float Feature::smallest_distance(const line & line1, const line & line2)
{
    float distance_start_start = calculate_distance(line1.x_start, line2.x_start, line1.y_start, line2.y_start);

    float distance_start_end = calculate_distance(line1.x_start, line2.x_end, line1.y_start, line2.y_end);

    float distance_end_start = calculate_distance(line1.x_end, line2.x_start, line1.y_end, line2.y_start);

    float distance_end_end = calculate_distance(line1.x_end, line2.x_end, line1.y_end, line2.y_end);

    return std::min(std::min(std::min(distance_start_start, distance_start_end),
                             distance_end_start),
                    distance_end_end);
}


// Calculate the distance between two points
float Feature::calculate_distance(float x1, float x2, float y1, float y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}