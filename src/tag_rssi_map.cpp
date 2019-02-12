#include "pipe_sim/tag_rssi_map.h"

TagRSSIMap::TagRSSIMap(long epc, std::list<Point> points)
    :epc_(epc), points_(points)
{
    interval_ = calculateInterval();
    bounds_ = calculateBounds();
}

TagRSSIMap::TagRSSIMap(bool direction, double start_distance, double map_distance, double length, double interval,
                       int32_t epc, double variance, double a, double b, double c, double d)
{
    std::list<Point> points;
    if(direction)
    {
        for(double distance = start_distance; distance <= start_distance + length && distance <= map_distance; distance = distance + interval)
        {
            double rssi = a + b * (distance - start_distance) + c * sin(d * (distance - start_distance));
            Point point(rssi, distance, variance);
            points.push_back(point);
        }
    }
    else
    {
        for(double distance = start_distance; distance >= start_distance - length && distance >= 0; distance = distance - interval)
        {
            double rssi = a - b * (distance - start_distance) - c * sin(d * (distance - start_distance));
            Point point(rssi, distance, variance);
            points.push_back(point);
        }
    }
    points_ = points;
    epc_ = epc;
}

double TagRSSIMap::calculateInterval()
{
    std::list<double> intervals;
    auto prior_it = points_.begin();
    for(auto point_it = ++points_.begin(); point_it != points_.end(); point_it++)
    {
        prior_it = point_it;
        prior_it--;
        intervals.push_back(fabs(prior_it->getDistance() - point_it->getDistance()));
    }
    intervals.sort();
    double interval;
    int interval_count = 0;
    double previous = 0;
    int count = 0;
    for(auto it = intervals.begin(); it != intervals.end(); it++)
    {
        if(fabs(*it - previous) < EPSILON) count ++;
        else
        {
            if(count > interval_count)
            {
                interval = previous;
                interval_count = count;
            }
            count = 0;
        }
        previous = *it;
    }
    if(count > interval_count)
    {
        interval = previous;
        interval_count = count;
    }
    return interval;
}

std::vector<std::pair<double, double>> TagRSSIMap::calculateBounds()
{
    std::vector<std::pair<double, double>> bounds;
    std::pair<double, double> bound;
    bound.first = points_.front().getDistance();
    double previous = points_.front().getDistance();
    for(auto point_it = ++points_.begin(); point_it != points_.end(); point_it++)
    {
        if(fabs(interval_ - fabs(previous - point_it->getDistance())) > EPSILON)
        {
            bound.second = previous;
            bounds.push_back(bound);
            bound.first = point_it->getDistance();
        }
        previous = point_it->getDistance();
    }
    bound.second = points_.back().getDistance();
    bounds.push_back(bound);
    return bounds;
}


long TagRSSIMap::getEPC() const
{
    return epc_;
}

std::vector<std::pair<double, double>> TagRSSIMap::getBounds()
{
    return bounds_;
}

std::list<Point> TagRSSIMap::getPoints() const
{
    return points_;
}

std::vector<pipe_sim::Point> TagRSSIMap::findIntersections(double rssi, double range)
{
    std::vector<pipe_sim::Point> points;
    std::vector<Point> short_list;

    bool in_range = false;
    for(auto points_it = points_.begin(); points_it != points_.end();
        points_it++)
    {
        if( fabs(points_it->getRSSI() - rssi) < range)
        {
            short_list.push_back(*points_it);
            in_range = true;
        }
        else
        {
            if(in_range)
            {
                double difference = range;
                Point point;
                for(auto short_list_it = short_list.begin();
                    short_list_it != short_list.end(); short_list_it++)
                {
                    if(fabs(short_list_it->getRSSI() - rssi) < difference)
                    {
                        point = *short_list_it;
                        difference = fabs(short_list_it->getRSSI() - rssi);
                    }
                }
                pipe_sim::Point msg_point;
                msg_point.distance = point.getDistance();
                msg_point.variance = point.getVariance();
                points.push_back(msg_point);
                short_list.clear();
                in_range = false;
            }
        }
    }
    return points;
}

bool TagRSSIMap::checkDistance(double distance)
{
    for(auto bound:bounds_)
    {
        if(distance > bound.first && distance < bound.second) return true;
    }
    return false;
}



pipe_sim::Point TagRSSIMap::getPointDistance(double distance)
{
    if(checkDistance(distance))
    {
        double min_difference = 1000;
        pipe_sim::Point min_point;
        for(auto point:points_)
        {
            if(fabs(distance - point.getDistance()) < min_difference)
            {
                min_difference = fabs(distance - point.getDistance());
                min_point.distance = point.getDistance();
                min_point.variance = point.getVariance();
                min_point.rssi = point.getRSSI();
                min_point.epc = getEPC();
            }
        }
        return min_point;
    }
}

void TagRSSIMap::print()
{
    std::cout << "EPC: " << epc_ << std::endl;
    std::cout << "Inverval: " << interval_ << std::endl;
    std::cout << "Points: " << std::endl;
    for(auto it = points_.begin(); it != points_.end(); it++)
    {
        std::cout << "   - ";
        it->print();
    }
    std::cout << "Bounds: " << std::endl;
    for(auto it = bounds_.begin(); it != bounds_.end(); it++)
    {
        std::cout << "   - { " << it->first << " - " << it->second << " }" << std::endl;
    }
}

