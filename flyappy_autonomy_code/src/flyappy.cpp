#include <iostream>
#include <numeric>
#include <math.h>

#include "flyappy_autonomy_code/flyappy.hpp"

Flyappy::Flyappy()
  : clearance_radius_(0.2),
    target_velocity_(0, 0),
    forward_(2, 0),
    up_(0, 35/10.0),
    down_(0, -35/10.0),
    counter_(0)
{
    reset();
}

Flyappy::Vec2D::Vec2D(float vx, float vy)
  : x(vx), y(vy)
{}

void Flyappy::Vec2D::operator=(const Vec2D& v)
{
    x = v.x;
    y = v.y;
}

bool Flyappy::Vec2D::operator==(const Vec2D& v)
{
    return (x == v.x) && (y == v.y);
}

void Flyappy::reset()
{
    target_velocity_ = forward_;
    collision_avoidance_ = false;
    found_exit_ = false;
    passing_wall_ = false;
}

void Flyappy::magnetDrive(const std::vector<float>& ranges, float angle_min, float angle_increment)
{
    float angle = angle_min;
    float max_velocity = 3;
    Vec2D direction(max_velocity, 0);

    bool passing = false;
    bool avoiding = false;
    float repel = 0;
    float attract = 0;

    if (counter_ > 30) counter_ = 0;
    else if (counter_ > 0) counter_++;
    else
    {
        for(const auto& range : ranges)
        {
            repel += -sin(angle)*(1/(range));
            attract += sin(angle)*range;
            angle += angle_increment;

            if(range < 0.8)
            {
                avoiding = true;
                if (range < 0.4)
                {
                    passing = true;
                }
            }
        }
        attract = attract / static_cast<float>(ranges.size());
        attract = attract * 30;
        repel = repel / static_cast<float>(ranges.size());
        repel = repel * 30;

        if (!passing)
        {
            if (avoiding) direction.y += repel;
            else direction.y += attract;
        }
        
        if (direction.y > max_velocity) direction.y = max_velocity;
        if (direction.y < -max_velocity) direction.y = -max_velocity;
    }
    target_velocity_ = direction;
}

void Flyappy::collisionAvoidance(const std::vector<float>& ranges, float angle_min, float angle_increment)
{
    auto collisions = collisionDetection(ranges, angle_min, angle_increment);
    if (collision_avoidance_)
    {
        if(found_exit_)
        {
            if (passing_wall_)
            {
                if (!(collisions & Floor))
                {
                    std::cout << "Collision averted" << std::endl;
                    reset();
                }
            }
            else
            {
                if (collisions & Floor)
                {
                    passing_wall_ = true;
                }
            }
        }
        else
        {
            if ((collisions & Floor) && target_velocity_ == down_)
            {
                std::cout << "Bottom collision Hazard" << std::endl;
                target_velocity_ = up_;
            }
            else if (!(collisions & Top) && target_velocity_ == up_)
            {
                std::cout << "Clear way detected above" << std::endl;
                target_velocity_ = forward_;
                found_exit_ = true;
            }
            // else if (!(collisions & Bottom) && target_velocity_ == down_)
            // {
            //     std::cout << "Clear way detected bellow" << std::endl;
            //     target_velocity_ = forward_;
            //     found_exit_ = true;
            // }
        }
    }
    else
    {
        if (collisions & Front)
        {
            std::cout << "Front collision Hazard" << std::endl;
            target_velocity_ = down_;
            collision_avoidance_ = true;
        }
    }
}

uint8_t Flyappy::collisionDetection(
    const std::vector<float>& ranges, float angle_min, float angle_increment)
{
    uint8_t collisions = None;
    float angle = angle_min;
    for(const auto& range : ranges)
    {
        if (range < 2.0)
        {
            auto y = sin(angle) * range;  // heigth
            auto x = cos(angle) * range;  // width
            if (x < 1.5 &&
                y < clearance_radius_ &&
                y > -clearance_radius_)
            {
                collisions |= Front;
            }
            if (x < 2.5 &&
                y < clearance_radius_*2 &&
                y > clearance_radius_*0)
            {
                collisions |= Top;
            }
            // TODO: adjust detection window based on current velocity
            if (x < 2.5 &&
                y < -clearance_radius_*0 &&
                y > -clearance_radius_*2)
            {
                collisions |= Bottom;
            }
            if (x < 0.5 &&
                y < 0 &&
                y > -2.0)
            {
                collisions |= Floor;
            }
        }
        angle += angle_increment;
    }
    return collisions;
}

Flyappy::Vec2D Flyappy::getAcceleration(float vx, float vy)
{
    auto ax = (target_velocity_.x - vx) * 30;
    if (ax < -3) ax = -3;
    if (ax > 3) ax = 3;
    auto ay = (target_velocity_.y - vy) * 30;
    if (ay < -35) ay = -35;
    if (ay > 35) ay = 35;
    return Vec2D(ax, ay);
}
