#pragma once

#include <vector>
#include <deque>

class Flyappy
{
  public:

    Flyappy();

    enum Collision {
      None = 0x00,
      Front = 0x01,
      Top = 0x02, 
      Bottom = 0x04,
      Floor = 0x08
    };

    struct Vec2D
    {
      Vec2D(float vx, float vy);
      void operator=(const Vec2D& v);
      bool operator==(const Vec2D& v);
      float x;
      float y;
    };

    void magnetDrive(
      const std::vector<float>& ranges, float angle_min, float angle_increment);

    void collisionAvoidance(
      const std::vector<float>& ranges, float angle_min, float angle_increment);

    uint8_t collisionDetection(
      const std::vector<float>& ranges, float angle_min, float angle_increment);

    Vec2D getAcceleration(float vx, float vy);

    void reset();

  private:
      
    const float clearance_radius_;
    Vec2D target_velocity_;
    const Vec2D forward_;
    const Vec2D up_;
    const Vec2D down_;
    int counter_;
    bool collision_avoidance_;
    bool found_exit_;
    bool passing_wall_;
};
