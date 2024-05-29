/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-06-26
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include <rclcpp/rclcpp.hpp>

namespace sim_robot {
/*
 * Bicycle kinematics model (rear wheel):
 *  dot_x = v(t) * cos(theta(t))
 *  dot_y = v(t) * sin(theta(t))
 *  dot_theta = v(t)/L * tan(delta(t))
 * State: (x, y, theta)
 * Control input: (v, delta) - velocity, steering angle of front wheel
 */
class BicycleKinematics
{
public:
    struct CtrlInput
    {
        CtrlInput(double vel = 0.0, double ster = 0.0)
            : v(vel)
            , delta(ster)
        {}

        double v;
        double delta;
    };

    struct State
    {
        /* data */
        State(double x, double y, double phi)
            : x_(x)
            , y_(y)
            , phi_(phi)
        {}
        State  operator+(const State& rhs) const { return State(this->x_ + rhs.x_, this->y_ + rhs.y_, this->phi_ + rhs.phi_); }
        double x_;
        double y_;
        double phi_;
    };

    BicycleKinematics(double wheel_base, rclcpp::Duration min_sim_time);

    void calKinematics(State& current_state, const CtrlInput& u, const rclcpp::Duration sim_time);

private:
    double           wheel_base_;
    rclcpp::Duration min_sim_time_;
};
}   // namespace sim_robot

#endif /* BICYCLE_MODEL_HPP */
