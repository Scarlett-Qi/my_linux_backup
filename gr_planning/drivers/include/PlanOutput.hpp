#pragma once

class PlanOutput {
    public:
        PlanOutput() {}

        PlanOutput(const PlanOutput &other) : 
            x_(other.x_), y_(other.y_), angle_(other.angle_) {}

        ~PlanOutput()=default;

        // 位置
        double x_, y_;

        // 角度
        double angle_;

        void setPosition(const double &x, const double &y) {
            x_ = x;
            y_ = y;
        }

        void setAngle(const float &angle) {
            angle_ = angle;
        }
};