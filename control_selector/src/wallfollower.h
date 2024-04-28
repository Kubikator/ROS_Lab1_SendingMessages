#pragma once

#include<ros/ros.h>
#include"control.h"

class Wallfollower : public Control{

    double min_range;
    bool obstacle = false;
    double max_vel;
    double max_omega;
    double error, old_error, int_error, diff_error;
    double target_range = 2.;

    double calculate_error();

    public: 
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override;

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Wallfollower"; }

    Wallfollower(double range = 1.0, double maxv = 0.5, double maxw = 0.5);
};