#include "line_control.h"
#include "math.h"
#include <std_msgs/Float64.h>

void LineControl::laserCallback(const sensor_msgs::LaserScan& msg)
{

  // проверим нет ли вблизи робота препятствия
  const double kMinObstacleDistance = 0.3;
  for (size_t i = 0; i<msg.ranges.size(); i++)
  {
      if ( msg.ranges[i] < kMinObstacleDistance )
      {
          obstacle = true;
          ROS_WARN_STREAM("OBSTACLE!!!");
          break;
      }
  }
}

void LineControl::poseCallback(const nav_msgs::Odometry& msg)
{
  ROS_DEBUG_STREAM("Pose msg: x = "<<msg.pose.pose.position.x<<
          " y = "<<msg.pose.pose.position.y<<
          " theta = "<<2*atan2(msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w) );
  // обновляем переменные класса, отвечающие за положение робота
  x = msg.pose.pose.position.x;
  y = msg.pose.pose.position.y;
  theta = 2*atan2(msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w);
}


double LineControl::cross_track_err_line()
{
    return line_y - y;
}



double LineControl::cross_track_err_circle()
{
    double dx = cx - x;
    double dy = cy - y;
    double e = sqrt(dx*dx + dy*dy) - R;
    return  e;
}

double LineControl::cross_track_err_ellipse()
{
    double error;
    if (x <= cleft_x){
        error = sqrt(pow((cleft_x - x),2) + pow(cleft_y - y,2)) - R;
    } else if ( x >= cright_x){
        error = sqrt(pow((cright_x - x),2) + pow(cright_y - y,2)) - R;
    } else if (y > 0){
        error = up_line_y - y;
    } else {
        error = down_line_y - y;
    }
    return error;
}

void LineControl::publish_error(double e)
{
    std_msgs::Float64 err;
    err.data = e;
    err_pub.publish(err);
}

void LineControl::timerCallback(const ros::TimerEvent&)
{
    ROS_DEBUG_STREAM("on timer ");
    // сообщение с помощью которого задается
    // управление угловой и линейной скоростью
    geometry_msgs::Twist cmd;
    // при создании структура сообщения заполнена нулевыми значениями

    // если вблизи нет препятствия то задаем команды
    if ( !obstacle )
    {
        //  вычислим текущую ошибку управления
        double err = cross_track_err_ellipse();
        //  публикация текущей ошибки
        publish_error(err);
        //  интегрируем ошибку
        int_error += err;
        //  диффференцируем ошибку
        double diff_error = err - old_error;
        //   запоминаем значение ошибки для следующего момента времени
        old_error = err;
        cmd.linear.x = task_vel;
        //  ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
        if (y > 0 && x >= cleft_x && x <= cright_x){
            cmd.angular.z = -(prop_factor * err + int_factor*int_error + diff_error * diff_factor);
        } else {
            cmd.angular.z = prop_factor * err + int_factor*int_error + diff_error * diff_factor;
        }
        ROS_DEBUG_STREAM("error = "<<err<<" cmd v="<<cmd.linear.x<<" w = "<<cmd.angular.z);
    }
    //  отправляем (публикуем) команду
    cmd_pub.publish(cmd);
}

LineControl::LineControl():
    int_error(0.0),
    old_error(0.0),
    obstacle(false),
    node("~")
{
    ROS_INFO_STREAM("LineControl initialisation");
    //  читаем параметры
    line_y = node.param("line_y", -10.0);
    cx = node.param("cx", -6);
    cy = node.param("cy", 0);
    R = node.param("R", 6);
    task_vel = node.param("task_vel", 1.0);
    prop_factor = node.param("prop_factor", 0.1);
    int_factor = node.param("int_factor", 0.0);
    diff_factor = node.param("diff_factor", 0.0);
    min_obstacle_range = node.param("min_obstacle_range", 1.0);
    double dt = node.param("dt", 0.1);
    cleft_x = -6;
    cleft_y = cright_y = 0;
    cright_x = 6;
    up_line_y = 6;
    down_line_y = -6;

    //  подписываемся на необъодимые данные
    laser_sub = node.subscribe("/scan", 100, &LineControl::laserCallback, this);
    pose_sub = node.subscribe("/base_pose_ground_truth", 100, &LineControl::poseCallback, this);
    timer1 = node.createTimer(ros::Duration(dt), &LineControl::timerCallback, this);
    cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    err_pub = node.advertise<std_msgs::Float64> ("/err", 100);


}
