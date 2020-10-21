
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include "autoware_msgs/DetectedObject.h"
#include <autoware_msgs/DetectedObjectArray.h>
#include "zr_msgs/object_info.h"
#include "autoware_msgs/PointsImage.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <deque>
float X;
float Y;                                               //X and Y are the coordinates of the cloest person to the Camera.

void follow_vel_control(float &dist_person_min)
{
    float follow_range = 1.5;
    float vel_diff_turn = 0.1; // 定义左右转速度的改变量的大小， 大小还需要调整
    //  float vel_diff_turn = 0.15;
    //     float theta_ = (minIndex/2 + 67.5) * 3.14159265 / 180;
    //fix the vel in follow mode
    float robot_radius = 0.3;
    float vel_L = 0.8;
    float vel_R = 0.8;
   // float y = Y;
    float x = X;
    //     ROS_INFO("Index: %f    ; Min: %f", minIndex, rangeMin);
    //     ROS_INFO("X: %f    ; Y: %f", x, y);

    //case go straight
    if (fabs(x) <= robot_radius + 0.1) //在竖直方向判断
    {
        ROS_INFO("now the robot and the leader are in the same line");
        if (dist_person_min > (follow_range + 0.2))
        {
            ROS_INFO("the leader is far with robot, the robot will speed up to make the distance smaller!!!");
            vel_L = vel_L * (1.0 - follow_range / dist_person_min); //这里加一个反馈控制，距离越远，车的速度越大
            vel_R = vel_R * (1.0 - follow_range / dist_person_min);
        }
        if (dist_person_min <= (follow_range + 0.2) && dist_person_min >= (follow_range - 0.2)) //定义跟随范围为1.3---1.7
        {
            ROS_INFO("the distance betwwen the robot and leader is in good value");
            vel_L = 0;
            vel_R = 0;
        }
        if (dist_person_min < (follow_range - 0.2))
        {
            ROS_INFO("the distance now is too small, please be careful!!!");
            vel_R = 0;
            vel_L = 0; //这里是否考虑刹车？如果小于跟随范围，小车还有速度的话，是否考虑刹车 brake_flag
        }
    }
    if (x < (-robot_radius - 0.1)) //判断人是否左偏， turn left
    {
        vel_L = 0.5;
        vel_R = 0.5;
        ROS_INFO("now the leader is on the robot's left");
        float diff_L = fabs(x) - fabs(-robot_radius - 0.1); //定义人的左偏程度
        vel_L -= vel_diff_turn * (diff_L / fabs(x));        //这里diff/fabs(x)为左偏程度， 人左偏的越厉害，轮子速度改变量越大，角速度也越大
        vel_R += vel_diff_turn * (diff_L / fabs(x));
        if (x > robot_radius + 0.1) //判断人是否偏右， 偏右则右转
        {
            vel_L = 0.5;
            vel_R = 0.5;
            ROS_INFO("now the leader is on the robot's right");
            float diff_R = x - (robot_radius + 0.1);
            vel_L -= vel_diff_turn * (diff_R / fabs(x)); //这里diff/fabs(x)为右偏程度， 人右偏的越厉害，轮子速度改变量越大，角速度也越大
            vel_R += vel_diff_turn * (diff_R / fabs(x));
        }

    }
    std::cout << "vel_L is " << vel_L << std::endl;
    std::cout << "vel_R is " << vel_R << std::endl;
}

void Person_select_follow(std::vector<float> &distance_raw, std::vector<std::string> &label_raw, std::vector<float> &x_raw, std::vector<float> &y_raw)
{
    int n = distance_raw.size();
    std::vector<std::string> label; // label of person
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> distance;
//    std::cout << "n = :::"<< n <<std::endl;
//     std::cout << "444 = :::"<< distance_raw[n-1] <<std::endl;
        for (int i = 0; i < n; i++)
        {
            if (label_raw[i] == "person")
            {
                distance.push_back(distance_raw[i]);
                label.push_back(label_raw[i]);
                x.push_back(x_raw[i]);
                y.push_back(y_raw[i]);
            }           
        }
        if (!distance.size())
        {
            ROS_INFO("there is no leader before the robot, please make sure there is one leader");
            return;
        }
        else
        {
            int num = distance.size();
            int num1 = label.size();
            int num2 = x.size();
            if (!(num = num1 = num2))
            {
                ROS_INFO("there is something wrong");
                return;
            }
            else
            {
                float dist_person_min = 999;
                float min_index = -1;
                for (int i = 0; i < num; i++)
                {
                    if (dist_person_min > distance[i])
                    {
                        dist_person_min = distance[i];
                        min_index = i;
                    }
                }
                std::cout << "distance = " << dist_person_min << std::endl;
                std::cout << "55555===" << distance[num-1]<<std::endl;
                X = x[min_index];
                Y = y[min_index];
                std::cout << "x in the space is  " << X << std::endl;
                std::cout << "y in the space is  " << Y << std::endl;
                follow_vel_control(dist_person_min);
            }
        }
}
//by jie get the disdtance of all labels
static void DistanceCallback(const zr_msgs::object_info &result)
{
    std::vector<float> distance_raw;    // all objects (people, car ...)
    std::vector<std::string> label_raw; //all labels
    std::vector<float> x_raw;
    std::vector<float> y_raw;
    if (result.distance.empty())
    {
        ROS_ERROR("this frame did not detect anything, it will enter into next frame!!!!!");
        return;
    }
    //ROS_INFO("size of distance", result.distance.size());
    // std::cout << "222222====" << result.distance.size() << std::endl;
    // std::cout << "333333====" << result.label.size() << std::endl;
    distance_raw = result.distance;
    //label.push_back(result->label);
    label_raw = result.label;
    x_raw = result.x;
    y_raw = result.y;
    //std::cout<<"size=   "<<distance_raw.size()<<std::endl;
    std::cout << label_raw[0] << std::endl;
    Person_select_follow(distance_raw, label_raw, x_raw, y_raw);
    //     bounding_box_raw.clear();
}

int main(int argc, char *argv[])
{

    // Write to console that we are starting trajectory generation
    //#ifdef Z_DEBUG
    //	ROS_INFO_STREAM("Trajectory Generation Begins: ");
    //#endif
    ros::init(argc, argv, "follower_node_1");

    // Create node handles
    ros::NodeHandle nh;
    //ros::NodeHandle private_nh("~");

    // Publish the following topics:
    //    g_vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 1);
    //    g_vis_clo_pub = nh.advertise<visualization_msgs::Marker>("closest_waypoint_mark", 1);
    // Publish the curvature information:
    // ros::Publisher vel_L_R_pub = nh.advertise<geometry_msgs::TwistStamped>("/twist_raw", 10);
    //    ros::Publisher state_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("lattice_state", 10);

    // Subscribe to the following topics:
    ros::Subscriber distance_subscriber = nh.subscribe("/obj_detect_distance", 1, DistanceCallback);
    //  ros::Subscriber  label_coord_subscriber = nh.subscribe("/detection/image_detector/objects",1,CoordCallback);
    //    ros::Subscriber  intrinsic = nh.subscribe("/camera_info", 1, intrinsic_callback);
     ros::Rate loop_rate(25);
    //Person_sort();
    //   pseudo_follow();

     while (ros::ok())
      {

     ros::spinOnce();
      loop_rate.sleep();

      }
    //ros::spin();
    return 0;
}
