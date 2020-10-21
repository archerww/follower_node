
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
#include <std_msgs/Int32.h>

ros::Publisher vel_L_R_pub;
ros::Publisher change_follow_flag;
static float vel_L = 0.5;
static float vel_R = 0.5; // 定义全局左右轮速度，直线行驶时使用
static float vel_turn_L = 0.3;
static float vel_turn_R = 0.3; // 定义转向时左右轮的速度
static float robot_radius = 0.3;
static float vel_diff_turn = 0.03; // 定义左右转速度的改变量的大小， 大小还需要调整
static float vel_diff_line = 0.05; //定义直行时左右轮速度改变量的大小
static float thre_off = 0.0;       //判断左右转的偏移量
float linear;
float angle;
static float thre_dist = 4.0; //定义阈值（跟踪范围）
float X;
float Y;                       //X and Y are the coordinates of the cloest person to the Camera.
float index_bear;              //用于存放检测出特殊标签的指定索引
std::deque<float> data_leader; //用于存放上一时刻leader的空间数据信息
std_msgs::Int32 follow_flag;   //1 代表跟随， 0 代表不跟随

std_msgs::Int32 change_flag;
void follow_vel_control(float &dist_person_min)
{
    float follow_range = 1.5;
    //   float vel_L = 0.3;
    //  float vel_R = 0.3;
    float y = Y;
    float x = X;

    geometry_msgs::Twist twist_follow;
    if (dist_person_min >= thre_dist) // thre_dist = 4
    {
        vel_L = 0.0;
        vel_R = 0.0;
    }
    else
    {
        //case go straight
        if (fabs(x) <= robot_radius + thre_off) //在竖直方向判断
        {
            ROS_INFO("now the robot and the leader are in the same line and the distance is in the good value!");
            if (dist_person_min > (follow_range + 0.2) && dist_person_min < thre_dist)
            {
                ROS_INFO("the leader is far with robot, the robot will speed up to make the distance smaller!!!");
                //vel_L = vel_L * (dist_person_min / follow_range) * 1.1; //这里加一个反馈控制，距离越远，车的速度越大
                // vel_R = vel_R * (dist_person_min / follow_range) * 1.1;
                vel_L += vel_diff_line * (dist_person_min / (follow_range + 0.2));
                vel_R += vel_diff_line * (dist_person_min / (follow_range + 0.2));
            }
            else
            {
                if (dist_person_min <= (follow_range + 0.2) && dist_person_min >= (follow_range - 0.2)) //定义跟随范围为1.3---1.7
                {
                    ROS_INFO("the distance betwwen the robot and leader is in good value");
                    vel_L = 0.5;
                    vel_R = 0.5; //合理范围内匀速行驶
                }
                else //situation: (dist_person_min < (follow_range - 0.2))
                {
                    if (dist_person_min < (follow_range - 0.2) && dist_person_min > 0.9)
                    {
                        ROS_INFO("the distance now is too small, please be careful!!!");
                        //  vel_R = vel_R * (dist_person_min / (follow_range + 0.5));
                        // vel_L = vel_L * (dist_person_min / (follow_range + 0.5)); //避障的优先级会高于跟随算法，若距离太近，会启动避障。
                        vel_L -= vel_diff_line * ((follow_range - 0.2) / dist_person_min);
                        vel_R -= vel_diff_line * ((follow_range - 0.2) / dist_person_min);
                    }
                    else //situation : dist_person_min <= 0.9
                    {
                        vel_L = 0;
                        vel_R = 0;
                    }
                }
            }
            std::cout << "vel_L" << vel_L << std::endl;
            std::cout << "vel_R" << vel_R << std::endl;
            linear = (vel_L + vel_R) / 2;
            angle = (vel_R - vel_L) / 0.54;
            twist_follow.linear.x = linear;
            twist_follow.angular.z = angle;
            vel_L_R_pub.publish(twist_follow);
            vel_turn_L = 0.3;
            vel_turn_R = 0.3;
        }
        else
        {
            if (x < (-robot_radius - thre_off)) //判断人是否左偏， turn left
            {
                ROS_INFO("now the leader is on the robot's left, the robot will turn left!");
                vel_turn_L -= vel_diff_turn * (fabs(x) / robot_radius); //这里diff/fabs(x)为左偏程度， 人左偏的越厉害，轮子速度改变量越大，角速度也越大
                vel_turn_R += vel_diff_turn * (fabs(x) / robot_radius); //当机器人左右转向速度过快或过慢时，可调节vel_diff_turn的值的大小
                if (!(vel_L > 0))
                {
                    vel_L = vel_R / 2;
                }
            }
            else //(x > robot_radius + 0.05) //判断人是否偏右， 偏右则右转
            {
                ROS_INFO("now the leader is on the robot's right");
                float diff = fabs(robot_radius + thre_off);
                vel_turn_L += vel_diff_turn * (fabs(x) / robot_radius);
                vel_turn_R -= vel_diff_turn * (fabs(x) / robot_radius);
                //这里diff/fabs(x)为右偏程度， 人右偏的越厉害，轮子速度改变量越大，角速度也越大

                if (!(vel_R > 0))
                {
                    vel_R = vel_L / 2;
                }
            }
        }

        linear = (vel_turn_L + vel_turn_R) / 2;
        angle = (vel_turn_R - vel_turn_L) / 0.54;
        twist_follow.linear.x = linear;
        twist_follow.angular.z = angle;
        vel_L_R_pub.publish(twist_follow);
    }
}
//如何设计让bool只判断第一帧或前几帧，而不是一直处于循环中，可以设置flag，例如前几帧flag为0， 之后flag为1
bool label_choose(std::vector<std::string> &label_raw) //bool 函数来判断传入的label里面是否含有指定标签
{
    int m = label_raw.size();
    //可以直接把j存下来，放在person_select_follow里面调用x y distance数据。
    //第一帧检测到bear时发送topic，播放语音“已连接”
    for (int j = 0; j < m; j++)
    {
        if (label_raw[j] == "bear")
        {
            index_bear = j;
            return true;
        }
    }
    return false;
}
void Person_select_follow(std::vector<float> &distance_raw, std::vector<std::string> &label_raw, std::vector<float> &x_raw, std::vector<float> &y_raw)
{
    int n = distance_raw.size();
    std::vector<std::string> label; // label of person
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> distance;
    // 此段代码为 当leader的背上带有另一个明显的标签时，筛选出标签，达到选中行人的目的
    //   若某些帧未检测出此标签 或者leader转向时由于角度原因无法正确探测背部label时，则进入最近距离筛选阶段
    if (label_choose(label_raw))
    {
        int j;
        j = index_bear;
        //等待label_choose的j数据by姜
        float distance_selected = distance_raw[j];
        X = x_raw[j];
        Y = y_raw[j]; //取出背部标签对应的空间坐标数据，其中的distance即为leader的距离
                      // std::vector<float> distance;
        data_leader.push_back(X);
        data_leader.push_back(Y);
        data_leader.push_back(distance_selected);
        while (data_leader.size() > 3)
        {
            data_leader.pop_front();
        }
        //while的作用用于总是保留上一帧的元素，而不保留其他帧元素
        if (distance_selected > 3.0)
        {
            return;
        }
        follow_vel_control(distance_selected);
    }
    else
    {
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
        if (distance.empty())
        {
            ROS_INFO("There is no leader before the robot, please make sure there is one leader");
            return;
        }
        else
        {
            ROS_INFO("now begin to find the cloest person!!");
            float dist_person_min = 99;
            float min_index = -1;
            for (int i = 0; i < num; i++)
            {
                if (dist_person_min > distance[i])
                {
                    dist_person_min = distance[i];
                    min_index = i;
                }
                else
                {
                    continue;
                }
            }
            // std::cout << "distance = " << dist_person_min << std::endl;
            X = x[min_index];
            Y = y[min_index];
            if (dist_person_min > thre_dist)
            {
                return; //可能由于某一帧距离筛选错误，所以返回下一帧继续筛选
            }
            if (data_leader.empty()) //对于第一帧来考虑， 之后的每一帧与此无关
            {
                data_leader.push_back(X);
                data_leader.push_back(Y);
                data_leader.push_back(dist_person_min);
            }
            float thre1 = fabs(dist_person_min - data_leader[2]); //data_leader为上一帧leade的空间数据，如果未检测出背上标签
            //而是选出离机器人最近距离的行人，则需要比较当前帧与上一帧的数据差异，若差异小于阈值，则认为选择正确，若大于阈值，则再次进行筛选
            float thre2 = fabs(X - data_leader[0]);
            //  float thre3 = fabd(Y - data_leader[1]);
            if (thre1 <= 0.3 && thre2 <= 0.2)
            {
                ROS_INFO("Now we select the leader correctly!!!");
                data_leader.push_back(X);
                data_leader.push_back(Y);
                data_leader.push_back(dist_person_min); //若特殊标签某一帧或某几帧未检测成功，但通过行人检测出的与上一帧差别不大的话
                //则选中此为leader的空间信息
                while (data_leader.size() > 3)
                {
                    data_leader.pop_front();
                }
                std::cout << "situation 1 : x in the space is : " << X << std::endl;
                std::cout << "situation 1 : y in the space is : " << Y << std::endl;
                std::cout << "situation 1 : distance in the space is : " << dist_person_min << std::endl;
            }
            else
            {
                ROS_INFO("now we didn't select the leader correctly, so we will choose the real leader!!! ");
                for (int i = 0; i < num; i++)
                {
                    if (fabs(data_leader[2] - distance[i]) <= 0.3 && fabs(data_leader[0] - x[i]) <= 0.2)
                    {
                        dist_person_min = distance[i];
                        X = x[i];
                        Y = y[i];
                        data_leader.push_back(X);
                        data_leader.push_back(Y);
                        data_leader.push_back(dist_person_min);
                        break;
                    }
                }
                while (data_leader.size() > 3)
                {
                    data_leader.pop_front();
                }
                std::cout << "situation 2 : x in the space is : " << X << std::endl;
                std::cout << "situation 2 : y in the space is : " << Y << std::endl;
                std::cout << "situation 2 : distance in the space is : " << dist_person_min << std::endl;
            }
            follow_vel_control(dist_person_min);
        }
        // follow_vel_control(dist_person_min);
    }
    // follow_vel_control(data_leader[2]);
}
//by jie get the disdtance of all labels
static void StartFlagCallback(const std_msgs::Int32 &msg)
{
    follow_flag.data = msg.data;
}
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
    distance_raw = result.distance;
    label_raw = result.label;
    x_raw = result.x;
    y_raw = result.y;
    if (follow_flag.data == 0)
    {
        if (label_choose(label_raw))
        {
            ROS_INFO("now we have detected the special label, you can set flag to start following!!!"); //发送语音,  这里需要考虑通过什么样的方式可以更改flag的值
            change_flag.data = 3;
            change_follow_flag.publish(change_flag.data);
        }
        else
        {
            ROS_INFO("now we didn't detected any special label, please make sure you have the correct posture!!! you can also set the flag to 1");
            //这里如果没有检测到特殊标签，执行第二种方案，即第一帧时以选中距离最近的人为依据
            change_flag.data = 4;
            change_follow_flag.publish(change_flag.data);
        }
        return;
    }
    else
    {
        ROS_INFO("now we can start following!!!!");
        Person_select_follow(distance_raw, label_raw, x_raw, y_raw);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "follower_node_1");
    // Create node handles
    ros::NodeHandle nh;

    follow_flag.data = 1;

    vel_L_R_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    change_follow_flag = nh.advertise<std_msgs::Int32>("change_follow_flag", 10);
    // Subscribe to the following topics:
    ros::Subscriber distance_subscriber = nh.subscribe("/obj_detect_distance", 1, DistanceCallback);
    ros::Subscriber follow_flag_subscribe = nh.subscribe("start_flag", 1, StartFlagCallback);
    ros::Rate loop_rate(25);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
