#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/thread.hpp"
#include "Eigen/Dense"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class TF_NAV {

    public:
        TF_NAV();
        void run();
        void tf_listener_fun();
        void position_pub();
        void goal1_listener();
        void goal2_listener();
        void goal3_listener();
        void goal4_listener();

        void goal5_listener();
        void goal6_listener();
        void goal7_listener();

        void marker_listener();         
        void send_goal();
        void sender (Eigen::Vector3d, Eigen::Vector4d, int, move_base_msgs::MoveBaseGoal &);

    private:

        ros::NodeHandle _nh;

        ros::Publisher _position_pub;

        Eigen::Vector3d _home_pos;

        Eigen::Vector3d _cur_pos;
        Eigen::Vector4d _cur_or;
        
        Eigen::Vector3d _goal_pos1;
        Eigen::Vector4d _goal_or1;

        Eigen::Vector3d _goal_pos2;
        Eigen::Vector4d _goal_or2;

        Eigen::Vector3d _goal_pos3;
        Eigen::Vector4d _goal_or3;

        Eigen::Vector3d _goal_pos4;
        Eigen::Vector4d _goal_or4;

        Eigen::Vector3d _goal_pos5;
        Eigen::Vector4d _goal_or5;

        Eigen::Vector3d _goal_pos6;
        Eigen::Vector4d _goal_or6;

        Eigen::Vector3d _goal_pos7;
        Eigen::Vector4d _goal_or7;        

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


};