#include "../include/tf_nav.h"
#include <tf/transform_broadcaster.h>
 
 
using namespace std;
 
Eigen::Vector3d aruco_pose;
Eigen::Vector4d aruco_or;
void poseCallback(Eigen::Vector3d , Eigen::Vector4d );
 
TF_NAV::TF_NAV() {
 
    _position_pub = _nh.advertise<geometry_msgs::PoseStamped>( "/fra2mo/pose", 1 );
    _cur_pos << 0.0, 0.0, 0.0;
    _cur_or << 0.0, 0.0, 0.0, 1.0;
    _goal_pos1 << 0.0, 0.0, 0.0;
    _goal_or1 << 0.0, 0.0, 0.0, 1.0;
    _goal_pos2 << 0.0, 0.0, 0.0;
    _goal_or2 << 0.0, 0.0, 0.0, 1.0;
 
    _goal_pos3 << 0.0, 0.0, 0.0;
    _goal_or3 << 0.0, 0.0, 0.0, 1.0;
 
    _goal_pos4 << 0.0, 0.0, 0.0;
    _goal_or4 << 0.0, 0.0, 0.0, 1.0;
 
    _goal_pos5 << 0.0, 0.0, 0.0;
    _goal_or5 << 0.0, 0.0, 0.0, 1.0;
 
    _goal_pos6 << 0.0, 0.0, 0.0;
    _goal_or6 << 0.0, 0.0, 0.0, 1.0;
 
    _goal_pos7 << 0.0, 0.0, 0.0;
    _goal_or7 << 0.0, 0.0, 0.0, 1.0;
 
    _home_pos << -15, 7.5, 0.0;
}
 
void TF_NAV::tf_listener_fun() {
    ros::Rate r( 5 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
    while ( ros::ok() )
    {
        try {
            listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform( "map", "base_footprint", ros::Time(0), transform );
 
        }
        catch( tf::TransformException &ex ) {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        _cur_pos << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _cur_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        position_pub();
        r.sleep();
    }
 
}
 
void TF_NAV::position_pub() {
 
    geometry_msgs::PoseStamped pose;
 
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
 
    pose.pose.position.x = _cur_pos[0];
    pose.pose.position.y = _cur_pos[1];
    pose.pose.position.z = _cur_pos[2];
 
    pose.pose.orientation.w = _cur_or[0];
    pose.pose.orientation.x = _cur_or[1];
    pose.pose.orientation.y = _cur_or[2];
    pose.pose.orientation.z = _cur_or[3];
 
    _position_pub.publish(pose);
}
 
void TF_NAV::goal1_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal1", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal1", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        _goal_pos1 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or1 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal1 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal2_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal2", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal2", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos2 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or2 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal2 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal3_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal3", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal3", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos3 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or3 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal3 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal4_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal4", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal4", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos4 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or4 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal4 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal5_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal5", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal5", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos5 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or5 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal4 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal6_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal6", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal6", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos6 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or6 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal4 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::goal7_listener() {
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "goal7", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "goal7", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
 
        _goal_pos7 << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        _goal_or7 << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
        // std::cout<< "goal4 "<< _goal_pos << endl;
        r.sleep();
    }    
}
 
void TF_NAV::marker_listener(){
    ros::Rate r( 1 );
    tf::TransformListener listener;
    tf::StampedTransform transform;
 
    while ( ros::ok() )
    {
        try
        {
            listener.waitForTransform( "map", "aruco_marker_frame", ros::Time( 0 ), ros::Duration( 10.0 ) );
            listener.lookupTransform( "map", "aruco_marker_frame", ros::Time( 0 ), transform );
        }
        catch( tf::TransformException &ex )
        {
            ROS_ERROR("%s", ex.what());
            r.sleep();
            continue;
        }
        aruco_pose << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        aruco_or << transform.getRotation().w(),  transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z();
 
        r.sleep();
    }   
 
}         
 
 
void TF_NAV::send_goal() {
    ros::Rate r( 5 );
    int cmd;
    move_base_msgs::MoveBaseGoal goal;
 
    while ( ros::ok() )
    {
        poseCallback(aruco_pose,aruco_or);  

        std::cout<<"\nInsert 1 to send goal from TF "<<std::endl;
        std::cout<<"Insert 2 to send home position goal "<<std::endl;
        std::cout<<"Insert 3 to send marker position goal "<<std::endl;
        std::cout<<"Inser your choice"<<std::endl;
        std::cin>>cmd;
 
        if ( cmd == 1) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            // 3 4 2 5 1 6 7
 
            sender(_goal_pos3,_goal_or3, 3, goal);
            ac.sendGoal(goal);
            ac.waitForResult();
 
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("The mobile robot arrived in the TF goal 3");
                sender(_goal_pos4,_goal_or4, 4, goal);
                ac.sendGoal(goal);
                ac.waitForResult();
 
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The mobile robot arrived in the TF goal 4");
                    sender(_goal_pos2,_goal_or2, 2, goal);
                    ac.sendGoal(goal);
                    ac.waitForResult();
 
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        ROS_INFO("The mobile robot arrived in the TF goal 2");
                        sender(_goal_pos5,_goal_or5, 5, goal);
                        ac.sendGoal(goal);
                        ac.waitForResult();
 
                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            ROS_INFO("The mobile robot arrived in the TF goal 5");
                            sender(_goal_pos1,_goal_or1, 1, goal); 
                            ac.sendGoal(goal);
                            ac.waitForResult();
 
                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                ROS_INFO("The mobile robot arrived in the TF goal 1");
                                sender(_goal_pos6,_goal_or6, 6, goal);
                                ac.sendGoal(goal);
                                ac.waitForResult();
 
                                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                    ROS_INFO("The mobile robot arrived in the TF goal 6");
                                    sender(_goal_pos7,_goal_or7, 7, goal);
                                    ac.sendGoal(goal);
                                    ac.waitForResult();
 
                                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                        ROS_INFO("The mobile robot arrived in the TF goal 7");
                                    }
                                    else ROS_INFO("The base failed to move to 7 for some reason");
                                }
                                else ROS_INFO("The base failed to move to 6 for some reason");
                            }
                            else ROS_INFO("The base failed to move to 1 for some reason");
                        }
                        else ROS_INFO("The base failed to move to 5 for some reason");
                    }
                    else ROS_INFO("The base failed to move to 2 for some reason");
                }
                else ROS_INFO("The base failed to move to 4 for some reason");
            }
            else ROS_INFO("The base failed to move to 3 for some reason");
        }
 
        else if ( cmd == 2 ) {
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = _home_pos[0];
            goal.target_pose.pose.position.y = _home_pos[1];
            goal.target_pose.pose.position.z = _home_pos[2];
 
            goal.target_pose.pose.orientation.w = 1.0;
            goal.target_pose.pose.orientation.x = 0.0;
            goal.target_pose.pose.orientation.y = 0.0;
            goal.target_pose.pose.orientation.z = 0.0;
 
            ROS_INFO("Sending HOME position as goal");
            ac.sendGoal(goal);
 
            ac.waitForResult();
 
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("The mobile robot arrived in the HOME position");
            else
                ROS_INFO("The base failed to move to HOME for some reason");
        }
 
        else if (cmd == 3){
            MoveBaseClient ac("move_base", true);
            while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            }
            boost::thread tf_listener_aruco_t( &TF_NAV::marker_listener, this );
 
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = -15.5;
            goal.target_pose.pose.position.y = 7.5;
            goal.target_pose.pose.orientation.w = 0.0;
            goal.target_pose.pose.orientation.z = 1.0;
 
            ROS_INFO("Sending OBSTACLE 9 goal");
            ac.sendGoal(goal);
 
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
 
                ROS_INFO("The mobile robot arrived in the OBSTACLE 9 position");
                //cout<<"arucopose: " << aruco_pose <<endl;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                //cout<<"arucoor: " << aruco_or <<endl;
                
                goal.target_pose.pose.position.x = aruco_pose[0] + 1.0;
                goal.target_pose.pose.position.y = aruco_pose[1];
 
                ROS_INFO("Sending MARKER goal");
                ac.sendGoal(goal);
 
                ac.waitForResult();
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("The mobile robot arrived in the MARKER position");
                    }
                    else
                        ROS_INFO("The base failed to move to MARKER for some reason");
            }
            else
                ROS_INFO("The base failed to move to OBSTACLE 9 for some reason");
 
        }
 
        else {
            ROS_INFO("Wrong input!");
        }
        r.sleep();
    }
}
 
void TF_NAV::run() {
    boost::thread tf_listener_fun_t( &TF_NAV::tf_listener_fun, this );
    boost::thread tf_listener_goal1_t( &TF_NAV::goal1_listener, this );
    boost::thread tf_listener_goal2_t( &TF_NAV::goal2_listener, this );
    boost::thread tf_listener_goal3_t( &TF_NAV::goal3_listener, this );
    boost::thread tf_listener_goal4_t( &TF_NAV::goal4_listener, this );
    boost::thread tf_listener_goal5_t( &TF_NAV::goal5_listener, this );
    boost::thread tf_listener_goal6_t( &TF_NAV::goal6_listener, this );
    boost::thread tf_listener_goal7_t( &TF_NAV::goal7_listener, this );
  
    //std::cout<< "goal1 "<< _goal_pos1 << endl;
    //std::cout<< "goal2 "<< _goal_pos2 << endl;
    //std::cout<< "goal3 "<< _goal_pos3 << endl;
    //std::cout<< "goal4 "<< _goal_pos4 << endl;
    boost::thread send_goal_t( &TF_NAV::send_goal, this );
 
    // boost::thread tf_listener_goal2_t( &TF_NAV::goal2_listener, this );
    // std::cout<< "goal2 "<< _goal_pos << endl;
    // boost::thread send_goal2_t( &TF_NAV::send_goal, this );
 
    // boost::thread tf_listener_goal3_t( &TF_NAV::goal3_listener, this );
    // std::cout<< "goal3 "<< _goal_pos << endl;
    // boost::thread send_goal3_t( &TF_NAV::send_goal, this );
 
    // boost::thread tf_listener_goal4_t( &TF_NAV::goal4_listener, this );
    // std::cout<< "goal4 "<< _goal_pos << endl;
    // boost::thread send_goal4_t( &TF_NAV::send_goal, this );
 
    ros::spin();
}
 
void TF_NAV::sender(Eigen::Vector3d _pos, Eigen::Vector4d _or, int num, move_base_msgs::MoveBaseGoal &goal){
 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = _pos[0];
    goal.target_pose.pose.position.y = _pos[1];
    goal.target_pose.pose.position.z = _pos[2];
 
    goal.target_pose.pose.orientation.w = _or[0];
    goal.target_pose.pose.orientation.x = _or[1];
    goal.target_pose.pose.orientation.y = _or[2];
    goal.target_pose.pose.orientation.z = _or[3];
 
    ROS_INFO("Sending goal: %d",num);
}
 
 
void poseCallback(Eigen::Vector3d aruco_pose, Eigen::Vector4d aruco_or){
 
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(aruco_pose[0], aruco_pose[1], aruco_pose[2]));
 
   tf::Quaternion q(aruco_or[1], aruco_or[2], aruco_or[3], aruco_or[0]);

   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "aruco_pose_tf"));
}
 
 
int main( int argc, char** argv ) {
    ros::init(argc,argv,"tf_navigation");
    TF_NAV tfnav;
    tfnav.run();
    return 0;
}