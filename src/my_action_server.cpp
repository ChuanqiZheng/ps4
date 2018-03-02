// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <example_action_server/demoAction.h>
#include <ps4/ps4Action.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <std_msgs/Bool.h> 
using namespace std;

bool g_goal_status=true;
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose;

double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
int g_try = 0;
void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);


class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"

    //actionlib::SimpleActionServer<example_action_server::demoAction> as_;
    actionlib::SimpleActionServer<ps4::ps4Action> as_;

    // here are some message types to communicate with our client(s)

    //example_action_server::demoGoal goal_; // goal message, received from client
    //example_action_server::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    //example_action_server::demoFeedback feedback_; // for feedback 
    ps4::ps4Goal goal_; // goal message, received from client
    ps4::ps4Result result_; // put results here, to be sent back to the client when done w/ goal
    ps4::ps4Feedback feedback_; // for feedback

    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<ps4::ps4Action>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "path_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "path_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}


//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
        /*if(g_try = 0){
        if(as_.isPreemptRequested())
          {
            ROS_WARN("goal cancelled!");
            g_goal_status = false;
            break;
          }}*/
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    g_twist_cmd.linear.x=0.0;
    g_twist_commander.publish(g_twist_cmd);
    loop_timer.sleep(); 
    //do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          /*if(g_try = 0){
          if(as_.isPreemptRequested())
          {
            ROS_WARN("goal cancelled!");
            g_goal_status = false;
            break;
          }}
          */
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    g_twist_cmd.linear.x=0.0;
    g_twist_commander.publish(g_twist_cmd);
    loop_timer.sleep(); 
    //do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 
 float dx = goal_pose.position.x - current_pose.position.x;
 float dy = goal_pose.position.y - current_pose.position.y;
 dist = sqrt(dx*dx + dy*dy);
 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    heading = atan2(dy,dx);
 }
}

void do_inits(ros::NodeHandle &nh_) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = nh_.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);    
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<ps4::ps4Action>::GoalConstPtr& goal) {
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = goal->nav_path.poses.size();
    ROS_INFO("received path request with %d poses",npts);    
    yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation);
    g_goal_status = true;
    for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = goal->nav_path.poses[i].pose; //get next pose from vector of poses
        
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        //ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired, pose_desired.position.x,pose_desired.position.y); 
        //ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        //ROS_INFO("travel distance = %f",travel_distance);         
        
        
        // a quaternion is overkill for navigation in a plane; really only need a heading angle
        // this yaw is measured CCW from x-axis
        // GET RID OF NEXT LINE AFTER FIXING get_yaw_and_dist()
        //yaw_desired = convertPlanarQuat2Phi(pose_desired.orientation); //from i'th desired pose
        
        //ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        //yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        //do_spin(spin_angle); // carry out this incremental action
    //ROS_INFO("spin_angle is %lf",spin_angle);

        ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_angle)/g_spin_speed;
    //ROS_INFO("final_time for spin is %lf",final_time);
    g_twist_cmd.angular.z= sgn(spin_angle)*g_spin_speed;
    while(timer<final_time) {
       /* if(g_try!=1){
        if(as_.isPreemptRequested())
          {
            ROS_WARN("goal cancelled!");
            g_goal_status = false;
            g_try = 1;
            break;
          }
          }*/
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    g_twist_cmd.linear.x=0.0;
    g_twist_commander.publish(g_twist_cmd);
    loop_timer.sleep(); 

        g_current_pose.orientation = pose_desired.orientation;
        yaw_current = yaw_desired;
        // we will just assume that this action was successful--really should have sensor feedback here
        if(!g_goal_status)
        break;
         // assumes got to desired orientation precisely
        
        
        //FIX THE NEXT LINE, BASED ON get_yaw_and_dist()
        //do_move(1.0);  // move forward 1m...just for illustration; SHOULD compute this from subgoal pose
        //do_move(travel_distance);

    timer=0.0;
    final_time = fabs(travel_distance)/g_move_speed;
    //ROS_INFO("travel distance is %lf",travel_distance);
    //ROS_INFO("final_time for move is %lf",final_time);
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(travel_distance)*g_move_speed;
    while(timer<final_time) {
          if(g_try!=1){
          if(as_.isPreemptRequested())
          {
            ROS_WARN("goal cancelled!");
            g_goal_status = false;
            g_try = 1;
            break;
          }
          }
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    g_twist_cmd.linear.x=0.0;
    g_twist_commander.publish(g_twist_cmd);
    loop_timer.sleep();

        g_current_pose.position = pose_desired.position;
        if(!g_goal_status)
        break;
        }

     geometry_msgs::PoseStamped pose_stamped;
     pose_stamped.pose.position = g_current_pose.position;
     //result_.nav_path.poses.pose.position = g_current_pose.position;
     result_.nav_path.poses.push_back(pose_stamped);
     as_.setSucceeded(result_);
/*
    ROS_INFO("in executeCB");
    ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    ros::Rate timer(1.0); // 1Hz timer
    countdown_val_ = goal->input;
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    while (countdown_val_>0) {
       ROS_INFO("countdown = %d",countdown_val_);
       
       // each iteration, check if cancellation has been ordered
       if (as_.isPreemptRequested()){	
          ROS_WARN("goal cancelled!");
          //result_.output = countdown_val_;
          //as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 		}
	
 	   //if here, then goal is still valid; provide some feedback
 	   feedback_.fdbk = countdown_val_; // populate feedback message with current countdown value
 	   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
       countdown_val_--; //decrement the timer countdown
       timer.sleep(); //wait 1 sec between loop iterations of this timer
    }
    //if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = countdown_val_; //value should be zero, if completed countdown
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
*/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_action_server_node"); // name this node 
    ros::NodeHandle nh_;
    ROS_INFO("instantiating the my_action_server: ");

    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    do_inits(nh_);

    ROS_INFO("Ready to accept paths.");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();
    
    return 0;
}

