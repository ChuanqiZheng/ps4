// timer_client: works together with action server called "timer_action"
// in source: example_action_server_w_fdbk.cpp
// this code could be written using classes instead (e.g. like the corresponding server)
//  see: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
//#include<example_action_server/demoAction.h> //reference action message in this package
#include<ps4/ps4Action.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
using namespace std;
bool g_lidar_alarm=false; 

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
/*void doneCb(const actionlib::SimpleClientGoalState& state,
        const ps4::ps4ResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //ROS_INFO("got result output = %d",result->output);
    //g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const ps4::ps4FeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}
*/
void alarmCallback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm) {
     ROS_INFO("LIDAR alarm received!"); 
  }
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "my_action_client_node"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(1.0);
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm",1,alarmCallback);
        ps4::ps4Goal goal; 
        ps4::ps4Goal goal2; 
        double sample_dt = 0.01;
        ros::Rate loop_timer(1/sample_dt);
        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<ps4::ps4Action> action_client("path_action", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        
        /*
           int countdown_goal = 1; //user will specify a timer value
        while(countdown_goal>=0 && ros::ok()) {
           cout<<"enter a desired timer value, in seconds (0 to abort, <0 to quit): ";
           cin>>countdown_goal;
           if (countdown_goal==0) { //see if user wants to cancel current goal
             ROS_INFO("cancelling goal");
             action_client.cancelGoal(); //this is how one can cancel a goal in process
           }
           if (countdown_goal<0) { //option for user to shut down this client
              ROS_INFO("this client is quitting");
              return 0;
           }
           //if here, then we want to send a new timer goal to the action server
           ROS_INFO("sending timer goal= %d seconds to timer action server",countdown_goal);
           goal.input = countdown_goal; //populate a goal message
           //here are some options:
           //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
           */
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    pose.position.x = 3.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    goal.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x=8.0; 
    pose_stamped.pose.position.y=5.2; 
    goal.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x=2.0; 
    pose_stamped.pose.position.y=5.4; 
    goal.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x=0.2; 
    pose_stamped.pose.position.y=7.0; 
    goal.nav_path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x= 0.5; 
    pose_stamped.pose.position.y=11.5; 
    goal.nav_path.poses.push_back(pose_stamped);

    action_client.sendGoal(goal); 

    while(1)
    {
      ros::spinOnce();
      if(g_lidar_alarm)
      {
        pose.position.x = 0.5; // say desired x-coord is 1
        pose.position.y = 11.8;
        pose.position.z = 0.0; // let's hope so!
        pose.orientation.x = 0.0; //always, for motion in horizontal plane
        pose.orientation.y = 0.0; // ditto
        pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
        pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
        pose_stamped.pose = pose;
        goal2.nav_path.poses.push_back(pose_stamped);

        pose_stamped.pose.position.x= 3.5; 
        pose_stamped.pose.position.y= 12.0; 
        goal2.nav_path.poses.push_back(pose_stamped);

        action_client.sendGoal(goal2);
        break;
      }
      loop_timer.sleep();
    }


           //action_client.sendGoal(goal,&doneCb); // send goal and specify a callback function
           //or, send goal and specify callbacks for "done", "active" and "feedback"
           //action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
           
           //this example will loop back to the the prompt for user input.  The main function will be
           // suspended while waiting on user input, but the callbacks will still be alive
           //if user enters a new goal value before the prior request is completed, the prior goal will
           // be aborted and the new goal will be installed
        
       
    return 0;
}

