// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_DISTANCE_around = 0.4;

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
int ping_index_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
bool laser_alarm_=false;
bool obstacle_left_ = false;
int ping_index_i_ = 0;
float ping_dist_i_ = 3.0;
/*int ping_index_left_ = 0;
float ping_dist_left_ = 3.0;
int ping_index_right_ = 0;
float ping_dist_right_ = 3.0;*/
int ping_index_j_ = 0;
float ping_dist_j_ = 1.0;
float former_dist_left_ = 1.0;
float former_dist_right_ = 1.0;
float former_dist_valve_ = 1.0;
int j = 0;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::Publisher obstacle_left_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_ = (int) ((0.0 -angle_min_)/angle_increment_);
        ROS_INFO("LIDAR setup: ping_index = %d",ping_index_);
        
    }

   for(ping_index_i_=ping_index_-40;ping_index_i_<ping_index_+40;ping_index_i_++){
   ping_dist_i_ = laser_scan.ranges[ping_index_i_];
   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   if (ping_dist_i_<MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
       if(ping_dist_in_front_>=MIN_SAFE_DISTANCE){
          if(ping_index_i_>ping_index_){
          obstacle_left_=true;
          }
          else obstacle_left_=false;
          }
       else{
          ping_index_j_ = ping_index_i_;
          former_dist_left_ = ping_dist_i_;
          former_dist_right_ = ping_dist_i_;
          for(j=1;j<300;j++){//loop forever, untill find a gap
          ping_index_j_ = ping_index_i_+j;//check left
          ping_dist_j_ = laser_scan.ranges[ping_index_j_];
          former_dist_valve_ = former_dist_left_+0.3;
          if(ping_dist_j_ > former_dist_valve_){
             if (ping_index_j_>ping_index_){
             obstacle_left_=false;
             }
             else{
             obstacle_left_=true;
             }
          break;
          }
          else{
          former_dist_left_ = ping_dist_j_;
          }

          ping_index_j_ = ping_index_i_-j;//check right
          ping_dist_j_ = laser_scan.ranges[ping_index_j_];
          former_dist_valve_ = former_dist_right_+0.3;
          if(ping_dist_j_ > former_dist_valve_){
             if (ping_index_j_>ping_index_){
             obstacle_left_=false;
             }
             else{
             obstacle_left_=true;
             }
          break;
          }
          else{
          former_dist_right_ = ping_dist_j_;
          }
          }//"for loop" end
          }
       break;
   }
   else {
       laser_alarm_=false;
   }
   }//"for loop" end

   if (laser_alarm_ == false){
   for(ping_index_i_=ping_index_+40;ping_index_i_<ping_index_+125;ping_index_i_++){
   ping_dist_i_ = laser_scan.ranges[ping_index_i_];
   if (ping_dist_i_<MIN_SAFE_DISTANCE_around) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
       obstacle_left_=true;
       break;
   }
   else {
       laser_alarm_=false;
   }
   }
   }

   if (laser_alarm_ == false){
   for(ping_index_i_=ping_index_-125;ping_index_i_<ping_index_-40;ping_index_i_++){
   ping_dist_i_ = laser_scan.ranges[ping_index_i_];
   if (ping_dist_i_<MIN_SAFE_DISTANCE_around) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
       obstacle_left_=false;
       break;
   }
   else {
       laser_alarm_=false;
   }
   }
   }
   

    /*
   ping_dist_in_front_ = laser_scan.ranges[ping_index_];
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }*/
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);

   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg); 

   std_msgs::Bool obstacle_left_msg;
   obstacle_left_msg.data = obstacle_left_;
   obstacle_left_publisher_.publish(obstacle_left_msg); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Publisher pub3 = nh.advertise<std_msgs::Bool>("obstacle_left", 1);  
    obstacle_left_publisher_ = pub3;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

