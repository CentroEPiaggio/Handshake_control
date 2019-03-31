#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>

#include <sstream>

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}


int main(int argc, char **argv){
  
  ros::init(argc, argv, "handshake_control_type");

  
  ros::NodeHandle n;
  char digit; 
  int required_control;

  ros::Publisher control_type_pub = n.advertise<std_msgs::Int32>("handshake_control_type_topic", 1);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;

  while (ros::ok()){

    if (kbhit()){
      
      char digit = getch();

      if (digit == '1' || digit == '2' || digit == '3' || digit == '4' || digit == '5' || digit == '6'){

      required_control = digit - '0';
      std::cout << "Controller Type "<< required_control <<" Enabled" << std::endl;
      }
    }

    
    control_type_pub.publish(required_control);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}