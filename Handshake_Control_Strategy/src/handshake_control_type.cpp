#include "ros/ros.h"
#include "std_msgs/String.h"

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


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "handshake_control_type");

  
  ros::NodeHandle n;
  int digit, required_control;

  ros::Publisher control_type_pub = n.advertise<std_msgs::int32>("handshake_control_type_topic", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {

    if (kbhit()){
      
      (void)getch();
      while (!(control_type == 1 || control_type == 2 || control_type == 3 || control_type == 4 || control_type == 5 || control_type == 6 || control_type == 7 || control_type == 8)){

      std::cout << "Invalid control strategy paramether, please digit from 1 to 6" << std::endl;
      std::cin >> digit;
      
      if(scanf("%d%c", &digit, &term) != 2 || term != '\n'){

      return -1;

      }
    
      required_control = digit;
      std::cout << "Controller Type "<< required_control <<" Enabled" << std::endl;

    }

    
    chatter_pub.publish(required_control);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}