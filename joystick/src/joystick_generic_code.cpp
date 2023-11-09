#include <thread>

#include "classes/EstadoTeleops.h"
#include "classes/joystick/VehicleJoystickController.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joystick_node");
  ROS_DEBUG_STREAM("joystick_node");
  ros::NodeHandle nh("~");
  ros::Rate rate(10);
  JoystickController joystick(nh);

  joystick.initialize();

  std::thread joys(&JoystickController::readData, &joystick);

  while (ros::ok()) {
    if (joystick.status == EstadoScooter::JOYSTICK) {
      joystick.publishData();
    }

    if (joystick.mac_update || joystick.pair_req) {
      joystick.initPairing();
      joystick.initialize();
      joystick.requestHeartbeat();
    }

    if (joystick.reload_device) {
      joystick.initialize();
      joystick.reload_device = false;
    }
    rate.sleep();
    ros::spinOnce();
  }

  // joys.join();
  return 0;
}
