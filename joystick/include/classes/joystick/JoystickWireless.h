#ifndef _UAPI_LINUX_JOYSTICK_H
#define _UAPI_LINUX_JOYSTICK_H

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
/*
 * Version
 */

#define JS_VERSION 0x020100

/*
 * Types and constants for reading from /dev/js
 */

#define JS_EVENT_BUTTON 0x01 /* button pressed/released */
#define JS_EVENT_AXIS 0x02   /* joystick moved */
#define JS_EVENT_INIT 0x80   /* initial state of device */

/*xbox buttons*/
#define A 0
#define B 1
#define X 3
#define Y 4
#define LB 6
#define RB 7
#define SQUARES 10
#define LINES 11
#define LEFT_THUMBSTICK_BUTTON 13
#define RIGHT_THUMBSTICK_BUTTON 14

/*xbox axes*/
#define JS_LEFT_X 0
#define JS_LEFT_y 1
#define JS_RIGHT_X 2
#define JS_RIGHT_Y 3
#define RT 4
#define LT 5
#define CROSS_X 6
#define CROSS_Y 7

struct js_event {
  __u32 time;  /* event timestamp in milliseconds */
  __s16 value; /* value */
  __u8 type;   /* event type */
  __u8 number; /* axis/button number */
};

/*
 * IOCTL commands for joystick driver
 */

#define JSIOCGVERSION _IOR('j', 0x01, __u32) /* get driver version */

#define JSIOCGAXES _IOR('j', 0x11, __u8)    /* get number of axes */
#define JSIOCGBUTTONS _IOR('j', 0x12, __u8) /* get number of buttons */
#define JSIOCGNAME(len) \
  _IOC(_IOC_READ, 'j', 0x13, len) /* get identifier string */

#define JSIOCSCORR _IOW('j', 0x21, struct js_corr) /* set correction values */
#define JSIOCGCORR _IOR('j', 0x22, struct js_corr) /* get correction values */

#define JSIOCSAXMAP _IOW('j', 0x31, __u8[ABS_CNT]) /* set axis mapping */
#define JSIOCGAXMAP _IOR('j', 0x32, __u8[ABS_CNT]) /* get axis mapping */
#define JSIOCSBTNMAP \
  _IOW('j', 0x33, __u16[KEY_MAX - BTN_MISC + 1]) /* set button mapping */
#define JSIOCGBTNMAP \
  _IOR('j', 0x34, __u16[KEY_MAX - BTN_MISC + 1]) /* get button mapping */

using namespace std;

#endif

class Joystick {
 public:
  struct js_event event;

  int axis, direction, value;
  int type, number;
  int fd{-1};
  size_t axes_no, buttons_no;
  const char *device;

  void set_device() { device = "/dev/input/js0"; };
  int open_fd();
  int close_fd();
  int button_count();
  int axis_count();
  int read_event(struct js_event *event);
  int get_axis_state(struct js_event *event);
  void update_event();
};

/*
 * This function updates the joystick variables acording to the type of event.
 */
void Joystick::update_event() {
  type = event.type;
  ROS_DEBUG("update_event call");
  switch (type) {
    case JS_EVENT_BUTTON:
      number = event.number;
      value = event.value;
      //~ cout << number << "," << value << endl;
      break;

    case JS_EVENT_AXIS:
      get_axis_state(&event);
      //~ cout << axis << "," << direction << "," << value << endl;
      break;

    default:
      break;
  }
}

int Joystick::read_event(struct js_event *event) {
  ssize_t bytes;

  bytes = read(fd, event, sizeof(*event));

  if (bytes == sizeof(*event)) return 0;

  ROS_INFO_STREAM("Event: Couldn't read full event.");
  // cout << "Event: Couldn't read full event." << endl;
  return -1;
}

int Joystick::get_axis_state(struct js_event *event) {
  size_t ax = event->number;

  axis = ax;
  switch (axis % 3) {
    case 0:
      direction = 0;
      break;

    case 1:
      direction = 1;
      break;

    case 2:
      direction = 2;
      break;
  }
  value = event->value;

  return 1;
}

int Joystick::axis_count() {
  __u8 axes;

  if (ioctl(fd, JSIOCGAXES, &axes) == -1) {
    cout << "Axis: Couldn't count joystick axis." << endl;
    return -1;
  }
  axes_no = axes;
  cout << "axis: " << axes_no << endl;
  return 1;
}

int Joystick::button_count() {
  __u8 buttons;

  if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1) {
    cout << "Buttons: Couldn't count joystick buttons." << endl;
    return -1;
  }
  buttons_no = buttons;
  cout << "buttons: " << buttons_no << endl;
  return 1;
}

int Joystick::open_fd() {
  fd = open(device, O_RDONLY);
  if (fd < 0) {
    // cout << "fd: Couldn't open joystick event." << endl;
    return -1;
  }
  cout << "fd: " << fd << endl;
  return fd;
}

int Joystick::close_fd() {
  int nRet = close(fd);
  if (nRet < 0) {
    // cout << "fd: Couldn't close joystick device." << endl;
    return nRet;
  }
  fd = -1;
  cout << "Device closed, fd: " << fd << endl;
  return nRet;
}