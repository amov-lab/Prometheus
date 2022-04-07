/*
 * joy_node
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// \author: Blaise Gassend

#include <memory>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/input.h>
#include <linux/joystick.h>
#include <math.h>
#include <sys/stat.h>
#include <unistd.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>


int closedir_cb(DIR *dir)
{
  if (dir)
    return closedir(dir);
  return 0;
}


/// \brief Opens, reads from and publishes joystick events
class Joystick
{
private:
  ros::NodeHandle nh_;
  bool open_;
  bool sticky_buttons_;
  bool default_trig_val_;
  std::string joy_dev_;
  std::string joy_dev_name_;
  std::string joy_dev_ff_;
  double deadzone_;
  double autorepeat_rate_;    // in Hz.  0 for no repeat.
  double coalesce_interval_;  // Defaults to 100 Hz rate limit.
  int event_count_;
  int pub_count_;
  ros::Publisher pub_;
  double lastDiagTime_;

  int ff_fd_;
  struct ff_effect joy_effect_;
  bool update_feedback_;

  diagnostic_updater::Updater diagnostic_;

  typedef std::unique_ptr<DIR, decltype(&closedir)> dir_ptr;

  /// \brief Publishes diagnostics and status
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    double now = ros::Time::now().toSec();
    double interval = now - lastDiagTime_;
    if (open_)
    {
      stat.summary(0, "OK");
    }
    else
    {
      stat.summary(2, "Joystick not open.");
    }

    stat.add("topic", pub_.getTopic());
    stat.add("device", joy_dev_);
    stat.add("device name", joy_dev_name_);
    stat.add("dead zone", deadzone_);
    stat.add("autorepeat rate (Hz)", autorepeat_rate_);
    stat.add("coalesce interval (s)", coalesce_interval_);
    stat.add("recent joystick event rate (Hz)", event_count_ / interval);
    stat.add("recent publication rate (Hz)", pub_count_ / interval);
    stat.add("subscribers", pub_.getNumSubscribers());
    stat.add("default trig val", default_trig_val_);
    stat.add("sticky buttons", sticky_buttons_);
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = now;
  }

  /*! \brief Returns the device path of the first joystick that matches joy_name.
   *         If no match is found, an empty string is returned.
   */
  std::string get_dev_by_joy_name(const std::string& joy_name)
  {
    const char path[] = "/dev/input";  // no trailing / here
    struct dirent *entry;
    struct stat stat_buf;

    dir_ptr dev_dir(opendir(path), &closedir_cb);
    if (dev_dir == nullptr)
    {
      ROS_ERROR("Couldn't open %s. Error %i: %s.", path, errno, strerror(errno));
      return "";
    }

    while ((entry = readdir(dev_dir.get())) != nullptr)
    {
      // filter entries
      if (strncmp(entry->d_name, "js", 2) != 0)  // skip device if it's not a joystick
      {
        continue;
      }
      std::string current_path = std::string(path) + "/" + entry->d_name;
      if (stat(current_path.c_str(), &stat_buf) == -1)
      {
        continue;
      }
      if (!S_ISCHR(stat_buf.st_mode))  // input devices are character devices, skip other
      {
        continue;
      }

      // get joystick name
      int joy_fd = open(current_path.c_str(), O_RDONLY);
      if (joy_fd == -1)
      {
        continue;
      }

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
      {
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));
      }

      close(joy_fd);

      ROS_INFO("Found joystick: %s (%s).", current_joy_name, current_path.c_str());

      if (strcmp(current_joy_name, joy_name.c_str()) == 0)
      {
        return current_path;
      }
    }

    return "";
  }

  /*! \brief Autodetection of the force feedback device. If autodetection fails,
   *         returns empty string.
   * \param joy_dev A nonempty path to the joy device we search force feedback for.
   */
  std::string get_ff_dev(const std::string& joy_dev)
  {
    const char path[] = "/dev/input/by-id";  // no trailing / here
    struct dirent *entry;

    // the selected joy can be a symlink, but we want the canonical /dev/input/jsX
    char realpath_buf[PATH_MAX];
    char *res = realpath(joy_dev.c_str(), realpath_buf);
    if (res == nullptr)
    {
      return "";
    }

    dir_ptr dev_dir(opendir(path), &closedir_cb);
    if (dev_dir == nullptr)
    {
      ROS_ERROR("Couldn't open %s. Error %i: %s.", path, errno, strerror(errno));
      return "";
    }

    const std::string joy_dev_real(realpath_buf);
    std::string joy_dev_id;

    // first, find the device in /dev/input/by-id that corresponds to the selected joy,
    // i.e. its realpath is the same as the selected joy's one

    while ((entry = readdir(dev_dir.get())) != nullptr)
    {
      res = strstr(entry->d_name, "-joystick");
      // filter entries
      if (res == nullptr)  // skip device if it's not a joystick
      {
        continue;
      }

      const auto current_path = std::string(path) + "/" + entry->d_name;
      res = realpath(current_path.c_str(), realpath_buf);
      if (res == nullptr)
      {
        continue;
      }

      const std::string dev_real(realpath_buf);
      if (dev_real == joy_dev_real)
      {
        // we found the ID device which maps to the selected joy
        joy_dev_id = current_path;
        break;
      }
    }

    // if no corresponding ID device was found, the autodetection won't work
    if (joy_dev_id.empty())
    {
      return "";
    }

    const auto joy_dev_id_prefix = joy_dev_id.substr(0, joy_dev_id.length() - strlen("-joystick"));
    std::string event_dev;

    // iterate through the by-id dir once more, this time finding the -event-joystick file with the
    // same prefix as the ID device we've already found
    dev_dir = dir_ptr(opendir(path), &closedir_cb);
    while ((entry = readdir(dev_dir.get())) != nullptr)
    {
      res = strstr(entry->d_name, "-event-joystick");
      if (res == nullptr)  // skip device if it's not an event joystick
      {
        continue;
      }

      const auto current_path = std::string(path) + "/" + entry->d_name;
      if (current_path.find(joy_dev_id_prefix) != std::string::npos)
      {
        ROS_INFO("Found force feedback event device %s", current_path.c_str());
        event_dev = current_path;
        break;
      }
    }

    return event_dev;
  }

public:
  Joystick() : nh_(), diagnostic_(), ff_fd_(-1)
  {}

  void set_feedback(const sensor_msgs::JoyFeedbackArray::ConstPtr& msg)
  {
    if (ff_fd_ == -1)
    {
      return;  // we arent ready yet
    }

    size_t size = msg->array.size();
    for (size_t i = 0; i < size; i++)
    {
      // process each feedback
      if (msg->array[i].type == 1 && ff_fd_ != -1)  // TYPE_RUMBLE
      {
        // if id is zero, thats low freq, 1 is high
        joy_effect_.direction = 0;  // down
        joy_effect_.type = FF_RUMBLE;
        if (msg->array[i].id == 0)
        {
          joy_effect_.u.rumble.strong_magnitude = (static_cast<float>(0xFFFFU))*msg->array[i].intensity;
        }
        else
        {
          joy_effect_.u.rumble.weak_magnitude = (static_cast<float>(0xFFFFU))*msg->array[i].intensity;
        }

        joy_effect_.replay.length = 1000;
        joy_effect_.replay.delay = 0;

        update_feedback_ = true;
      }
    }
  }

  /// \brief Opens joystick port, reads from port and publishes while node is active
  int main(int argc, char **argv)
  {
    diagnostic_.add("Joystick Driver Status", this, &Joystick::diagnostics);
    diagnostic_.setHardwareID("none");

    // Parameters
    ros::NodeHandle nh_param("~");
    pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    ros::Subscriber sub = nh_.subscribe("joy/set_feedback", 10, &Joystick::set_feedback, this);
    nh_param.param<std::string>("dev", joy_dev_, "/dev/input/js0");
    nh_param.param<std::string>("dev_ff", joy_dev_ff_, "/dev/input/event0");
    nh_param.param<std::string>("dev_name", joy_dev_name_, "");
    nh_param.param<double>("deadzone", deadzone_, 0.05);
    nh_param.param<double>("autorepeat_rate", autorepeat_rate_, 0);
    nh_param.param<double>("coalesce_interval", coalesce_interval_, 0.001);
    nh_param.param<bool>("default_trig_val", default_trig_val_, false);
    nh_param.param<bool>("sticky_buttons", sticky_buttons_, false);

    // Checks on parameters
    if (!joy_dev_name_.empty())
    {
        std::string joy_dev_path = get_dev_by_joy_name(joy_dev_name_);
        if (joy_dev_path.empty())
        {
          ROS_ERROR("Couldn't find a joystick with name %s. Falling back to default device.", joy_dev_name_.c_str());
        }
        else
        {
          ROS_INFO("Using %s as joystick device.", joy_dev_path.c_str());
          joy_dev_ = joy_dev_path;
        }
    }

    if (autorepeat_rate_ > 1 / coalesce_interval_)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f Hz) > 1/coalesce_interval (%f Hz) "
        "does not make sense. Timing behavior is not well defined.", autorepeat_rate_, 1/coalesce_interval_);
    }

    if (deadzone_ >= 1)
    {
      ROS_WARN("joy_node: deadzone greater than 1 was requested. The semantics of deadzone have changed. "
        "It is now related to the range [-1:1] instead of [-32767:32767]. For now I am dividing your deadzone "
        "by 32767, but this behavior is deprecated so you need to update your launch file.");
      deadzone_ /= 32767;
    }

    if (deadzone_ > 0.9)
    {
      ROS_WARN("joy_node: deadzone (%f) greater than 0.9, setting it to 0.9", deadzone_);
      deadzone_ = 0.9;
    }

    if (deadzone_ < 0)
    {
      ROS_WARN("joy_node: deadzone_ (%f) less than 0, setting to 0.", deadzone_);
      deadzone_ = 0;
    }

    if (autorepeat_rate_ < 0)
    {
      ROS_WARN("joy_node: autorepeat_rate (%f) less than 0, setting to 0.", autorepeat_rate_);
      autorepeat_rate_ = 0;
    }

    if (coalesce_interval_ < 0)
    {
      ROS_WARN("joy_node: coalesce_interval (%f) less than 0, setting to 0.", coalesce_interval_);
      coalesce_interval_ = 0;
    }

    // Parameter conversions
    double autorepeat_interval = 1 / autorepeat_rate_;
    double scale = -1. / (1. - deadzone_) / 32767.;
    double unscaled_deadzone = 32767. * deadzone_;

    js_event event;
    struct timeval tv;
    fd_set set;
    int joy_fd;
    event_count_ = 0;
    pub_count_ = 0;
    lastDiagTime_ = ros::Time::now().toSec();

    // Big while loop opens, publishes
    while (nh_.ok())
    {
      open_ = false;
      diagnostic_.force_update();
      bool first_fault = true;
      while (true)
      {
        ros::spinOnce();
        if (!nh_.ok())
        {
          goto cleanup;
        }
        joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        if (joy_fd != -1)
        {
          // There seems to be a bug in the driver or something where the
          // initial events that are to define the initial state of the
          // joystick are not the values of the joystick when it was opened
          // but rather the values of the joystick when it was last closed.
          // Opening then closing and opening again is a hack to get more
          // accurate initial state data.
          close(joy_fd);
          joy_fd = open(joy_dev_.c_str(), O_RDONLY);
        }
        if (joy_fd != -1)
        {
          break;
        }
        if (first_fault)
        {
          ROS_ERROR("Couldn't open joystick %s. Will retry every second.", joy_dev_.c_str());
          first_fault = false;
        }
        sleep(1.0);
        diagnostic_.update();
      }

      auto dev_ff = joy_dev_ff_;
      if (joy_dev_ff_.empty())
      {
        dev_ff = get_ff_dev(joy_dev_);
      }

      if (!dev_ff.empty())
      {
        ff_fd_ = open(dev_ff.c_str(), O_RDWR);

        /* Set the gain of the device*/
        int gain = 100;           /* between 0 and 100 */
        struct input_event ie;      /* structure used to communicate with the driver */

        ie.type = EV_FF;
        ie.code = FF_GAIN;
        ie.value = 0xFFFFUL * gain / 100;

        if (write(ff_fd_, &ie, sizeof(ie)) == -1)
        {
          ROS_WARN("Couldn't set gain on joystick force feedback: %s", strerror(errno));
        }

        memset(&joy_effect_, 0, sizeof(joy_effect_));
        joy_effect_.id = -1;
        joy_effect_.direction = 0;  // down
        joy_effect_.type = FF_RUMBLE;
        joy_effect_.u.rumble.strong_magnitude = 0;
        joy_effect_.u.rumble.weak_magnitude = 0;
        joy_effect_.replay.length = 1000;
        joy_effect_.replay.delay = 0;

        // upload the effect
        int ret = ioctl(ff_fd_, EVIOCSFF, &joy_effect_);
      }

      char current_joy_name[128];
      if (ioctl(joy_fd, JSIOCGNAME(sizeof(current_joy_name)), current_joy_name) < 0)
      {
        strncpy(current_joy_name, "Unknown", sizeof(current_joy_name));
      }

      ROS_INFO("Opened joystick: %s (%s). deadzone_: %f.", joy_dev_.c_str(), current_joy_name, deadzone_);
      open_ = true;
      diagnostic_.force_update();

      bool tv_set = false;
      bool publication_pending = false;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      sensor_msgs::Joy joy_msg;  // Here because we want to reset it on device close.
      double val;  // Temporary variable to hold event values
      while (nh_.ok())
      {
        ros::spinOnce();

        bool publish_now = false;
        bool publish_soon = false;
        FD_ZERO(&set);
        FD_SET(joy_fd, &set);

        int select_out = select(joy_fd+1, &set, nullptr, nullptr, &tv);
        if (select_out == -1)
        {
          tv.tv_sec = 0;
          tv.tv_usec = 0;
          continue;
        }

        // play the rumble effect (can probably do this at lower rate later)
        if (ff_fd_ != -1)
        {
          struct input_event start;
          start.type = EV_FF;
          start.code = joy_effect_.id;
          start.value = 1;
          if (write(ff_fd_, (const void*) &start, sizeof(start)) == -1)
          {
            break;  // fd closed
          }

          // upload the effect
          if (update_feedback_ == true)
          {
            int ret = ioctl(ff_fd_, EVIOCSFF, &joy_effect_);
            update_feedback_ = false;
          }
        }

        if (FD_ISSET(joy_fd, &set))
        {
          if (read(joy_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
          {
            break;  // Joystick is probably closed. Definitely occurs.
          }

          joy_msg.header.stamp = ros::Time().now();
          event_count_++;
          switch (event.type)
          {
          case JS_EVENT_BUTTON:
          case JS_EVENT_BUTTON | JS_EVENT_INIT:
            if (event.number >= joy_msg.buttons.size())
            {
              size_t old_size = joy_msg.buttons.size();
              joy_msg.buttons.resize(event.number+1);
              for (size_t i = old_size; i < joy_msg.buttons.size(); i++)
              {
                joy_msg.buttons[i] = 0.0;
              }
            }
            if (sticky_buttons_)
            {
              if (event.value == 1)
              {
                joy_msg.buttons[event.number] = 1 - joy_msg.buttons[event.number];
              }
            }
            else
            {
              joy_msg.buttons[event.number] = (event.value ? 1 : 0);
            }
            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT))
            {
              publish_now = true;
            }
            else
            {
              publish_soon = true;
            }
            break;
          case JS_EVENT_AXIS:
          case JS_EVENT_AXIS | JS_EVENT_INIT:
            val = event.value;
            if (event.number >= joy_msg.axes.size())
            {
              size_t old_size = joy_msg.axes.size();
              joy_msg.axes.resize(event.number+1);
              for (size_t i = old_size; i < joy_msg.axes.size(); i++)
              {
                joy_msg.axes[i] = 0.0;
              }
            }
            if (default_trig_val_)
            {
              // Allows deadzone to be "smooth"
              if (val > unscaled_deadzone)
              {
                val -= unscaled_deadzone;
              }
              else if (val < -unscaled_deadzone)
              {
                val += unscaled_deadzone;
              }
              else
              {
                val = 0;
              }
              joy_msg.axes[event.number] = val * scale;
              // Will wait a bit before sending to try to combine events.
              publish_soon = true;
              break;
            }
            else
            {
              if (!(event.type & JS_EVENT_INIT))
              {
                val = event.value;
                if (val > unscaled_deadzone)
                {
                  val -= unscaled_deadzone;
                }
                else if (val < -unscaled_deadzone)
                {
                  val += unscaled_deadzone;
                }
                else
                {
                  val = 0;
                }
                joy_msg.axes[event.number] = val * scale;
              }

              publish_soon = true;
              break;
            }
            default:
              ROS_WARN("joy_node: Unknown event type. Please file a ticket. "
                "time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
              break;
          }
        }
        else if (tv_set)  // Assume that the timer has expired.
        {
          joy_msg.header.stamp = ros::Time().now();
          publish_now = true;
        }

        if (publish_now)
        {
          // Assume that all the JS_EVENT_INIT messages have arrived already.
          // This should be the case as the kernel sends them along as soon as
          // the device opens.
          joy_msg.header.stamp = ros::Time().now();
          joy_msg.header.frame_id = joy_dev_.c_str();
          pub_.publish(joy_msg);

          publish_now = false;
          tv_set = false;
          publication_pending = false;
          publish_soon = false;
          pub_count_++;
        }

        // If an axis event occurred, start a timer to combine with other
        // events.
        if (!publication_pending && publish_soon)
        {
          tv.tv_sec = trunc(coalesce_interval_);
          tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
          publication_pending = true;
          tv_set = true;
        }

        // If nothing is going on, start a timer to do autorepeat.
        if (!tv_set && autorepeat_rate_ > 0)
        {
          tv.tv_sec = trunc(autorepeat_interval);
          tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
          tv_set = true;
        }

        if (!tv_set)
        {
          tv.tv_sec = 1;
          tv.tv_usec = 0;
        }

        diagnostic_.update();
      }  // End of joystick open loop.

      close(ff_fd_);
      close(joy_fd);
      ros::spinOnce();
      if (nh_.ok())
      {
        ROS_ERROR("Connection to joystick device lost unexpectedly. Will reopen.");
      }
    }

  cleanup:
    ROS_INFO("joy_node shut down.");

    return 0;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_node");
  Joystick j;
  return j.main(argc, argv);
}
