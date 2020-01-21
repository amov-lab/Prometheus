#! /usr/bin/env python

import rospy
import time
from threading import Lock
from sensor_msgs.msg import Joy

class Listener:
    def __init__(self, topic_name, topic_type, wait_for_data=False):
        """
        Listener is a wrapper around a subscriber where the callback simply records the latest msg.

        Listener does not consume the message 
            (for consuming behavior, use the standard ros callback pattern)
        Listener does not check timestamps of message headers

        Parameters:
            topic_name (str):      name of topic to subscribe to
            topic_type (msg_type): type of message received on topic
            wait_for_data (bool):  block constructor until a message has been received
        """

        self.data = None
        self.lock = Lock()
            
        self.subscriber = rospy.Subscriber(topic_name, topic_type, self.callback)
        self.get(wait_for_data)

        
    def callback(self, msg):
        with self.lock:
            self.data = msg

    def get(self, block_until_data=True):
        """
        Returns the latest msg from the subscribed topic

        Parameters:
            block_until_data (bool): block if no message has been received yet. 
                                     Guarantees a msg is returned (not None)
        """
        wait_for(lambda: not (block_until_data and self.data is None))
            
        with self.lock:
            return self.data

    
def wait_for(func):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """
    
    while not func() and not rospy.is_shutdown():
        time.sleep(0.01)

        
def joy_to_xbox(joy):
    """
    Transforms a joystick sensor_msg to a XBox controller for easier code readability
    
    Parameters:
    joy (sensor_msgs/Joy): xbox msg

    Returns:
    xbox struct where fields are the button names
    """
    class Xbox_msg():
        pass
    x = Xbox_msg()
    x.A, x.B, x.X, x.Y, x.LB, x.RB, \
        x.back, x.start, x.power,\
        x.stick_button_left, x.stick_button_right, \
        x.DL, x.DR, x.DU, x.DD = joy.buttons
    x.LH, x.LV, x.LT, x.RH, x.RV, x.RT, x.DH, x.DV = joy.axes
    return x


class Xbox():
    def __init__(self, joystick_topic="joy"):
        self.xbox_listener = Listener(joystick_topic, Joy)

    def get_buttons_state(self):
        """
        Returns an xbox struct of the last joystick message received
        """
        return joy_to_xbox(self.xbox_listener.get())

    def get_button(self, button):
        """
        Return value of button or axis of the controller
        0 or 1 for buttons
        -1.0 to 1.0 (at most) for axes
        """
        return getattr(self.get_buttons_state(), button)

    def wait_for_button(self, button, message=True):
        """
        Waits for button press on xbox.

        Parameters:
        button (str):   Name of xbox button. "A", "B", "X", ...
        message (bool): log a message informing the user?
        """
        if message:
            rospy.loginfo("Waiting for xbox button: " + button)
            
        wait_for(lambda: not self.get_button(button) == 0)
