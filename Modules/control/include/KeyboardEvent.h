#pragma once
#include <termios.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"

# define U_KEY_NONE -1
# define U_KEY_SPACE 32
# define U_KEY_E 101
# define U_KEY_F 102
# define U_KEY_Q 113
# define U_KEY_R 114
# define U_KEY_S 115
# define U_KEY_T 116
# define U_KEY_U 117
# define U_KEY_V 118
# define U_KEY_W 119
# define U_KEY_X 120
# define U_KEY_Y 121
# define U_KEY_Z 122

using namespace std;

class KeyboardEvent{
    static int  input_flag; //keyboard flag
    static char buff; // keyboard buff
    static void getch();// get keyboard input. We need all instance to use the same function to catch the keyboard
public:
    KeyboardEvent();
    void RosWhileLoopRun(); // this should be placed inside the ros while loop
    char GetPressedKey();
};
