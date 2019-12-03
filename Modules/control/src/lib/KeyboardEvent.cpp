#include "KeyboardEvent.h"

KeyboardEvent::KeyboardEvent()
{
    input_flag = -1;
    buff = 0;
}

void KeyboardEvent::getch()
{
    input_flag = -1; //if not key is pressed, then return -1 for the keyboard_input. reset this flag every time
    fd_set set;
    struct timeval timeout;
    int rv;
    buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    //timeout.tv_usec = 1000;//
    timeout.tv_usec = 1000;
    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
            ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
            ROS_ERROR("tcsetattr ICANON");

    if(rv == -1){
            ROS_ERROR("select");
    }
    else if(rv == 0)
    {
            //{ROS_INFO("-----");
    }
    else
    {read(filedesc, &buff, len);
    input_flag = 1;}

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
            ROS_ERROR ("tcsetattr ~ICANON");
}
char KeyboardEvent::GetPressedKey()
{
    char key;
    if(input_flag>-1)//if keyboard is pressed, then perform the callbacks
    {
        key = buff;
    }else{
      key = U_KEY_NONE;
    }
    return key;
}
void KeyboardEvent::RosWhileLoopRun()
{
    getch();
}
int  KeyboardEvent::input_flag;
char KeyboardEvent::buff;
