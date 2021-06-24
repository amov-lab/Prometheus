#ifndef PRINTF_UTILS_H
#define PRINTF_UTILS_H

#include <string>
using namespace std;

#define RED "\033[0;1;31m"
#define GREEN "\033[0;1;32m"
#define YELLOW "\033[0;1;33m"
#define BLUE "\033[0;1;34m"
#define PURPLE "\033[0;1;35m"
#define DEEPGREEN "\033[0;1;36m"
#define WHITE "\033[0;1;37m"

#define RED_IN_WHITE "\033[0;47;31m"
#define GREEN_IN_WHITE "\033[0;47;32m"
#define YELLOW_IN_WHITE "\033[0;47;33m"

#define TAIL "\033[0m"


// Example:
// cout << GREEN << "Test for Green text." << TAIL <<endl; 

// 字背景颜色范围:40--49            字颜色: 30--39  
// 40:黑                          30: 黑  
// 41:红                          31: 红  
// 42:绿                          32: 绿  
// 43:黄                          33: 黄  
// 44:蓝                          34: 蓝  
// 45:紫                          35: 紫  
// 46:深绿                        36: 深绿  
// 47:白色                        37: 白色  

// 参考资料：https://blog.csdn.net/u014470361/article/details/81512330
// cout <<  "\033[0;1;31m" << "Hello World, Red color!"  << "\033[0m" << endl;
// printf(  "\033[0;30;41m color!!! \033[0m Hello \n");
// ROS_INFO("\033[1;33;41m----> Hightlight color.\033[0m");
// 其中41的位置代表字的背景色, 30的位置是代表字的颜色，0 是字的一些特殊属性，0代表默认关闭，一些其他属性如闪烁、下划线等。     

// ROS_INFO_STREAM_ONCE ("\033[1;32m---->Setting to OFFBOARD Mode....\033[0m");//绿色只打印一次

#endif
