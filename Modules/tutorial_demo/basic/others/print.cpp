#include <unistd.h>
#include "printf_utils.h"

/*
PCOUT(float interval, string color, string msg)
    : insterval 打印到间隔时长，单位为秒。为负数时，为等待条形式。
    : color 答应文字的颜色
    : msg 打印到字符串

基本原理：
    PCOUT是一个宏，每调用一次会根据当前行实例化一个命名唯一静态的对象，用于保存上一时刻的打印时间。
    当下一次循环到此处时，比较时间间隔判断是否打印。
*/

int main(int, char **)
{
    // 打印等待条
    for (int i = 0; i < 100; ++i)
    {
        PCOUT(-1, WHITE, "打印等待条,白色");
        usleep(1000 * 1000 * 0.1);
    }
    // 每隔1秒打印一次
    for (int i = 0; i < 100; ++i)
    {
        PCOUT(1, GREEN, "每隔1秒打印一次,绿色");
        // 休眠0.1秒
        usleep(1000 * 1000 * 0.1);
    }
    // 每个2秒打印一次
    for (int i = 0; i < 100; ++i)
    {
        PCOUT(2, YELLOW, "每隔2秒打印一次,黄色");
        // 休眠0.1秒
        usleep(1000 * 1000 * 0.1);
    }
    // 每次都打印
    for (int i = 0; i < 100; ++i)
    {
        PCOUT(0, RED, "每次都打印,红色");
        // 休眠0.1秒
        usleep(1000 * 1000 * 0.1);
    }
}