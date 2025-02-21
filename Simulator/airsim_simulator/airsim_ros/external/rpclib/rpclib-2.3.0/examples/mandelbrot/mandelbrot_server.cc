#include <iostream>
#include <math.h>
#include <time.h>

#include "rpc/server.h"
#include "mandelbrot.h"

int mandelbrot(double cr, double ci, int max_iterations) {
    int i = 0;
    double zr = 0.0, zi = 0.0;
    while (i < max_iterations && zr * zr + zi * zi < 4.0) {
        double temp = zr * zr - zi * zi + cr;
        zi = 2.0 * zr * zi + ci;
        zr = temp;
        i++;
    }
    return i;
}

double to_real(int x, int width, double minR, double maxR) {
    double range = maxR - minR;
    return x * (range / width) + minR;
}

double to_im(int y, int height, double minI, double maxI) {
    double range = maxI - minI;
    return y * (range / height) + minI;
}

int main() {
    int maxN = 255;
    double minR = -1.5, maxR = 0.8, minI = -1.0, maxI = 1.0;

    rpc::server srv(rpc::constants::DEFAULT_PORT);

    srv.bind("get_time", []() {
        time_t rawtime;
        struct tm *timeinfo;
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        return asctime(timeinfo);
    });

    srv.bind("get_mandelbrot", [&](int width, int height) {
        pixel_data data;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                double cr = to_real(x, width, minR, maxR);
                double ci = to_im(y, height, minI, maxI);
                int n = mandelbrot(cr, ci, maxN);

                unsigned char r = ((int)(fabs(n * cosf(n))) % 256);
                unsigned char g = ((n * 3) % 256);
                unsigned char b = (n % 256);

                data.push_back({r, g, b});
            }
        }

        return data;
    });

    srv.async_run(2);
    std::cout << "Press [ENTER] to exit the server." << std::endl;
    std::cin.ignore();
    return 0;
}
