#include <iostream>

#include "rpc/server.h"
#include "rpc/this_handler.h"

double divide(double a, double b) {
    if (b == 0.0) {
        rpc::this_handler().respond_error(
                std::make_tuple(1, "Division by zero"));
    }
    return a / b;
}

struct subtractor {
    double operator()(double a, double b) {
        return a - b;
    }
};

struct multiplier {
    double multiply(double a, double b) {
        return a * b;
    }
};

int main() {
    rpc::server srv(rpc::constants::DEFAULT_PORT);
    subtractor s;
    multiplier m;

    // It's possible to bind non-capturing lambdas
    srv.bind("add", [](double a, double b) { return a + b; });
    // ... arbitrary callables
    srv.bind("sub", s);
    // ... free functions
    srv.bind("div", &divide);
    // ... member functions with captured instances in lambdas
    srv.bind("mul", [&m](double a, double b) { return m.multiply(a, b); });

    srv.run();

    return 0;
}
