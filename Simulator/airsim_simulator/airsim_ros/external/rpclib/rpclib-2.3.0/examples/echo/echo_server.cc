#include "rpc/server.h"
#include <iostream>

int main() {
    rpc::server srv(rpc::constants::DEFAULT_PORT);
    std::cout << "registered on  port " << srv.port() << std::endl;

    srv.bind("echo", [](std::string const& s) {
        return s;
    });

    srv.run();
    return 0;
}
