#include <iostream>
#include "rpc/client.h"

int main() {
    rpc::client client("127.0.0.1", rpc::constants::DEFAULT_PORT);
    auto result = client.call("add", 2, 3).as<int>();
    std::cout << "The result is: " << result << std::endl;
    return 0;
}
