#include "rpc/this_server.h"

namespace rpc
{

this_server_t &this_server() {
    static thread_local this_server_t instance;
    return instance;
}

void this_server_t::stop() {
    stopping_ = true;
}

void this_server_t::cancel_stop() {
    stopping_ = false;
}

} /* rpc */
