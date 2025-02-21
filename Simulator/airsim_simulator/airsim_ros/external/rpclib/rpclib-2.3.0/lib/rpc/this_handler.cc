#include "rpc/this_handler.h"

namespace rpc {

this_handler_t &this_handler() {
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wexit-time-destructors"
#endif
    static thread_local this_handler_t instance;
#ifdef __clang__
#pragma clang diagnostic pop
#endif
    return instance;
}

void this_handler_t::disable_response() { resp_enabled_ = false; }

void this_handler_t::enable_response() { resp_enabled_ = true; }

void this_handler_t::clear() {
    error_.set(RPCLIB_MSGPACK::object());
    resp_.set(RPCLIB_MSGPACK::object());
    enable_response();
}

} /* rpc */
