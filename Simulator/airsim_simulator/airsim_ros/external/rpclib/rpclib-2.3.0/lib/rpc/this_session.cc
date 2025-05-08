#include "rpc/this_session.h"

namespace rpc
{

this_session_t &this_session() {
    static thread_local this_session_t instance;
    return instance;
}

void this_session_t::post_exit() {
    exit_ = true;
}

void this_session_t::clear() {
    exit_ = false;
}

session_id_t this_session_t::id() const {
    return id_;
}

void this_session_t::set_id(session_id_t value) {
    id_ = value;
}

    
} /* rpc */ 
