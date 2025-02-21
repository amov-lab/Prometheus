#include "rpc/detail/response.h"
#include "rpc/detail/log.h"
#include "rpc/detail/util.h"

#include <assert.h>

namespace rpc {
namespace detail {

response::response() : id_(0), error_(), result_(), empty_(false) {}

response::response(RPCLIB_MSGPACK::object_handle o) : response() {
    response_type r;
    o.get().convert(r);
    // TODO: check protocol [t.szelei 2015-12-30]
    id_ = std::get<1>(r);
    auto &&error_obj = std::get<2>(r);
    if (!error_obj.is_nil()) {
        error_ = std::make_shared<RPCLIB_MSGPACK::object_handle>();
        *error_ = RPCLIB_MSGPACK::clone(error_obj);
    }
    result_ = std::make_shared<RPCLIB_MSGPACK::object_handle>(
         std::get<3>(r), std::move(o.zone()));
}

RPCLIB_MSGPACK::sbuffer response::get_data() const {
    RPCLIB_MSGPACK::sbuffer data;
    response_type r(1, id_, error_ ? error_->get() : RPCLIB_MSGPACK::object(),
                    result_ ? result_->get() : RPCLIB_MSGPACK::object());
    RPCLIB_MSGPACK::pack(data, r);
    return data;
}

uint32_t response::get_id() const { return id_; }

std::shared_ptr<RPCLIB_MSGPACK::object_handle> response::get_error() const { return error_; }

std::shared_ptr<RPCLIB_MSGPACK::object_handle> response::get_result() const {
    return result_;
}

response response::empty() {
    response r;
    r.empty_ = true;
    return r;
}

bool response::is_empty() const { return empty_; }

void response::capture_result(RPCLIB_MSGPACK::object_handle &r) {
    if (!result_) {
        result_ = std::make_shared<RPCLIB_MSGPACK::object_handle>();
    }
    result_->set(std::move(r).get());
}

void response::capture_error(RPCLIB_MSGPACK::object_handle &e) {
    if (!error_) {
        error_ = std::shared_ptr<RPCLIB_MSGPACK::object_handle>();
    }
    error_->set(std::move(e).get());
}

} /* detail */
} /* rpc */
