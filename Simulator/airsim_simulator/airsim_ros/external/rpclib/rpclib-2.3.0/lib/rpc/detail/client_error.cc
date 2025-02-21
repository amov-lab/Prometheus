#include "format.h"

#include "rpc/detail/client_error.h"

namespace rpc {
namespace detail {

client_error::client_error(code c, const std::string &msg)
    : what_(RPCLIB_FMT::format("client error C{0:04x}: {1}",
                               static_cast<uint16_t>(c), msg)) {}

const char *client_error::what() const noexcept { return what_.c_str(); }
}
}

