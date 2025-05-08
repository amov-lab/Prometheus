#include "rpc/nonstd/optional.hpp"

// This is no-op; the reason it exists is to avoid
// the weak vtables problem. For more info, see
// https://stackoverflow.com/a/23749273/140367
const char* nonstd::bad_optional_access::what() const noexcept {
    return std::logic_error::what();
}
