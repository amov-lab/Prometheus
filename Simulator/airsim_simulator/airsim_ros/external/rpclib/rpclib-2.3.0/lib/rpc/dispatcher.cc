#include "rpc/dispatcher.h"
#include "format.h"
#include "rpc/detail/client_error.h"
#include "rpc/this_handler.h"

namespace rpc {
namespace detail {

using detail::response;

void dispatcher::dispatch(RPCLIB_MSGPACK::sbuffer const &msg) {
    auto unpacked = RPCLIB_MSGPACK::unpack(msg.data(), msg.size());
    dispatch(unpacked.get());
}

response dispatcher::dispatch(RPCLIB_MSGPACK::object const &msg,
                              bool suppress_exceptions) {
    switch (msg.via.array.size) {
    case 3:
        return dispatch_notification(msg, suppress_exceptions);
    case 4:
        return dispatch_call(msg, suppress_exceptions);
    default:
        return response::empty();
    }
}

response dispatcher::dispatch_call(RPCLIB_MSGPACK::object const &msg,
                                   bool suppress_exceptions) {
    call_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == 0);

    auto &&id = std::get<1>(the_call);
    auto &&name = std::get<2>(the_call);
    auto &&args = std::get<3>(the_call);

    auto it_func = funcs_.find(name);

    if (it_func != end(funcs_)) {
        LOG_DEBUG("Dispatching call to '{}'", name);
        try {
            auto result = (it_func->second)(args);
            return response::make_result(id, std::move(result));
        } catch (rpc::detail::client_error &e) {
            return response::make_error(
                id, RPCLIB_FMT::format("rpclib: {}", e.what()));
        } catch (std::exception &e) {
            if (!suppress_exceptions) {
                throw;
            }
            return response::make_error(
                id,
                RPCLIB_FMT::format("rpclib: function '{0}' (called with {1} "
                                   "arg(s)) "
                                   "threw an exception. The exception "
                                   "contained this information: {2}.",
                                   name, args.via.array.size, e.what()));
        } catch (rpc::detail::handler_error &) {
            // doing nothing, the exception was only thrown to
            // return immediately
        } catch (rpc::detail::handler_spec_response &) {
            // doing nothing, the exception was only thrown to
            // return immediately
        } catch (...) {
            if (!suppress_exceptions) {
                throw;
            }
            return response::make_error(
                id,
                RPCLIB_FMT::format("rpclib: function '{0}' (called with {1} "
                                   "arg(s)) threw an exception. The exception "
                                   "is not derived from std::exception. No "
                                   "further information available.",
                                   name, args.via.array.size));
        }
    }
    return response::make_error(
        id, RPCLIB_FMT::format("rpclib: server could not find "
                               "function '{0}' with argument count {1}.",
                               name, args.via.array.size));
}

response dispatcher::dispatch_notification(RPCLIB_MSGPACK::object const &msg,
                                           bool suppress_exceptions) {
    notification_t the_call;
    msg.convert(the_call);

    // TODO: proper validation of protocol (and responding to it)
    // auto &&type = std::get<0>(the_call);
    // assert(type == static_cast<uint8_t>(request_type::notification));

    auto &&name = std::get<1>(the_call);
    auto &&args = std::get<2>(the_call);

    auto it_func = funcs_.find(name);

    if (it_func != end(funcs_)) {
        LOG_DEBUG("Dispatching call to '{}'", name);
        try {
            auto result = (it_func->second)(args);
        } catch (rpc::detail::handler_error &) {
            // doing nothing, the exception was only thrown to
            // return immediately
        } catch (rpc::detail::handler_spec_response &) {
            // doing nothing, the exception was only thrown to
            // return immediately
        } catch (...) {
            if (!suppress_exceptions) {
                throw;
            }
        }
    }
    return response::empty();
}

void dispatcher::enforce_arg_count(std::string const &func, std::size_t found,
                                   std::size_t expected) {
    using detail::client_error;
    if (found != expected) {
        throw client_error(
            client_error::code::wrong_arity,
            RPCLIB_FMT::format(
                "Function '{0}' was called with an invalid number of "
                "arguments. Expected: {1}, got: {2}",
                func, expected, found));
    }
}

void dispatcher::enforce_unique_name(std::string const &func) {
    auto pos = funcs_.find(func);
    if (pos != end(funcs_)) {
        throw std::logic_error(
            RPCLIB_FMT::format("Function name already bound: '{}'. "
                               "Please use unique function names", func));
    }
}

}
} /* rpc */
