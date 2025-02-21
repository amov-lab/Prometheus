
namespace rpc {
namespace detail {

template <typename F> void dispatcher::bind(std::string const &name, F func) {
    enforce_unique_name(name);
    bind(name, func, typename detail::func_kind_info<F>::result_kind(),
         typename detail::func_kind_info<F>::args_kind());
}

template <typename F>
void dispatcher::bind(std::string const &name, F func,
                      detail::tags::void_result const &,
                      detail::tags::zero_arg const &) {
    enforce_unique_name(name);
    funcs_.insert(
        std::make_pair(name, [func, name](RPCLIB_MSGPACK::object const &args) {
            enforce_arg_count(name, 0, args.via.array.size);
            func();
            return rpc::detail::make_unique<RPCLIB_MSGPACK::object_handle>();
        }));
}

template <typename F>
void dispatcher::bind(std::string const &name, F func,
                      detail::tags::void_result const &,
                      detail::tags::nonzero_arg const &) {
    using detail::func_traits;
    using args_type = typename func_traits<F>::args_type;

    enforce_unique_name(name);
    funcs_.insert(
        std::make_pair(name, [func, name](RPCLIB_MSGPACK::object const &args) {
            constexpr int args_count = std::tuple_size<args_type>::value;
            enforce_arg_count(name, args_count, args.via.array.size);
            args_type args_real;
            args.convert(args_real);
            detail::call(func, args_real);
            return rpc::detail::make_unique<RPCLIB_MSGPACK::object_handle>();
        }));
}

template <typename F>
void dispatcher::bind(std::string const &name, F func,
                      detail::tags::nonvoid_result const &,
                      detail::tags::zero_arg const &) {
    using detail::func_traits;

    enforce_unique_name(name);
    funcs_.insert(std::make_pair(name, [func,
                                        name](RPCLIB_MSGPACK::object const &args) {
        enforce_arg_count(name, 0, args.via.array.size);
        auto z = rpc::detail::make_unique<RPCLIB_MSGPACK::zone>();
        auto result = RPCLIB_MSGPACK::object(func(), *z);
        return rpc::detail::make_unique<RPCLIB_MSGPACK::object_handle>(result, std::move(z));
    }));
}

template <typename F>
void dispatcher::bind(std::string const &name, F func,
                      detail::tags::nonvoid_result const &,
                      detail::tags::nonzero_arg const &) {
    using detail::func_traits;
    using args_type = typename func_traits<F>::args_type;

    enforce_unique_name(name);
    funcs_.insert(std::make_pair(name, [func,
                                        name](RPCLIB_MSGPACK::object const &args) {
        constexpr int args_count = std::tuple_size<args_type>::value;
        enforce_arg_count(name, args_count, args.via.array.size);
        args_type args_real;
        args.convert(args_real);
        auto z = rpc::detail::make_unique<RPCLIB_MSGPACK::zone>();
        auto result = RPCLIB_MSGPACK::object(detail::call(func, args_real), *z);
        return rpc::detail::make_unique<RPCLIB_MSGPACK::object_handle>(result, std::move(z));
    }));
}
}
}
