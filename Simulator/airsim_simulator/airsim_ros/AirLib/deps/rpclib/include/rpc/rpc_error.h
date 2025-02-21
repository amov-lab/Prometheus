#pragma once

#ifndef RPC_ERROR_H_NEOOSTKY
#define RPC_ERROR_H_NEOOSTKY

#include <exception>
#include <system_error>

#include "rpc/msgpack.hpp"

namespace rpc {

//! \brief This exception is thrown by the client when the server signals an
//! error during a call.
//!
//! This type allows clients to handle arbitrary error objects as the
//! msgpack-rpc specification allows. In client code you probably don't want to
//! throw it, hence its constructor is private.
class rpc_error : public std::runtime_error {
public:
    rpc_error(std::string const &what_arg, std::string const &function_name,
              std::shared_ptr<RPCLIB_MSGPACK::object_handle> o);

    //! \brief Returns the name of the function that was
    //! called on the server while the error occurred.
    std::string get_function_name() const;

    //! \brief Returns the error object that the server
    //! provided.
    virtual RPCLIB_MSGPACK::object_handle &get_error();

private:
    std::string func_name_;
    std::shared_ptr<RPCLIB_MSGPACK::object_handle> ob_h_;
};

//! \brief This exception is thrown by the client when either the connection
//! or a call takes more time than it is set in set_timeout.
//! \note There isn't necessarily a timeout set, it is an optional value.
class timeout : public std::runtime_error {
public:
    explicit timeout(std::string const &what_arg);

    //! \brief Describes the exception.
    const char *what() const noexcept override;

private:
    std::string formatted;
};

//! \brief This exception is throw by the client when the connection or call
//! causes a system error
class system_error : public std::system_error {
public:
    using std::system_error::system_error;
    const char* what() const noexcept;
};

} /* rpc */


#endif /* end of include guard: RPC_ERROR_H_NEOOSTKY */
