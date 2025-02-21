#pragma once

#ifndef ASYNC_WRITER_H_HQIRH28I
#define ASYNC_WRITER_H_HQIRH28I

#include "asio.hpp"
#include "rpc/msgpack.hpp"
#include <condition_variable>
#include <deque>
#include <memory>
#include <thread>

namespace rpc {

class client;

namespace detail {

//! \brief Common logic for classes that have a write queue with async writing.
class async_writer : public std::enable_shared_from_this<async_writer> {
public:
    async_writer(RPCLIB_ASIO::io_service *io,
                 RPCLIB_ASIO::ip::tcp::socket socket)
        : socket_(std::move(socket)), write_strand_(*io), exit_(false) {}

    void close() {
        exit_ = true;

        auto self = shared_from_this();
        write_strand_.post([this, self]() {
            LOG_INFO("Closing socket");
            std::error_code e;
            socket_.shutdown(
                RPCLIB_ASIO::ip::tcp::socket::shutdown_both, e);
            if (e) {
                LOG_WARN("std::system_error during socket shutdown. "
                            "Code: {}. Message: {}", e.value(), e.message());
            }
            socket_.close();
        });
    }

    bool is_closed() const {
        return exit_.load();
    }

    void do_write() {
        if (exit_) {
            return;
        }
        auto self(shared_from_this());
        auto &item = write_queue_.front();
        // the data in item remains valid until the handler is called
        // since it will still be in the queue physically until then.
        RPCLIB_ASIO::async_write(
            socket_, RPCLIB_ASIO::buffer(item.data(), item.size()),
            write_strand_.wrap(
                [this, self](std::error_code ec, std::size_t transferred) {
                    (void)transferred;
                    if (!ec) {
                        write_queue_.pop_front();
                        if (write_queue_.size() > 0) {
                            if (!exit_) {
                                do_write();
                            }
                        }
                    } else {
                        LOG_ERROR("Error while writing to socket: {}", ec);
                    }
                }));
    }

    void write(RPCLIB_MSGPACK::sbuffer &&data) {
        write_queue_.push_back(std::move(data));
        if (write_queue_.size() > 1) {
            return; // there is an ongoing write chain so don't start another
        }

        do_write();
    }

    RPCLIB_ASIO::ip::tcp::socket& socket() {
        return socket_;
    }

protected:
    template <typename Derived>
    std::shared_ptr<Derived> shared_from_base() {
        return std::static_pointer_cast<Derived>(shared_from_this());
    }

    RPCLIB_ASIO::strand& write_strand() {
        return write_strand_;
    }

private:
    RPCLIB_ASIO::ip::tcp::socket socket_;
    RPCLIB_ASIO::strand write_strand_;
    std::atomic_bool exit_{false};
    std::deque<RPCLIB_MSGPACK::sbuffer> write_queue_;
    RPCLIB_CREATE_LOG_CHANNEL(async_writer)
};

} /* detail */
} /* rpc  */

#endif /* end of include guard: ASYNC_WRITER_H_HQIRH28I */
