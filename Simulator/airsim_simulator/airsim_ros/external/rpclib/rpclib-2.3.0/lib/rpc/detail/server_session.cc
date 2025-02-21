#include "rpc/detail/server_session.h"

#include "rpc/config.h"
#include "rpc/server.h"
#include "rpc/this_handler.h"
#include "rpc/this_server.h"
#include "rpc/this_session.h"

#include "rpc/detail/log.h"

namespace rpc {
namespace detail {

static constexpr std::size_t default_buffer_size =
    rpc::constants::DEFAULT_BUFFER_SIZE;

server_session::server_session(server *srv, RPCLIB_ASIO::io_service *io,
                               RPCLIB_ASIO::ip::tcp::socket socket,
                               std::shared_ptr<dispatcher> disp,
                               bool suppress_exceptions)
    : async_writer(io, std::move(socket)),
      parent_(srv),
      io_(io),
      read_strand_(*io),
      disp_(disp),
      pac_(),
      suppress_exceptions_(suppress_exceptions) {
    pac_.reserve_buffer(default_buffer_size); // TODO: make this configurable
                                              // [sztomi 2016-01-13]
}

void server_session::start() { do_read(); }

void server_session::close() {
    LOG_INFO("Closing session.");
    async_writer::close();

    auto self(shared_from_base<server_session>());
    write_strand().post([this, self]() {
        parent_->close_session(self);
    });
}

void server_session::do_read() {
    auto self(shared_from_base<server_session>());
    constexpr std::size_t max_read_bytes = default_buffer_size;
    socket().async_read_some(
        RPCLIB_ASIO::buffer(pac_.buffer(), default_buffer_size),
        // I don't think max_read_bytes needs to be captured explicitly
        // (since it's constexpr), but MSVC insists.
        read_strand_.wrap([this, self, max_read_bytes](std::error_code ec,
                                                       std::size_t length) {
            if (is_closed()) { return; }
            if (!ec) {
                pac_.buffer_consumed(length);
                RPCLIB_MSGPACK::unpacked result;
                while (pac_.next(result) && !is_closed()) {
                    auto msg = result.get();
                    output_buf_.clear();

                    // any worker thread can take this call
                    auto z = std::shared_ptr<RPCLIB_MSGPACK::zone>(
                        result.zone().release());
                    io_->post([this, self, msg, z]() {
                        this_handler().clear();
                        this_session().clear();
                        this_session().set_id(reinterpret_cast<session_id_t>(this));
                        this_server().cancel_stop();

                        auto resp = disp_->dispatch(msg, suppress_exceptions_);

                        // There are various things that decide what to send
                        // as a response. They have a precedence.

                        // First, if the response is disabled, that wins
                        // So You Get Nothing, You Lose! Good Day Sir!
                        if (!this_handler().resp_enabled_) {
                            return;
                        }

                        // Second, if there is an error set, we send that
                        // and only third, if there is a special response, we
                        // use it
                        if (!this_handler().error_.get().is_nil()) {
                            LOG_WARN("There was an error set in the handler");
                            resp.capture_error(this_handler().error_);
                        } else if (!this_handler().resp_.get().is_nil()) {
                            LOG_WARN("There was a special result set in the "
                                     "handler");
                            resp.capture_result(this_handler().resp_);
                        }

                        if (!resp.is_empty()) {
#ifdef _MSC_VER
                            // doesn't compile otherwise.
                            write_strand().post(
                                [=]() { write(resp.get_data()); });
#else
                            write_strand().post(
                                [this, self, resp, z]() { write(resp.get_data()); });
#endif
                        }

                        if (this_session().exit_) {
                            LOG_WARN("Session exit requested from a handler.");
                            // posting through the strand so this comes after
                            // the previous write
                            write_strand().post([this]() { close(); });
                        }

                        if (this_server().stopping()) {
                            LOG_WARN("Server exit requested from a handler.");
                            // posting through the strand so this comes after
                            // the previous write
                            write_strand().post(
                                [this]() { parent_->close_sessions(); });
                        }
                    });
                }

                if (!is_closed()) {
                    // resizing strategy: if the remaining buffer size is
                    // less than the maximum bytes requested from asio,
                    // then request max_read_bytes. This prompts the unpacker
                    // to resize its buffer doubling its size
                    // (https://github.com/msgpack/msgpack-c/issues/567#issuecomment-280810018)
                    if (pac_.buffer_capacity() < max_read_bytes) {
                        LOG_TRACE("Reserving extra buffer: {}", max_read_bytes);
                        pac_.reserve_buffer(max_read_bytes);
                    }
                    do_read();
                }
            } else if (ec == RPCLIB_ASIO::error::eof ||
                       ec == RPCLIB_ASIO::error::connection_reset) {
                LOG_INFO("Client disconnected");
                self->close();
            } else {
                LOG_ERROR("Unhandled error code: {} | '{}'", ec, ec.message());
            }
        }));
}

} /* detail */
} /* rpc */
