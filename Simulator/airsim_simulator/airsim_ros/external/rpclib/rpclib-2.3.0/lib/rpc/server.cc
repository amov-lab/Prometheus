#include "rpc/server.h"

#include <atomic>
#include <memory>
#include <stdexcept>
#include <stdint.h>
#include <thread>

#include "asio.hpp"
#include "format.h"

#include "rpc/this_server.h"
#include "rpc/detail/dev_utils.h"
#include "rpc/detail/log.h"
#include "rpc/detail/log.h"
#include "rpc/detail/server_session.h"
#include "rpc/detail/thread_group.h"

using namespace rpc::detail;
using RPCLIB_ASIO::ip::tcp;
using namespace RPCLIB_ASIO;


namespace rpc {

struct server::impl {
    impl(server *parent, std::string const &address, uint16_t port)
        : parent_(parent),
          io_(),
          acceptor_(io_),
          socket_(io_),
          suppress_exceptions_(false) {
            auto ep = tcp::endpoint(ip::address::from_string(address), port);
            acceptor_.open(ep.protocol());
            acceptor_.set_option(tcp::acceptor::reuse_address(true));
            acceptor_.bind(ep);
            acceptor_.listen();
          }

    impl(server *parent, uint16_t port)
        : parent_(parent),
          io_(),
          acceptor_(io_),
          socket_(io_),
          suppress_exceptions_(false) {
            auto ep = tcp::endpoint(tcp::v4(), port);
            acceptor_.open(ep.protocol());
            acceptor_.set_option(tcp::acceptor::reuse_address(true));
            acceptor_.bind(ep);
            acceptor_.listen();            
          }

    void start_accept() {
        acceptor_.async_accept(socket_, [this](std::error_code ec) {
            if (!ec) {
                LOG_INFO("Accepted connection.");
                auto s = std::make_shared<server_session>(
                    parent_, &io_, std::move(socket_), parent_->disp_,
                    suppress_exceptions_);
                s->start();
                std::unique_lock<std::mutex> lock(sessions_mutex_);
                sessions_.push_back(s);
            } else {
                LOG_ERROR("Error while accepting connection: {}", ec);
            }
            if (!this_server().stopping())
                start_accept();
            // TODO: allow graceful exit [sztomi 2016-01-13]
        });
    }

    void close_sessions() {
        std::unique_lock<std::mutex> lock(sessions_mutex_);
        auto sessions_copy = sessions_;
        sessions_.clear();
        lock.unlock();

        // release shared pointers outside of the mutex
        for (auto &session : sessions_copy) {
            session->close();
        }

        if (this_server().stopping())
            acceptor_.cancel();
    }

    void stop() {
        io_.stop();
        loop_workers_.join_all();
    }

    unsigned short port() const {
        return acceptor_.local_endpoint().port();        
    }

    server *parent_;
    io_service io_;
    ip::tcp::acceptor acceptor_;
    ip::tcp::socket socket_;
    rpc::detail::thread_group loop_workers_;
    std::vector<std::shared_ptr<server_session>> sessions_;
    std::atomic_bool suppress_exceptions_;
    RPCLIB_CREATE_LOG_CHANNEL(server)
    std::mutex sessions_mutex_;
};

RPCLIB_CREATE_LOG_CHANNEL(server)

server::server(uint16_t port)
    : pimpl(new server::impl(this, port)), disp_(std::make_shared<dispatcher>()) {
    LOG_INFO("Created server on localhost:{}", port);
    pimpl->start_accept();
}

server::server(server&& other) noexcept {
    *this = std::move(other);
}

server::server(std::string const &address, uint16_t port)
    : pimpl(new server::impl(this, address, port)),
    disp_(std::make_shared<dispatcher>()) {
    LOG_INFO("Created server on address {}:{}", address, port);
    pimpl->start_accept();
}

server::~server() {
    if (pimpl) {
        pimpl->stop();
    }
}

server& server::operator=(server &&other) {
    if (this != &other) {
        pimpl = std::move(other.pimpl);
        other.pimpl = nullptr;
        disp_ = std::move(other.disp_);
        other.disp_ = nullptr;
    }
    return *this;
}

void server::suppress_exceptions(bool suppress) {
    pimpl->suppress_exceptions_ = suppress;
}

void server::run() { pimpl->io_.run(); }

void server::async_run(std::size_t worker_threads) {
    pimpl->loop_workers_.create_threads(worker_threads, [this]() {
        name_thread("server");
        LOG_INFO("Starting");
        pimpl->io_.run();
        LOG_INFO("Exiting");
    });
}

void server::stop() { pimpl->stop(); }

unsigned short server::port() const { return pimpl->port(); }

void server::close_sessions() { pimpl->close_sessions(); }

void server::close_session(std::shared_ptr<detail::server_session> const &s) {
  std::unique_lock<std::mutex> lock(pimpl->sessions_mutex_);
  auto it = std::find(begin(pimpl->sessions_), end(pimpl->sessions_), s);
  std::shared_ptr<server_session> session;
  if (it != end(pimpl->sessions_)) {
    session = *it;
    pimpl->sessions_.erase(it);
  }
  lock.unlock();
  // session shared pointer is released outside of the mutex
}

} /* rpc */
