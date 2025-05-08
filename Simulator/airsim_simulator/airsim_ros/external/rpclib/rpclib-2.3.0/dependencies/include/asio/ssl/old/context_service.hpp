//
// ssl/old/context_service.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2005 Voipster / Indrek dot Juhani at voipster dot com
// Copyright (c) 2005-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_SSL_OLD_CONTEXT_SERVICE_HPP
#define ASIO_SSL_OLD_CONTEXT_SERVICE_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"
#include <string>
#include <boost/noncopyable.hpp>
#include "asio/error.hpp"
#include "asio/io_service.hpp"
#include "asio/ssl/context_base.hpp"
#include "asio/ssl/old/detail/openssl_context_service.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace ssl {
namespace old {

/// Default service implementation for a context.
class context_service
#if defined(GENERATING_DOCUMENTATION)
  : public clmdep_asio::io_service::service
#else
  : public clmdep_asio::detail::service_base<context_service>
#endif
{
private:
  // The type of the platform-specific implementation.
  typedef old::detail::openssl_context_service service_impl_type;

public:
#if defined(GENERATING_DOCUMENTATION)
  /// The unique service identifier.
  static clmdep_asio::io_service::id id;
#endif

  /// The type of the context.
#if defined(GENERATING_DOCUMENTATION)
  typedef implementation_defined impl_type;
#else
  typedef service_impl_type::impl_type impl_type;
#endif

  /// Constructor.
  explicit context_service(clmdep_asio::io_service& io_service)
    : clmdep_asio::detail::service_base<context_service>(io_service),
      service_impl_(clmdep_asio::use_service<service_impl_type>(io_service))
  {
  }

  /// Return a null context implementation.
  impl_type null() const
  {
    return service_impl_.null();
  }

  /// Create a new context implementation.
  void create(impl_type& impl, context_base::method m)
  {
    service_impl_.create(impl, m);
  }

  /// Destroy a context implementation.
  void destroy(impl_type& impl)
  {
    service_impl_.destroy(impl);
  }

  /// Set options on the context.
  clmdep_asio::error_code set_options(impl_type& impl,
      context_base::options o, clmdep_asio::error_code& ec)
  {
    return service_impl_.set_options(impl, o, ec);
  }

  /// Set peer verification mode.
  clmdep_asio::error_code set_verify_mode(impl_type& impl,
      context_base::verify_mode v, clmdep_asio::error_code& ec)
  {
    return service_impl_.set_verify_mode(impl, v, ec);
  }

  /// Load a certification authority file for performing verification.
  clmdep_asio::error_code load_verify_file(impl_type& impl,
      const std::string& filename, clmdep_asio::error_code& ec)
  {
    return service_impl_.load_verify_file(impl, filename, ec);
  }

  /// Add a directory containing certification authority files to be used for
  /// performing verification.
  clmdep_asio::error_code add_verify_path(impl_type& impl,
      const std::string& path, clmdep_asio::error_code& ec)
  {
    return service_impl_.add_verify_path(impl, path, ec);
  }

  /// Use a certificate from a file.
  clmdep_asio::error_code use_certificate_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      clmdep_asio::error_code& ec)
  {
    return service_impl_.use_certificate_file(impl, filename, format, ec);
  }

  /// Use a certificate chain from a file.
  clmdep_asio::error_code use_certificate_chain_file(impl_type& impl,
      const std::string& filename, clmdep_asio::error_code& ec)
  {
    return service_impl_.use_certificate_chain_file(impl, filename, ec);
  }

  /// Use a private key from a file.
  clmdep_asio::error_code use_private_key_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      clmdep_asio::error_code& ec)
  {
    return service_impl_.use_private_key_file(impl, filename, format, ec);
  }

  /// Use an RSA private key from a file.
  clmdep_asio::error_code use_rsa_private_key_file(impl_type& impl,
      const std::string& filename, context_base::file_format format,
      clmdep_asio::error_code& ec)
  {
    return service_impl_.use_rsa_private_key_file(impl, filename, format, ec);
  }

  /// Use the specified file to obtain the temporary Diffie-Hellman parameters.
  clmdep_asio::error_code use_tmp_dh_file(impl_type& impl,
      const std::string& filename, clmdep_asio::error_code& ec)
  {
    return service_impl_.use_tmp_dh_file(impl, filename, ec);
  }

  /// Set the password callback.
  template <typename PasswordCallback>
  clmdep_asio::error_code set_password_callback(impl_type& impl,
      PasswordCallback callback, clmdep_asio::error_code& ec)
  {
    return service_impl_.set_password_callback(impl, callback, ec);
  }

private:
  // Destroy all user-defined handler objects owned by the service.
  void shutdown_service()
  {
  }

  // The service that provides the platform-specific implementation.
  service_impl_type& service_impl_;
};

} // namespace old
} // namespace ssl
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_SSL_OLD_CONTEXT_SERVICE_HPP
