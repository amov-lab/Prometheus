//
// ip/impl/basic_endpoint.hpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_IP_IMPL_BASIC_ENDPOINT_HPP
#define ASIO_IP_IMPL_BASIC_ENDPOINT_HPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#if !defined(ASIO_NO_IOSTREAM)

#include "asio/detail/throw_error.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace ip {

template <typename Elem, typename Traits, typename InternetProtocol>
std::basic_ostream<Elem, Traits>& operator<<(
    std::basic_ostream<Elem, Traits>& os,
    const basic_endpoint<InternetProtocol>& endpoint)
{
  clmdep_asio::ip::detail::endpoint tmp_ep(endpoint.address(), endpoint.port());
  clmdep_asio::error_code ec;
  std::string s = tmp_ep.to_string(ec);
  if (ec)
  {
    if (os.exceptions() & std::basic_ostream<Elem, Traits>::failbit)
      clmdep_asio::detail::throw_error(ec);
    else
      os.setstate(std::basic_ostream<Elem, Traits>::failbit);
  }
  else
    for (std::string::iterator i = s.begin(); i != s.end(); ++i)
      os << os.widen(*i);
  return os;
}

} // namespace ip
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // !defined(ASIO_NO_IOSTREAM)

#endif // ASIO_IP_IMPL_BASIC_ENDPOINT_HPP
