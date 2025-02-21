//
// ip/impl/address.ipp
// ~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASIO_IP_IMPL_ADDRESS_IPP
#define ASIO_IP_IMPL_ADDRESS_IPP

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
# pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "asio/detail/config.hpp"
#include <typeinfo>
#include "asio/detail/throw_error.hpp"
#include "asio/detail/throw_exception.hpp"
#include "asio/error.hpp"
#include "asio/ip/address.hpp"
#include "asio/system_error.hpp"

#include "asio/detail/push_options.hpp"

namespace clmdep_asio {
namespace ip {

address::address()
  : type_(ipv4),
    ipv4_address_(),
    ipv6_address_()
{
}

address::address(const clmdep_asio::ip::address_v4& ipv4_address)
  : type_(ipv4),
    ipv4_address_(ipv4_address),
    ipv6_address_()
{
}

address::address(const clmdep_asio::ip::address_v6& ipv6_address)
  : type_(ipv6),
    ipv4_address_(),
    ipv6_address_(ipv6_address)
{
}

address::address(const address& other)
  : type_(other.type_),
    ipv4_address_(other.ipv4_address_),
    ipv6_address_(other.ipv6_address_)
{
}

#if defined(ASIO_HAS_MOVE)
address::address(address&& other)
  : type_(other.type_),
    ipv4_address_(other.ipv4_address_),
    ipv6_address_(other.ipv6_address_)
{
}
#endif // defined(ASIO_HAS_MOVE)

address& address::operator=(const address& other)
{
  type_ = other.type_;
  ipv4_address_ = other.ipv4_address_;
  ipv6_address_ = other.ipv6_address_;
  return *this;
}

#if defined(ASIO_HAS_MOVE)
address& address::operator=(address&& other)
{
  type_ = other.type_;
  ipv4_address_ = other.ipv4_address_;
  ipv6_address_ = other.ipv6_address_;
  return *this;
}
#endif // defined(ASIO_HAS_MOVE)

address& address::operator=(const clmdep_asio::ip::address_v4& ipv4_address)
{
  type_ = ipv4;
  ipv4_address_ = ipv4_address;
  ipv6_address_ = clmdep_asio::ip::address_v6();
  return *this;
}

address& address::operator=(const clmdep_asio::ip::address_v6& ipv6_address)
{
  type_ = ipv6;
  ipv4_address_ = clmdep_asio::ip::address_v4();
  ipv6_address_ = ipv6_address;
  return *this;
}

clmdep_asio::ip::address_v4 address::to_v4() const
{
  if (type_ != ipv4)
  {
    std::bad_cast ex;
    clmdep_asio::detail::throw_exception(ex);
  }
  return ipv4_address_;
}

clmdep_asio::ip::address_v6 address::to_v6() const
{
  if (type_ != ipv6)
  {
    std::bad_cast ex;
    clmdep_asio::detail::throw_exception(ex);
  }
  return ipv6_address_;
}

std::string address::to_string() const
{
  if (type_ == ipv6)
    return ipv6_address_.to_string();
  return ipv4_address_.to_string();
}

std::string address::to_string(clmdep_asio::error_code& ec) const
{
  if (type_ == ipv6)
    return ipv6_address_.to_string(ec);
  return ipv4_address_.to_string(ec);
}

address address::from_string(const char* str)
{
  clmdep_asio::error_code ec;
  address addr = from_string(str, ec);
  clmdep_asio::detail::throw_error(ec);
  return addr;
}

address address::from_string(const char* str, clmdep_asio::error_code& ec)
{
  clmdep_asio::ip::address_v6 ipv6_address =
    clmdep_asio::ip::address_v6::from_string(str, ec);
  if (!ec)
  {
    address tmp;
    tmp.type_ = ipv6;
    tmp.ipv6_address_ = ipv6_address;
    return tmp;
  }

  clmdep_asio::ip::address_v4 ipv4_address =
    clmdep_asio::ip::address_v4::from_string(str, ec);
  if (!ec)
  {
    address tmp;
    tmp.type_ = ipv4;
    tmp.ipv4_address_ = ipv4_address;
    return tmp;
  }

  return address();
}

address address::from_string(const std::string& str)
{
  return from_string(str.c_str());
}

address address::from_string(const std::string& str,
    clmdep_asio::error_code& ec)
{
  return from_string(str.c_str(), ec);
}

bool address::is_loopback() const
{
  return (type_ == ipv4)
    ? ipv4_address_.is_loopback()
    : ipv6_address_.is_loopback();
}

bool address::is_unspecified() const
{
  return (type_ == ipv4)
    ? ipv4_address_.is_unspecified()
    : ipv6_address_.is_unspecified();
}

bool address::is_multicast() const
{
  return (type_ == ipv4)
    ? ipv4_address_.is_multicast()
    : ipv6_address_.is_multicast();
}

bool operator==(const address& a1, const address& a2)
{
  if (a1.type_ != a2.type_)
    return false;
  if (a1.type_ == address::ipv6)
    return a1.ipv6_address_ == a2.ipv6_address_;
  return a1.ipv4_address_ == a2.ipv4_address_;
}

bool operator<(const address& a1, const address& a2)
{
  if (a1.type_ < a2.type_)
    return true;
  if (a1.type_ > a2.type_)
    return false;
  if (a1.type_ == address::ipv6)
    return a1.ipv6_address_ < a2.ipv6_address_;
  return a1.ipv4_address_ < a2.ipv4_address_;
}

} // namespace ip
} // namespace clmdep_asio

#include "asio/detail/pop_options.hpp"

#endif // ASIO_IP_IMPL_ADDRESS_IPP
