This chapter is about features that are planned in the upcoming versions of `rpclib`. Keep in mind
that this serves merely as a draft of current ideas; something being here is not guaranteed to ever
be implmented; and anything that's *not* here can be implemented at any time.

## Support different transports

The library is currently TCP-only. It should be possible to use domain pipes/windows pipes which
are useful for fast IPC. UDP can be added as well, although I'm not sure how conformant that would
be with msgpack-rpc. Implementing transports might be exposed to the users of the library as well.

## Support different protocols

There are protocols that are structured very similarly to `msgpack-rpc`, such as `json-rpc`.
Considering this, it won't be terribly hard to refactor the wrapper-generation to allow
consuming and producing different protocols.

### Support framed-msgpack-rpc

nodejs uses a variation of msgpack-rpc, called framed-msgpack-rpc. It's the same protocol, but the
messages are delimited with their sizes which allows avoiding reallocations for large messages.

## Code generator

Using libclang, it's fairly easy to parse a C++ header file. With attributes provided by the
library, it should be possible to mark a class for RPC exposure and the code generator should take
care of creating a specialized server for the class (or client for an interface).

### Support IDLs of other RPC libraries

Apache Thrift, gRPC, etc. have IDLs that describe a service. `rpclib` should provide tools to parse
these IDLs to allow using `rpclib` as a drop-in replacement.
