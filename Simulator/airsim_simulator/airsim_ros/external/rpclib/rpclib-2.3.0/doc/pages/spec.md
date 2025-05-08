# MessagePack-RPC Protocol specification

This specification was copied from https://github.com/msgpack-rpc/msgpack-rpc/blob/master/spec.md
(without the "sales pitch" part).

The protocol consists of a request message and the corresponding response message or a notification message (without a response). The server must send a response message in reply with the request message.

## Request message

The request message is a four-element array, shown below.

```
[type, msgid, method, params]
```

### type

Must be zero (integer). Zero means that this message is the request message.

### msgid

The 32-bit unsigned integer number. This number is used as a sequence number. The server replies with a requested msgid.

### method

The string, which represents the method name to call.

### params

The array of the function arguments. The elements of this array are arbitrary objects. If the
function takes no arguments, this is an empty array (not `nil`).

## Response message

The response message is a four-element array, shown below.

```
[type, msgid, error, result]
```

### type

Must be one (integer). One means that this message is the response message.

### msgid

The 32-bit unsigned integer number. This corresponds to the msgid of the request message.

### error

If the method is executed correctly, this field should be `nil`. If the error occurred at the server-side, then this field is an arbitrary object which represents the error.

### result

An arbitrary object, which represents the returned result of the function. If an error occurred, this field should be `nil`.

## Notification message

The notification message is a three-element array, shown below.

```
[type, method, params]
```

### type

Must be two (integer). Two means that this message is a notification message.

### method

The string, which represents the method name to call.

### params

The array of the function arguments. The elements of this array are arbitrary objects. If the
function takes no arguments, this is an empty array (not `nil`).


# The order of responses

The server implementations don't have to send the responses in the order of receiveing the requests. If they receive the multiple messages, they may reply in any order.

# Client implementation details

There are some client features which a client library should implement.

## Synchronous calls

The client is blocked until the RPC is finished.

```java
Client client = new Client("localhost", 1985);
Object result = client.call("method_name", arg1, arg2, arg3);
```

## Asynchronous calls

The following figure shows how asynchronous call works.

The the call function returns a `future` object immediately.

```java
Client client = new Client("localhost", 1985);
Future future = client.asyncCall("method_name", arg1, arg2, arg3);
future.join();
Object result = future.getResult();
```

```java
Client client = new Client(...);
Future f1 = client.asyncCall("method1");
Future f2 = client.asyncCall("method2");
f1.join();
f2.join();
```

## Multiple transports

The implementation should support multiple transports like TCP, UDP, UNIX domain socket if possible.
