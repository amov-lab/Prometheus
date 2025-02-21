Welcome to the Primer! This document is a tutorial introduction to `rpclib` for absolute beginners. If you are new to the library and prefer detailed instructions and explanation, you are in the right place. If short examples with less explanation work better for you, you might want to check out the [Cookbook](cookbook.md)!

The tutorial is sturctured as follows: in the first part, writing servers is explained with one simple and one more advanced example.  In the second part, the corresponding clients are implemented.

## Prerequisites

Knowledge-wise, this tutorial assumes that you have an intermediate grasp of C++ and that you have an idea of what [RPC](https://en.wikipedia.org/wiki/Remote_procedure_call) (Remote Procedure Call) is.

For your build environment, make sure that you are able to compile and link a program with
`rpclib`. The [Getting Started](gettingstarted.md) page can help you with that.

## Introduction

`rpclib` is a RPC library that provides both a _client_ and a _server_ implementation. The _server_ allows you to expose functions of your program to be called remotely, while the _client_ allows you to call functions of servers. You can use the `rpclib` client and server in tandem (even in the same program, if you want to), but it's not a requirement.

As other RPC libraries, `rpclib` is a good candidate for inter-process communication. Also, there exist many implementations of the protocol in a large amount of languages, which makes it a possible inter-language communication bridge.

msgpack-RPC is the protocol that `rpclib` uses for dispatching and encoding calls to functions. The protocol is based on [msgpack](http://msgpack.org), a fast and compact format. For details on how exactly it is structured, see the [Specification](spec.md) chapter.

# Writing servers

In the first part of this tutorial, we will learn about writing server applications. Two example
applications will be implemented step-by-step.

## Calculator, the "Hello World" of RPC libraries.

Our first server application will expose four functions: `add`, `subtract`, `multiply`, `divide`. For the sake of this example, the functions are implemented as various callable entities.

``` cpp
#include "rpc/server.h"

double divide(double a, double b) { return a / b; }

struct subtractor {
    double operator()(double a, double b) { return a - b; }
};

struct multiplier {
    double multiply(double a, double b) { return a * b; }
};

int main() {
    subtractor s;
    multiplier m;
    auto add = [](double a, double b) { return a + b; };

    // ...

    return 0;
}

```

Now, let's create the server:

```cpp
rpc::server srv(8080);
```

This server will listen on port 8080 (but not right away after construction - we need to `run` it). Next, we _bind_ the functors to names in order to expose them:

```cpp
srv.bind("add", [](double a, double b) { return a + b; });
srv.bind("sub", s);
srv.bind("div", &divide);
srv.bind("mul", [&m](double a, double b) { return m.multiply(a, b); });
```

These are the names that the client can use to call our functions. There is nothing stopping you from binding `divide` to the name `"add"`, but it's a good practice to use names that reflect the source names. It is also possible to bind the same function to multiple names.

!!! info
    Under the hood, each `bind` statement generates a compile-time a wrapper function that takes a `msgpack` object, then decodes it into the real parameters of the bound function (if any) and calls the bound function. If the function has a return value the wrapper is generated so that it encodes the result as a `msgpack` object which the server can send to the client in a response. More information on this mechanism can be found in the [Internals](internals.md) chapter.

After we exposed the function, we need to `run` the server:

```cpp
srv.run();
```

`run` is a blocking function, but it also has a non-blocking pair called `async_run`. When `run` is called, the server starts listening on the port we assigned to it in its constructor, and its internal event loop will start processing the incoming requests.

This is now a functioning (although, in some ways, incomplete) server. The complete listing so far:

```cpp
#include "rpc/server.h"

double divide(double a, double b) { return a / b; }

struct subtractor {
    double operator()(double a, double b) { return a - b; }
};

struct multiplier {
    double multiply(double a, double b) { return a * b; }
};

int main() {
    rpc::server srv(8080);

    subtractor s;
    multiplier m;

    srv.bind("add", [](double a, double b) { return a + b; });
    srv.bind("sub", s);
    srv.bind("div", &divide);
    srv.bind("mul", [&m](double a, double b) { return m.multiply(a, b); });

    srv.run();

    return 0;
}
```

If you want, you can fire up a [quick python script](https://github.com/msgpack-rpc/msgpack-rpc-python) to test it (don't worry, we'll write a client with `rpclib`, too).

## Responding with errors

There is, however, an issue with this server. Did you spot it? Any client can easily make it crash just by calling `div` with a 0 divider (causing division by zero). What can we do about this? Well of course, we can just check the divider and _not_ perform the division. We still need to return _something_ though:

```cpp
#include "rpc/server.h"

double divide(double a, double b) {
    if (b == 0) {
        return 0.0; // <- ugh, that's not very good :S
    }
    return a / b;
}
```

This is enough to avoid the crash, but it's fundamentally broken: division by zero does not yield zero, after all. The client gets an arbitrary result (oblivious to the fact that there was an error), which is most likely not what they want.

Luckily, the msgpack-rpc protocol supports error signaling. We need to modify our `divide` function a little bit to utilize this functionality:


```cpp
#include "rpc/server.h"
#include "rpc/this_handler.h"

double divide(double a, double b) {
    if (b == 0) {
        rpc::this_handler().respond_error("Division by zero");
    }
    return a / b;
}
```

This is better. The error is [stringly-typed](http://c2.com/cgi/wiki?StringlyTyped), but it's
better than an arbitrary result. To amend this, we can return practically any object, such as
a tuple that contains an error code besides the message:

```cpp
double divide(double a, double b) {
    if (b == 0.0) {
        rpc::this_handler().respond_error(
                std::make_tuple(1, "Division by zero"));
    }
    return a / b;
}
```

!!! info
    `msgpack-rpc` does not define the structure of error objects, so making up something like this is
    perfectly fine (in fact, I'd encourage you to do this with well-defined error codes). Consider
    it part of your server interface and document accordingly.

You might be puzzled about why we are not returning after setting the error. The reason for this is that `respond_error` throws an internal exception that is handled inside the library. (_This can be considered an implementation detail, but it's good to know what happens here (and it's unlikely to change_).

Now, with the added error handling, our server is bullet-proof. Or is it?

### What about _my_ exceptions?

Our little calculator server is pretty stable at this point, but real-world applications often have to deal with exceptions. In general, exceptions should be handled at the library users' discretion (that is, caught on the handler level). So by default, `rpclib` doesn't do anything with them. If an exception leaves the handler, it is an unhandled exception and your server will crash. Yet, there are cases when you can't or don't want to handle exceptions in the handler. To facilitate this, `rpclib` provides a way to automatically turn exceptions into RPC errors:

```cpp
srv.suppress_exceptions(true);
```

With this, you can call functions that throw or throw exceptions:

```cpp
double divide(double a, double b) {
    if (b == 0) {
        rpc::this_handler().respond_error(
                std::make_tuple(1, "Division by zero"));
    }
    else if (b == 1) {
        throw std::runtime_error("Come on!");
    }
    throw std::logic_error("What am I doing here?");
}
```

So yes, this means that if you set `suppress_excpetions` to `true`, you might as well signal errors from handlers by throwing exceptions. Be advised that `respond_error` is still valid and remains the preferred way to do so (especially that it's the only way to respond with structured error objects).

What exactly happens to the suppressed exception? `rpclib` will try to catch `std::exceptions` and use their `what()` members to get a string representation which it sets as an error.

What if you throw something that is not a `std::exception`-descendant? First of all, shame on you. Second, `rpclib` will send an error message letting your clients know that you threw something that is not a `std::exception` (*shaming you in front of your clients*). Don't do this, really.

## A more complicated server - Parallel mandelbrot-generation

The following example demonstrates parallel processing and binding custom data types. The server itself will have two functions: one for getting the current date and time, and one for getting a rendering of the mandelbrot set. The two functions can be called asynchronously by a client.

### Using custom types as parameters

Anything that msgpack can process can be used as a parameter or return value for a bound function. In order to teach msgpack about your custom types, you need to use the `MSGPACK_DEFINE_ARRAY` or `MSGPACK_DEFINE_MAP` macros.

!!! info
    The difference between the two macros is that the array only contains the data values after each other, while the map also contains the names of the values. The latter gives more flexibility, the former is more compact.

In our mandelbrot example, we will want to send pixel data to the clients, so let's define a struct:

```cpp
struct pixel {
    unsigned char r, g, b;
    MSGPACK_DEFINE_ARRAY(r, g, b)
};

using pixel_data = std::vector<pixel>;
```

We will share this definition between the client and server, so for our purposes it's best to put it in a common header.

Like in the first example, we create the server and bind the functions we expose. This time we are using lambdas as the bound functions.

```cpp
rpc::server srv(8080);

srv.bind("get_time", []() {
    time_t rawtime;
    struct tm *timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    return asctime(timeinfo);
});

srv.bind("get_mandelbrot", [&](int width, int height) {
    pixel_data data;
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            // ...
        }
    }

    return data;
});
```

The exact contents of these functions is not a concern for our purposes, just note that the `get_time` returns a value very quickly, while `get_mandelbrot` computes a large array of numbers for several seconds.

## Running the server asynchrously and utilizing workers

In the first example, we called the blocking `run` function of the server to start it. Here, we are going to use `async_run`. There are two important differences.

. `run` blocks, `async_run` returns after starting the server.
. `async_run` supports spawning worker threads for executing the bound functions.

In this example, we call it like this:

```cpp
srv.async_run(2);
```

This will spawn two worker threads in the server (so now there are three in the program, because the main thread already exists). The threads will wait until there is work to do.

!!! info
    "Work" is not only executing handlers. Processing network I/O is also part of the work that threads can take. You don't need an extra thread per connection though, because processing the I/O is typically not very processor-intensive.

Now this server can take a call to `get_mandelbrot`, start executing it and in the meantime it can finish multiple `get_time` calls. The handlers are only executed by these worker threads, the main thread is free to continue.

# Writing clients

Creating msgpack-rpc clients with `rpclib` happens very similarly to servers. Mirroring the server examples above, we will implement their corresponding clients.

## The Calculator client

The `client` object is instantiated like this:

```cpp
rpc::client client("127.0.0.1", 8080);
```

The important difference, compared to a server, is that we also need to specify the host to connect to.

Another difference is that the client tries to connect to the server right away during construction (but the construction of the client is not a blocking call). The client object can be used right away:

```cpp
double five = c.call("add", 2, 3).as<double>();
```

Let's try and call all of the exposed functions. With a little bit of logging to the standard
output, the complete listing looks like this so far:

```cpp
#include <iostream>

#include "rpc/client.h"

int main() {
    rpc::client c("localhost", 8080);

    std::cout << "add(2, 3) = ";
    double five = c.call("add", 2, 3).as<double>();
    std::cout << five << std::endl;

    std::cout << "sub(3, 2) = ";
    double one = c.call("sub", 3, 2).as<double>();
    std::cout << one << std::endl;

    std::cout << "mul(5, 0) = ";
    double zero = c.call("mul", five, 0).as<double>();
    std::cout << zero << std::endl;

    std::cout << "div(3, 0) = ";
    double hmm = c.call("div", 3, 0).as<double>();
    std::cout << hmm << std::endl;

    return 0;
}
```

## Error handling

Any request that a client makes might potentially receive an error response. In the calculator server, we decided to respond with a `tuple` containing an error code and a message. `rpclib` allows you to handle these error objects by catching `rpc::rpc_error` exceptions. To handle the errors the server throws, we would wrap the calls like this:

```cpp
try {
    std::cout << "add(2, 3) = ";
    double five = c.call("add", 2, 3).as<double>();
    std::cout << five << std::endl;

    std::cout << "sub(3, 2) = ";
    double one = c.call("sub", 3, 2).as<double>();
    std::cout << one << std::endl;

    std::cout << "mul(5, 0) = ";
    double zero = c.call("mul", five, 0).as<double>();
    std::cout << zero << std::endl;

    std::cout << "div(3, 0) = ";
    double hmm = c.call("div", 3, 0).as<double>();
    std::cout << hmm << std::endl;
} catch (rpc::rpc_error &e) {
    std::cout << std::endl << e.what() << std::endl;
    std::cout << "in function " << e.get_function_name() << ": ";

    using err_t = std::tuple<int, std::string>;
    auto err = e.get_error().as<err_t>();
    std::cout << "[error " << std::get<0>(err) << "]: " << std::get<1>(err)
              << std::endl;
    return 1;
}
```
As you would expect, the output looks like this:

```
add(2, 3) = 5
sub(3, 2) = 1
mul(5, 0) = 0
div(3, 0) =
rpclib: a handler responded with an error
in function 'div': [error 1]: Division by zero
```

That's pretty much all we need for the calculator client.

### The anatomy of a `call`

`call` does a couple of things:

  * If the client is not yet connected to the server, it waits until it connects (this might block until the connection is established)
  * Sends a "call" message to the server
  * Waits for the response and returns it as a msgpack object - this blocks until the response is read.

In the example above, you can see how getting a strongly typed value from the result is done: using the `as` member template. This takes the msgpack object and tries to deserialize it into the type given. If that fails, you will get a `type_error`.

`call` takes at least one parameter (the name of the function to call), and an arbitrary number and type of other paramters that are meant to be passed to the function being called. Each parameter has to be serializable by msgpack.

!!! tip
    See [msgpack adaptors](https://github.com/msgpack/msgpack-c/wiki/v1_1_cpp_adaptor) from the msgpack documentation for more information on serializing and deserializing custom types.

## The Mandelbrot client

The client for the mandelbrot server above is interesting because we will take advantage of the multiple workers in the server. In order to do that, instead of `call` we are going to use `async_call`.

`async_call` is very similar to `call`, but it does not wait for the response. Instead, it will return a [future](http://en.cppreference.com/w/cpp/thread/future), allowing us to continue our program flow and retrieve the result later (which the server can compute in the meantime).

```cpp
rpc::client c("127.0.0.1", 8080);

// this returns immediately:
auto result_obj = c.async_call("get_mandelbrot", width, height);

// we can now call another function and wait for its result:
auto current_time = c.call("get_time").as<std::string>();

// ... after some time, retrieve the result (optionally wait for it)
auto result = result_obj.get().as<pixel_data>();
```

The call to `get_time` can be performed with `call` (no need for `async_call`), because the other call is running on a different worker.

!!! info
    **What would happen if our server only had one worker thread?** We would get the same output, but with more delay: The server would only start processing the `get_time` call after it finished executing and writing the response of `get_mandelbrot`. Essentially, a single-threaded server works in a "queue" fashion. The same thing would happen if the server was simple under heavy load.

### Async servers vs. async clients vs. parallel execution

Does the asynchonous nature of `async_call` depend on the server or the load of the server then? No, it does not. It's important to realize that `async_call` is still asynchronous even if the server does not execute requests in parallel. If there are multiple clients connected to the server, their requests are processed in a more queued manner (still two requests processed at the same time).

!!! tip
    `rpclib` uses a simple convention: `foo` is a synchronous call, `async_foo` is asynchronous. This conventions was adapted from Asio. The latter only means that the call returns "immediately" (or rather, very quickly and without finishing all of the work).

The two worker threads in the mandelbrot server can serve two clients in parallel. Or two calls of the same client, which happens in the example. In order to be able to send two requests in an interleaved fashion, we first use `async_call` which allows the control flow of the client to continue.

# Where to go from here

The [Cookbook](cookbook.md) features most (if not all) intended use cases of rpclib - it's a great
place to continue.

If you are interested in the internal design of `rpclib`, take a look at the [Internals](internals.md)
page.

