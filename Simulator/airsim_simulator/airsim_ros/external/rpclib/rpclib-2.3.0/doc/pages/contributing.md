This project is MIT-licensed and the development is open. All contributions are welcome. This
document provides information on why, how and what to contribute.

## Why contribute

At the moment, this project is an approachable, intermediate-advanced level C++ project. If you contribute code, you may deepen your knowledge in these areas:

  * RPC clients and servers
  * Networking (through asio), TCP
  * Scalable architecture
  * Async programming
  * Template metaprogramming * Modern C++ (C++14 and beyond) * Google Test and Google Mock
  * Travis CI & Appveyor
  * Continous delivery
  * CMake

### I've never contributed to open source, can I try here?

Absolutely! First-time contributors are very much welcome in this project. Having your pull request
accepted by a project is a great accomplishment. Even if your pull request only fixes a typo, don't
hesitate to send it. If it's useful and good quality, it will be merged.

### Am I going to be attributed for my contribution?

Yes. Each release will list the names of contributors. If you don't want to be attributed, please
state it explicitly in the pull request. If you don't have your name on your Github profile or in
the commit, your username will be used.

## What can be contributed

### Issues

If you find any issues with a released or development version of `rpclib`, open an issue ticket on
Github. Please try to include:

  * as many details as possible
  * exact steps for reproducing the problem (if applicable)
  * pedantic description of what the expected behavior would be

### Code

Code contributions are welcome as well, no matter how small they are.

Some guidelines (this section will be expanded later):

  * Use clang-format with the included configuration file
  * Avoid whitespace-only diffs in files
  * Use descriptive commit messages that can serve as a useful piece of code history
  * Make sure that all unit tests pass
  * Write new, meaningful unit tests for new code
  * Document any public code with doxygen
  * Update the documentation in the docs folder (if applicable)
  * Add examples to the [Cookbook](cookbook.md)
  * Don't add external dependencies

Pull requests for open issues are preferred over others.

Please keep in mind that some pull requests may not fit into the vision of the project and may be
rejected despite matching the above criteria. If you start working on a larger code contribution, it might be a good idea to let the project know about it (especially if there is no open issue about it).

### Support scripts

`rpclib` is not only C++: it uses a fair amount of CMake, bash, Powershell and python to achieve
the end result. These supporting scripts also require continued development, refactoring, cleanup
and new features. Innovative ideas about how to do this all better are welcome (it's kind of a mess now, admittedly).

### Example projects

The more example projects `rpclib` ships with, the better. This is perhaps the most rewarding code
contribution option, because the guidelines are less strict regarding these.

  * Create both a server and client application
  * Create a CMakeLists.txt (similarly to existing ones)
  * Make sure that your example application demonstrates a use-case that is not demonstrated by
    other examples (don't worry, there are plenty of those)
  * Add lots of coments that explain the code
  * You may add any external dependencies

### Documentation, proofreading

`rpclib` aims to have an exemplary quality documenation. I'm not a native speaker, so it's likely
that there are sentences with weird constructs, misspelled words or just typos in many places.
Correcting those are invaluable help for me. To top it off, there is always room to expand, clean
up and just generally improve the documentation to provide an easy learning curve for new users of
the library.

### HTML+CSS

The very site you are reading now is built using mkdocs and doxygen; but in some places it contains
hand-written HTML and CSS. This is not my area of expertise. But documentation is not only about
content, it's also the presentation. Changes that improve the robustness of the HTML and CSS are a huge win for the project.
