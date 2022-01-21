#include <stdexcept>
#include <string>
#include <sstream>

#ifndef ARC_EXCEPTIONS_HPP
#define ARC_EXCEPTIONS_HPP

#define throw_arc_exception(type, ...) arc_exceptions::ArcException<type>(__FILE__, __LINE__, __VA_ARGS__)

namespace arc_exceptions
{
    template <typename ExceptionType>
    inline void ArcException(const char* file, const std::size_t line, const std::string& message)
    {
       std::ostringstream stream;
       stream << message << ": " << file << ": " << line;
       throw ExceptionType(stream.str());
    }
}

#endif // ARC_EXCEPTIONS_HPP
