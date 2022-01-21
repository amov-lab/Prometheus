#ifndef FILESYSTEM_HPP
#define FILESYSTEM_HPP

#include <iostream>
#include <boost/filesystem.hpp>
#include "arc_utilities/arc_exceptions.hpp"

namespace arc_utilities
{
    void CreateDirectory(const boost::filesystem::path& p)
    {
        if (!boost::filesystem::is_directory(p))
        {
            std::cout << "\x1b[33;1m" << p << " does not exist! Creating ... ";

            // NOTE: create_directories should be able to return true in this case
            // however due to a bug related to a trailing '/' this is not currently
            // the case in my version of boost
            // https://svn.boost.org/trac/boost/ticket/7258
            boost::filesystem::create_directories(p);
            if (boost::filesystem::is_directory(p))
    //            if (boost::filesystem::create_directories(p))
            {
                std::cout << "Succeeded!\x1b[37m\n";
            }
            else
            {
                std::cout << "\x1b[31;1mFailed!\x1b[37m\n";
                throw_arc_exception(std::runtime_error, "Unable to create directory, likely a 'trailing slash' issue");
            }
        }
    }
}

#endif // FILESYSTEM_HPP
