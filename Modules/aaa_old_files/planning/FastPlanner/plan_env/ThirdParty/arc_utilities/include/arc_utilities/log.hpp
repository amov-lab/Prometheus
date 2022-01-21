#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <boost/filesystem.hpp>

#ifndef LOG_HPP
#define LOG_HPP

#define LOG(log, message)                   \
    (log).logMessage(                       \
        static_cast<std::ostringstream&>(   \
            std::ostringstream().flush()    \
            << std::setprecision(12)        \
            << (message)                    \
       ).str()                              \
   )

#define LOG_STREAM(log, message)            \
    (log).logMessage(                       \
        static_cast<std::ostringstream&>(   \
            std::ostringstream().flush()    \
            << std::setprecision(12)        \
            << message                      \
       ).str()                              \
   )

#define LOG_COND(log, cond, message)        \
    if ((cond)) LOG(log, message)


#define LOG_COND_STREAM(log, cond, message) \
    if ((cond)) LOG_STREAM(log, message)

// TODO: confirm that I havn't made any mistakes in this file
namespace Log
{
    class Log
    {
        public:
            Log(const std::string& filename, bool add_header = true)
                : filename_(filename)
            {
                // If it hasn't been opened, assume that it is because the
                // directory doesn't exist.
                boost::filesystem::path p(filename_);
                boost::filesystem::path dir = p.parent_path();
                if (!boost::filesystem::is_directory(dir))
                {
                    std::cerr << "\x1b[33;1m" << dir << " does not exist! Creating ... ";

                    // NOTE: create_directories should be able to return true in this case
                    // however due to a bug related to a trailing '/' this is not currently
                    // the case in my version of boost
                    // https://svn.boost.org/trac/boost/ticket/7258
                    boost::filesystem::create_directories(dir);
                    if (boost::filesystem::is_directory(dir))
        //            if (boost::filesystem::create_directories(p))
                    {
                        std::cerr << "Succeeded!\x1b[37m\n";
                    }
                    else
                    {
                        std::cerr << "\x1b[31;1mFailed!\x1b[37m\n";
                    }
                }

                out_file_.open(filename, std::ios_base::out | std::ios_base::trunc);
                // check if we've succesfully opened the file
                if (!out_file_.is_open())
                {
                    std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << filename << "\x1b[37m \n";
                    throw std::invalid_argument("filename must be write-openable");
                }

                if (add_header)
                {
                    time_t rawtime;
                    tm * timeinfo;
                    char buffer[80];

                    time(&rawtime);
                    timeinfo = localtime(&rawtime);

                    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

                    out_file_ << buffer << std::endl;
                }
            }

            /** Copy constructor */
            Log(const Log& other)
                : filename_(other.filename_)
                , out_file_(filename_, std::ios_base::out | std::ios_base::app)
            {
            }

            /** Move constructor */
            Log(Log&& other)
                : filename_(other.filename_)
                , out_file_(filename_, std::ios_base::out | std::ios_base::app)
            {
                other.out_file_.close();
            }

            /** Destructor */
            ~Log()
            {
                if (out_file_.is_open())
                {
                    out_file_.close();
                }
            }

            /** Copy assignment operator */
            Log& operator= (const Log& other)
            {
                Log tmp(other); // re-use copy-constructor
                *this = std::move(tmp); // re-use move-assignment
                return *this;
            }

            /** Move assignment operator */
            Log& operator= (Log&& other)
            {
                std::swap(filename_, other.filename_);
                other.out_file_.close();

                if (out_file_.is_open())
                {
                    out_file_.close();
                }

                out_file_.open(filename_, std::ios_base::out | std::ios_base::app);

                return *this;
            }

            void logMessage(const std::string& message)
            {
                out_file_ << message << std::endl;
            }

        private:
            std::string filename_;
            std::ofstream out_file_;
    };
}

#endif // LOG_HPP
