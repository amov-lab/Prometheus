#include <iostream>
#include <iomanip>
#include <array>
#include <vector>
#include <deque>
#include <forward_list>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>

#ifndef PRETTY_PRINT_HPP
#define PRETTY_PRINT_HPP

// Handy functions for printing vectors and pairs
namespace PrettyPrint
{
    // Base template function for printing types
    template <typename T>
    inline std::string PrettyPrint(const T& toprint, const bool add_delimiters=false, const std::string& separator=", ")
    {
        UNUSED(add_delimiters);
        UNUSED(separator);
        std::ostringstream strm;
        strm << toprint;
        return strm.str();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                       PROTOTYPES ONLY                                         /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<>
    inline std::string PrettyPrint(const bool& bool_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::Vector2d& vector_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::Vector3d& vector_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::Vector4d& vector_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::VectorXd& vector_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::MatrixXd& matrix_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::Quaterniond& quaternion_to_print, const bool add_delimiters, const std::string& separator);

    template<>
    inline std::string PrettyPrint(const Eigen::Isometry3d& transform_to_print, const bool add_delimiters, const std::string& separator);

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::pair<A, B>& pairtoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, size_t N>
    inline std::string PrettyPrint(const std::array<T, N>& arraytoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::vector<T, Allocator>& vectoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::list<T, Allocator>& listtoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::forward_list<T, Allocator>& listtoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::deque<T, Allocator>& dequetoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename A, typename B, typename Compare=std::less<A>, typename Allocator=std::allocator<std::pair<const A, B>>>
    inline std::string PrettyPrint(const std::map<A, B, Compare, Allocator>& maptoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename A, typename B, typename Compare=std::less<A>, typename Allocator=std::allocator<std::pair<const A, B>>>
    inline std::string PrettyPrint(const std::multimap<A, B, Compare, Allocator>& maptoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Compare=std::less<T>, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::set<T, Compare, Allocator>& settoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Compare=std::less<T>, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::multiset<T, Compare, Allocator>& settoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename A, typename B, typename Hash=std::hash<A>, typename Predicate=std::equal_to<A>, typename Allocator=std::allocator<std::pair<const A, B>>>
    inline std::string PrettyPrint(const std::unordered_map<A, B, Hash, Predicate, Allocator>& maptoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename A, typename B, typename Hash=std::hash<A>, typename Predicate=std::equal_to<A>, typename Allocator=std::allocator<std::pair<const A, B>>>
    inline std::string PrettyPrint(const std::unordered_multimap<A, B, Hash, Predicate, Allocator>& maptoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Hash=std::hash<T>, typename Predicate=std::equal_to<T>, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::unordered_set<T, Hash, Predicate, Allocator>& settoprint, const bool add_delimiters=false, const std::string& separator=", ");

    template <typename T, typename Hash=std::hash<T>, typename Predicate=std::equal_to<T>, typename Allocator=std::allocator<T>>
    inline std::string PrettyPrint(const std::unordered_multiset<T, Hash, Predicate, Allocator>& settoprint, const bool add_delimiters=false, const std::string& separator=", ");

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////                                   IMPLEMENTATIONS ONLY                                        /////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<>
    inline std::string PrettyPrint(const bool& bool_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(add_delimiters);
        UNUSED(separator);
        if (bool_to_print)
        {
            return "true";
        }
        else
        {
            return "false";
        }
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Vector2d& vector_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        std::ostringstream strm;
        strm << std::setprecision(12);
        if (add_delimiters)
        {
            strm << "Vector2d: <x: " << vector_to_print(0) << " y: " << vector_to_print(1) << ">";
        }
        else
        {
            strm << vector_to_print(0) << ", " << vector_to_print(1);
        }
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Vector3d& vector_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        std::ostringstream strm;
        strm << std::setprecision(12);
        if (add_delimiters)
        {
            strm << "Vector3d: <x: " << vector_to_print.x() << " y: " << vector_to_print.y() << " z: " << vector_to_print.z() << ">";
        }
        else
        {
            strm << vector_to_print.x() << ", " << vector_to_print.y() << ", " << vector_to_print.z();;
        }
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Vector4d& vector_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        std::ostringstream strm;
        strm << std::setprecision(12);
        if (add_delimiters)
        {
            strm << "Vector4d: <x: " << vector_to_print(0) << " y: " << vector_to_print(1) << " z: " << vector_to_print(2) << " w: " << vector_to_print(3) << ">";
        }
        else
        {
            strm << vector_to_print(0) << ", " << vector_to_print(1) << ", " << vector_to_print(2) << ", " << vector_to_print(3);
        }
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::VectorXd& vector_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        Eigen::IOFormat io_format(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "", "");
        if (add_delimiters)
        {
            io_format = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "VectorXd: <", ">");;
        }
        std::ostringstream strm;
        strm << std::setprecision(12);
        strm << vector_to_print.format(io_format);
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::MatrixXd& matrix_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        Eigen::IOFormat io_format(Eigen::StreamPrecision, 0, ", ", "\n", "", "", "", "");
        if (add_delimiters)
        {
            io_format = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "", "", "MatrixXd:\n[", "]");;
        }
        std::ostringstream strm;
        strm << std::setprecision(12);
        strm << matrix_to_print.format(io_format);
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Quaterniond& quaternion_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        std::ostringstream strm;
        strm << std::setprecision(12);
        if (add_delimiters)
        {
            strm << "Quaterniond <x: " << quaternion_to_print.x() << " y: " << quaternion_to_print.y() << " z: " << quaternion_to_print.z() << " w: " << quaternion_to_print.w() << ">";
        }
        else
        {
            strm << quaternion_to_print.x() << ", " << quaternion_to_print.y() << ", " << quaternion_to_print.z() << ", " << quaternion_to_print.w();
        }
        return strm.str();
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Isometry3d& transform_to_print, const bool add_delimiters, const std::string& separator)
    {
        UNUSED(separator);
        std::ostringstream strm;
        strm << std::setprecision(12);
        Eigen::Vector3d vector_to_print = transform_to_print.translation();
        Eigen::Quaterniond quaternion_to_print(transform_to_print.rotation());
        if (add_delimiters)
        {
            strm << "Isometry3d <x: " << vector_to_print.x() << " y: " << vector_to_print.y() << " z: " << vector_to_print.z() << ">, <x: " << quaternion_to_print.x() << " y: " << quaternion_to_print.y() << " z: "  << quaternion_to_print.z() << " w: " << quaternion_to_print.w() << ">";
        }
        else
        {
            strm << vector_to_print.x() << ", " << vector_to_print.y() << ", " << vector_to_print.z() << " : " << quaternion_to_print.x() << ", " << quaternion_to_print.y() << ", "  << quaternion_to_print.z() << ", " << quaternion_to_print.w();
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::pair<A, B>& pairtoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (add_delimiters)
        {
            strm << "<" << PrettyPrint(pairtoprint.first, add_delimiters, separator) << ": " << PrettyPrint(pairtoprint.second, add_delimiters, separator) << ">";
        }
        else
        {
            strm << PrettyPrint(pairtoprint.first, add_delimiters, separator) << ": " << PrettyPrint(pairtoprint.second, add_delimiters, separator);
        }
        return strm.str();
    }

    template <typename T, size_t N>
    inline std::string PrettyPrint(const std::array<T, N>& arraytoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (arraytoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[" << PrettyPrint(arraytoprint[0], add_delimiters, separator);
                for (size_t idx = 1; idx < arraytoprint.size(); idx++)
                {
                    strm << separator << PrettyPrint(arraytoprint[idx], add_delimiters, separator);
                }
                strm << "]";
            }
            else
            {
                strm << PrettyPrint(arraytoprint[0], add_delimiters, separator);
                for (size_t idx = 1; idx < arraytoprint.size(); idx++)
                {
                    strm << separator << PrettyPrint(arraytoprint[idx], add_delimiters, separator);
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Allocator>
    inline std::string PrettyPrint(const std::vector<T, Allocator>& vectoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (vectoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[" << PrettyPrint(vectoprint[0], add_delimiters, separator);
                for (size_t idx = 1; idx < vectoprint.size(); idx++)
                {
                    strm << separator << PrettyPrint(vectoprint[idx], add_delimiters, separator);
                }
                strm << "]";
            }
            else
            {
                strm << PrettyPrint(vectoprint[0], add_delimiters, separator);
                for (size_t idx = 1; idx < vectoprint.size(); idx++)
                {
                    strm << separator << PrettyPrint(vectoprint[idx], add_delimiters, separator);
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Allocator>
    inline std::string PrettyPrint(const std::list<T, Allocator>& listtoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (listtoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::list<T, Allocator>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::list<T, Allocator>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Allocator>
    inline std::string PrettyPrint(const std::forward_list<T, Allocator>& listtoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (listtoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::forward_list<T, Allocator>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::forward_list<T, Allocator>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Allocator>
    inline std::string PrettyPrint(const std::deque<T, Allocator>& dequetoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (dequetoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::deque<T, Allocator>::const_iterator itr;
                for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
                {
                    if (itr != dequetoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::deque<T, Allocator>::const_iterator itr;
                for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
                {
                    if (itr != dequetoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B, typename Compare, typename Allocator>
    inline std::string PrettyPrint(const std::map<A, B, Compare, Allocator>& maptoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::map<A, B, Compare, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::map<A, B, Compare, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B, typename Compare, typename Allocator>
    inline std::string PrettyPrint(const std::multimap<A, B, Compare, Allocator>& maptoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::multimap<A, B, Compare, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::multimap<A, B, Compare, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Compare, typename Allocator>
    inline std::string PrettyPrint(const std::set<T, Compare, Allocator>& settoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::set<T, Compare, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::set<T, Compare, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Compare, typename Allocator>
    inline std::string PrettyPrint(const std::multiset<T, Compare, Allocator>& settoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::multiset<T, Compare, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::multiset<T, Compare, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B, typename Hash, typename Predicate, typename Allocator>
    inline std::string PrettyPrint(const std::unordered_map<A, B, Hash, Predicate, Allocator>& maptoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::unordered_map<A, B, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::unordered_map<A, B, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B, typename Hash, typename Predicate, typename Allocator>
    inline std::string PrettyPrint(const std::unordered_multimap<A, B, Hash, Predicate, Allocator>& maptoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::unordered_multimap<A, B, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::unordered_multimap<A, B, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << separator << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Hash, typename Predicate, typename Allocator>
    inline std::string PrettyPrint(const std::unordered_set<T, Hash, Predicate, Allocator>& settoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::unordered_set<T, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::unordered_set<T, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T, typename Hash, typename Predicate, typename Allocator>
    inline std::string PrettyPrint(const std::unordered_multiset<T, Hash, Predicate, Allocator>& settoprint, const bool add_delimiters, const std::string& separator)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::unordered_multiset<T, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::unordered_multiset<T, Hash, Predicate, Allocator>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << separator << PrettyPrint(*itr, add_delimiters, separator);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters, separator);
                    }
                }
            }
        }
        return strm.str();
    }
}

#endif // PRETTY_PRINT_HPP
