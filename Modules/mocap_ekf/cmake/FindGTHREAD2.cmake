# - Try to find the GTHREAD2 libraries
# Once done this will define
#
#  GTHREAD2_FOUND - system has GTHREAD2
#  GTHREAD2_INCLUDE_DIR - the GTHREAD2 include directory
#  GTHREAD2_LIBRARY - GTHREAD2 library

# Copyright (c) 2008 Laurent Montel, <montel@kde.org>
#
# Redistribution aqqqnd use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# From the KDE source tree


if(GTHREAD2_INCLUDE_DIR AND GTHREAD2_LIBRARIES)
    # Already in cache, be silent
    set(GTHREAD2_FIND_QUIETLY TRUE)
endif(GTHREAD2_INCLUDE_DIR AND GTHREAD2_LIBRARIES)

if (NOT WIN32)
   include(UsePkgConfig)
   pkgconfig(gthread-2.0 _LibGTHREAD2IncDir _LibGTHREAD2LinkDir _LibGTHREAD2LinkFlags _LibGTHREAD2Cflags)
endif(NOT WIN32)

find_path(GTHREAD2_MAIN_INCLUDE_DIR glib.h
          PATH_SUFFIXES glib-2.0
          PATHS ${_LibGTHREAD2IncDir} )

# search the glibconfig.h include dir under the same root where the library is found
find_library(GTHREAD2_LIBRARY 
             NAMES gthread-2.0
             PATHS ${_LibGTHREAD2LinkDir} )

get_filename_component(gthread2LibDir "${GTHREAD2_LIBRARY}" PATH)

find_path(GTHREAD2_INTERNAL_INCLUDE_DIR glibconfig.h
          PATH_SUFFIXES glib-2.0/include
          PATHS ${_LibGTHREAD2IncDir} "${GTHREAD2LibDir}" ${CMAKE_SYSTEM_LIBRARY_PATH})

set(GTHREAD2_INCLUDE_DIR "${GTHREAD2_MAIN_INCLUDE_DIR}")

# not sure if this include dir is optional or required
# for now it is optional
if(GTHREAD2_INTERNAL_INCLUDE_DIR)
  set(GTHREAD2_INCLUDE_DIR ${GTHREAD2_INCLUDE_DIR} "${GTHREAD2_INTERNAL_INCLUDE_DIR}")
endif(GTHREAD2_INTERNAL_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTHREAD2  DEFAULT_MSG  GTHREAD2_LIBRARY GTHREAD2_MAIN_INCLUDE_DIR)

mark_as_advanced(GTHREAD2_INCLUDE_DIR GTHREAD2_LIBRARY)