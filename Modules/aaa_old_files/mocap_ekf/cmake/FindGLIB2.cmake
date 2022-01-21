# - Try to find the GLIB2 libraries
# Once done this will define
#
#  GLIB2_FOUND - system has glib2
#  GLIB2_INCLUDE_DIR - the glib2 include directory
#  GLIB2_LIBRARY - glib2 library

# Copyright (c) 2008 Laurent Montel, <montel@kde.org>
#
# Redistribution aqqqnd use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

# From the KDE source tree


if(GLIB2_INCLUDE_DIR AND GLIB2_LIBRARIES)
    # Already in cache, be silent
    set(GLIB2_FIND_QUIETLY TRUE)
endif(GLIB2_INCLUDE_DIR AND GLIB2_LIBRARIES)

if (NOT WIN32)
   include(UsePkgConfig)
   pkgconfig(glib-2.0 _LibGLIB2IncDir _LibGLIB2LinkDir _LibGLIB2LinkFlags _LibGLIB2Cflags)
endif(NOT WIN32)

find_path(GLIB2_MAIN_INCLUDE_DIR glib.h
          PATH_SUFFIXES glib-2.0
          PATHS ${_LibGLIB2IncDir} )

# search the glibconfig.h include dir under the same root where the library is found
             
             find_library(GLIB2_LIBRARY 
             NAMES glib-2.0
             PATHS ${_LibGLIB2LinkDir} )
             

get_filename_component(glib2LibDir "${GLIB2_LIBRARY}" PATH)

find_path(GLIB2_INTERNAL_INCLUDE_DIR glibconfig.h
          PATH_SUFFIXES glib-2.0/include
          PATHS ${_LibGLIB2IncDir} "${glib2LibDir}" ${CMAKE_SYSTEM_LIBRARY_PATH})

set(GLIB2_INCLUDE_DIR "${GLIB2_MAIN_INCLUDE_DIR}")

# not sure if this include dir is optional or required
# for now it is optional
if(GLIB2_INTERNAL_INCLUDE_DIR)
  set(GLIB2_INCLUDE_DIR ${GLIB2_INCLUDE_DIR} "${GLIB2_INTERNAL_INCLUDE_DIR}")
endif(GLIB2_INTERNAL_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLIB2  DEFAULT_MSG  GLIB2_LIBRARY GLIB2_MAIN_INCLUDE_DIR)

mark_as_advanced(GLIB2_INCLUDE_DIR GLIB2_LIBRARY)