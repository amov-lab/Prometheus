prefix=@PROJECT_SPACE_DIR

Name: @(CATKIN_PACKAGE_PREFIX + PROJECT_NAME)
Description: Description of @PROJECT_NAME
Version: @PROJECT_VERSION
Cflags: @(' '.join(['-I%s' % include for include in PROJECT_PKG_CONFIG_INCLUDE_DIRS]))
Libs: -L@PROJECT_SPACE_DIR/lib @(' '.join(PKG_CONFIG_LIBRARIES_WITH_PREFIX))
Requires: @(PROJECT_CATKIN_DEPENDS)
