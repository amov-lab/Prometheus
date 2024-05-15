#!/bin/bash
# Script to validate SDF, according to SDFormat XSD schemas

echo "Gazebo SDF's schemas validation started..."

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
REPO_DIR=$(dirname "${SCRIPT_DIR}")

# Pass the dirs of the models by command line args; if not set, defaults are used
MODELS_DIR=${1:-"${REPO_DIR}/models"}
WORLDS_DIR=${2:-"${REPO_DIR}/worlds"}
# Pass another schema as a 3rd argument, if one wants to use a customized schema
# Default: the root downloaded from http://sdformat.org/schemas/root.xsd and its
# sub-schemas for validation. Note: internet connection is required so that
# xmllint uses the official schema
export SCHEMA_DIR=${3:-"${SCRIPT_DIR}/schemas"}
export SCHEMA_ROOT="${SCHEMA_DIR}/root.xsd"

# Return value; by default is 0 (no errors)
RET=0

# First, download the schemas
source ${SCRIPT_DIR}/schema_download.bash

# Helper function to run xmllint validation
xmllint_run() {
	$(xmllint -schema ${SCHEMA_ROOT} $1 --noout)
	ERROR_CODE=$?

	# If there are errors, the script will exit 1
	if [[ ${ERROR_CODE} != 0 ]]; then
		echo ${ERROR_CODE}
	fi
}

# Helper function to delete the schemas dir at the script exit
delete_schema() {
	rm -rf ${SCHEMA_DIR}
}

# Validate the models SDF schemas according to http://sdformat.org/schemas/root.xsd format
# Currently some SDF files are being skipped while the schemas are not updated/customized
# TODO: customize gps.xsd, magnetometer.xsd, altimeter.xsd, imu.xsd, camera.xsd
# TODO: make the generated SDF from xacro remove unused args (for iris.sdf)
# TODO: add a way SDFormat schema consider Xacro xmlns declarations (for delta_wing.sdf)
if [ -d ${MODELS_DIR} ]; then
	echo "Validating models at ${MODELS_DIR}:"
	while read fname; do
		ret="$(xmllint_run ${fname})"
		if [[ $ret -ge 1 ]]; then
			RET="$ret"
		fi
	done <<<"$(find ${MODELS_DIR} -type f -name '*.sdf' \
		! -name '3DR_gps_mag.sdf' ! -name 'px4flow.sdf' \
		! -name 'pixhawk.sdf' ! -name 'c920.sdf' \
		! -name 'iris.sdf' ! -name 'iris_hitl.sdf' ! -name 'delta_wing.sdf' ! -name 'r1_rover.sdf' \
		! -name 'fpv_cam.sdf' ! -name 'iris_triple_depth_camera.sdf')"
else
	echo "${MODELS_DIR} doesn't exist!"
	delete_schema
fi

# Validate the worlds SDF schemas according to http://sdformat.org/schemas/root.xsd format
if [ -d ${WORLDS_DIR} ]; then
	echo "Validating worlds at ${WORLDS_DIR}:"
	while read fname; do
		ret="$(xmllint_run ${fname})"
		if [[ $ret -ge 1 ]]; then
			RET="$ret"
		fi
	done <<<"$(find ${WORLDS_DIR} -type f -name '*.world')"
else
	echo "${WORLDS_DIR} doesn't exist!"
	delete_schema
fi

# If every SDFs are validated positively, exit 0, else exit 1
if [[ ${RET} -gt 0 ]]; then
	echo "Validation not successful! Check in the command line log for the reason"
	delete_schema
	exit 1
else
	echo "All SDFs validated positively!"
	delete_schema
	exit 0
fi
