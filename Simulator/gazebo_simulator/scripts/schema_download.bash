#!/bin/bash
# Download helper script for official sdformat schemas
# These should then be modified individually

SCRIPT_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
export SCHEMA_DIR="${SCRIPT_DIR}/schemas"
export SCHEMA_URL="http://sdformat.org/schemas"

download_schemas() {
	for file in \
		"actor" "altimeter" "audio_sink" "audio_source" "box_shape" \
		"camera" "collision" "contact" "cylinder_shape" "forcetorque" \
		"geometry" "gps" "gripper" "gui" "heightmap_shape" "image_shape" \
		"imu" "inertial" "joint" "light" "link" "logical_camera" "magnetometer" \
		"material" "mesh_shape" "model" "physics" "plane_shape" "plugin" \
		"polyline_shape" "population" "projector" "ray" "rfid" "rfidtag" \
		"road" "root" "scene" "sensor" "sonar" "sphere_shape" "spherical_coordinates" \
		"state" "surface" "transceiver" "types" "visual" "world"
		do wget -qnc "${SCHEMA_URL}/$file.xsd" -P "$1"
	done
	echo "Schemas download complete!"
}

check_schemas() {
	# check if schemas dir exist first
	if [ -d "${SCHEMA_DIR}" ]; then
		echo "No schemas found. Downloading..."
		download_schemas ${SCHEMA_DIR}
	else
		echo "Schemas folder not found. Creating and downloading schemas..."
		mkdir -p ${SCHEMA_DIR}
		download_schemas ${SCHEMA_DIR}
	fi
	# clear the URL prefix from the downloaded schemas
	sed -i 's/http:\/\/sdformat.org\/schemas\///g' ${SCHEMA_DIR}/*
}

check_schemas
