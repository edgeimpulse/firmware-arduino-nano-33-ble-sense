#!/bin/bash

#First we update the index of available boards
arduino-cli core update-index
#and download the core needed with specified version
arduino-cli core install arduino:mbed@1.1.4

#We need to do same for the libraries
arduino-cli lib update-index
arduino-cli lib install Arduino_LSM9DS1@1.0.0	#Inertial sensor library


#Check if installation went ok
output_string=$(arduino-cli core list)

if [[ $output_string != *"arduino:mbed"* ]]; then
	echo -e "\e[31m"
  	echo "ERROR: Failed to install core arduino:mbed"
	echo -e "\e[39m"
fi

output_string=$(arduino-cli lib list)

if [[ $output_string != *"Arduino_LSM9DS1"* ]]; then
	echo -e "\e[31m"
  	echo "ERROR: Failed to install Arduino LSM9DS1 library"
	echo -e "\e[39m"
fi

echo "If no errors occurred, the arduino cli configuration ended successfully"