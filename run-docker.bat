@echo off

REM Define variables
set IMAGE_NAME=ros_foxy
set HOST_WORKSPACE=C:\Users\sachi\Downloads\Docker\ros2_osrf

REM Build the Docker image
docker build -t %IMAGE_NAME% .

REM Run the Docker container
docker run -it --rm -v "%HOST_WORKSPACE%:/workspace" %IMAGE_NAME%