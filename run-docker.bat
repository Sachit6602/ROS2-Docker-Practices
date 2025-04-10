@echo off

REM Define variables
set IMAGE_NAME=ros_foxy
set HOST_WORKSPACE=C:\Users\sachi\Downloads\Docker\ros2_osrf

REM Get host IP for DISPLAY (Windows host IP inside WSL/docker network)
for /f "tokens=2 delims=[]" %%a in ('ping -n 1 host.docker.internal ^| findstr "["') do set HOST_IP=%%a

REM Build the Docker image
docker build -t %IMAGE_NAME% .


REM Run the Docker container with GUI support
docker run -it --rm ^
    --name ros2 ^
    -v "%HOST_WORKSPACE%:/workspace" ^
    -e DISPLAY=host.docker.internal:0.0 ^
    %IMAGE_NAME%
