# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# Firmware release version, embedded in the app descriptor (esp_app_desc_t)
# and reported in the bootloader log as "App version".
set(PROJECT_VER "1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xiao_motor_firmware)
