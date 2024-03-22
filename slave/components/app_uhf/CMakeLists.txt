# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(srcs "app_uhf.c")
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS include
                    INCLUDE_DIRS "."
                    REQUIRES main)