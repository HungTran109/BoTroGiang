# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(srcs "tcp_console.c" "app_debug.c" "lwrb.c" "app_cli.c" "app_shell.c")
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES driver 
                    PRIV_REQUIRES network)