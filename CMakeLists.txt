CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# include-----------------------------------------------------------------------
include(./cmake/CMakeLists_env.txt)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc_stm32.cmake)

message("current CMakeLists.txt file path is ${CMAKE_CURRENT_SOURCE_DIR}")
#exec_program("cd ${CMAKE_CURRENT_SOURCE_DIR};pwd | sed 's/^\(.*\)[/]//'" RETURN_VARIABLE RET_VAL OUTPUT_VARIABLE OUT_VAL)
string(REGEX REPLACE "^\(.*\)[/]" "" OUTPUT_VAL ${CMAKE_CURRENT_SOURCE_DIR})
set(project_name ${OUTPUT_VAL})
#message("project name is ${project_name}")

project(${project_name})

enable_language(ASM)
#FIND_PACKAGE(CMSIS REQUIRED)
#FIND_PACKAGE(StdPeriphLib REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake) 

#message("${CMAKE_TOOLCHAIN_FILE}")
#message("${CMAKE_MODULE_PATH}")
#宏预定义放到后面为什么不行？
add_definitions(-DUSE_STDPERIPH_DRIVER)
add_definitions(-DSTM32F10X_MD)

#set(libraries $(PROJECT_SOURCE_DIR)/libc/libc.a $(PROJECT_SOURCE_DIR)/libc/libm.a)
include_directories(
		${PROJECT_SOURCE_DIR}/inc
		${PROJECT_SOURCE_DIR}/lib/CMSIS/CM3/CoreSupport
		${PROJECT_SOURCE_DIR}/lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x
		${PROJECT_SOURCE_DIR}/lib/STM32F10x_StdPeriph_Driver/inc
)



#SET(PROJECT_SOURCES
#    main.c
#)
#file(GLOB APP_SRC "./src/app/*.c")
file(GLOB APP_SRC1 "src/**/*.c")
file(GLOB APP_SRC2 "lib/CMSIS/CM3/CoreSupport/*.c")
file(GLOB APP_SRC3 "lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/*.c")
#不同的编译器选用不同的startup文件
set(CMSIS_STARTUP_SOURCE "lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_md.s")
file(GLOB APP_SRC4 "lib/STM32F10x_StdPeriph_Driver/src/*.c")
# 只有add进去的才会生成compile_commands.json
add_executable(${CMAKE_PROJECT_NAME} ${APP_SRC1} ${APP_SRC2} ${APP_SRC3} ${APP_SRC4} ${CMSIS_STARTUP_SOURCE})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# add inc/ to include directories
set(BOINC_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/inc")
file(GLOB SRC_INC_ALL_FILES ${BOINC_INCLUDE_DIR} ${BOINC_INCLUDE_DIR}/*)
foreach(_FILE ${SRC_INC_ALL_FILES})
        if(IS_DIRECTORY ${_FILE})
                message(${_FILE})
                include_directories(${_FILE})
        endif()
endforeach()

# NOTE: add new subdir below, 
#       and switch to <./main/CMakeLists.txt> add target_link_libraries()
file(GLOB_RECURSE src_CMakeLists "src/**/CMakeLists.txt")
#file(GLOB_RECURSE src_CMakeLists "lib/CMSIS/CM3/CoreSupport/CMakeLists.txt")
#file(GLOB_RECURSE src_CMakeLists "lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/CMakeLists.txt")
#file(GLOB_RECURSE src_CMakeLists "lib/STM32F10x_StdPeriph_Driver/src/CMakeLists.txt")
file(GLOB_RECURSE lib_CMakeLists "lib/**/CMakeLists.txt")
set(list_CMakeLists ${lib_CMakeLists} ${src_CMakeLists})
set(Foo_DIRS "")
foreach (_file ${list_CMakeLists})
        get_filename_component(_dir ${_file} PATH)
        list (APPEND Foo_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES Foo_DIRS)
foreach (_file ${Foo_DIRS})
#        #message("${_file}")
        add_subdirectory("${_file}")
endforeach()

target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        core_surpport
        device_surpport
        std_periph
        app
        bsp
        at
        base
)

STM32_SET_HSE_VALUE(${CMAKE_PROJECT_NAME} ${STM32_HSE_VALUE})
STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# install
set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/bin)
install(
        
)

