cmake_minimum_required(VERSION 3.10)
project(OTSM)

# 先编译 OHAL 库
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../HAL ${CMAKE_BINARY_DIR}/HAL_build)

# 获取当前目录下所有的 .cpp 和 .c 文件
file(GLOB OTSM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.c)

# 创建 OTSM 库
add_library(OTSM SHARED ${OTSM_SOURCES})

# 让 OTSM 依赖 OHAL，并链接 OHAL 库
target_link_libraries(OTSM PRIVATE OHAL)

# 让 OTSM 头文件能被其他模块使用
#target_include_directories(OTSM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../HAL)

# 指定库的安装路径
install(TARGETS OTSM DESTINATION /usr/local/lib)  # 安装到 /usr/local/lib
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION /usr/local/include/OTSM)  # 头文件
