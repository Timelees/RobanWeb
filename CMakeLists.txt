cmake_minimum_required(VERSION 3.5) # CMake install : https://cmake.org/download/
project(robanweb LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "D:/QT/6.2.4/mingw_64") # Qt Kit Dir
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt6 COMPONENTS Widgets REQUIRED) # Qt COMPONENTS
find_package(Qt6 COMPONENTS WebSockets REQUIRED)
find_package(Qt6 COMPONENTS Sql REQUIRED)

# 设置包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 设置UIC的搜索路径，让它在ui目录中查找.ui文件
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")

# 收集源文件
file(GLOB SRC_FILES 
    "src/*.cpp"
)

# 收集UI文件
file(GLOB UI_FILES 
    "ui/*.ui"
)

# 收集头文件（用于MOC）
file(GLOB HEADER_FILES 
    "include/*.h"
)

# aux_source_directory(./src srcs)


# 指定MSVC UTF-8编码  
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# 创建可执行文件
add_executable(${PROJECT_NAME}
    WIN32 # 如果需要调试终端，请注释此行
    ${SRC_FILES} 
    ${UI_FILES}
    ${HEADER_FILES}
) 



# 链接Qt库
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets) # Qt6 Shared Library
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::WebSockets) 
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Sql)

# 设置目标属性，确保MOC能找到头文件
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}  # 构建目录，用于找到自动生成的ui_*.h文件
)

if(WIN32)
    set_target_properties(robanweb PROPERTIES WIN32_EXECUTABLE FALSE)
endif()