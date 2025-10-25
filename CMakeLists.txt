cmake_minimum_required(VERSION 3.5) # CMake install : https://cmake.org/download/
project(robanweb LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "E:/QT6/6.5.10/mingw_64") # Qt Kit Dir
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt6 COMPONENTS Widgets REQUIRED) # Qt COMPONENTS
find_package(Qt6 COMPONENTS WebSockets REQUIRED)
find_package(Qt6 COMPONENTS Sql REQUIRED)
find_package(Qt6 COMPONENTS OpenGLWidgets REQUIRED)

# 设置包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 设置UIC的搜索路径，让它在ui目录中查找.ui文件
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")

# 收集源文件（递归）
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Exclude test-only sources from the main executable (they are built as separate targets)
list(REMOVE_ITEM SRC_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/util/test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/util/test_viewer.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/util/test.cpp"
)

# 收集UI文件（递归以防子目录）
file(GLOB_RECURSE UI_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui"
)

# 收集头文件（用于MOC，递归）
file(GLOB_RECURSE HEADER_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
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
target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::OpenGLWidgets)


# 链接OpenGL库
# Link system OpenGL (on Windows use opengl32)
if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE opengl32)
else()
    find_package(OpenGL REQUIRED)
    if (TARGET OpenGL::GL)
        target_link_libraries(${PROJECT_NAME} PRIVATE OpenGL::GL)
    endif()
endif()

    # If user placed prebuilt assimp files in project root lib/ and bin/, prefer them
    if (NOT ASSIMP_LIB)
        set(_proj_assimp_lib "${CMAKE_CURRENT_SOURCE_DIR}/lib/libassimp.dll.a")
        if (EXISTS ${_proj_assimp_lib})
            set(ASSIMP_LIB ${_proj_assimp_lib})
            message(STATUS "Using project-local Assimp import library: ${ASSIMP_LIB}")
        endif()
    endif()

    # If a runtime DLL exists in project bin/, copy it next to the target executable after build
    set(_proj_assimp_dll "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll")
    if (EXISTS ${_proj_assimp_dll})
        message(STATUS "Found project-local Assimp DLL: ${_proj_assimp_dll}")
        # Add post-build copy for main target
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_proj_assimp_dll}" $<TARGET_FILE_DIR:${PROJECT_NAME}>
        )
        # Also copy for test_assimp if target exists (guarded later after target creation)
        set(_copy_assimp_dll_for_test TRUE)
    endif()
# 设置目标属性，确保MOC能找到头文件
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}  # 构建目录，用于找到自动生成的ui_*.h文件
)

if(WIN32)
    set_target_properties(robanweb PROPERTIES WIN32_EXECUTABLE FALSE)
endif()


# assimp测试
# Create a small viewer test executable that uses Qt and Assimp to display a model
set(TEST_VIEWER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/util/test_viewer.cpp")
if (EXISTS ${TEST_VIEWER_SRC})
    add_executable(test_viewer ${TEST_VIEWER_SRC})
    target_include_directories(test_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
    target_link_libraries(test_viewer PRIVATE Qt6::Widgets Qt6::OpenGLWidgets)

    # prefer imported target if available
    if (TARGET assimp::assimp)
        target_link_libraries(test_viewer PRIVATE assimp::assimp)
    elseif (ASSIMP_LIB)
        target_link_libraries(test_viewer PRIVATE ${ASSIMP_LIB})
    else()
        message(WARNING "Assimp not found for test_viewer. Set ASSIMP_ROOT or put libassimp.dll.a into project lib/ to link.")
    endif()

    if (WIN32)
        target_link_libraries(test_viewer PRIVATE opengl32)
    else()
        find_package(OpenGL REQUIRED)
        if (TARGET OpenGL::GL)
            target_link_libraries(test_viewer PRIVATE OpenGL::GL)
        endif()
    endif()

    # Copy project-local DLL next to the executable if present
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll")
        add_custom_command(TARGET test_viewer POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll" $<TARGET_FILE_DIR:test_viewer>
        )
    endif()
endif()