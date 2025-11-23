cmake_minimum_required(VERSION 3.5) # CMake install : https://cmake.org/download/
project(robanweb LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "E:/QT6/6.5.10/mingw_64") # Qt Kit Dir
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure reproducible runtime output directories so that when Qt Creator
# or VSCode run the built executable the binary and its runtime resources
# live in predictable locations. This avoids "working directory" differences
# between IDEs being the reason assets/models are not found at runtime.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# For multi-config generators (Visual Studio etc.) set per-config output dirs too
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg} ${CMAKE_BINARY_DIR}/bin)
endforeach()


find_package(Qt6 COMPONENTS Widgets REQUIRED) # Qt COMPONENTS
find_package(Qt6 COMPONENTS WebSockets REQUIRED)
find_package(Qt6 COMPONENTS Sql REQUIRED)
find_package(Qt6 COMPONENTS OpenGLWidgets REQUIRED)

# Try to find Assimp (optional). If found, CMake will provide target assimp::assimp
find_package(assimp QUIET)

# Try more variants to improve detection on Windows/vcpkg environments
if (NOT TARGET assimp::assimp)
    find_package(assimp CONFIG QUIET)
endif()
if (NOT TARGET assimp::assimp)
    find_package(Assimp QUIET)
endif()


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
# Some environments return different path separators, so filter entries that reside under src/test or are named test_*.cpp
set(_FILTERED_SRC_FILES)
foreach(_f ${SRC_FILES})
    string(REPLACE "\\" "/" _f_unix "${_f}")
    string(TOLOWER "${_f_unix}" _f_unix_low)
    if(_f_unix_low MATCHES "/src/test/" OR _f_unix_low MATCHES "/src/util/test" OR _f_unix_low MATCHES "/src/test_")
        message(VERBOSE "Excluding test source from main target: ${_f}")
    else()
        list(APPEND _FILTERED_SRC_FILES "${_f}")
    endif()
endforeach()
set(SRC_FILES ${_FILTERED_SRC_FILES})

# 收集UI文件（递归以防子目录）
file(GLOB_RECURSE UI_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/ui/*.ui"
)

# 收集Qt资源文件，后续直接把新的.qrc文件放进对应目录即可自动加入
set(_qrc_search_paths
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/resources"
    "${CMAKE_CURRENT_SOURCE_DIR}/res"
)
set(QRC_FILES)
foreach(_qrc_path IN LISTS _qrc_search_paths)
    if (EXISTS "${_qrc_path}")
        file(GLOB QRC_CANDIDATES "${_qrc_path}/*.qrc")
        list(APPEND QRC_FILES ${QRC_CANDIDATES})
    endif()
endforeach()
list(REMOVE_DUPLICATES QRC_FILES)


# 收集头文件（用于MOC，递归）
file(GLOB_RECURSE HEADER_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

# 确保 robotControlDialog.ui 被包含
list(APPEND UI_FILES "${CMAKE_CURRENT_SOURCE_DIR}/ui/robotControlDialog.ui")


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
    ${QRC_FILES}
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
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_proj_assimp_dll}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        )
        # Also copy for test_assimp if target exists (guarded later after target creation)
        set(_copy_assimp_dll_for_test TRUE)
    endif()

    # Copy the assets directory next to the executable so runtime can load models/textures
    set(_project_assets_dir "${CMAKE_CURRENT_SOURCE_DIR}/assets")
    if(EXISTS "${_project_assets_dir}")
        message(STATUS "Adding post-build step to copy assets to output directory: ${_project_assets_dir}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_assets_dir}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
        )
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_assets_dir}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets"
        )
    else()
        message(STATUS "No local assets/ directory found at: ${_project_assets_dir}")
    endif()

    # Copy the config directory next to the executable so runtime can read configuration files
    set(_project_config_dir "${CMAKE_CURRENT_SOURCE_DIR}/config")
    if(EXISTS "${_project_config_dir}")
        message(STATUS "Adding post-build step to copy config to output directory: ${_project_config_dir}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_config_dir}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/config"
        )
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_config_dir}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/config"
        )
    else()
        message(STATUS "No local config/ directory found at: ${_project_config_dir}")
    endif()

    set(_project_database_dir "${CMAKE_CURRENT_SOURCE_DIR}/database")
    if(EXISTS "${_project_database_dir}")
        message(STATUS "Adding post-build step to copy config to output directory: ${_project_database_dir}")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_database_dir}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/database"
        )
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_project_database_dir}" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/database"
        )
    else()
        message(STATUS "No local database/ directory found at: ${_project_config_dir}")
    endif()

    # Link Assimp to the main project if available (ModelDisplay and other code use Assimp)
    if (TARGET assimp::assimp)
        target_link_libraries(${PROJECT_NAME} PRIVATE assimp::assimp)
    elseif (ASSIMP_LIB)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${ASSIMP_LIB})
    else()
        message(WARNING "Assimp not linked to ${PROJECT_NAME}. If your project uses Assimp (model loading), set ASSIMP_ROOT or provide libassimp in project lib/ to link.")
    endif()
# 设置目标属性，确保MOC能找到头文件
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}  # 构建目录，用于找到自动生成的ui_*.h文件
)

## Ensure this target places its runtime/library outputs in the canonical directories
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
)

if(WIN32)
    set_target_properties(robanweb PROPERTIES WIN32_EXECUTABLE FALSE)
endif()

# Build an OBJECT library from the main sources so test targets can link the
# compiled object files (this avoids duplicating implementation in test targets
# while keeping the main executable layout unchanged).
## Build object list excluding the main entry point so we don't pull in multiple mains
set(ROBANWEB_OBJ_SOURCES)
foreach(_f ${SRC_FILES})
    # exclude the primary executable entrypoint
    if (NOT (_f STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"))
        list(APPEND ROBANWEB_OBJ_SOURCES "${_f}")
    endif()
endforeach()

add_library(robanweb_objs OBJECT ${ROBANWEB_OBJ_SOURCES} ${HEADER_FILES} ${UI_FILES})
target_sources(robanweb_objs PRIVATE ${QRC_FILES})
target_include_directories(robanweb_objs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
# Link Qt modules here so autogen and compilation have proper include paths. Note: OBJECT libraries do not
# propagate link libraries to consuming targets, so test executables still need to link required Qt libs.
target_link_libraries(robanweb_objs PRIVATE Qt6::Widgets Qt6::WebSockets Qt6::Sql Qt6::OpenGLWidgets)



# assimp测试
# Create a small viewer test executable that uses Qt and Assimp to display a model
# set(TEST_VIEWER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/test/test_viewer.cpp")
# set(TEST_MODEL_BONE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/test/test_model_bone.cpp")
# set(TEST_MODEL_SCENE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/test/test_model_scene.cpp")
# set(TEST_SCENE_LOC_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/test/test_scene_loc.cpp")
# if (EXISTS ${TEST_VIEWER_SRC})
#     add_executable(test_viewer ${TEST_VIEWER_SRC})
#     add_executable(test_model_bone ${TEST_MODEL_BONE_SRC})
#     add_executable(test_model_scene ${TEST_MODEL_SCENE_SRC})
#     add_executable(test_scene_loc ${TEST_SCENE_LOC_SRC})
#     # Ensure test targets that use RobotManager link its implementation
    
#     target_include_directories(test_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
#     target_include_directories(test_model_bone PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
#     target_include_directories(test_model_scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
#     target_include_directories(test_scene_loc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_BINARY_DIR})
#     target_link_libraries(test_viewer PRIVATE Qt6::Widgets Qt6::OpenGLWidgets Qt6::WebSockets Qt6::Sql)
#     target_link_libraries(test_model_bone PRIVATE  Qt6::Widgets Qt6::OpenGLWidgets Qt6::WebSockets Qt6::Sql)
#     target_link_libraries(test_model_scene PRIVATE  Qt6::Widgets Qt6::OpenGLWidgets Qt6::WebSockets Qt6::Sql)
#     target_link_libraries(test_scene_loc PRIVATE  Qt6::Widgets Qt6::OpenGLWidgets Qt6::WebSockets Qt6::Sql)
#     # Link in compiled object files from main sources so tests get the implementation
#     target_link_libraries(test_viewer PRIVATE $<TARGET_OBJECTS:robanweb_objs>)
#     target_link_libraries(test_model_bone PRIVATE $<TARGET_OBJECTS:robanweb_objs>)
#     target_link_libraries(test_model_scene PRIVATE $<TARGET_OBJECTS:robanweb_objs>)
#     target_link_libraries(test_scene_loc PRIVATE $<TARGET_OBJECTS:robanweb_objs>)
#     # prefer imported target if available
#     if (TARGET assimp::assimp)
#         target_link_libraries(test_viewer PRIVATE assimp::assimp)
#         target_link_libraries(test_model_bone PRIVATE assimp::assimp)
#         target_link_libraries(test_model_scene PRIVATE assimp::assimp)
#         target_link_libraries(test_scene_loc PRIVATE assimp::assimp)
#     elseif (ASSIMP_LIB)
#         target_link_libraries(test_viewer PRIVATE ${ASSIMP_LIB})
#         target_link_libraries(test_model_bone PRIVATE ${ASSIMP_LIB})
#         target_link_libraries(test_model_scene PRIVATE ${ASSIMP_LIB})
#         target_link_libraries(test_scene_loc PRIVATE ${ASSIMP_LIB})
#     else()
#         message(WARNING "Assimp not found for test_viewer. Set ASSIMP_ROOT or put libassimp.dll.a into project lib/ to link.")
#     endif()

#     if (WIN32)
#         target_link_libraries(test_viewer PRIVATE opengl32)
#         target_link_libraries(test_model_bone PRIVATE opengl32)
#         target_link_libraries(test_model_scene PRIVATE opengl32)
#         target_link_libraries(test_scene_loc PRIVATE opengl32)
#     else()
#         find_package(OpenGL REQUIRED)
#         if (TARGET OpenGL::GL)
#             target_link_libraries(test_viewer PRIVATE OpenGL::GL)
#             target_link_libraries(test_model_bone PRIVATE OpenGL::GL)
#             target_link_libraries(test_model_scene PRIVATE OpenGL::GL)
#             target_link_libraries(test_scene_loc PRIVATE OpenGL::GL)
#         endif()
#     endif()

#     # Copy project-local DLL next to the executable if present
#     if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll")
#         add_custom_command(TARGET test_viewer POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll" $<TARGET_FILE_DIR:test_viewer>
#         )
#         add_custom_command(TARGET test_model_bone POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll" $<TARGET_FILE_DIR:test_model_bone>
#         )
#         add_custom_command(TARGET test_model_scene POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll" $<TARGET_FILE_DIR:test_model_scene>
#         )
#         add_custom_command(TARGET test_scene_loc POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             "${CMAKE_CURRENT_SOURCE_DIR}/bin/libassimp-6.dll" $<TARGET_FILE_DIR:test_scene_loc>
#         )
#     endif()
# endif()