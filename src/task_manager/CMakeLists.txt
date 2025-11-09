# 编译任务管理器
# 设置包含目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

# 任务管理器源文件
set(TASK_MANAGER_SOURCES
    taskmanager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/task_manager/taskmanager.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/task_manager/moc_taskmanager.cpp
)

# 添加为库，可以被主程序链接
add_library(taskmanager STATIC ${TASK_MANAGER_SOURCES})

# 链接Qt库
target_link_libraries(taskmanager Qt6::Core Qt6::Widgets Qt6::WebSockets)