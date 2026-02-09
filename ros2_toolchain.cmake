# 先执行 source 脚本（需要绝对路径）
execute_process(
    COMMAND bash -c "source /opt/ros/humble/setup.bash && env"
    OUTPUT_VARIABLE ROS_ENV_OUTPUT
)

# 解析环境变量并设置到 CMake 中
string(REPLACE "\n" ";" ROS_ENV_LIST "${ROS_ENV_OUTPUT}")
foreach(ENV_LINE ${ROS_ENV_LIST})
    string(REGEX MATCH "^([^=]+)=(.*)$" ENV_MATCH "${ENV_LINE}")
    if(ENV_MATCH)
        set(ENV{${CMAKE_MATCH_1}} "${CMAKE_MATCH_2}")
    endif()
endforeach()