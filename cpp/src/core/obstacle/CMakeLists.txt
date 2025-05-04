
file(GLOB OBSTACLE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_obstacle STATIC ${OBSTACLE_SRC} )
                  
target_include_directories( core_obstacle PUBLIC
    ${CMAKE_SOURCE_DIR}/include/core/obstacle 
)

target_link_libraries( core_obstacle PRIVATE 
    core_edge  
    core_node 
    core_util 
)
