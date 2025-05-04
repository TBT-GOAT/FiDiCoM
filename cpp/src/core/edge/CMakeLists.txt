
file(GLOB EDGE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_edge STATIC ${EDGE_SRC} )
                  
target_include_directories( core_edge PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)

target_link_libraries( core_edge PRIVATE 
    core_net 
    core_node 
    core_obstacle 
)