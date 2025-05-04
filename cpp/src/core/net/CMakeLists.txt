
file(GLOB NET_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_net STATIC ${NET_SRC} )
                  
target_include_directories( core_net PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)

target_link_libraries( core_net PRIVATE 
    core_edge 
    core_geom 
    core_node 
    core_obstacle 
    core_util 
    core_weight_mapper
)
