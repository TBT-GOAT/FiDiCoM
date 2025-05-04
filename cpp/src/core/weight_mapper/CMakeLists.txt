
file(GLOB WEIGHT_MAPPER_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_weight_mapper STATIC ${WEIGHT_MAPPER_SRC} )
                  
target_include_directories( core_weight_mapper PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)

target_link_libraries( core_weight_mapper PRIVATE 
    core_edge 
    core_net
    core_node 
    core_obstacle 
)