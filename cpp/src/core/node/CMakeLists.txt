
file(GLOB NODE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_node STATIC ${NODE_SRC} )
                  
target_include_directories( core_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)