
file(GLOB GEOM_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_geom STATIC ${GEOM_SRC} )
                  
target_include_directories( core_geom PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)