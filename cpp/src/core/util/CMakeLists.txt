
file(GLOB UTIL_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_util STATIC ${UTIL_SRC} )
                  
target_include_directories( core_util PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)