
file(GLOB WTSN_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( domain_wtsn STATIC ${WTSN_SRC} )
                  
target_include_directories( domain_wtsn PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries( domain_wtsn PRIVATE 
    core_util
    core_edge
)
