
file(GLOB FLP_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( domain_flp STATIC ${FLP_SRC} )
                  
target_include_directories( domain_flp PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries( domain_flp PRIVATE 
    core_net 
)
