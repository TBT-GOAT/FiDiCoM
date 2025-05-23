
file(GLOB REGION_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( core_region STATIC ${REGION_SRC} )
                  
target_include_directories( core_region PUBLIC
    ${CMAKE_SOURCE_DIR}/include 
)

# target_link_libraries( core_region PRIVATE 
#     core_edge  
#     core_node 
#     core_util 
# )
