
file(GLOB TRF_SI_2025_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

add_library( project_trfsi2025 STATIC ${TRF_SI_2025_SRC} )
                  
target_include_directories( project_trfsi2025 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries( project_trfsi2025 PRIVATE 
    domain_wtsn
)
