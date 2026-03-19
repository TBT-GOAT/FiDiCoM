
# アプリケーション名
project(main_app)

# 実行ファイルの作成
add_executable(main main.cpp)
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# include for local package
target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/include)

# リンクするライブラリ
target_link_libraries(main PRIVATE 
    CGAL::CGAL
    OpenMP::OpenMP_CXX
    pagmo
    core_edge
    core_geom
    core_net
    core_node
    core_obstacle
    core_region
    core_util
    core_weight_mapper
    domain_flp
    domain_wtsn
    project_trfsi2025
)