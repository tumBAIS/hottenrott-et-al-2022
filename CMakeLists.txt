cmake_minimum_required(VERSION 3.12)
project(00_Code)

include_directories("C:/Program Files/boost/boost_1_72_0")
link_directories("C:/Program Files/boost/boost_1_72_0/stage/lib")

include_directories("C:/gurobi811/win64/include")
link_directories("C:/gurobi811/win64/lib")

set(SOURCE_FILES main.cpp
        instance_list.hpp
        instance_reader.hpp
        preprocessing.hpp
        preprocess_scenario.hpp
        initial_solution.hpp
        route_finder.hpp
        master_problem.hpp
        aux_node.hpp
        node_solver.hpp
        mip_ub_solver.hpp
        branching.hpp
        bandp.hpp
        output_writer.hpp
        C:/gurobi811/win64/include/gurobi_c++.h)
add_executable(00_Code ${SOURCE_FILES})

set_property(TARGET ${target_name} PROPERTY CXX_STANDARD 17)
target_link_libraries (00_Code gurobi_c++md2017)
target_link_libraries (00_Code gurobi81)