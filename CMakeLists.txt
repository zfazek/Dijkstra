set (CMAKE_CXX_FLAGS "-std=c++11 -O2 -Werror -Wall")

add_executable(dijkstra
    graph.cc
    dijkstra.cc
)
