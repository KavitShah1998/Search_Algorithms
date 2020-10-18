cmake_minimum_required(VERSION 3.10.2)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR /home/kshah/installation/OpenCV-master/installation/OpenCV-master/lib/cmake/opencv4)
project(Search_Algorithms )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(dfs DFS_On_Map.cpp )
target_link_libraries( dfs ${OpenCV_LIBS} )

add_executable(bfs BFS_With_Map.cpp)
target_link_libraries (bfs ${OpenCV_LIBS} )

add_executable(dijkstra Dijkstra_On_Map.cpp)
target_link_libraries (dijkstra ${OpenCV_LIBS})

add_executable (a_star A_Star.cpp)
target_link_libraries (a_star ${OpenCV_LIBS})

add_executable (wa_star Weighted_A_star.cpp)
target_link_libraries (wa_star ${OpenCV_LIBS})