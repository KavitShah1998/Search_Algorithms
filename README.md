# Search_Algorithms
This repository contains implementation of graph-search algorithms in maze-like environments in C++

Algorithms implemented: Breadth-First Search, Depth-First Search, Dijkstra, A*, Weighted-A*

Breadth-First Search 
<p float="center">
<img src="Videos/gif/Maze_solver_with_bfs(1).gif" width="49%"/>
</p>

Depth-First Search 

<p float="center">
<img src="Videos/gif/Maze_solver_with_dfs.gif" width="49%"/>
</p>

Dijkstra's Algorithm 

<p float="center">
<img src="Videos/gif/Maze_solver_with_dijkstra.gif" width="49%"/>
</p>

A* 

<p float="center">
<img src="Videos/gif/Maze_solver_with_a_star.gif" width="49%"/>
</p>

Weighted-A*

<p float="center">
<img src="Videos/gif/Maze_solver_with_wa_star.gif" width="49%"/>
</p>


## Dependencies
You will need OpenCV library for visualizing this project

You can find the installing instruction for OpenCV4 on Ubuntu 18-04 [here](https://www.learnopencv.com/install-opencv-4-on-ubuntu-18.04/)

Don't forget to configure the OpenCV_DIR in the CMakeLists.txt with {path-to-your-OpenCV-library}

## Installation 

Create a build directory

        mkdir build

Go into the build directory

        cd build

Run the following commands to complete the build process

        cmake ..
        make


## Running
Once you have successfully build your project you can run the executables with the following commands from inside your build directory

DFS
         
         ./dfs
        
BFS

        ./bfs

Dijkstra

        ./dijkstra


A*

        ./a_star

Weighted-A*

        ./wa_star



## References & Useful resources

* Maze Generator for the code has been used from [here](https://scipython.com/blog/making-a-maze/)

* Installing [OpenCV4](https://www.learnopencv.com/install-opencv-4-on-ubuntu-18-04/) for Ubuntu 18.04


