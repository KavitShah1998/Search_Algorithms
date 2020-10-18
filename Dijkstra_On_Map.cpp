#include <opencv2/opencv.hpp>
#include<iostream>
#include<cstdlib>
#include<vector>
#include<unordered_set>
#include<queue>
using namespace std;
#define INF 9999


/*
CODE API 

Member functions of CLASS "path"


	priority_queue<node*, vector<node*>, node_cmp> open;  // a priority queue implementation of to-be-visited nodes 
	unordered_set<node*> closed;  // a hashmap implementation of the visited nodes;
	unordered_set<node*>::iterator it;

	path(vector<vector<int>> map); // To initialize the variables and call all the methods of the class
	
	void input(int start[2], int goal[2]);// To take input from the user / set input coordinates
	
	void Dijkstra(); // The method where all the computation is performed (This is where the Dijkstra algorithm runs)
	void displayOutputMap(); // This method is used to update the 'output_map' 2D vector for the final path found by Dijkstra
	std::vector<node*> getPath();			// returns the vector of path

	template <typename T>
	void display(vector<vector<T>> matrix);  // Displaying the 2D maps for visualization
	void printResults();


Non-member functions
	std::vector<std::vector<int>> getMapFromImage(cv::Mat _image) 			// converts the grayscale maze image into a binary 2-d vector of grid-map
	void colorMap(cv::Mat _color_image, std::vector<node*> _output_path)     // does the display function of simulating motion from start-point to goal-point
	void makeCircles(cv::Mat& _color_image, int start[2], int goal[2])		// encircles start point and goal point with blue and red circle respectively

*/


struct node {
	int x;
	int y;
	
	float g;
	
	node* prev;
};

class path {
private:
	vector<vector<int>> copy; 				// This is used to create a local copy of the map accesible to all the functions of this class
	vector<vector<node*>> adr; 				// This is used to store address of nodes for each element of the input map (2D vector)
	vector<vector<char>> output_map; 			// This is used to store and display the optimum path after the output_map of Weighted A* 
	vector<vector<char>> progress_map; 		// This is used to store and display the exploration process of Weighted A*

	vector<node*> output_path; 

	int i_start{0}; // x coordinate of start node 
	int j_start{0}; // y coordinate of start node 
	int i_goal{0};  // x coordinate of end node 
	int j_goal{0};  // y coordinate of end node 

	bool found_flag = false;  // this flag will be set to true if the element is found and we will exit of the while loop in Astar method

	void createNodes();  // To create nodes for each element of input map and store its address in addr 

	void exploreNeighbours(int curr_i, int curr_j);  // Search the neighbours of a given node and add them to the queue if they are valid
	bool is_valid(int i, int j, int neighbour_index); // to check the validity of the neighbouring nodes
	void updateDistance(int i, int j, int neighbour_index); // To update the distance of a neighbouring node if the node is a valid node
	vector<vector<char>> initialize(vector<vector<int>> cop);  // To initialize the 2D vectors output_map and progress map

	// This struct node is used to adjust the 'priority_queue' with nodes as element .. prioritizes elements based on their g-values
	struct node_cmp : public std::binary_function<node*, node*, bool> 
	{
		bool operator() (const node* a, node* b)const
		{
			return a->g > b->g;
		}
	};

public:
	/*  ***********************  */
	/*    CODE API - member methods   */

	priority_queue<node*, vector<node*>, node_cmp> open;  		// a priority queue implementation of to-be-visited nodes 
	unordered_set<node*> closed;  								// a hashmap implementation of the visited nodes;
	unordered_set<node*>::iterator it;

	path(vector<vector<int>> map); 								// To initialize the variables and call all the methods of the class
	
	void input(int start[2], int goal[2]);						// To take input from the user / set input coordinates
	
	void Dijkstra(); 											// The method where all the computation is performed (This is where the Dijkstra algorithm runs)
	void displayOutputMap(); 									// This method is used to update the 'output_map' 2D vector for the final path found by Dijkstra
	std::vector<node*> getPath();								// returns the vector of path

	template <typename T>
	void display(vector<vector<T>> matrix);  // Displaying the 2D maps for visualization
	void printResults();	

	/*  ***********************  */
};



std::vector<std::vector<int>> getMapFromImage(cv::Mat _image);			// converts the grayscale maze image into a binary 2-d vector of grid-map
void colorMap(cv::Mat _color_image, std::vector<node*> _output_path);    // does the display function of simulating motion from start-point to goal-point
void makeCircles(cv::Mat& _color_image, int start[2], int goal[2]);		// encircles start point and goal point with blue and red circle respectively


/* ABOUT FUNCTON 
// This constructor handles all the function calls of the class
// Responsibility of this function:
	// Take map as input from the main function
	// Initiailze all the variables
	// call the createNodes method*/
path::path(vector<vector<int>> map)
{
	// initializing the copy vector:: The 2D vector 'map' is a local variable, so we store its value in copy; a variable of the class which is available to all the functions of the class
	copy = map;


	cout << "\n\n DIjKSTRA's Algorithm\n";
	// Initializing the adress vector
	for (int i = 0; i < map.size(); i++)
	{
		vector<node*> tempvec;
		for (int j = 0; j < map[1].size(); j++)
		{
			tempvec.push_back(NULL);
		}
		adr.push_back(tempvec);
	}

	createNodes();
	output_map = initialize(copy);
	progress_map = initialize(copy);

	
}


// This function is called by the constructor to create a matrix of same size as that of the given maze but to store the adresses of each nodes
void path::createNodes() 
{
	node* n = NULL;
	for (int i = 0; i < copy.size(); i++)
	{
		for (int j = 0; j < copy[1].size(); j++)
		{
			n = new node;
			
			n->g = INF;
			n->x = i;
			n->y = j;
			n->prev = NULL;
			adr[i][j] = n; // storing the address of new node for discretized point [x,y] in adr[i][j]
		}
	}
}

// This function is to get the start and end co-irdinates from the user
void path::input(int start[2], int goal[2]) 
{
	int row = copy.size() - 1;
	int col = copy[1].size() - 1;

	//cout << "\n Enter coordinates of start:x = ";
	//cin >> i_start-1;
	i_start = start[0];
	//cout << "\n Enter coordinates of start:y = ";
	//cin >> j_start-1;
	j_start = start[1];
	//cout << "\n Enter coordinates of end:x = ";
	//cin >> i_goal-1;
	i_goal = goal[0];
	//cout << "\n Enter coordinates of end:y = ";
	//cin >> j_goal-1;
	j_goal = goal[1];

	// checking the input arguments
	if (i_start<0 || i_start>row || j_start <0 || j_start > col || i_goal<0 || i_goal>row || j_goal <0 || j_goal > col)
	{
		cerr << "\n Error: Input Co-ordinated out of range";
		exit;
	}
	else if (copy[i_start][j_start] == 1 || copy[i_goal][j_goal] == 1)
	{
		cerr << "\n Input data not in free space";
		exit;
	}

	cout << "\n Start Coordinates: (" << i_start << " , " << j_start << ")";
	cout << "\n End Coordinates: (" << i_goal << " , " << j_goal << ") \n";
	


	node* temp = adr[i_start][j_start];// Also update the g(x) of the start node here itself.
	temp->g = 0;
}


/*For small 2D vector maps, you can uncomment the following line of code ot print the maps in the terminal*/
void path::printResults(){
	cout << "\n\n\n Input Map:\n ";
	display(copy);
	
	cout << "\n\n\n Map exploration by Dijkstra:\n\n";
	display(progress_map);

	output_map[i_start][j_start] = 'S';
	output_map[i_goal][j_goal] = 'G';
	display(output_map);
}

/*Goal of this function is to store the start element in the 'Open' set which is a priority queue of all the nodes. The priority queue in c++ gets updated when any new element is inserted into the queue
	 We pop the top element of the priority queue as that has the MINIMUM g(x) (i.e cost to go) and store that top element in the 'CLOSED' set which is a collection of all the visited nodes. We then EXPLORE its neighbours 
	 While EXPLORING the neighbours of the given node, we CHECK if that node is an OBSTACLE or is already been visited and exists in the CLOSED set (to ease this task of searching an element in a container, we implement CLOSED as an unordered set)
	 While EXPLORING if a Node's g(x) gets updated, we store the address of previous node which called this node in 'prev' pointer inside the struct 'node'
	 ALl these steps are executed in a loop until the end goal node is encountered.*/
void path::Dijkstra()
{
	// We initialize the OPEN queue with our start node.
	open.push(adr[i_start][j_start]);
	int i_curr = 0;
	int j_curr = 0;
	found_flag = false;
	while (!open.empty() && found_flag != true)
	{
		node* temp = open.top();
		open.pop();
		i_curr = temp->x;
		j_curr = temp->y;

		progress_map[i_curr][j_curr] = '"';

		if (i_curr == i_goal && j_curr == j_goal)
		{
	
			found_flag = true;  // set false true and terminate the loop

		}
		else
		{
			exploreNeighbours(i_curr, j_curr);
		}

	}


}


// This function checks the neighbours of given nodes if they are in free space, have already been visited, are out of bounds or lie in obstacles
	// If the node is a free space, it updates its distance g 
void path::exploreNeighbours(int curr_i, int curr_j)
{

	int i;
	int j;

	// North
	if (is_valid(curr_i, curr_j, 0) == true)
	{
		i = curr_i - 1;
		j = curr_j;
		//cout << "\n North Valid";
		updateDistance(i, j, 0);

	}


	// South
	if (is_valid(curr_i, curr_j, 1) == true)
	{
		i = curr_i + 1;
		j = curr_j;
		//cout << "\n South Valid";
		updateDistance(i, j, 1);

	}

	// East
	if (is_valid(curr_i, curr_j, 2) == true)
	{
		i = curr_i;
		j = curr_j + 1;
		// << "\n East Valid";
		updateDistance(i, j, 2);
	}

	// West
	if (is_valid(curr_i, curr_j, 3) == true)
	{
		i = curr_i;
		j = curr_j - 1;
		//cout << "\n West Valid";
		updateDistance(i, j, 3);

	}

}

//This function checks the neighbours of given nodes if they are in free space, have already been visited, are out of bounds or lie in obstacles
	//Neighbour_index = {N,0} {S,1}  {E,2}  {W,3}
bool path::is_valid(int i, int j, int neighbour_index)
{

	int row = copy.size() - 1;
	int col = copy[1].size() - 1;
	switch (neighbour_index)
	{
	case 0:  // check north
		if (i <= 0)
		{

			return false;
		}
		else
		{
			it = closed.find(adr[i - 1][j]);
			if (copy[i - 1][j] == 1 || it != closed.end())  // upper element is an obstacle  OR the upper element has already been visited
			{
				return false;
			}
			else
			{
				//cout << "\n North Valid";
				return true;
			}
		}
		break;


	case 1:  // check south
		if (i >= row)
		{
			//cout << "\n South InValid";
			return false;
		}
		else
		{
			it = closed.find(adr[i + 1][j]);
			if (copy[i + 1][j] == 1 || it != closed.end())  // lower element is an obstacle  OR the lower element has already been visited
			{
				//cout << "\n South NotValid";
				return false;
			}
			else
			{
				//cout << "\n South Valid";
				return true;
			}
		}
		break;


	case 2:  // check east
		if (j >= col)
		{

			return false;
		}
		else
		{
			it = closed.find(adr[i][j + 1]);
			if (copy[i][j + 1] == 1 || it != closed.end())  // east element is an obstacle  OR the east element has already been visited
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		break;
	case 3:  // check west
		if (j <= 0)
		{

			return false;
		}
		else
		{
			it = closed.find(adr[i][j - 1]);
			if (copy[i][j - 1] == 1 || it != closed.end())  // west element is an obstacle OR the west element has already been visited
			{
				return false;
			}
			else
			{
				return true;
			}
		}
		break;

	}

}


// This function updates a particular neighbour's distance as given by the neighbour_index
	//Neighbour_index = {N,0} {S,1}  {E,2}  {W,3}
void path::updateDistance(int i, int j, int neighbour_index)  	// here the i and j represent index of Neighbours and the parent node
{
	node* temp = NULL;
	node* parent = NULL;


	switch (neighbour_index)
	{
	case 0: // update North
		temp = adr[i][j];
		parent = adr[i + 1][j]; 				// parent is the node whose north element we are updating here. Thus it will lie exactly BELOW it


		if (temp->g > parent->g + 1) 		   // update distance if and only if current distance is more than parent distance + cost to go
		{
			temp->g = parent->g + 1;   	       // update the cost to node
			
			temp->prev = parent;        	   // store the prev of current node as parent of current
			open.push(temp); 				   // push this node into the priority queue
		}
		break;

	case 1: // update South
		temp = adr[i][j];
		parent = adr[i - 1][j]; 			   // parent is the node whose south element we are updating here. Thus it will lie exactly ABOVE it


		if (temp->g > parent->g + 1)  		   // update distance if and only if current distance is more than parent distance + cost to go
		{
			temp->g = parent->g + 1;		   // update the cost to node
			
			temp->prev = parent;        	   // store the prev of current node as parent of current
			open.push(temp);  				   // push this node into the priority queue
		}
		break;

	case 2: // update East
		temp = adr[i][j];
		parent = adr[i][j - 1]; 			   // parent is the node whose east element we are updating here. Thus it will lie exactly to its left


		if (temp->g > parent->g + 1)  		   // update distance if and only if current distance is more than parent distance + cost to go
		{
			temp->g = parent->g + 1;	 	   // update the cost to node
			
			temp->prev = parent;         	   // store the prev of current node as parent of current
			open.push(temp);  // push this node into the priority queue
		}
		break;

	case 3: // update West
		temp = adr[i][j];
		parent = adr[i][j + 1]; 			   // parent is the node whose west element we are updating here. Thus it will lie exactly to its right


		if (temp->g > parent->g + 1)  		   // update distance if and only if current distance is more than parent distance + cost to go
		{
			temp->g = parent->g + 1;	 	   // update the cost to node
			
			temp->prev = parent;         	   // store the prev of current node as parent of current
			open.push(temp);  // push this node into the priority queue
		}
		break;
	}
}


vector<vector<char>> path::initialize(vector<vector<int>> cop)
{
	// iniliazing the 2D vectors 'output_map' and 'progress_map'
	vector<vector<char>> local_vector;
	for (int i = 0; i < cop.size(); i++)  
	{
		vector<char> op1;
		for (int j = 0; j < cop[1].size(); j++)
		{
			op1.push_back('0');
		}
		local_vector.push_back(op1);
	}

	// creating our map in output_map
	for (int i = 0; i < local_vector.size(); i++)
	{
		for (int j = 0; j < local_vector[1].size(); j++)
		{
			if (copy[i][j] == 1)
			{
				local_vector[i][j] = 'X';  // X represents obstacle 
			}
			else if (copy[i][j] == 0)
				local_vector[i][j] = '0';  // O represents free space
		}
	}
	return local_vector;

}



/* This function is imp*/
// This method is used to update the 'output_map' 2D vector  & to store the optimum path from A to B
void path::displayOutputMap()
{

	int i_l = i_goal;
	int j_l = j_goal;
	node* curr = NULL;
	node* parent = NULL;
	//cout << "\n\n\n Optimum path by Dijkstra\n\n";
	while (true)
	{

		output_map[i_l][j_l] = '.';  // Dot represents the optimum path
		if (i_l == i_start && j_l == j_start)
		{
			break;
		}
		curr = adr[i_l][j_l];
		parent = (curr->prev);

		output_path.insert(output_path.begin(),curr);
		i_l = parent->x;
		j_l = parent->y;
	}

	//printResults();
	cout<<"\n\n\n Total cost of Optimum path: "<< adr[i_goal][j_goal]->g<<"\n\n";

}


// This function is used to display the 2D vectors for visualization which contain the optimum path info and exploration of Dijkstra
template <typename T >
void path::display(vector<vector<T >> matrix)
{

	for (int i = 0; i < matrix.size(); i++)
	{
		cout << "\n";

		for (int j = 0; j < matrix[1].size(); j++)
		{
			cout << "\t " << matrix[i][j];
		}
	}
}

/* Getter function for returning the final output path as a vector of nde* */
std::vector<node*> path::getPath(){
	return output_path;
}


/*
Get the 2d vector from image of the maze*/
std::vector<std::vector<int>> getMapFromImage(cv::Mat _image){
	std::vector<std::vector<int>> grid;
	int r = _image.rows;
	int c = _image.cols;

	for (int i=0; i<r; i++){
		std::vector<int> curr_row;
		for (int j=0; j<c; j++){
			curr_row.push_back(  (_image.at<uchar>(i,j) == 255)? 0 : 1 );
		}
		grid.push_back(curr_row);
	}

	return grid;
}

/*
Simulation of results using opencv, input maze image and the final output_path*/
void colorMap(cv::Mat _color_image, std::vector<node*> _output_path){
	int size = _output_path.size();


	std::string window_name = "Maze solver with dijkstra";
	cv::namedWindow(window_name);
	cv::Vec3b color(255, 0, 0);

	// wait for the user to press teh key for coninuing the simulation
	std::cout << " Press a key to continue. \n"; 	
	cv::imshow(window_name, _color_image);
	int k = cv::waitKey(0);    						 // wait for use to press a key

	if(!size){
		std::cout << "\n No path found \n";
		return;
	}	

	for (int i=0; i<size; i++){
		//_color_image.at<cv::Vec3b>(cv::Point((_output_path[i])->y, (_output_path[i])->x)) = color;   // ..... cv::point(y,x)
		cv::circle(_color_image, cv::Point((_output_path[i])->y, (_output_path[i])->x),1, cv::Scalar(255,0,255), -1, 8, 0);     // ..... cv::point(y,x)
		cv::imshow(window_name, _color_image);
		int k = cv::waitKey(1);     				// delay in display of 0.001 sec
		if(k == 27)
			break;
	}
	int start[2] = {_output_path[0]->x, _output_path[0]->y};
	int goal[2]  = {_output_path[size-1]->x , _output_path[size-1]->y};
	makeCircles(_color_image, start, goal);
	cv::imshow(window_name, _color_image);
	cv::waitKey(0);    							 // wait for user to press a key
	cv::destroyAllWindows();
}

/*
Mark start and end locations on the map as circles*/
void makeCircles(cv::Mat& _color_image, int start[2], int goal[2]){
	auto _yellow = cv::Scalar(0,255,0);
	auto _red    = cv::Scalar(0, 0, 255);
	cv::circle(_color_image, cv::Point(start[1],start[0]), 10,_yellow, -1, 8, 0);    // ..... cv::point(y,x)
	cv::circle(_color_image, cv::Point(goal[1], goal[0]), 10, _red, -1, 8, 0);     // ..... cv::point(y,x)
}


int main()
{

	//int start[2] = {1,1};
	//int goal[2] = {6,7};
	
	int start[2] = {40, 40};
	int goal[2] = {700, 360};
		// get image
	cv::Mat maze = cv::imread("/home/kshah/Search_Algorithms/maze_generator/maze.jpg");

	if(maze.empty()){
		std::cout << "Image not found " << "\n";
		return 0;
	}

    cv::Mat maze_gray;

	//resize image
	cv::resize(maze, maze, cv::Size(), 0.5, 0.5);

	//cvt image to grayscale
	cv::cvtColor(maze, maze_gray, cv::COLOR_BGR2GRAY);

	// encircle start(blue) and goal(red) locations on the image
	makeCircles(maze, start, goal);

	// get the grid-map from gray images
	vector<vector<int>> map1 = getMapFromImage(maze_gray);

	/*
	vector<vector<int>> map1 = {{0,0,1,0,0,0,0,0},
								{0,0,1,0,0,0,0,0},
								{0,0,1,0,1,1,1,1},
								{0,0,1,0,0,0,0,0},
								{0,0,0,0,1,0,0,0},
								{0,0,0,0,1,0,0,0},
								{1,1,1,1,1,0,0,0},
								{0,0,0,0,1,0,0,0}};  */

	// create path object
	path p1(map1);

	// pass input start and end states to the object
	p1.input(start,goal);

	//run the DFS function
	p1.Dijkstra();
	p1.displayOutputMap();
	// print results in the terminal as grid-maps
	//p1.display_map(map1);
	//p1.printResults();
	


	//obtain the output path from the object
	std::vector<node*> output_path = p1.getPath();

	std::cout << "Output path size : " << output_path.size() << "\n";
	// simulate the DFS function
	colorMap(maze, output_path);


	return 0;
}
