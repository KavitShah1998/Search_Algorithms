//om
#include <vector>
#include <iostream>
#include <cstdlib>
#include <unordered_set>
#include <stack>
#include <opencv2/opencv.hpp>
using namespace std;
# define INF 9999


/*
CODE API 

Member functions of CLASS "path"


	path(vector<vector<int>> map);  		// initialize the variables
	void input(int start[0], int goal[0]);  // Take the data about the start and end coordinates as input from the user
	
	void DFS();  							// Executes the main Depth-First_Search Algorithm

	template < typename T1>  				// creating a template to print int as well as char type data of vectors
	void display_map(vector<vector<T1>>);   // This function prints the occupancy_maps(2D vectors) for better visualization of data 
	void makePath();						// updates the final path in the 2D vector output path once BFS Search has been performed;
	std::vector<nde*> getPath();			// returns the stack of path

	void printStack(stack<nde*> H); 		// prints out the stack


Non-member functions
	std::vector<std::vector<int>> getMapFromImage(cv::Mat _image) 			// converts the grayscale maze image into a binary 2-d vector of grid-map
	void colorMap(cv::Mat _color_image, std::vector<nde*> _output_path)     // does the display function of simulating motion from start-point to goal-point
	void makeCircles(cv::Mat& _color_image, int start[2], int goal[2])		// encircles start point and goal point with blue and red circle respectively

*/


// define the struct node;
struct nde {
	// this struct will have variable to store x-cord, y-cord, addr of 'parent' node

	int x;
	int y;
	float cost;
	nde* parent;
};


class path {
private:
	// declaring here the main variables required in the code which would be used by all the functions of the class 'path'
	vector<vector<int >> copy;  				// To store the data of map obtained from user through main();
	vector<vector<char>> output_map;  			// A 2D vector to show the optimal path on map
	vector<vector<char>> progress_map; 		    // A vector to visualize how does this algorithm GROW in th search space
	vector<vector<nde*> > addr;  				// vector to store address of each new node;

	std::stack<nde*> S; 					// a Stack for storing the elements which need to be visited next
    std::vector<nde*> output_path;  		// a Stack for storing the nodes on the output path with goal at stack bottom and start point at stack top

	int i_start;  				// x-co-ordinate of start point
	int j_start;  				// y-coordinate of start point
	int i_goal;  				// x- coordinate of end point
	int j_goal;					// y- coordinate of end point

	int row;  				// rows in the map
	int col;  				// columns in the map
	
	int obst_pixel = 0;

	float diag_cost = 1.4;					 // cost to traverse diagonally
	float non_diag_cost = 1; 				 // cost to traverse horizontally / vertically

	unordered_set<nde*> Open;  		// defining an Open list for storing address of all the Potential Elements To be Visited ( This has the same data as S, it is used to search elements in the stack with O(1) complexity as searching in a stack is costly)
	unordered_set<nde*> Closed;     // defining a Closed List for storing address of all the Visited Elements

	unordered_set<nde*>::iterator it_Open;    // defining an iterator for the open list
	unordered_set<nde*>::iterator it_Closed;  // defining an iteretor for the closed list

	void createNodes();				    	// This function creates nodes for all the elements of the maps;

	void searchForNeighbours(nde*);  		// Search the neighbours of a given element and add them to the stack S if they are valid
	bool isValid(nde*, int);
	bool add2Stk(nde*); 					// adds the element in the stack S as well as Open list if it not yet added
	void removeFromStk(nde*); 				// removes the popped element from stack S and Open lists and adds them to the Closed list

public:
	/*  ***********************  */
	/*    CODE API - member methods   */

	path(vector<vector<int>> map);  		// initialize the variables
	void input(int start[0], int goal[0]);  // Take the data about the start and end coordinates as input from the user
	
	void DFS();  							// Executes the main Depth-First_Search Algorithm

	template < typename T1>  				// creating a template to print int as well as char type data of vectors
	void display_map(vector<vector<T1>>);   // This function prints the occupancy_maps(2D vectors) for better visualization of data 
	void makePath();						// updates the final path in the 2D vector output path once BFS Search has been performed;
	std::vector<nde*> getPath();			// returns the vector of path

	void printStack(stack<nde*> H); 		// prints out the stack
	void printResults();

	/*  ***********************  */
};


std::vector<std::vector<int>> getMapFromImage(cv::Mat _image);			// converts the grayscale maze image into a binary 2-d vector of grid-map
void colorMap(cv::Mat _color_image, std::vector<nde*> _output_path);    // does the display function of simulating motion from start-point to goal-point
void makeCircles(cv::Mat& _color_image, int start[2], int goal[2]);		// encircles start point and goal point with blue and red circle respectively



/* ABOUT FUNCTON 
// This constructor handles all the function calls of the class
// Responsibility of this function:
	// Take map as input from the main function
	// Initiailze all the variables
	// call the createNodes method*/
path::path(vector<vector<int>> map) {
	

	copy = map; 						 // store the contents of map in class 'path' which is local for all members of this class

	row = copy.size() - 1;
	col = copy[1].size() - 1;

	cout << "\n\n Depth First Search:\n";

	for (int i = 0; i < map.size(); i++)
	{
		vector<char> temp_output_map;   // to store rows of output path as vectors
		vector<char> temp_progress_map;  // to store rows of progress map as vectors
		vector<nde*> temp_addr;  		 // to store rows of address of nodes as vectors

		for (int j = 0; j < map[1].size(); j++)
		{
			if (map[i][j] == 1)
			{
				temp_output_map.push_back('X');
				temp_progress_map.push_back('X');
			}
			else
			{
				temp_output_map.push_back('0');
				temp_progress_map.push_back('0');
			}
			temp_addr.push_back(NULL);
			
		}
		output_map.push_back(temp_output_map);
		progress_map.push_back(temp_progress_map);
		addr.push_back(temp_addr);
	}  // here we have initialized the vectors to visualize the data using X and 0s; where X represents obstacle and 0 represents free space;

	createNodes();

	
}


/*For small 2D vector maps, you can uncomment the following line of code ot print the maps in the terminal*/
void path::printResults()
{
	output_map[i_goal][j_goal] = 'G';
	output_map[i_start][j_start] = 'S';

	cout << "\n\n Input Map:\n";
	display_map(copy);
	// display the results

	cout << "\n\n Displaying Progress_map:\n\n";
	display_map(progress_map);

	cout << "\n\n Displaying Output_map:\n\n";
	display_map(output_map);
}



// This function takes in input from the user about the start and end goal locations
// Responsibility of input()
	//Take x_start and y_start from user;
	//Take x_end   and y_end   from user;
void path::input(int start[2], int goal[2])
{

	//cout << "\n Enter the co-odinates of the start point (X & Y):";
	//cin >> i_start>>j_start;
	i_start = start[0];
	j_start = start[1];

	addr[i_start][j_start]->cost = 0;
	//cout << "\n Enter the co-ordinates of the end point (X & Y):";
	//cin >> i_goal >> j_goal;
	i_goal = goal[0];
	j_goal = goal[1];

	cout<<"\n\n Start Co-ordinates: ("<< i_start<<" , "<<j_start<<")";
	cout<<"\n End Co-ordinates: ("<< i_goal<<" , "<<j_goal<<") \n";
}



//Responsibility of this node:
	// create nodes for each element of input map and store their address in addr 2D vector
void path::createNodes()
{

	nde* n = NULL;
	for (int i = 0; i <= row; i++)
	{
		for (int j = 0; j <= col; j++)
		{
			n = new nde;
			n->x = i;
			n->y = j;
			n->cost = INF;
			n->parent = NULL;
			addr[i][j] = n;
		}
	}
}



// Responsibility of this function:
	// Insert Start node into Stack S
	// Until Stack is empty search and add neighbours of top element to the stack if not added already
void path::DFS()
{
	nde* curr = addr[i_start][j_start];
	add2Stk(curr);
	
	
	while (!S.empty())
	{
		curr = S.top();
		if (curr->x == i_goal && curr->y == j_goal)
		{
			break;
		}
		progress_map[curr->x][curr->y] = '#';
		removeFromStk(curr);
		searchForNeighbours(curr);

	}
	makePath();
}


// Print the elements of the Stack ( For testing )
void path::printStack(stack<nde*> H)
{
	while (!H.empty())
	{
		cout << "\nStack (i,j)" << H.top()->x << "," << H.top()->y;
		H.pop();
	}
	cout << "\n Stack is empty";
}


// Responsibility of this function
	// If the element at Top of the Stack S is not yet added in the Closed List then:
	// Pop ele from Stack S
	// Pop ele from Open List
	// Push ele into the Closed List
void path::removeFromStk(nde* ele)
{

	it_Open = Open.find(ele);
	it_Closed = Closed.find(ele);
	if (it_Open != Open.end()) 			 // element to  be removed exists in Open
	{
		S.pop();
		Open.erase(it_Open);
		Closed.insert(ele);
	}
	else if (it_Closed != Closed.end())  // To keep a check on unexpected cases 
	{
		cout << "\n Wierd Case encountered";
	}

}


// Responsibility of this function:
	// If an element being searched does not exist in Open List as well as Closed List:
	// Add the element to the Stack S
	// Add the element to the Open List
bool path::add2Stk(nde* ele)
{

	it_Open = Open.find(ele);
	it_Closed = Closed.find(ele);
	if (it_Open == Open.end() && it_Closed == Closed.end())  // element not found; Therefore add to Stack S as well as Open list
	{
		S.push(ele);
		Open.insert(ele);
		return true;
	}
	else
	{
		return false;
	}
}


//Responsibility of this function:
	//Given a node, search if its neighbours are valid
	// If valid, add them to the Stack S
void path::searchForNeighbours(nde* ele)
{

	nde* temp;
	bool condition_check;
	int ele_i = ele->x;
	int ele_j = ele->y;

	// Seach North
	if (isValid(ele, 0) != false)
	{
		temp = addr[ele_i - 1][ele_j];
		condition_check = add2Stk(temp);
		if (condition_check == true)
		{
			temp->parent = ele;
			temp->cost = ele->cost + non_diag_cost;
		}

		//cout << "\n North N Added";
	}

	// Search South
	if (isValid(ele, 1) != false)
	{
		temp = addr[ele_i + 1][ele_j];
		condition_check = add2Stk(temp);
		if (condition_check == true)
		{
			temp->parent = ele;
			temp->cost = ele->cost + non_diag_cost;
		}
		//cout << "\n South N Added";
	}

	// Search East
	if (isValid(ele, 2) != false)
	{
		temp = addr[ele_i][ele_j + 1];
		condition_check = add2Stk(temp);
		if (condition_check == true)
		{
			temp->parent = ele;
			temp->cost = ele->cost + non_diag_cost;
		}
		//cout << "\n East N Added";
	}

	// Search West
	if (isValid(ele, 3) != false)
	{
		temp = addr[ele_i][ele_j - 1];
		condition_check = add2Stk(temp);
		if (condition_check == true)
		{
			temp->parent = ele;
			temp->cost = ele->cost + non_diag_cost;
		}
		//cout << "\n West N Added";
	}
}

// Here We check if a particular neighbour of the node is valid or not
// NeighBour Indices:: {N=0}, {S=1}, {E=2}, {W=3};


//Responsibility of this node:
// Check if a neighbour of node particularlu is not out of bounds
// Check if a neighbour of a node is not an obstacle
bool path::isValid(nde* ele, int neighbour_index)
{
	int x = ele->x;
	int y = ele->y;
	progress_map[x][y] = '"';


	switch (neighbour_index)
	{
	case 0:						// checking North element
		if (x <= 0)  					// north element out of bound
		{
			return false;
		}
		else
		{
			return (copy[x-1][y] != 1);
		}
	case 1:						// checking South element
		if (x >= row)  					// south element out of bound
		{
			return false;
		}
		else
		{
			return (copy[x+1][y] != 1);
		}

	case 2:						// checking East element
		if (y >= col)  					// East element out of bound
		{
			return false;
		}
		else
		{
			return (copy[x][y+1] != 1);
		}
	case 3:						// checking west element
		if (y <= 0)  					// west element out of bound
		{
			return false;
		}
		else
		{
			return (copy[x][y-1] != 1);
		}
	}

}


//Responsibility of this function
	// Display the map for visualization
template <typename T1>
void path::display_map(vector<vector<T1>> x)
{
	int row = x.size();
	int col = x[0].size();
	for (int i = 0; i < row; i++)
	{
		cout << "\n";
		for (int j = 0; j < col; j++)
		{
			cout << "\t" << x[i][j];
		}
	}
	std::cout << "\n";
}


//Responsibility of this function 
	// Back trace the optimum path from end node to start node and make changes in the output_map 2D vector
void path::makePath()
{
	nde* temp = addr[i_goal][j_goal];
	nde* start_node = addr[i_start][j_start];
	
	while (temp != NULL)
	{
		output_map[temp->x][temp->y] = '.';
		//cout << "\n (i,j)= " << temp->x << "," << temp->y;
		output_path.insert(output_path.begin(), temp);
		temp = temp->parent;
	}


	/* UNCOMMENT THE FOLLOWING LINE OF CODE IF YOU WANT TO SEE THE FINAL PATH FROM START TO GOAL COMPUTED BY THE ALGORITHM*/
	/*std::cout << "*********** \n Displaying output path \n";
	auto it = output_path.begin();
	while(it!= output_path.end()){
		std::cout << (*(it))->x << " - " << (*(it))->y << "\n";
		it++; 
	}

	std::cout << " *********** \n";
	*/

	cout << "\n Total Cost to reach goal (" << i_goal << "," << j_goal << ") = " << addr[i_goal][j_goal]->cost<<"\n\n\n";
}


/* Getter function for returning the final output path as a vector of nde* */
std::vector<nde*> path::getPath(){
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
void colorMap(cv::Mat _color_image, std::vector<nde*> _output_path){
	int size = _output_path.size();
	

	std::string window_name = "Maze solver with DFS";
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
	int start[2] = {40,40}; 
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

	// create path object
	path p1(map1);

	// pass input start and end states to the object
	p1.input(start,goal);

	//run the DFS function
	p1.DFS();

	// print results in the terminal as grid-maps
	//p1.display_map(map1);
	//p1.printResults();
	


	//obtain the output path from the object
	std::vector<nde*> output_path = p1.getPath();

	// simulate the DFS function
	colorMap(maze, output_path);

	return 0;
}
