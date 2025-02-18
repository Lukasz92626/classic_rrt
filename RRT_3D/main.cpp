#include <iostream>
#include <vector>
#include <random>
#include <queue>
#include <cmath>
#include <deque>

#define MIN_DIST 5
#define MAX_DIST 10
#define MAX_ITER 10000

#define START_X 0
#define START_Y 0
#define START_Z 0

#define FINISH_X 50
#define FINISH_Y 50
#define FINISH_Z 50

#define BOARD_X 50
#define BOARD_Y 50
#define BOARD_Z 50

using namespace std;

class Node {
private:
	double x; //x coordinate
	double y; //y coordinate
	double z; //z coordinate
public:
	Node* parent; //pointer to parent node
	vector<Node*> children; //vector of pointers to children nodes
	Node() : x(0.0), y(0.0), z(0.0) {
		parent = nullptr;
	}
	Node(double nx, double ny, double nz) : x(nx), y(ny), z(nz) {
		parent = nullptr;
	}
	Node(double nx, double ny, double nz, Node* nparent) : x(nx), y(ny), z(nz) {
		parent = nparent;
	}
	~Node() {
		for (int i = 0; i < children.size(); ++i) {
			delete children[i];
		}
	}
	
	double get_x() { return x; }
	double get_y() { return y; }
	double get_z() { return z; }

	void set_x(double nx) {
		x = nx;
	}
	void set_y(double ny) {
		y = ny;
	}
	void set_z(double nz) {
		z = nz;
	}
	void set_parent(Node* nparent) {
		parent = nparent;
	}
	void add_children(Node* nchild) {
		children.push_back(nchild);
	}
	void print_values() {
		cout << "(" << x << ", " << y << ", " << z << ")"  << endl;
	}
};

class RRT {
public:
	Node* start; //start node of tree
	Node* finish; //finish node of tree
	const int min_distance; //minimum distance to exist node
	const int max_distance; //maximum distance to exist node
	const int board_size_x; //size of board along x-axis
	const int board_size_y; //size of board along y-axis
	const int board_size_z; //size of board along z-axis
	const int max_iterations; //maximum number of iterations
	deque<Node*> planned_path; //deque with planned path with RRT algorithm

	RRT() : min_distance(MIN_DIST), max_distance(MAX_DIST), max_iterations(MAX_ITER), board_size_x(BOARD_X), board_size_y(BOARD_Y), board_size_z(BOARD_Z) {
		start = new Node(START_X, START_Y, START_Z);
		finish = new Node(FINISH_X, FINISH_Y, FINISH_Z);
	}
	~RRT() {
		delete start;
	}

	double distance(Node* node_1, Node* node_2) {
		return sqrt(pow(node_1->get_x() - node_2->get_x(), 2) + pow(node_1->get_y() - node_2->get_y(), 2) + pow(node_1->get_z() - node_2->get_z(), 2));
	}

	Node* random_point() {
		random_device rd;
		mt19937 gen(rd());
		uniform_int_distribution<int> dist_x(0, board_size_x);
		int x = dist_x(gen);
		uniform_int_distribution<int> dist_y(0, board_size_y);
		int y = dist_y(gen);
		uniform_int_distribution<int> dist_z(0, board_size_z);
		int z = dist_z(gen);
		Node* random_node = new Node(x, y, z);
		return random_node;
	}
	Node* find_nearest_node(Node* new_node) {
		Node* nearest = start; //nearest node to current node
		double dist = distance(start, new_node); //distance from nearest node to new_node
		queue<Node*> node_queue; //queue with nodes to verify
		node_queue.push(start);
		while (!node_queue.empty()) {
			Node* current = node_queue.front(); //current node
			for (int i = 0; i < current->children.size(); ++i) {
				node_queue.push(current->children[i]);
			}
			double current_dist = distance(current, new_node); //distance from new_node to current node
			if (current_dist < dist) {
				dist = current_dist;
				nearest = current;
			}
			node_queue.pop();
		}
		return nearest;
	}
	bool verify_finish(Node* new_node) {
		if (distance(finish, new_node) <= min_distance + 1)
			return true;
		return false;
	}
	void add_node_to_tree(Node* new_node, Node* parent) {
		parent->add_children(new_node);
		new_node->parent = parent;
	}
	void save_path() {
		Node* current = finish; //current node
		planned_path.clear();
		while (current->parent != nullptr) {
			planned_path.push_front(current);
			current = current->parent;
		}
		planned_path.push_front(current);
	}
	void print_path() {
		for (int i = 0; i < planned_path.size(); ++i) {
			cout << i + 1 << " ";
			planned_path[i]->print_values();
		}
	}
	void execute_rrt() {
		bool is_finish = false; //contains bool information about distance to finish
		int i = 0;
		cout << "Start: ";
		start->print_values();
		cout << "Finish: ";
		finish->print_values();
		do {
			Node* new_node = nullptr;
			Node* nearest = nullptr;
			double dist = 0;
			do {
				new_node = random_point();
				nearest = find_nearest_node(new_node);
				dist = distance(new_node, nearest);
			} while (dist < min_distance);
			//cout << "New node: ";
			//new_node->print_values();
			if (dist > max_distance) {
				double dx = new_node->get_x() - nearest->get_x();
				double dy = new_node->get_y() - nearest->get_y();
				double dz = new_node->get_z() - nearest->get_z();
				double dist_near = sqrt(dx * dx + dy * dy + dz * dz);
				new_node->set_x(nearest->get_x() + (dx / dist_near) * MAX_DIST);
				new_node->set_y(nearest->get_y() + (dy / dist_near) * MAX_DIST);
				new_node->set_z(nearest->get_z() + (dz / dist_near) * MAX_DIST);
				//cout << "Changed new node: ";
				//new_node->print_values();
			}
			is_finish = verify_finish(new_node);
			if (is_finish) {
				add_node_to_tree(finish, nearest);
				finish->set_parent(nearest);
			}
			else
				add_node_to_tree(new_node, nearest);
			++i;
		} while (!is_finish && i <= max_iterations);
		if (i > max_iterations) {
			cout << "Max iterations" << endl;
			cout << "Path not found" << endl;
		}
		else {
			cout << "Found path" << endl;
			cout << "Iterations: " << i << endl;
			cout << "Path:" << endl;
			save_path();
			print_path();
		}
	}
};

int main() {
	RRT classic_rrt; //classic RRT implementation class
	classic_rrt.execute_rrt();
	return 0;
}