#include <assert.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <queue>
#include <set>
#include "types.cpp"

namespace dfs {


void FindChildren(const scalar& size, const std::vector<std::vector<char> >& obstacles, node* the_node) {

	for(int i=0;i<the_node->action.size();i++) {
		the_node->action[i] = 1;
	}

	if(the_node->state.x<1) {
		the_node->action[LOWER_LEFT] = 0;
		the_node->action[LEFT] = 0;
		the_node->action[UPPER_LEFT] = 0;
	}
	else if(the_node->state.x>size.x-2) {
		the_node->action[LOWER_RIGHT] = 0;
		the_node->action[RIGHT] = 0;
		the_node->action[UPPER_RIGHT] = 0;
	}


	if(the_node->state.y>size.y-2) {
		the_node->action[LOWER_LEFT] = 0;
		the_node->action[LOWER] = 0;
		the_node->action[LOWER_RIGHT] = 0;
	}
	else if(the_node->state.y<1) {
		the_node->action[UPPER_LEFT] = 0;
		the_node->action[UPPER] = 0;
		the_node->action[UPPER_RIGHT] = 0;
	}

		for(int direction=0;direction<8;direction++) {
			scalar location = the_node->state + Direction(direction);
			if(location.x >= 0 && location.x < size.x && location.y >=0 && location.y < size.y) {
				if(obstacles[location.x][location.y]==OBSTACLE) {
					the_node->action[direction] = 0;
				}
			}
		}
}
void PushChildren(const node& parent_node,std::queue<node>* frontier, std::set<node>* visited) {
	node temp_node;
	temp_node.parent = parent_node.state;

	//look at each neighbor and if that action is in action && not in visited
	//then push it into the frontier and visited
	for(int direction=0;direction<8;direction++) {
		temp_node.state = parent_node.state+Direction(direction);
		if(parent_node.action[direction]==1 && visited->find(temp_node)==visited->end()) {
			visited->insert(temp_node);
			frontier->push(temp_node);
		}
	}

}

node DepthFirstSearch(const scalar& size, const std::vector<std::vector<char> >& obstacles, std::set<node>* visited,cv::Mat* graphical_display, cv::VideoWriter& writer) {

	static const int kImageScale = 600/size.y;
	node goal_node;
	goal_node.state.x = size.x-1;
	goal_node.state.y = 0;

	node initial;
	initial.parent(0,0);
	initial.state.x = 0;
	initial.state.y = size.y-1;
	initial.path_cost = 0;

	std::queue<node> frontier;

	frontier.push(initial);



	while(frontier.size()>0) {
		node this_node = frontier.front();
		visited->insert(this_node);
		frontier.pop();

		if(this_node.state==goal_node.state) {
			return this_node;
		}

		FindChildren(size,obstacles,&this_node);

		PushChildren(this_node,&frontier,visited);

		//show current location
		for(int x=0;x<kImageScale;x++) {
			for(int y=0;y<kImageScale;y++) {
				graphical_display->at<cv::Vec3b>(this_node.state.y*kImageScale+y,this_node.state.x*kImageScale+x)[0]=255;
			}
		}
		imshow("progress",*graphical_display);
		writer<<*graphical_display;
		cv::waitKey(1);

	}
	//if we get this far there was no path found
	node no_path;
	no_path.state.x=-1;
	return no_path;
}

} //namespace dfs
