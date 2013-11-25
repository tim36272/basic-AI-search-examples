#include <assert.h>
#include <iostream>

#include <set> //std::set and std::multiset

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "types.cpp"

namespace a_star {

class heuristic_comparison {
public:
	bool operator() (const node& lhs, const node& rhs) const {
		return (lhs.path_cost+lhs.heuristic_cost) < (rhs.path_cost+rhs.heuristic_cost);
	}
};

typedef std::multiset<node,heuristic_comparison > OpenSetType;

std::set<node>::iterator FindState(const scalar& goal_state, std::set<node>* visited) {
	std::set<node>::iterator cursor = visited->begin();
	while(cursor!=visited->end()) {
		if(cursor->state==goal_state) {
			return cursor;
		}
		cursor++;
	}
	return visited->end();
}

std::multiset<node,heuristic_comparison>::iterator FindState(const scalar& goal_state, std::multiset<node,heuristic_comparison>* frontier) {
	std::multiset<node,heuristic_comparison>::iterator cursor = frontier->begin();
	while(cursor!=frontier->end()) {
		if(cursor->state==goal_state) {
			return cursor;
		}
		cursor++;
	}
	return frontier->end();
}

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

void PushChildren(const node& goal,const node& parent_node,OpenSetType* open_set, std::set<node>* closed_set) {
	node temp_node;
	temp_node.parent = parent_node.state;

	//look at each neighbor and if that action is in action && not in visited
	//then push it into the frontier and visited
	for(int direction=0;direction<8;direction++) {
		temp_node.state = parent_node.state+Direction(direction);
		//calculate the heuristic and path costs
		temp_node.path_cost = parent_node.path_cost+sqrt(pow(Direction(direction).x,2)+pow(Direction(direction).y,2));

		if(parent_node.action[direction]==1) {
			//lookup the child in the closed list
			std::multiset<node,heuristic_comparison>::iterator found_at_frontier = FindState(temp_node.state,open_set);
			std::set<node>::iterator found_at = FindState(temp_node.state,closed_set);

			if(!(found_at==closed_set->end()) && (temp_node.path_cost > parent_node.path_cost)) {
				continue;
			}

			if(found_at_frontier==open_set->end() || (temp_node.path_cost < parent_node.path_cost)) {

				temp_node.parent = parent_node.state;
				int x_distance = goal.state.x-temp_node.state.x,
					y_distance = goal.state.y-temp_node.state.y;
				temp_node.heuristic_cost= sqrt(pow(x_distance,2)+pow(y_distance,2));

				if(found_at_frontier!=open_set->end()) {
					open_set->erase(found_at_frontier);
					open_set->insert(temp_node);
				}
				else {
					open_set->insert(temp_node);
				}
			}

		}
	}

}


node A_Star(const scalar& size, const std::vector<std::vector<char> >& obstacles, std::set<node>* closed_set,cv::Mat* graphical_display, cv::VideoWriter& writer) {

	static const int kImageScale = 600/size.y;
	node goal_node;
	goal_node.state.x = size.x-1;
	goal_node.state.y = 0;

	node initial;
	initial.parent(0,0);
	initial.state.x = 0;
	initial.state.y = size.y-1;
	initial.path_cost = 0;
	initial.heuristic_cost = sqrt(pow(size.x-1,2)+pow(size.y-1,2));

	OpenSetType open_set;
	open_set.insert(initial);

	while(open_set.size() > 0) {
		node current = *(open_set.begin());
		if((open_set.begin()->state==goal_node.state)) {
			return current;
		}
		open_set.erase(open_set.begin());
		closed_set->insert(current);
		FindChildren(size,obstacles,&current);

		PushChildren(goal_node,current,&open_set,closed_set);

		for(int x=0;x<kImageScale;x++) {
			for(int y=0;y<kImageScale;y++) {
				graphical_display->at<cv::Vec3b>(current.state.y*kImageScale+y,current.state.x*kImageScale+x)[1]=255;
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
} //namespace greedy
