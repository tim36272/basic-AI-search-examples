#ifndef TYPES_H
#define TYPES_H

#include <bitset> // std::bitset

#define OPEN 0
#define OBSTACLE 1
#define FRONTIER 2
#define EXPLORED 3
#define LOWER_LEFT 	0
#define LEFT 				1
#define UPPER_LEFT	2
#define UPPER 			3
#define UPPER_RIGHT 4
#define RIGHT 			5
#define LOWER_RIGHT 6
#define LOWER 			7

struct scalar {
	int x,y;
	scalar() {}
	scalar(int x_in, int y_in) {x=x_in; y=y_in;}
	void operator ()(int x_in, int y_in) {
		x = x_in; y=y_in;
	}
	scalar operator +(const scalar& rhs) const{
		scalar out;
		out.x = this->x+rhs.x;
		out.y = this->y+rhs.y;
		return out;
	}
	bool operator ==(const scalar& rhs) const {
		return this->x==rhs.x && this->y==rhs.y;
	}

};

struct node {
	scalar parent;
	std::bitset<8> action;
	scalar state;
	double path_cost;
	double heuristic_cost;

	bool operator <(const node& rhs) const{
		int lhs_key = this->state.x*10000+this->state.y;
		int rhs_key = rhs.state.x*10000+rhs.state.y;

		return lhs_key < rhs_key;
	}

};
scalar Direction(int direction) {
	assert(direction >= 0 && direction <= 7);
	switch(direction) {
		case LOWER_LEFT:
			return scalar(-1,1);
			break;
		case LEFT:
			return scalar(-1,0);
			break;
		case UPPER_LEFT:
			return scalar(-1,-1);
			break;
		case UPPER:
			return scalar(0,-1);
			break;
		case UPPER_RIGHT:
			return scalar(1,-1);
			break;
		case RIGHT:
			return scalar(1,0);
			break;
		case LOWER_RIGHT:
			return scalar(1,1);
			break;
		case LOWER:
			return scalar(0,1);
			break;
	}
	return scalar(0,0);
}
#endif //TYPES_H
