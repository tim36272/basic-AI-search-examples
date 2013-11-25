#include <assert.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
#include <set>

#include "dfs.cpp"
#include "greedy.cpp"
#include "a_star.cpp"
#include "types.cpp"

static const int kMaxObstacleSize = 20;

void GenerateObstacles(int number_of_obstacles, const scalar& size, std::vector<std::vector<char> >* obstacles);

int main() {
	//seed random number generator
	srand(std::time(NULL));
	//load grid size
	std::ifstream inf;
	
	inf.clear();
	inf.open("../dimensions");
	if(!inf.good()) {
		std::cout<<"Could not find dimensions file"<<std::endl;
		return 0;
	}
	
	scalar size;
	int kNumberOfObstacles;
	
	inf >>size.y>>size.x>>kNumberOfObstacles;
	
	if(size.y<=0 || size.x<=0 || kNumberOfObstacles<0) {
		std::cout<<"The dimensions or number of obstacles were bad"<<std::endl;
	}
	
	std::cout<<"Grid size: ("<<size.y<<","<<size.x<<")"<<std::endl;

	//make an obstacle map
	//which is a 2D vector with size r x c
	std::vector<std::vector<char> > obstacles( size.x, std::vector<char> ( size.y, OPEN ) );
	
	GenerateObstacles(kNumberOfObstacles,size,&obstacles);
		
		
		//find the path
		cv::namedWindow("progress",CV_WINDOW_AUTOSIZE);
		cv::moveWindow("progress",50,50);
		std::set<node> visited;
		cv::Mat menu(200,670,CV_8UC3,cv::Scalar(0,0,0));
		cv::putText(menu,"Focus this window and select a number:",cv::Point(5,25),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		cv::putText(menu,"1",cv::Point(5,55),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
		cv::putText(menu,": depth first search",cv::Point(20,55),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		cv::putText(menu,"2",cv::Point(5,85),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
		cv::putText(menu,": greedy search",cv::Point(20,85),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		cv::putText(menu,"3",cv::Point(5,115),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
		cv::putText(menu,": A* search",cv::Point(20,115),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		cv::putText(menu,"4",cv::Point(5,145),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
		cv::putText(menu,": generate new obstacles",cv::Point(20,145),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		cv::putText(menu,"5",cv::Point(5,175),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),1,8);
		cv::putText(menu,": quit",cv::Point(20,175),CV_FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
		imshow("Menu",menu);

		//graphics
		static const int kImageScale = 600/size.y;
		cv::Mat graphical_display(cv::Size(size.y*kImageScale,size.x*kImageScale),CV_8UC3,cv::Scalar(0));
		for(int y=0;y<size.y;y++) {
			for(int x=0;x<size.x;x++) {
				for(int x_scale=0;x_scale<kImageScale;x_scale++) {
					for(int y_scale=0;y_scale<kImageScale;y_scale++) {
						if(obstacles[x][y]!=OPEN) {
							graphical_display.at<cv::Vec3b>(y*kImageScale+y_scale,x*kImageScale+x_scale)[0] = 255;
							graphical_display.at<cv::Vec3b>(y*kImageScale+y_scale,x*kImageScale+x_scale)[1] = 255;

						}
					}
				}
			}
		}
		imshow("progress",graphical_display);

		cv::VideoWriter writer;
		writer.open("dfs.avi",CV_FOURCC('D','I','V','X'),60,cv::Size(600,600),true);

		char key = cv::waitKey(0);
		bool found_new = false;
		while(key!='5') {
			node goal;
			switch(key) {
			case '1':
				visited.clear();
				goal = dfs::DepthFirstSearch(size,obstacles,&visited,&graphical_display,writer);
				found_new = true;
				break;
			case '2':
				visited.clear();
				goal = greedy::Greedy(size,obstacles,&visited,&graphical_display,writer);
				found_new = true;
				break;
			case '3':
				visited.clear();
				goal = a_star::A_Star(size,obstacles,&visited,&graphical_display,writer);
				found_new = true;
				break;
			case '4':
				GenerateObstacles(kNumberOfObstacles,size,&obstacles);
				graphical_display.setTo(cv::Scalar(0,0,0));
				for(int y=0;y<size.y;y++) {
					for(int x=0;x<size.x;x++) {
						for(int x_scale=0;x_scale<kImageScale;x_scale++) {
							for(int y_scale=0;y_scale<kImageScale;y_scale++) {
								if(obstacles[x][y]!=OPEN) {
									graphical_display.at<cv::Vec3b>(y*kImageScale+y_scale,x*kImageScale+x_scale)[0] = 255;
									graphical_display.at<cv::Vec3b>(y*kImageScale+y_scale,x*kImageScale+x_scale)[1] = 255;

								}
							}
						}
					}
				}
				imshow("progress",graphical_display);
				break;
			}

			if(goal.state.x!=-1 && found_new) {
				std::cout<<"Path found"<<std::endl;
				//show the path
				node current = goal;
				while(current.state.x!=0 && current.state.y!=(size.y-1)) {
					cv::rectangle(graphical_display,
							cv::Point(current.state.x*kImageScale,current.state.y*kImageScale+kImageScale),
							cv::Point(current.state.x*kImageScale+kImageScale,current.state.y*kImageScale),
							cv::Scalar(255,255,255),-1,4);
					node parent;
					parent.state = current.parent;
					current = *(visited.find(parent));
					imshow("progress",graphical_display);
					writer<<graphical_display;
					cv::waitKey(10);
				}
				found_new = false;
			}
			else if(found_new){
				std::cout<<"No path found"<<std::endl;
			}
			key = cv::waitKey(0);
		}
		return 0;
}
void GenerateObstacles(int number_of_obstacles, const scalar& size, std::vector<std::vector<char> >* obstacles) {
	//set all to zero
	for(int x=0;x<size.x;x++) {
		for(int y=0;y<size.y;y++) {
			(*obstacles)[x][y] = OPEN;
		}
	}
	//randomly generate obstacles
		//generate upper left and lower right
		int upper_left_x,upper_left_y,lower_right_x,lower_right_y;
		for(int obstacle_index=0;obstacle_index<number_of_obstacles;obstacle_index++) {
			upper_left_x = rand()%size.x;
			upper_left_y = rand()%size.y;

			lower_right_x = rand()%kMaxObstacleSize+upper_left_x;
			lower_right_y = rand()%kMaxObstacleSize+upper_left_y;

			if(lower_right_x > size.y) lower_right_x = size.y-1;
			if(lower_right_y > size.x) lower_right_y = size.x-1;


			for(int x_index=upper_left_x;x_index<lower_right_x;x_index++) {
				for(int y_index=upper_left_y;y_index<lower_right_y;y_index++) {
					(*obstacles)[x_index][y_index] = OBSTACLE;
				}
			}
		}
}

