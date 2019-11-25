#pragma once
#include "box.h"
#include <vector>

typedef struct coordinate
{
	float x, y;
}coordinate;

typedef struct coordinate_warped : coordinate
{
	
};

typedef struct tracker
{
	long long int tracker_No;
	box bbox;
//	float max_score;
	long long int first_frame;
	long long int last_frame;
	std::vector<coordinate>trace;
	float speed;
}tracker;

//void Tracker_Init()
//void tracker_update(std::vector<tracker>& trackers, std::vector<detection>& dets, long long int frame, int& last_trackNo);
void tracker_update(std::vector<tracker>& trackers, std::vector<tracker>& trackers_pre, std::vector<detection>& dets, long long int frame, int& last_trackNo);

//detection get_BestDetection(tracker tra, std::vector<detection> dets);