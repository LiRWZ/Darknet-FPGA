#include "track.h"
#include <vector>
#include "box.h"

using namespace std;

#define SIGMA_IOU (float)0.3

//void tracker_update(vector<tracker>& trackers, vector<detection>& dets, long long int frame, int& last_trackNo)
//{
//	//delete the tracker which (frame - tracker.last_fram) > 3 
//	for (auto p = trackers.begin(); p != trackers.end();)
//	{
//		if ((frame - p->last_frame) > 15)
//		{
//			//printf("erase a tracker\n");
//			p = trackers.erase(p);
//		}
//		else
//			p++;
//	}
//	
//	for (auto p = trackers.begin(); p != trackers.end(); p++)
//	{
//		float maxiou = 0;
//		int bestIndex = 0;
//		//get the best detection
//		for (int i = 0; i < dets.size(); i++)
//		{
//			float temp = box_iou(p->bbox, dets[i].bbox);
//			if (maxiou < temp)
//			{
//				bestIndex = i;
//				maxiou = temp;
//			}
//		}
//		if (maxiou >= SIGMA_IOU)
//		{
//			p->bbox = dets[bestIndex].bbox;
//			p->last_frame = frame;
//			coordinate coortemp;
//			coortemp.x = p->bbox.x;
//			coortemp.y = p->bbox.y;
//			p->trace.push_back(coortemp);
//			//printf("erase a detection\n");
//			dets.erase(dets.begin() + bestIndex);
//		}
//	}
//	for (auto p = dets.begin(); p != dets.end(); p++)
//	{
//		tracker temp;
//		temp.bbox = p->bbox;
//		temp.last_frame = frame;
//		temp.tracker_No = last_trackNo++;
//		//printf("create a new tracker\n");
//		coordinate coortemp;
//		coortemp.x = p->bbox.x;
//		coortemp.y = p->bbox.y;
//		temp.trace.push_back(coortemp);
//		trackers.push_back(temp);
//	}
//}

void tracker_update(vector<tracker>& trackers, vector<tracker>& trackers_pre ,vector<detection>& dets, long long int frame, int& last_trackNo)
{
	//printf("frame:%ld\n",frame);
	//delete the tracker which (frame - tracker.last_fram) > 3 
	for (auto p = trackers.begin(); p != trackers.end();p++)
	{
		if ((frame - p->last_frame) > 3)
		{
			//printf("erase a tracker\n");
			trackers.erase(p);
			p--;
		}
	}

	for (auto p = trackers_pre.begin(); p != trackers_pre.end();p++)
	{
		if (p->trace.size() > 15)
		{
			p->tracker_No = last_trackNo++;
			trackers.push_back(*p);
			trackers_pre.erase(p);
			p--;
			//printf("create a new tracker\n size of tracker:%d\nsize of tracker_pre:%d\n", trackers.size(), trackers_pre.size());
		}
	}

	for (auto p = trackers.begin(); p != trackers.end();p++)
	{
		float maxiou = 0;
		int bestIndex = 0;
		//get the best detection
		for (int i = 0; i < dets.size(); i++)
		{
			float temp = box_iou(p->bbox, dets[i].bbox);
			if (maxiou < temp)
			{
				bestIndex = i;
				maxiou = temp;
			}
		}
		if (maxiou >= SIGMA_IOU)
		{
			p->bbox = dets[bestIndex].bbox;
			p->last_frame = frame;
			coordinate coortemp;
			coortemp.x = p->bbox.x;
			coortemp.y = p->bbox.y + p->bbox.h / 2.0;
			p->trace.push_back(coortemp);
			dets.erase(dets.begin() + bestIndex);
			//printf("Aerase a detection\nsize of detection:%d\n", dets.size());
		}
		else//没有匹配到最佳目标，仍然向trace中加入一个坐标点，该坐标点为最后检测到的坐标点
		{
			coordinate coortemp;
			coortemp.x = p->trace.back().x;
			coortemp.y = p->trace.back().y;
			p->trace.push_back(coortemp);
		}
	}
	for (auto p = trackers_pre.begin(); p != trackers_pre.end(); p++)
	{
		float maxiou = 0;
		int bestIndex = 0;
		//get the best detection
		for (int i = 0; i < dets.size(); i++)
		{
			float temp = box_iou(p->bbox, dets[i].bbox);
			if (maxiou < temp)
			{
				bestIndex = i;
				maxiou = temp;
			}
		}
		if (maxiou >= SIGMA_IOU)
		{
			p->bbox = dets[bestIndex].bbox;
			p->last_frame = frame;
			coordinate coortemp;
			coortemp.x = p->bbox.x;
			coortemp.y = p->bbox.y + p->bbox.h / 2.0;
			p->trace.push_back(coortemp);
			//printf("erase a detection\n");
			dets.erase(dets.begin() + bestIndex);
			//printf("Berase a detection\nsize of detection:%d\n", dets.size());
		}
		//没有匹配到最佳的detection，删除该tracker
		else
		{
			//printf("##\n");
			trackers_pre.erase(p);
			p--;
		}	
	}

	for (auto p = dets.begin(); p != dets.end(); p++)
	{
		tracker temp;
		temp.bbox = p->bbox;
		temp.first_frame = frame;
		temp.last_frame = frame;
		temp.tracker_No = -1;
		temp.speed = 0;
		coordinate coortemp;
		coortemp.x = p->bbox.x;
		coortemp.y = p->bbox.y + p->bbox.h / 2.0;
		temp.trace.push_back(coortemp);
		temp.speed = .0;
		trackers_pre.push_back(temp);
		//printf("create a new tracker_pre\n,size of trackers_pre %d:\n",trackers_pre.size());
	}
}
