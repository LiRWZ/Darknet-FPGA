#ifndef DEMO_H
#define DEMO_H

#include "image.h"
#include "darknet.h"
#include <vector>
#ifdef __cplusplus
extern "C" {
#endif
void demo(char *cfgfile, char *weightfile, float thresh, float hier_thresh, int cam_index, const char *filename, char **names, int classes,
    int frame_skip, char *prefix, char *out_filename, int mjpeg_port, int json_port, int dont_show, int ext_output, int letter_box_in);
#ifdef __cplusplus
}
#endif
void makecopy_detection(detection* det, std::vector<detection>& dets_copy, float thresh, int classnum);
#endif
