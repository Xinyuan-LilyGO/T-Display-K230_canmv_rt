#ifndef _CV2_UTILS
#define _CV2_UTILS

#include <vector>
#include <algorithm>
#include <cmath>
#include <stdio.h>

typedef struct BoxInfo
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int label;
    
    float vArea;
} BoxInfo;

typedef struct Framesize
{
    int width;
    int height;
} Framesize;

void nms(std::vector<BoxInfo> &input_boxes, float NMS_THRESH);
void nms_e(std::vector<BoxInfo*> &input_boxes, float NMS_THRESH);

#endif
