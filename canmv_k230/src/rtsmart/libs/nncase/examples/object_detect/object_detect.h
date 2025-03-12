#ifndef _OBJECT_DETECT
#define _OBJECT_DETECT

#include "utils.h"
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_tensor.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdint.h>

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;
using namespace std;

class objectDetect
{
public:
    objectDetect(float obj_thresh, float nms_thresh, const char *kmodel_file);
    ~objectDetect();
    void set_input(const unsigned char *buf, size_t size);
    void run();
    void get_output();
    void post_process(std::vector<BoxInfo> &result);
    void dump(const float *p, size_t size);

    std::vector<std::string> labels
    {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
        "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
        "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };

private:
    float obj_thresh;
    float nms_thresh;
    Framesize frame_size = {320, 256};

    interpreter interp_od;

    int net_len = 320;
    int anchors_num = 3;
    int classes_num = 80;
    int channels = anchors_num * (5 + classes_num);
    int first_size;
    int second_size;
    int third_size;
    float anchors_0[3][2] = { { 10, 13 }, { 16, 30 }, { 33, 23 } };
    float anchors_1[3][2] = { { 30, 61 }, { 62, 45 }, { 59, 119 } };
    float anchors_2[3][2] = { { 116, 90 }, { 156, 198 }, { 373, 326 } };

    float *foutput_0;
    float *foutput_1;
    float *foutput_2;
};
#endif
