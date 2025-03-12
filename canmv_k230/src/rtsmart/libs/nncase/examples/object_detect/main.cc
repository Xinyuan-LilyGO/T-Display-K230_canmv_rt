#include <iostream>
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_tensor.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "utils.h"
#include "object_detect.h"

using namespace std;

#define ENABLE_PROFILING 1

void read_binary_file(const char *file_name, char *buffer)
{
    std::ifstream ifs(file_name, std::ios::binary);
    ifs.seekg(0, ifs.end);
    size_t len = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    ifs.read(buffer, len);
    ifs.close();
}

int main(int argc, char *argv[])
{
    std::cout << "case " << argv[0] << " built at " << __DATE__ << " " << __TIME__ << std::endl;
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <kmodel> <image>" << std::endl;
        return -1;
    }

    float obj_thresh = 0.2;
    float nms_thresh = 0.45;
    objectDetect od(obj_thresh, nms_thresh, argv[1]);

    // preprocess image
    cv::Mat img1 = cv::imread(argv[2]);
    cv::Mat img2 = letterbox(img1, 320, 320);

    // hwc -> chw
    auto vec = hwc2chw(img2);
    {
#if ENABLE_PROFILING
        ScopedTiming st("od set_input");
#endif
        od.set_input(vec.data(), vec.size());
    }

    {
#if ENABLE_PROFILING
        ScopedTiming st("od run");
#endif
        od.run();
    }

    {
#if ENABLE_PROFILING
        ScopedTiming st("od get output");
#endif
        od.get_output();
    }

    std::vector<BoxInfo> result;
    {
#if ENABLE_PROFILING
        ScopedTiming st("post process");
#endif
        od.post_process(result);
    }

    {
#if ENABLE_PROFILING
        ScopedTiming st("draw result");
#endif
        int obj_cnt = 0;
        for (auto r : result)
        {
            std::string text = od.labels[r.label] + ":" + std::to_string(round(r.score * 100) / 100.0);
            std::cout << "text = " << text << std::endl;
            cv::rectangle(img1, cv::Rect(r.x1, r.y1, r.x2 - r.x1 + 1, r.y2 - r.y1 + 1), cv::Scalar(255, 255, 255), 2, 2, 0);
            cv::Point origin;
            origin.x = 30;
            origin.y = 20 + 20 * obj_cnt;
            cv::putText(img1, text, origin, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, 8, 0);
            obj_cnt += 1;
        }
        cv::imwrite("od_result.jpg", img1);
    }

    return 0;
}