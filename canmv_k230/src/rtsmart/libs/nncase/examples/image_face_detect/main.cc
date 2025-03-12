#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <stdlib.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "mobile_retinaface.h"
#include "mpi_sys_api.h"

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;

std::vector<uint8_t> hwc2chw(cv::Mat &img)
{
    std::vector<uint8_t> vec;
    std::vector<cv::Mat> rgbChannels(3);
    cv::split(img, rgbChannels);
    for (auto i = 0; i < rgbChannels.size(); i++)
    {
        std::vector<uint8_t> data = std::vector<uint8_t>(rgbChannels[i].reshape(1, 1));
        vec.insert(vec.end(), data.begin(), data.end());
    }

    return vec;
}

std::atomic<bool> ai_stop(false);

void ai_proc(const char *kmodel_file, const char *image_file)
{
    // input data
    size_t paddr = 0;
    void *vaddr = nullptr;
    cv::Mat img = cv::imread(image_file);
    int ret = kd_mpi_sys_mmz_alloc_cached(&paddr, &vaddr, "allocate", "anonymous", img.total() * img.elemSize());
    if (ret)
    {
        std::cerr << "physical_memory_block::allocate failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }

    auto vec = hwc2chw(img);
    memcpy(reinterpret_cast<char *>(vaddr), vec.data(), vec.size());

    MobileRetinaface model(kmodel_file, img.channels(), img.rows, img.cols);
    size_t idx = 0;
    while (!ai_stop)
    {
        // run kpu
        model.run(reinterpret_cast<uintptr_t>(vaddr), reinterpret_cast<uintptr_t>(paddr));
        auto result = model.get_result();

        for (size_t i = 0; i < result.boxes.size(); i++)
        {
            auto box = result.boxes[i];
            auto landmark = result.landmarks[i];
            cv::rectangle(img, cv::Point(box.x1, box.y1), cv::Point(box.x2, box.y2), cv::Scalar(0, 0, 255), 2);

            for (int j = 0; j < 5; j++)
            {
                cv::circle(img, cv::Point(landmark.points[2 * j + 0], landmark.points[2 * j + 1]), 1, cv::Scalar(255, 0, 0), 2);
            }
        }

        if (result.boxes.size() > 0)
        {
            idx++;
            idx %= 10;
            std::string src_file(image_file);
            std::string dst_file = src_file.substr(0, src_file.find("."));
            cv::imwrite(dst_file + "_result_" + std::to_string(idx) + ".jpg", img);
        }
    }

    // free memory
    ret = kd_mpi_sys_mmz_free(paddr, vaddr);
    if (ret)
    {
        std::cerr << "free failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }
}

int main(int argc, char *argv[])
{
    std::cout << "case " << argv[0] << " built at " << __DATE__ << " " << __TIME__ << std::endl;
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <kmodel> <rgb_image>" << std::endl;
        return -1;
    }

    std::cout << "Press 'q + enter' to exit!!!" << std::endl;
    std::thread t(ai_proc, argv[1], argv[2]);
    while (getchar() != 'q')
    {
        usleep(10000);
    }

    ai_stop = true;
    t.join();

    return 0;
}