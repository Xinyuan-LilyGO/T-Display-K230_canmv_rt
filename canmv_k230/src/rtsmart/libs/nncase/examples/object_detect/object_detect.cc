#include "object_detect.h"
#include "utils.h"

template <class T>
std::vector<T> read_binary_file(const char *file_name)
{
    std::ifstream ifs(file_name, std::ios::binary);
    ifs.seekg(0, ifs.end);
    size_t len = ifs.tellg();
    std::vector<T> vec(len / sizeof(T), 0);
    ifs.seekg(0, ifs.beg);
    ifs.read(reinterpret_cast<char *>(vec.data()), len);
    ifs.close();
    return vec;
}

objectDetect::objectDetect(float obj_thresh, float nms_thresh, const char *kmodel_file)
:obj_thresh(obj_thresh), nms_thresh(nms_thresh)
{
    int first_len = net_len / 8;
    first_size = first_len * first_len;
    int second_len = net_len / 16;
    second_size = second_len * second_len;
    int third_len = net_len / 32;
    third_size = third_len * third_len;

    foutput_0 = new float[channels * first_size];
    foutput_1 = new float[channels * second_size];
    foutput_2 = new float[channels * third_size];

    // load kmodel
    std::ifstream ifs(kmodel_file, std::ios::binary);
    interp_od.load_model(ifs).expect("load_model failed");

    // create input tensors
    for (size_t i = 0; i < interp_od.inputs_size(); i++)
    {
        auto desc = interp_od.input_desc(i);
        auto shape = interp_od.input_shape(i);
        auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create input tensor");
        interp_od.input_tensor(i, tensor).expect("cannot set input tensor");
    }

    // create output tensors
    // for (size_t i = 0; i < interp_od.outputs_size(); i++)
    // {
    //     auto desc = interp_od.output_desc(i);
    //     auto shape = interp_od.output_shape(i);
    //     auto tensor = host_runtime_tensor::create(desc.datatype, shape, hrt::pool_shared).expect("cannot create output tensor");
    //     interp_od.output_tensor(i, tensor).expect("cannot set output tensor");
    // }
}


objectDetect::~objectDetect()
{
    delete[] foutput_0;
    delete[] foutput_1;
    delete[] foutput_2;
}

void objectDetect::set_input(const unsigned char *buf, size_t size)
{
    auto tensor = interp_od.input_tensor(0).expect("cannot get input tensor");
    auto dst = tensor.impl()->to_host().unwrap()->buffer().as_host().unwrap().map(map_access_::map_write).unwrap().buffer();
    memcpy(reinterpret_cast<char *>(dst.data()), buf, size);
    auto ret = hrt::sync(tensor, sync_op_t::sync_write_back, true);
    if (!ret.is_ok())
    {
        std::cerr << "hrt::sync failed" << std::endl;
        std::abort();
    }
}

void objectDetect::run()
{
    interp_od.run().expect("error occurred in running model");
}

void objectDetect::get_output()
{
    // first output
    {
        auto out = interp_od.output_tensor(0).expect("cannot get output tensor");
        auto buf = out.impl()->to_host().unwrap()->buffer().as_host().unwrap().map(map_access_::map_read).unwrap().buffer();
        float *output_0 = reinterpret_cast<float *>(buf.data());

        int first_len = net_len / 8;
        int first_size = first_len * first_len;

        for (size_t j = 0; j < first_size; j++)
        {
            for (size_t c = 0; c < channels; c++)
            {
                foutput_0[j * channels + c] = output_0[c * first_size + j];
            }
        }
    }

    // second output
    {
        auto out = interp_od.output_tensor(1).expect("cannot get output tensor");
        auto buf = out.impl()->to_host().unwrap()->buffer().as_host().unwrap().map(map_access_::map_read).unwrap().buffer();
        float *output_1 = reinterpret_cast<float *>(buf.data());

        int second_len = net_len / 16;
        int second_size = second_len * second_len;
        for (size_t j = 0; j < second_size; j++)
        {
            for (size_t c = 0; c < channels; c++)
            {
                foutput_1[j * channels + c] = output_1[c * second_size + j];
            }
        }
    }

    // third output
    {
        auto out = interp_od.output_tensor(2).expect("cannot get output tensor");
        auto buf = out.impl()->to_host().unwrap()->buffer().as_host().unwrap().map(map_access_::map_read).unwrap().buffer();
        float *output_2 = reinterpret_cast<float *>(buf.data());

        int third_len = net_len / 32;
        int third_size = third_len * third_len;

        for (size_t j = 0; j < third_size; j++)
        {
            for (size_t c = 0; c < channels; c++)
            {
                foutput_2[j * channels + c] = output_2[c * third_size + j];
            }
        }
    }
}

void objectDetect::post_process(std::vector<BoxInfo> &result)
{
    auto boxes0 = decode_infer(foutput_0, net_len, 8, classes_num, frame_size, anchors_0, obj_thresh);
    result.insert(result.begin(), boxes0.begin(), boxes0.end());
    auto boxes1 = decode_infer(foutput_1, net_len, 16, classes_num, frame_size, anchors_1, obj_thresh);
    result.insert(result.begin(), boxes1.begin(), boxes1.end());
    auto boxes2 = decode_infer(foutput_2, net_len, 32, classes_num, frame_size, anchors_2, obj_thresh);
    result.insert(result.begin(), boxes2.begin(), boxes2.end());
    nms(result, nms_thresh);
}

void objectDetect::dump(const float *p, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        std::cout << p[i] << " ";
    }
    std::cout << std::endl;
}