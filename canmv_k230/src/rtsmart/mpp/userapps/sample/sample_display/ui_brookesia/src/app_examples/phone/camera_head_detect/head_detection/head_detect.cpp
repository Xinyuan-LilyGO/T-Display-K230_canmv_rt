#include "head_detect.h"
#include <thread>
#include "k_vo_comm.h"
#include "mpi_vicap_api.h"
#include "mpi_vo_api.h"
#include "mpi_sys_api.h"
#include <nncase/runtime/runtime_op_utility.h>
#include <iostream>
//#include "utils.h"
#include "head_detection.h"




using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;


#define ISP_CHN1_HEIGHT (720)
#define ISP_CHN1_WIDTH  (1280)
#define ISP_CHN0_WIDTH  (560)
#define ISP_CHN0_HEIGHT (992)
static bool flag_model_run;
pthread_t thread_handle_head_detect;
#if 0
static void *fn_face_detect(void *arg)
{
    MobileRetinaface model("test.kmodel", 3, ISP_CHN1_HEIGHT, ISP_CHN1_WIDTH);
    DetectResult box_result;
    std::vector<face_coordinate> boxes;
    
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_vo_draw_frame vo_frame = (k_vo_draw_frame) {
    1,
    16,
    16,
    128,
    128,
    1
};
    k_video_frame_info dump_info;
    size_t size = 3 * ISP_CHN1_WIDTH * ISP_CHN1_HEIGHT;
    int face_count = 1;
    //int dump_num=0;
    while(flag_model_run)
    {
    
    //printf("dump_num:%d\n",dump_num++);
        memset(&dump_info, 0 , sizeof(k_video_frame_info));
        int ret = kd_mpi_vicap_dump_frame(dev_num, VICAP_CHN_ID_1, VICAP_DUMP_YUV, &dump_info, 1000);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_frame failed.\n");
            break;
        }

        auto vbvaddr = kd_mpi_sys_mmap(dump_info.v_frame.phys_addr[0], size);
     
        boxes.clear();

        model.run(reinterpret_cast<uintptr_t>(vbvaddr), reinterpret_cast<uintptr_t>(dump_info.v_frame.phys_addr[0]));
        kd_mpi_sys_munmap(vbvaddr, size);
        // get face boxes
        box_result = model.get_result();
        boxes = box_result.boxes;

        if(boxes.size() < face_count)
        {
            for (size_t i = boxes.size(); i < face_count; i++)
            {
                vo_frame.draw_en = 0;
                vo_frame.frame_num = i + 1;
                kd_mpi_vo_draw_frame(&vo_frame);
            }
        }

        for (size_t i = 0, j = 0; i < boxes.size(); i += 1)
        {
            //printf("(x1:%d,y1:%d)-(x2:%d,y2:%d)\n",boxes[i].x1,boxes[i].y1,boxes[i].x2,boxes[i].y2);
            vo_frame.draw_en = 1;
            /* vo rotation 90 */
            vo_frame.line_x_start = ISP_CHN0_WIDTH-((uint32_t)boxes[i].y2) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_start = (((uint32_t)boxes[i].x1) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
            vo_frame.line_x_end = ISP_CHN0_WIDTH-((uint32_t)boxes[i].y1) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_end =(((uint32_t)boxes[i].x2) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
                       
            /* vo rotation 0 */
            /*vo_frame.line_x_start = ((uint32_t)boxes[i].x1) * ISP_CHN0_WIDTH / ISP_CHN1_WIDTH;
            vo_frame.line_y_start = ((uint32_t)boxes[i].y1) * ISP_CHN0_HEIGHT / ISP_CHN1_HEIGHT;
            vo_frame.line_x_end = ((uint32_t)boxes[i].x2) * ISP_CHN0_WIDTH / ISP_CHN1_WIDTH;
            vo_frame.line_y_end = ((uint32_t)boxes[i].y2) * ISP_CHN0_HEIGHT / ISP_CHN1_HEIGHT;*/
                           
            vo_frame.frame_num = ++j;
            kd_mpi_vo_draw_frame(&vo_frame);
        }

        face_count = boxes.size();
        ret = kd_mpi_vicap_dump_release(dev_num, VICAP_CHN_ID_1, &dump_info);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_release failed.\n");
        }
    usleep(10000);
    }
    
     for(size_t i = 0;i < boxes.size();i++)
    {
        vo_frame.draw_en = 0;
        vo_frame.frame_num = i + 1;
        kd_mpi_vo_draw_frame(&vo_frame);
    }
    boxes.clear();
    return NULL;
}
#endif
static void *fn_head_detect(void *arg)
{
std::vector<Detection> results;
k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_vo_draw_frame vo_frame = (k_vo_draw_frame) {
    1,
    16,
    16,
    128,
    128,
    1
};
    k_video_frame_info dump_info;
    size_t size = 3 * ISP_CHN1_WIDTH * ISP_CHN1_HEIGHT;
    int head_count = 5;
    //int dump_num=0;
    
    
    // alloc memory
    size_t paddr = 0;
    void *vaddr = nullptr;
    int ret = kd_mpi_sys_mmz_alloc_cached(&paddr, &vaddr, "allocate", "anonymous", size);
    if (ret)
    {
        std::cerr << "physical_memory_block::allocate failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }
        float score_thres=0.4;
        float nms_thres=0.3;
        int debug_mode=0;//是否需要调试，0、1、2分别表示不调试、简单调试、详细调试
    HeadDetection head_detection("head_detection.kmodel", score_thres, nms_thres, {3, ISP_CHN1_HEIGHT, ISP_CHN1_WIDTH}, reinterpret_cast<uintptr_t>(vaddr), reinterpret_cast<uintptr_t>(paddr), debug_mode);
    
    
    
    while(flag_model_run)
    {
    
    //printf("dump_num:%d\n",dump_num++);
        memset(&dump_info, 0 , sizeof(k_video_frame_info));
        int ret = kd_mpi_vicap_dump_frame(dev_num, VICAP_CHN_ID_1, VICAP_DUMP_YUV, &dump_info, 1000);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_frame failed.\n");
            break;
        }

        auto vbvaddr = kd_mpi_sys_mmap(dump_info.v_frame.phys_addr[0], size);
        memcpy(vaddr, (void *)vbvaddr, ISP_CHN1_HEIGHT * ISP_CHN1_WIDTH * 3);  // 这里以后可以去掉，不用copy  
        kd_mpi_sys_munmap(vbvaddr, size);
  
        // get head boxes
	results.clear();      
        head_detection.pre_process();        
        head_detection.inference();        
        head_detection.post_process({ISP_CHN1_WIDTH, ISP_CHN1_HEIGHT}, results,false);
        //printf("post_process  after ...results.size=%d\n",results.size());
         if(results.size() < head_count)
        {
            for (size_t i = results.size(); i < head_count; i++)
            {
                vo_frame.draw_en = 0;
                vo_frame.frame_num = i + 1;
                kd_mpi_vo_draw_frame(&vo_frame);
            }
        }
           
    for (int i = 0, j = 0; i < results.size(); ++i)
    {
        auto& bbox = results[i].box;
        uint32_t x1=(uint32_t)bbox.x;
        uint32_t y1=(uint32_t)bbox.y;
        uint32_t x2=(uint32_t)(bbox.x+bbox.width);
        uint32_t y2=(uint32_t)(bbox.y+bbox.height);        
           vo_frame.draw_en = 1;
            /* vo rotation 90 */
            vo_frame.line_x_start = ISP_CHN0_WIDTH-((uint32_t)y2) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_start = (((uint32_t)x1) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
            vo_frame.line_x_end = ISP_CHN0_WIDTH-((uint32_t)y1) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_end =(((uint32_t)x2) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
            vo_frame.frame_num = ++j;
            kd_mpi_vo_draw_frame(&vo_frame);              
               
        if(results[i].class_id==0)
          {    
            
             head_count += 1;
          }
   }
     head_count = results.size();
        ret = kd_mpi_vicap_dump_release(dev_num, VICAP_CHN_ID_1, &dump_info);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_release failed.\n");
        }
    usleep(10000);                 
      }//while  
        for(size_t i = 0;i < results.size();i++)
    {
        vo_frame.draw_en = 0;
        vo_frame.frame_num = i + 1;
        kd_mpi_vo_draw_frame(&vo_frame);
    }
    results.clear();
      // free memory
    ret = kd_mpi_sys_mmz_free(paddr, vaddr);
    if (ret)
    {
        std::cerr << "free failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }  
        
        
        
        
        
        
        

        /*cv::Mat osd_frame(ISP_CHN0_HEIGHT, ISP_CHN0_WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        {
            //ScopedTiming st("osd draw", atoi(argv[5]));
            cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            head_detection.draw_result(osd_frame,results,false);
            cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_CLOCKWISE);
        }
      memcpy(pic_vaddr, osd_frame.data, osd_width * osd_height * 4);*/

#if 0
 int src_w = src_img.cols;
    int src_h = src_img.rows;
    int max_src_size = std::max(src_w,src_h);

    int head_count = 0;
    for (int i = 0; i < results.size(); ++i)
    {
        auto& bbox = results[i].box; 
        //else
        {		
            int x = std::max(0,int(float(bbox.x)/ isp_shape_.width * src_w));
            int y = std::max(0,int(float(bbox.y) / isp_shape_.height * src_h));
            int w = float(bbox.width) / isp_shape_.width * src_w;
            int h = float(bbox.height) / isp_shape_.height  * src_h;
            
            if (debug_mode_ > 0)
            {
                cv::rectangle(src_img, cv::Rect(x, y , w, h), cv::Scalar(255,0, 0, 255), 3, 2, 0);
                std::string label_name = results[i].className; 
                cv::putText(src_img,label_name,cv::Point(x,y),cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(255,255, 0, 255), 1, 8, 0);
                if(results[i].class_id==0)
                {    
                    head_count += 1;
                }
            }
            else
            {
                if(results[i].class_id==0)
                {    
                    cv::rectangle(src_img, cv::Rect(x, y , w, h), cv::Scalar(255,0, 0, 255), 3, 2, 0);
                    head_count += 1;
                }
            }           
        } 
    }

   
    {
        std::string str = "headcount : " + std::to_string(head_count); 
        cv::putText(src_img,str,cv::Point(10,50),cv::FONT_HERSHEY_COMPLEX,2,cv::Scalar(255,255, 0, 255), 1, 8, 0);
    }
#endif

/*
    MobileRetinaface model("test.kmodel", 3, ISP_CHN1_HEIGHT, ISP_CHN1_WIDTH);
    DetectResult box_result;
    std::vector<face_coordinate> boxes;
    
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_vo_draw_frame vo_frame = (k_vo_draw_frame) {
    1,
    16,
    16,
    128,
    128,
    1
};
    k_video_frame_info dump_info;
    size_t size = 3 * ISP_CHN1_WIDTH * ISP_CHN1_HEIGHT;
    int face_count = 1;
    //int dump_num=0;
    while(flag_model_run)
    {
    
    //printf("dump_num:%d\n",dump_num++);
        memset(&dump_info, 0 , sizeof(k_video_frame_info));
        int ret = kd_mpi_vicap_dump_frame(dev_num, VICAP_CHN_ID_1, VICAP_DUMP_YUV, &dump_info, 1000);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_frame failed.\n");
            break;
        }

        auto vbvaddr = kd_mpi_sys_mmap(dump_info.v_frame.phys_addr[0], size);
     
        boxes.clear();

        model.run(reinterpret_cast<uintptr_t>(vbvaddr), reinterpret_cast<uintptr_t>(dump_info.v_frame.phys_addr[0]));
        kd_mpi_sys_munmap(vbvaddr, size);
        // get face boxes
        box_result = model.get_result();
        boxes = box_result.boxes;

        if(boxes.size() < face_count)
        {
            for (size_t i = boxes.size(); i < face_count; i++)
            {
                vo_frame.draw_en = 0;
                vo_frame.frame_num = i + 1;
                kd_mpi_vo_draw_frame(&vo_frame);
            }
        }

        for (size_t i = 0, j = 0; i < boxes.size(); i += 1)
        {
            //printf("(x1:%d,y1:%d)-(x2:%d,y2:%d)\n",boxes[i].x1,boxes[i].y1,boxes[i].x2,boxes[i].y2);
            vo_frame.draw_en = 1;
            vo_frame.line_x_start = ISP_CHN0_WIDTH-((uint32_t)boxes[i].y2) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_start = (((uint32_t)boxes[i].x1) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
            vo_frame.line_x_end = ISP_CHN0_WIDTH-((uint32_t)boxes[i].y1) * ISP_CHN0_WIDTH / ISP_CHN1_HEIGHT;
            vo_frame.line_y_end =(((uint32_t)boxes[i].x2) * ISP_CHN0_HEIGHT / ISP_CHN1_WIDTH);
                
            vo_frame.frame_num = ++j;
            kd_mpi_vo_draw_frame(&vo_frame);
        }

        face_count = boxes.size();
        ret = kd_mpi_vicap_dump_release(dev_num, VICAP_CHN_ID_1, &dump_info);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_release failed.\n");
        }
    usleep(10000);
    }
    
     for(size_t i = 0;i < boxes.size();i++)
    {
        vo_frame.draw_en = 0;
        vo_frame.frame_num = i + 1;
        kd_mpi_vo_draw_frame(&vo_frame);
    }
    boxes.clear();
    */
    return NULL;
}


#if 0
void video_proc(char *argv[])
{
    vivcap_start();

    k_video_frame_info vf_info;
    void *pic_vaddr = NULL;       //osd

    memset(&vf_info, 0, sizeof(vf_info));

    vf_info.v_frame.width = osd_width;
    vf_info.v_frame.height = osd_height;
    vf_info.v_frame.stride[0] = osd_width;
    vf_info.v_frame.pixel_format = PIXEL_FORMAT_ARGB_8888;
    block = vo_insert_frame(&vf_info, &pic_vaddr);

    // alloc memory
    size_t paddr = 0;
    void *vaddr = nullptr;
    size_t size = SENSOR_CHANNEL * SENSOR_HEIGHT * SENSOR_WIDTH;
    int ret = kd_mpi_sys_mmz_alloc_cached(&paddr, &vaddr, "allocate", "anonymous", size);
    if (ret)
    {
        std::cerr << "physical_memory_block::allocate failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }

    HeadDetection head_detection(argv[1], atof(argv[2]), atof(argv[3]), {SENSOR_CHANNEL, SENSOR_HEIGHT, SENSOR_WIDTH}, reinterpret_cast<uintptr_t>(vaddr), reinterpret_cast<uintptr_t>(paddr), atoi(argv[5]));

    std::vector<Detection> results;

    while (!isp_stop)
    {
        ScopedTiming st("total time", 1);

        {
            ScopedTiming st("read capture", atoi(argv[5]));
            // VICAP_CHN_ID_1 out rgb888p
            memset(&dump_info, 0 , sizeof(k_video_frame_info));
            ret = kd_mpi_vicap_dump_frame(vicap_dev, VICAP_CHN_ID_1, VICAP_DUMP_YUV, &dump_info, 1000);
            if (ret) {
                printf("sample_vicap...kd_mpi_vicap_dump_frame failed.\n");
                continue;
            }
        }
            

        {
            ScopedTiming st("isp copy", atoi(argv[5]));
            // 从vivcap中读取一帧图像到dump_info
            auto vbvaddr = kd_mpi_sys_mmap_cached(dump_info.v_frame.phys_addr[0], size);
            memcpy(vaddr, (void *)vbvaddr, SENSOR_HEIGHT * SENSOR_WIDTH * 3);  // 这里以后可以去掉，不用copy
            kd_mpi_sys_munmap(vbvaddr, size);
        }

        results.clear();

        head_detection.pre_process();
        head_detection.inference();

        head_detection.post_process({SENSOR_WIDTH, SENSOR_HEIGHT}, results,false);

        cv::Mat osd_frame(osd_height, osd_width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        #if defined(CONFIG_BOARD_K230D_CANMV)
        {
            ScopedTiming st("osd draw", atoi(argv[5]));
            cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
            head_detection.draw_result(osd_frame,results,false);
            cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_CLOCKWISE);
        }
        #elif defined(CONFIG_BOARD_K230_CANMV_01STUDIO)
        {
            #if defined(STUDIO_HDMI)
            {
                ScopedTiming st("osd draw", atoi(argv[5]));
                head_detection.draw_result(osd_frame,results,false);
            }
            #else
            {
                ScopedTiming st("osd draw", atoi(argv[5]));
                cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_COUNTERCLOCKWISE);
                head_detection.draw_result(osd_frame,results,false);
                cv::rotate(osd_frame, osd_frame, cv::ROTATE_90_CLOCKWISE);
            }
            #endif
        }
        #else
        {
            ScopedTiming st("osd draw", atoi(argv[5]));
            head_detection.draw_result(osd_frame,results,false);
        }
        #endif

        {
            ScopedTiming st("osd copy", atoi(argv[5]));
            memcpy(pic_vaddr, osd_frame.data, osd_width * osd_height * 4);
            //显示通道插入帧
            kd_mpi_vo_chn_insert_frame(osd_id+3, &vf_info);  //K_VO_OSD0
            ret = kd_mpi_vicap_dump_release(vicap_dev, VICAP_CHN_ID_1, &dump_info);
            if (ret) {
                printf("sample_vicap...kd_mpi_vicap_dump_release failed.\n");
            }
        }
    }

    vo_osd_release_block();
    vivcap_stop();


    // free memory
    ret = kd_mpi_sys_mmz_free(paddr, vaddr);
    if (ret)
    {
        std::cerr << "free failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }

}

#endif














void head_detect_start()
{
flag_model_run=true;
pthread_create(&thread_handle_head_detect, NULL, fn_head_detect, NULL);

}

void head_detect_stop()
{
flag_model_run=false;
pthread_join(thread_handle_head_detect, NULL);

}






