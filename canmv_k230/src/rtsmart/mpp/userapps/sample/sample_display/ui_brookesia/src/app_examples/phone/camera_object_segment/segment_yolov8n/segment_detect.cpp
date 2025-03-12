#include "segment_detect.h"
#include <thread>
#include "k_vo_comm.h"
#include "mpi_vicap_api.h"
#include "mpi_vo_api.h"
#include "mpi_sys_api.h"
#include <nncase/runtime/runtime_op_utility.h>
#include <iostream>
#include "segment.h"




using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;

extern k_u8 *pic_addr;
#define ISP_CHN1_HEIGHT (720)
#define ISP_CHN1_WIDTH  (1280)
#define ISP_CHN0_WIDTH  (560)
#define ISP_CHN0_HEIGHT (992)
static bool flag_model_run;
pthread_t thread_segment_detect;
static void *fn_segment_detect(void *arg)
{
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
    int object_count = 5;
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
    cv::Scalar color_list_for_segment[] = {
    cv::Scalar(255, 255, 255, 0),
    cv::Scalar(255, 0, 255, 0),
    cv::Scalar(255, 50, 220, 255),
    cv::Scalar(255, 255, 0, 255),
    cv::Scalar(255, 0, 0, 255),
    cv::Scalar(255, 0, 170, 255),
    cv::Scalar(255, 0, 255, 255)};
  
    float conf_thres=0.1;        //多目标分割 分数阈值
    float nms_thres= 0.5;       //多目标分割 非极大值抑制阈值
    float mask_thres= 0.5;     //多目标分割 掩码阈值
    int debug_mode=0;//是否需要调试，0、1、2分别表示不调试、简单调试、详细调试
    
    Seg seg("yolov8n_seg_320.kmodel", conf_thres, nms_thres, mask_thres, {3, ISP_CHN1_HEIGHT, ISP_CHN1_WIDTH}, reinterpret_cast<uintptr_t>(vaddr), reinterpret_cast<uintptr_t>(paddr), debug_mode);
    std::vector<OutputSeg> det_results;
    char segment_text[30]; 
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
  
        // get boxes
	det_results.clear(); 
	
	seg.pre_process();
        seg.inference();
        seg.post_process({ISP_CHN1_WIDTH, ISP_CHN1_HEIGHT}, det_results);
	     
        
        
        
        if(det_results.size() < object_count)
        {
            for (size_t i = det_results.size(); i < object_count; i++)
            {
                vo_frame.draw_en = 0;
                vo_frame.frame_num = i + 1;
                kd_mpi_vo_draw_frame(&vo_frame);
            }
        }
       
        //cv::Mat draw_frame(ISP_CHN0_HEIGHT, ISP_CHN0_WIDTH, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        cv::Mat draw_frame(1232, 568, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        for (int i = 0, j = 0; i < det_results.size(); ++i)
        {
        auto& bbox = det_results[i].box;
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
        
            
            memset(segment_text,0,30);
	    sprintf(segment_text, "%s,%.2f",det_results[i].label.c_str(),det_results[i].confidence);
            printf("segment detect:%s,id=%d\n",segment_text,det_results[i].id);
           //draw_frame
           {
               int x=vo_frame.line_x_start;
               int y=vo_frame.line_y_start;
               int w= vo_frame.line_x_end-vo_frame.line_x_start;
               int h= vo_frame.line_y_end-vo_frame.line_y_start;
                cv::rectangle(draw_frame, cv::Rect(x, y , w, h), cv::Scalar(255,255, 255, 255), 2, 2, 0);
		cv::putText(draw_frame, segment_text , {x,std::max(int(y-10),0)}, cv::FONT_HERSHEY_COMPLEX, 2, color_list_for_segment[det_results[i].id%6], 2, 8, 0);
                      
           }
            
        object_count+=1;
        }
        int offset=568*100*0;
        //memcpy(pic_addr, draw_frame.data, ISP_CHN0_WIDTH * ISP_CHN0_HEIGHT * 4);
        //memcpy(pic_addr+offset, draw_frame.data+offset, (568 * 1232 * 4)-offset-(ISP_CHN0_HEIGHT*568));
        memcpy(pic_addr+offset, draw_frame.data+offset, (ISP_CHN0_HEIGHT*568)-offset);
        object_count = det_results.size();
        ret = kd_mpi_vicap_dump_release(dev_num, VICAP_CHN_ID_1, &dump_info);
        if (ret) {
            printf("sample_vicap...kd_mpi_vicap_dump_release failed.\n");
        }
    usleep(10000);                 
      }//while  
        for(size_t i = 0;i < det_results.size();i++)
    {
        vo_frame.draw_en = 0;
        vo_frame.frame_num = i + 1;
        kd_mpi_vo_draw_frame(&vo_frame);
    }
    det_results.clear();
      // free memory
    ret = kd_mpi_sys_mmz_free(paddr, vaddr);
    if (ret)
    {
        std::cerr << "free failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        std::abort();
    }  
        
    return NULL;
}

void segment_detect_start()
{
flag_model_run=true;
pthread_create(&thread_segment_detect, NULL, fn_segment_detect, NULL);

}

void segment_detect_stop()
{
flag_model_run=false;
pthread_join(thread_segment_detect, NULL);

}






