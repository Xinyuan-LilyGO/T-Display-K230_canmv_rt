#include "face_detect.h"
#include <thread>
#include "k_vo_comm.h"
#include "mpi_vicap_api.h"
#include "mpi_vo_api.h"
#include "mpi_sys_api.h"
#include <nncase/runtime/runtime_op_utility.h>
#include "mobile_retinaface.h"

using namespace nncase;
using namespace nncase::runtime;
using namespace nncase::runtime::detail;


#define ISP_CHN1_HEIGHT (720)
#define ISP_CHN1_WIDTH  (1280)
#define ISP_CHN0_WIDTH  (560)
#define ISP_CHN0_HEIGHT (992)
static bool flag_model_run;
pthread_t thread_handle_face_detect;
static void *fn_face_detect(void *arg)
{
    MobileRetinaface model("/sdcard/test.kmodel", 3, ISP_CHN1_HEIGHT, ISP_CHN1_WIDTH);
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


void face_detect_start()
{
flag_model_run=true;
pthread_create(&thread_handle_face_detect, NULL, fn_face_detect, NULL);

}

void face_detect_stop()
{
flag_model_run=false;
pthread_join(thread_handle_face_detect, NULL);

}






