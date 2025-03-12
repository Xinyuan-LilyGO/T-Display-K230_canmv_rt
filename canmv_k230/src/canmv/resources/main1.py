from media.display import *
from media.media import *
from media.sensor import *
import time, os, sys, gc
import lvgl as lv
from machine import TOUCH
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
import ujson
import nncase_runtime as nn
import ulab.numpy as np
import time,utime
import image
import random
import aidemo





from media.pyaudio import * #导入pyaudio模块，用于采集和播放音频
import media.wave as wave   #导入wav模块，用于保存和加载wav音频文件


from machine import Pin,SoftSPI,SPI
from machine import FPIOA
from  libs.lora.sx127x import SX1276



DISPLAY_WIDTH = 568
DISPLAY_HEIGHT = 1232
app_exit=0
sensor = None
counter=1
#msg_transfer=""
label_lora_tx=None
label_lora_rx=None
modem_lora=None
timer_lora_send=None
timer_lora_receive=None
soft_spi=None


p_audio=None
#wf=None
#CHUNK=0
#stream_audio=None

# 自定义YOLOv8检测类
class ObjectDetectionApp(AIBase):
    def __init__(self,kmodel_path,labels,model_input_size,max_boxes_num,confidence_threshold=0.5,nms_threshold=0.2,rgb888p_size=[224,224],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        self.labels=labels
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 阈值设置
        self.confidence_threshold=confidence_threshold
        self.nms_threshold=nms_threshold
        self.max_boxes_num=max_boxes_num
        # sensor给到AI的图像分辨率
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        # 检测框预置颜色值
        self.color_four=[(255, 220, 20, 60), (255, 119, 11, 32), (255, 0, 0, 142), (255, 0, 0, 230),
                         (255, 106, 0, 228), (255, 0, 60, 100), (255, 0, 80, 100), (255, 0, 0, 70),
                         (255, 0, 0, 192), (255, 250, 170, 30), (255, 100, 170, 30), (255, 220, 220, 0),
                         (255, 175, 116, 175), (255, 250, 0, 30), (255, 165, 42, 42), (255, 255, 77, 255),
                         (255, 0, 226, 252), (255, 182, 182, 255), (255, 0, 82, 0), (255, 120, 166, 157)]
        # 宽高缩放比例
        self.x_factor = float(self.rgb888p_size[0])/self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1])/self.model_input_size[1]
        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    # 配置预处理操作，这里使用了resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            result=results[0]
            result = result.reshape((result.shape[0] * result.shape[1], result.shape[2]))
            output_data = result.transpose()
            boxes_ori = output_data[:,0:4]
            scores_ori = output_data[:,4:]
            confs_ori = np.max(scores_ori,axis=-1)
            inds_ori = np.argmax(scores_ori,axis=-1)
            boxes,scores,inds = [],[],[]
            for i in range(len(boxes_ori)):
                if confs_ori[i] > self.confidence_threshold:
                    scores.append(confs_ori[i])
                    inds.append(inds_ori[i])
                    x = boxes_ori[i,0]
                    y = boxes_ori[i,1]
                    w = boxes_ori[i,2]
                    h = boxes_ori[i,3]
                    left = int((x - 0.5 * w) * self.x_factor)
                    top = int((y - 0.5 * h) * self.y_factor)
                    right = int((x + 0.5 * w) * self.x_factor)
                    bottom = int((y + 0.5 * h) * self.y_factor)
                    boxes.append([left,top,right,bottom])
            if len(boxes)==0:
                return []
            boxes = np.array(boxes)
            scores = np.array(scores)
            inds = np.array(inds)
            # NMS过程
            keep = self.nms(boxes,scores,self.nms_threshold)
            dets = np.concatenate((boxes, scores.reshape((len(boxes),1)), inds.reshape((len(boxes),1))), axis=1)
            dets_out = []
            for keep_i in keep:
                dets_out.append(dets[keep_i])
            dets_out = np.array(dets_out)
            dets_out = dets_out[:self.max_boxes_num, :]
            return dets_out

    # 绘制结果
    def draw_result(self,osd_img,dets):
        with ScopedTiming("display_draw",self.debug_mode >0):
            if dets:
                osd_img.clear()
                for det in dets:
                    x1, y1, x2, y2 = map(lambda x: int(round(x, 0)), det[:4])
                    x= x1*self.display_size[0] // self.rgb888p_size[0]
                    y= y1*self.display_size[1] // self.rgb888p_size[1]
                    w = (x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = (y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    osd_img.draw_rectangle(x,y, w, h, color=self.get_color(int(det[5])),thickness=4)
                    osd_img.draw_string_advanced( x , y-50,32," " + self.labels[int(det[5])] + " " + str(round(det[4],2)) , color=self.get_color(int(det[5])))
            else:
                osd_img.clear()


    # 多目标检测 非最大值抑制方法实现
    def nms(self,boxes,scores,thresh):
        """Pure Python NMS baseline."""
        x1,y1,x2,y2 = boxes[:, 0],boxes[:, 1],boxes[:, 2],boxes[:, 3]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = np.argsort(scores,axis = 0)[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            new_x1,new_y1,new_x2,new_y2,new_areas = [],[],[],[],[]
            for order_i in order:
                new_x1.append(x1[order_i])
                new_x2.append(x2[order_i])
                new_y1.append(y1[order_i])
                new_y2.append(y2[order_i])
                new_areas.append(areas[order_i])
            new_x1 = np.array(new_x1)
            new_x2 = np.array(new_x2)
            new_y1 = np.array(new_y1)
            new_y2 = np.array(new_y2)
            xx1 = np.maximum(x1[i], new_x1)
            yy1 = np.maximum(y1[i], new_y1)
            xx2 = np.minimum(x2[i], new_x2)
            yy2 = np.minimum(y2[i], new_y2)
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            new_areas = np.array(new_areas)
            ovr = inter / (areas[i] + new_areas - inter)
            new_order = []
            for ovr_i,ind in enumerate(ovr):
                if ind < thresh:
                    new_order.append(order[ovr_i])
            order = np.array(new_order,dtype=np.uint8)
        return keep

    # 根据当前类别索引获取框的颜色
    def get_color(self, x):
        idx=x%len(self.color_four)
        return self.color_four[idx]
# 自定义人脸检测类，继承自AIBase基类
class FaceDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, anchors, confidence_threshold=0.5, nms_threshold=0.2, rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)  # 调用基类的构造函数
        self.kmodel_path = kmodel_path  # 模型文件路径
        self.model_input_size = model_input_size  # 模型输入分辨率
        self.confidence_threshold = confidence_threshold  # 置信度阈值
        self.nms_threshold = nms_threshold  # NMS（非极大值抑制）阈值
        self.anchors = anchors  # 锚点数据，用于目标检测
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]  # sensor给到AI的图像分辨率，并对宽度进行16的对齐
        self.display_size = [ALIGN_UP(display_size[0], 16), display_size[1]]  # 显示分辨率，并对宽度进行16的对齐
        self.debug_mode = debug_mode  # 是否开启调试模式
        self.ai2d = Ai2d(debug_mode)  # 实例化Ai2d，用于实现模型预处理
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)  # 设置Ai2d的输入输出格式和类型

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):  # 计时器，如果debug_mode大于0则开启
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size  # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，可以通过设置input_image_size自行修改输入尺寸
            top, bottom, left, right = self.get_padding_param()  # 获取padding参数
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [104, 117, 123])  # 填充边缘
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)  # 缩放图像
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])  # 构建预处理流程

    # 自定义当前任务的后处理，results是模型输出array列表，这里使用了aidemo库的face_det_post_process接口
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            post_ret = aidemo.face_det_post_process(self.confidence_threshold, self.nms_threshold, self.model_input_size[1], self.anchors, self.rgb888p_size, results)
            if len(post_ret) == 0:
                return post_ret
            else:
                return post_ret[0]

    # 绘制检测结果到画面上
    def draw_result(self, osd_img, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                osd_img.clear()  # 清除OSD图像
                for det in dets:
                    # 将检测框的坐标转换为显示分辨率下的坐标
                    x, y, w, h = map(lambda x: int(round(x, 0)), det[:4])
                    x = x * self.display_size[0] // self.rgb888p_size[0]
                    y = y * self.display_size[1] // self.rgb888p_size[1]
                    w = w * self.display_size[0] // self.rgb888p_size[0]
                    h = h * self.display_size[1] // self.rgb888p_size[1]
                    osd_img.draw_rectangle(x, y, w, h, color=(255, 255, 0, 255), thickness=2)  # 绘制矩形框
            else:
                osd_img.clear()

    # 获取padding参数
    def get_padding_param(self):
        dst_w = self.model_input_size[0]  # 模型输入宽度
        dst_h = self.model_input_size[1]  # 模型输入高度
        ratio_w = dst_w / self.rgb888p_size[0]  # 宽度缩放比例
        ratio_h = dst_h / self.rgb888p_size[1]  # 高度缩放比例
        ratio = min(ratio_w, ratio_h)  # 取较小的缩放比例
        new_w = int(ratio * self.rgb888p_size[0])  # 新宽度
        new_h = int(ratio * self.rgb888p_size[1])  # 新高度
        dw = (dst_w - new_w) / 2  # 宽度差
        dh = (dst_h - new_h) / 2  # 高度差
        top = int(round(0))
        bottom = int(round(dh * 2 + 0.1))
        left = int(round(0))
        right = int(round(dw * 2 - 0.1))
        return top, bottom, left, right

# 自定义人体关键点检测类
class PersonKeyPointApp(AIBase):
    def __init__(self,kmodel_path,model_input_size,confidence_threshold=0.2,nms_threshold=0.5,rgb888p_size=[1280,720],display_size=[1920,1080],debug_mode=0):
        super().__init__(kmodel_path,model_input_size,rgb888p_size,debug_mode)
        self.kmodel_path=kmodel_path
        # 模型输入分辨率
        self.model_input_size=model_input_size
        # 置信度阈值设置
        self.confidence_threshold=confidence_threshold
        # nms阈值设置
        self.nms_threshold=nms_threshold
        # sensor给到AI的图像分辨率
        self.rgb888p_size=[ALIGN_UP(rgb888p_size[0],16),rgb888p_size[1]]
        # 显示分辨率
        self.display_size=[ALIGN_UP(display_size[0],16),display_size[1]]
        self.debug_mode=debug_mode
        #骨骼信息
        self.SKELETON = [(16, 14),(14, 12),(17, 15),(15, 13),(12, 13),(6,  12),(7,  13),(6,  7),(6,  8),(7,  9),(8,  10),(9,  11),(2,  3),(1,  2),(1,  3),(2,  4),(3,  5),(4,  6),(5,  7)]
        #肢体颜色
        self.LIMB_COLORS = [(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255),(255, 255, 51,  255),(255, 255, 51,  255),(255, 255, 51,  255),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0)]
        #关键点颜色，共17个
        self.KPS_COLORS = [(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 0,   255, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 255, 128, 0),(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255),(255, 51,  153, 255)]

        # Ai2d实例，用于实现模型预处理
        self.ai2d=Ai2d(debug_mode)
        # 设置Ai2d的输入输出格式和类型
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT,nn.ai2d_format.NCHW_FMT,np.uint8, np.uint8)

    # 配置预处理操作，这里使用了pad和resize，Ai2d支持crop/shift/pad/resize/affine，具体代码请打开/sdcard/app/libs/AI2D.py查看
    def config_preprocess(self,input_image_size=None):
        with ScopedTiming("set preprocess config",self.debug_mode > 0):
            # 初始化ai2d预处理配置，默认为sensor给到AI的尺寸，您可以通过设置input_image_size自行修改输入尺寸
            ai2d_input_size=input_image_size if input_image_size else self.rgb888p_size
            top,bottom,left,right=self.get_padding_param()
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]],[1,3,self.model_input_size[1],self.model_input_size[0]])

    # 自定义当前任务的后处理
    def postprocess(self,results):
        with ScopedTiming("postprocess",self.debug_mode > 0):
            # 这里使用了aidemo库的person_kp_postprocess接口
            results = aidemo.person_kp_postprocess(results[0],[self.rgb888p_size[1],self.rgb888p_size[0]],self.model_input_size,self.confidence_threshold,self.nms_threshold)
            return results

    #绘制结果，绘制人体关键点
    def draw_result(self,osd_img,res):
        with ScopedTiming("display_draw",self.debug_mode >0):
            if res[0]:
                osd_img.clear()
                kpses = res[1]
                for i in range(len(res[0])):
                    for k in range(17+2):
                        if (k < 17):
                            kps_x,kps_y,kps_s = round(kpses[i][k][0]),round(kpses[i][k][1]),kpses[i][k][2]
                            kps_x1 = int(float(kps_x) * self.display_size[0] // self.rgb888p_size[0])
                            kps_y1 = int(float(kps_y) * self.display_size[1] // self.rgb888p_size[1])
                            if (kps_s > 0):
                                osd_img.draw_circle(kps_x1,kps_y1,5,self.KPS_COLORS[k],4)
                        ske = self.SKELETON[k]
                        pos1_x,pos1_y= round(kpses[i][ske[0]-1][0]),round(kpses[i][ske[0]-1][1])
                        pos1_x_ = int(float(pos1_x) * self.display_size[0] // self.rgb888p_size[0])
                        pos1_y_ = int(float(pos1_y) * self.display_size[1] // self.rgb888p_size[1])

                        pos2_x,pos2_y = round(kpses[i][(ske[1] -1)][0]),round(kpses[i][(ske[1] -1)][1])
                        pos2_x_ = int(float(pos2_x) * self.display_size[0] // self.rgb888p_size[0])
                        pos2_y_ = int(float(pos2_y) * self.display_size[1] // self.rgb888p_size[1])

                        pos1_s,pos2_s = kpses[i][(ske[0] -1)][2],kpses[i][(ske[1] -1)][2]
                        if (pos1_s > 0.0 and pos2_s >0.0):
                            osd_img.draw_line(pos1_x_,pos1_y_,pos2_x_,pos2_y_,self.LIMB_COLORS[k],4)
                    gc.collect()
            else:
                osd_img.clear()

    # 计算padding参数
    def get_padding_param(self):
        dst_w = self.model_input_size[0]
        dst_h = self.model_input_size[1]
        input_width = self.rgb888p_size[0]
        input_high = self.rgb888p_size[1]
        ratio_w = dst_w / input_width
        ratio_h = dst_h / input_high
        if ratio_w < ratio_h:
            ratio = ratio_w
        else:
            ratio = ratio_h
        new_w = (int)(ratio * input_width)
        new_h = (int)(ratio * input_high)
        dw = (dst_w - new_w) / 2
        dh = (dst_h - new_h) / 2
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw - 0.1))
        return  top, bottom, left, right














def lora_deinit():
    print(lora_deinit)
    global soft_spi
    global timer_lora_send
    global timer_lora_receive
    if timer_lora_send is not None:
        #timer_lora_send.pause()
        timer_lora_send.set_repeat_count(0)
        timer_lora_send=None
    if timer_lora_receive is not None:
        #timer_lora_receive.pause()
        timer_lora_receive.set_repeat_count(0)
        timer_lora_receive=None
    if soft_spi is not None:
        soft_spi.deinit()
        soft_spi=None
def custom_timer_cb_lora_send(void):
    global counter
    global label_lora_tx
    global modem_lora
    if soft_spi is not None and modem_lora is not None:
        print("custom_timer_cb_lora_send")
        print("Sending...")
        modem_lora.send(f"abc123#{counter}".encode())
        msg_transfer="abc123#{}".format(counter)
        print(msg_transfer)
        label_lora_tx.set_text(msg_transfer)
        counter += 1

def custom_timer_cb_lora_receive(void):
    global label_lora_rx
    global modem_lora
    print("custom_timer_cb_lora_receive")
    print("Receiving...")
    rx = modem_lora.recv(timeout_ms=1000)
    if rx:
        print(f"Received: {rx!r}")
        msg_transfer="{!r}".format(rx)
        label_lora_rx.set_text(msg_transfer)
    else:
        print("Timeout!")
        label_lora_rx.set_text("timeout...")
def lora_ui_init():
    global label_lora_tx
    global label_lora_rx
    col_dsc = [100, 400, lv.GRID_TEMPLATE_LAST]
    row_dsc = [100, 100, lv.GRID_TEMPLATE_LAST]

    # Create a container with grid
    cont = lv.obj(lv.scr_act())
    cont.set_style_grid_column_dsc_array(col_dsc, 0)
    cont.set_style_grid_row_dsc_array(row_dsc, 0)
    cont.set_size(550, 250)
    cont.center()
    cont.set_layout(lv.LAYOUT_GRID.value)

    label = lv.label(cont)
    # Stretch the cell horizontally and vertically too
    # Set span to 1 to make the cell 1 column/row sized
    label.set_grid_cell(lv.GRID_ALIGN.STRETCH, 0, 1,lv.GRID_ALIGN.STRETCH, 0, 1)
    label.set_text("Send:")
    label.center()
    #obj = lv.btn(cont)
    label_lora_tx= lv.label(cont)
    label_lora_tx.set_text("")
    # Stretch the cell horizontally and vertically too
    # Set span to 1 to make the cell 1 column/row sized
    label_lora_tx.set_grid_cell(lv.GRID_ALIGN.STRETCH, 1, 1,lv.GRID_ALIGN.STRETCH, 0, 1)
    label_lora_tx.center()

    label = lv.label(cont)
    # Stretch the cell horizontally and vertically too
    # Set span to 1 to make the cell 1 column/row sized
    label.set_grid_cell(lv.GRID_ALIGN.STRETCH, 0, 1,lv.GRID_ALIGN.STRETCH, 1, 1)
    label.set_text("Received:")
    label.center()
    #obj = lv.btn(cont)
    label_lora_rx= lv.label(cont)
    label_lora_rx.set_text("")
    # Stretch the cell horizontally and vertically too
    # Set span to 1 to make the cell 1 column/row sized
    label_lora_rx.set_grid_cell(lv.GRID_ALIGN.STRETCH, 1, 1,lv.GRID_ALIGN.STRETCH, 1, 1)
    label_lora_rx.center()

def lora_init():
    fpioa = FPIOA()
    fpioa.set_function(3,FPIOA.GPIO3)
    fpioa.set_function(4,FPIOA.GPIO4)
    fpioa.set_function(5,FPIOA.GPIO5)
    fpioa.set_function(6,FPIOA.GPIO6)
    fpioa.set_function(7,FPIOA.GPIO7)
    fpioa.set_function(8,FPIOA.GPIO8)
    fpioa.set_function(9,FPIOA.GPIO9)

    tcxo_en=Pin(3,Pin.OUT)
    #irq=Pin(4,Pin.INPUT)
    rst=Pin(5,Pin.OUT)
    cs=Pin(6,Pin.OUT)
    clk=Pin(7,Pin.OUT) ##后面随着固件注释掉
    #REST
    fpioa.help(3)
    rst.value(1)  #default 3.3v
    rst.value(0)
    rst.value(1)
    #rst.init(Pin.OUT, value=0)
    #enable lora
    tcxo_en.value(1) ##default 1.6v

def lora_test_demo():
    global counter
    global modem_lora
    global timer_lora_send
    global timer_lora_receive
    global soft_spi
    counter=0
    lora_cfg_data = {
       "freq_khz": 915000,#916000 868000 915000
       "sf": 8,#or 10
       "bw": "125",  # kHz 500
       "coding_rate": 8,
       "preamble_len": 12,
       "output_power": 17,  # dBm  17
       "tx_ant": "PA_BOOST"
    }
    if soft_spi is None:
        soft_spi=SoftSPI(0,baudrate=2000000, polarity=0, phase=0,bits=8,miso=9, mosi=8, sck=7)
    modem_lora=SX1276(spi=soft_spi,cs=Pin(6,Pin.OUT),dio0=None,dio1=None,reset=Pin(5,Pin.OUT),lora_cfg=lora_cfg_data)
    if timer_lora_send is None:
        timer_lora_send=lv.timer_create(custom_timer_cb_lora_send,2000,None)
    if timer_lora_receive is None:
        timer_lora_receive=lv.timer_create(custom_timer_cb_lora_receive,2000,None)

def exit_check():
    try:
        os.exitpoint()
    except KeyboardInterrupt as e:
        print("user stop: ", e)
        return True
    return False
def audio_init():
    global p_audio
    p_audio = PyAudio()
    p_audio.initialize() #初始化PyAudio对象
def play_audio(filename):
    try:
        global p_audio
        #创建音频输出流，设置的音频参数均为wave中获取到的参数
        wf = wave.open(filename, 'rb')#打开wav文件
        print("sampwidth:{}".format(wf.get_sampwidth()))#2:bit16,3:bit24,4:bit32
        print("format:{},channels:{},rate:{}".format(p_audio.get_format_from_width(wf.get_sampwidth()),wf.get_channels(),wf.get_framerate()))
        CHUNK = int(wf.get_framerate()/25)#设置音频chunk值
        stream_audio = p_audio.open(format=p_audio.get_format_from_width(wf.get_sampwidth()),
                    channels=wf.get_channels(),
                    rate=wf.get_framerate(),
                    output=True,frames_per_buffer=CHUNK)

        #设置音频输出流的音量
        stream_audio.volume(vol=85)
        print(f"chunk:{CHUNK}")
        data = wf.read_frames(CHUNK)#从wav文件中读取数一帧数据
        print(len(data))
        while data:
            stream_audio.write(data)  #将帧数据写入到音频输出流中
            data = wf.read_frames(CHUNK) #从wav文件中读取数一帧数据
            if exit_check():
                break
    except BaseException as e:
            print(f"Exception {e}")
    finally:
        print("")
        stream_audio.stop_stream() #停止音频输出流
        stream_audio.close()#关闭音频输出流
        #p_audio.close(stream_audio)
        #p_audio.terminate()#释放音频对象
        wf.close()#关闭wav文件
        #MediaManager.deinit() #释放vb buffer
def lora_test():
    print("lora test")
    lora_deinit()
    lora_ui_init()
    lora_init()
    lora_test_demo()
def music_test():
    print("music test")
    #play_audio("/sdcard/examples/test.wav")
    #play_audio("/sdcard/examples/44k_16.wav")
    play_audio("/sdcard/examples/utils/44k_16.wav")
    #play_audio("/sdcard/examples/44k_24.wav")#暂不支持
def hdmi_test():
    print("hdmi test")

def save_img(img, chn):
    if img.format() == image.YUV420:
        suffix = "yuv420sp"
    elif img.format() == image.RGB888:
        suffix = "rgb888"
    elif img.format() == image.RGBP888:
        suffix = "rgb888p"
    else:
        suffix = "unkown"

    filename = f"/sdcard/camera_chn_{chn:02d}_{img.width()}x{img.height()}.{suffix}"
    filename = f"/sdcard/camera_chn_{chn:02d}_{img.width()}x{img.height()}.png"
    print("save capture image to file:", filename)
    img.to_png().save(filename)

def camera_show_object_detect():
    fpioa = FPIOA()

    fpioa.set_function(7,FPIOA.IIC4_SCL)
    fpioa.set_function(8,FPIOA.IIC4_SDA)
    global sensor
    print("camera_show")
    try_count=0
    sensor.run()
    # k230保持不变，k230d可调整为[640,360]
    rgb888p_size = [568, 320]
    display_size=[568,320]
    # 模型路径
    kmodel_path="/sdcard/examples/kmodel/yolov8n_320.kmodel"
    labels = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]
    # 其它参数设置
    confidence_threshold = 0.2
    nms_threshold = 0.2
    max_boxes_num = 50
    # 初始化自定义目标检测实例
    ob_det=ObjectDetectionApp(kmodel_path,labels=labels,model_input_size=[320,320],max_boxes_num=max_boxes_num,confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    ob_det.config_preprocess()

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total",1):
                # 获取当前帧数据
                img_frame1 = sensor.snapshot(chn = CAM_CHN_ID_1)
                Display.show_image(img_frame1,x=0,y=456, alpha = 128)
                img = sensor.snapshot(chn = CAM_CHN_ID_2)
                # 推理当前帧
                res=ob_det.run(img.to_numpy_ref())
                # 绘制结果到PipeLine的osd图像
                osd_img = image.Image(568, 320, image.ARGB8888)
                ob_det.draw_result(osd_img,res)
                # 显示当前的绘制结果
                if(try_count>800):
                    osd_img.clear()
                    Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                    gc.collect()
                    break
                Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                gc.collect()
                try_count+=1
    except Exception as e:
        print(e)
        #sys.print_exception(e)
    #finally:
        #person_kp.deinit()




def camera_show_face_detect():
    fpioa = FPIOA()

    fpioa.set_function(7,FPIOA.IIC4_SCL)
    fpioa.set_function(8,FPIOA.IIC4_SDA)
    global sensor
    app_exit=0
    print("camera_show")
    try_count=0
    sensor.run()
    # k230保持不变，k230d可调整为[640,360]
    rgb888p_size = [568, 320]
    display_size=[568,320]
    # 模型路径
    kmodel_path = "/sdcard/examples/kmodel/face_detection_320.kmodel"
    # 其它参数设置
    confidence_threshold = 0.5
    nms_threshold = 0.2
    anchor_len = 4200
    det_dim = 4
    anchors_path = "/sdcard/examples/utils/prior_data_320.bin"
    anchors = np.fromfile(anchors_path, dtype=np.float)
    anchors = anchors.reshape((anchor_len, det_dim))
    # 初始化自定义人脸检测实例
    face_det = FaceDetectionApp(kmodel_path, model_input_size=[320, 320], anchors=anchors, confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, rgb888p_size=rgb888p_size, display_size=display_size, debug_mode=0)
    face_det.config_preprocess()  # 配置预处理

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total",1):
                # 获取当前帧数据
                img_frame1 = sensor.snapshot(chn = CAM_CHN_ID_1)
                Display.show_image(img_frame1,x=0,y=456, alpha = 128)
                img = sensor.snapshot(chn = CAM_CHN_ID_2)
                # 推理当前帧
                res=face_det.run(img.to_numpy_ref())
                # 绘制结果到PipeLine的osd图像
                osd_img = image.Image(568, 320, image.ARGB8888)
                face_det.draw_result(osd_img,res)
                # 显示当前的绘制结果
                if(try_count>800):
                    osd_img.clear()
                    Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                    gc.collect()
                    break
                Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                gc.collect()
                try_count+=1
    except Exception as e:
        print(e)
        #sys.print_exception(e)
    #finally:
        #person_kp.deinit()

def camera_show_person_keypoint_detect():
    fpioa = FPIOA()

    fpioa.set_function(7,FPIOA.IIC4_SCL)
    fpioa.set_function(8,FPIOA.IIC4_SDA)
    global sensor
    app_exit=0
    print("camera_show")
    try_count=0
    sensor.run()
    # k230保持不变，k230d可调整为[640,360]
    rgb888p_size = [568, 320]
    display_size=[568,320]
    # 模型路径
    kmodel_path="/sdcard/examples/kmodel/yolov8n-pose.kmodel"
    # 其它参数设置
    confidence_threshold = 0.2
    nms_threshold = 0.5
    # 初始化自定义人体关键点检测实例
    person_kp=PersonKeyPointApp(kmodel_path,model_input_size=[320,320],confidence_threshold=confidence_threshold,nms_threshold=nms_threshold,rgb888p_size=rgb888p_size,display_size=display_size,debug_mode=0)
    person_kp.config_preprocess()

    try:
        while True:
            os.exitpoint()
            with ScopedTiming("total",1):
                # 获取当前帧数据
                img_frame1 = sensor.snapshot(chn = CAM_CHN_ID_1)
                Display.show_image(img_frame1,x=0,y=456, alpha = 128)
                img = sensor.snapshot(chn = CAM_CHN_ID_2)
                # 推理当前帧
                res=person_kp.run(img.to_numpy_ref())
                # 绘制结果到PipeLine的osd图像
                osd_img = image.Image(568, 320, image.ARGB8888)
                person_kp.draw_result(osd_img,res)
                # 显示当前的绘制结果
                if(try_count>800):
                    osd_img.clear()
                    Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                    gc.collect()
                    break
                Display.show_image(osd_img,x=0,y=456, alpha = 128,layer = Display.LAYER_OSD1)
                gc.collect()
                try_count+=1
    except Exception as e:
        print(e)
        #sys.print_exception(e)
    #finally:
        #person_kp.deinit()
def sensor_init():
    global sensor
    print("sensor_init")
    fpioa = FPIOA()

    fpioa.set_function(7,FPIOA.IIC4_SCL)
    fpioa.set_function(8,FPIOA.IIC4_SDA)
    fpioa.help(7)
    fpioa.help(8)
    sensor = Sensor()
    # construct a Sensor object with default configure
    # sensor reset
    sensor.reset()
    # set hmirror
    sensor.set_hmirror(True)
    # sensor vflip
    sensor.set_vflip(True)

    # set chn0 output size
    sensor.set_framesize(width = 568, height = 320)
    # set chn0 output format
    sensor.set_pixformat(Sensor.YUV420SP)

    sensor.set_framesize(width = 568, height = 320, chn = CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB888, chn = CAM_CHN_ID_1)
    sensor.set_framesize(width = 568, height = 320, chn = CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn = CAM_CHN_ID_2)
    # bind sensor chn0 to display layer video 1
    #bind_info = sensor.bind_info(x = 0, y = 456)#456
    #Display.bind_layer(**bind_info, layer = Display.LAYER_VIDEO1)

def display_init():
    # use hdmi for display
    Display.init(Display.RM69A10,width = DISPLAY_WIDTH, height = DISPLAY_HEIGHT, to_ide = True,osd_num = 2)
    # init media manager
    MediaManager.init()

def display_deinit():
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(50)
    # deinit display
    Display.deinit()
    # release media buffer
    MediaManager.deinit()

def disp_drv_flush_cb(disp_drv, area, color):
    global disp_img1, disp_img2

    if disp_drv.flush_is_last() == True:
        if disp_img1.virtaddr() == uctypes.addressof(color.__dereference__()):
            Display.show_image(disp_img1)
            #print(f"disp disp_img1 {disp_img1}")
        else:
            Display.show_image(disp_img2)
            #print(f"disp disp_img2 {disp_img2}")
        #time.sleep(0.01)
        time.sleep_ms(10)

    disp_drv.flush_ready()

class touch_screen():
    def __init__(self):
        self.state = lv.INDEV_STATE.RELEASED

        self.indev_drv = lv.indev_create()
        self.indev_drv.set_type(lv.INDEV_TYPE.POINTER)
        self.indev_drv.set_read_cb(self.callback)
        self.touch = TOUCH(1,0)

    def callback(self, driver, data):
        x, y, state = 0, 0, lv.INDEV_STATE.RELEASED
        tp = self.touch.read(1)
        if len(tp):
            x, y, event = tp[0].x, tp[0].y, tp[0].event
            if event == 2 or event == 3 or event == 1 :
                state = lv.INDEV_STATE.PRESSED
        data.point = lv.point_t({'x': x, 'y': y})
        data.state = state
        #print(tp)
def lvgl_init():
    global disp_img1, disp_img2

    lv.init()
    disp_drv = lv.disp_create(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    disp_drv.set_flush_cb(disp_drv_flush_cb)
    #lv.scr_act().set_style_bg_opa(0, 0)
    #lv.layer_bottom().set_style_bg_opa(0, 0)
    disp_img1 = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    disp_img2 = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)
    disp_drv.set_draw_buffers(disp_img1.bytearray(), disp_img2.bytearray(), disp_img1.size(), lv.DISP_RENDER_MODE.DIRECT)
    tp = touch_screen()

def lvgl_deinit():
    global disp_img1, disp_img2

    lv.deinit()
    del disp_img1
    del disp_img2

def btn_clicked_event(event):
    btn = lv.btn.__cast__(event.get_target())
    label = lv.label.__cast__(btn.get_user_data())
    app_exit=1
    global counter
    counter += 1
    lora_deinit()
    print(f"btn_clicked_event#{counter},label={label.get_text()}")
    if "Human\nPosture" == label.get_text():
        camera_show_person_keypoint_detect()
        #camera_show_face_detect()#
        #camera_show_object_detect()
    if "Lora" == label.get_text():
        lora_test()
    if "Music" == label.get_text():
        music_test()
    if "Face\nDetect" == label.get_text():
        camera_show_face_detect()
    if "Object\nDetect" == label.get_text():
        camera_show_object_detect()
    if "HDMI" == label.get_text():
        hdmi_test()


def user_gui_init():

    # Create a container with ROW flex direction
    cont_row = lv.obj(lv.scr_act())
    cont_row.set_size(500, 150)
    cont_row.align(lv.ALIGN.BOTTOM_MID, 0, -20)
    cont_row.set_flex_flow(lv.FLEX_FLOW.ROW)

    # Add items to the row
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))
    label = lv.label(obj)
    label.set_text("Human\nPosture")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button

    # Add items to the row
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))

    label = lv.label(obj)
    label.set_text("Lora")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button
    # Add items to the row
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))
    label = lv.label(obj)
    label.set_text("Music")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button
    # Add items to the row
    '''
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))
    label = lv.label(obj)
    label.set_text("HDMI")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button
    '''
    # Add items to the row
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))
    label = lv.label(obj)
    label.set_text("Face\nDetect")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button
    # Add items to the row
    '''
    obj = lv.btn(cont_row)
    obj.set_size(100, lv.pct(100))
    label = lv.label(obj)
    label.set_text("Object\nDetect")
    label.center()
    obj.set_user_data(label)
    obj.add_event(btn_clicked_event, lv.EVENT.CLICKED, None)# Assign a callback to the button
    '''

def custom_timer_cb(void):
    print("timer_cb")

def main():
    os.exitpoint(os.EXITPOINT_ENABLE)
    try:
        audio_init()
        sensor_init()
        display_init()
        lvgl_init()
        user_gui_init()
        while True:
            time.sleep_ms(lv.task_handler())
    except BaseException as e:
        import sys
        sys.print_exception(e)
    finally:
        lvgl_deinit()
        display_deinit()
        gc.collect()

if __name__ == "__main__":
    main()
