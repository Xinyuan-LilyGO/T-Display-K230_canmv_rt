#!/bin/bash

# build image classfy model
python3 ./scripts/mbv2_tflite.py --target k230 --model models/mbv2.tflite --dataset calibration_dataset

# build object detect model
python3 ./scripts/yolov5s_onnx.py --target k230 --model models/yolov5s.onnx --dataset calibration_dataset

# check simu
export PATH=$PATH:/usr/local/lib/python3.8/dist-packages/
python3 scripts/yolov5s_onnx_simu.py \
--model models/yolov5s.onnx --model_input object_detect/data/input_fp32.bin \
--kmodel tmp/yolov5s_onnx/test.kmodel --kmodel_input object_detect/data/input_uint8.bin

# build face detect model
python3 ./scripts/mobile_retinaface.py --target k230 --model models/mobile_retinaface.onnx --dataset calibration_dataset
