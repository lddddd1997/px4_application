#!/usr/bin/env python3
yolov5_path = '/home/amov/ld_ws/yolov5'
model_file = '/home/amov/ld_ws/yolov5/drone2_28.pt'
# yolov5_path = '/home/p520c/Documents/yolov5'
# model_file = '/home/p520c/Documents/yolov5/runs/train/exp19/weights/last.pt'
import sys
sys.path.append(yolov5_path)

import rospy
import time
import socket
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn
import pyrealsense2 as rs

SOCKET_ON = False
FILE = Path(yolov5_path + '/detect.py').resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import apply_classifier, check_img_size, check_imshow, check_requirements, check_suffix, colorstr, \
    increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh, LOGGER
from utils.plots import Annotator, colors
from utils.torch_utils import select_device, time_sync
from utils.datasets import letterbox

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from px4_application.msg import RealsenseTarget
from px4_application.msg import RealsenseTargets

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'yolov5s.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default=ROOT / 'data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt

def LoadImagesInfo(img):
    img_size=640
    cap=None
    path=None
    img0 = img
    img = letterbox(img0, new_shape=img_size)[0]
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    return path, img, img0, cap, ''

def Detect():
    # for path, img, im0s, vid_cap, s in dataset:
    
    dataset = LoadImagesInfo(color_image)
    path = dataset[0]
    img = dataset[1]
    im0s = dataset[2]
    vid_cap = dataset[3]
    s = dataset[4]

    t1 = time_sync()

    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim
    t2 = time_sync()
    dt[0] += t2 - t1

    # Inference
    visualize = False
    pred = model(img, augment=augment, visualize=visualize)[0]
    t3 = time_sync()
    dt[1] += t3 - t2

    # NMS
    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
    dt[2] += time_sync() - t3

    # Second-stage classifier (optional)
    if classify:
        pred = apply_classifier(pred, modelc, img, im0s)
    # Process predictions
    for i, det in enumerate(pred):  # per image
    
        im0 = im0s
        #save_path = str(save_dir / p.name)  # img.jpg
        #txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt
        s += '%gx%g ' % img.shape[2:]  # print string
        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        imc = im0.copy() if save_crop else im0  # for save_crop
        annotator = Annotator(im0, line_width=line_thickness, example=str(names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            if SOCKET_ON:
                # [[x1,y1,x2,y2,conf,cls],...]
                target_det = det.tolist()
                target_cnt = len(target_det)
                client_fd.send('y'.encode('utf-8')) # ['y' 目标数 [目标编号 x1 y1 x2 y2 conf cls ...]]
                client_fd.send('{:03d} '.format(target_cnt).encode('utf-8'))

                for target_num, target_info in enumerate(target_det):
                    # print(target_num, target_info[:])
                    client_fd.send('{:03d} {:03d} {:03d} {:03d} {:03d} {:03d} {:03d} '.format(
                    target_num,
                    int(target_info[0]),
                    int(target_info[1]),
                    int(target_info[2]),
                    int(target_info[3]),
                    int(target_info[4] * 100),
                    int(target_info[5])).encode('utf-8'))
            else:
                target_det = det.tolist()
                rs_targets = RealsenseTargets(Header(None, rospy.Time.now(), ''), []) # list
                for target_num, target_info in enumerate(target_det): # 处理检测信息
                    rs_tg = RealsenseTarget()
                    rs_tg.id = target_num
                    rs_tg.xmin = int(target_info[0])
                    rs_tg.ymin = int(target_info[1])
                    rs_tg.xmax = int(target_info[2])
                    rs_tg.ymax = int(target_info[3])
                    rs_tg.conf = int(target_info[4] * 100)
                    rs_tg.cls = int(target_info[5])

                    # realsense 3d检测
                    [pixel_x, pixel_y] = [int(639 - (target_info[0] + target_info[2]) / 2), int(479 - (target_info[1] + target_info[3]) / 2)] # 深度图目标像素点，color图翻转，深度图不翻转
                    dist_to_pixel = float(0)
                    [real_pixel_x, real_pixel_y] = [pixel_x, pixel_y]

                    bbox_w = int(target_info[2] - target_info[0])
                    bbox_h = int(target_info[3] - target_info[1])
                    # cv2.circle(im0, (pixel_x - int(bbox_w / 4), pixel_y - int(bbox_h / 4)), 3, (0, 0, 255), 5)
                    # cv2.circle(im0, (pixel_x - int(bbox_w / 4), pixel_y + int(bbox_h / 4)), 3, (0, 0, 255), 5)
                    # cv2.circle(im0, (pixel_x + int(bbox_w / 4), pixel_y - int(bbox_h / 4)), 3, (0, 0, 255), 5)
                    # cv2.circle(im0, (pixel_x + int(bbox_w / 4), pixel_y + int(bbox_h / 4)), 3, (0, 0, 255), 5)
                    for delta_x in range(int(-bbox_w / 4), int(bbox_w / 4)):
                        for delta_y in range(int(-bbox_h / 4), int(bbox_h / 4)): # 扫描中心点周围半个边框大的区域
                            scan_dist = aligned_depth_frame.get_distance(pixel_x + delta_x, pixel_y + delta_y)
                            if scan_dist != 0:
                                if dist_to_pixel == 0 or scan_dist < dist_to_pixel:
                                    dist_to_pixel = scan_dist
                                    real_pixel_x = pixel_x + delta_x
                                    real_pixel_y = pixel_y + delta_y
                    if(dist_to_pixel == 0): # 如果深度估计有问题,则忽略该目标
                        continue
                    # point_3d = rs.rs2_deproject_pixel_to_point(depth_intr, [real_pixel_x, real_pixel_y], dist_to_pixel) # 使用real_pixel_x这个可能导致xy波动
                    point_3d = rs.rs2_deproject_pixel_to_point(depth_intr, [pixel_x, pixel_x], dist_to_pixel)
                    rs_tg.position_from_realsense = Vector3(point_3d[0], point_3d[1], point_3d[2])
                    point_3d[0] = -point_3d[0] # 相机坐标系旋转
                    point_3d[1] = -point_3d[1]
                    rs_targets.realsense_targets.append(rs_tg) # list

                    cv2.circle(im0, (639 - real_pixel_x, 479 - real_pixel_y), 3, (255, 0, 0), 5) # 画出目标中心点
                    cv2.putText(im0, "detected:", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.putText(im0, "True", (130, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.putText(im0, "x: " + str(round(point_3d[0], 3)), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.putText(im0, "y: " + str(round(point_3d[1], 3)), (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.putText(im0, "z: " + str(round(point_3d[2], 3)), (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    cv2.putText(im0, "iner time: " + str(round(t3 - t2, 3)), (5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    # cv2.putText(im0, "fps: " + str(round(detect_fps, 3)), (5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                    cv2.circle(rendered_depth_image, (real_pixel_x, real_pixel_y), 3, (0, 0, 255), 5) # 画出目标中心点

                rs_targets_pub.publish(rs_targets) # 发送检测信息
            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):
                if save_txt:  # Write to file
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                    with open(txt_path + '.txt', 'a') as f:
                        f.write(('%g ' * len(line)).rstrip() % line + '\n')

                if save_img or save_crop or view_img:  # Add bbox to image
                    c = int(cls)  # integer class
                    label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(14, True)) # 14为蓝色
                    # if save_crop:
                    #     save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
        else:
        #     print("no target detected...")
            if SOCKET_ON:
                client_fd.send('n'.encode('utf-8'))
            else:
                cv2.putText(im0, "detected:", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(im0, "False", (130, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(im0, "x: " + str(round(0.0, 3)), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(im0, "y: " + str(round(0.0, 3)), (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(im0, "z: " + str(round(0.0, 3)), (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(im0, "iner time: " + str(round(t3 - t2, 3)), (5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                rs_targets_pub.publish(RealsenseTargets(Header(None, rospy.Time.now(), ''), []))
        # Print time (inference-only)
        # print(f'{s}Done. ({t3 - t2:.3f}s)')

        # Stream results
        im0 = annotator.result()
        if view_img:
            # im0 = im0[:, :, ::-1]
            # cv2.imshow("detect", im0)

            all_images = np.hstack((im0, rendered_depth_image))
            cv2.imshow("detect", all_images)
            cv2.waitKey(1)  # 1 millisecond
            if save_vid:
                vid_saver.write(im0)

        # Save results (image with detections)
        if save_img:
            if dataset.mode == 'image':
                cv2.imwrite(save_path, im0)
            else:  # 'video' or 'stream'
                if vid_path[i] != save_path:  # new video
                    vid_path[i] = save_path
                    if isinstance(vid_writer[i], cv2.VideoWriter):
                        vid_writer[i].release()  # release previous video writer
                    if vid_cap:  # video
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    else:  # stream
                        fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path += '.mp4'
                    vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                vid_writer[i].write(im0)

# def main(opt):
#     check_requirements(exclude=('tensorboard', 'thop'))
#     run(**vars(opt))

def ImageCallback(image):
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
    Detect(ros_image)

def timer_callback():
       print("tttt")

if __name__ == "__main__":
    
    #opt = parse_opt()
    #main(opt)

    if SOCKET_ON:
        socket_file = '/home/p520c/Documents/yolov5/unix_socket'
        if os.path.exists(socket_file):
            os.remove(socket_file)
            print('remove socket file: %s'%socket_file)
        else:
            print('no socket file: %s'%socket_file)

        socket_fd = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        print('create socket file: %s'%socket_file)

        socket_fd.bind(socket_file)
        socket_fd.listen(1)
        print('waiting for client to connect...')
        client_fd, client_addr = socket_fd.accept()
        print('client connected')

    weights=model_file  # model.pt path(s)
    source=0  # file/dir/URL/glob, 0 for webcam
    imgsz=[640,640]  # inference size (pixels)
    conf_thres=0.25  # confidence threshold
    iou_thres=0.45  # NMS IOU threshold
    max_det=1  # maximum detections per image
    device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    view_img=True  # show results
    save_txt=False  # save results to *.txt
    save_conf=False  # save confidences in --save-txt labels
    save_crop=False  # save cropped prediction boxes
    nosave=True  # do not save images/videos
    classes=None  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False  # class-agnostic NMS
    augment=False  # augmented inference
    visualize=False  # visualize features
    update=False  # update all models
    project=ROOT / 'runs/detect'  # save results to project/name
    name='exp'  # save results to project/name
    exist_ok=False  # existing project/name ok, do not increment
    line_thickness=3  # bounding box thickness (pixels)
    hide_labels=False  # hide labels
    hide_conf=False  # hide confidences
    half=False  # use FP16 half-precision inference
    dnn=False

    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Initialize
    device = select_device(device)
    half &= device.type != 'cpu'  # half precision only supported on CUDA
    # Load model
    w = str(weights[0] if isinstance(weights, list) else weights)
    classify, suffix, suffixes = False, Path(w).suffix.lower(), ['.pt', '.onnx', '.tflite', '.pb', '']
    check_suffix(w, suffixes)  # check weights have acceptable suffix
    pt, onnx, tflite, pb, saved_model = (suffix == x for x in suffixes)  # backend booleans
    stride, names = 64, [f'class{i}' for i in range(1000)]  # assign defaults
    
    model = torch.jit.load(w) if 'torchscript' in w else attempt_load(weights, map_location=device)
    stride = int(model.stride.max())  # model stride
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if half:
        model.half()  # to FP16
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    vid_path, vid_writer = [None], [None]
    model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once
    dt, seen = [0.0, 0.0, 0.0], 0




    rospy.init_node('yolo_detector_sub_topic')
    # image_topic = "/typhoon_h480_2/depth_camera/rgb/image_raw"
    # image_topic = "realsense2/color_image"
    # rospy.Subscriber(image_topic, Image, ImageCallback, queue_size=1, buff_size=52428800)
    
    rs_targets_pub = rospy.Publisher('yolo_detector/realsense_targets', RealsenseTargets, queue_size=5)
    
    # rospy.spin()
    config = rs.config()   #定义配置config
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)  #配置depth流
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)   #配置color流
    rs_pipe = rs.pipeline()  #定义流程pipeline
    profile = rs_pipe.start(config)  #流程开始
    align_to = rs.stream.color  #与color流对齐
    align = rs.align(align_to)


    for i in range(10):
        frames = rs_pipe.wait_for_frames()  #等待获取图像帧
        aligned_frames = align.process(frames)  #获取对齐帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧
        depth_intr = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数
        # print(depth_intr)

    colorizer = rs.colorizer()
    detect_fps = 1.0
    vid_saver = cv2.VideoWriter("/home/amov/data/exp" + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + ".mp4", cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (640, 480), True)
    save_vid = True
    while not rospy.is_shutdown():
        start_time = time_sync()
        frames = rs_pipe.wait_for_frames()  #等待获取图像帧

        aligned_frames = align.process(frames)  #获取对齐帧

        aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧
        
        image_raw = np.asanyarray(color_frame.get_data())  # RGB图
        color_image = cv2.flip(image_raw, -1) # color图翻转，深度图不翻转
        # depth_image = np.asanyarray(aligned_depth_frame.get_data()) # 深度图
        rendered_depth_image = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data()) #渲染深度图

        # all_images = np.hstack((color_image, rendered_depth_image))
        # cv2.imshow('color and depth images', all_images)
        Detect()
        end_time = time_sync()
        detect_fps = 1 / (end_time - start_time)
        # print('fps: ', 1 / (end_time - start_time))
    print("exit!")
    vid_saver.release()
    rs_pipe.stop()
        


