from ctypes import *
import random
import os
import time
import darknet
import argparse
try:
    import sys
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2
except Exception as e:
    pass


def parser():
    parser = argparse.ArgumentParser(description="YOLO Object Detection")
    parser.add_argument("--input", type=str, default=0,
                        help="video source. If empty, uses webcam 0 stream")
    parser.add_argument("--out_filename", type=str, default="",
                        help="inference video name. Not saved if empty")
    parser.add_argument("--weights", default="./z_weight_cfg_data/yolov4-tiny_kudos_last.weights",
                        help="yolo weights path")
    parser.add_argument("--dont_show", action='store_true',
                        help="windown inference display. For headless systems")
    parser.add_argument("--ext_output", action='store_true',
                        help="display bbox coordinates of detected objects")
    parser.add_argument("--config_file", default="./z_weight_cfg_data/yolov4-tiny_kudos.cfg",
                        help="path to config file")
    parser.add_argument("--data_file", default="./z_weight_cfg_data/kudos_obj.data",
                        help="path to data file")
    parser.add_argument("--thresh", type=float, default=.30,
                        help="remove detections with confidence below this value")
    return parser.parse_args()


def str2int(video_path):
    """
    argparse returns and string althout webcam uses int (0, 1 ...)
    Cast to int if needed
    """
    try:
        return int(video_path)
    except ValueError:
        return video_path


def check_arguments_errors(args):
    assert 0 < args.thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
    if not os.path.exists(args.config_file):
        raise(ValueError("Invalid config path {}".format(os.path.abspath(args.config_file))))
    if not os.path.exists(args.weights):
        raise(ValueError("Invalid weight path {}".format(os.path.abspath(args.weights))))
    if not os.path.exists(args.data_file):
        raise(ValueError("Invalid data file path {}".format(os.path.abspath(args.data_file))))
    if str2int(args.input) == str and not os.path.exists(args.input):
        raise(ValueError("Invalid video path {}".format(os.path.abspath(args.input))))


def set_saved_video(output_video, size):
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    #fps = int(input_video.get(cv2.CAP_PROP_FPS))
    fps = 60
    video = cv2.VideoWriter(output_video, fourcc, fps, size)
    return video

def Initialize_darknet(args):
    network, class_names, class_colors = darknet.load_network(
        args.config_file,
        args.data_file, 
        args.weights,
        batch_size=1,
    )
    width = darknet.network_width(network)
    height = darknet.network_height(network)
    return network, class_names, class_colors, width, height

def getResults_with_darknet(ret, frame, darknet_input_width, darknet_input_height, darknet_network, darknet_class_names, darknet_class_colors,config_args):
    if ret != False:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (darknet_input_width, darknet_input_height), interpolation=cv2.INTER_LINEAR)
        img_for_detect = darknet.make_image(darknet_input_width, darknet_input_height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        prev_time = time.time()
        detections = darknet.detect_image(darknet_network, darknet_class_names, img_for_detect, thresh=config_args.thresh)
        fps = int(1/(time.time() - prev_time))
        print("FPS: {}".format(fps))
        darknet.print_detections(detections, config_args.ext_output)
        darknet.free_image(img_for_detect)
        random.seed(3)  # deterministic bbox colors
        video = set_saved_video(config_args.out_filename, (darknet_input_width, darknet_input_height))

        if frame_resized is not None:
            image = darknet.draw_boxes(detections, frame_resized, darknet_class_colors)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            if config_args.out_filename is not None:
                video.write(image)
            if not config_args.dont_show:
                return image, detections
