#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import time
import math
import yaml
import cv2
import numpy as np
import rospy

from pathlib import Path
from typing import List, Tuple
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from vision_pkg.msg import YOLOResult, Detection
from vision_pkg.srv import DetectYOLO, DetectYOLOResponse

# 尝试导入 onnxruntime
try:
    import onnxruntime as ort
except Exception as e:
    rospy.logfatal("未找到 onnxruntime，请先安装：pip3 install onnxruntime-gpu 或 pip3 install onnxruntime")
    raise

# 默认 COCO 类别
COCO_NAMES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat','traffic light',
    'fire hydrant','stop sign','parking meter','bench','bird','cat','dog','horse','sheep','cow',
    'elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase',
    'frisbee','skis','snowboard','sports ball','kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple','sandwich',
    'orange','broccoli','carrot','hot dog','pizza','donut','cake','chair','couch',
    'potted plant','bed','dining table','toilet','tv','laptop','mouse','remote','keyboard','cell phone',
    'microwave','oven','toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush'
]

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=False, scaleFill=False, stride=32):
    shape = img.shape[:2]  # (h,w)
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])  # ratio
    if not scaleFill:
        new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # width, height padding
        dw /= 2
        dh /= 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, (r, r), (left, top)
    else:
        img = cv2.resize(img, (new_shape[1], new_shape[0]), interpolation=cv2.INTER_LINEAR)
        return img, (new_shape[1] / shape[1], new_shape[0] / shape[0]), (0, 0)

def xywh2xyxy(x):
    y = np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # x1
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # y1
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # x2
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # y2
    return y

def scale_boxes(img_shape, boxes, img0_shape, ratio_pad=None):
    if ratio_pad is None:
        # 兼容：如果未提供 ratio_pad，则按比例自推导
        gain = min(img_shape[0] / img0_shape[0], img_shape[1] / img0_shape[1])
        pad = ((img_shape[1] - img0_shape[1] * gain) / 2, (img_shape[0] - img0_shape[0] * gain) / 2)  # (pad_w, pad_h)
    else:
        gain = min(img_shape[0] / img0_shape[0], img_shape[1] / img0_shape[1])
        pad = ratio_pad  # (pad_w, pad_h)

    boxes[:, [0, 2]] -= pad[0]
    boxes[:, [1, 3]] -= pad[1]
    boxes[:, :4] /= gain
    # clip
    boxes[:, 0] = boxes[:, 0].clip(0, img0_shape[1])
    boxes[:, 1] = boxes[:, 1].clip(0, img0_shape[0])
    boxes[:, 2] = boxes[:, 2].clip(0, img0_shape[1])
    boxes[:, 3] = boxes[:, 3].clip(0, img0_shape[0])
    return boxes

def nms_numpy(boxes, scores, iou_thres):
    # boxes: (N,4) in xyxy
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1).clip(min=0) * (y2 - y1).clip(min=0)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = (xx2 - xx1).clip(min=0)
        h = (yy2 - yy1).clip(min=0)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= iou_thres)[0]
        order = order[inds + 1]
    return keep

class YOLOOnnxDetector:
    def __init__(self):
        rospy.init_node('yolo_onnx_detector', anonymous=True)
        self.bridge = CvBridge()

        # 参数
        self.debug_mode = rospy.get_param('~debug_mode', True)
        self.publish_result = rospy.get_param('~publish_result', True)
        self.image_topic = rospy.get_param('~image_topic', '/vision/ffmpeg/image_raw')

        # 模型/推理参数
        self.onnx_model = rospy.get_param('~onnx_model', '/home/ymc/git/me/Dji_EP_ws/src/vision_pkg/model/fruit_best.onnx')
        self.img_size = int(rospy.get_param('~img_size', 640))
        self.conf_thres = float(rospy.get_param('~conf_thres', 0.25))
        self.iou_thres = float(rospy.get_param('~iou_thres', 0.45))
        self.max_det = int(rospy.get_param('~max_det', 1000))
        self.agnostic_nms = bool(rospy.get_param('~agnostic_nms', False))
        self.device = rospy.get_param('~device', '')  # 'cpu' or 'cuda'

        # 类别名，可用 ~names_file 覆盖
        self.names_file = rospy.get_param('~names_file', '/home/ymc/git/me/Dji_EP_ws/src/vision_pkg/model/user_voc.yaml')
        self.names = self.load_names(self.names_file) if self.names_file else COCO_NAMES

        # 切换开关
        self.start_detection_flag = False

        # 初始化 ONNX
        self.init_onnx()

        # 发布/订阅/服务
        if self.publish_result:
            self.result_pub = rospy.Publisher('yolo_result', YOLOResult, queue_size=1)
            if self.debug_mode:
                self.debug_image_pub = rospy.Publisher('yolo_debug_image', Image, queue_size=1)

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.toggle_sub = rospy.Subscriber('start_yolo_detection', Empty, self.start_yolo_detection_callback)
        self.detect_service = rospy.Service('detect_yolo', DetectYOLO, self.detect_service_callback)

        rospy.loginfo("ONNX YOLO 检测器已启动")
        rospy.loginfo(f"模型: {self.onnx_model_resolved}")
        rospy.loginfo(f"设备: {self.device_selected}")
        rospy.loginfo(f"图像话题: {self.image_topic}")
        rospy.loginfo(f"调试模式: {self.debug_mode}, 发布结果: {self.publish_result}")

    def load_names(self, path: str) -> List[str]:
        try:
            if path.endswith(('.yaml', '.yml')):
                with open(path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                names = data.get('names')
                if isinstance(names, dict):
                    # 可能是 {0:'person',1:'bicycle',...}
                    return [names[k] for k in sorted(names.keys())]
                elif isinstance(names, list):
                    return names
            else:
                # 每行一个类别
                with open(path, 'r', encoding='utf-8') as f:
                    return [line.strip() for line in f if line.strip()]
        except Exception as e:
            rospy.logwarn(f"加载类别文件失败，使用默认COCO类别: {e}")
        return COCO_NAMES

    def resolve_onnx_model(self, model_path: str) -> str:
        p = Path(model_path)
        if p.is_file() and p.suffix.lower() == '.onnx':
            return str(p.resolve())
        if p.is_dir():
            cands = sorted(glob.glob(str(p / '*.onnx')), key=lambda x: os.path.getmtime(x), reverse=True)
            if cands:
                return str(Path(cands[0]).resolve())
        raise FileNotFoundError(f"未找到ONNX模型: {model_path}")

    def init_onnx(self):
        self.onnx_model_resolved = self.resolve_onnx_model(self.onnx_model)
        providers = ['CPUExecutionProvider']
        self.device_selected = 'cpu'
        if self.device != 'cpu' and 'CUDAExecutionProvider' in ort.get_available_providers():
            providers = [('CUDAExecutionProvider', {'device_id': 0}), 'CPUExecutionProvider']
            self.device_selected = 'cuda'
        self.session = ort.InferenceSession(self.onnx_model_resolved, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        out_names = [o.name for o in self.session.get_outputs()]
        self.output_names = out_names
        rospy.loginfo(f"ONNX providers: {self.session.get_providers()}")
        rospy.loginfo(f"ONNX outputs: {self.output_names}")

    def preprocess(self, img_bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[float,float], Tuple[float,float], np.ndarray]:
        im0 = img_bgr
        img, ratio, pad = letterbox(im0, self.img_size, auto=False, scaleFill=False, stride=32)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_rgb = img_rgb.transpose(2, 0, 1).astype(np.float32)  # CHW
        img_rgb /= 255.0
        img_rgb = np.expand_dims(img_rgb, axis=0)  # NCHW
        return img_rgb, ratio, pad, im0

    def postprocess(self, preds: List[np.ndarray], ratio, pad, im0_shape) -> np.ndarray:
        # 支持 1 输出或 3 输出（拼接）
        if isinstance(preds, list):
            if len(preds) == 1:
                pred = preds[0]
            else:
                pred = np.concatenate(preds, axis=1)
        else:
            pred = preds
        if pred.ndim == 3:
            pred = np.squeeze(pred, axis=0)  # (num, no)

        # 解析 [x,y,w,h,obj,cls...]
        boxes_xywh = pred[:, :4]
        obj = pred[:, 4:5]
        cls = pred[:, 5:]
        cls_score = obj * cls  # (num, num_classes)

        cls_ids = np.argmax(cls_score, axis=1)
        scores = cls_score[np.arange(cls_score.shape[0]), cls_ids]

        # 置信度过滤
        keep = scores > self.conf_thres
        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        cls_ids = cls_ids[keep]

        if boxes_xywh.size == 0:
            return np.zeros((0, 6), dtype=np.float32)

        # xywh -> xyxy (在letterbox后的尺度)
        boxes = xywh2xyxy(boxes_xywh.copy())

        # 将 boxes 缩放回原图
        boxes = scale_boxes((self.img_size, self.img_size), boxes, im0_shape, ratio_pad=pad)

        # 类别过滤（可选）
        classes_param = rospy.get_param('~classes', None)
        if classes_param is not None and len(classes_param) > 0:
            cls_mask = np.isin(cls_ids, np.array(classes_param, dtype=int))
            boxes, scores, cls_ids = boxes[cls_mask], scores[cls_mask], cls_ids[cls_mask]

        if boxes.shape[0] == 0:
            return np.zeros((0, 6), dtype=np.float32)

        # NMS（按类或无关类）
        if self.agnostic_nms:
            keep_idx = nms_numpy(boxes, scores, self.iou_thres)
            boxes, scores, cls_ids = boxes[keep_idx], scores[keep_idx], cls_ids[keep_idx]
        else:
            final_idx = []
            for c in np.unique(cls_ids):
                idxs = np.where(cls_ids == c)[0]
                keep_c = nms_numpy(boxes[idxs], scores[idxs], self.iou_thres)
                final_idx.extend(idxs[keep_c])
            if len(final_idx):
                boxes, scores, cls_ids = boxes[final_idx], scores[final_idx], cls_ids[final_idx]
            else:
                boxes = np.zeros((0, 4)); scores = np.array([]); cls_ids = np.array([])

        # 限制数量
        if boxes.shape[0] > self.max_det:
            order = scores.argsort()[::-1][: self.max_det]
            boxes, scores, cls_ids = boxes[order], scores[order], cls_ids[order]

        # 返回 [x1,y1,x2,y2,score,cls]
        out = np.concatenate([boxes, scores[:, None], cls_ids[:, None]], axis=1)
        return out

    def detect_once(self, cv_image: np.ndarray) -> YOLOResult:
        result = YOLOResult()
        result.header = Header(stamp=rospy.Time.now(), frame_id="camera")
        result.detected = False
        result.detections = []

        try:
            inp, ratio, pad, im0 = self.preprocess(cv_image)
            ort_inputs = {self.input_name: inp}
            ort_outs = self.session.run(self.output_names, ort_inputs)
            dets = self.postprocess(ort_outs, ratio, pad, im0.shape[:2])

            if dets.shape[0] > 0:
                result.detected = True
                for x1, y1, x2, y2, conf, cls_id in dets:
                    d = Detection()
                    d.class_id = int(cls_id)
                    if 0 <= d.class_id < len(self.names):
                        d.class_name = self.names[d.class_id]
                    else:
                        d.class_name = str(d.class_id)
                    d.confidence = float(conf)

                    x1i, y1i, x2i, y2i = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
                    d.bbox.x = x1i
                    d.bbox.y = y1i
                    d.bbox.width = max(0, x2i - x1i)
                    d.bbox.height = max(0, y2i - y1i)

                    d.center.x = d.bbox.x + d.bbox.width / 2.0
                    d.center.y = d.bbox.y + d.bbox.height / 2.0
                    d.center.z = 0.0

                    result.detections.append(d)

        except Exception as e:
            rospy.logerr(f"ONNX 检测错误: {e}")

        return result

    def draw_debug(self, image: np.ndarray, res: YOLOResult) -> np.ndarray:
        canvas = image.copy()
        if res.detected:
            for det in res.detections:
                x1 = det.bbox.x
                y1 = det.bbox.y
                x2 = det.bbox.x + det.bbox.width
                y2 = det.bbox.y + det.bbox.height
                cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 215, 255), 2)
                label = f'{det.class_name} {det.confidence:.2f}'
                cv2.putText(canvas, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 215, 255), 2)
        status = f"Detections: {len(res.detections)}" if res.detected else "No detections"
        cv2.putText(canvas, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        return canvas

    def start_yolo_detection_callback(self, _msg):
        self.start_detection_flag = not self.start_detection_flag
        rospy.loginfo("YOLO检测 %s" % ("开始" if self.start_detection_flag else "关闭"))

    def image_callback(self, msg: Image):
        if not self.start_detection_flag:
            if self.debug_mode and self.publish_result:
                # 可选：发布未开始提示图
                pass
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            res = self.detect_once(cv_image)

            if self.publish_result:
                self.result_pub.publish(res)
                if self.debug_mode:
                    dbg = self.draw_debug(cv_image, res)
                    dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
                    dbg_msg.header = msg.header
                    self.debug_image_pub.publish(dbg_msg)

        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")

    def detect_service_callback(self, req):
        resp = DetectYOLOResponse()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
            res = self.detect_once(cv_image)
            resp.success = True
            resp.result = res
            resp.message = "检测完成" + (f", 检测到{len(res.detections)}个目标" if res.detected else ", 未检测到目标")
        except Exception as e:
            resp.success = False
            resp.message = f"检测失败: {e}"
        return resp

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YOLOOnnxDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass