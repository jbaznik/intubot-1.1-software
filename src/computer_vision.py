import sys
import json
import time
import cv2
from PIL import Image
import torch
import torch.utils.data
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import torchvision



class ComputerVision:
    def __init__(self, default_threshold=0.5):
        self.model = None
        self.model_name = None
        self.changing_model = False
        self.common_objects_running = False
        self.processing_one_frame = False

        self.threshold_map = {}
        self.label_list = []
        self.num_classes = 1

        self.default_threshold = default_threshold

        if torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = None

    def load(self, model_name, path_to_weights, path_to_labels, path_to_thresholds):
        """
        loads data from files, downloads PyTorch model and adapts it to our need
        """
        if not self.device:
            print('CUDA not available')
            sys.exit(0)

        self.changing_model = True
        while self.processing_one_frame:
            time.sleep(0.01)  # wait till last frame is finished processing before changing model

        # load labels and thresholds from files. stop program is files are not found
        try:
            with open(path_to_thresholds, encoding='utf-8') as f:
                self.threshold_map = json.load(f)
        except FileNotFoundError:
            print(f'file {path_to_thresholds} not found')
            sys.exit(0)
        try:
            with open(path_to_labels, encoding='utf-8') as f:
                self.label_list = json.load(f)
        except FileNotFoundError:
            print(f'file {path_to_labels} not found')
            sys.exit(0)

        self.model_name = model_name
        # calculate number of classes (label_list must include background as class 0)
        self.num_classes = len(list(self.label_list))

        # load an object detection model pre-trained on COCO
        model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(
            pretrained=True)  # get the number of input features for the classifier
        in_features = model.roi_heads.box_predictor.cls_score.in_features

        # replace the pre-trained head with a new one
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, self.num_classes)
        model.load_state_dict(torch.load(path_to_weights, map_location=self.device))
        model = model.to(self.device)
        model.eval()
        self.model = model
        self.changing_model = False

    def get_rel_dimensions(self, box, frame_size):
        """
        Calculates relative dimensions of the box (area, center, width, height)

        Args:
            box (list): [x_min, y_min, x_max, y_max]
            frame_size (list): [height, width]

        return:
            float: center_x
            float: center_y
            float: width
            float: height
            float: area
        """
        xmin, ymin, xmax, ymax = box
        center_x_abs = (xmin + xmax)//2
        center_y_abs = (ymin + ymax)//2

        # normalize by frame width
        center_x_rel = center_x_abs/frame_size[1]
        center_y_rel = center_y_abs/frame_size[1]

        width_abs = xmax-xmin
        height_abs = ymax-ymin
        abs_area = width_abs*height_abs
        frame_area = frame_size[0]*frame_size[1]
        rel_area = abs_area/frame_area

        width_rel = width_abs/frame_size[1]
        height_rel = height_abs/frame_size[1]

        return [center_x_rel, center_y_rel, width_rel, height_rel, rel_area]

    def object_detection(self, frame):
        """
        performs object detection on frame
        return a dict of best predictions
        """
        # without frame or model or device, no detection can be performed
        if frame is None or not self.model or not self.device or self.changing_model is True:
            return {}

        self.processing_one_frame = True

        # run PyTorch Object detection
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        image_tensor = torchvision.transforms.ToTensor()(image)
        with torch.no_grad():
            prediction = self.model([image_tensor.to(self.device)])

        # only store the best prediction per label and ignore predictions lower than defined threshold
        best_prediction = {}
        for element in range(len(prediction[0]["boxes"])):
            box = prediction[0]["boxes"][element].cpu().tolist()  # [x_min, y_min, x_max, y_max]
            score = prediction[0]["scores"][element].cpu().item()
            label_num = int(prediction[0]["labels"][element].cpu().item())
            label = self.label_list[label_num]

            if (label in self.threshold_map and score > self.threshold_map[label]) or score > self.default_threshold:
                if label not in best_prediction or score > best_prediction[label]['score']:
                    dimensions = self.get_rel_dimensions(box, frame.shape)
                    best_prediction[label] = {
                        'score': score, 'box': box, 'center': dimensions[0: 2],
                        'width': dimensions[2],
                        'height': dimensions[3],
                        'area': dimensions[4]}

        self.processing_one_frame = False
        return best_prediction
