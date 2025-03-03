import os

from ultralytics import YOLO

from roast.glogging import Logger
from roast.settings import ROOT_DIR
from roast.vision.configs import ObjectDetectionParameters


class ObjectDetection:
    """Object tracking class."""

    LOG = Logger(_module_name="Object Detection")

    model = None
    device = None

    def __init__(
        self,
        model_path: str = "yolov8_grayscale_v0.pt",
        use_gpu=True,
        debug=False,
        output_info="xyxy",
    ):
        """Object Detection Initialization

        Args:
            model_path (str, optional): Path to the model.
                                        Defaults to "yolov8_grayscale_v0.torchscript".
            use_gpu (bool, optional): Use CUDA devices. Defaults to True.
            debug (bool, optional): Add Debug information. Defaults to False.
            output_info (str, optional): The output format for the detection.
                . Possible options: `xyxy`, `xyxyn`, `xywh`, `xywhn`. Defaults to "xyxy".
        """
        self._weight_path = os.path.join(ROOT_DIR, "models", model_path)
        self.use_gpu = use_gpu
        self.debug = debug

        if output_info not in ["xyxy", "xyxyn", "xywh", "xywhn"]:
            raise ValueError(
                "Invalid output_info. Possible options: `xyxy`, `xyxyn`, `xywh`, `xywhn`"
            )
        self.output_info = output_info

        self.model = YOLO(self._weight_path, task="detect")

        self.LOG.INFO("Object Detection Model Initialized.")

    def detect(self, img, format_result=True):
        """Detect objects in an image.

        Args:
        img (numpy.ndarray): Undistorted image to detect objects in.

        Return:
        tuple: A tuple containing the bounding boxes, scores, and labels."""

        prediction = self.model(
            img,
            iou=ObjectDetectionParameters.iou_threshold,
            conf=ObjectDetectionParameters.confidence_threshold,
            device=ObjectDetectionParameters.device if self.use_gpu else "cpu",
            agnostic_nms=ObjectDetectionParameters.agnostic_nms,
            max_det=ObjectDetectionParameters.max_detections,
            verbose=self.debug,
        )
        if format_result:
            return self._format_result(prediction)

        return prediction

    def _format_result(self, prediction):
        """Format the result of the detection."""

        bounding_boxes = []
        for result in prediction:
            bboxes = result.boxes

            # Parse the results
            if self.output_info == "xyxy":
                bb = bboxes.xyxy.tolist()
            elif self.output_info == "xywh":
                bb = bboxes.xywh.tolist()
            elif self.output_info == "xyxyn":
                bb = bboxes.xyxyn.tolist()
            elif self.output_info == "xywhn":
                bb = bboxes.xywhn.tolist()

            bounding_boxes.append(bb)

        return bounding_boxes
