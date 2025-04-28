!pip install roboflow
!pip install torch torchvision torchaudio
!pip install ultralytics


from roboflow import Roboflow
rf = Roboflow(api_key="yzexk933agcrumzWH3J0")
project = rf.workspace("nust-airworks").project("lz-fplgm")
version = project.version(1)
dataset = version.download("yolov8")


# ✅ Import YOLO from Ultralytics
from ultralytics import YOLO
import torch

# ✅ Check if CUDA (GPU) is available
device = 0 if torch.cuda.is_available() else 'cpu'
print("Using device:", device)

# ✅ Load YOLOv8-nano pretrained model
model = YOLO("yolov8n.pt")

# ✅ Train the model
model.train(
    data=dataset.location + "/data.yaml",  # path to Roboflow dataset
    epochs=100,
    imgsz=640,
    batch=8,  # safer for both GPU and CPU
    device=device  # use GPU if available, else CPU
)


# Load your trained model
model = YOLO("runs/detect/train/weights/best.pt")

# Export to ONNX format
model.export(format="onnx")


model = YOLO("yolov8s.pt")
model.export(format="ncnn")


# Print scalar metrics
print("mAP50:", metrics.box.map50)
print("mAP50-95:", metrics.box.map)

# Precision and recall (per-class) — take average over all classes
precision = metrics.box.p
recall = metrics.box.r

print("Precision (avg):", sum(precision)/len(precision))
print("Recall (avg):", sum(recall)/len(recall))
