!nvidia-smi
import os
HOME = os.getcwd()
print(HOME)
# Pip install method (recommended)

!pip install ultralytics==8.2.103 -q

from IPython import display
display.clear_output()

import ultralytics
ultralytics.checks()
from ultralytics import YOLO

from IPython.display import display, Image
!pip install roboflow

from roboflow import Roboflow
rf = Roboflow(api_key="yzexk933agcrumzWH3J0")
project = rf.workspace("nust-airworks").project("lz-fplgm")
version = project.version(1)
dataset = version.download("yolov8")
%cd {HOME}

!yolo task=detect mode=train model=yolov8s.pt data={dataset.location}/data.yaml epochs=65 imgsz=800 plots=True
%cd {HOME}
Image(filename=f'{HOME}/runs/detect/train/confusion_matrix.png', width=600)
%cd {HOME}
Image(filename=f'{HOME}/runs/detect/train/results.png', width=600)
