from ultralytics import YOLO

# Load model
model = YOLO("yolov8n.pt")
# model = YOLO("D:/A_Project/PyCharm_Project/Application_Source/runs/detect/train/weights/best.pt")
# model = YOLO("yolov8n-seg.pt")
# model = YOLO("D:/A_Project/PyCharm_Project/Application_Source/runs/segment/train/weights/best.pt")

if __name__ == '__main__':
    # Train model
    results = model.train(data="mydataset.yaml", epochs=30, device="0", batch=2)
