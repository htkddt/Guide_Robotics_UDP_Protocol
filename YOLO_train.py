from ultralytics import YOLO

# Load model
# model = YOLO("yolov5nu.pt")
model = YOLO("yolov8n.pt")

if __name__ == '__main__':
    # Train model
    results = model.train(data="mydataset.yaml", epochs=40, device="0", batch=2)
