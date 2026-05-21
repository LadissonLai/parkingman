import cv2
import numpy as np
from fastapi import FastAPI, UploadFile, File
import uvicorn
from ultralytics import YOLO
import base64

app = FastAPI(title="YOLO-OBB Parking Space Detection Server")

# Initialize the model once globally
MODEL_PATH = '/home/u20/codes/vlm-nav/yolo-obb/runs/obb/train8/weights/best.pt'
model = YOLO(MODEL_PATH)
print(f"Server YOLO-OBB model loaded from {MODEL_PATH}")

@app.post("/detect")
async def detect(file: UploadFile = File(...)):
    # Read the uploaded image file
    contents = await file.read()
    nparr = np.frombuffer(contents, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    if img is None:
        return {"error": "Invalid image"}

    # Run inference
    results = model(img, verbose=False)
    result = results[0]

    detections = []
    if result.obb is not None:
        xywhr_data = result.obb.xywhr.cpu().numpy()
        confs_data = result.obb.conf.cpu().numpy()
        
        for i, row in enumerate(xywhr_data):
            px, py, w_px, h_px, r_rad = row
            confidence = float(confs_data[i])
            detections.append({
                "xywhr": [float(px), float(py), float(w_px), float(h_px), float(r_rad)],
                "conf": confidence
            })

    # Generate the annotated image and encode it to base64
    annotated_image = result.plot(conf=True, labels=False)
    _, buffer = cv2.imencode('.jpg', annotated_image)
    annotated_image_b64 = base64.b64encode(buffer).decode('utf-8')

    return {
        "detections": detections,
        "annotated_image": annotated_image_b64
    }

if __name__ == "__main__":
    # Start the server on port 9898
    uvicorn.run(app, host="0.0.0.0", port=9898)
