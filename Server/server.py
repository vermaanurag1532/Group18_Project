from flask import Flask, request, jsonify
import cv2
import mediapipe as mp
import numpy as np

app = Flask(_name_)

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)

@app.route('/detect', methods=['POST'])
def detect_face():
    # Get the image file from the request
    file = request.files.get('image')
    if file is None:
        return jsonify({"error": "No image file in request"}), 400

    file_data = file.read()
    if not file_data:
        return jsonify({"error": "Empty image file"}), 400

    print("Received image data:", len(file_data))

    # Decode the JPEG image
    np_image = np.frombuffer(file_data, np.uint8)
    image = cv2.imdecode(np_image, cv2.IMREAD_COLOR)
    if image is None:
        return jsonify({"error": "Failed to decode image"}), 400

    # Get the image dimensions
    image_height, image_width, _ = image.shape

    # Convert the BGR image to RGB
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Perform face detection
    results = face_detection.process(rgb_image)

    # Prepare response with face coordinates
    face_data = []
    if results.detections:
        for detection in results.detections:
            print(detection)
            bbox = detection.location_data.relative_bounding_box
            # Calculate bounding box center coordinates in pixels
            x_center = (bbox.xmin + bbox.width / 2) * image_width
            y_center = (bbox.ymin + bbox.height / 2) * image_height
            face_data.append({
                "x": x_center,
                "y": y_center,
                "width": bbox.width,
                "height": bbox.height,       
            })

        # Extend face_data with the recognized faces data
        # face_data.extend(face_recognition)
        # If face not dettected then enter details



    return jsonify(face_data)


@app.route('/test', methods=['GET'])
def test_route():
    return "hello"

if _name_ == '_main_':
    app.run(host='0.0.0.0', port=6000)