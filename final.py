from ultralytics import YOLO
import cv2

# Load the YOLO model (replace "yolov8n.pt" with your model path if different)
model = YOLO("yolov8n.pt")

# Set the classes to detect (change [0] to the class index for "person" if different)
classes = [0]  # Assuming 0 is the index for "person"
conf_thresh = 0.5  # Confidence threshold
source = 0
# Open the webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Couldn't open webcam.")
    exit()

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()

    if not ret:
        print("Error: Couldn't read frame from webcam.")
        break
    
    # Detect people using the YOLO model
    results = model.predict(source=frame, show=False,conf=conf_thresh, classes=classes)
    # Assuming results is a list of detections
    class_names = ['person']
    # Count the number of people in the frame
    # ...

    # Count the number of people in the frame
    person_count = 0

    for result in results:
        boxes = result.boxes  # Boxes object for bbox outputs
        probs = result.probs  # Class probabilities for classification outputs
        cls = boxes.cls.tolist()  # Convert tensor to list
        xyxy = boxes.xyxy
        xywh = boxes.xywh  # box with xywh format, (N, 4)
        conf = boxes.conf
        # print(cls)
        for class_index in cls:
            class_name = class_names[int(class_index)]
            # print("Class:", class_name)
            # Check if the detected class is "person" and confidence is above a threshold
            if class_name == 'person' and conf.any() > 0.5:
                person_count += 1
                # print("Person detected!")
                # print("Bounding Box:", boxes)
                # print("Confidence:", conf)

    print("Number of people in the frame:", person_count)

    # Prepare text to display
    text = f"Total number of people: {person_count}"
    additional_text = "AeroKLE"
    additional_text2 = "SAE ADDC 2024"
    #print(text)

    # Add text to the frame
    cv2.putText(frame, additional_text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150, 0), 2)
    cv2.putText(frame, additional_text2, (15, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150,255 ), 2)
    cv2.putText(frame, text, (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 152, 255), 2)
    results = model.predict(source=frame,save=True,show=True, conf=conf_thresh, classes=classes)

    # Show the frame
    #cv2.imshow("Frame", frame)

    # Exit on 'q' key press
    if cv2.waitKey(1) == ord("q"):
        break
#print(text[:5])
# Release resources
cap.release()
cv2.destroyAllWindows()
