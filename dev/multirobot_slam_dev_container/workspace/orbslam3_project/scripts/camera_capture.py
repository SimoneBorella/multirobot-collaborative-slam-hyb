import cv2

# Open a video capture object (camera 0 is usually the default webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Unable to access the camera.")
else:
    # Get camera information
    camera_info = {
        "Width": cap.get(cv2.CAP_PROP_FRAME_WIDTH),
        "Height": cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
        "Frames per second (FPS)": cap.get(cv2.CAP_PROP_FPS),
        "FourCC Codec": cap.get(cv2.CAP_PROP_FOURCC),
        "Frame Count": cap.get(cv2.CAP_PROP_FRAME_COUNT),
        "Brightness": cap.get(cv2.CAP_PROP_BRIGHTNESS),
        "Contrast": cap.get(cv2.CAP_PROP_CONTRAST),
        "Saturation": cap.get(cv2.CAP_PROP_SATURATION),
        "Hue": cap.get(cv2.CAP_PROP_HUE),
        "Gain": cap.get(cv2.CAP_PROP_GAIN),
        "Exposure": cap.get(cv2.CAP_PROP_EXPOSURE),
        "White Balance Temperature": cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
        "Auto White Balance": cap.get(cv2.CAP_PROP_AUTO_WB),
    }

    print("Camera Information:")
    for key, value in camera_info.items():
        print(f"{key}: {value}")



while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    cv2.imshow("Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

