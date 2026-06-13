import cv2 as cv

def test_camera_input():
    # Initialize the camera
    cap1 = cv.VideoCapture(1)
    cap2 = cv.VideoCapture(2)
    
    # Check if the camera opened successfully
    assert cap1.isOpened(), "Error: Could not open camera."
    
    # Capture a single frame
    ret1, frame1 = cap1.read()
    #ret2, frame2 = cap2.read()
    
    

    cv.imshow("Camera 1", frame1)
    #cv.imshow("Camera 2", frame2)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #show info about frame 1
    print("Frame 1 shape:", frame1.shape)
    print("Frame 1 data type:", frame1.dtype)
    print("Frame 1 size:", frame1.size)
    print("Frame 1 dimensions:", frame1.ndim)
    print("Frame 1 channels:", frame1.shape[2] if frame1.ndim == 3 else 1)
    # Check if the frame was captured successfully
    
    # Release the camera
    cap1.release()
    cap2.release()
    
    # Optionally, you can display the captured frame (uncomment if needed)
    # cv.imshow("Captured Frame", frame)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    
    
if __name__ == "__main__":
    test_camera_input()