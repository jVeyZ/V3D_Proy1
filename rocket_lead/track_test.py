import argparse
import cv2
import numpy as np


class ROISelector:
    def __init__(self):
        self.dragging = False
        self.start = None
        self.end = None
        self.roi = None

    def reset(self):
        self.dragging = False
        self.start = None
        self.end = None
        self.roi = None

    def on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start = (x, y)
            self.end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.end = (x, y)
            self.roi = self._rect_from_points(self.start, self.end)

    @staticmethod
    def _rect_from_points(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        return (x, y, w, h)

    def draw(self, frame):
        if self.start is None or self.end is None:
            return frame
        if self.dragging:
            x, y, w, h = self._rect_from_points(self.start, self.end)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 200, 0), 2)
        elif self.roi is not None:
            x, y, w, h = self.roi
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 200, 0), 2)
        return frame


class ColorTracker:
    def __init__(self, hsv_margin=(12, 50, 50), min_area=400):
        self.hsv_lower = None
        self.hsv_upper = None
        self.hsv_margin = np.array(hsv_margin, dtype=np.int32)
        self.min_area = int(min_area)
        self.last_bbox = None

    def set_from_roi(self, frame_bgr, roi):
        x, y, w, h = roi
        if w <= 5 or h <= 5:
            return False
        crop = frame_bgr[y:y + h, x:x + w]
        if crop.size == 0:
            return False
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        center = np.median(hsv.reshape(-1, 3), axis=0).astype(np.int32)
        lower = np.clip(center - self.hsv_margin, 0, 255)
        upper = np.clip(center + self.hsv_margin, 0, 255)
        self.hsv_lower = lower.astype(np.uint8)
        self.hsv_upper = upper.astype(np.uint8)
        return True

    def reset(self):
        self.hsv_lower = None
        self.hsv_upper = None
        self.last_bbox = None

    def track(self, frame_bgr):
        if self.hsv_lower is None or self.hsv_upper is None:
            return None
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.min_area:
            return None
        x, y, w, h = cv2.boundingRect(best)
        self.last_bbox = (x, y, w, h)
        return self.last_bbox


def parse_args():
    parser = argparse.ArgumentParser(description="Color tracker test with ROI selection")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    parser.add_argument("--min-area", type=int, default=400, help="Minimum contour area")
    return parser.parse_args()


def main():
    args = parse_args()
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("No se pudo abrir la camara.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    selector = ROISelector()
    tracker = ColorTracker(min_area=args.min_area)

    window = "Color Tracker"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, selector.on_mouse)

    print("Arrastra para seleccionar ROI. Teclas: s=usar ROI, c=limpiar, q=salir")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Frame no disponible")
            break

        if selector.roi is not None:
            selector.draw(frame)

        bbox = tracker.track(frame)
        if bbox is not None:
            x, y, w, h = bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Tracking", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        elif tracker.hsv_lower is not None:
            cv2.putText(frame, "Sin deteccion", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        else:
            cv2.putText(frame, "Selecciona ROI", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

        cv2.imshow(window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("c"):
            selector.reset()
            tracker.reset()
        if key == ord("s") and selector.roi is not None:
            if tracker.set_from_roi(frame, selector.roi):
                print("ROI registrada para tracking")
            else:
                print("ROI invalida")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
