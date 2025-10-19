import cv2
import numpy as np
from collections import deque

class GreenPixelTracker:
    def __init__(self):
        # --- Camera Setup ---
        camera_index = 1
        print(f"Attempting to open phone camera at index {camera_index}...")
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            print(f"⚠️ Camera {camera_index} not found — falling back to default webcam (index 0).")
            self.camera = cv2.VideoCapture(0)

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        print("✅ Camera ready.")

        # --- ArUco Setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.transform_matrix = None
        self.marker_size = 200
        self.margin = 40

        # --- Canvas Setup ---
        self.canvas_size = (720, 1280)
        self.canvas = self._make_fiducial_canvas()
        self.drawing_enabled = False
        self.last_draw_point = None

        # --- Smoothing for jitter reduction ---
        self.position_history = deque(maxlen=5)  # Keep last 5 positions
        
        # --- Color Detection (Green) - Adjusted for better sensitivity ---
        self.green_lower = np.array([40, 40, 0])  # More sensitive
        self.green_upper = np.array([80, 255, 255])
        self.min_area = 150  # Lower threshold

        print("\n✅ Ready! Project the 'Drawing Canvas (Project This)' window full-screen on the wall.")
        print("Press 'c' to calibrate, 'd' to toggle drawing, 'r' to reset canvas, 'q' to quit.")
        print("Press '+/-' to adjust detection sensitivity.\n")

    # ----------------------------------------------------------------
    def _make_fiducial_canvas(self):
        canvas = np.ones((self.canvas_size[0], self.canvas_size[1], 3), dtype=np.uint8) * 230
        aruco_dict = self.aruco_dict

        corners = [
            (self.margin, self.margin),
            (self.canvas_size[1] - self.marker_size - self.margin, self.margin),
            (self.canvas_size[1] - self.marker_size - self.margin, self.canvas_size[0] - self.marker_size - self.margin),
            (self.margin, self.canvas_size[0] - self.marker_size - self.margin)
        ]

        for i, (x, y) in enumerate(corners):
            marker = cv2.aruco.generateImageMarker(aruco_dict, i, self.marker_size)
            marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
            canvas[y:y+self.marker_size, x:x+self.marker_size] = marker_bgr

        return canvas

    # ----------------------------------------------------------------
    def detect_fiducials(self, frame):
        """Detect ArUco markers"""
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        frame_marked = frame.copy()
        
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame_marked, corners, ids)
            ids_list = ids.flatten().tolist()
            
            cv2.putText(frame_marked, f"Detected IDs: {ids_list}", (30, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if all(fid in ids_list for fid in [0, 1, 2, 3]):
                centers = {}
                for i, fid in enumerate(ids.flatten()):
                    if fid in [0, 1, 2, 3]:
                        c = corners[i][0].mean(axis=0)
                        centers[fid] = c
                
                if len(centers) == 4:
                    src_pts = np.float32([
                        centers[0],
                        centers[1],
                        centers[2],
                        centers[3]
                    ])
                    return src_pts, frame_marked
        else:
            cv2.putText(frame_marked, "No markers detected", (30, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return None, frame_marked

    def setup_transform(self, src_points):
        """Setup perspective transform with correct marker center positions"""
        w, h = self.canvas_size[1], self.canvas_size[0]
        
        # Destination points should be the CENTER of each marker on the canvas
        marker_center_offset = self.marker_size // 2
        
        dst_points = np.float32([
            [self.margin + marker_center_offset, self.margin + marker_center_offset],  # Top-left
            [w - self.margin - marker_center_offset, self.margin + marker_center_offset],  # Top-right
            [w - self.margin - marker_center_offset, h - self.margin - marker_center_offset],  # Bottom-right
            [self.margin + marker_center_offset, h - self.margin - marker_center_offset]  # Bottom-left
        ])
        
        self.transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        print("✅ Perspective transform calibrated!")
        print(f"   Marker centers mapped to canvas coordinates")

    def transform_point(self, point):
        if self.transform_matrix is None:
            return point
        p = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(p, self.transform_matrix)
        return tuple(transformed[0][0].astype(int))

    def smooth_position(self, position):
        """Apply smoothing to reduce jitter"""
        if position is None:
            return None
        
        self.position_history.append(position)
        
        # Average last N positions
        if len(self.position_history) > 0:
            avg_x = int(np.mean([p[0] for p in self.position_history]))
            avg_y = int(np.mean([p[1] for p in self.position_history]))
            return (avg_x, avg_y)
        
        return position

    def detect_green(self, frame):
        """Improved green detection with tighter filtering"""
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        # More aggressive noise reduction
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Additional erosion to tighten detection
        mask = cv2.erode(mask, kernel, iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.position_history.clear()
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            self.position_history.clear()
            return None, mask

        # Use bounding box center instead of centroid for more precision
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2
        
        return (cx, cy), mask

    def run(self):
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("⚠️ Camera feed lost — check your phone connection.")
                break

            # Detect fiducials first
            fiducials, frame_marked = self.detect_fiducials(frame)
            
            # Detect green
            green_pos, mask = self.detect_green(frame)
            
            # Apply smoothing
            green_pos_smoothed = self.smooth_position(green_pos)

            # Draw green detection on the marked frame
            if green_pos_smoothed is not None:
                # Show green circle on camera view
                cv2.circle(frame_marked, green_pos_smoothed, 10, (0, 255, 0), 2)
                cv2.circle(frame_marked, green_pos_smoothed, 3, (0, 255, 0), -1)
                
                # If calibrated, show where it maps to on canvas
                if self.transform_matrix is not None:
                    draw_pos = self.transform_point(green_pos_smoothed)
                    
                    # Clamp to canvas bounds
                    draw_pos = (
                        max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                        max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                    )
                    
                    # Only draw lines when drawing mode is ON
                    if self.drawing_enabled:
                        if self.last_draw_point is not None:
                            cv2.line(self.canvas, self.last_draw_point, draw_pos, (0, 0, 0), 3)
                        self.last_draw_point = draw_pos
                    else:
                        self.last_draw_point = None
            else:
                self.last_draw_point = None
                self.position_history.clear()

            # Add status text
            calibrated_text = "CALIBRATED" if self.transform_matrix is not None else "NOT CALIBRATED (press 'c')"
            calibrated_color = (0, 255, 0) if self.transform_matrix is not None else (0, 0, 255)
            cv2.putText(frame_marked, calibrated_text, (30, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, calibrated_color, 2)
            
            drawing_text = "DRAWING: ON" if self.drawing_enabled else "DRAWING: OFF"
            drawing_color = (0, 255, 255) if self.drawing_enabled else (128, 128, 128)
            cv2.putText(frame_marked, drawing_text, (30, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, drawing_color, 2)
            
            if green_pos_smoothed:
                cv2.putText(frame_marked, f"Pos: {green_pos_smoothed}", (30, 200),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Create a display copy with cursor overlay
            canvas_display = self.canvas.copy()

            # Draw cursor on the display copy (if we have a position)
            if green_pos_smoothed is not None and self.transform_matrix is not None:
                draw_pos = self.transform_point(green_pos_smoothed)
                draw_pos = (
                    max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                    max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                )
                cursor_color = (0, 0, 255) if self.drawing_enabled else (0, 165, 255)
                cv2.circle(canvas_display, draw_pos, 8, cursor_color, 2)
                cv2.circle(canvas_display, draw_pos, 3, cursor_color, -1)

            cv2.imshow("Phone Camera View", frame_marked)
            cv2.imshow("Drawing Canvas (Project This)", canvas_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'):
                self.drawing_enabled = not self.drawing_enabled
                self.last_draw_point = None
                print(f"Drawing mode: {'ON' if self.drawing_enabled else 'OFF'}")
            elif key == ord('c'):
                if fiducials is not None:
                    self.setup_transform(fiducials)
                else:
                    print("❌ Fiducials not detected — ensure all 4 are visible.")
            elif key == ord('r'):
                self.canvas = self._make_fiducial_canvas()
                self.last_draw_point = None
                print("Canvas reset.")
            elif key == ord('+') or key == ord('='):
                self.min_area += 50
                print(f"Min area: {self.min_area} (less sensitive)")
            elif key == ord('-') or key == ord('_'):
                self.min_area = max(100, self.min_area - 50)
                print(f"Min area: {self.min_area} (more sensitive)")

        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tracker = GreenPixelTracker()
    tracker.run()
