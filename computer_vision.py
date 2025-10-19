import cv2
import numpy as np
from collections import deque
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import time

# BLE UUIDs - MUST match the ESP32 code
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class GreenPixelTracker:
    def __init__(self):
        print("\n" + "="*60)
        print("🎨 PRESSURE-SENSITIVE DRAWING SYSTEM")
        print("="*60)

        # ---------- Performance knobs (PC-side) ----------
        self.proc_fps = 20           # Vision processing rate (Hz)
        self.ui_fps = 30             # UI refresh rate (Hz)
        self.proc_period_ms = int(1000 / self.proc_fps)
        self.ui_period_ms = int(1000 / self.ui_fps)
        self.last_proc_ms = 0
        self.last_ui_ms = 0

        # LED ROI search window (pixels). Smaller → faster, but be sure it's big enough.
        self.roi_half = 80           # half-size of ROI box around last LED position
        self.roi_downscale = 1.0     # 0.5 to speed up more (coords are scaled back)

        # ---------- Camera Setup ----------
        camera_index = 1
        print(f"\n📷 Initializing camera...")
        print(f"   Trying camera index {camera_index} (iPhone/External)...")

        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            print(f"   ⚠️  Camera {camera_index} not available")
            print(f"   Trying default webcam (index 0)...")
            self.camera = cv2.VideoCapture(0)

        if self.camera.isOpened():
            # Reduce resolution for better performance
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # Drop buffering so we don't build a backlog
            try:
                self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass

            # OpenCV perf flags: fewer threads = less CPU jitter, keep optimizations on
            try:
                cv2.setUseOptimized(True)
                cv2.setNumThreads(2)   # adjust 1–4 depending on your CPU
            except Exception:
                pass

            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"   ✅ Camera ready: {actual_width}x{actual_height} (reduced for performance)")
        else:
            print("   ❌ No camera available!")
            exit(1)

        # ---------- ArUco Setup ----------
        print("\n🎯 Setting up ArUco marker detection...")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.transform_matrix = None
        self.marker_size = 200
        self.margin = 40
        self.awaiting_calibration = False  # only detect markers when True
        print("   ✅ ArUco detector initialized")
        print("   Looking for markers: 0, 1, 2, 3")

        # ---------- Canvas Setup ----------
        print("\n🖼️  Creating canvas...")
        self.canvas_size = (720, 1280)
        self.canvas = self._make_fiducial_canvas()
        self.calibration_canvas = self.canvas.copy()
        self.is_calibrated = False
        self.last_draw_point = None
        print(f"   ✅ Canvas created: {self.canvas_size[1]}x{self.canvas_size[0]}")
        print("   ArUco markers placed at corners")

        # ---------- Smoothing ----------
        self.position_history = deque(maxlen=5)
        self.last_blue_pos_raw = None  # for ROI seeding

        # ---------- Color Detection (Blue LED) ----------
        print("\n🔵 Configuring blue LED tracking...")
        self.blue_lower = np.array([200, 100, 50])   # Lower bound: Hue, Saturation, Value
        self.blue_upper = np.array([230, 255, 255])  # Upper bound
        self.min_area = 50
        print("   ✅ HSV range configured for blue LED")
        print(f"   Hue: 100-130, Saturation: 150-255, Value: 50-255")
        print(f"   Minimum detection area: {self.min_area} pixels")

        # ---------- Force Sensor Setup ----------
        print("\n⚡ Force sensor configuration...")
        self.force_value = 0
        self.force_threshold = 614  # ~0.6 N
        self.min_thickness = 1
        self.max_thickness = 15
        self.force_max = 1024
        self.sensor_max_newtons = 1.0
        print(f"   Initial threshold: {self.force_threshold} ADC (~{self.to_newtons(self.force_threshold):.3f}N)")
        print(f"   Line thickness range: {self.min_thickness}-{self.max_thickness} pixels")
        print(f"   Max sensor value: {self.force_max} (~{self.sensor_max_newtons}N)")

        # ---------- BLE ----------
        self.ble_client = None
        self.ble_connected = False
        self.running = True
        self.last_data_received = 0
        self.device_address = None   # cache address to skip scans next time

        print("\n🚀 Starting BLE connection thread...")
        self.ble_thread = threading.Thread(target=self.run_ble_loop, daemon=True)
        self.ble_thread.start()

        print("\n" + "="*60)
        print("✅ SYSTEM READY")
        print("="*60)
        print("📋 Instructions:")
        print("   1. Wait for BLE connection to 'ForceStylus'")
        print("   2. Project 'Drawing Canvas' window full-screen")
        print("   3. Press 'c' to calibrate (show all 4 markers)")
        print("   4. Point blue LED at canvas and press force sensor to draw!")
        print("\n⌨️  Keyboard Controls:")
        print("   c  = Calibrate system (shows markers, then switches to black)")
        print("   r  = Reset canvas")
        print("   +  = Increase force threshold (harder press needed)")
        print("   -  = Decrease force threshold (easier to draw)")
        print("   q  = Quit")
        print("="*60 + "\n")

    # ----------------------------------------------------------------
    # BLE Functions
    # ----------------------------------------------------------------
    def run_ble_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect_ble())
        loop.close()

    async def fast_find_device(self, name_substring="ForceStylus", timeout=0.4):
        """
        Try to find quickly by name; returns device or None.
        Uses find_device_by_filter when available; falls back to discover.
        """
        try:
            # Newer Bleak has this utility; quicker than full discovery on some OSes
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name and name_substring in d.name), timeout=timeout
            )
            return device
        except Exception:
            # Fallback: short discovery
            devices = await BleakScanner.discover(timeout=timeout)
            for d in devices:
                if d.name and name_substring in d.name:
                    return d
            return None

    def _on_disconnect(self, client):
        # Called from Bleak transport thread
        print("\n🔌 BLE disconnected (callback).")
        self.ble_connected = False

    async def connect_ble(self):
        """Find and connect to the ESP32 via BLE with fast auto-reconnect"""
        backoff = 0.3  # very quick retry

        while self.running:
            print("\n" + "="*60)
            print("🔍 SCANNING/CONNECTING BLUETOOTH")
            print("="*60)

            try:
                device = None
                if self.device_address:
                    # Skip scan: connect directly to cached address
                    print(f"   Using cached address: {self.device_address}")
                    device = type("Tmp", (), {"address": self.device_address})()
                else:
                    print("   Fast find 'ForceStylus'… (≤0.4s)")
                    device = await self.fast_find_device("ForceStylus", timeout=0.4)

                if not device:
                    print("   ❌ Not found. Quick retry…")
                    await asyncio.sleep(backoff)
                    continue

                address = device.address if hasattr(device, "address") else device
                print(f"\n🔗 Connecting to {address} …")
                async with BleakClient(address, disconnected_callback=self._on_disconnect, timeout=5.0) as client:
                    self.ble_client = client
                    self.ble_connected = True
                    if not self.device_address:
                        # Cache for future ultra-fast reconnects
                        self.device_address = address

                    print("✅ Connected successfully!")
                    print("📡 Subscribing to force sensor notifications…")
                    await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                    print("✅ Receiving force data from ESP32!")

                    # Lightweight keep-alive / progress
                    last_value_print = -1
                    last_data_time = time.monotonic()

                    while self.running and client.is_connected:
                        # Watchdog for data
                        if (time.monotonic() - last_data_time) > 2.0:
                            print("\n⚠️  No data in 2s (still connected)…")
                            last_data_time = time.monotonic()

                        if self.force_value != last_value_print:
                            # Do not spam; only print when visible change
                            last_value_print = self.force_value
                            last_data_time = time.monotonic()

                        await asyncio.sleep(0.02)  # 50 Hz cooperative yield

                    # Clean up
                    try:
                        if client.is_connected:
                            await client.stop_notify(CHARACTERISTIC_UUID)
                    except Exception:
                        pass

            except Exception as e:
                print(f"\n❌ BLE Error: {e}")

            finally:
                self.ble_connected = False
                print("✋ BLE connection closed")
                if self.running:
                    print(f"🔄 Reconnect in {backoff:.1f}s…")
                    await asyncio.sleep(backoff)

    def notification_handler(self, sender, data):
        """Called whenever ESP32 sends new force data (2 bytes: LSB, MSB)"""
        if len(data) == 2:
            self.force_value = data[0] | (data[1] << 8)
            self.last_data_received = time.time()

    # ----------------------------------------------------------------
    # Force Sensor Logic
    # ----------------------------------------------------------------
    def get_line_thickness(self):
        if self.force_value < self.force_threshold:
            return 0
        force_range = self.force_max - self.force_threshold
        thickness_range = self.max_thickness - self.min_thickness
        force_above = max(0, self.force_value - self.force_threshold)
        thickness = self.min_thickness + (force_above / max(1, force_range)) * thickness_range
        return int(max(self.min_thickness, min(self.max_thickness, thickness)))

    def is_drawing_enabled(self):
        return self.force_value >= self.force_threshold

    def to_newtons(self, adc_value):
        return (adc_value / self.force_max) * self.sensor_max_newtons

    # ----------------------------------------------------------------
    # Canvas and Vision
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

    def _make_clean_canvas(self):
        return np.zeros((self.canvas_size[0], self.canvas_size[1], 3), dtype=np.uint8)

    def detect_fiducials(self, frame):
        """Detect ArUco markers (only called in calibration mode)"""
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
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
                    src_pts = np.float32([centers[0], centers[1], centers[2], centers[3]])
                    return src_pts, frame_marked
        else:
            cv2.putText(frame_marked, "No markers detected", (30, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return None, frame_marked

    def setup_transform(self, src_points):
        w, h = self.canvas_size[1], self.canvas_size[0]
        marker_center_offset = self.marker_size // 2
        dst_points = np.float32([
            [self.margin + marker_center_offset, self.margin + marker_center_offset],
            [w - self.margin - marker_center_offset, self.margin + marker_center_offset],
            [w - self.margin - marker_center_offset, h - self.margin - marker_center_offset],
            [self.margin + marker_center_offset, h - self.margin - marker_center_offset]
        ])
        self.transform_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        self.is_calibrated = True
        self.awaiting_calibration = False
        self.canvas = self._make_clean_canvas()
        print("\n" + "="*60)
        print("✅ CALIBRATION SUCCESSFUL!")
        print("="*60)

    def transform_point(self, point):
        if self.transform_matrix is None:
            return point
        p = np.array([[[point[0], point[1]]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(p, self.transform_matrix)
        return tuple(transformed[0][0].astype(int))

    def smooth_position(self, position):
        if position is None:
            return None
        self.position_history.append(position)
        if len(self.position_history) > 0:
            avg_x = int(np.mean([p[0] for p in self.position_history]))
            avg_y = int(np.mean([p[1] for p in self.position_history]))
            return (avg_x, avg_y)
        return position

    def detect_blue(self, frame):
        """
        Detect blue LED with ROI acceleration:
        - If we have a last position, search a small box around it.
        - Optional downscale inside ROI to reduce work even more.
        """
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        H, W = frame.shape[:2]
        # Choose ROI
        if self.last_blue_pos_raw is not None:
            cx, cy = self.last_blue_pos_raw
            x0 = max(0, cx - self.roi_half)
            y0 = max(0, cy - self.roi_half)
            x1 = min(W, cx + self.roi_half)
            y1 = min(H, cy + self.roi_half)
            roi = frame[y0:y1, x0:x1]
            offset = (x0, y0)
        else:
            roi = frame
            offset = (0, 0)

        # Optional downscale inside ROI
        if self.roi_downscale != 1.0:
            roi_small = cv2.resize(roi, None, fx=self.roi_downscale, fy=self.roi_downscale, interpolation=cv2.INTER_LINEAR)
        else:
            roi_small = roi

        hsv = cv2.cvtColor(roi_small, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        # Simple morphology
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.position_history.clear()
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            self.position_history.clear()
            return None, mask

        M = cv2.moments(largest)
        if M["m00"] <= 0:
            return None, mask

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # Scale back if we downscaled ROI
        if self.roi_downscale != 1.0:
            cx = int(cx / self.roi_downscale)
            cy = int(cy / self.roi_downscale)

        # Convert to full-frame coordinates
        cx += offset[0]
        cy += offset[1]

        # Update last raw pos for next ROI
        self.last_blue_pos_raw = (cx, cy)
        return (cx, cy), mask

    # ----------------------------------------------------------------
    # Main Loop
    # ----------------------------------------------------------------
    def run(self):
        last_frame_marked = None
        last_canvas_display = None

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("⚠️ Camera feed lost")
                break

            now_ms = int(time.time() * 1000)

            # --- PROCESSING: only at proc_fps ---
            if now_ms - self.last_proc_ms >= self.proc_period_ms:
                self.last_proc_ms = now_ms

                # Only run ArUco detection when calibrating or not calibrated
                if (not self.is_calibrated) or self.awaiting_calibration:
                    fiducials, frame_marked = self.detect_fiducials(frame)
                else:
                    fiducials = None
                    frame_marked = frame.copy()

                # LED detection (fast, uses ROI)
                blue_pos, _ = self.detect_blue(frame)
                blue_pos_smoothed = self.smooth_position(blue_pos)

                # Get drawing parameters from force sensor
                drawing_enabled = self.is_drawing_enabled()
                line_thickness = self.get_line_thickness()

                # Draw blue LED cursor on camera view
                if blue_pos_smoothed is not None:
                    cv2.circle(frame_marked, blue_pos_smoothed, 10, (255, 0, 0), 2)
                    cv2.circle(frame_marked, blue_pos_smoothed, 3, (255, 0, 0), -1)

                    if self.transform_matrix is not None:
                        draw_pos = self.transform_point(blue_pos_smoothed)
                        draw_pos = (
                            max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                            max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                        )
                        if drawing_enabled and line_thickness > 0:
                            if self.last_draw_point is not None:
                                cv2.line(self.canvas, self.last_draw_point, draw_pos,
                                         (255, 255, 255), line_thickness)
                            self.last_draw_point = draw_pos
                        else:
                            self.last_draw_point = None
                else:
                    self.last_draw_point = None
                    self.position_history.clear()

                # Status overlays
                if self.is_calibrated:
                    calibrated_text = "CALIBRATED - BLACK CANVAS MODE"
                    calibrated_color = (0, 255, 0)
                else:
                    calibrated_text = "NOT CALIBRATED (press 'c' to calibrate)"
                    calibrated_color = (0, 0, 255)
                cv2.putText(frame_marked, calibrated_text, (30, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, calibrated_color, 2)

                ble_status = "CONNECTED" if self.ble_connected else "DISCONNECTED"
                ble_color = (0, 255, 0) if self.ble_connected else (0, 0, 255)
                cv2.putText(frame_marked, f"BLE: {ble_status}", (30, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, ble_color, 2)

                force_newtons = self.to_newtons(self.force_value)
                threshold_n = self.to_newtons(self.force_threshold)
                force_text = f"Force: {self.force_value} ({force_newtons:.3f}N) | Threshold: {self.force_threshold} ({threshold_n:.3f}N)"
                cv2.putText(frame_marked, force_text, (30, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

                drawing_text = f"DRAWING: {'ON' if drawing_enabled else 'OFF'} (Thickness: {line_thickness}px)"
                drawing_color = (0, 255, 255) if drawing_enabled else (128, 128, 128)
                cv2.putText(frame_marked, drawing_text, (30, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, drawing_color, 2)

                # Compose canvas display with cursor
                canvas_display = self.canvas.copy()
                if blue_pos_smoothed is not None and self.transform_matrix is not None:
                    draw_pos = self.transform_point(blue_pos_smoothed)
                    draw_pos = (
                        max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                        max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                    )
                    cursor_radius = max(5, line_thickness + 2)
                    cursor_color = (255, 255, 255) if self.is_calibrated else (0, 165, 255)
                    if not drawing_enabled:
                        cursor_color = (128, 128, 128)
                    cv2.circle(canvas_display, draw_pos, cursor_radius, cursor_color, 2)
                    cv2.circle(canvas_display, draw_pos, 3, cursor_color, -1)

                last_frame_marked = frame_marked
                last_canvas_display = canvas_display

                # Handle calibration trigger (only after processing step)
                if self.awaiting_calibration and fiducials is not None:
                    self.setup_transform(fiducials)

            # --- UI: only refresh at ui_fps ---
            if last_frame_marked is not None and (now_ms - self.last_ui_ms >= self.ui_period_ms):
                self.last_ui_ms = now_ms
                cv2.imshow("Phone Camera View", last_frame_marked)
                if last_canvas_display is not None:
                    cv2.imshow("Drawing Canvas (Project This)", last_canvas_display)

            key = cv2.waitKey(1) & 0xFF  # small wait; we're pacing via fps gates
            if key == ord('q'):
                break
            elif key == ord('c'):
                # Enter calibration mode: show markers and start detecting
                self.awaiting_calibration = True
                self.is_calibrated = False
                self.canvas = self._make_fiducial_canvas()
                print("📐 Calibration mode: show all 4 ArUco markers, then press 'c' again if needed.")
            elif key == ord('r'):
                if self.is_calibrated:
                    self.canvas = self._make_clean_canvas()
                    print("🎨 Drawing reset (clean black canvas)")
                else:
                    self.canvas = self._make_fiducial_canvas()
                    print("🎨 Canvas reset (showing calibration markers)")
                self.last_draw_point = None
            elif key == ord('+') or key == ord('='):
                self.force_threshold += 25
                threshold_n = self.to_newtons(self.force_threshold)
                print(f"⬆️  Force threshold: {self.force_threshold} ({threshold_n:.3f}N)")
            elif key == ord('-') or key == ord('_'):
                self.force_threshold = max(0, self.force_threshold - 25)
                threshold_n = self.to_newtons(self.force_threshold)
                print(f"⬇️  Force threshold: {self.force_threshold} ({threshold_n:.3f}N)")

        # Cleanup
        self.running = False
        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = GreenPixelTracker()
    tracker.run()
