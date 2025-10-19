import cv2
import numpy as np
from collections import deque
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import websockets
import base64
from io import BytesIO

# BLE UUIDs - MUST match the ESP32 code
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"


class WebSocketVideoCapture:
    def __init__(self, port=8765):
        self.frame = None
        self.frame_lock = threading.Lock()
        self.running = True
        self.port = port
        
        # Start websocket server in background
        self.ws_thread = threading.Thread(target=self.run_websocket_server, daemon=True)
        self.ws_thread.start()
        
    def run_websocket_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def handle_client(websocket, path):
            print(f"📱 Browser connected from {websocket.remote_address}")
            try:
                async for message in websocket:
                    # Decode JPEG bytes to numpy array
                    nparr = np.frombuffer(message, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        with self.frame_lock:
                            self.frame = frame
            except Exception as e:
                print(f"⚠️ WebSocket error: {e}")
        
        start_server = websockets.serve(handle_client, "localhost", self.port)
        print(f"🌐 WebSocket server started on ws://localhost:{self.port}")
        print(f"   Open video_stream.html in your browser")
        
        loop.run_until_complete(start_server)
        loop.run_forever()
    
    def read(self):
        with self.frame_lock:
            if self.frame is None:
                return False, None
            return True, self.frame.copy()
    
    def release(self):
        self.running = False
    
    def isOpened(self):
        return self.running
    
    def set(self, prop, value):
        pass  # Ignore resolution settings
    
    def get(self, prop):
        with self.frame_lock:
            if self.frame is None:
                return 0
            if prop == cv2.CAP_PROP_FRAME_WIDTH:
                return self.frame.shape[1]
            elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
                return self.frame.shape[0]
        return 0

class GreenPixelTracker:
    def __init__(self):
        print("\n" + "="*60)
        print("🎨 PRESSURE-SENSITIVE DRAWING SYSTEM")
        print("="*60)
        self.camera = WebSocketVideoCapture(port=8765)
        
        # --- Camera Setup ---
        USE_WEBSOCKET = True  # Toggle this to switch modes
        
        if USE_WEBSOCKET:
            print(f"\n📷 Initializing WebSocket video stream...")
            self.camera = WebSocketVideoCapture(port=8765)
            print(f"   ✅ WebSocket server ready")
            print(f"   Open video_stream.html in your browser to start streaming")
        else:
            # Original camera code
            camera_index = 1
            print(f"\n📷 Initializing camera...")
            print(f"   Trying camera index {camera_index} (iPhone/External)...")
            
            self.camera = cv2.VideoCapture(camera_index)
            if not self.camera.isOpened():
                print(f"   ⚠️  Camera {camera_index} not available")
                print(f"   Trying default webcam (index 0)...")
                self.camera = cv2.VideoCapture(0)

            if not self.camera.isOpened():
                print("   ❌ No camera available!")
                exit(1)
        
        # Set resolution (works for both modes)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_width > 0:
            print(f"   ✅ Video ready: {actual_width}x{actual_height}")
        # --- ArUco Setup ---
        print("\n🎯 Setting up ArUco marker detection...")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.transform_matrix = None
        self.marker_size = 200
        self.margin = 40
        print("   ✅ ArUco detector initialized")
        print("   Looking for markers: 0, 1, 2, 3")

        # --- Canvas Setup ---
        print("\n🖼️  Creating canvas...")
        self.canvas_size = (720, 1280)
        self.canvas = self._make_fiducial_canvas()
        self.calibration_canvas = self.canvas.copy()  # Keep calibration version
        self.is_calibrated = False
        self.last_draw_point = None
        print(f"   ✅ Canvas created: {self.canvas_size[1]}x{self.canvas_size[0]}")
        print("   ArUco markers placed at corners")

        # --- Smoothing ---
        self.position_history = deque(maxlen=5)
        
        # --- Color Detection (Blue LED) ---
        print("\n🔵 Configuring blue LED tracking...")
        # Blue LED HSV range - adjust these if needed
        self.blue_lower = np.array([100, 150, 50])   # Lower bound: Hue, Saturation, Value
        self.blue_upper = np.array([130, 255, 255])  # Upper bound
        self.min_area = 50  # Smaller area for LED (LEDs are point sources)
        print("   ✅ HSV range configured for blue LED")
        print(f"   Hue: 100-130, Saturation: 150-255, Value: 50-255")
        print(f"   Minimum detection area: {self.min_area} pixels")

        # --- Force Sensor Setup (SingleTact 1N - Correct Protocol) ---
        print("\n⚡ Force sensor configuration...")
        self.force_value = 0
        self.force_threshold = 614  # 0.6 N threshold (614/1024 * 1.0 = 0.6N)
        self.min_thickness = 1
        self.max_thickness = 15
        self.force_max = 1024  # SingleTact 1N raw range (0-1024)
        self.sensor_max_newtons = 1.0  # 1 Newton max
        print(f"   Initial threshold: {self.force_threshold} ADC (~{self.to_newtons(self.force_threshold):.3f}N)")
        print(f"   Line thickness range: {self.min_thickness}-{self.max_thickness} pixels")
        print(f"   Max sensor value: {self.force_max} (~{self.sensor_max_newtons}N)")
        
        # BLE connection
        self.ble_client = None
        self.ble_connected = False
        self.running = True
        
        # Start BLE in background thread
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
        """Run BLE operations in separate thread with its own event loop"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect_ble())
        loop.close()

    async def connect_ble(self):
        """Find and connect to the ESP32 via BLE with auto-reconnect"""
        
        while self.running:  # Auto-reconnect loop
            print("\n" + "="*60)
            print("🔍 SCANNING FOR BLUETOOTH DEVICES")
            print("="*60)
            print("   Looking for: 'ForceStylus'")
            print("   Timeout: 10 seconds")
            print("   Make sure your ESP32 is powered on!\n")
            
            try:
                # Scan for devices
                devices = await BleakScanner.discover(timeout=10.0)
                
                print(f"📱 Found {len(devices)} Bluetooth device(s):\n")
                
                esp32_device = None
                for i, device in enumerate(devices, 1):
                    device_name = device.name or "Unknown"
                    
                    # Highlight our device
                    if "Force" in device_name or device_name == "ForceStylus":
                        print(f"   {i}. ✅ {device_name:25s} ({device.address})")
                        print(f"      ↑↑↑ THIS IS YOUR FORCE STYLUS! ↑↑↑")
                        # Try to get RSSI if available
                        try:
                            print(f"      Signal: {device.rssi} dBm")
                        except AttributeError:
                            pass  # RSSI not available on this platform
                        esp32_device = device
                    else:
                        print(f"   {i}. {device_name:25s} ({device.address})")
                
                print()
                
                if not esp32_device:
                    print("❌ ForceStylus not found!")
                    print("\n   ⏳ Will retry in 5 seconds...\n")
                    await asyncio.sleep(5)
                    continue  # Try scanning again
                
                print("─"*60)
                print(f"✅ Found ForceStylus!")
                print(f"   Address: {esp32_device.address}")
                print("─"*60)
                print("\n🔗 Connecting to device...")
                
                # Connect to the device
                async with BleakClient(esp32_device.address) as client:
                    self.ble_client = client
                    self.ble_connected = True
                    
                    print("✅ Connected successfully!")
                    
                    # Find our service
                    service_found = False
                    for service in client.services:
                        if service.uuid.lower() == SERVICE_UUID.lower():
                            service_found = True
                            print(f"   ✓ Force sensor service found")
                            break
                    
                    if not service_found:
                        print("   ⚠️  Expected service not found")
                    
                    print("\n📡 Subscribing to force sensor notifications...")
                    
                    # Subscribe to notifications
                    await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                    
                    print("✅ Receiving force data from ESP32!")
                    print("─"*60)
                    print("\n💡 Press the sensor to see values update")
                    print("   Drawing will start when force > threshold\n")
                    
                    # Keep connection alive and show live data
                    last_value_print = 0
                    try:
                        while self.running and client.is_connected:
                            # Print force value when it changes
                            if self.force_value != last_value_print:
                                if self.force_value > 20 or last_value_print > 20:
                                    newtons = self.to_newtons(self.force_value)
                                    bar_length = min(40, (self.force_value * 40) // self.force_max)
                                    bar = "█" * bar_length
                                    
                                    # Show if drawing is active
                                    status = "DRAWING" if self.is_drawing_enabled() else "standby"
                                    thickness = self.get_line_thickness()
                                    
                                    print(f"   Force: {self.force_value:4d} ({newtons:.3f}N) │{bar:40s}│ {status} (thick={thickness})", end='\r')
                                    last_value_print = self.force_value
                            
                            await asyncio.sleep(0.05)
                    
                    except Exception as e:
                        print(f"\n⚠️  Connection lost: {e}")
                    
                    # Graceful cleanup
                    print("\n\n🔌 Cleaning up connection...")
                    try:
                        if client.is_connected:
                            await client.stop_notify(CHARACTERISTIC_UUID)
                    except Exception:
                        pass  # Ignore cleanup errors
                        
            except Exception as e:
                print(f"\n❌ BLE Error: {e}")
                print("   Connection lost or failed")
            
            finally:
                self.ble_connected = False
                print("\n✋ BLE connection closed")
                
                if self.running:
                    print("\n🔄 Attempting to reconnect in 3 seconds...")
                    await asyncio.sleep(3)
                    # Loop will retry connection
                else:
                    break  # User quit, exit loop

    def notification_handler(self, sender, data):
        """Called whenever ESP32 sends new force data"""
        # Convert 2-byte array back to integer
        # data[0] = low byte, data[1] = high byte
        if len(data) == 2:
            self.force_value = data[0] | (data[1] << 8)

    # ----------------------------------------------------------------
    # Force Sensor Logic
    # ----------------------------------------------------------------
    
    def get_line_thickness(self):
        """Calculate line thickness based on force"""
        if self.force_value < self.force_threshold:
            return 0  # No drawing below threshold
        
        force_range = self.force_max - self.force_threshold
        thickness_range = self.max_thickness - self.min_thickness
        
        force_above_threshold = self.force_value - self.force_threshold
        thickness = self.min_thickness + (force_above_threshold / force_range) * thickness_range
        
        return int(max(self.min_thickness, min(self.max_thickness, thickness)))

    def is_drawing_enabled(self):
        """Check if force is above threshold"""
        return self.force_value >= self.force_threshold
    
    def to_newtons(self, adc_value):
        """Convert ADC value to Newtons"""
        return (adc_value / self.force_max) * self.sensor_max_newtons

    # ----------------------------------------------------------------
    # Canvas and Vision Functions
    # ----------------------------------------------------------------
    
    def _make_fiducial_canvas(self):
        """Create canvas with ArUco markers for calibration"""
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
        """Create clean black canvas for drawing"""
        return np.zeros((self.canvas_size[0], self.canvas_size[1], 3), dtype=np.uint8)

    def detect_fiducials(self, frame):
        """Detect ArUco markers"""
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
        """Setup perspective transform and switch to clean canvas"""
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
        
        # Switch to clean black canvas after calibration
        self.canvas = self._make_clean_canvas()
        
        print("\n" + "="*60)
        print("✅ CALIBRATION SUCCESSFUL!")
        print("="*60)
        print("   Camera view is now mapped to canvas")
        print("   Switched to clean black canvas")
        print("   You can now draw with white lines!")
        print("="*60 + "\n")

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
        
        if len(self.position_history) > 0:
            avg_x = int(np.mean([p[0] for p in self.position_history]))
            avg_y = int(np.mean([p[1] for p in self.position_history]))
            return (avg_x, avg_y)
        
        return position

    def detect_blue(self, frame):
        """Detect blue LED"""
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        # Less aggressive morphology for LED (point source)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.position_history.clear()
            return None, mask

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            self.position_history.clear()
            return None, mask

        # Use bounding box center for LED position
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        cy = y + h // 2
        
        return (cx, cy), mask

    # ----------------------------------------------------------------
    # Main Loop
    # ----------------------------------------------------------------
    
    def run(self):
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("⚠️ Camera feed lost")
                break

            # Detect fiducials and blue LED
            fiducials, frame_marked = self.detect_fiducials(frame)
            blue_pos, mask = self.detect_blue(frame)
            blue_pos_smoothed = self.smooth_position(blue_pos)

            # Get drawing parameters from force sensor
            drawing_enabled = self.is_drawing_enabled()
            line_thickness = self.get_line_thickness()

            # Draw blue LED detection
            if blue_pos_smoothed is not None:
                # Show blue circle on camera view
                cv2.circle(frame_marked, blue_pos_smoothed, 10, (255, 0, 0), 2)  # Blue circle
                cv2.circle(frame_marked, blue_pos_smoothed, 3, (255, 0, 0), -1)
                
                if self.transform_matrix is not None:
                    draw_pos = self.transform_point(blue_pos_smoothed)
                    draw_pos = (
                        max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                        max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                    )
                    
                    # DRAW ONLY IF FORCE IS ABOVE THRESHOLD
                    if drawing_enabled and line_thickness > 0:
                        if self.last_draw_point is not None:
                            # Draw line in WHITE with variable thickness based on pressure
                            cv2.line(self.canvas, self.last_draw_point, draw_pos, 
                                   (255, 255, 255), line_thickness)
                        self.last_draw_point = draw_pos
                    else:
                        # Not pressing hard enough - don't draw
                        self.last_draw_point = None
            else:
                self.last_draw_point = None
                self.position_history.clear()

            # Status display
            if self.is_calibrated:
                calibrated_text = "CALIBRATED - BLACK CANVAS MODE"
                calibrated_color = (0, 255, 0)
            else:
                calibrated_text = "NOT CALIBRATED (press 'c' to calibrate)"
                calibrated_color = (0, 0, 255)
            
            cv2.putText(frame_marked, calibrated_text, (30, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, calibrated_color, 2)
            
            # BLE connection status
            ble_status = "CONNECTED" if self.ble_connected else "DISCONNECTED"
            ble_color = (0, 255, 0) if self.ble_connected else (0, 0, 255)
            cv2.putText(frame_marked, f"BLE: {ble_status}", (30, 160),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, ble_color, 2)
            
            # Force sensor status with Newtons
            force_newtons = self.to_newtons(self.force_value)
            threshold_n = self.to_newtons(self.force_threshold)
            force_text = f"Force: {self.force_value} ({force_newtons:.3f}N) | Threshold: {self.force_threshold} ({threshold_n:.3f}N)"
            cv2.putText(frame_marked, force_text, (30, 200),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            
            drawing_text = f"DRAWING: {'ON' if drawing_enabled else 'OFF'} (Thickness: {line_thickness}px)"
            drawing_color = (0, 255, 255) if drawing_enabled else (128, 128, 128)
            cv2.putText(frame_marked, drawing_text, (30, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, drawing_color, 2)

            # Canvas display with cursor
            canvas_display = self.canvas.copy()

            if blue_pos_smoothed is not None and self.transform_matrix is not None:
                draw_pos = self.transform_point(blue_pos_smoothed)
                draw_pos = (
                    max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                    max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                )
                
                # Cursor size reflects line thickness
                cursor_radius = max(5, line_thickness + 2)
                # White cursor on black background when calibrated, colored on gray when not
                if self.is_calibrated:
                    cursor_color = (255, 255, 255) if drawing_enabled else (128, 128, 128)
                else:
                    cursor_color = (0, 0, 255) if drawing_enabled else (0, 165, 255)
                cv2.circle(canvas_display, draw_pos, cursor_radius, cursor_color, 2)
                cv2.circle(canvas_display, draw_pos, 3, cursor_color, -1)

            cv2.imshow("Phone Camera View", frame_marked)
            cv2.imshow("Drawing Canvas (Project This)", canvas_display)

            # Keyboard controls
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                if fiducials is not None:
                    self.setup_transform(fiducials)
                else:
                    print("❌ Cannot calibrate - markers not visible")
                    print("   → Make sure all 4 ArUco markers are visible in camera")
                    if self.is_calibrated:
                        print("   💡 To recalibrate: press 'r' to show markers, then 'c'")
            elif key == ord('r'):
                # Reset canvas based on calibration state
                if self.is_calibrated:
                    # Stay in drawing mode with clean black canvas
                    self.canvas = self._make_clean_canvas()
                    print("🎨 Drawing reset (clean black canvas)")
                else:
                    # Reset to calibration mode with markers
                    self.canvas = self._make_fiducial_canvas()
                    print("🎨 Canvas reset (showing calibration markers)")
                self.last_draw_point = None
            elif key == ord('+') or key == ord('='):
                self.force_threshold += 25
                threshold_n = self.to_newtons(self.force_threshold)
                print(f"⬆️  Force threshold: {self.force_threshold} ({threshold_n:.3f}N) - harder press needed")
            elif key == ord('-') or key == ord('_'):
                self.force_threshold = max(0, self.force_threshold - 25)
                threshold_n = self.to_newtons(self.force_threshold)
                print(f"⬇️  Force threshold: {self.force_threshold} ({threshold_n:.3f}N) - easier to draw")

        # Cleanup
        self.running = False
        self.camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tracker = GreenPixelTracker()
    tracker.run()