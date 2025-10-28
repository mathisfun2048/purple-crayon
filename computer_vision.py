import cv2
import numpy as np
from collections import deque
import asyncio
from bleak import BleakClient, BleakScanner
import threading
import time
import os  # NEW: For file path validation

# BLE UUIDs - MUST match the ESP32 code
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class GreenPixelTracker:
    def __init__(self):
        print("\n" + "="*60)
        print("🎨 PRESSURE-SENSITIVE DRAWING SYSTEM (FAST)")
        print("="*60)

        # ---------- Performance knobs (PC-side) ----------
        self.proc_fps = 15            # Vision processing rate (Hz)  ← lower for less CPU
        self.ui_fps   = 24            # UI refresh rate (Hz)
        self.proc_period_ms = int(1000 / self.proc_fps)
        self.ui_period_ms   = int(1000 / self.ui_fps)
        self.last_proc_ms = 0
        self.last_ui_ms   = 0

        # LED ROI search window (pixels). Small = faster. Expands if target lost.
        self.roi_half_base = 60       # base half-size of ROI box
        self.roi_half      = self.roi_half_base
        self.roi_downscale = 1.0      # 0.5 to speed up more (coords are scaled back)
        self.max_fullframe_checks = 10  # INCREASED: full-frame searches per second when lost

        # Top-K & bright-core gating to limit pixels used for centroid
        self.TOP_K      = 200         # cap pixels used for centroid (fast & stable)
        self.V_BRIGHT   = 150         # INCREASED: V-channel threshold to reduce false positives (was 100)
        self.ERODE_ON   = True        # one erode to shrink bloom
        self.ERODE_K    = np.ones((3,3), np.uint8)
        self.MIN_DETECTION_AREA = 15  # NEW: Minimum pixel count to consider a detection valid

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
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # Drop buffering so we don't build a backlog
            try:
                self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            try:
                cv2.setUseOptimized(True)
                cv2.setNumThreads(2)   # tune 1–4 depending on CPU
            except Exception:
                pass
            actual_width  = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"   ✅ Camera ready: {actual_width}x{actual_height} (performance mode)")
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

        # ---------- Undo History ----------
        self.canvas_history = deque(maxlen=50)  # Store last 50 states
        self.save_canvas_state()  # Save initial state

        # ---------- Smoothing ----------
        self.position_history = deque(maxlen=5)
        self.last_green_pos_raw = None  # for ROI seeding
        self.last_fullframe_check = 0.0

        # ---------- Color Detection (Green LED) ----------
        print("\n🟢 Configuring GREEN LED tracking (HSV)…")
        self.green_lower = np.array([40, 50, 50])  # H,S,V - Wider green range
        self.green_upper = np.array([80, 255, 255])
        self.min_area = 20  # LEDs are point sources; keep small
        print("   ✅ HSV range configured for green LED")
        print("   ✅ Using H=40-80 (proper green range)")

        # ---------- Drawing Modes ----------
        self.BRUSH_MODE = 0
        self.ERASER_MODE = 1
        self.drawing_mode = self.BRUSH_MODE

        # ========== NEW: COLOR PALETTE ==========
        # Define 5 colors that users can select with keys 1-5
        # Each color is in BGR format (Blue, Green, Red) - that's how OpenCV stores colors
        self.color_palette = {
            1: (255, 255, 255),  # White (B=255, G=255, R=255)
            2: (0, 0, 255),      # Red (B=0, G=0, R=255)
            3: (0, 255, 0),      # Green (B=0, G=255, R=0)
            4: (255, 0, 0),      # Blue (B=255, G=0, R=0)
            5: (0, 255, 255),    # Yellow (B=0, G=255, R=255)
        }
        # Color names for display (matching the keys above)
        self.color_names = {
            1: "White",
            2: "Red", 
            3: "Green",
            4: "Blue",
            5: "Yellow"
        }
        # Start with white color (key 1)
        self.current_color = self.color_palette[1]
        self.current_color_key = 1
        print("\n🎨 Color palette initialized:")
        print("   [1] White  [2] Red  [3] Green  [4] Blue  [5] Yellow")

        # ========== NEW: IMAGE PASTING FEATURE ==========
        # This will store the image that the user wants to paste
        self.paste_image = None
        # This controls the size of the pasted image (you can adjust this)
        self.paste_image_size = 150  # pixels width/height (default)
        self.paste_size_min = 50      # Minimum paste size
        self.paste_size_max = 500     # Maximum paste size
        self.paste_size_step = 25     # How much to change size each time
        print("\n🖼️  Image pasting feature ready")
        print("   Press [I] to enter image file path, then [P] to paste at cursor")
        print(f"   Press [ ] to decrease/increase paste size (current: {self.paste_image_size}px)")

        # ---------- Force Sensor Setup ----------
        print("\n⚡ Force sensor configuration...")
        self.force_value = 0
        
        # AUTO-TUNING THRESHOLD SYSTEM
        self.force_history = deque(maxlen=150)  # Track last ~10 seconds at 15fps
        self.baseline_percentile = 30  # Use 30th percentile as baseline (robust to peaks)
        self.sensitivity_margin = 80  # How much above baseline = drawing (adjustable with +/-)
        self.force_threshold = 280  # Initial value, will auto-adjust
        self.auto_tune_enabled = True  # Toggle with 'a' key
        
        self.min_thickness = 1
        self.max_thickness = 30  # DOUBLED from 15 to 30 for more sensitivity
        self.eraser_multiplier = 2.5  # Eraser is 2.5x larger than brush
        self.force_max = 1024
        self.sensor_max_newtons = 1.0
        print(f"   🤖 AUTO-TUNING ENABLED: Threshold adapts to sensor drift")
        print(f"   Initial sensitivity margin: {self.sensitivity_margin} ADC above baseline")
        print(f"   Eraser size: {self.eraser_multiplier}x brush size")

        # ---------- BLE ----------
        self.ble_client = None
        self.ble_connected = False
        self.running = True
        self.last_data_received = 0
        self.device_address = None   # cache address to skip scans

        print("\n🚀 Starting BLE connection thread…")
        self.ble_thread = threading.Thread(target=self.run_ble_loop, daemon=True)
        self.ble_thread.start()

        print("\n" + "="*60)
        print("✅ SYSTEM READY")
        print("="*60)
        print("⌨️  Calibration: [C]alibrate")
        print("⌨️  Drawing: [B]rush | [E]raser | [X]Clear | [U]ndo")
        print("⌨️  Colors: [1]White [2]Red [3]Green [4]Blue [5]Yellow")
        print("⌨️  Images: [I]Enter image path | [P]aste at cursor | [ ]Size-/+")
        print("⌨️  Settings: [+/-]Sensitivity | [A]uto-tune | [R]eset")
        print("⌨️  Debug: [M]ask | [T]HSV tuner | [,/.]Brightness | [Q]uit")
        print("="*60 + "\n")

    # ----------------------------------------------------------------
    # NEW: Image Loading and Pasting Functions
    # ----------------------------------------------------------------
    def change_paste_size(self, increase=True):
        """
        Change the size of pasted images.
        If an image is already loaded, it will be re-resized.
        
        increase: True to increase size, False to decrease
        """
        old_size = self.paste_image_size
        
        if increase:
            # Increase size (up to maximum)
            self.paste_image_size = min(self.paste_size_max, 
                                       self.paste_image_size + self.paste_size_step)
        else:
            # Decrease size (down to minimum)
            self.paste_image_size = max(self.paste_size_min, 
                                       self.paste_image_size - self.paste_size_step)
        
        # Check if size actually changed
        if old_size == self.paste_image_size:
            if increase:
                print(f"⚠️  Maximum paste size reached: {self.paste_size_max}px")
            else:
                print(f"⚠️  Minimum paste size reached: {self.paste_size_min}px")
            return
        
        # Print new size
        direction = "increased" if increase else "decreased"
        print(f"📏 Paste size {direction}: {self.paste_image_size}px (was {old_size}px)")
        
        # If an image is already loaded, re-resize it to the new size
        if self.paste_image is not None:
            # If we have the original image, resize from that (better quality)
            if hasattr(self, 'original_paste_image'):
                self.paste_image = cv2.resize(self.original_paste_image, 
                                             (self.paste_image_size, self.paste_image_size))
                print(f"   ✅ Loaded image resized to {self.paste_image_size}x{self.paste_image_size}")
            else:
                # No original, resize from current (may lose quality)
                self.paste_image = cv2.resize(self.paste_image, 
                                             (self.paste_image_size, self.paste_image_size))
                print(f"   ⚠️  Image resized (may have quality loss)")
                print("   Tip: Reload with [I] for best quality at this size")
    
    def load_image(self):
        """
        Prompts user to enter an image file path via terminal.
        The image is loaded and stored in self.paste_image.
        """
        print("\n" + "="*60)
        print("📂 LOAD IMAGE")
        print("="*60)
        print("Enter the full path to your image file.")
        print("TIP: You can drag and drop the file into the terminal!")
        print("Example: /Users/yourname/Pictures/image.png")
        print("Press Enter without typing anything to cancel.")
        print("="*60)
        
        # Get file path from user
        file_path = input("Image path: ").strip()
        
        # Remove quotes if user copy-pasted with quotes
        file_path = file_path.strip('"').strip("'")
        
        # Check if user cancelled
        if not file_path:
            print("⚠️  Cancelled - no file path entered")
            return
        
        # Check if file exists
        if not os.path.exists(file_path):
            print(f"❌ File not found: {file_path}")
            print("   Make sure the path is correct and try again")
            return
        
        # Try to load the image using OpenCV
        img = cv2.imread(file_path)
        
        if img is not None:
            # Image loaded successfully!
            # Store the original image so we can re-resize it later without quality loss
            self.original_paste_image = img.copy()
            
            # Resize it to our standard paste size for consistency
            # cv2.resize changes the image dimensions
            self.paste_image = cv2.resize(img, (self.paste_image_size, self.paste_image_size))
            print(f"✅ Image loaded successfully!")
            print(f"   Resized to {self.paste_image_size}x{self.paste_image_size} pixels")
            print("   Press [P] when cursor is where you want to paste")
            print("   Press [ or ] to change paste size")
        else:
            print(f"❌ Failed to load image: {file_path}")
            print("   Make sure it's a valid image file (PNG, JPG, etc.)")
        
        print("="*60 + "\n")
    
    def paste_image_at_cursor(self, cursor_pos):
        """
        Pastes the loaded image onto the canvas at the cursor position.
        The image is centered on the cursor.
        
        cursor_pos: (x, y) tuple of where to paste the image
        """
        # Check if we have an image to paste
        if self.paste_image is None:
            print("⚠️  No image loaded! Press [I] to load an image first")
            return
        
        # Check if we're calibrated (we need the transform to work)
        if not self.is_calibrated:
            print("⚠️  Please calibrate first (press [C])")
            return
        
        # Save canvas state for undo functionality
        self.save_canvas_state()
        
        # Get the image dimensions
        img_h, img_w = self.paste_image.shape[:2]
        
        # Calculate top-left corner position to center the image on cursor
        # We subtract half the width and height to center it
        x = cursor_pos[0] - img_w // 2  # // means integer division
        y = cursor_pos[1] - img_h // 2
        
        # Calculate bottom-right corner
        x_end = x + img_w
        y_end = y + img_h
        
        # Make sure we don't paste outside the canvas bounds
        # We need to clip the coordinates to stay within canvas
        
        # Clipping for canvas boundaries
        canvas_h, canvas_w = self.canvas.shape[:2]
        
        # Calculate how much of the image fits in the canvas
        # If image goes off-screen, we only paste the visible part
        src_x_start = max(0, -x)  # If x is negative, start from this pixel in source image
        src_y_start = max(0, -y)  # If y is negative, start from this pixel in source image
        src_x_end = img_w - max(0, x_end - canvas_w)  # If goes past right edge
        src_y_end = img_h - max(0, y_end - canvas_h)  # If goes past bottom edge
        
        # Adjust paste position to canvas bounds
        dst_x_start = max(0, x)
        dst_y_start = max(0, y)
        dst_x_end = min(canvas_w, x_end)
        dst_y_end = min(canvas_h, y_end)
        
        # Extract the portion of image that fits
        img_portion = self.paste_image[src_y_start:src_y_end, src_x_start:src_x_end]
        
        # Paste the image onto the canvas
        # We directly copy the pixels from the image to the canvas
        try:
            self.canvas[dst_y_start:dst_y_end, dst_x_start:dst_x_end] = img_portion
            print(f"✅ Image pasted at ({cursor_pos[0]}, {cursor_pos[1]})")
        except Exception as e:
            print(f"❌ Error pasting image: {e}")

    # ----------------------------------------------------------------
    # HSV Tuner for finding exact color values
    # ----------------------------------------------------------------
    def test_color_range(self):
        """Interactive HSV tuner - adjust sliders to find your LED's exact color"""
        print("\n" + "="*60)
        print("🎨 HSV COLOR TUNER")
        print("="*60)
        print("Adjust sliders until your LED appears WHITE in the mask window")
        print("Press 'p' to print current values")
        print("Press 'q' to quit tuner")
        print("="*60 + "\n")
        
        cv2.namedWindow("HSV Tuner - Mask")
        cv2.namedWindow("HSV Tuner - Original")
        
        # Create sliders with current values as defaults
        cv2.createTrackbar("H Low", "HSV Tuner - Mask", 40, 179, lambda x: None)
        cv2.createTrackbar("H High", "HSV Tuner - Mask", 80, 179, lambda x: None)
        cv2.createTrackbar("S Low", "HSV Tuner - Mask", 50, 255, lambda x: None)
        cv2.createTrackbar("S High", "HSV Tuner - Mask", 255, 255, lambda x: None)
        cv2.createTrackbar("V Low", "HSV Tuner - Mask", 50, 255, lambda x: None)
        cv2.createTrackbar("V High", "HSV Tuner - Mask", 255, 255, lambda x: None)
        
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break
            
            # Get slider values
            h_low = cv2.getTrackbarPos("H Low", "HSV Tuner - Mask")
            h_high = cv2.getTrackbarPos("H High", "HSV Tuner - Mask")
            s_low = cv2.getTrackbarPos("S Low", "HSV Tuner - Mask")
            s_high = cv2.getTrackbarPos("S High", "HSV Tuner - Mask")
            v_low = cv2.getTrackbarPos("V Low", "HSV Tuner - Mask")
            v_high = cv2.getTrackbarPos("V High", "HSV Tuner - Mask")
            
            # Create mask with current settings
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower = np.array([h_low, s_low, v_low])
            upper = np.array([h_high, s_high, v_high])
            mask = cv2.inRange(hsv, lower, upper)
            
            # Add text overlay with current values
            mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_display, f"H: {h_low}-{h_high}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(mask_display, f"S: {s_low}-{s_high}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(mask_display, f"V: {v_low}-{v_high}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(mask_display, "Press 'p' to print values", (10, 450),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Show windows
            cv2.imshow("HSV Tuner - Mask", mask_display)
            cv2.imshow("HSV Tuner - Original", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('p'):
                print("\n" + "="*60)
                print("📋 CURRENT HSV VALUES:")
                print("="*60)
                print(f"self.green_lower = np.array([{h_low}, {s_low}, {v_low}])")
                print(f"self.green_upper = np.array([{h_high}, {s_high}, {v_high}])")
                print("="*60 + "\n")
            elif key == ord('q'):
                break
        
        cv2.destroyWindow("HSV Tuner - Mask")
        cv2.destroyWindow("HSV Tuner - Original")
        print("✅ HSV Tuner closed\n")

    # ----------------------------------------------------------------
    # BLE Functions
    # ----------------------------------------------------------------
    def run_ble_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.connect_ble())
        loop.close()

    async def fast_find_device(self, name_substring="ForceStylus", timeout=0.25):
        """Quickly find by name; returns device or None."""
        try:
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name and name_substring in d.name), timeout=timeout
            )
            return device
        except Exception:
            devices = await BleakScanner.discover(timeout=timeout)
            for d in devices:
                if d.name and name_substring in d.name:
                    return d
            return None

    def _on_disconnect(self, client):
        print("\n🔌 BLE disconnected (callback).")
        self.ble_connected = False

    async def connect_ble(self):
        """Find and connect to the ESP32 via BLE with very fast auto-reconnect"""
        backoff = 0.2  # quick retry

        while self.running:
            print("\n" + "="*60)
            print("🔍 SCANNING/CONNECTING BLUETOOTH (FAST)")
            print("="*60)

            try:
                device = None
                if self.device_address:
                    print(f"   Using cached address: {self.device_address}")
                    device = type("Tmp", (), {"address": self.device_address})()
                else:
                    print("   Fast find 'ForceStylus'… (≤0.25s)")
                    device = await self.fast_find_device("ForceStylus", timeout=0.25)

                if not device:
                    print("   ❌ Not found. Quick retry…")
                    await asyncio.sleep(backoff)
                    continue

                address = device.address if hasattr(device, "address") else device
                print(f"\n🔗 Connecting to {address} …")
                async with BleakClient(address, disconnected_callback=self._on_disconnect, timeout=3.0) as client:
                    self.ble_client = client
                    self.ble_connected = True
                    if not self.device_address:
                        self.device_address = address  # cache for next time

                    print("✅ Connected! Subscribing to force data…")
                    await client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                    print("✅ Receiving force data from ESP32!")

                    last_data_time = time.monotonic()
                    while self.running and client.is_connected:
                        if (time.monotonic() - last_data_time) > 2.0:
                            print("\n⚠️  No data in 2s (still connected)…")
                            last_data_time = time.monotonic()
                        await asyncio.sleep(0.02)  # cooperative yield

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
        """ESP32 sends 2 bytes: LSB, MSB"""
        if len(data) == 2:
            self.force_value = data[0] | (data[1] << 8)
            self.last_data_received = time.time()
            
            # Record force value for auto-tuning
            self.force_history.append(self.force_value)

    # ----------------------------------------------------------------
    # Force Sensor Logic (with AUTO-TUNING)
    # ----------------------------------------------------------------
    def update_adaptive_threshold(self):
        """
        AUTO-TUNING: Calculates threshold based on recent force history.
        Uses baseline (low percentile) + sensitivity margin to detect peaks.
        This adapts to sensor drift automatically!
        """
        if not self.auto_tune_enabled:
            return  # Manual mode: use fixed threshold
        
        # Need enough data to calculate baseline
        if len(self.force_history) < 30:
            return  # Keep initial threshold until we have history
        
        # Calculate baseline: use 30th percentile (ignores high peaks, tracks low/rest state)
        baseline = np.percentile(self.force_history, self.baseline_percentile)
        
        # Set threshold = baseline + sensitivity margin
        self.force_threshold = int(baseline + self.sensitivity_margin)
        
        # Safety bounds: keep threshold reasonable
        self.force_threshold = max(50, min(900, self.force_threshold))
    
    def get_line_thickness(self):
        if self.force_value < self.force_threshold:
            return 0
        force_range = max(1, self.force_max - self.force_threshold)
        thickness_range = self.max_thickness - self.min_thickness
        force_above = max(0, self.force_value - self.force_threshold)
        thickness = self.min_thickness + (force_above / force_range) * thickness_range
        base_thickness = int(max(self.min_thickness, min(self.max_thickness, thickness)))
        
        # Make eraser larger
        if self.drawing_mode == self.ERASER_MODE:
            return int(base_thickness * self.eraser_multiplier)
        return base_thickness

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

    def save_canvas_state(self):
        """Save current canvas state for undo"""
        self.canvas_history.append(self.canvas.copy())

    def undo(self):
        """Restore previous canvas state"""
        if len(self.canvas_history) > 1:  # Keep at least one state
            self.canvas_history.pop()  # Remove current state
            self.canvas = self.canvas_history[-1].copy()  # Restore previous
            print("↩️  Undo successful")
        else:
            print("⚠️  Nothing to undo")

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
        self.save_canvas_state()  # Save clean canvas state after calibration
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

    def detect_green(self, frame):
        """
        Fast green LED detection with ROI, V bright-core, top-K cap, and one erode.
        Returns (cx, cy), mask  or  (None, mask).
        """
        if frame.shape[-1] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        H, W = frame.shape[:2]

        # Choose ROI (expand if recently lost, fallback to full frame occasionally)
        use_full = False
        now = time.monotonic()
        if self.last_green_pos_raw is None or (now - self.last_fullframe_check) >= (1.0 / max(1, self.max_fullframe_checks)):
            use_full = (self.last_green_pos_raw is None)
            if not use_full and (now - self.last_fullframe_check) >= (1.0 / self.max_fullframe_checks):
                # periodic full-frame sweep to recover if drifted
                use_full = True
                self.last_fullframe_check = now

        if use_full:
            roi = frame
            offset = (0, 0)
            self.roi_half = self.roi_half_base  # reset after full frame
        else:
            cx0, cy0 = self.last_green_pos_raw
            x0 = max(0, cx0 - self.roi_half)
            y0 = max(0, cy0 - self.roi_half)
            x1 = min(W, cx0 + self.roi_half)
            y1 = min(H, cy0 + self.roi_half)
            roi = frame[y0:y1, x0:x1]
            offset = (x0, y0)

        # Optional downscale inside ROI
        if self.roi_downscale != 1.0:
            roi_small = cv2.resize(roi, None, fx=self.roi_downscale, fy=self.roi_downscale, interpolation=cv2.INTER_LINEAR)
        else:
            roi_small = roi

        hsv = cv2.cvtColor(roi_small, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # HSV gate to green
        mask = cv2.inRange(hsv, self.green_lower, self.green_upper)

        # Bright-core gate on V channel to ignore glow/bleed
        if self.V_BRIGHT is not None:
            bright = cv2.threshold(v, self.V_BRIGHT, 255, cv2.THRESH_BINARY)[1]
            mask = cv2.bitwise_and(mask, bright)

        # One erode to shrink bloom
        if self.ERODE_ON:
            mask = cv2.erode(mask, self.ERODE_K, iterations=1)

        # Collect candidate pixels
        ys, xs = np.where(mask > 0)
        
        # NEW: Check if we have enough pixels to be a valid detection
        if ys.size < self.MIN_DETECTION_AREA:
            self.position_history.clear()
            # widen ROI slightly if lost
            self.roi_half = min(max(self.roi_half_base, self.roi_half + 10), max(H, W))
            return None, mask

        # Cap to TOP_K brightest to bound work
        vals = v[ys, xs]
        K = min(self.TOP_K, vals.size)
        # np.argpartition is O(n)
        idx = np.argpartition(vals, -K)[-K:]
        xs_k = xs[idx]; ys_k = ys[idx]
        wgt  = vals[idx].astype(np.float32)

        # brightness-weighted centroid
        cx = int(np.average(xs_k, weights=wgt))
        cy = int(np.average(ys_k, weights=wgt))

        # Scale back if ROI downscaled
        if self.roi_downscale != 1.0:
            cx = int(cx / self.roi_downscale)
            cy = int(cy / self.roi_downscale)

        # Convert to full-frame coordinates
        cx += offset[0]
        cy += offset[1]

        # Update last raw pos and tighten ROI again
        self.last_green_pos_raw = (cx, cy)
        self.roi_half = max(self.roi_half_base, int(self.roi_half_base))  # snap back to base
        return (cx, cy), mask

    # ----------------------------------------------------------------
    # Main Loop
    # ----------------------------------------------------------------
    def run(self):
        last_frame_marked = None
        last_canvas_display = None
        show_mask = True  # Show mask by default to help debugging
        was_drawing = False  # Track if we were drawing in the last frame
        
        # Track cursor position for image pasting
        current_canvas_cursor = None

        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("⚠️ Camera feed lost")
                break

            now_ms = int(time.time() * 1000)

            # --- PROCESSING: only at proc_fps ---
            if now_ms - self.last_proc_ms >= self.proc_period_ms:
                self.last_proc_ms = now_ms
                
                # AUTO-TUNE: Update threshold based on force history
                self.update_adaptive_threshold()

                # ArUco only when calibrating or not yet calibrated
                if (not self.is_calibrated) or self.awaiting_calibration:
                    fiducials, frame_marked = self.detect_fiducials(frame)
                else:
                    fiducials = None
                    frame_marked = frame.copy()

                # Fast GREEN LED detection (ROI + top-K)
                green_pos, mask = self.detect_green(frame)
                green_pos_smoothed = self.smooth_position(green_pos)

                drawing_enabled = self.is_drawing_enabled()
                line_thickness  = self.get_line_thickness()

                # Draw LED cursor on camera view
                if green_pos_smoothed is not None:
                    cv2.circle(frame_marked, green_pos_smoothed, 10, (0, 255, 0), 2)
                    cv2.circle(frame_marked, green_pos_smoothed, 3,  (0, 255, 0), -1)

                    if self.transform_matrix is not None:
                        draw_pos = self.transform_point(green_pos_smoothed)
                        draw_pos = (
                            max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                            max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                        )
                        
                        # Store cursor position for pasting
                        current_canvas_cursor = draw_pos
                        
                        if drawing_enabled and line_thickness > 0:
                            # Save canvas state at the start of a new stroke
                            if not was_drawing:
                                self.save_canvas_state()
                                was_drawing = True
                            
                            if self.last_draw_point is not None:
                                # ========== MODIFIED: Use current_color instead of fixed white ==========
                                # Choose color based on mode
                                if self.drawing_mode == self.BRUSH_MODE:
                                    color = self.current_color  # Use selected color
                                else:  # ERASER_MODE
                                    color = (0, 0, 0)  # Black for eraser
                                
                                cv2.line(self.canvas, self.last_draw_point, draw_pos,
                                         color, line_thickness)
                            self.last_draw_point = draw_pos
                        else:
                            self.last_draw_point = None
                            was_drawing = False
                else:
                    self.last_draw_point = None
                    self.position_history.clear()
                    was_drawing = False
                    current_canvas_cursor = None

                # Status overlays (done at processing rate, not UI rate)
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
                threshold_n   = self.to_newtons(self.force_threshold)
                
                # Show baseline if auto-tuning
                if self.auto_tune_enabled and len(self.force_history) >= 30:
                    baseline = np.percentile(self.force_history, self.baseline_percentile)
                    baseline_n = self.to_newtons(baseline)
                    force_text = f"Force: {self.force_value} ({force_newtons:.3f}N) | Baseline: {int(baseline)} ({baseline_n:.3f}N) | Thresh: {self.force_threshold}"
                else:
                    force_text = f"Force: {self.force_value} ({force_newtons:.3f}N) | Threshold: {self.force_threshold} ({threshold_n:.3f}N)"
                
                cv2.putText(frame_marked, force_text, (30, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                # Auto-tune status
                auto_tune_text = f"AUTO-TUNE: {'ON' if self.auto_tune_enabled else 'OFF'} | Sensitivity: +{self.sensitivity_margin}"
                auto_tune_color = (0, 255, 0) if self.auto_tune_enabled else (128, 128, 128)
                cv2.putText(frame_marked, auto_tune_text, (30, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, auto_tune_color, 2)

                drawing_text  = f"DRAWING: {'ON' if drawing_enabled else 'OFF'} (Thickness: {line_thickness}px)"
                drawing_color = (0, 255, 255) if drawing_enabled else (128, 128, 128)
                cv2.putText(frame_marked, drawing_text, (30, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, drawing_color, 2)
                
                # LED brightness threshold (for debugging cursor issues)
                brightness_text = f"LED Brightness Filter: {self.V_BRIGHT} (use ,/. to adjust)"
                cv2.putText(frame_marked, brightness_text, (30, 290),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 255, 128), 2)
                
                # Drawing mode indicator
                mode_text = f"MODE: {'BRUSH' if self.drawing_mode == self.BRUSH_MODE else 'ERASER'}"
                mode_color = (255, 255, 255) if self.drawing_mode == self.BRUSH_MODE else (0, 165, 255)
                cv2.putText(frame_marked, mode_text, (30, 320),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
                
                # ========== NEW: Color indicator ==========
                color_name = self.color_names[self.current_color_key]
                color_text = f"COLOR: {color_name} (Key {self.current_color_key})"
                # Draw the text in the actual color (with white outline for visibility)
                cv2.putText(frame_marked, color_text, (30, 351),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)  # Black outline
                cv2.putText(frame_marked, color_text, (30, 350),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.current_color, 2)
                
                # ========== NEW: Image pasting status ==========
                if self.paste_image is not None:
                    paste_text = f"IMAGE LOADED ({self.paste_image_size}x{self.paste_image_size}px) - Press [P] to paste | [ ] to resize"
                    cv2.putText(frame_marked, paste_text, (30, 380),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Keybinds at the bottom of the screen
                h, w = frame_marked.shape[:2]
                keybind_y_start = h - 110  # Made taller for more lines
                cv2.rectangle(frame_marked, (0, keybind_y_start - 10), (w, h), (0, 0, 0), -1)  # Black background
                cv2.putText(frame_marked, "KEYS: [B]rush | [E]raser | [X]Clear | [U]ndo | [C]alibrate | [+/-]Sensitivity | [Q]uit", 
                           (10, keybind_y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame_marked, "      [R]eset | [A]uto-tune | [M]ask | [T]HSV Tuner | [,/.]Brightness", 
                           (10, keybind_y_start + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.putText(frame_marked, "      [1-5]Colors | [I]Load Image | [P]aste Image | [ ]Resize", 
                           (10, keybind_y_start + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

                # Compose canvas display with cursor
                canvas_display = self.canvas.copy()
                if green_pos_smoothed is not None and self.transform_matrix is not None:
                    draw_pos = self.transform_point(green_pos_smoothed)
                    draw_pos = (
                        max(0, min(self.canvas_size[1] - 1, draw_pos[0])),
                        max(0, min(self.canvas_size[0] - 1, draw_pos[1]))
                    )
                    cursor_radius = max(5, line_thickness + 2)
                    
                    # ========== MODIFIED: Cursor shows current color ==========
                    # Cursor color based on mode
                    if self.drawing_mode == self.BRUSH_MODE:
                        cursor_color = self.current_color if self.is_calibrated else (0, 165, 255)
                    else:  # ERASER_MODE
                        cursor_color = (0, 165, 255)  # Orange for eraser
                    
                    if not drawing_enabled:
                        cursor_color = (128, 128, 128)
                    
                    cv2.circle(canvas_display, draw_pos, cursor_radius, cursor_color, 2)
                    cv2.circle(canvas_display, draw_pos, 3, cursor_color, -1)

                last_frame_marked   = frame_marked
                last_canvas_display = canvas_display
                
                # Show detection mask for debugging
                if show_mask:
                    mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    
                    # Count total white pixels in mask
                    total_pixels = np.count_nonzero(mask)
                    
                    # Draw detection info
                    cv2.putText(mask_display, "WHITE = Detected Green", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(mask_display, f"Pixels detected: {total_pixels}", (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.putText(mask_display, f"Min required: {self.MIN_DETECTION_AREA}", (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # Show detection status
                    if green_pos is not None:
                        status_text = f"TRACKING at ({green_pos[0]}, {green_pos[1]})"
                        status_color = (0, 255, 0)
                    else:
                        if total_pixels > 0:
                            status_text = f"REJECTED (too few pixels)"
                            status_color = (0, 165, 255)
                        else:
                            status_text = "NO DETECTION"
                            status_color = (0, 0, 255)
                    
                    cv2.putText(mask_display, status_text, (10, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                    
                    # Draw detection point if found
                    if green_pos is not None:
                        cv2.circle(mask_display, green_pos, 10, (0, 255, 0), 2)
                        cv2.circle(mask_display, green_pos, 3, (0, 255, 0), -1)
                    
                    cv2.putText(mask_display, "Press 'm' to hide | 't' for HSV tuner", (10, mask_display.shape[0] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    cv2.imshow("Green Detection Mask (Debugging)", mask_display)

                # Handle calibration trigger (only after processing step)
                if self.awaiting_calibration and fiducials is not None:
                    self.setup_transform(fiducials)

            # --- UI: only refresh at ui_fps ---
            if last_frame_marked is not None and (now_ms - self.last_ui_ms >= self.ui_period_ms):
                self.last_ui_ms = now_ms
                cv2.imshow("Phone Camera View", last_frame_marked)
                if last_canvas_display is not None:
                    cv2.imshow("Drawing Canvas (Project This)", last_canvas_display)

            key = cv2.waitKey(1) & 0xFF  # small wait; pacing via fps gates
            if key == ord('q'):
                break
            elif key == ord('b'):
                # Brush mode
                self.drawing_mode = self.BRUSH_MODE
                print("✏️  Switched to BRUSH mode")
            elif key == ord('e'):
                # Eraser mode
                self.drawing_mode = self.ERASER_MODE
                print("🧹 Switched to ERASER mode")
            elif key == ord('x'):
                # Clear canvas
                if self.is_calibrated:
                    self.canvas = self._make_clean_canvas()
                    self.save_canvas_state()
                    print("🗑️  Canvas cleared!")
                else:
                    print("⚠️  Calibrate first before clearing")
                self.last_draw_point = None
            elif key == ord('u'):
                # Undo last action
                self.undo()
                self.last_draw_point = None
            elif key == ord('c'):
                # Enter calibration mode: show markers and start detecting
                self.awaiting_calibration = True
                self.is_calibrated = False
                self.canvas = self._make_fiducial_canvas()
                print("📐 Calibration mode: show all 4 ArUco markers.")
            elif key == ord('r'):
                if self.is_calibrated:
                    self.canvas = self._make_clean_canvas()
                    self.save_canvas_state()
                    print("🎨 Drawing reset (clean black canvas)")
                else:
                    self.canvas = self._make_fiducial_canvas()
                    print("🎨 Canvas reset (showing calibration markers)")
                self.last_draw_point = None
            elif key == ord('a'):
                # Toggle auto-tuning
                self.auto_tune_enabled = not self.auto_tune_enabled
                status = "ENABLED" if self.auto_tune_enabled else "DISABLED"
                print(f"🤖 Auto-tune: {status}")
                if not self.auto_tune_enabled:
                    print(f"   Manual threshold locked at: {self.force_threshold}")
            elif key == ord('+') or key == ord('='):
                # Increase sensitivity (lower threshold = more sensitive)
                if self.auto_tune_enabled:
                    self.sensitivity_margin += 25
                    print(f"⬆️  Sensitivity: +{self.sensitivity_margin} above baseline (more force needed)")
                else:
                    self.force_threshold += 25
                    threshold_n = self.to_newtons(self.force_threshold)
                    print(f"⬆️  Manual threshold: {self.force_threshold} ({threshold_n:.3f}N)")
            elif key == ord('-') or key == ord('_'):
                # Decrease sensitivity (higher threshold = less sensitive)
                if self.auto_tune_enabled:
                    self.sensitivity_margin = max(25, self.sensitivity_margin - 25)
                    print(f"⬇️  Sensitivity: +{self.sensitivity_margin} above baseline (less force needed)")
                else:
                    self.force_threshold = max(0, self.force_threshold - 25)
                    threshold_n = self.to_newtons(self.force_threshold)
                    print(f"⬇️  Manual threshold: {self.force_threshold} ({threshold_n:.3f}N)")
            elif key == ord('m'):
                # Toggle mask display
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow("Green Detection Mask (Debugging)")
                print(f"🎭 Mask display: {'ON' if show_mask else 'OFF'}")
            elif key == ord('t'):
                # Launch HSV tuner
                print("🎨 Launching HSV tuner...")
                cv2.destroyAllWindows()
                self.test_color_range()
                # Recreate main windows after tuner closes
                print("🔄 Returning to main application...")
            
            # ========== NEW: Color selection keys (1-5) ==========
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                # Convert ASCII code to number (1-5)
                color_key = int(chr(key))
                self.current_color = self.color_palette[color_key]
                self.current_color_key = color_key
                color_name = self.color_names[color_key]
                print(f"🎨 Color changed to: {color_name}")
            
            # ========== NEW: Image pasting keys ==========
            elif key == ord('i'):
                # Load an image
                self.load_image()
            
            elif key == ord('p'):
                # Paste image at cursor position
                if current_canvas_cursor is not None:
                    self.paste_image_at_cursor(current_canvas_cursor)
                else:
                    print("⚠️  Cursor not detected - cannot paste image")
            
            # ========== NEW: Image size adjustment keys ==========
            elif key == ord('['):
                # Decrease paste image size
                self.change_paste_size(increase=False)
            
            elif key == ord(']'):
                # Increase paste image size
                self.change_paste_size(increase=True)
            
            # ========== NEW: Brightness threshold adjustment ==========
            elif key == ord(',') or key == ord('<'):
                # Decrease V_BRIGHT threshold (more sensitive, detects more)
                self.V_BRIGHT = max(50, self.V_BRIGHT - 10)
                print(f"🔆 Brightness threshold: {self.V_BRIGHT} (more sensitive)")
            
            elif key == ord('.') or key == ord('>'):
                # Increase V_BRIGHT threshold (less sensitive, detects less)
                self.V_BRIGHT = min(255, self.V_BRIGHT + 10)
                print(f"🔆 Brightness threshold: {self.V_BRIGHT} (less sensitive)")

        # Cleanup
        self.running = False
        self.camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = GreenPixelTracker()
    tracker.run()