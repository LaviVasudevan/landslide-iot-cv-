#!/usr/bin/env python3
"""
monitor.py - Real-time Landslide Detection
Processes live PiCamera feed using optical flow analysis
Outputs detection status to console (0=SAFE, 1=MIDDLE, 2=DANGER)
"""

import cv2
import json
import numpy as np
from picamera2 import Picamera2
import time
from collections import deque

class LandslideDetector:
    """Main landslide detection logic"""
    def __init__(self, config):
        self.config = config
        self.roi_corners = np.array(config['roi_coordinates'], dtype=np.int32)
        self.thresholds = config['thresholds']
        
        # Detection parameters
        self.motion_threshold = self.thresholds['motion_threshold']
        self.alignment_threshold = self.thresholds['alignment_threshold']
        self.persistence_required = self.thresholds['persistence_required']
        self.motion_pixel_threshold = self.thresholds['motion_pixel_threshold']
        
        # State tracking
        self.consecutive_motion = 0
        self.consecutive_safe = 0  # NEW: Track consecutive safe frames
        self.alert_triggered = False
        self.frame_count = 0
        self.last_output_time = time.time()
        self.output_interval = 1.0  # Output every 1 second
        
        # Create ROI mask
        h = config['image_height']
        w = config['image_width']
        self.roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(self.roi_mask, [self.roi_corners], 255)
        
        # Previous frame for optical flow
        self.prev_gray = None
        
        # Last heartbeat time
        self.last_heartbeat = time.time()
        self.heartbeat_interval = 30  # seconds
    
    def process_frame(self, frame):
        """Process single frame and return detection status"""
        self.frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return self._get_status(0, 0, 0, 0)
        
        # Optical Flow - EXACT same parameters as Streamlit app
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray,
            None, 0.5, 3, 15, 3, 5, 1.2, 0
        )
        
        # Calculate magnitude and direction - EXACT same logic
        mag = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
        norm = np.linalg.norm(flow, axis=2, keepdims=True)
        unit_flow = np.divide(flow, norm + 1e-6)
        downward_alignment = unit_flow[..., 1]
        
        # FIX 1: More strict downward motion detection within ROI
        # Only count pixels that are:
        # 1. Inside ROI (mask == 255)
        # 2. Have sufficient motion magnitude
        # 3. Have strong downward alignment (not just slightly downward)
        downward_mask = (
            (self.roi_mask == 255) &  # Must be in ROI
            (mag > self.motion_threshold) &  # Sufficient motion
            (downward_alignment > self.alignment_threshold)  # Strong downward component
        )
        
        num_downward_pixels = int(np.sum(downward_mask))
        
        # Calculate averages ONLY within ROI - more accurate
        roi_indices = (self.roi_mask == 255)
        roi_flow_mag = mag[roi_indices]
        roi_downward_align = downward_alignment[roi_indices]
        
        avg_flow_mag = float(np.mean(roi_flow_mag)) if len(roi_flow_mag) > 0 else 0.0
        avg_downward_align = float(np.mean(roi_downward_align)) if len(roi_downward_align) > 0 else 0.0
        
        # Update consecutive motion counter with stricter logic
        if num_downward_pixels > self.motion_pixel_threshold:
            self.consecutive_motion += 1
            self.consecutive_safe = 0  # Reset safe counter
        else:
            self.consecutive_motion = max(0, self.consecutive_motion - 1)
            self.consecutive_safe += 1  # Increment safe counter
        
        # FIX 2: Reset alert if conditions become safe for sustained period
        # Reset alert if we've had enough consecutive safe frames
        if self.consecutive_safe >= self.persistence_required:
            self.alert_triggered = False
        
        # Calculate risk indicators - stricter criteria
        risk_indicators = 0
        if num_downward_pixels > self.motion_pixel_threshold:
            risk_indicators += 1
        if avg_flow_mag > self.motion_threshold * 1.2:  # Slightly higher threshold
            risk_indicators += 1
        if avg_downward_align > self.alignment_threshold:
            risk_indicators += 1
        if self.consecutive_motion >= self.persistence_required:
            risk_indicators += 2
        
        # Determine risk level (0=SAFE, 1=MIDDLE, 2=DANGER)
        risk_level = 0
        if self.consecutive_motion >= self.persistence_required:
            risk_level = 2  # DANGER - persistent downward motion
            self.alert_triggered = True
        elif num_downward_pixels > self.motion_pixel_threshold:
            risk_level = 1  # MIDDLE - motion detected but not persistent
        
        # Update previous frame
        self.prev_gray = gray
        
        return self._get_status(num_downward_pixels, avg_flow_mag, risk_level, risk_indicators)
    
    def _get_status(self, pixels, magnitude, risk, indicators):
        """Format status dictionary"""
        return {
            'frame': self.frame_count,
            'downward_pixels': pixels,
            'flow_magnitude': magnitude,
            'consecutive_motion': self.consecutive_motion,
            'consecutive_safe': self.consecutive_safe,
            'risk_level': risk,
            'risk_indicators': indicators,
            'alert': self.alert_triggered,
            'timestamp': int(time.time())
        }
    
    def should_output(self):
        """Check if it's time to output status"""
        if time.time() - self.last_output_time >= self.output_interval:
            self.last_output_time = time.time()
            return True
        return False

def main():
    """Main monitoring loop"""
    print("="*60)
    print("Landslide Detection Monitor")
    print("="*60)
    
    # Load configuration
    try:
        with open("picamroi.json", "r") as f:
            config = json.load(f)
        print(f"\n✓ Loaded configuration:")
        print(f"  Camera ID: {config['camera_id']}")
        print(f"  Location: {config['location']}")
        print(f"  Setup Date: {config['setup_date']}")
    except FileNotFoundError:
        print("\n✗ Error: picamroi.json not found!")
        print("  Please run setupcam.py first to configure ROI")
        return
    
    # Initialize detector
    detector = LandslideDetector(config)
    print("\n✓ Detector initialized")
    
    # Initialize camera
    print("\nInitializing camera...")
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (config['image_width'], config['image_height']), "format": "RGB888"}
    )
    picam2.configure(camera_config)
    picam2.start()
    print("✓ Camera started")
    
    time.sleep(2)  # Camera warm-up
    
    print("\n" + "="*60)
    print("Monitoring started - Press Ctrl+C to quit")
    print("Output: 0=SAFE, 1=MIDDLE, 2=DANGER")
    print("="*60 + "\n")
    
    # Optional: Display window (set to False for headless)
    display_enabled = True
    if display_enabled:
        cv2.namedWindow("Landslide Monitor", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Landslide Monitor", 1280, 720)
    
    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Process frame
            status = detector.process_frame(frame)
            
            # Output status at intervals
            if detector.should_output():
                # Simple output: just the risk level (0, 1, or 2)
                print(status['risk_level'])
                
                # Optional: More detailed output (comment/uncomment as needed)
                # print(f"{status['risk_level']} | Pixels: {status['downward_pixels']} | Flow: {status['flow_magnitude']:.2f} | Consec: {status['consecutive_motion']}/{status['consecutive_safe']}")
            
            # Display video with overlay (optional)
            if display_enabled:
                display = frame.copy()
                
                # Draw ROI overlay
                overlay = display.copy()
                cv2.fillPoly(overlay, [detector.roi_corners], (0, 255, 0))
                display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)
                cv2.polylines(display, [detector.roi_corners], True, (0, 255, 0), 2)
                
                # Status text overlay
                h, w = display.shape[:2]
                risk_labels = ['SAFE', 'MIDDLE', 'DANGER']
                risk_colors = [(0, 255, 0), (0, 165, 255), (0, 0, 255)]
                
                # Dark background for text
                cv2.rectangle(display, (0, 0), (350, 155), (0, 0, 0), -1)
                
                # Status
                color = risk_colors[status['risk_level']]
                cv2.putText(display, f"Status: {risk_labels[status['risk_level']]}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(display, f"Pixels: {status['downward_pixels']}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Flow: {status['flow_magnitude']:.2f}", 
                           (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Motion: {status['consecutive_motion']} Safe: {status['consecutive_safe']}", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Risk: {status['risk_indicators']}/5", 
                           (10, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Alert banner if danger (only shows when alert is active)
                if status['alert']:
                    cv2.rectangle(display, (0, h - 60), (w, h), (0, 0, 255), -1)
                    cv2.putText(display, "LANDSLIDE DETECTED!", 
                               (w//2 - 200, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                
                cv2.imshow("Landslide Monitor", display)
                
                # Press 'q' to quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        picam2.stop()
        if display_enabled:
            cv2.destroyAllWindows()
        print("✓ Stopped")

if __name__ == "__main__":
    main()
