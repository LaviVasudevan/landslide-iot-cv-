#!/usr/bin/env python3
"""
setupcam.py - ROI Setup for Landslide Detection
Captures frame from PiCamera and allows user to define ROI
Saves coordinates to picamroi.json
"""

import cv2
import json
import numpy as np
from picamera2 import Picamera2
import time

# Global variables for mouse callback
roi_corners = []
current_point = None
dragging_point = None
frame = None

def mouse_callback(event, x, y, flags, param):
    """Handle mouse events for ROI selection"""
    global roi_corners, current_point, dragging_point, frame
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Check if clicking near existing point (within 10 pixels)
        for i, corner in enumerate(roi_corners):
            dist = np.sqrt((x - corner[0])**2 + (y - corner[1])**2)
            if dist < 15:
                dragging_point = i
                return
        
        # Add new point if less than 4 corners
        if len(roi_corners) < 4:
            roi_corners.append([x, y])
    
    elif event == cv2.EVENT_MOUSEMOVE:
        current_point = (x, y)
        
        # Drag existing point
        if dragging_point is not None:
            roi_corners[dragging_point] = [x, y]
    
    elif event == cv2.EVENT_LBUTTONUP:
        dragging_point = None

def draw_roi(img, corners, current_pos=None):
    """Draw ROI overlay on image"""
    display = img.copy()
    
    if len(corners) > 0:
        # Draw connecting lines
        if len(corners) > 1:
            for i in range(len(corners)):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % len(corners)])
                cv2.line(display, pt1, pt2, (0, 255, 0), 2)
        
        # Draw filled polygon if 4 corners defined
        if len(corners) == 4:
            pts = np.array(corners, dtype=np.int32)
            overlay = display.copy()
            cv2.fillPoly(overlay, [pts], (0, 255, 0))
            display = cv2.addWeighted(display, 0.7, overlay, 0.3, 0)
        
        # Draw corner points
        colors = [(255, 0, 0), (0, 255, 0), (0, 165, 255), (128, 0, 128)]
        labels = ["TL", "TR", "BR", "BL"]
        
        for i, corner in enumerate(corners):
            color = colors[i] if i < 4 else (255, 255, 255)
            label = labels[i] if i < 4 else f"P{i}"
            
            cv2.circle(display, tuple(corner), 8, color, -1)
            cv2.circle(display, tuple(corner), 10, (255, 255, 255), 2)
            cv2.putText(display, label, (corner[0] + 15, corner[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    # Instructions
    h, w = display.shape[:2]
    instructions = [
        "Click to add corner points (4 total)",
        "Drag points to adjust position",
        "Press 's' to save and exit",
        "Press 'r' to reset",
        "Press 'q' to quit without saving"
    ]
    
    y_offset = 30
    for i, text in enumerate(instructions):
        cv2.putText(display, text, (10, y_offset + i * 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Status
    status = f"Corners: {len(corners)}/4"
    cv2.putText(display, status, (10, h - 20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    return display

def save_roi_config(corners, camera_id="CAM_01", location="", thresholds=None):
    """Save ROI configuration to JSON file"""
    if thresholds is None:
        thresholds = {
            "motion_threshold": 1.5,
            "alignment_threshold": 0.9,
            "persistence_required": 3,
            "motion_pixel_threshold": 200
        }
    
    config = {
        "roi_coordinates": corners,
        "camera_id": camera_id,
        "location": location,
        "setup_date": time.strftime("%Y-%m-%d %H:%M:%S"),
        "thresholds": thresholds,
        "image_width": frame.shape[1],
        "image_height": frame.shape[0]
    }
    
    with open("picamroi.json", "w") as f:
        json.dump(config, f, indent=4)
    
    print(f"\n✓ ROI configuration saved to picamroi.json")
    print(f"  Camera ID: {camera_id}")
    print(f"  Location: {location}")
    print(f"  Corners: {corners}")

def main():
    """Main ROI setup function"""
    global roi_corners, frame
    
    print("="*60)
    print("Landslide Detection - ROI Setup")
    print("="*60)
    
    # Initialize PiCamera2
    print("\nInitializing camera...")
    picam2 = Picamera2()
    
    # Configure camera for preview
    config = picam2.create_preview_configuration(
        main={"size": (1280, 720), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    
    print("✓ Camera initialized")
    print("\nWaiting for camera to stabilize...")
    time.sleep(2)
    
    # Capture initial frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    
    # Try to load existing ROI
    try:
        with open("picamroi.json", "r") as f:
            config = json.load(f)
            roi_corners = config.get("roi_coordinates", [])
            print(f"\n✓ Loaded existing ROI with {len(roi_corners)} corners")
    except FileNotFoundError:
        print("\nNo existing ROI found, starting fresh")
        h, w = frame.shape[:2]
        # Default quadrilateral
        roi_corners = [
            [int(w * 0.25), int(h * 0.25)],
            [int(w * 0.75), int(h * 0.25)],
            [int(w * 0.75), int(h * 0.75)],
            [int(w * 0.25), int(h * 0.75)]
        ]
    
    # Setup window
    window_name = "ROI Setup - Landslide Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    print("\n" + "="*60)
    print("ROI Setup Interface")
    print("="*60)
    print("Controls:")
    print("  - Click to add corner points (4 corners needed)")
    print("  - Drag existing points to adjust")
    print("  - Press 's' to save configuration")
    print("  - Press 'r' to reset all corners")
    print("  - Press 'q' to quit without saving")
    print("="*60 + "\n")
    
    while True:
        # Capture fresh frame
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Draw ROI overlay
        display = draw_roi(frame, roi_corners, current_point)
        
        # Show frame
        cv2.imshow(window_name, display)
        
        # Handle keyboard input
        key = cv2.waitKey(30) & 0xFF
        
        if key == ord('s'):
            if len(roi_corners) == 4:
                # Get additional info
                print("\n" + "-"*60)
                camera_id = input("Enter Camera ID [CAM_01]: ").strip() or "CAM_01"
                location = input("Enter Location [Lonavala_Ghat]: ").strip() or "Lonavala_Ghat"
                
                save_roi_config(roi_corners, camera_id, location)
                print("\n✓ Setup complete! You can now run monitor.py")
                break
            else:
                print(f"\n⚠ Need exactly 4 corners (currently have {len(roi_corners)})")
        
        elif key == ord('r'):
            roi_corners = []
            print("\n✓ ROI reset")
        
        elif key == ord('q'):
            print("\n⚠ Exiting without saving")
            break
    
    # Cleanup
    cv2.destroyAllWindows()
    picam2.stop()
    print("\n✓ Camera stopped")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
