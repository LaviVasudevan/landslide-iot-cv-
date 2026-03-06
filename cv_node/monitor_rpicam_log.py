#!/usr/bin/env python3
"""
CV_Monitor_LoRa.py - Landslide Detection with LoRa Transmission
Processes live PiCamera feed using optical flow analysis
Transmits via LoRa: "C0" (SAFE) or "C2" (DANGER)
Source ID: C = Computer Vision Pipeline
"""

import cv2
import json
import numpy as np
from picamera2 import Picamera2
import time
import spidev
import lgpio
from datetime import datetime

# LoRa Configuration
RST_PIN = 25
SPI_BUS = 0
SPI_DEVICE = 0

# LoRa Registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0C
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_IRQ_FLAGS = 0x12
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39
REG_VERSION = 0x42
REG_PA_DAC = 0x4D

MODE_LONG_RANGE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_TX = 0x03

class LoRaTransmitter:
    """LoRa transmission handler"""
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 500000
        self.spi.mode = 0
    
    def read_reg(self, addr):
        return self.spi.xfer2([addr & 0x7F, 0x00])[1]
    
    def write_reg(self, addr, val):
        self.spi.xfer2([addr | 0x80, val])
    
    def reset_lora(self):
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(h, RST_PIN)
        lgpio.gpio_write(h, RST_PIN, 0)
        time.sleep(0.01)
        lgpio.gpio_write(h, RST_PIN, 1)
        time.sleep(0.1)
        lgpio.gpiochip_close(h)
    
    def init_lora(self):
        self.reset_lora()
        
        version = self.read_reg(REG_VERSION)
        if version != 0x12:
            print(f"Warning: LoRa Version 0x{version:02X}")
            return False
        
        self.write_reg(REG_OP_MODE, MODE_SLEEP)
        time.sleep(0.01)
        self.write_reg(REG_OP_MODE, MODE_SLEEP | MODE_LONG_RANGE)
        time.sleep(0.01)
        
        # 433MHz
        frf = int((433E6 / 32000000.0) * 524288)
        self.write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_reg(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_reg(REG_FRF_LSB, frf & 0xFF)
        
        self.write_reg(REG_FIFO_TX_BASE_ADDR, 0x00)
        self.write_reg(REG_LNA, 0x23)
        self.write_reg(REG_MODEM_CONFIG_1, 0x72)  # SF7, BW125, CR4/5
        self.write_reg(REG_MODEM_CONFIG_2, 0x74)
        self.write_reg(REG_MODEM_CONFIG_3, 0x04)
        self.write_reg(REG_PREAMBLE_MSB, 0x00)
        self.write_reg(REG_PREAMBLE_LSB, 0x08)
        self.write_reg(REG_SYNC_WORD, 0x12)
        self.write_reg(REG_PA_CONFIG, 0x8F)
        self.write_reg(REG_PA_DAC, 0x87)
        self.write_reg(REG_OP_MODE, MODE_STDBY | MODE_LONG_RANGE)
        time.sleep(0.01)
        
        print("✓ LoRa Ready (433MHz, SF7)")
        return True
    
    def send_packet(self, data):
        self.write_reg(REG_OP_MODE, MODE_STDBY | MODE_LONG_RANGE)
        time.sleep(0.01)
        self.write_reg(REG_IRQ_FLAGS, 0xFF)
        self.write_reg(REG_FIFO_ADDR_PTR, 0x00)
        
        for byte in data:
            self.write_reg(REG_FIFO, byte)
        
        self.write_reg(REG_PAYLOAD_LENGTH, len(data))
        self.write_reg(REG_OP_MODE, MODE_TX | MODE_LONG_RANGE)
        
        timeout = 0
        while (self.read_reg(REG_IRQ_FLAGS) & 0x08) == 0:
            time.sleep(0.01)
            timeout += 1
            if timeout > 200:
                break
        
        self.write_reg(REG_IRQ_FLAGS, 0xFF)
        self.write_reg(REG_OP_MODE, MODE_STDBY | MODE_LONG_RANGE)
        time.sleep(0.01)
    
    def transmit(self, zone):
        """Transmit zone status: C0 (SAFE) or C2 (DANGER)"""
        packet = f"C{zone}"
        data = [ord(c) for c in packet]
        self.send_packet(data)
        return packet
    
    def close(self):
        self.spi.close()


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
        self.consecutive_safe = 0
        self.alert_triggered = False
        self.frame_count = 0
        self.last_output_time = time.time()
        self.output_interval = 5.0  # Output every 10 seconds (matching transmission interval)
        
        # Create ROI mask
        h = config['image_height']
        w = config['image_width']
        self.roi_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(self.roi_mask, [self.roi_corners], 255)
        
        # Previous frame for optical flow
        self.prev_gray = None
        
        # Transmission counter
        self.tx_count = 0

        # Session logging
        self.session_start = datetime.now().isoformat()
        self.frame_log = []
        self.processing_times = []
        self.alert_events = []
        self._was_alert = False
    
    def process_frame(self, frame):
        """Process single frame and return detection status"""
        t_start = time.time()
        self.frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            return self._get_status(0, 0, 0, 0)
        
        # Optical Flow - EXACT same parameters as original
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray,
            None, 0.5, 3, 15, 3, 5, 1.2, 0
        )
        
        # Calculate magnitude and direction
        mag = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
        norm = np.linalg.norm(flow, axis=2, keepdims=True)
        unit_flow = np.divide(flow, norm + 1e-6)
        downward_alignment = unit_flow[..., 1]
        
        # Strict downward motion detection within ROI
        downward_mask = (
            (self.roi_mask == 255) &
            (mag > self.motion_threshold) &
            (downward_alignment > self.alignment_threshold)
        )
        
        num_downward_pixels = int(np.sum(downward_mask))
        
        # Calculate averages ONLY within ROI
        roi_indices = (self.roi_mask == 255)
        roi_flow_mag = mag[roi_indices]
        roi_downward_align = downward_alignment[roi_indices]
        
        avg_flow_mag = float(np.mean(roi_flow_mag)) if len(roi_flow_mag) > 0 else 0.0
        avg_downward_align = float(np.mean(roi_downward_align)) if len(roi_downward_align) > 0 else 0.0
        
        # Update consecutive motion counter
        if num_downward_pixels > self.motion_pixel_threshold:
            self.consecutive_motion += 1
            self.consecutive_safe = 0
        else:
            self.consecutive_motion = max(0, self.consecutive_motion - 1)
            self.consecutive_safe += 1
        
        # Reset alert if conditions become safe for sustained period
        if self.consecutive_safe >= self.persistence_required:
            self.alert_triggered = False
        
        # Calculate risk indicators
        risk_indicators = 0
        if num_downward_pixels > self.motion_pixel_threshold:
            risk_indicators += 1
        if avg_flow_mag > self.motion_threshold * 1.2:
            risk_indicators += 1
        if avg_downward_align > self.alignment_threshold:
            risk_indicators += 1
        if self.consecutive_motion >= self.persistence_required:
            risk_indicators += 2
        
        # Determine risk level - BINARY ONLY: 0=SAFE or 2=DANGER
        # No middle state (1) - eliminate yellow warning
        risk_level = 0
        if self.consecutive_motion >= self.persistence_required:
            risk_level = 1  # DANGER - persistent downward motion
            self.alert_triggered = True
        # Note: We skip risk_level = 1 entirely
        
        # Update previous frame
        self.prev_gray = gray

        proc_ms = (time.time() - t_start) * 1000
        self.processing_times.append(proc_ms)

        # Log alert transitions
        if risk_level == 1 and not self._was_alert:
            self.alert_events.append({'event': 'ALERT_START', 'frame': self.frame_count, 'timestamp': int(time.time())})
        elif risk_level == 0 and self._was_alert:
            self.alert_events.append({'event': 'ALERT_END', 'frame': self.frame_count, 'timestamp': int(time.time())})
        self._was_alert = (risk_level == 1)

        # Log frame
        self.frame_log.append({
            'frame': self.frame_count,
            'timestamp': int(time.time()),
            'downward_pixels': num_downward_pixels,
            'flow_magnitude': round(avg_flow_mag, 4),
            'downward_alignment': round(avg_downward_align, 4),
            'consecutive_motion': self.consecutive_motion,
            'consecutive_safe': self.consecutive_safe,
            'risk_indicators': risk_indicators,
            'risk_level': risk_level,
            'processing_ms': round(proc_ms, 2)
        })

        return self._get_status(num_downward_pixels, avg_flow_mag, risk_level, risk_indicators)
    
    def _get_status(self, pixels, magnitude, risk, indicators):
        """Format status dictionary"""
        return {
            'frame': self.frame_count,
            'downward_pixels': pixels,
            'flow_magnitude': magnitude,
            'consecutive_motion': self.consecutive_motion,
            'consecutive_safe': self.consecutive_safe,
            'risk_level': risk,  # Only 0 or 2
            'risk_indicators': indicators,
            'alert': self.alert_triggered,
            'timestamp': int(time.time())
        }
    
    def should_output(self):
        """Check if it's time to output/transmit status"""
        if time.time() - self.last_output_time >= self.output_interval:
            self.last_output_time = time.time()
            return True
        return False

    def save_session(self, config):
        """Save full session data to JSON on exit"""
        if not self.frame_log:
            print("No frames logged, skipping save.")
            return

        pt = self.processing_times
        pixels   = [f['downward_pixels'] for f in self.frame_log]
        mags     = [f['flow_magnitude'] for f in self.frame_log]
        aligns   = [f['downward_alignment'] for f in self.frame_log]
        risks    = [f['risk_level'] for f in self.frame_log]

        output = {
            'metadata': {
                'camera_id': config.get('camera_id', 'unknown'),
                'location': config.get('location', 'unknown'),
                'image_width': config.get('image_width'),
                'image_height': config.get('image_height'),
                'roi_corners': config.get('roi_coordinates'),
                'session_start': self.session_start,
                'session_end': datetime.now().isoformat(),
                'total_frames': self.frame_count,
                'total_transmissions': self.tx_count
            },
            'parameters': {
                'motion_threshold': self.motion_threshold,
                'alignment_threshold': self.alignment_threshold,
                'persistence_required': self.persistence_required,
                'motion_pixel_threshold': self.motion_pixel_threshold,
                'transmission_interval_s': self.output_interval
            },
            'summary': {
                'processing_metrics': {
                    'avg_processing_ms': round(sum(pt) / len(pt), 2),
                    'std_processing_ms': round(float(np.std(pt)), 2),
                    'min_processing_ms': round(min(pt), 2),
                    'max_processing_ms': round(max(pt), 2),
                    'avg_fps': round(1000 / (sum(pt) / len(pt)), 2)
                },
                'motion_metrics': {
                    'max_downward_pixels': max(pixels),
                    'avg_downward_pixels': round(sum(pixels) / len(pixels), 2),
                    'max_flow_magnitude': round(max(mags), 4),
                    'avg_flow_magnitude': round(sum(mags) / len(mags), 4),
                    'max_downward_alignment': round(max(aligns), 4),
                    'avg_downward_alignment': round(sum(aligns) / len(aligns), 4)
                },
                'detection_metrics': {
                    'landslide_detected': any(r == 1 for r in risks),
                    'max_consecutive_motion': max(f['consecutive_motion'] for f in self.frame_log),
                    'frames_at_safe': risks.count(0),
                    'frames_at_danger': risks.count(1),
                    'alert_events': self.alert_events
                }
            },
            'frame_data': self.frame_log
        }

        filename = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(output, f, indent=2)

        print(f"\n✓ Session saved → {filename}")
        print(f"  Frames: {self.frame_count} | TX packets: {self.tx_count} | "
              f"Avg: {output['summary']['processing_metrics']['avg_processing_ms']}ms | "
              f"Danger frames: {output['summary']['detection_metrics']['frames_at_danger']}")


def main():
    """Main monitoring loop"""
    print("="*60)
    print("CV Landslide Detection with LoRa Transmission")
    print("Source ID: C (Computer Vision)")
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
    
    # Initialize LoRa
    print("\nInitializing LoRa...")
    lora = LoRaTransmitter()
    if not lora.init_lora():
        print("✗ LoRa initialization failed!")
        return
    
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
    print("Transmission: C0=SAFE, C2=DANGER (every 10 sec)")
    print("="*60 + "\n")
    
    # Optional: Display window (set to False for headless)
    display_enabled = True
    if display_enabled:
        cv2.namedWindow("CV Monitor + LoRa", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CV Monitor + LoRa", 1280, 720)
    
    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Process frame
            status = detector.process_frame(frame)
            
            # Transmit status at intervals (every 10 seconds)
            if detector.should_output():
                # Transmit via LoRa: C0 or C2 only
                packet = lora.transmit(status['risk_level'])
                detector.tx_count += 1
                
                status_label = "SAFE" if status['risk_level'] == 0 else "DANGER"
                print(f"[TX {detector.tx_count:3d}] Sent: {packet} ({status_label}) | "
                      f"Pixels: {status['downward_pixels']} | "
                      f"Flow: {status['flow_magnitude']:.2f} | "
                      f"Motion: {status['consecutive_motion']} Safe: {status['consecutive_safe']}")
            
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
                risk_labels = ['SAFE', '', 'DANGER']  # No middle state
                risk_colors = [(0, 255, 0), (0, 0, 0), (0, 0, 255)]  # Green or Red only
                
                # Dark background for text
                cv2.rectangle(display, (0, 0), (380, 180), (0, 0, 0), -1)
                
                # Status
                color = risk_colors[status['risk_level']]
                cv2.putText(display, f"Status: {risk_labels[status['risk_level']]}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                cv2.putText(display, f"Source: C (Computer Vision)", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Pixels: {status['downward_pixels']}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Flow: {status['flow_magnitude']:.2f}", 
                           (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"Motion: {status['consecutive_motion']} Safe: {status['consecutive_safe']}", 
                           (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(display, f"TX Count: {detector.tx_count}", 
                           (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Alert banner if danger (only shows when alert is active)
                if status['alert']:
                    cv2.rectangle(display, (0, h - 60), (w, h), (0, 0, 255), -1)
                    cv2.putText(display, "LANDSLIDE DETECTED!", 
                               (w//2 - 200, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                
                cv2.imshow("CV Monitor + LoRa", display)
                
                # Press 'q' to quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user")
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        picam2.stop()
        lora.close()
        if display_enabled:
            cv2.destroyAllWindows()
        detector.save_session(config)
        print(f"✓ Stopped (Transmitted {detector.tx_count} packets)")

if __name__ == "__main__":
    main()
