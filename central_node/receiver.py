#!/usr/bin/env python3
"""
Raspberry Pi 5 - Landslide Detection Receiver with Multi-Modal Risk Assessment

Receives codes from two transmitters:
  - CV Transmitter: C0 or C1 (CV_flag)
  - Sensor Transmitter: S0, S1, S2, S3, S4 (alertCount)
  
Risk assessment runs every 10 seconds based on latest values.

Hardware:
  - LoRa Module on SPI (RST on GPIO 25)
  - Piezo Buzzer on GPIO 23

Risk Zones:
  - RED (2): CV_flag = 1 → Buzzer ON (continuous alarm)
  - YELLOW (1): CV_flag = 0 AND alertCount >= 3 → Buzzer OFF
  - GREEN (0): CV_flag = 0 AND alertCount < 3 → Buzzer OFF
"""

import spidev
import time
import lgpio
import threading
from datetime import datetime


# ==============================================================================
# GPIO CONFIGURATION
# ==============================================================================

RST_PIN = 25        # LoRa reset pin
BUZZER_PIN = 23     # Piezo buzzer signal pin
LED_YELLOW_PIN = 17 # Yellow alert LED (Physical Pin 11)


# ==============================================================================
# LORA REGISTERS
# ==============================================================================

REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_VERSION = 0x42
REG_PKT_RSSI_VALUE = 0x1A
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39


# ==============================================================================
# LORA MODES
# ==============================================================================

MODE_LONG_RANGE = 0x80
MODE_SLEEP = 0x00
MODE_STDBY = 0x01
MODE_RX_CONTINUOUS = 0x05


# ==============================================================================
# ANSI COLOR CODES
# ==============================================================================

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
CYAN = '\033[96m'
RESET = '\033[0m'
BOLD = '\033[1m'


# ==============================================================================
# GLOBAL VARIABLES - SENSOR STATE
# ==============================================================================

CV_flag = 0                 # CV transmitter: 0 or 1
alertCount = 0              # Sensor transmitter: 0 to 4
sensorThreshold = 3         # Threshold for yellow zone


# ==============================================================================
# GLOBAL VARIABLES - BUZZER CONTROL
# ==============================================================================

gpio_handle = None
alarm_active = False
alarm_thread = None


# ==============================================================================
# GLOBAL VARIABLES - ZONE STATE
# ==============================================================================

current_zone = 0            # 0=GREEN, 1=YELLOW, 2=RED


# ==============================================================================
# THREAD SAFETY
# ==============================================================================

state_lock = threading.Lock()


# ==============================================================================
# LORA COMMUNICATION FUNCTIONS
# ==============================================================================

def read_reg(spi, addr):
    """Read a single byte from LoRa register"""
    return spi.xfer2([addr & 0x7F, 0x00])[1]


def write_reg(spi, addr, val):
    """Write a single byte to LoRa register"""
    spi.xfer2([addr | 0x80, val])


def reset_lora():
    """Hardware reset LoRa module via GPIO"""
    h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(h, RST_PIN)
    lgpio.gpio_write(h, RST_PIN, 0)
    time.sleep(0.01)
    lgpio.gpio_write(h, RST_PIN, 1)
    time.sleep(0.01)
    lgpio.gpiochip_close(h)


def init_lora(spi):
    """Initialize LoRa module with configuration matching Arduino transmitters"""
    # Hardware reset
    reset_lora()
    
    # Verify LoRa module version
    version = read_reg(spi, REG_VERSION)
    if version != 0x12:
        raise Exception(f"LoRa not found! Version: 0x{version:02X}")
    
    # Enter sleep mode
    write_reg(spi, REG_OP_MODE, MODE_SLEEP)
    time.sleep(0.01)
    
    # Enable LoRa mode
    write_reg(spi, REG_OP_MODE, MODE_SLEEP | MODE_LONG_RANGE)
    time.sleep(0.01)
    
    # Set frequency to 433MHz
    frf = int((433E6 / 32000000.0) * 524288)
    write_reg(spi, REG_FRF_MSB, (frf >> 16) & 0xFF)
    write_reg(spi, REG_FRF_MID, (frf >> 8) & 0xFF)
    write_reg(spi, REG_FRF_LSB, frf & 0xFF)
    
    # Configure modem: SF7, BW125kHz, CR4/5
    write_reg(spi, REG_MODEM_CONFIG_1, 0x72)  # BW=125kHz, CR=4/5
    write_reg(spi, REG_MODEM_CONFIG_2, 0x74)  # SF=7, CRC on
    write_reg(spi, REG_MODEM_CONFIG_3, 0x04)  # AGC auto on
    
    # Set sync word to match Arduino (0x12)
    write_reg(spi, REG_SYNC_WORD, 0x12)
    
    # Set FIFO base addresses
    write_reg(spi, REG_FIFO_RX_BASE_ADDR, 0)
    
    # Enter standby mode
    write_reg(spi, REG_OP_MODE, MODE_STDBY | MODE_LONG_RANGE)
    time.sleep(0.01)
    
    print("✓ LoRa initialized (433MHz, SF7, BW125, CR4/5)")


def receive_packet(spi):
    """
    Check for and read incoming LoRa packet
    Returns: (message, rssi) tuple or None
    """
    irq = read_reg(spi, REG_IRQ_FLAGS)
    
    if irq & 0x40:  # RX done flag
        # Clear interrupt flags
        write_reg(spi, REG_IRQ_FLAGS, 0xFF)
        
        # Read packet length
        length = read_reg(spi, REG_RX_NB_BYTES)
        
        # Get FIFO address
        addr = read_reg(spi, REG_FIFO_ADDR_PTR)
        write_reg(spi, REG_FIFO_ADDR_PTR, addr)
        
        # Read packet data
        data = []
        for i in range(length):
            data.append(read_reg(spi, REG_FIFO))
        
        # Get RSSI value
        rssi = read_reg(spi, REG_PKT_RSSI_VALUE) - 164
        
        # Convert bytes to string
        try:
            message = ''.join(chr(b) for b in data)
            return (message, rssi)
        except:
            return None
    
    return None


# ==============================================================================
# MESSAGE PARSING
# ==============================================================================

def parse_message(message):
    """
    Parse received message and update sensor states
    Expected formats: C0, C1, S0, S1, S2, S3, S4
    Returns: True if valid message, False otherwise
    """
    global CV_flag, alertCount
    
    message = message.strip().upper()
    
    with state_lock:
        # CV Transmitter messages: C0 or C1
        if message.startswith('C'):
            try:
                value = int(message[1:])
                if value in [0, 1]:
                    CV_flag = value
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"{CYAN}[{timestamp}] CV status updated: CV_flag = {CV_flag}{RESET}")
                    return True
            except:
                pass
        
        # Sensor Transmitter messages: S0 to S4
        elif message.startswith('S'):
            try:
                value = int(message[1:])
                if 0 <= value <= 4:
                    alertCount = value
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"{CYAN}[{timestamp}] Sensor status updated: alertCount = {alertCount}{RESET}")
                    return True
            except:
                pass
    
    return False


# ==============================================================================
# RISK ASSESSMENT ALGORITHM
# ==============================================================================

def assess_risk():
    """
    Multi-Modal Risk Classification Algorithm
    
    Based on CV_flag and alertCount, determines zone output:
      - RED (2): CV_flag = 1 → Immediate danger, buzzer ON
      - YELLOW (1): CV_flag = 0 AND alertCount >= threshold → Warning, buzzer OFF
      - GREEN (0): CV_flag = 0 AND alertCount < threshold → Normal, buzzer OFF
    
    Returns: (zoneOutput, buzzerState) tuple
    """
    global CV_flag, alertCount, sensorThreshold
    
    with state_lock:
        cv = CV_flag
        alert = alertCount
    
    timestamp = datetime.now().strftime("%H:%M:%S")
    
    # RED ZONE: Critical landslide detection
    if cv == 1:
        zone_output = 2
        buzzer_state = True
        log_critical_event(timestamp, alert)
        return (zone_output, buzzer_state)
    
    # YELLOW ZONE: Warning level
    elif alert >= sensorThreshold:
        zone_output = 1
        buzzer_state = False
        log_warning(timestamp, alert)
        return (zone_output, buzzer_state)
    
    # GREEN ZONE: Normal conditions
    else:
        zone_output = 0
        buzzer_state = False
        log_normal(timestamp, alert)
        return (zone_output, buzzer_state)


# ==============================================================================
# LOGGING FUNCTIONS
# ==============================================================================

def log_critical_event(timestamp, alert_count):
    """Log critical event (RED ZONE)"""
    print(f"{BOLD}{RED}[{timestamp}] CRITICAL EVENT: CV_flag=1, alertCount={alert_count}{RESET}")


def log_warning(timestamp, alert_count):
    """Log warning event (YELLOW ZONE)"""
    print(f"{BOLD}{YELLOW}[{timestamp}] WARNING: alertCount={alert_count} >= {sensorThreshold}{RESET}")


def log_normal(timestamp, alert_count):
    """Log normal status (GREEN ZONE)"""
    print(f"{BOLD}{GREEN}[{timestamp}] NORMAL: CV_flag=0, alertCount={alert_count} < {sensorThreshold}{RESET}")


def print_zone_status(zone_output):
    """Print zone status with color-coded header"""
    if zone_output == 2:
        print(f"{BOLD}{RED}{'='*60}")
        print(f"  RED ZONE - LANDSLIDE DANGER! EVACUATE IMMEDIATELY!")
        print(f"{'='*60}{RESET}\n")
    elif zone_output == 1:
        print(f"{BOLD}{YELLOW}{'='*60}")
        print(f"  YELLOW ZONE - WARNING! MONITOR CONDITIONS CLOSELY")
        print(f"{'='*60}{RESET}\n")
    else:
        print(f"{BOLD}{GREEN}{'='*60}")
        print(f"  GREEN ZONE - ALL SYSTEMS NORMAL")
        print(f"{'='*60}{RESET}\n")


# ==============================================================================
# BUZZER CONTROL FUNCTIONS
# ==============================================================================

def init_buzzer():
    """Initialize piezo buzzer GPIO pin"""
    global gpio_handle
    gpio_handle = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(gpio_handle, BUZZER_PIN)
    lgpio.gpio_write(gpio_handle, BUZZER_PIN, 0)  # Start with buzzer off
    lgpio.gpio_claim_output(gpio_handle, LED_YELLOW_PIN)
    lgpio.gpio_write(gpio_handle, LED_YELLOW_PIN, 0)  # Start with LED off
    print("✓ Buzzer initialized on GPIO 23")
    print("✓ Yellow LED initialized on GPIO 24")


def play_tone(frequency, duration):
    """
    Generate a tone at specified frequency for given duration
    Uses software PWM to drive piezo buzzer
    """
    global gpio_handle
    
    if gpio_handle is None or frequency == 0:
        time.sleep(duration)
        return
    
    period = 1.0 / frequency
    half_period = period / 2.0
    cycles = int(duration * frequency)
    
    for _ in range(cycles):
        if not alarm_active:  # Check if alarm should stop
            break
        lgpio.gpio_write(gpio_handle, BUZZER_PIN, 1)
        time.sleep(half_period)
        lgpio.gpio_write(gpio_handle, BUZZER_PIN, 0)
        time.sleep(half_period)


def alarm_cycle():
    """
    Continuous alarm pattern - runs in background thread
    Produces two-tone siren effect
    """
    global alarm_active, gpio_handle
    
    while alarm_active:
        # Two-tone siren pattern
        play_tone(1000, 0.4)  # Low tone
        if not alarm_active:
            break
        play_tone(2000, 0.4)  # High tone
        if not alarm_active:
            break
        
        # Short pause between cycles
        time.sleep(0.1)
    
    # Ensure buzzer is off when stopped
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, BUZZER_PIN, 0)


def start_alarm():
    """Start the alarm in a background thread"""
    global alarm_active, alarm_thread
    
    if alarm_active:
        return  # Already running
    
    alarm_active = True
    alarm_thread = threading.Thread(target=alarm_cycle, daemon=True)
    alarm_thread.start()
    print(f"{RED}>>> ALARM STARTED <<<{RESET}")


def stop_alarm():
    """Stop the alarm and ensure buzzer is off"""
    global alarm_active, alarm_thread, gpio_handle
    
    if not alarm_active:
        return  # Already stopped
    
    alarm_active = False
    
    # Wait for alarm thread to finish
    if alarm_thread:
        alarm_thread.join(timeout=1.0)
    
    # Ensure buzzer is off
    if gpio_handle:
        lgpio.gpio_write(gpio_handle, BUZZER_PIN, 0)
    
    print(f"{GREEN}>>> ALARM STOPPED <<<{RESET}")


# ==============================================================================
# PERIODIC RISK ASSESSMENT
# ==============================================================================

def risk_assessment_loop():
    """
    Periodic risk assessment loop - runs every 10 seconds
    Evaluates sensor states and updates zone/buzzer accordingly
    """
    global current_zone
    
    while True:
        time.sleep(10)  # Wait 10 seconds between assessments
        
        # Perform risk assessment
        zone_output, buzzer_state = assess_risk()
        
        # Update zone status if changed
        if zone_output != current_zone:
            current_zone = zone_output
            print_zone_status(zone_output)
            
            # Control buzzer based on zone
            if buzzer_state:
                start_alarm()
            else:
                stop_alarm()

            # Control yellow LED
            if gpio_handle:
                if zone_output == 1:   # YELLOW — LED on
                    lgpio.gpio_write(gpio_handle, LED_YELLOW_PIN, 1)
                else:                  # GREEN or RED — LED off
                    lgpio.gpio_write(gpio_handle, LED_YELLOW_PIN, 0)


# ==============================================================================
# MAIN PROGRAM
# ==============================================================================

def main():
    """Main program entry point"""
    global gpio_handle, CV_flag, alertCount
    
    # Print startup banner
    print("=" * 60)
    print("  LANDSLIDE DETECTION SYSTEM - Multi-Modal Risk Assessment")
    print("  Raspberry Pi 5 Receiver")
    print("=" * 60)
    print(f"\nRisk Assessment Parameters:")
    print(f"  τ_sensor (threshold) = {sensorThreshold}")
    print(f"  Assessment interval = 10 seconds")
    print(f"\nExpected Messages:")
    print(f"  CV Transmitter: C0, C1")
    print(f"  Sensor Transmitter: S0, S1, S2, S3, S4")
    print("=" * 60)
    
    # Initialize SPI for LoRa communication
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 500000
    spi.mode = 0
    
    try:
        # Initialize hardware
        init_buzzer()
        init_lora(spi)
        
        # Enter continuous RX mode
        write_reg(spi, REG_OP_MODE, MODE_RX_CONTINUOUS | MODE_LONG_RANGE)
        
        # Start risk assessment thread
        assessment_thread = threading.Thread(target=risk_assessment_loop, daemon=True)
        assessment_thread.start()
        
        # System ready
        print("\n✓ System ready - Listening for sensor data...\n")
        print(f"Current Status: CV_flag={CV_flag}, alertCount={alertCount}")
        print("-" * 60)
        
        packet_count = 0
        
        # Main reception loop
        while True:
            packet = receive_packet(spi)
            
            if packet:
                message, rssi = packet
                packet_count += 1
                
                # Parse and update sensor state
                if not parse_message(message):
                    # Unknown message format
                    print(f"[{packet_count}] Unknown message: '{message}' (RSSI: {rssi} dBm)")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print(f"\n\n{BOLD}System stopped by user{RESET}")
        print(f"Total packets received: {packet_count}")
        print(f"Final state: CV_flag={CV_flag}, alertCount={alertCount}")
    
    except Exception as e:
        print(f"\n{RED}Error: {e}{RESET}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        stop_alarm()
        if gpio_handle:
            lgpio.gpio_write(gpio_handle, LED_YELLOW_PIN, 0)  # Ensure LED off
            lgpio.gpiochip_close(gpio_handle)
        spi.close()
        print("Cleanup complete")


# ==============================================================================
# PROGRAM ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
    main()
