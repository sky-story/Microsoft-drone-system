#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Drone Control System GUI - Demo Mode
Test the interface without a real drone
"""

import time
import threading
import random
import numpy as np
import cv2
from flask import jsonify, request
from drone_control_gui import (
    app, control_system,
    SystemState
)


def simulate_drone_connection():
    """Simulate drone connection"""
    print("🔗 Simulating drone connection...")
    control_system.drone_status.connected = True
    control_system.drone_status.battery = 85
    control_system.drone_status.satellites = 15
    control_system.drone_status.gps_fix = True
    control_system.drone_status.latitude = 47.6218425
    control_system.drone_status.longitude = -122.1769126
    control_system.log("✅ Demo drone connected", "success")


def simulate_video_feed():
    """Generate fake video frames for demo"""
    frame_count = 0
    while control_system.drone_status.connected:
        # Create a test pattern frame
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # Moving gradient background
        t = frame_count * 0.02
        for y in range(720):
            for x in range(0, 1280, 4):
                b = int(30 + 20 * np.sin(t + x * 0.01))
                g = int(30 + 20 * np.sin(t + y * 0.01 + 1))
                r = int(50 + 20 * np.sin(t + (x + y) * 0.005 + 2))
                frame[y, x:x+4] = [b, g, r]
        
        # Add text overlay
        cv2.putText(frame, "DEMO MODE - No Drone Connected", (350, 340),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 200, 255), 2)
        cv2.putText(frame, f"Frame: {frame_count}", (550, 400),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 1)
        
        # Draw a fake crosshair
        cx, cy = 640, 360
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 1)
        cv2.circle(frame, (cx, cy), 20, (0, 255, 0), 1)
        
        with control_system.frame_lock:
            control_system.current_frame = frame
        
        frame_count += 1
        time.sleep(0.033)  # ~30fps


def simulate_mission():
    """Simulate full mission"""
    time.sleep(5)
    print("\n🚀 Starting demo mission...\n")
    
    # Takeoff
    control_system.drone_status.flying = True
    control_system.log("✈️ Simulated takeoff", "success")
    time.sleep(2)
    
    # Navigation
    control_system.navigation_status.state = SystemState.RUNNING.value
    control_system.log("🧭 Simulating GPS navigation", "info")
    
    for progress in range(0, 101, 5):
        control_system.navigation_status.progress = progress
        control_system.navigation_status.current_distance = 100 * (1 - progress / 100)
        control_system.navigation_status.message = f"Flying... {progress}%"
        time.sleep(0.5)
    
    control_system.navigation_status.state = SystemState.COMPLETED.value
    control_system.log("✅ Navigation complete", "success")
    time.sleep(2)
    
    # Object tracking
    control_system.perception_status.state = SystemState.RUNNING.value
    control_system.perception_status.detected = True
    control_system.log("👁️ Simulating object tracking", "info")
    
    for i in range(30):
        control_system.perception_status.tracking = True
        control_system.perception_status.confidence = 0.8 + random.random() * 0.15
        control_system.perception_status.message = f"Tracking... {i+1}/30"
        time.sleep(0.3)
    
    control_system.perception_status.stable = True
    control_system.log("✅ Target locked, triggering winch", "success")
    time.sleep(2)
    
    # Winch
    control_system.winch_status.state = SystemState.RUNNING.value
    control_system.winch_status.current_action = "LOWERING"
    control_system.log("⬇️ LOWER command sent", "info")
    time.sleep(3)
    
    control_system.winch_status.current_action = "PULLING"
    control_system.log("⬆️ PULL command sent", "info")
    time.sleep(3)
    
    control_system.winch_status.current_action = "STOP"
    control_system.log("⏹️ STOP command sent", "success")
    time.sleep(1)
    
    control_system.winch_status.state = SystemState.COMPLETED.value
    control_system.perception_status.state = SystemState.COMPLETED.value
    control_system.log("🎉 Mission complete!", "success")


def update_battery():
    """Simulate battery drain"""
    while control_system.drone_status.connected:
        if control_system.drone_status.flying:
            control_system.drone_status.battery = max(
                0, 
                control_system.drone_status.battery - random.random() * 0.1
            )
        time.sleep(5)


def main():
    """Demo mode main"""
    print("=" * 60)
    print("  Drone Control System - Web GUI (Demo Mode)")
    print("=" * 60)
    print()
    print("  🎭 Demo mode:")
    print("  - No real drone needed")
    print("  - Simulated mission flow")
    print("  - UI testing")
    print()
    print("  URL: http://0.0.0.0:5000")
    print("  Press Ctrl+C to stop")
    print("=" * 60)
    print()
    
    # Simulate connection
    simulate_drone_connection()
    
    # Start fake video feed
    threading.Thread(target=simulate_video_feed, daemon=True).start()
    
    # Start battery simulation
    threading.Thread(target=update_battery, daemon=True).start()
    
    # Start demo mission after delay
    threading.Thread(target=simulate_mission, daemon=True).start()
    
    # Run Flask (no SocketIO)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()
