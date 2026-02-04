import os
import sys
import time
import subprocess
import signal
import shutil
import argparse

# --- CONFIGURATION ---
BAG_DIR = "./input_bags"
MAP_DIR = "./output_maps"
LAUNCH_PACKAGE = "midcone_rtabmap"
LAUNCH_FILE = "map_mavros_final.launch.py"

def run_mapping_pipeline(request_id):
    print(f"--- [START] Processing Request ID: {request_id} ---")

    bag_path = os.path.join(BAG_DIR, request_id)
    map_output_path = os.path.abspath(os.path.join(MAP_DIR, f"{request_id}.db"))
    
    # Check if bag exists
    if not os.path.exists(bag_path):
        print(f"[ERROR] Bag path not found: {bag_path}")
        return False

    # Ensure output dir exists
    os.makedirs(MAP_DIR, exist_ok=True)

    print(f"[INFO] Preparing bag data...")
    print(f"[INFO] Launching RTAB-Map...")
    
    # We pass the specific database path so it saves exactly where we want
    launch_cmd = [
        "ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE,
        "use_sim_time:=true",
        "subscribe_odom_info:=false", # Set to true if you added /odom_info to recorder
        f"database_path:={map_output_path}",
        "args:=--delete_db_on_start" # Overwrite if ID exists
    ]
    
    mapping_process = subprocess.Popen(launch_cmd)

    # Give RTAB-Map a few seconds to initialize
    time.sleep(5)

    # 4. PLAY BAG (Blocking Process)
    print(f"[INFO] Playing Rosbag...")
    play_cmd = [
        "ros2", "bag", "play", bag_path,
        "--clock",   # Crucial for Sim Time
        "--rate", "1.0" # Play at 1x speed. Increase to 2.0 or 3.0 for faster processing (if CPU handles it)
    ]

    try:
        # this blocks until the bag finishes playing
        subprocess.run(play_cmd, check=True)
        print("[INFO] Bag playback finished.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to play bag: {e}")
        mapping_process.kill()
        return False

    # 5. WAIT AND SAVE
    # Wait a moment for the last few messages to be processed
    time.sleep(5)
    
    print(f"[INFO] Saving Map and stopping nodes...")
    
    # Send SIGINT (Ctrl+C) to the launch process to trigger orderly shutdown (saving .db)
    mapping_process.send_signal(signal.SIGINT)
    
    # Wait for it to close gracefully
    try:
        mapping_process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        print("[WARN] Process stuck, killing forcefullly...")
        mapping_process.kill()

    # 6. VERIFY RESULT
    if os.path.exists(map_output_path):
        size_mb = os.path.getsize(map_output_path) / (1024 * 1024)
        print(f"--- [SUCCESS] Map saved: {map_output_path} ({size_mb:.2f} MB) ---")
        return True
    else:
        print(f"--- [FAILURE] Map file was not created. ---")
        return False

if __name__ == "__main__":
    # Allow running from command line for testing
    # Usage: python3 process_map.py flight_data_verified
    parser = argparse.ArgumentParser(description="Process Rosbag to Map")
    parser.add_argument("request_id", type=str, help="The ID of the bag folder in input_bags/")
    args = parser.parse_args()

    run_mapping_pipeline(args.request_id)