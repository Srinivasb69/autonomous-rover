from flask import Flask, request, jsonify
import serial
import threading
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import StaticTransformBroadcaster
import time
import math

app = Flask(__name__)

# --- Global variables ---
ser = None  # Define the serial connection globally
latest_scan = {}  # Store the latest LiDAR scan data
behavior_mode = "manual"  # Can be "manual", "avoid", or "follow"
target_object = None  # Store information about object being followed

# --- Behavior control variables ---
# Object avoidance parameters
min_distance = 0.5  # Minimum safe distance (meters)
linear_speed = 0.2  # Default linear speed
angular_speed = 0.5  # Default angular speed

# Object following parameters
target_distance = 0.7  # Desired distance to maintain from object (meters)
distance_tolerance = 0.1  # Tolerance for distance
max_detection_distance = 3.0  # Max distance to detect objects
min_object_size = 5  # Minimum consecutive points to consider an object

# --- Arduino thread function ---
def arduino_thread():
    global ser
    while True:
        time.sleep(1)  # Check every second
        try:
            if ser is None or not ser.is_open:
                try:
                    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                    print("✅ Serial connection to Arduino re-established.")
                except Exception as e:
                    # Silent failure, will retry on next loop
                    pass
        except Exception as e:
            print(f"⚠️ Arduino thread error: {e}")

# --- Start Arduino monitoring thread ---
arduino_monitoring = threading.Thread(target=arduino_thread, daemon=True)
arduino_monitoring.start()

# --- LiDAR Node Setup using ROS 2 ---
class ScanListener(Node):
    def __init__(self):
        super().__init__('scan_listener')
        
        # Create subscription to scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
            
        # Create a static transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transform once
        self.publish_static_transform()
        
        # Create timer for behavior processing
        self.behavior_timer = self.create_timer(0.1, self.process_behavior)
        
    def publish_static_transform(self):
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'laser'
        
        # Set translation (x, y, z)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        
        # Set rotation (quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        print("✅ Published static transform from base_link to laser")

    def listener_callback(self, msg):
        global latest_scan
        latest_scan = {
            'ranges': list(msg.ranges),
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'time_increment': float(msg.time_increment),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max)
        }
    
    def process_behavior(self):
        """Process the current behavior mode based on latest scan data"""
        global behavior_mode, target_object
        
        if behavior_mode == "manual" or not latest_scan or len(latest_scan.get('ranges', [])) == 0:
            # No automatic control in manual mode or if no scan data
            return
            
        if behavior_mode == "avoid":
            self.handle_obstacle_avoidance()
        elif behavior_mode == "follow":
            self.handle_object_following()
    
    def handle_obstacle_avoidance(self):
        """Process obstacle avoidance behavior"""
        global latest_scan
        
        ranges = latest_scan.get('ranges', [])
        if not ranges:
            return
            
        total_angles = len(ranges)
        # Define front, left, and right regions
        front_start = int(total_angles * 0.95)  # -18 degrees
        front_end = int(total_angles * 0.05)    # +18 degrees
        
        left_start = int(total_angles * 0.05)   # +18 degrees
        left_end = int(total_angles * 0.25)     # +90 degrees
        
        right_start = int(total_angles * 0.75)  # -90 degrees
        right_end = int(total_angles * 0.95)    # -18 degrees
        
        # Get ranges for each region
        front_ranges = [ranges[i] for i in range(front_start, total_angles)] + [ranges[i] for i in range(0, front_end)]
        left_ranges = [ranges[i] for i in range(left_start, left_end)]
        right_ranges = [ranges[i] for i in range(right_start, right_end)]
        
        # Filter out invalid values and get minimums
        front_ranges = [r for r in front_ranges if isinstance(r, float) and not math.isinf(r) and not math.isnan(r)]
        left_ranges = [r for r in left_ranges if isinstance(r, float) and not math.isinf(r) and not math.isnan(r)]
        right_ranges = [r for r in right_ranges if isinstance(r, float) and not math.isinf(r) and not math.isnan(r)]
        
        front_min = min(front_ranges) if front_ranges else float('inf')
        left_min = min(left_ranges) if left_ranges else float('inf')
        right_min = min(right_ranges) if right_ranges else float('inf')
        
        # Obstacle avoidance logic
        command = ""
        if front_min < min_distance:
            # Obstacle in front, need to turn
            if left_min > right_min:
                command = "L"  # Turn left
                print("🔄 Obstacle ahead, turning left")
            else:
                command = "R"  # Turn right
                print("🔄 Obstacle ahead, turning right")
        else:
            # No obstacle in front, move forward with adjustments
            if left_min < min_distance:
                command = "FR"  # Slight right
                print("➡️ Obstacle on left, adjusting right")
            elif right_min < min_distance:
                command = "FL"  # Slight left
                print("⬅️ Obstacle on right, adjusting left")
            else:
                command = "F"  # Forward
                print("⬆️ Path clear, moving forward")
        
        # Send command to Arduino
        send_arduino_command(command)
    
    def handle_object_following(self):
        """Process object following behavior"""
        global latest_scan, target_object
        
        ranges = latest_scan.get('ranges', [])
        if not ranges:
            return
            
        angle_min = latest_scan.get('angle_min', 0)
        angle_increment = latest_scan.get('angle_increment', 0)
        
        # Find object to follow
        object_distance, object_angle = self.find_closest_object(ranges, angle_min, angle_increment)
        
        if object_distance is None:
            # No object found, search by rotating
            send_arduino_command("L")
            print("🔍 Searching for object...")
            return
            
        # Update target object info
        target_object = {
            "distance": object_distance,
            "angle": object_angle,
            "angle_degrees": math.degrees(object_angle)
        }
        
        # Follow the object
        distance_error = object_distance - target_distance
        
        # Determine command based on position of the object
        command = ""
        angle_degrees = math.degrees(object_angle)
        
        if abs(distance_error) < distance_tolerance:
            # At correct distance
            if abs(angle_degrees) < 10:
                command = "S"  # Stop
                print(f"⏹️ Tracking object at {object_distance:.2f}m")
            elif angle_degrees > 0:
                command = "L"  # Turn left to face object
                print(f"↩️ Turning left to face object at {angle_degrees:.1f}°")
            else:
                command = "R"  # Turn right to face object
                print(f"↪️ Turning right to face object at {angle_degrees:.1f}°")
        else:
            # Need to adjust distance
            if distance_error > 0:
                # Too far, move closer
                if abs(angle_degrees) < 15:
                    command = "F"  # Move forward
                    print(f"⬆️ Moving closer to object at {object_distance:.2f}m")
                elif angle_degrees > 0:
                    command = "FL"  # Forward-left
                    print(f"↖️ Moving closer-left to object at {angle_degrees:.1f}°")
                else:
                    command = "FR"  # Forward-right
                    print(f"↗️ Moving closer-right to object at {angle_degrees:.1f}°")
            else:
                # Too close, move back
                if abs(angle_degrees) < 15:
                    command = "B"  # Move backward
                    print(f"⬇️ Moving away from object at {object_distance:.2f}m")
                elif angle_degrees > 0:
                    command = "BL"  # Backward-left
                    print(f"↙️ Moving away-left from object at {angle_degrees:.1f}°")
                else:
                    command = "BR"  # Backward-right
                    print(f"↘️ Moving away-right from object at {angle_degrees:.1f}°")
        
        # Send command to Arduino
        send_arduino_command(command)
    
    def find_closest_object(self, ranges, angle_min, angle_increment):
        """Find an object to follow in the scan data"""
        # Find consecutive points that could be an object
        objects = []
        current_object = []
        
        for i, r in enumerate(ranges):
            # Check if point is within detection range
            if r < max_detection_distance and not math.isinf(r) and not math.isnan(r):
                current_object.append((i, r))
            elif len(current_object) > 0:
                # End of an object
                if len(current_object) >= min_object_size:
                    objects.append(current_object)
                current_object = []
        
        # Check if we have an object at the end of the array
        if len(current_object) >= min_object_size:
            objects.append(current_object)
            
        if not objects:
            return None, None
            
        # Find the largest object (most points)
        largest_object = max(objects, key=len)
        
        # Calculate center of the object
        avg_idx = sum(i for i, _ in largest_object) / len(largest_object)
        avg_range = sum(r for _, r in largest_object) / len(largest_object)
        
        # Convert index to angle in radians
        angle = angle_min + (avg_idx * angle_increment)
        
        return avg_range, angle

def start_ros2_node():
    rclpy.init()
    node = ScanListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# --- Start ROS 2 in Background Thread ---
ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
ros_thread.start()
print("📡 ROS 2 node for LiDAR started.")

def send_arduino_command(command):
    """Send command to Arduino through serial"""
    global ser
    if not ser or not ser.is_open:
        try:
            ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            print("✅ Serial connection to Arduino established.")
        except serial.SerialException:
            print("❌ Could not establish serial connection for command:", command)
            return False
    
    try:
        ser.write_timeout = 1.0
        ser.write((command + '\n').encode())
        return True
    except Exception as e:
        print(f"❌ Error sending command to Arduino: {e}")
        try:
            ser.close()
        except:
            pass
        ser = None
        return False

def start_lidar():
    # First check if LiDAR is already running
    ps_output = subprocess.run(
        "ps -ef | grep rplidar | grep -v grep", 
        shell=True, 
        stdout=subprocess.PIPE, 
        stderr=subprocess.PIPE
    )
    
    if ps_output.returncode == 0:
        print("🔄 LiDAR process already running, killing existing process...")
        # Kill existing LiDAR processes
        subprocess.run(
            "pkill -f rplidar",
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(1)  # Give it time to shut down
    
    print("🚀 Launching LiDAR...")
    command = """
    bash -c '
    source ~/ros2_ws/install/setup.bash
    ros2 launch rplidar_ros view_rplidar_a1_launch.py
    '
    """
    
    # Run the command in the background using nohup
    with open("lidar_output.log", "w") as log_file:
        subprocess.Popen(
            command,
            shell=True,
            stdout=log_file,
            stderr=log_file,
            executable='/bin/bash'
        )
    
    print("✅ LiDAR process started")
    time.sleep(2)  # Give the LiDAR time to initialize

# --- Flask Endpoints ---
@app.route('/')
def index():
    return "🤖 Rover Backend is Online."

@app.route('/command', methods=['POST'])
def send_command():
    global ser, behavior_mode, target_object, latest_scan
    
    try:
        json_data = request.get_json()
        command = json_data.get('command', '').strip().upper()
        print(f"➡ Command received: {command}")
        
        # Check LiDAR status before changing modes
        lidar_active = len(latest_scan.get('ranges', [])) > 0
        if not lidar_active and command in ["AVOID", "FOLLOW"]:
            # Attempt to restart LiDAR if it's down and we need it
            print("⚠️ LiDAR appears to be offline. Attempting to restart...")
            restart_lidar_thread = threading.Thread(target=start_lidar, daemon=True)
            restart_lidar_thread.start()
            time.sleep(2)  # Give LiDAR a moment to start
        
        # Check for behavior control commands
        if command == "AVOID":
            behavior_mode = "avoid"
            # Reset the system first to ensure clean state
            send_arduino_command('RESET')
            send_arduino_command('E')  # Enable motors
            return jsonify({"status": "success", "message": "Obstacle avoidance mode activated"}), 200
        elif command == "FOLLOW":
            behavior_mode = "follow"
            target_object = None  # Reset target object
            # Reset the system first to ensure clean state
            send_arduino_command('RESET')
            send_arduino_command('E')  # Enable motors
            return jsonify({"status": "success", "message": "Object following mode activated"}), 200
        elif command == "STOP":
            # Stop all autonomous functions
            behavior_mode = "manual"
            target_object = None
            
            # First stop the motors
            send_arduino_command('S')
            time.sleep(0.1)  # Small delay
            
            # Then do a full reset
            send_arduino_command('RESET')
            time.sleep(0.1)  # Small delay
            
            # Make sure motors are enabled for future commands
            send_arduino_command('E')
            
            print("🛑 Rover stopped and reset to manual mode")
            return jsonify({"status": "success", "message": "Stopped all autonomous functions"}), 200
        elif command == "MANUAL":
            behavior_mode = "manual"
            return jsonify({"status": "success", "message": "Manual control mode activated"}), 200
        
        # For manual control, send commands directly to Arduino
        if behavior_mode == "manual":
            if not ser or not ser.is_open:
                try:
                    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
                    print("✅ Serial connection to Arduino established.")
                except serial.SerialException as e:
                    return jsonify({"status": "error", "message": "Serial port not available"}), 500
            
            if send_arduino_command(command):
                return jsonify({"status": "success", "message": "Command sent"}), 200
            else:
                return jsonify({"status": "error", "message": "Failed to send command"}), 500
        else:
            return jsonify({"status": "error", "message": f"Cannot send manual commands in {behavior_mode} mode"}), 400
    except Exception as e:
        print(f"❌ Error processing command: {e}")
        # Try to reconnect next time
        try:
            if ser:
                ser.close()
        except:
            pass
        ser = None
        return jsonify({"status": "error", "message": "Command failed"}), 400

@app.route('/restart_lidar', methods=['POST'])
def restart_lidar():
    try:
        start_lidar()
        return jsonify({"status": "success", "message": "LiDAR restart initiated"}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": f"Failed to restart LiDAR: {str(e)}"}), 500

@app.route('/scan', methods=['GET'])
def get_scan():
    if latest_scan and len(latest_scan.get('ranges', [])) > 0:
        # Replace non-finite values like inf, -inf, nan with a large number or 0
        cleaned_ranges = [
            r if isinstance(r, float) and r not in [float('inf'), float('-inf')] and not (r != r) else 0.0
            for r in latest_scan['ranges']
        ]
        
        # Downsample if needed
        if len(cleaned_ranges) > 360:
            step = len(cleaned_ranges) // 360
            ranges = cleaned_ranges[::step][:360]
            angle_increment = latest_scan['angle_increment'] * step
        else:
            ranges = cleaned_ranges
            angle_increment = latest_scan['angle_increment']
        
        return jsonify({
            'ranges': ranges,
            'angle_min': latest_scan['angle_min'],
            'angle_max': latest_scan['angle_max'],
            'angle_increment': angle_increment,
            'time_increment': latest_scan['time_increment'],
            'range_min': latest_scan['range_min'],
            'range_max': latest_scan['range_max']
        })
    else:
        return jsonify({'error': 'No LiDAR data available yet'}), 503

@app.route('/status', methods=['GET'])
def status():
    global behavior_mode, target_object
    
    lidar_status = len(latest_scan.get('ranges', [])) > 0
    arduino_status = ser is not None and ser.is_open
    
    status_data = {
        'arduino': "connected" if arduino_status else "disconnected",
        'lidar': "receiving data" if lidar_status else "no data",
        'mode': behavior_mode,
        'timestamp': time.time()
    }
    
    # Add target information if we're following an object
    if behavior_mode == "follow" and target_object:
        status_data['target'] = target_object
    
    return jsonify(status_data)

@app.route('/health', methods=['GET'])
def health_check():
    lidar_status = len(latest_scan.get('ranges', [])) > 0
    arduino_status = ser is not None and ser.is_open
    
    return jsonify({
        'arduino': "connected" if arduino_status else "disconnected",
        'lidar': "receiving data" if lidar_status else "no data",
        'timestamp': time.time()
    })

@app.route('/settings', methods=['POST'])
def update_settings():
    global min_distance, target_distance, distance_tolerance, max_detection_distance, min_object_size
    
    try:
        json_data = request.get_json()
        
        # Update object avoidance parameters
        if 'min_distance' in json_data:
            min_distance = float(json_data['min_distance'])
        
        # Update object following parameters
        if 'target_distance' in json_data:
            target_distance = float(json_data['target_distance'])
        if 'distance_tolerance' in json_data:
            distance_tolerance = float(json_data['distance_tolerance'])
        if 'max_detection_distance' in json_data:
            max_detection_distance = float(json_data['max_detection_distance'])
        if 'min_object_size' in json_data:
            min_object_size = int(json_data['min_object_size'])
        
        return jsonify({
            'status': 'success',
            'settings': {
                'min_distance': min_distance,
                'target_distance': target_distance,
                'distance_tolerance': distance_tolerance,
                'max_detection_distance': max_detection_distance,
                'min_object_size': min_object_size
            }
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 400

@app.route('/stop', methods=['POST'])
def stop_rover():
    global behavior_mode, target_object
    
    behavior_mode = "manual"  # Reset behavior to manual
    target_object = None  # Clear any tracked object
    
    # First stop the motors
    send_arduino_command('S')
    time.sleep(0.1)  # Small delay
    
    # Then do a full reset
    send_arduino_command('RESET')
    time.sleep(0.1)  # Small delay
    
    # Make sure motors are enabled for future commands
    send_arduino_command('E')
    
    print("🛑 Rover stopped and reset to manual mode")
    return jsonify({"status": "stopped", "mode": "manual"})

# --- Start Flask Server ---
if __name__ == '__main__':
    start_lidar()
    print("✅ Flask server starting on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000)
