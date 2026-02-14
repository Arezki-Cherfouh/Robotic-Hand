import cv2
import mediapipe as mp
import serial
import time
import sys

# Serial port configuration - Update this to match your Arduino port
# Windows: 'COM3', 'COM4', etc.
# Mac: '/dev/tty.usbmodem14101', '/dev/tty.usbserial', etc.
# Linux: '/dev/ttyACM0', '/dev/ttyUSB0', etc.
SERIAL_PORT = '/dev/ttyACM0'  # Change this to your Arduino port
BAUD_RATE = 9600

class HandDetector:
    def __init__(self):
        # Initialize MediaPipe Hand solution
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
        # Finger tip and base landmarks (MediaPipe indices)
        self.finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky
        self.finger_pips = [2, 6, 10, 14, 18]  # PIP joints for comparison
        
        # Serial connection
        self.serial_conn = None
        
    def connect_arduino(self):
        """Establish serial connection with Arduino"""
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {SERIAL_PORT}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")
            print("Available ports:")
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            for port in ports:
                print(f"  - {port.device}")
            return False
    
    def is_finger_open(self, landmarks, finger_idx):
        """
        Determine if a finger is open (extended) or closed
        Returns True if finger is open, False if closed
        """
        tip = self.finger_tips[finger_idx]
        pip = self.finger_pips[finger_idx]
        
        # Special case for thumb (different orientation)
        if finger_idx == 0:
            # Thumb: compare x-coordinates (horizontal distance)
            return landmarks[tip].x < landmarks[pip].x if landmarks[tip].x < 0.5 else landmarks[tip].x > landmarks[pip].x
        else:
            # Other fingers: compare y-coordinates (vertical distance)
            # Tip should be above PIP joint when finger is extended
            return landmarks[tip].y < landmarks[pip].y
    
    def get_finger_states(self, landmarks):
        """
        Get the state of all 5 fingers
        Returns list of 5 boolean values [thumb, index, middle, ring, pinky]
        True = open, False = closed
        """
        return [self.is_finger_open(landmarks, i) for i in range(5)]
    
    def send_to_arduino(self, finger_states):
        """
        Send finger states to Arduino via serial
        Format: "T,I,M,R,P\n" where each is 0 (closed) or 1 (open)
        Example: "1,1,0,0,0\n" = thumb and index open, rest closed
        """
        if self.serial_conn and self.serial_conn.is_open:
            # Convert boolean to int (1 or 0)
            data = ','.join([str(int(state)) for state in finger_states]) + '\n'
            self.serial_conn.write(data.encode())
    
    def draw_finger_states(self, image, finger_states):
        """Draw finger state indicators on the image"""
        finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
        y_offset = 30
        
        for i, (name, state) in enumerate(zip(finger_names, finger_states)):
            color = (0, 255, 0) if state else (0, 0, 255)  # Green if open, Red if closed
            status = "OPEN" if state else "CLOSED"
            text = f"{name}: {status}"
            cv2.putText(image, text, (10, y_offset + i * 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    def run(self):
        """Main detection loop"""
        # Connect to Arduino
        if not self.connect_arduino():
            print("Running without Arduino connection (display only mode)")
        
        # Initialize webcam
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("Error: Could not open webcam")
            return
        
        print("Hand detection started. Press 'q' to quit.")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame")
                break
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert BGR to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process frame with MediaPipe
            results = self.hands.process(rgb_frame)
            
            # Check if hand detected
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw hand landmarks
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )
                    
                    # Get finger states
                    finger_states = self.get_finger_states(hand_landmarks.landmark)
                    
                    # Display finger states on screen
                    self.draw_finger_states(frame, finger_states)
                    
                    # Send to Arduino
                    self.send_to_arduino(finger_states)
            else:
                # No hand detected
                cv2.putText(frame, "No hand detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Display frame
            cv2.imshow('Hand Gesture Detection', frame)
            
            # Check for quit command
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Cleanup
        cap.release()
        cv2.destroyAllWindows()
        if self.serial_conn:
            self.serial_conn.close()
        self.hands.close()

if __name__ == "__main__":
    print("=" * 50)
    print("Hand Gesture Detection System")
    print("=" * 50)
    print("\nInstructions:")
    print("1. Update SERIAL_PORT variable to match your Arduino port")
    print("2. Upload the Arduino sketch first")
    print("3. Run this script")
    print("4. Show your hand to the webcam")
    print("5. Press 'q' to quit\n")
    
    detector = HandDetector()
    detector.run()