#!/usr/bin/env python3
"""
Voice-controlled teleoperation for Companio
Commands: forward, left, right, reverse/backward, stop, over/exit, speed up, speed down
Runs continuously until 'over' or 'exit' command
"""

import rospy
from geometry_msgs.msg import Twist
import speech_recognition as sr
import subprocess
import os
import sys

class VoiceTeleop:
    def __init__(self):
        rospy.init_node('voice_teleop', anonymous=True)
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Speed settings
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # Speech recognizer
        self.recognizer = sr.Recognizer()
        
        # Audio device settings (SIPEED Mic)
        self.audio_device = 'plughw:2,0'
        self.sample_rate = 16000
        self.record_duration = 3
        
        # Temp audio file
        self.audio_file = '/tmp/voice_command.wav'
        
        # Running flag
        self.running = True
        
        rospy.loginfo("Voice Teleop initialized!")
        rospy.loginfo("Commands: forward, left, right, reverse/backward, stop, speed up, speed down, over/exit")
        rospy.loginfo(f"Linear speed: {self.linear_speed} m/s, Angular speed: {self.angular_speed} rad/s")
    
    def record_audio(self):
        """Record audio using arecord from SIPEED Mic"""
        try:
            rospy.loginfo("🎤 Recording... SPEAK NOW!")
            
            # Record using arecord
            result = subprocess.run([
                'arecord',
                '-D', self.audio_device,
                '-f', 'S16_LE',
                '-r', str(self.sample_rate),
                '-c', '1',
                '-d', str(self.record_duration),
                self.audio_file
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                rospy.loginfo("✓ Recording complete!")
                return True
            else:
                rospy.logerr(f"Recording failed: {result.stderr}")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error recording audio: {e}")
            return False
    
    def recognize_command(self):
        """Recognize voice command from recorded audio"""
        try:
            # Record audio
            if not self.record_audio():
                return None
            
            # Check if file exists
            if not os.path.exists(self.audio_file):
                rospy.logerr("Audio file not found!")
                return None
            
            # Recognize speech
            with sr.AudioFile(self.audio_file) as source:
                audio_data = self.recognizer.record(source)
                rospy.loginfo("🔍 Recognizing...")
                
                # Use Google Speech Recognition
                text = self.recognizer.recognize_google(audio_data)
                rospy.loginfo(f"✓ You said: '{text}'")
                
                return text.lower()
                
        except sr.UnknownValueError:
            rospy.logwarn("✗ Could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.logerr(f"✗ Speech recognition error: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"✗ Error: {e}")
            return None
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Published: linear={linear_x:.2f}, angular={angular_z:.2f}")
    
    def process_command(self, command):
        """Process voice command and publish appropriate velocity"""
        if not command:
            return True  # Continue running
        
        # Check for exit commands
        if 'over' in command or 'exit' in command or 'quit' in command:
            rospy.loginfo("🛑 EXIT command received!")
            self.publish_velocity(0.0, 0.0)  # Stop robot
            return False  # Stop running
        
        # Movement commands
        if 'forward' in command or 'ahead' in command or 'go' in command:
            rospy.loginfo("⬆️  Moving FORWARD")
            self.publish_velocity(self.linear_speed, 0.0)
        
        elif 'left' in command or 'turn left' in command:
            rospy.loginfo("⬅️  Turning LEFT")
            self.publish_velocity(0.0, self.angular_speed)
        
        elif 'right' in command or 'turn right' in command:
            rospy.loginfo("➡️  Turning RIGHT")
            self.publish_velocity(0.0, -self.angular_speed)
        
        elif 'reverse' in command or 'backward' in command or 'back' in command:
            rospy.loginfo("⬇️  Moving BACKWARD")
            self.publish_velocity(-self.linear_speed, 0.0)
        
        elif 'stop' in command or 'halt' in command or 'wait' in command:
            rospy.loginfo("🛑 STOP")
            self.publish_velocity(0.0, 0.0)
        
        # Speed control commands
        elif 'speed up' in command or 'faster' in command or 'increase speed' in command:
            self.linear_speed += self.speed_increment
            self.angular_speed += self.speed_increment
            rospy.loginfo(f"⚡ SPEED UP: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
            self.publish_velocity(0.0, 0.0)  # Stop after speed change
        
        elif 'speed down' in command or 'slower' in command or 'decrease speed' in command:
            self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
            self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
            rospy.loginfo(f"🐌 SPEED DOWN: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
            self.publish_velocity(0.0, 0.0)  # Stop after speed change
        
        else:
            rospy.logwarn(f"❓ Unknown command: '{command}'")
        
        return True  # Continue running
    
    def run(self):
        """Main loop - continuously listen for voice commands"""
        rospy.loginfo("="*60)
        rospy.loginfo("🎤 VOICE TELEOP STARTED - Listening for commands...")
        rospy.loginfo("="*60)
        rospy.loginfo("Say: 'forward', 'left', 'right', 'reverse', 'stop'")
        rospy.loginfo("     'speed up', 'speed down', 'over' (to exit)")
        rospy.loginfo("="*60)
        
        rate = rospy.Rate(0.33)  # ~3 seconds per cycle
        
        try:
            while self.running and not rospy.is_shutdown():
                # Listen for command
                command = self.recognize_command()
                
                # Process command
                if not self.process_command(command):
                    break  # Exit command received
                
                rate.sleep()
        
        except KeyboardInterrupt:
            rospy.loginfo("\n🛑 Keyboard interrupt received")
        
        finally:
            # Stop robot before exiting
            rospy.loginfo("Stopping robot...")
            self.publish_velocity(0.0, 0.0)
            
            # Clean up temp file
            if os.path.exists(self.audio_file):
                os.remove(self.audio_file)
            
            rospy.loginfo("Voice Teleop terminated.")

if __name__ == '__main__':
    try:
        voice_teleop = VoiceTeleop()
        voice_teleop.run()
    except rospy.ROSInterruptException:
        pass

