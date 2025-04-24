/*
 * SerialProtocol.h
 * 
 * This file defines the serial communication protocol for the WAVEGO robot.
 * It standardizes commands and responses to ensure reliable communication
 * with the Raspberry Pi controller.
 */

#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <ArduinoJson.h>

// Command types supported
enum CommandType {
  TEXT_CMD,   // Plain text commands
  JSON_CMD    // JSON formatted commands
};

// Function to send accelerometer data over serial when requested
void sendAccelData() {
  // Update accelerometer data first
  accXYZUpdate();
  
  // Create JSON string with accelerometer data
  String accelJSON = "{\"acc_x\":" + String(ACC_X) + 
                     ",\"acc_y\":" + String(ACC_Y) + 
                     ",\"acc_z\":" + String(ACC_Z) + "}";
                     
  // Send over serial with the expected prefix
  Serial.println("ACCEL_DATA:" + accelJSON);
}

// Function to send gyroscope data over serial when requested
void sendGyroData() {
  // Update gyroscope data
  myIMU.readSensor();
  xyzFloat gyroRaw = myIMU.getGyrRawValues();
  xyzFloat angles = myIMU.getAngles();
  
  // Create JSON string with gyroscope data
  String gyroJSON = "{\"gyro_x\":" + String(gyroRaw.x) + 
                    ",\"gyro_y\":" + String(gyroRaw.y) + 
                    ",\"gyro_z\":" + String(gyroRaw.z) + 
                    ",\"angle_x\":" + String(angles.x) + 
                    ",\"angle_y\":" + String(angles.y) + 
                    ",\"angle_z\":" + String(angles.z) + "}";
                    
  // Send over serial with the expected prefix
  Serial.println("GYRO_DATA:" + gyroJSON);
}

// Function to reset gyroscope calibration
void resetGyro() {
  myIMU.init();
  delay(100);
  myIMU.autoOffsets();
  Serial.println("ACK:GYRO_RESET");
}

// Process a text command
bool processTextCommand(String command) {
  command.trim();  // Remove any whitespace
  
  if (command == "PING") {
    Serial.println("PONG");
    return true;
  }
  else if (command == "GET_ACCEL") {
    sendAccelData();
    return true;
  }
  else if (command == "GET_GYRO") {
    sendGyroData();
    return true;
  }
  else if (command == "RESET_GYRO") {
    resetGyro();
    return true;
  }
  
  // Command not recognized
  return false;
}

// Sends a standard acknowledgment response for JSON commands
void sendAck(String status = "CMD_PROCESSED") {
  Serial.println("ACK:" + status);
}

#endif // SERIAL_PROTOCOL_H
