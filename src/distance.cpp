#include "distance.h"
#include "config.h"
#include "status_led.h"
#include <Wire.h>
#include <vl53l4cx_class.h>

static VL53L4CX sensor(&Wire, -1);
static bool sensorReady = false;

bool distanceInit() {
  Serial.println("[DEBUG] Starting distance sensor init...");
  
  // Power cycle the sensor using XSHUT
  pinMode(XSHUT_PIN_FRONT, OUTPUT);
  digitalWrite(XSHUT_PIN_FRONT, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN_FRONT, HIGH);
  delay(100);  // Give sensor more time to boot up
  
  Serial.println("[DEBUG] XSHUT power cycled");
  
  // Initialize the sensor
  int status = sensor.begin();
  Serial.print("[DEBUG] sensor.begin() returned: ");
  Serial.println(status);
  
  if (status != 0) {
    Serial.println("VL53L4CX sensor not found!");
    statusLedRed();
    return false;
  }
  
  // Don't turn off - directly initialize
  // The VL53L4CX_Off() call was causing the issue
  
  // Initialize sensor - this loads the firmware
  status = sensor.InitSensor(0x29); // Default I2C address
  Serial.print("[DEBUG] InitSensor returned: ");
  Serial.println(status);
  
  if (status != VL53L4CX_ERROR_NONE) {
    Serial.print("VL53L4CX sensor init failed! Status: ");
    Serial.println(status);
    
    // Try a soft reset and reinit
    Serial.println("[DEBUG] Attempting sensor reset...");
    sensor.VL53L4CX_Off();
    delay(10);
    
    status = sensor.InitSensor(0x29);
    Serial.print("[DEBUG] InitSensor retry returned: ");
    Serial.println(status);
    
    if (status != VL53L4CX_ERROR_NONE) {
      Serial.println("VL53L4CX sensor init failed after retry!");
      statusLedRed();
      return false;
    } else {
      statusLedGreen();
    }
  }
  
  // Set distance mode to medium range (up to ~1.3m)
  status = sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  Serial.print("[DEBUG] SetDistanceMode returned: ");
  Serial.println(status);
  
  if (status != VL53L4CX_ERROR_NONE) {
    Serial.print("VL53L4CX set distance mode failed! Status: ");
    Serial.println(status);
    // Continue anyway, sensor might work with default mode
  }
  
  // Set timing budget (measurement time per ranging)
  // Higher = more accurate but slower. 33ms is good balance.
  status = sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);
  Serial.print("[DEBUG] SetTimingBudget returned: ");
  Serial.println(status);
  
  if (status != VL53L4CX_ERROR_NONE) {
    Serial.println("[DEBUG] Timing budget failed, using default");
  }
  
  status = sensor.VL53L4CX_StartMeasurement();
  Serial.print("[DEBUG] StartMeasurement returned: ");
  Serial.println(status);
  
  if (status != VL53L4CX_ERROR_NONE) {
    Serial.print("VL53L4CX sensor start failed! Status: ");
    Serial.println(status);
    statusLedRed();
    return false;
  }
  
  sensorReady = true;
  Serial.println("Distance sensor initialized successfully!");
  statusLedGreen(); // Indicate success
  return true;
}

int getDistanceFront() {
  static int lastValidDistance = -1; // Cache last valid reading
  static bool debugOnce = true;
  static int readCount = 0;
  
  if (!sensorReady) {
    if (debugOnce) {
      Serial.println("[DEBUG] Sensor not ready!");
      debugOnce = false;
    }
    return lastValidDistance;
  }
  
  VL53L4CX_MultiRangingData_t data;
  uint8_t NewDataReady = 0;
  int status;
  
  // Non-blocking check - only read if data is ready
  status = sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  
  if (status == VL53L4CX_ERROR_NONE && NewDataReady) {
    sensor.VL53L4CX_GetMultiRangingData(&data);
    
    // Clear interrupt and start next measurement
    sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    
    readCount++;
    
    // Debug info for first few readings
    if (readCount <= 5) {
      Serial.print("[DEBUG] Read #");
      Serial.print(readCount);
      Serial.print(" - Objects: ");
      Serial.print(data.NumberOfObjectsFound);
      Serial.print(", Status: ");
      Serial.print(data.RangeData[0].RangeStatus);
      Serial.print(", Distance: ");
      Serial.print(data.RangeData[0].RangeMilliMeter);
      Serial.println(" mm");
    }
    
    // Update cached distance if valid (status 0 = good measurement)
    // For medium range, we're less strict - accept measurements with reasonable status
    if (data.NumberOfObjectsFound > 0) {
      // Status 0 = valid range
      // Status 4 = out of bounds (can still use)
      // Status 7 = wrapped target (ignore)
      if (data.RangeData[0].RangeStatus == 0 || data.RangeData[0].RangeStatus == 4) {
        lastValidDistance = data.RangeData[0].RangeMilliMeter;
      }
    }
  }
  
  return lastValidDistance; // Return latest reading (or -1 if never got valid data)
}

bool distanceDataReady() {
  if (!sensorReady) {
    return false;
  }
  
  uint8_t NewDataReady = 0;
  sensor.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  return NewDataReady != 0;
}
