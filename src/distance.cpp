#include "distance.h"
#include <Wire.h>
#include <vl53l4cx_class.h>
#include "config.h"


TwoWire I2CBus0 = TwoWire(0);
TwoWire I2CBus1 = TwoWire(1);

// Left is alone on Bus 1
VL53L4CX sensorLeft(&I2CBus1, XSHUT_PIN_LEFT); 

// Center and Right share Bus 0
VL53L4CX sensorCenter(&I2CBus0, XSHUT_PIN_CENTER);
VL53L4CX sensorRight(&I2CBus0, XSHUT_PIN_RIGHT);

// EMA smoothing factor — sensor fires at ~30Hz (33ms budget).
// 0.2 → tau ≈ 132ms, enough to reject single-sample spikes without too much lag.
#define EMA_ALPHA 0.2f

// Stored as float internally so the EMA isn't quantized; exposed as int.
static float left_filtered   = -1.0f;
static float center_filtered = -1.0f;
static float right_filtered  = -1.0f;

// Latest unfiltered center reading — used for collision abort to bypass EMA lag.
static volatile int center_raw = -1;

static bool left_ready   = false;
static bool center_ready = false;
static bool right_ready  = false;

static bool initSingleSensor(VL53L4CX &sensor, uint8_t address, const char *name)
{
  sensor.begin(); 

  if (address != SENSOR_DEFAULT_ADDR) {
    sensor.VL53L4CX_SetDeviceAddress(address);
  }

  if (sensor.InitSensor(address) != VL53L4CX_ERROR_NONE) {
    Serial.printf("[ToF] %s init failed!\n", name);
    return false;
  }

  sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000); // 33ms updates
  sensor.VL53L4CX_StartMeasurement();
  
  Serial.printf("[ToF] %s ready at address 0x%02X\n", name, address);
  return true;
}


static void readSingleSensor(VL53L4CX &sensor, float &filtered, volatile int* rawOut = nullptr)
{
  uint8_t dataReady = 0;
  sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);

  if (!dataReady) return;

  VL53L4CX_MultiRangingData_t data;
  sensor.VL53L4CX_GetMultiRangingData(&data);
  sensor.VL53L4CX_ClearInterruptAndStartMeasurement();

  if (data.NumberOfObjectsFound <= 0) return;

  // VL53L4CX is a multi-target sensor — RangeData[0] is NOT guaranteed to be
  // the nearest object. Scan all detected targets and take the closest valid one.
  int bestRaw = -1;
  for (int i = 0; i < data.NumberOfObjectsFound; i++) {
    if (data.RangeData[i].RangeStatus != 0) continue;  // status 4 = sigma fail, produces garbage/negative values
    int r = data.RangeData[i].RangeMilliMeter;
    if (r < 1 || r > 6000) continue;
    if (bestRaw < 0 || r < bestRaw) bestRaw = r;
  }
  if (bestRaw < 0) return;  // no valid target this cycle

  if (rawOut) *rawOut = bestRaw;

  // Seed the filter on first valid reading instead of blending from -1
  if (filtered < 0.0f) {
    filtered = (float)bestRaw;
  } else {
    filtered = EMA_ALPHA * (float)bestRaw + (1.0f - EMA_ALPHA) * filtered;
  }
}

// API
bool distanceInit()
{
  Serial.println("[ToF] Booting I2C Buses...");
  
  I2CBus0.begin(SDA_PIN_0, SCL_PIN_0);
  I2CBus0.setClock(400000); // 400kHz Fast Mode
  
  I2CBus1.begin(SDA_PIN_1, SCL_PIN_1);
  I2CBus1.setClock(400000);

  // All sensors start in reset
  pinMode(XSHUT_PIN_LEFT, OUTPUT);
  pinMode(XSHUT_PIN_CENTER, OUTPUT);
  pinMode(XSHUT_PIN_RIGHT, OUTPUT);
  
  digitalWrite(XSHUT_PIN_LEFT, LOW);
  digitalWrite(XSHUT_PIN_CENTER, LOW);
  digitalWrite(XSHUT_PIN_RIGHT, LOW);
  delay(20);

  // 2. Boot Center first
  digitalWrite(XSHUT_PIN_CENTER, HIGH);
  delay(50); // Wait for boot
  center_ready = initSingleSensor(sensorCenter, SENSOR_CENTER_ADDR, "Center");

  // 3. Boot Right next (Center is now at 0x2A, so 0x29 is free for Right)
  digitalWrite(XSHUT_PIN_RIGHT, HIGH);
  delay(50);
  right_ready = initSingleSensor(sensorRight, SENSOR_DEFAULT_ADDR, "Right");

  // 4. Boot Left (It's alone on Bus 1, so no address conflicts)
  digitalWrite(XSHUT_PIN_LEFT, HIGH);
  delay(50);
  left_ready = initSingleSensor(sensorLeft, SENSOR_DEFAULT_ADDR, "Left");

  return (left_ready && center_ready && right_ready);
}

void distanceUpdateAll()
{
  if (left_ready)   readSingleSensor(sensorLeft,   left_filtered);
  if (center_ready) readSingleSensor(sensorCenter, center_filtered, &center_raw);
  if (right_ready)  readSingleSensor(sensorRight,  right_filtered);
}

// Returns -1 if no valid reading has arrived yet
int getDistanceLeft()     { return (int)left_filtered;   }
int getDistanceFront()    { return (int)center_filtered; }
int getDistanceFrontRaw() { return (int)center_raw;      }
int getDistanceRight()    { return (int)right_filtered;  }