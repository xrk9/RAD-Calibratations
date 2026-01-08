#include <Wire.h>
#define MPU_ADDR 0x68

float ax,ay,az,gx,gy,gz;
float ax_offset, ay_offset, az_offset,gx_offset,gy_offset,gz_offset;
float raw_ax,raw_ay,raw_az,raw_gx,raw_gy,raw_gz;

void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readMPUraw() {
  // Start reading at ACCEL_XOUT_H (0x3B) and request 14 bytes
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false; // non-zero = error

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (available < 14) return false;

  // Read as unsigned and combine to signed int16_t safely
  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_ax = (int16_t)((hi << 8) | lo) - 151 + 14;
  hi = Wire.read(); lo = Wire.read(); raw_ay = (int16_t)((hi << 8) | lo) + 159 + 23 - 1;
  hi = Wire.read(); lo = Wire.read(); raw_az = (int16_t)((hi << 8) | lo) - 8192 + 1346;
  // temp high/low (skip)
  (void)Wire.read(); (void)Wire.read();
  hi = Wire.read(); lo = Wire.read(); raw_gx = (int16_t)((hi << 8) | lo) + 148 + 3;
  hi = Wire.read(); lo = Wire.read(); raw_gy = (int16_t)((hi << 8) | lo) - 1 +4;
  hi = Wire.read(); lo = Wire.read(); raw_gz = (int16_t)((hi << 8) | lo) + 45-6;

  return true;
}

void beginMPU(){
  Wire.begin();
  Wire.setClock(400000); // optional: faster I2C (ok for short wires)
  Serial.begin(115200);
  delay(100);

  // Wake up
  writeMPU(0x6B, 0x00); // PWR_MGMT_1: clear sleep
  delay(10);

  // DLPF = 4 => ~20 Hz (good for videography)
  writeMPU(0x1A, 0x04);

  // SMPLRT_DIV = 0 -> sample rate = gyro output rate / (1 + SMPLRT_DIV)
  writeMPU(0x19, 0x00);

  // Gyro config ±2000 dps (FS_SEL = 3 -> bits 4:3 = 11 -> 0x18)
  writeMPU(0x1B, 0x18);

  // Accel config ±4g (AFS_SEL = 1 -> bits 4:3 = 01 -> 0x08)
  writeMPU(0x1C, 0x08);

  delay(100);

}

void getRaw(){
  // Read sensors, return early if I2C read failed
  if (!readMPUraw()) {
    Serial.print("failed bitch,  yo chip broke man !!");
    // I2C read failed; skip this cycle
    return;
  }

  // Convert raw to physical units
  // Accel: ±4g => 8192 LSB/g
  ax = (float)raw_ax/8192;
  ay = (float)raw_ay/8192;
  az = (float)raw_az/8192;

  // Gyro: ±2000 dps => 16.4 LSB/(deg/s)
  gx = (float)(raw_gx) / 16.4;
  gy = (float)(raw_gy) / 16.4;
  gz = (float)(raw_gz) / 16.4;

}

void calibrateGyro() {
  const int calSamples = 3000;
  long sumX = 0, sumY = 0, sumZ = 0;
  int validSamples = 0;
  delay(500);   // Let sensors settle

  // Collect samples
  while (validSamples < calSamples) {
    if (readMPUraw()) {          // Only use valid readings
      sumX += raw_gx;
      sumY += raw_gy;
      sumZ += raw_gz;
      validSamples++;
    }
    delay(1);
  }

  // Average offsets
  gx_offset = (float)sumX / validSamples;
  gy_offset = (float)sumY / validSamples;
  gz_offset = (float)sumZ / validSamples;
}

void calibrateACcc() {
  const int calSamples = 3000;
  long sumX = 0, sumY = 0, sumZ = 0;
  int validSamples = 0;
  delay(500);   // Let sensors settle

  // Collect samples
  while (validSamples < calSamples) {
    if (readMPUraw()) {          // Only use valid readings
      sumX += raw_ax;
      sumY += raw_ay;
      sumZ += raw_az;
      validSamples++;
    }
    delay(1);
  }

  // Average offsets
  ax_offset = (float)sumX / validSamples;
  ay_offset = (float)sumY / validSamples;
  az_offset = (float)sumZ / validSamples;
}

void showOffset(){
  Serial.println("-----------------Offset values ------------------");
  Serial.print("Offset in X [w]= ");
  Serial.print(gx_offset);
  Serial.print(" Offset in Y [w]= ");
  Serial.print(gy_offset);
  Serial.print(" Offset in Z [w]= ");
  Serial.print(gz_offset);
  Serial.print("  ||  Offset in X [g]= ");
  Serial.print(ax_offset);
  Serial.print(" Offset in Y [g]= ");
  Serial.print(ay_offset);
  Serial.print(" Offset in Z [g]= ");
  Serial.println(az_offset);
}

void show(){
    getRaw();
    Serial.print("(gx,gy,gz) : (");
    Serial.print(gx);
    Serial.print("      ,");
    Serial.print(gy);
    Serial.print("      ,");
    Serial.print(gz);
    Serial.print(        ") || (ax,ay,az) : (");
    Serial.print(ax);
    Serial.print("      ,");
    Serial.print(ay);
    Serial.print("      ,");
    Serial.print(az);
    Serial.println(       ")");
}

void setup() {
  beginMPU();
}
void loop() {
  show();
  calibrateGyro();
  calibrateACcc();
  showOffset();
}