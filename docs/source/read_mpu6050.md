# Mengukur Orientasi dengan MPU6050

MPU6050 menggunakan komunikasi I2C yang tipe komunikasi digital yang sangat mudah digunakan.

## Wokwi
Pengukuran dan perhitungan bisa dilakukan secara simulasi melalui website [Wokwi](wokwi.com).

Langkah-langkah:
- Pilih microcontroller ESP32 untuk simplifikasi.
- Tambahkan komponen MPU6050.
- Cukup hubungkan pin VCC, GND, SCL, dan SDA dengan ESP32.
- Mulai coding asyique.
- Coding sendiri yaa, kalau mau lihat contoh bisa klik MPU6050-nya, nanti muncul tanda tanya. Nahh di situ dokumentasinya.

**note**: kalau mau simpel bisa pakai library Adafruit atau yang lainnya. Tapi berikut contoh cara membaca data dari Gyroscope dan Accelerometer tanpa library apapun kecuali Wire.h untuk komunikasi I2C.

## Read Gyroscope

```arduino
#include <Wire.h>

constexpr int MPU_ADDR = 0x68; // Alamat I2C MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050 (keluar dari sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Register PWR_MGMT_1
  Wire.write(0);     // Set ke 0 untuk mengaktifkan sensor
  Wire.endTransmission(true);

  // Optional: set gyro full scale = ±250 °/s (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // Gyro Configuration
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission(true);

  Serial.println("MPU6050 Gyroscope Ready");
}

void loop() {
  int16_t gyroX, gyroY, gyroZ;

  // Mulai baca dari register GYRO_XOUT_H
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // Start at GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Baca 6 byte

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  // Konversi ke derajat/detik
  // Sensitivity: 131 LSB per °/s untuk ±250 dps
  float gx = gyroX / 131.0;
  float gy = gyroY / 131.0;
  float gz = gyroZ / 131.0;

  Serial.print("Gyro X: "); Serial.print(gx); Serial.print(" °/s");
  Serial.print("  |  Y: "); Serial.print(gy); Serial.print(" °/s");
  Serial.print("  |  Z: "); Serial.println(gz); Serial.print(" °/s");

  delay(50);
}
```

## Read Accelerometer

```arduino
#include <Wire.h>

const int MPU_ADDR = 0x68; // Alamat I2C MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050 (keluar dari sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // Set ke 0 = aktif
  Wire.endTransmission(true);

  // Optional: set accelerometer full-scale = ±2g (default)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission(true);

  Serial.println("MPU6050 Accelerometer Ready");
}

void loop() {
  int16_t accX, accY, accZ;

  // Baca mulai dari register ACCEL_XOUT_H
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  // Konversi ke satuan g
  // Sensitivitas: 16384 LSB per g untuk ±2g
  float ax = accX / 16384.0;
  float ay = accY / 16384.0;
  float az = accZ / 16384.0;

  Serial.print("Acc X: "); Serial.print(ax); Serial.print(" g");
  Serial.print("  |  Y: "); Serial.print(ay); Serial.print(" g");
  Serial.print("  |  Z: "); Serial.println(az); Serial.print(" g");

  delay(50);
}
```