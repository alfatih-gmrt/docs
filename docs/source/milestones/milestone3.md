# Milestone 3

## Dependencies
- TCA9548A by Jonathan Dempsey
- Adafruit_VL53L0X by Adafruit
- Adafruit BNO055 by Adafruit

## TCA9548A

```arduino
#include "Wire.h"
#include "TCA9548A.h"

TCA9548A mux;

void setup() {
  Serial.begin(115200);

  mux.begin(Wire);

  // memastikan semua channel tertutup
  mux.closeAll();
}

void loop() {
  mux.openChannel(0);

  // ... komunikasi I2C untuk sensor

  mux.closeChannel(0);
}

```

## VL53L0X

```arduino
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  
  lox.rangingTest(&measure);

  if (measure.RangeStatus != 4) {  // 4 = phase failures have incorrect data
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  delay(100);
}
```

## BNO055

```arduino
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup() {
  Serial.begin(115200);

  if(!bno.begin()) {
    while(1);
  }

  bno.setExtCrystalUse(true);
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // jgn tanya kenapa z jadi x, x jadi z
  // dan kenapa dikasih minus
  // orientasi menurut Adafruit dan menurut fisik BNO055 beda
  // udah nyoba bno.setAxisRemap() masih sama aja
  double x = -euler.z();
  double y = -euler.y();
  double z = 360 - euler.x();

  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.println(z);

  delay(10);
}
```