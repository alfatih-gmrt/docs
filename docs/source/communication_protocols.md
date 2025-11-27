# Communication Protocols in Microcontroller

Dalam *microcontroller*, setidaknya terdapat 3 jenis protokol komunikasi, yaitu:  
- UART
- I2C
- SPI

## UART (Universal Asynchronous Receiver/Transmitter)

UART adalah protokol komunikasi yang paling dasar di mikrokontroler.

UART membutuhkan 2 pin untuk komunikasi, yaitu pin RX dan TX. Cara menghubungkannya seperti gambar di bawah.  

![UART](https://academy.nordicsemi.com/wp-content/uploads/2022/02/image-1024x474-1-1.png)

### Komunikasi Mikrokontroler dengan Komputer

Sebenarnya mikrokontroler seperti Arduino Uno dan ESP32 berkomunikasi dengan komputer melalui UART. Tetapi karena komputer menggunakan protokol USB, diperlukan *converter* antara keduanya.

UART ---> Converter UART to USB ---> USB

Jadi ketika kita menghubungkan mikrokontroler dengan port USB itu kita berkomunikasi dengan mikrokontroler melalui protokol UART.

Contoh *converter* USB to UART adalah CP2102, CH340, dan FT232RL.

### Program Arduino untuk Berkomunikasi dengan Komputer

Komunikasi menggunakan protokol UART pada Arduino dilakukan menggunakan *object* Serial.  
Contoh:

```arduino

void setup() {
    Serial.begin(115200);
}

void loop() {
    Serial.println("Hello from Arduino!");
    delay(1000);
}
```

```Serial.print()``` mengirim data dalam format ASCII yang merupakan format untuk ```string``` atau ```char```.

Apabila mau mengirim data ```bool```, ```int```, ```float```, atau ```double``` sebaiknya menggunakan ```Serial.print()```.  
Contoh:

```arduino

void setup() {
    Serial.begin(115200);
}

void loop() {
    float range = 12.34;

    int varSize = sizeof(range);  // variable size (float = 4)

    uint8_t buffer[varSize];
    memcpy(buffer, &range, varSize);

    Serial.write(buffer, varSize);

    delay(1000);
}
```

**belum selesai ditulis hehe**