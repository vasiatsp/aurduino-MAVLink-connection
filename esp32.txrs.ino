
#include<HardwareSerial.h>
//#define RX_PIN 6
//#define TX_PIN 7

HardwareSerial SerialPort1(1);
void setup() {

  Serial.begin(115200);
 SerialPort1.begin(115200, SERIAL_8N1, 6, 7);
 SerialPort1.println("Hello");
}

void loop() {
  int mess;
  
    //Serial.println("hello"); 

while (!SerialPort1.available());

   mess = SerialPort1.read();
  
    SerialPort1.print("Received message: ");
    SerialPort1.println((char) (mess + 'A'-'a'));
  

   
    delay(1000);
  }