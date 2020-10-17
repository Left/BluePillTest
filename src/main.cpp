// #include <optional>

#include <Arduino.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "protocol.pb.h"

#define __countof(x) (sizeof(x)/sizeof(x[0]))

/*
TIM_TypeDef *Instance2 = TIM3;
HardwareTimer *triac1Timer = new HardwareTimer(Instance2);
void Triac1Timer_callback(HardwareTimer*) {

}
*/

HardwareSerial serial2(PA3, PA2);
HardwareSerial serial3(PB11, PB10);

void setup() {
  serial3.begin(460800);
  serial2.begin(460800);

  // pinMode(PB9, INPUT);
  // pinMode(PC13, OUTPUT);
  pinMode(PC13, OUTPUT);

  pinMode(PA15, OUTPUT); 

  // digitalWrite(PC13, LOW);
  // digitalWrite(PA15, LOW);

  // Setup LED Timer
  // pwm_start(PA_15, 100, 50, MICROSEC_COMPARE_FORMAT);
  
  // pwm_start(PA_15, 1000, 800, MICROSEC_COMPARE_FORMAT);
  serial2.println("RESTART");
}

uint32_t t = millis();
uint32_t num = 0;
uint8_t state = 0;

uint8_t buffer[128];

void sendMessage(const Msg& message) {
    // Create a stream that will write to our buffer.
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    pb_encode(&stream, Msg_fields, &message);

    serial3.write(0x80);
    serial3.write(0x1d);
    serial3.write(0x7d);
    serial3.write(0x2e);
    serial3.write(0x00);
    serial3.write(0x03);
    serial3.write(0xb9);
    serial3.write(0x13);
    for (size_t i = 0; i < stream.bytes_written; ++i) {
      Serial3.write(buffer[i]);
    }
    serial3.write(0xff);
    serial3.write(0x5b);
    serial3.write(0xa1);
    serial3.write(0x35);
    serial3.write(0x33);
    serial3.write(0x6f);
    serial3.write(0xf5);
    serial3.write(0x37);
}

int8_t sensorWas = -1;

uint32_t ms = millis();
uint32_t pc13 = 1;
uint32_t st = LOW;
uint32_t x = 0;

void loop() {
  // uint32_t lvl = (x >> 5) & 0xff;
  
 // pwm_stop(PA_15);
 // pwm_start(PA_15, 100000, (x >> 10) % 1000, MICROSEC_COMPARE_FORMAT);
 // delay(30);

  // pwm_start(PA_15, 100, x, MICROSEC_COMPARE_FORMAT);
  // delay(100);
/*
  if ((millis() - ms) > 1000) {
    // int32_t state = (x / 10000000) & 0xff;
    // digitalWrite(PA15, st);
    digitalWrite(PC13, st);
    st = st == LOW ? HIGH : LOW;
  }
  ms = millis();
*/

  // pc13 = pc13 ? 0 : 1;
  
  pc13 = millis() % (4096);

  // serial2.print(pc13); 
  // serial2.println();
  while (serial3.available() > 8) {
    struct Signature {
        uint8_t start0;
        uint8_t start1;
        uint8_t start2;
        uint8_t start3;
        size_t size;
    } sig = { 42, 19, 53, 11, 0 };
    if (serial3.readBytes((uint8_t*) &sig, 8) == 8) {
      if (sig.start0 == 42 && sig.start1 == 19 && sig.start2 == 53 && sig.start3 == 11) {
        serial2.println(sig.size);
        std::vector<uint8_t> buf(sig.size, 0);
        serial3.readBytes(&buf[0], sig.size);
      }
      
    }
    // serial2.print(serial3.read());
  }

  // pwmWrite(PC13, pc13); 

  // analogWrite(PC13, pc13);
  // delay(1000);

  if (millis() - t >= 1000) {

    t = millis();
    // sendMessage(message);
  }

  // if (millis() % 10 == 0) {
    uint32_t pp = 0;
    if (millis() / 1000 % 60 < 5) {
      if (millis() / 1000 % 2 == 0) {
        pp = millis() % 1000;
      } else {
        pp = 1000 - millis() % 1000;
      }
      pwm_start(PA_15, 100, pp, RESOLUTION_10B_COMPARE_FORMAT);  
    } else {
      pwm_start(PA_15, 100, 1023, RESOLUTION_10B_COMPARE_FORMAT);  
    }
  // }

/*
  if (millis() - t >= 1000) {
    
    state = 1 - state;
    digitalWrite(PC13, state);

    SimpleMessage message = SimpleMessage_init_zero;
    
    // Fill in the lucky number
    message.id = t;
    message.num = num;
    message.num2 = digitalRead(PB9);
    strcpy(message.str, "Hello!");

    sendMessage(message);
    //      
  }
  */
}