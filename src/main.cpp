// #include <optional>
#include <Arduino.h>

#include "../../MyWiFiController/src/uart.h"

#define __countof(x) (sizeof(x)/sizeof(x[0]))

/*
TIM_TypeDef *Instance2 = TIM3;
HardwareTimer *triac1Timer = new HardwareTimer(Instance2);
void Triac1Timer_callback(HardwareTimer*) {

}
*/

HardwareSerial debugSerial(PA3, PA2); // UART2
HardwareSerial esp8266Serial(PB11, PB10); // 

uart::Sender sndr(
  [](const uint8_t* buffer, size_t size) {
    return esp8266Serial.write(buffer, size);
  }
);

uart::Receiver rcvr(
  []() {
    auto av = esp8266Serial.available();
    return av;
  },
  [](uint8_t* buffer, size_t size) {
    return esp8266Serial.readBytes(buffer, size);
  }
);

void setup() {
  esp8266Serial.begin(460800);
  debugSerial.begin(921600);

  // pinMode(PB9, INPUT);
  // pinMode(PC13, OUTPUT);
  pinMode(PC13, OUTPUT);

  pinMode(PA15, OUTPUT); 

  // digitalWrite(PC13, LOW);
  // digitalWrite(PA15, LOW);

  // Setup LED Timer
  // pwm_start(PA_15, 100, 50, MICROSEC_COMPARE_FORMAT);
  
  // pwm_start(PA_15, 1000, 800, MICROSEC_COMPARE_FORMAT);
  debugSerial.println("RESTART!");
}

uint32_t t = millis();
uint32_t num = 0;
uint8_t state = 0;

uint8_t buffer[128];

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
  rcvr.receive([](uint8_t* buffer, size_t size) {
    // 
    debugSerial.print(size); debugSerial.print(": ");
    for (size_t i = 0; i < size; ++i) {
      debugSerial.print((int) buffer[i]); debugSerial.print(" ");
    }
    debugSerial.println();
  });

  // pwmWrite(PC13, pc13); 

  // analogWrite(PC13, pc13);
  // delay(1000);

  if (millis() - t >= 1000) {

    t = millis();
    // debugSerial.println(t);
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