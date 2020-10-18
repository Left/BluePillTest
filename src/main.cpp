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

  // pinMode(PA15, OUTPUT); 

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

uint32_t pins[][16] = {
  { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15},
  { PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15}
  // { PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15}
};

PinName pins_[][16] = {
  { PA_0, PA_1, PA_2, PA_3, PA_4, PA_5, PA_6, PA_7, PA_8, PA_9, PA_10, PA_11, PA_12, PA_13, PA_14, PA_15},
  { PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7, PB_8, PB_9, PB_10, PB_11, PB_12, PB_13, PB_14, PB_15}
  // { PC_0, PC_1, PC_2, PC_3, PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13, PC_14, PC_15}
};


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

    auto cmd = buffer[0];
    auto pinABC = buffer[1];
    // debugSerial.print(pinABC); debugSerial.println(": pinABC");

    if (pinABC >= 0 && pinABC < sizeof(pins)/sizeof(pins[0])) {
      // 
      auto pinINDEX = buffer[2];
      auto prow = pins[pinABC];
      auto p_row = pins_[pinABC];
      if (pinINDEX >= 0 && pinINDEX < 16) {
        // debugSerial.println("pinRow ");

        auto PIN = prow[pinINDEX];
        auto PIN_ = p_row[pinINDEX];
        if (cmd == 0) {
          // Setup
          auto pinType = buffer[3];
          if (pinType == 0) {
            // Digital
            pinMode(PIN, OUTPUT);
            debugSerial.println("pinMode(PIN, OUTPUT);");
          } else if (pinType == 1) {
            // PWM
            pinMode(PIN, OUTPUT);
          } else if (pinType == 2) {
            // Input
            pinMode(PIN, INPUT);
          }
        } else if (cmd == 1) {
            // Digital write
            digitalWrite(PIN, buffer[3]);
            debugSerial.println("digitalWrite(PIN, buffer[3]);");
        } else if (cmd == 2) {
            // PWM write 
            pwm_start(PIN_, 100, buffer[3], RESOLUTION_8B_COMPARE_FORMAT);  
        }
        
      }
    }
    
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
    /*
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
    */
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