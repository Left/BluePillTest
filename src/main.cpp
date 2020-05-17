// #include <optional>

#include <Arduino.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "protocol.pb.h"

#define __countof(x) (sizeof(x)/sizeof(x[0]))

void setup() {
  pinMode(PB9, INPUT);
  pinMode(PC13, OUTPUT);

  Serial3.begin(460800);
}

uint32_t t = millis();
uint32_t num = 0;
uint8_t state = 0;

uint8_t buffer[128];

void sendMessage(const SimpleMessage& message) {
    // Create a stream that will write to our buffer.
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    pb_encode(&stream, Message_fields, &message);

    Serial3.write(0x80);
    Serial3.write(0x1d);
    Serial3.write(0x7d);
    Serial3.write(0x2e);
    Serial3.write(0x00);
    Serial3.write(0x03);
    Serial3.write(0xb9);
    Serial3.write(0x13);
    for (size_t i = 0; i < stream.bytes_written; ++i) {
      Serial3.write(buffer[i]);
    }
    Serial3.write(0xff);
    Serial3.write(0x5b);
    Serial3.write(0xa1);
    Serial3.write(0x35);
    Serial3.write(0x33);
    Serial3.write(0x6f);
    Serial3.write(0xf5);
    Serial3.write(0x37);
}

int8_t sensorWas = -1;

void loop() {
  int8_t pb9 = digitalRead(PB9);
  if (pb9 != sensorWas) {
    digitalWrite(PC13, pb9 ? 0 : 1);
    sensorWas = pb9;

    Message message = Message_init_zero;
    
    // Fill in the lucky number
    message.id = t;
    message.num2 = pb9;
    // strcpy(message.str, "Hello!");

    if (millis() - t >= 1000) {
      t = millis();
      sendMessage(message);
    }
  }
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