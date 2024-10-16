/*
 * This file is part of Power and SWR Meter OLED.
 *
 * <Your Project Name> is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * <Your Project Name> is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with <Your Project Name>. If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) <2024> Marcelo R. Gadotti (PP5MGT)
 *
 * Platform........: STM32F103C8T6 - Blue Pill
 * Initial version.: 0.1
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include "definitions.h"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Variables for button debounce
uint8_t buttonState;
uint8_t lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// FST from OLED screen (change display)
uint8_t screenState = 0;

// measured values
float fwdAdcVolts = 0.0;
float revAdcVolts = 0.0;
float fwdVp = 0.0;
float revVp = 0.0;
float fwdPwr = 0.0;
float fwdPwrPep = 0.0;
float revPwr = 0.0;
float swr = 0.0;

// map function using float values
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setupIo() {
  pinMode(SCREEN_BT_PIN, INPUT_PULLUP);
  pinMode(ADC_FWD, INPUT);
  pinMode(ADC_REV, INPUT);
  analogReadResolution(ADC_RES);
}

void readButton() {
  uint8_t reading = digitalRead(SCREEN_BT_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        screenState += 1;
        if (screenState > 4) screenState = 0;
      }
    }
  }
  lastButtonState = reading;
}

void startOledDisplay() {
  u8g2.begin();
  u8g2.setContrast(5);  // Change to increase OLED brightness
  u8g2.clearBuffer();
}


void startupMessage() {
  u8g2.setFont(u8g2_font_unifont_t_0_75);
  int16_t textWidth = u8g2.getStrWidth("PP5MGT");
  u8g2.drawStr((128 - textWidth) / 2, 20, "PP5MGT");
  textWidth = u8g2.getStrWidth("PW AND SWR METER");
  u8g2.drawStr((128 - textWidth) / 2, 35, "PW AND SWR METER");
  textWidth = u8g2.getStrWidth(VERSION);
  u8g2.drawStr((128 - textWidth) / 2, 50, VERSION);
  u8g2.sendBuffer();
  delay(300);
}

float readAdcVolts(uint8_t channel) {
  return (analogRead(channel) * ADC_VREF) / ADC_BITS;
}

void measureValues() {
  // filtered using a first-order IIR filter (smoothing filter)
  fwdAdcVolts = (0.4 * fwdAdcVolts) + (0.6 * readAdcVolts(ADC_FWD));
  revAdcVolts = (0.4 * revAdcVolts) + (0.6 * readAdcVolts(ADC_REV));
  fwdVp = ((fwdAdcVolts * ADC_SCALE) * TURNS_RATIO) + DIODE_DROP_F;
  revVp = ((revAdcVolts * ADC_SCALE) * TURNS_RATIO) + DIODE_DROP_R;

  fwdPwr = pow(fwdVp, 2) / 100;
  revPwr = pow(revVp, 2) / 100;
  fwdPwrPep = pow(fwdVp, 2) / 50;

  // TODO: If revPwr > fwdPwr, error, set to 1.0
  const float reflection_ratio = sqrt(revPwr / fwdPwr);
  swr = (1.0f + reflection_ratio) / (1.0f - reflection_ratio);
}

void drawBar(uint8_t progress, uint8_t y, uint8_t h) {
  u8g2.drawLine(0, y, 127, y);
  u8g2.drawBox(0, y + 1, progress, h);
  uint8_t y_l = y + h;
  u8g2.drawLine(0, y_l, 127, y_l);
  uint8_t y_step_1 = y + h + 1;
  uint8_t y_step_2 = y + h + 2;
  u8g2.drawLine(0, y_step_1, 0, y_step_2);
  u8g2.drawLine(15, y_step_1, 15, y_step_2);
  u8g2.drawLine(31, y_step_1, 31, y_step_2);
  u8g2.drawLine(47, y_step_1, 47, y_step_2);
  u8g2.drawLine(63, y_step_1, 63, y_step_2);
  u8g2.drawLine(79, y_step_1, 79, y_step_2);
  u8g2.drawLine(95, y_step_1, 95, y_step_2);
  u8g2.drawLine(111, y_step_1, 111, y_step_2);
  u8g2.drawLine(127, y_step_1, 127, y_step_2);
}

void drawSwrBar(uint8_t progress, uint8_t y, uint8_t h) {
  u8g2.drawLine(0, y, 127, y);
  u8g2.drawBox(0, y + 1, progress, h);
  uint8_t y_l = y + h;
  u8g2.drawLine(0, y_l, 127, y_l);
  uint8_t y_step_1 = y + h + 1;
  uint8_t y_step_2 = y + h + 2;
  u8g2.drawLine(0, y_step_1, 0, y_step_2);
  u8g2.drawLine(31, y_step_1, 31, y_step_2);
  u8g2.drawLine(63, y_step_1, 63, y_step_2);
  u8g2.drawLine(95, y_step_1, 95, y_step_2);
  u8g2.drawLine(127, y_step_1, 127, y_step_2);
}

void drawPwr() {
  u8g2.drawStr(0, 18, "FWD ");
  u8g2.drawStr(100, 18, "120");

  u8g2.setCursor(30, 18);
  u8g2.print(fwdPwr, 2);
  uint16_t pwr = (fwdPwr > 120.0) ? 120 : fwdPwr;
  uint8_t progress = map(pwr, 0, 120, 0, 128);
  drawBar(progress, 21, 7);
}

void drawPwrPep() {
  u8g2.drawStr(0, 18, "PEP ");
  u8g2.drawStr(100, 18, "120");

  u8g2.setCursor(30, 18);
  u8g2.print(fwdPwrPep, 2);
  uint16_t pwr = (fwdPwrPep > 120.0) ? 120 : fwdPwrPep;
  uint8_t progress = map(pwr, 0, 120, 0, 128);
  drawBar(progress, 21, 7);
}

void drawRev() {
  u8g2.drawStr(0, 48, "REV ");
  u8g2.drawStr(100, 48, "120");

  u8g2.setCursor(30, 48);
  u8g2.print(revPwr, 2);
  uint16_t pwr = (revPwr > 120.0) ? 120 : revPwr;
  uint8_t progress = map(pwr, 0, 120, 0, 128);
  drawBar(progress, 51, 7);
}

void drawSwr() {
  u8g2.drawStr(0, 48, "SWR ");
  u8g2.setCursor(30, 48);
  u8g2.print(swr, 2);
  // test swr is behind 1.0 and 3.0
  uint8_t progress = mapFloat((swr > 3.0) ? 3.0 : swr, 1.0, 3.0, 0, 128);
  drawSwrBar(progress, 51, 7);
}

void drawPwrBig() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fur14_tr);
  u8g2.drawStr(0, 18, "FWD ");

  u8g2.setCursor(50, 18);
  u8g2.print(fwdPwr, 2);
  uint16_t pwr = (fwdPwr > 120.0) ? 120 : fwdPwr;
  uint8_t progress = map(pwr, 0, 120, 0, 128);
  drawBar(progress, 35, 15);
  u8g2.sendBuffer();
}

void drawPwrSwr() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_0_75);
  drawPwr();
  drawSwr();
  u8g2.sendBuffer();
}

void drawPwrPepSwr() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_0_75);
  drawPwrPep();
  drawSwr();
  u8g2.sendBuffer();
}

// const uint8_t totalSamples = 108;
// uint16_t samples[totalSamples];
// uint16_t currentSampleIndex = 0;
// uint8_t graphHeight = 60;
// float maxAdcValue = 4095.0;

// void drawGraph() {
//   u8g2.clearBuffer();
//   int adcValue = analogRead(ADC_FWD);
//   samples[currentSampleIndex] = adcValue;
//   currentSampleIndex = (currentSampleIndex + 1) % totalSamples;

//   u8g2.setFont(u8g2_font_5x8_tr);
//   u8g2.drawStr(0, 6, "1200");
//   u8g2.drawStr(0, 63, "0");

//   u8g2.drawLine(20, 63, 128, 63);
//   u8g2.drawLine(20, 0, 20, 63);


//   for (int i = 1; i < totalSamples; i++) {
//     int prevIndex = (currentSampleIndex + i - 1) % totalSamples;
//     int currIndex = (currentSampleIndex + i) % totalSamples;

//     int y1 = 64 - (samples[prevIndex] / maxAdcValue) * graphHeight;
//     int y2 = 64 - (samples[currIndex] / maxAdcValue) * graphHeight;

//     int x1 = map(i - 1, 0, totalSamples, 20, 128);
//     int x2 = map(i, 0, totalSamples, 20, 128);

//     u8g2.drawLine(x1, y1, x2, y2);
//   }
//   u8g2.setFont(u8g2_font_unifont_t_0_75);
//   u8g2.setCursor(90, 12);
//   u8g2.print(adcValue);

//   u8g2.sendBuffer();
// }

void drawPwrRev() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_0_75);
  drawPwr();
  drawRev();
  u8g2.sendBuffer();
}

void printValues() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_0_75);

  // FWD
  u8g2.drawStr(0, 18, "F ");
  u8g2.setCursor(25, 18);
  u8g2.print(fwdPwr, 1);
  u8g2.setCursor(78, 18);
  u8g2.print(fwdVp, 2);
  u8g2.setCursor(0, 31);
  u8g2.print(fwdAdcVolts, 3);
  u8g2.print("V");

  // REV
  u8g2.drawStr(0, 48, "R ");
  u8g2.setCursor(25, 48);
  u8g2.print(revPwr, 1);
  u8g2.setCursor(78, 48);
  u8g2.print(revVp, 2);
  u8g2.setCursor(0, 61);
  u8g2.print(revAdcVolts, 3);
  u8g2.print("V");
  u8g2.setCursor(63, 61);
  u8g2.print(swr, 2);

  u8g2.sendBuffer();
}

void setup(void) {
  setupIo();
  startOledDisplay();
  startupMessage();
}

void loop(void) {
  readButton();
  measureValues();
  switch (screenState) {
    case 0:
      drawPwrSwr();
      break;
    case 1:
      drawPwrRev();
      break;
    case 2:
      drawPwrBig();
      break;
    case 3:
      printValues();
      break;
    case 4:
      drawPwrPepSwr();
      break;
  }
}
