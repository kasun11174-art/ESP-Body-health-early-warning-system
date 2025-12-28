#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MAX30102 sensor
MAX30105 sensor;

// Constants for heart rate sampling
const int samplingRate = 25; // Hz
const int duration = 3;      // seconds
const int sampleCount = samplingRate * duration;
long irBuffer[sampleCount];
float filteredIR[sampleCount];
unsigned long timeBuffer[sampleCount];

// Heart rate and temperature limits
int HR_High = 120;
int HR_Low = 60;
int Temp_High = 38;
int Temp_Low = 35;

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;

// I2C mutex
SemaphoreHandle_t i2cMutex;

// Buttons on D14, D27, D26, D25, D33, D32
const int buttonPins[] = {14, 27, 26, 25, 33, 32};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// State variable controlled by buttons
int Display_state = 0;
int hrReading = 0;
int tempReading = 0;

void movingAverageFilter(long* input, float* output, int len, int windowSize) {
  for (int i = 0; i < len; i++) {
    long sum = 0;
    int count = 0;
    for (int j = i - windowSize / 2; j <= i + windowSize / 2; j++) {
      if (j >= 0 && j < len) {
        sum += input[j];
        count++;
      }
    }
    output[i] = (float)sum / count;
  }
}

float analyzeBPM(float *data, unsigned long *timeStamps, int len) {
  int peakCount = 0;
  unsigned long lastPeakTime = 0;
  float intervalSum = 0;
  const float peakThreshold = 190000.0;

  for (int i = 1; i < len - 1; i++) {
    if (data[i] > data[i - 1] && data[i] > data[i + 1] && data[i] > peakThreshold) {
      if (lastPeakTime != 0) {
        float interval = (timeStamps[i] - lastPeakTime) / 1000.0;
        if (interval > 0.3 && interval < 1.5) {
          intervalSum += interval;
          peakCount++;
        }
      }
      lastPeakTime = timeStamps[i];
    }
  }

  if (peakCount > 0) {
    float avgInterval = intervalSum / peakCount;
    return 60.0 / avgInterval;
  } else {
    return 0;
  }
}

int Measure_HR() {
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    for (int i = 0; i < sampleCount; i++) {
      irBuffer[i] = sensor.getIR();
      timeBuffer[i] = millis();
      vTaskDelay(pdMS_TO_TICKS(1000 / samplingRate));
    }

    movingAverageFilter(irBuffer, filteredIR, sampleCount, 5);

    long irSum = 0;
    for (int i = 0; i < sampleCount; i++) {
      irSum += (long)filteredIR[i];
    }
    long irAvg = irSum / sampleCount;

    xSemaphoreGive(i2cMutex);

    if (irAvg < 200000) {
      Serial.println("No finger detected.");
      return 0;
    } else {
      float bpm = analyzeBPM(filteredIR, timeBuffer, sampleCount);
      if (bpm > 0) {
        Serial.print("BPM: ");
        Serial.println(bpm);
        return (int)bpm;
      } else {
        Serial.println("BPM not detected.");
        return 0;
      }
    }
  }
  return 0;
}

int Measure_Temp() {
  return 36; // simulated
}

void Task1(void *parameter) {
  while (true) {
    hrReading = Measure_HR();
    

    tempReading = Measure_Temp();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Task2(void *parameter) {
  while (true) {
    if (digitalRead(buttonPins[0]) == LOW) Display_state = 0;
    else if (digitalRead(buttonPins[1]) == LOW) Display_state = 1;
    else if (digitalRead(buttonPins[2]) == LOW) Display_state = 2;
    else if (digitalRead(buttonPins[5]) == LOW) Display_state = 3;

    xSemaphoreTake(i2cMutex, portMAX_DELAY);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    if (Display_state == 0) {
      display.println("Sensor Reading:");
      display.print("HR: "); display.println(hrReading);
      display.print("Temp: "); display.println(tempReading);
    } else if (Display_state == 1) {
      display.println("Set HR Limits");
      display.print("High: "); display.println(HR_High);
      display.print("Low : "); display.println(HR_Low);
      if (digitalRead(buttonPins[3]) == LOW) HR_High++;
      if (digitalRead(buttonPins[4]) == LOW) HR_Low--;
    } else if (Display_state == 2) {
      display.println("Set Temp Limits");
      display.print("High: "); display.println(Temp_High);
      display.print("Low : "); display.println(Temp_Low);
      if (digitalRead(buttonPins[3]) == LOW) Temp_High++;
      if (digitalRead(buttonPins[4]) == LOW) Temp_Low--;
    } else if (Display_state == 3) {
      display.println("All Limits");
      display.print("HR_H: "); display.println(HR_High);
      display.print("HR_L: "); display.println(HR_Low);
      display.print("T_H : "); display.println(Temp_High);
      display.print("T_L : "); display.println(Temp_Low);
    }

    display.display();
    xSemaphoreGive(i2cMutex);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Display Ready!"));
  display.display();

  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 not found.");
    while (1);
  }
  sensor.setup(0x1F, 4, 2, 100, 411, 4096);
  sensor.setPulseAmplitudeIR(0x3F);
  sensor.setPulseAmplitudeRed(0);

  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Failed to create I2C mutex");
    while (1);
  }

  xTaskCreatePinnedToCore(Task1, "Task1", 4096, NULL, 1, &Task1Handle, 0);
  xTaskCreatePinnedToCore(Task2, "Task2", 4096, NULL, 1, &Task2Handle, 1);
}

void loop() {
  // Empty - handled by tasks
}
