#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLEHIDDevice.h"
#include <Wire.h>
#include "mpu6050.h"
#include "MahonyAHRS.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include <EEPROM.h>


#define SENSIBILIDADE 30
#define MPU6050_ACC_GAIN 16384.0
#define MPU6050_GYRO_GAIN 131.072
#define BOTAO_PIN 10
#define BOTAO2_PIN 7
#define SLEEP_BUTTON_PIN 6
#define EEPROM_SIZE 512  

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
extern float yaw_mahony, pitch_mahony, roll_mahony;
float axR, ayR, azR, gxR, gyR, gzR;
float axg, ayg, azg, gxrs, gyrs, gzrs;
//float ax_filtro, ay_filtro, az_filtro, gx_filtro, gy_filtro, gz_filtro;
bool conectado = false;
//float yaw_filtro, pitch_filtro, roll_filtro;
const int MPU_addr = 0x68;  


class MouseIMU {
public:
  MouseIMU()
    : deviceConnected(false), report{ 0, 0, 0, 0 }, sleepMode(false), addr_eep(0) {}

  void init() {
    Serial.begin(115200);
    Serial.println("Initializing BLE Mouse");

    EEPROM.begin(EEPROM_SIZE); 
    initMPU();
    initBLE();
    initButtons();
    initMahony();
    initSleepButton();
  }

  void update() {
    unsigned long currentMillis = millis();
    unsigned long previousMillis;
    if (currentMillis - previousMillis >= 10) {
      previousMillis = currentMillis;
      checkButtons();
      checkSleepButton();

      if (sleepMode) {
        goToLightSleep();
      }

      if (deviceConnected && !sleepMode) {
        updateMouseReport();
        input->setValue(report, sizeof(report));
        input->notify();
      }
    }
  }


private:
  //MPU6050 mpu;
  BLEHIDDevice* hid;
  BLECharacteristic* input;
  bool deviceConnected;
  int lastButtonState = HIGH;
  int buttonState;
  int lastButton2State = HIGH;
  int button2State;
  int lastSleepButtonState = HIGH;
  int sleepButtonState;
  unsigned long lastDebounceTime = 0;
  unsigned long lastDebounceTime2 = 0;
  unsigned long lastSleepDebounceTime = 0;
  unsigned long debounceDelay = 50;
  int deadzoneMin = 30;
  uint8_t report[4];
  bool blinkState = false;
  bool sleepMode;

  int addr_eep;  
  typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
  } calibration_t;
  typedef union {
    calibration_t xyz;
    uint8_t buffer[12];
  } calibrationUnion_t;

  calibrationUnion_t calibValues;
  float GyX_offset, GyY_offset, GyZ_offset;

  void initMPU() {
    Wire.begin(9, 8);
    Wire.setClock(400000);
    delay(50);
    MPU6050_Init();
    Serial.println("MPU6050 connected");

    if (IMU_calibration()) {
      Serial.println("IMU Calibration successful");
    } else {
      Serial.println("IMU Calibration failed");
    }
  }
  void filtraIMU() {
    axg = (float)(AcX) / MPU6050_ACC_GAIN;
    ayg = (float)(AcY) / MPU6050_ACC_GAIN;
    azg = (float)(AcZ) / MPU6050_ACC_GAIN;
    gxrs = (float)(GyX - GyX_offset) / MPU6050_GYRO_GAIN * 0.01745329;
    gyrs = (float)(GyY - GyY_offset) / MPU6050_GYRO_GAIN * 0.01745329;
    gzrs = (float)(GyZ - GyZ_offset) / MPU6050_GYRO_GAIN * 0.01745329;
  }


#define SAMPLES 100  
#define FILTER_SIZE 5  
#define MEDIAN_ITERATIONS 3  

float gyroX_buffer[FILTER_SIZE] = {0};
float gyroY_buffer[FILTER_SIZE] = {0};
float gyroZ_buffer[FILTER_SIZE] = {0};
int buffer_index = 0;

// Função para calcular a mediana
float calculateMedian(float arr[], int size) {
    float sorted[size];
    memcpy(sorted, arr, size * sizeof(float));
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (sorted[i] > sorted[j]) {
                float temp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = temp;
            }
        }
    }
    return size % 2 == 0 ? (sorted[size / 2 - 1] + sorted[size / 2]) / 2 : sorted[size / 2];
}

// Função para aplicar o filtro de médias de medianas
float filterMedianAverage(float new_reading, float buffer[], int size) {
    buffer[buffer_index] = new_reading;
    buffer_index = (buffer_index + 1) % size;

    float medians[MEDIAN_ITERATIONS];
    for (int i = 0; i < MEDIAN_ITERATIONS; i++) {
        medians[i] = calculateMedian(buffer, size);
    }

    float sum = 0;
    for (int i = 0; i < MEDIAN_ITERATIONS; i++) {
        sum += medians[i];
    }
    return sum / MEDIAN_ITERATIONS;
}

// Função de calibração com filtro
bool IMU_calibration() {
    static bool calibrated = false;
    static int32_t counter = 0;
    static int32_t samples_x = 0;
    static int32_t samples_y = 0;
    static int32_t samples_z = 0;
    byte value;

    value = EEPROM.read(addr_eep);

    if (value == 1) {
        calibrated = true;
        for (int i = 0; i < 12; i++) {
            calibValues.buffer[i] = EEPROM.read(addr_eep + i + 1);
        }
        GyX_offset = calibValues.xyz.x;
        GyY_offset = calibValues.xyz.y;
        GyZ_offset = calibValues.xyz.z;
    } else if (!calibrated) {
        if (counter < SAMPLES) {
         
            samples_x += filterMedianAverage(GyX, gyroX_buffer, FILTER_SIZE);
            samples_y += filterMedianAverage(GyY, gyroY_buffer, FILTER_SIZE);
            samples_z += filterMedianAverage(GyZ, gyroZ_buffer, FILTER_SIZE);
        } else {
            GyX_offset = samples_x / SAMPLES;
            GyY_offset = samples_y / SAMPLES;
            GyZ_offset = samples_z / SAMPLES;

            calibValues.xyz.x = GyX_offset;
            calibValues.xyz.y = GyY_offset;
            calibValues.xyz.z = GyZ_offset;

            EEPROM.write(addr_eep, 1);
            for (int i = 0; i < 12; i++) {
                EEPROM.write(addr_eep + i + 1, calibValues.buffer[i]);
            }
            EEPROM.commit();
            calibrated = true;
            Serial.println("Calibração concluída com filtro de medianas.");
        }
        counter++;
    }
    return calibrated;
}



  void initButtons() {
    pinMode(BOTAO_PIN, INPUT_PULLUP);
    pinMode(BOTAO2_PIN, INPUT_PULLUP);
    Serial.println("Buttons initialized");
  }

  void initSleepButton() {
    pinMode(SLEEP_BUTTON_PIN, INPUT_PULLUP);

    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable((gpio_num_t)SLEEP_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    Serial.println("Sleep button initialized");
  }

  void checkSleepButton() {
    int reading = digitalRead(SLEEP_BUTTON_PIN);

    if (reading != lastSleepButtonState) {
      lastSleepDebounceTime = millis();
    }

    if ((millis() - lastSleepDebounceTime) > debounceDelay) {
      if (reading != sleepButtonState) {
        sleepButtonState = reading;
        if (sleepButtonState == LOW) {
          sleepMode = !sleepMode;
          Serial.println(sleepMode ? "Entering light sleep mode" : "Exiting light sleep mode");
        }
      }
    }

    lastSleepButtonState = reading;
  }

  void goToLightSleep() {
    Serial.println("Going to light sleep...");


    gpio_pullup_en((gpio_num_t)SLEEP_BUTTON_PIN);
    //gpio_pulldown_dis((gpio_num_t)SLEEP_BUTTON_PIN);

    esp_light_sleep_start();


    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
      sleepMode = false;
      Serial.println("Woke up from light sleep");
    }
  }

  void initMahony() {
    twoKp = 2.0f * 0.5f;
    twoKi = 2.0f * 0.0f;
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    Serial.println("Mahony filter initialized");
  }
  void checkButtons() {
    int reading = digitalRead(BOTAO_PIN);
    int reading2 = digitalRead(BOTAO2_PIN);


    if (reading != lastButtonState) {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
      }
    }

    lastButtonState = reading;


    if (reading2 != lastButton2State) {
      lastDebounceTime2 = millis();
    }

    if ((millis() - lastDebounceTime2) > debounceDelay) {
      if (reading2 != button2State) {
        button2State = reading2;
      }
    }

    lastButton2State = reading2;
  }

  void initBLE() {
    BLEDevice::init("ESP32 BLE Mouse");
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));

    hid = new BLEHIDDevice(pServer);
    input = hid->inputReport(1);

    hid->manufacturer()->setValue("Espressif");
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    hid->hidInfo(0x00, 0x02);

    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);
    pSecurity->setCapability(ESP_IO_CAP_NONE);
    pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    const uint8_t reportMap[] = {
      USAGE_PAGE(1), 0x01,       // Página de Uso (Generic Desktop)
      USAGE(1), 0x02,            // Uso (Mouse)
      COLLECTION(1), 0x01,       // Coleção (Aplicativo)
      USAGE(1), 0x01,            // Uso (Apontador)
      COLLECTION(1), 0x00,       // Coleção (Física)
      REPORT_ID(1), 0x01,        // ID do Relatório (1)
      USAGE_PAGE(1), 0x09,       // Página de Uso (Botão)
      USAGE_MINIMUM(1), 0x01,    // Uso Mínimo (Botão 1)
      USAGE_MAXIMUM(1), 0x03,    // Uso Máximo (Botão 3)
      LOGICAL_MINIMUM(1), 0x00,  // Mínimo Lógico (0)
      LOGICAL_MAXIMUM(1), 0x01,  // Máximo Lógico (1)
      REPORT_SIZE(1), 0x01,      // Tamanho do Relatório (1)
      REPORT_COUNT(1), 0x03,     // Contagem do Relatório (3)
      0x81, 0x02,                // Entrada (Dados, Variável, Absoluto)
      REPORT_SIZE(1), 0x05,      // Tamanho do Relatório (5)
      REPORT_COUNT(1), 0x01,     // Contagem do Relatório (1)
      0x81, 0x01,                // Entrada (Constante, Variável, Absoluto)
      USAGE_PAGE(1), 0x01,       // Página de Uso (Generic Desktop)
      USAGE(1), 0x30,            // Uso (X)
      USAGE(1), 0x31,            // Uso (Y)
      LOGICAL_MINIMUM(1), 0x81,  // Mínimo Lógico (-127)
      LOGICAL_MAXIMUM(1), 0x7f,  // Máximo Lógico (127)
      REPORT_SIZE(1), 0x08,      // Tamanho do Relatório (8)
      REPORT_COUNT(1), 0x02,     // Contagem do Relatório (2)
      0x81, 0x06,                // Entrada (Dados, Variável, Relativo)
      USAGE(1), 0x38,            // Uso (Wheel)
      LOGICAL_MINIMUM(1), 0x81,  // Mínimo Lógico (-127)
      LOGICAL_MAXIMUM(1), 0x7f,  // Máximo Lógico (127)
      REPORT_SIZE(1), 0x08,      // Tamanho do Relatório (8)
      REPORT_COUNT(1), 0x01,     // Contagem do Relatório (1)
      0x81, 0x06,                // Entrada (Dados, Variável, Relativo)
      END_COLLECTION(0),         // Fim da Coleção
      END_COLLECTION(0)          // Fim da Coleção
    };

    hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));
    hid->startServices();

    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_MOUSE);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);

    pAdvertising->start();
    Serial.println("advertising started");
  }


  float corrigeYaw(float sinal) {
    static float valorDeriv = 0, zero = 0;
    static float sinalCorrigido = 0, offset = 0;
    static float sinalAnterior = 0;
    static int s = 1, d = -1;

    valorDeriv = (sinal - zero);
    zero = sinal;

    if (valorDeriv <= -180) {
      offset += +360.0f;
    } else if (valorDeriv >= 180) {
      offset += -360.0f;
    }
    sinalCorrigido = sinal;
    sinalCorrigido += offset;

    return sinalCorrigido;
  }
  int8_t mouseHoriz(void) {
    static float horzZero = 0.0f;
    static float horzValue = 0.0f;  
    static float roll_corrigido = 0.0f;
    static int amostragem = 5;
    roll_corrigido = corrigePitch(roll_mahony);

    if (--amostragem == 0) {
      amostragem = 5;
      horzValue = (roll_corrigido - horzZero) * SENSIBILIDADE;
      horzZero = roll_corrigido;
    }
    return -horzValue;
  }
  float corrigePitch(float sinal) {
    static float valorDeriv = 0, zero = 0;
    static float sinalCorrigido = 0, offset = 0;
    static float sinalAnterior = 0;
    static int s = 1, d = -1;

    valorDeriv = (sinal - zero);
    zero = sinal;

    if (valorDeriv <= -180) {
      offset += +360.0f;
    } else if (valorDeriv >= 180) {
      offset += -360.0f;
    }
    sinalCorrigido = sinal;
    sinalCorrigido += offset;

    return sinalCorrigido;
  }
  int8_t mouseVert(void) {
    static float vertZero = 0.0f;
    static float vertValue = 0.0f;  
    static float pitch_corrigido = 0.0f;
    static int amostragem = 5;
    pitch_corrigido = corrigePitch(pitch_mahony);
    if (--amostragem == 0) {
      amostragem = 5;
      vertValue = (pitch_corrigido - vertZero) * SENSIBILIDADE;
      vertZero = pitch_corrigido;
    }
    return vertValue;
  }

  void updateMouseReport() {
    mpu6050_GetData();
    filtraIMU();
    if (IMU_calibration()) {
      //MahonyAHRSupdateIMU( gxrs,  gyrs,  gzrs , axg,  ayg,  azg);
      MahonyAHRSupdateIMU(gyrs, gzrs, gxrs, ayg, azg, axg);
      getRollPitchYaw_mahony();
      int8_t xchg = mouseHoriz() * 2;
      int8_t ychg = mouseVert() * 2;

      report[0] = (!buttonState << 0) | (!button2State << 1);
      report[1] = xchg;
      report[2] = ychg;
      report[3] = 0;
      //Serial.printf("X: %d, Y: %d:\n", xchg, ychg);
    }
  }

  class MyServerCallbacks : public BLEServerCallbacks {
  private:
    MouseIMU* mouseIMU;
  public:
    MyServerCallbacks(MouseIMU* _mouseIMU)
      : mouseIMU(_mouseIMU) {}

    void onConnect(BLEServer* pServer) {
      mouseIMU->deviceConnected = true;

      Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
      mouseIMU->deviceConnected = false;

      Serial.println("Device disconnected");
      pServer->startAdvertising();
    }
  };
};

MouseIMU mouseIMU;

void setup() {
  mouseIMU.init();
}

void loop() {
  mouseIMU.update();
}
