#include <SPI.h>
#include <RF24.h>

// Constant values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;

// Transmission address
constexpr byte address[6] = "RADIO";

// RF24 control pins
constexpr uint8_t CE_PIN  = 4;
constexpr uint8_t CSN_PIN = 5;

// VSPI pins
constexpr uint8_t VSPI_SCK  = 18;
constexpr uint8_t VSPI_MISO = 19;
constexpr uint8_t VSPI_MOSI = 23;

// GPIO Pins
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t BUZZER_PIN  = 15;
constexpr uint8_t WHITE_LIGHTS_ACTIVATE_PIN = 13;
constexpr uint8_t BLUE_LIGHTS_ACTIVATE_PIN = 12;
constexpr uint8_t R_MOTOR_ENA_PWM_PIN = 14;
constexpr uint8_t R_MOTOR_IN1_PIN = 27;
constexpr uint8_t R_MOTOR_IN2_PIN = 26;
constexpr uint8_t L_MOTOR_ENB_PWM_PIN = 25;
constexpr uint8_t L_MOTOR_IN3_PIN = 33;
constexpr uint8_t L_MOTOR_IN4_PIN = 32;


// Buzzer PWM Settings
constexpr uint16_t BUZZER_FREQ  = 400;
constexpr uint8_t BUZZER_PWM_RESOLUTION = 12;
constexpr uint8_t BUZZER_PWM_CHANNEL = 5;
constexpr uint16_t BUZZER_PWM_ON = 2048;
constexpr uint8_t BUZZER_PWM_OFF = 0;

// Motor PWM Settings
constexpr uint16_t MOTOR_PWM_FREQ  = 1000;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 12;
constexpr uint16_t MOTOR_PWM_MAX = 4095;
constexpr uint16_t MOTOR_PWM_OFF = 0;
constexpr uint8_t R_MOTOR_PWM_CHANNEL = 1;
constexpr uint8_t L_MOTOR_PWM_CHANNEL = 2;

char printBuffer[PRINT_BUFFER_SIZE];
char readBuffer[DATA_PAYLOAD_MAX_SIZE + 1];

RF24 radioReceiver(CE_PIN, CSN_PIN, 1000000);
SPIClass vspi(VSPI);

void setup() 
{
  // Begin USB Serial
  Serial.begin(115200);

  initializeGPIOPins();

  initializeBuzzer();

  initializeMotors();

  initializeRadioVSPIReceiver();
 
  // Turn onboard LED is initialization passed
  digitalWrite(LED_PIN, HIGH);

}

void loop()
{
  if(radioReceive())
  {
    Serial.println(readBuffer);
  }
}

void initializeGPIOPins()
{
  // LED Pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Vehicle LEDs Pins
  pinMode(WHITE_LIGHTS_ACTIVATE_PIN, OUTPUT);
  pinMode(BLUE_LIGHTS_ACTIVATE_PIN, OUTPUT);
  digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, LOW);
  digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, LOW);
}

void initializeBuzzer()
{
  ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_FREQ, BUZZER_PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
  ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_PWM_OFF);
}

void initializeMotors()
{
  // PWM setup
  ledcSetup(R_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(L_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(R_MOTOR_ENA_PWM_PIN, R_MOTOR_PWM_CHANNEL);
  ledcAttachPin(L_MOTOR_ENB_PWM_PIN, L_MOTOR_PWM_CHANNEL);

  // Set motor direction pins as outputs
  pinMode(R_MOTOR_IN1_PIN, OUTPUT);
  pinMode(R_MOTOR_IN2_PIN, OUTPUT);
  pinMode(L_MOTOR_IN3_PIN, OUTPUT);
  pinMode(L_MOTOR_IN4_PIN, OUTPUT);

    // Set Direction Pins Low
  digitalWrite(R_MOTOR_IN1_PIN, LOW);
  digitalWrite(R_MOTOR_IN2_PIN, LOW);
  digitalWrite(L_MOTOR_IN3_PIN, LOW);
  digitalWrite(L_MOTOR_IN4_PIN, LOW);

  // Turn off motors
  ledcWrite(R_MOTOR_PWM_CHANNEL, MOTOR_PWM_OFF);
  ledcWrite(L_MOTOR_PWM_CHANNEL, MOTOR_PWM_OFF);
}

void initializeRadioVSPIReceiver()
{
  vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, CSN_PIN);

  while (!radioReceiver.begin(&vspi)) 
  {
    printToSerial("Failed to initialize NRF24l01 module. Retrying...\n");
    delay(1000);
  }

  radioReceiver.openReadingPipe(1, address);
  radioReceiver.setPALevel(RF24_PA_MIN);
  radioReceiver.setDataRate( RF24_250KBPS );
  radioReceiver.startListening();

  printToSerial("NRF24l01 module initialized successfully.\n");
}

bool radioReceive()
{
    if (!radioReceiver.available())
    {
        return false;
    }

    size_t len = radioReceiver.getDynamicPayloadSize();

    if (len == 0 || len > DATA_PAYLOAD_MAX_SIZE)
    {
      printToSerial("Corrupted/Invalid packet received with length: %i\n", (int)len);
      len = DATA_PAYLOAD_MAX_SIZE;
    }

    radioReceiver.read(readBuffer, len);

    // Null terminate for safe string use
    readBuffer[len] = '\0';
  
    return true;
}

void printToSerial(const char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printBuffer, PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  Serial.print(printBuffer);
}