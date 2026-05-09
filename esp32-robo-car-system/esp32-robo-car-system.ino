#include <SPI.h>
#include <RF24.h>

// Constant values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;

// Transmission address
constexpr byte address[6] = "RADIO";

// Delays
unsigned long STATE_PROCESS_DELAY = 1;
unsigned long TRANSMIT_DELAY_MS = 1000;
unsigned long COMMS_LOSS_DELAY_MS = 1000;
unsigned long RECONNECT_DELAY_MS = 1000;

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
constexpr uint16_t MOTOR_PWM_FREQ  = 200;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 12;
constexpr uint16_t MOTOR_PWM_MAX = 4095;
constexpr uint16_t MOTOR_PWM_OFF = 0;
constexpr uint8_t R_MOTOR_PWM_CHANNEL = 1;
constexpr uint8_t L_MOTOR_PWM_CHANNEL = 2;

enum class ManualState 
{
  START,
  COMMS_CHECK,
  COMMS_LOSS_DELAY,
  RECONNECT_COMMS,
  RECONNECT_DELAY
};

enum class AutoState 
{
  START
};

ManualState currentManualState = ManualState::START;
AutoState currentAutoState = AutoState::START;

char printBuffer[PRINT_BUFFER_SIZE];
char receiveBuffer[DATA_PAYLOAD_MAX_SIZE + 1];
char sendBuffer[DATA_PAYLOAD_MAX_SIZE + 1];

RF24 radioTransceiver(CE_PIN, CSN_PIN, 1000000);
SPIClass vspi(VSPI);

bool spiOk = false;
bool commsOk = true;
bool delayStarted = false;
unsigned long lastStateProcessMs = 0;
unsigned long lastTransmitMs = 0;
unsigned long lastMessageReceivedMs = 0;
unsigned long delayStartMs = 0;

void setup() 
{
  // Begin USB Serial
  Serial.begin(115200);

  initializeGPIOPins();

  initializeBuzzer();

  initializeMotors();

  initializeRadioSPITransceiver();

}

void loop()
{
  // Manual State Machine Process Loop
  if((millis() - lastStateProcessMs) >= STATE_PROCESS_DELAY)
  {
    switch (currentManualState) 
    {
        case ManualState::START:
        {
            TransitionToNextState(ManualState::COMMS_CHECK);
            break;
        }

        case ManualState::COMMS_CHECK:
        {
            if (spiOk && commsOk)
            {

            }
            else
            {
                TransitionToNextState(ManualState::COMMS_LOSS_DELAY);
            }

            break;
        }

        case ManualState::COMMS_LOSS_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
          }
          else
          {
            if((millis() - delayStartMs) >= COMMS_LOSS_DELAY_MS)
            {
              delayStarted = false;
              TransitionToNextState(ManualState::RECONNECT_COMMS);
            }
          }
          break;
        }

        case ManualState::RECONNECT_COMMS:
        {
            if(!spiOk)
            {
              if(!reinitializeRadioSPITransceiver())
              {
                printToSerial("Failed to reinitialize NRF24l01 module. Retrying...\n");
                TransitionToNextState(ManualState::RECONNECT_DELAY);
              }
            }
            else if(spiOk && commsOk)
            {
              TransitionToNextState(ManualState::COMMS_CHECK);
            }
            break;
        }

        case ManualState::RECONNECT_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
          }
          else
          {
            if((millis() - delayStartMs) >= RECONNECT_DELAY_MS)
            {
              delayStarted = false;
              TransitionToNextState(ManualState::RECONNECT_COMMS);
            }
          }

          break;
        }
    }
  }

  if(spiOk && radioReceive())
  {
    commsOk = true;
    lastMessageReceivedMs = millis();
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
  digitalWrite(R_MOTOR_IN1_PIN, HIGH);
  digitalWrite(R_MOTOR_IN2_PIN, LOW);
  digitalWrite(L_MOTOR_IN3_PIN, HIGH);
  digitalWrite(L_MOTOR_IN4_PIN, LOW);

  // Turn off motors
  ledcWrite(R_MOTOR_PWM_CHANNEL, MOTOR_PWM_OFF);
  ledcWrite(L_MOTOR_PWM_CHANNEL, MOTOR_PWM_OFF);
}

void initializeRadioSPITransceiver()
{
  vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, CSN_PIN);

  while(!radioTransceiver.begin(&vspi)) 
  {
    printToSerial("Failed to initialize NRF24l01 module. Retrying...\n");
    delay(1000);
  }

  radioTransceiver.setPALevel(RF24_PA_MIN);
  radioTransceiver.setDataRate(RF24_250KBPS);

  // Open both pipes
  radioTransceiver.openWritingPipe(address);
  radioTransceiver.openReadingPipe(1, address);


  radioTransceiver.startListening();

  spiOk = true;

  printToSerial("NRF24l01 module initialized successfully.\n");
}

bool reinitializeRadioSPITransceiver()
{
  if(!radioTransceiver.begin(&vspi)) 
  {
    return false;
  }

  radioTransceiver.setPALevel(RF24_PA_MIN);
  radioTransceiver.setDataRate(RF24_250KBPS);

  // Open both pipes
  radioTransceiver.openWritingPipe(address);
  radioTransceiver.openReadingPipe(1, address);


  radioTransceiver.startListening();

  spiOk = true;

  printToSerial("NRF24l01 module reinitialized successfully.\n");
  
  return true;
}

void TransitionToNextState(ManualState nextState)
{
    currentManualState = nextState;
}

void TransitionToNextState(AutoState nextState)
{
    currentAutoState = nextState;
}

bool radioReceive()
{
    if(!radioTransceiver.available())
    {
        return false;
    }

    size_t len = radioTransceiver.getDynamicPayloadSize();

    if(len == 0)
    {
      printToSerial("SPI read command failed, communication is lost.\n", (int)len);
      spiOk = false;
      return false;
    }
    else if (len > DATA_PAYLOAD_MAX_SIZE)
    {
      printToSerial("Corrupted/Invalid packet received with length: %i\n", (int)len);
      spiOk = false;
      return false;  
    }

    radioTransceiver.read(receiveBuffer, len);

    // Null terminate for safe string use
    receiveBuffer[len] = '\0';
  
    return true;
}

bool radioSend(const char *dataPayload)
{
    // Determine usable length (max SEND_BUFFER_SIZE bytes)
    size_t len = strnlen(dataPayload, DATA_PAYLOAD_MAX_SIZE);

    if (len == 0)
    {
        printToSerial("Data Payload is empty.\n");
        return false;
    }

    // Copy into sendBuffer
    memcpy(sendBuffer, dataPayload, len);

    // Null terminate sendBuffer
    sendBuffer[len] = '\0';

    // Switch to TX
    radioTransceiver.stopListening();

    bool success = radioTransceiver.write(sendBuffer, len);

    // Switch Back to RX
    radioTransceiver.startListening();

    if(success)
    {
      return true;
    }
    else
    {
      return false;
    }
}

void printToSerial(const char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printBuffer, PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  Serial.print(printBuffer);
}