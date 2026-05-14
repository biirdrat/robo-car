#include <SPI.h>
#include <RF24.h>

// Constant values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;
constexpr uint8_t DATA_PAYLOAD_SIZE = 1;
constexpr uint16_t SPI_NOT_OK_LED_TOGGLE_MS = 500;
constexpr uint16_t COMMS_NOT_OK_LED_TOGGLE_MS = 100;
constexpr uint16_t STOP_TIME_MS = 500;
constexpr uint16_t CONTROLLER_X_CENTER = 1850;
constexpr uint16_t CONTROLLER_Y_CENTER = 1900;
constexpr uint16_t CONTROLLER_X_MAX = 4095;
constexpr uint16_t CONTROLLER_Y_MAX = 4095;
constexpr uint16_t CONTROLLER_X_MIN = 0;
constexpr uint16_t CONTROLLER_Y_MIN = 0;
constexpr uint16_t CONTROLLER_DEADZONE = 50;
constexpr float CONTROLLER_X_POS_NORMALIZE_SCALE =
    1.0f / (CONTROLLER_X_MAX - CONTROLLER_X_CENTER);
constexpr float CONTROLLER_X_NEG_NORMALIZE_SCALE =
    1.0f / (CONTROLLER_X_CENTER - CONTROLLER_X_MIN);
constexpr float CONTROLLER_Y_POS_NORMALIZE_SCALE =
    1.0f / (CONTROLLER_Y_MAX - CONTROLLER_Y_CENTER);
constexpr float CONTROLLER_Y_NEG_NORMALIZE_SCALE=
    1.0f / (CONTROLLER_Y_CENTER - CONTROLLER_Y_MIN);
constexpr float CONTROL_EXPONENTIAL_RESPONSE = 0.5f;

// Transmission address
constexpr byte address[6] = "RADIO";

// Delays
unsigned long STATE_PROCESS_DELAY = 1;
unsigned long TRANSMIT_DELAY_MS = 100;
unsigned long COMMS_CLEANUP_AND_DELAY_MS = 1000;
unsigned long ATTEMPT_RECONNECT_DELAY_MS = 1000;
unsigned long DRIVE_VEHICLE_DELAY_MS = 20;
unsigned long CHECK_BUTTONS_DELAY_MS = 50;

// Comms timeout
constexpr uint16_t COMMS_TIMEOUT_MS = 1000;

// RF24 control pins
constexpr uint8_t CE_PIN  = 4;
constexpr uint8_t CSN_PIN = 5;

// VSPI pins
constexpr uint8_t VSPI_SCK  = 18;
constexpr uint8_t VSPI_MISO = 19;
constexpr uint8_t VSPI_MOSI = 23;

// GPIO Pins
constexpr uint8_t COMMS_LED_PIN = 2;
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
constexpr uint16_t MOTOR_PWM_MIN = 700;
constexpr uint16_t MOTOR_PWM_MAX = 4095;
constexpr uint16_t MOTOR_PWM_OFF = 0;
constexpr uint8_t R_MOTOR_PWM_CHANNEL = 1;
constexpr uint8_t L_MOTOR_PWM_CHANNEL = 2;

enum class ManualState 
{
  START,
  CHECK_COMMS,
  GET_CONTROLLER_DATA,
  PROCESS_MOVEMENT,
  DRIVE_VEHICLE,
  DRIVE_VEHICLE_DELAY,
  COMMS_CLEANUP_AND_DELAY,
  RECONNECT_COMMS,
  ATTEMPT_RECONNECT_DELAY
};

enum class AutoState 
{
  START
};

ManualState currentManualState = ManualState::START;
AutoState currentAutoState = AutoState::START;

char printBuffer[PRINT_BUFFER_SIZE];
uint8_t receiveBuffer[DATA_PAYLOAD_MAX_SIZE];
uint8_t sendBuffer[DATA_PAYLOAD_MAX_SIZE];

RF24 radioTransceiver(CE_PIN, CSN_PIN, 1000000);
SPIClass vspi(VSPI);

// Logic Variables
bool spiOk = false;
bool commsOk = false;
bool delayStarted = false;
uint8_t commsLEDStatus = false;
unsigned long lastStateProcessMs = 0;
unsigned long lastTransmitMs = 0;
unsigned long lastMessageReceivedMs = 0;
unsigned long delayStartMs = 0;
unsigned long lastCommsLEDToggleMs = 0;
unsigned long lastButtonsCheckMs = 0;
unsigned long lastMovementMs = 0;
uint8_t lightMode = 0;
bool lightsOn = false;
bool vehicleStopped = true;

// Controller Data Variables
bool but0Val = false;
bool but1Val = false;
bool but2Val = false;
bool but3Val = false;
bool joystickButtonVal = false;
uint16_t controllerXVal = 0;
uint16_t controllerYVal = 0;
bool prevBut0Val = false;
bool prevBut1Val = false;
bool prevBut2Val = false;
bool prevBut3Val = false;
bool prevJoystickButtonVal = false;
float normalizedControlXVal = 0.0;
float normalizedControlYVal = 0.0;

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
            TransitionToNextState(ManualState::CHECK_COMMS);
            break;
        }

        case ManualState::CHECK_COMMS:
        {
          // Check SPI
          if(spiOk)
          {
            // Check if messages are still being received
            if((millis() - lastMessageReceivedMs) >= COMMS_TIMEOUT_MS)
            {
              commsOk = false;
            }

            if(commsOk)
            {
              TransitionToNextState(ManualState::GET_CONTROLLER_DATA);
            }
            else
            {
              commsOk = false;
              TransitionToNextState(ManualState::COMMS_CLEANUP_AND_DELAY);
            }
          }
          else
          {
            TransitionToNextState(ManualState::COMMS_CLEANUP_AND_DELAY);
          }

          break;
        }

        case ManualState::GET_CONTROLLER_DATA:
        {
          getControllerData();
    
          TransitionToNextState(ManualState::PROCESS_MOVEMENT);
          break;
        }

        case ManualState::PROCESS_MOVEMENT:
        {
          processMovement();

          TransitionToNextState(ManualState::DRIVE_VEHICLE);
          break;
        }

        case ManualState::DRIVE_VEHICLE:
        {
          driveVehicle();

          TransitionToNextState(ManualState::DRIVE_VEHICLE_DELAY);
          break;
        }

        case ManualState::DRIVE_VEHICLE_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
          }
          else
          {
            if((millis() - delayStartMs) >= DRIVE_VEHICLE_DELAY_MS)
            {
              delayStarted = false;
              TransitionToNextState(ManualState::CHECK_COMMS);
            }
          }

          break;
        }

        case ManualState::COMMS_CLEANUP_AND_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
            lightsOn = false;
            setMotor(
              R_MOTOR_PWM_CHANNEL,
              R_MOTOR_IN1_PIN,
              R_MOTOR_IN2_PIN,
              0.0f);
            setMotor(
                L_MOTOR_PWM_CHANNEL,
                L_MOTOR_IN3_PIN,
                L_MOTOR_IN4_PIN,
                0.0f);
            digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, LOW);
            digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, LOW);
            ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_PWM_OFF);     
          }
          else
          {
            if((millis() - delayStartMs) >= COMMS_CLEANUP_AND_DELAY_MS)
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
                TransitionToNextState(ManualState::ATTEMPT_RECONNECT_DELAY);
              }
            }
            else if(spiOk && commsOk)
            {
              TransitionToNextState(ManualState::CHECK_COMMS);
            }
            break;
        }

        case ManualState::ATTEMPT_RECONNECT_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
          }
          else
          {
            if((millis() - delayStartMs) >= ATTEMPT_RECONNECT_DELAY_MS)
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

  if(spiOk && ((millis() - lastTransmitMs) >= TRANSMIT_DELAY_MS))
  {
    uint8_t value = 0x0;
    radioSend(&value);
    lastTransmitMs = millis();
  }

  // Buttons handling
  if((millis() - lastButtonsCheckMs) >= CHECK_BUTTONS_DELAY_MS)
  { 
    // Button 0 pressed
    if(!prevBut0Val && but0Val)
    {
      lightsOn = !lightsOn;
      if(lightsOn)
      {
        switch(lightMode)
        {
            case 0:
            {
              digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, HIGH);
              break;
            }

            case 1:
            {
              digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, HIGH);
              digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, HIGH);
              break;
            }

            case 2:
            {
              digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, HIGH);
              break;
            }
          }
        }
        else
        {
          digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, LOW);
          digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, LOW);
        }
    }

    // Button 1 pressed
    if(!prevBut1Val && but1Val)
    {
      lightMode = (lightMode + 1) % 3;
      if(lightsOn)
      {
        switch(lightMode)
        {
            case 0:
            {
              digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, HIGH);
              digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, LOW);
              break;
            }

            case 1:
            {
              digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, HIGH);
              digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, HIGH);
              break;
            }

            case 2:
            {
              digitalWrite(BLUE_LIGHTS_ACTIVATE_PIN, HIGH);
              digitalWrite(WHITE_LIGHTS_ACTIVATE_PIN, LOW);
              break;
            }
          }
        }
    }

    // Button 2 pressed
    if(!prevBut2Val && but2Val)
    {
      ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_PWM_ON);
    }
    // Button 2 released 
    else if(prevBut2Val && !but2Val)
    { 
      ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_PWM_OFF);     
    }

    // Button 3 pressed
    if(!prevBut3Val && but3Val)
    {

    }

    // Joystick button pressed
    if(!prevJoystickButtonVal && joystickButtonVal)
    {

    }

    prevBut0Val = but0Val;
    prevBut1Val = but1Val;
    prevBut2Val = but2Val;
    prevBut3Val = but3Val;
    prevJoystickButtonVal = joystickButtonVal;

    lastButtonsCheckMs = millis();
  }

  // Check if vehicle is stopped
  if((millis() - lastMovementMs) >= STOP_TIME_MS)
  {
    vehicleStopped = true;
  }
  else
  {
    vehicleStopped = false;
  }

  // Comms LED Handling
  if(!spiOk)
  {
    if((millis() - lastCommsLEDToggleMs) >= SPI_NOT_OK_LED_TOGGLE_MS)
    {
      commsLEDStatus = !commsLEDStatus;
      digitalWrite(COMMS_LED_PIN, commsLEDStatus);
      lastCommsLEDToggleMs = millis();
    }
  }
  else if(!commsOk)
  {
    if((millis() - lastCommsLEDToggleMs) >= COMMS_NOT_OK_LED_TOGGLE_MS)
    {
      commsLEDStatus = !commsLEDStatus;
      digitalWrite(COMMS_LED_PIN, commsLEDStatus);
      lastCommsLEDToggleMs = millis();
    }
  }
  else
  {
    digitalWrite(COMMS_LED_PIN, HIGH);
  }
}

void initializeGPIOPins()
{
  // Comms LED Pin
  pinMode(COMMS_LED_PIN, OUTPUT);
  digitalWrite(COMMS_LED_PIN, LOW);

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
  radioTransceiver.setRetries(1, 3);

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
  radioTransceiver.setRetries(1, 3);

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
  
    return true;
}

bool radioSend(const uint8_t *dataPayload)
{
    // Copy into sendBuffer
    memcpy(sendBuffer, dataPayload, DATA_PAYLOAD_SIZE);

    // Switch to TX
    radioTransceiver.stopListening();

    bool success = radioTransceiver.write(sendBuffer, DATA_PAYLOAD_SIZE);

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

void getControllerData()
{
  // Get joystick analog values
  controllerXVal = ((uint16_t)receiveBuffer[0] << 4) | ((receiveBuffer[1] >> 4) & 0x0F);
  controllerYVal = (((uint16_t)receiveBuffer[1] & 0x0F) << 8) | receiveBuffer[2];

  // Get button values
  but0Val = receiveBuffer[3] & (1 << 0);
  but1Val = receiveBuffer[3] & (1 << 1);
  but2Val = receiveBuffer[3] & (1 << 2);
  but3Val = receiveBuffer[3] & (1 << 3);
  joystickButtonVal = receiveBuffer[3] & (1 << 4);
}

void processMovement()
{
  int16_t xOffset = (int16_t)controllerXVal - CONTROLLER_X_CENTER;
  int16_t yOffset = (int16_t)controllerYVal - CONTROLLER_Y_CENTER;

  if(abs(xOffset) < CONTROLLER_DEADZONE)
  {
      normalizedControlXVal = 0.0f;
  }
  else
  {
    if(xOffset >= 0)
    {
      normalizedControlXVal = xOffset * CONTROLLER_X_POS_NORMALIZE_SCALE;
    }
    else
    {
      normalizedControlXVal = xOffset * CONTROLLER_X_NEG_NORMALIZE_SCALE;
    }    
  }

  if(abs(yOffset) < CONTROLLER_DEADZONE)
  {
      normalizedControlYVal = 0.0f;
  }
  else
  {
      if(yOffset >= 0)
      {
          normalizedControlYVal = yOffset * CONTROLLER_Y_POS_NORMALIZE_SCALE;
      }
      else
      {
          normalizedControlYVal = yOffset * CONTROLLER_Y_NEG_NORMALIZE_SCALE;
      }
  }

  if(normalizedControlXVal > 0.0f || normalizedControlYVal > 0.0f)
  {
    lastMovementMs = millis();
  }
}

void setMotor(
    uint8_t pwmChannel,
    uint8_t forwardPin,
    uint8_t backwardPin,
    float speed)
{
    // Clamp
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;

    float magnitude = fabs(speed);

    uint16_t pwm = 0;

    // Apply minimum usable PWM
    if(magnitude > 0.0f)
    {
      pwm = MOTOR_PWM_MIN +
            (uint16_t)((MOTOR_PWM_MAX - MOTOR_PWM_MIN) * magnitude);

      if(pwm > MOTOR_PWM_MAX)
      {
          pwm = MOTOR_PWM_MIN;
      }
    }

    // Direction control
    if(speed > 0.0f)
    {
      digitalWrite(forwardPin, HIGH);
      digitalWrite(backwardPin, LOW);
    }
    else if(speed < 0.0f)
    {
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, HIGH);
    }
    else
    {
      digitalWrite(forwardPin, LOW);
      digitalWrite(backwardPin, LOW);
    }

    ledcWrite(pwmChannel, pwm);
}

float applySensitivityCurve(float input)
{

    return ((1.0f - CONTROL_EXPONENTIAL_RESPONSE) * input) +
           (CONTROL_EXPONENTIAL_RESPONSE * input * input * input);
}

void driveVehicle()
{
    float x = applySensitivityCurve(normalizedControlXVal);
    float y = applySensitivityCurve(normalizedControlYVal);

    // Reverse steering correction
    if(y < 0.0f)
    {
        x = -x;
    }

    float leftMotorSpeed  = y + x;
    float rightMotorSpeed = y - x;

    // Clamp
    if(leftMotorSpeed > 1.0f) leftMotorSpeed = 1.0f;
    if(leftMotorSpeed < -1.0f) leftMotorSpeed = -1.0f;

    if(rightMotorSpeed > 1.0f) rightMotorSpeed = 1.0f;
    if(rightMotorSpeed < -1.0f) rightMotorSpeed = -1.0f;

    setMotor(
        R_MOTOR_PWM_CHANNEL,
        R_MOTOR_IN1_PIN,
        R_MOTOR_IN2_PIN,
        rightMotorSpeed);

    setMotor(
        L_MOTOR_PWM_CHANNEL,
        L_MOTOR_IN3_PIN,
        L_MOTOR_IN4_PIN,
        leftMotorSpeed);
}

void printToSerial(const char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printBuffer, PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  Serial.print(printBuffer);
}