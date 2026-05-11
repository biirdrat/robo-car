#include <SPI.h>
#include <RF24.h>

// Constant values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;
constexpr uint8_t DATA_PAYLOAD_SIZE = 4;
constexpr uint16_t SPI_NOT_OK_LED_TOGGLE_MS = 500;
constexpr uint16_t COMMS_NOT_OK_LED_TOGGLE_MS = 100;

// Transmission address
constexpr byte address[6] = "RADIO";

// Delays
unsigned long STATE_PROCESS_DELAY = 1;
unsigned long TRANSMIT_DELAY_MS = 10;
unsigned long COMMS_LOSS_DELAY_MS = 1000;
unsigned long RECONNECT_DELAY_MS = 1000;

// Comms timeout
constexpr uint16_t COMMS_TIMEOUT_MS = 2000;

// Joystick pins
constexpr uint8_t JOYSTICK_X_PIN  = 36;
constexpr uint8_t JOYSTICK_Y_PIN  = 39;
constexpr uint8_t JOYSTICK_BUTTON_PIN  = 25;

// Button Digital IO Pins
constexpr uint8_t BUT0_PIN = 34;
constexpr uint8_t BUT1_PIN = 35;
constexpr uint8_t BUT2_PIN = 32;
constexpr uint8_t BUT3_PIN = 33;

// RF24 control pins
constexpr uint8_t CE_PIN  = 4;
constexpr uint8_t CSN_PIN = 5;

// VSPI pins
constexpr uint8_t VSPI_SCK  = 18;
constexpr uint8_t VSPI_MISO = 19;
constexpr uint8_t VSPI_MOSI = 23;

enum class ManualState 
{
  START,
  CHECK_COMMS,
  GET_INPUT_DATA,
  PACK_DATA,
  TRANSMIT_DATA,
  TRANSMIT_DELAY,
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

// GPIO Pins
constexpr uint8_t COMMS_LED_PIN = 2;

char printBuffer[PRINT_BUFFER_SIZE];
uint8_t receiveBuffer[DATA_PAYLOAD_MAX_SIZE];
uint8_t sendBuffer[DATA_PAYLOAD_MAX_SIZE];

RF24 radioTransceiver(CE_PIN, CSN_PIN, 1000000);
SPIClass vspi(VSPI);

// Logic Variables
bool spiOk = false;
bool commsOk = true;
bool delayStarted = false;
uint8_t commsLEDStatus = false;
unsigned long lastStateProcessMs = 0;
unsigned long lastTransmitMs = 0;
unsigned long lastMessageReceivedMs = 0;
unsigned long delayStartMs = 0;
unsigned long lastCommsLEDToggleMs = 0;
bool but0Val = false;
bool but1Val = false;
bool but2Val = false;
bool but3Val = false;
bool joystickButtonVal = false;
uint16_t joystickXVal = 0;
uint16_t joystickYVal = 0;

void setup() 
{
  // Begin USB Serial
  Serial.begin(115200);

  initializeGPIOPins();

  initializeRadioSPITransceiver();

  lastMessageReceivedMs = millis();  
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
            if((millis() - lastMessageReceivedMs) < COMMS_TIMEOUT_MS)
            {
              TransitionToNextState(ManualState::GET_INPUT_DATA);
            }
            else
            {
              commsOk = false;
              TransitionToNextState(ManualState::COMMS_LOSS_DELAY);
            }
          }
          else
          {
            TransitionToNextState(ManualState::COMMS_LOSS_DELAY);
          }

          break;
        }

        case ManualState::GET_INPUT_DATA:
        {
            getInputData();
            TransitionToNextState(ManualState::PACK_DATA);
            break;
        }

        case ManualState::PACK_DATA:
        {
            packData();
            TransitionToNextState(ManualState::TRANSMIT_DATA);
            break;
        }

        case ManualState::TRANSMIT_DATA:
        { 
            radioSend(sendBuffer);
            TransitionToNextState(ManualState::TRANSMIT_DELAY);
            break;
        }

        case ManualState::TRANSMIT_DELAY:
        {
          if(!delayStarted)
          {
            delayStarted = true;
            delayStartMs = millis();
          }
          else
          {
            if((millis() - delayStartMs) >= TRANSMIT_DELAY_MS)
            {
              delayStarted = false;
              TransitionToNextState(ManualState::CHECK_COMMS);
            }
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
              TransitionToNextState(ManualState::GET_INPUT_DATA);
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
    pinMode(BUT0_PIN, INPUT);
    pinMode(BUT1_PIN, INPUT);
    pinMode(BUT2_PIN, INPUT);
    pinMode(BUT3_PIN, INPUT);
    pinMode(JOYSTICK_BUTTON_PIN, INPUT);
    pinMode(COMMS_LED_PIN, OUTPUT);
    digitalWrite(COMMS_LED_PIN, LOW);
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

void getInputData()
{
  joystickXVal = analogRead(JOYSTICK_X_PIN);
  joystickYVal = analogRead(JOYSTICK_Y_PIN);  

  but0Val = digitalRead(BUT0_PIN);
  but1Val = digitalRead(BUT1_PIN);
  but2Val = digitalRead(BUT2_PIN);
  but3Val = digitalRead(BUT3_PIN);
  joystickButtonVal = !digitalRead(JOYSTICK_BUTTON_PIN);
}

void packData()
{
  memset(sendBuffer, 0, DATA_PAYLOAD_SIZE);

  sendBuffer[0] = (joystickXVal >> 4) & 0xFF;

  sendBuffer[1] = ((joystickXVal & 0x0F) << 4) |
                  ((joystickYVal >> 8) & 0x0F);

  sendBuffer[2] = joystickYVal & 0xFF;

  sendBuffer[3] = (but0Val << 0) |
                  (but1Val << 1) |
                  (but2Val << 2) |
                  (but3Val << 3) |
                  (joystickButtonVal << 4);
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

void printToSerial(const char *fmt, ...) 
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(printBuffer, PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  Serial.print(printBuffer);
}