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

// Joystick pins
constexpr uint8_t JOYSTICK_X_PIN  = 36;
constexpr uint8_t JOYSTICK_Y_PIN  = 39;
constexpr uint8_t JOYSTICK_BUTTON_PIN  = 25;

// Button Digital IO Pins
constexpr uint8_t BUT1_PIN = 34;
constexpr uint8_t BUT2_PIN = 35;
constexpr uint8_t BUT3_PIN = 32;
constexpr uint8_t BUT4_PIN = 33;

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
  COMMS_CHECK,
  GET_INPUT_DATA,
  PACK_DATA,
  TRANSMIT_DATA,
  TRANSMIT_DELAY,
  COMMS_LOSS_DELAY,
  RECONNECT_COMMS
};

enum class AutoState 
{
  START
};

ManualState currentManualState = ManualState::START;
AutoState currentAutoState = AutoState::START;

// GPIO Pins
constexpr uint8_t LED_PIN = 2;

char printBuffer[PRINT_BUFFER_SIZE];
char receiveBuffer[DATA_PAYLOAD_MAX_SIZE + 1];
char sendBuffer[DATA_PAYLOAD_MAX_SIZE + 1];

RF24 radioTransceiver(CE_PIN, CSN_PIN, 1000000);
SPIClass vspi(VSPI);

bool spiOk = false;
bool commsIsLost = false;
bool delayStarted = false;
unsigned long currentLoopMs = 0;
unsigned long lastStateProcessMs = 0;
unsigned long lastTransmitMs = 0;
unsigned long lastMessageReceivedMs = 0;

void setup() 
{
  // Begin USB Serial
  Serial.begin(115200);

  initializeGPIOPins();
  initializeRadioVSPITransceiver();

  lastMessageReceivedMs = millis();  
}

void loop() 
{
  currentLoopMs = millis();

  // Manual State Machine Process Loop
  if((currentLoopMs - lastStateProcessMs) >= STATE_PROCESS_DELAY)
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
            if (spiOk && !commsIsLost)
            {
                TransitionToNextState(ManualState::GET_INPUT_DATA);
            }
            else
            {
                TransitionToNextState(ManualState::COMMS_LOSS_DELAY);
            }

            break;
        }

        case ManualState::GET_INPUT_DATA:
        {

            break;
        }

        case ManualState::PACK_DATA:
        {

            break;
        }

        case ManualState::TRANSMIT_DATA:
        {

            break;
        }

        case ManualState::TRANSMIT_DELAY:
        {

            break;
        }

        case ManualState::COMMS_LOSS_DELAY:
        {

            break;
        }

        case ManualState::RECONNECT_COMMS:
        {

            break;
        }
    }
  }

  if(spiOk && radioReceive())
  {
    commsIsLost = false;
    lastMessageReceivedMs = millis();
  }
}

void initializeGPIOPins()
{
    pinMode(BUT1_PIN, INPUT);
    pinMode(BUT2_PIN, INPUT);
    pinMode(BUT3_PIN, INPUT);
    pinMode(BUT4_PIN, INPUT);
    pinMode(JOYSTICK_BUTTON_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void initializeRadioVSPITransceiver()
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

void reinitializeRadioVSPITransceiver()
{
  while(!radioTransceiver.begin(&vspi)) 
  {
    printToSerial("Failed to reinitialize NRF24l01 module. Retrying...\n");
    delay(1000);
  }

  radioTransceiver.setPALevel(RF24_PA_MIN);
  radioTransceiver.setDataRate(RF24_250KBPS);

  // Open both pipes
  radioTransceiver.openWritingPipe(address);
  radioTransceiver.openReadingPipe(1, address);


  radioTransceiver.startListening();

  spiOk = true;

  printToSerial("NRF24l01 module reinitialized successfully.\n");
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