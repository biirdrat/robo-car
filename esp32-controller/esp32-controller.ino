#include <SPI.h>
#include <RF24.h>

// Constant values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;

// Transmission address
constexpr byte address[6] = "RADIO";

// Analog pins
constexpr uint8_t JOYSTICK_X_PIN  = 36;
constexpr uint8_t JOYSTICK_Y_PIN  = 39;

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

// GPIO Pins
constexpr uint8_t LED_PIN = 2;

char printBuffer[PRINT_BUFFER_SIZE];
char sendBuffer[DATA_PAYLOAD_MAX_SIZE + 1];

RF24 radioSender(CE_PIN, CSN_PIN, 100000);
SPIClass vspi(VSPI);

void setup() 
{
  // Begin USB Serial
  Serial.begin(115200);

  initializeGPIOPins();
  initializeRadioVSPISender();

  // Turn onboard LED is initialization passed
  digitalWrite(LED_PIN, HIGH);
}

void loop() 
{
  radioSend("Hello");
  int raw1 = analogRead(JOYSTICK_X_PIN);
  int raw2 = analogRead(JOYSTICK_Y_PIN);
  
  printToSerial("%i %i\n", raw1, raw2);

  if(digitalRead(BUT1_PIN))
  {
    printToSerial("BUT1 PRESSED\n");
  }
  if(digitalRead(BUT2_PIN))
  {
    printToSerial("BUT2 PRESSED\n");
  }
  if(digitalRead(BUT3_PIN))
  {
    printToSerial("BUT3 PRESSED\n");
  }
  if(digitalRead(BUT4_PIN))
  {
    printToSerial("BUT4 PRESSED\n");
  }

  delay(1000);
}

void initializeGPIOPins()
{
    pinMode(BUT1_PIN, INPUT);
    pinMode(BUT2_PIN, INPUT);
    pinMode(BUT3_PIN, INPUT);
    pinMode(BUT4_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
}

void initializeRadioVSPISender()
{
  vspi.begin(VSPI_SCK, VSPI_MISO, VSPI_MOSI, CSN_PIN);

  while (!radioSender.begin(&vspi)) 
  {
    printToSerial("Failed to initialize NRF24l01 module. Retrying...\n");
    delay(1000);
  }

  radioSender.openWritingPipe(address);
  radioSender.setPALevel(RF24_PA_MIN);
  radioSender.setDataRate(RF24_250KBPS);
  radioSender.stopListening();
  
  printToSerial("NRF24l01 module initialized successfully.");
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

    bool success = radioSender.write(sendBuffer, len);

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