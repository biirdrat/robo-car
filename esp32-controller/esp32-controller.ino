#include <SPI.h>
#include <RF24.h>

// Constant numerical values
constexpr uint16_t PRINT_BUFFER_SIZE = 2000;
constexpr uint8_t DATA_PAYLOAD_MAX_SIZE = 32;

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

RF24 radioSender(CE_PIN, CSN_PIN);
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
  // put your main code here, to run repeatedly:

}

void initializeGPIOPins()
{
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