#include <LoRa_DSR.h>

#define LORA_BAND   920E6 // LoRa Band (Thailand)
#define PABOOST     true

byte currentID      = 93;        // This node address
byte destinationID  = 255;       // Original destination (0xFF broadcast)

LoRa_DSR DSR(currentID, destinationID, 2, true); // (_currentID, _destinationID, DeviceType = 1, _enableDebug = false)
                                    // 0 = End device low battery
                                    // 1 = Cluster station
                                    // 2 = Internet cluster station
                                    // 3 = Not connect

void setup()
{
  Serial.begin(115200);

  DSR.configForLoRaWAN(20, 12, 500E3, 5); // (_TXPOWER = 20, _SPREADING_FACTOR = 12, _BANDWIDTH = 500E3, _CODING_RATE = 5)
  DSR.begin(LORA_BAND); // (_LORA_BAND = 868E6, _PABOOST = true)
}

void loop()
{
  DSR.check_timer();

  String received = DSR.checkPacket();
  if (received != ""){
    Serial.println(received);
  }
}