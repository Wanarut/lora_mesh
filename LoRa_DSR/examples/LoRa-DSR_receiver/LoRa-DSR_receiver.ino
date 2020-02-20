#include <LoRa_DSR.h>

#define LORA_BAND   920E6 // LoRa Band (Thailand)
#define PABOOST     true

byte currentID      = 93;        // This node address
byte destinationID  = 255;       // Original destination (0xFF broadcast)

LoRa_DSR DSR(currentID, destinationID, 2/*, true*/); // (_currentID, _destinationID, DeviceType = 1, _enableDebug = false)
                                    // 0 = End device low battery
                                    // 1 = Cluster station
                                    // 2 = Internet cluster station
                                    // 3 = Not connect

void setup()
{
  Serial.begin(115200);

  DSR.configForLoRaWAN(14, 12, 125000, 5, 8, 0x34); // (_TXPOWER = 14, _SPREADING_FACTOR = 12, _BANDWIDTH = 125000, _CODING_RATE = 5, _PREAMBLE_LENGTH = 8, _SYNC_WORD = 0x34)
  delay(1000);
  DSR.begin(LORA_BAND); // (_LORA_BAND = 868E6, _PABOOST = true)

  Serial.println(F("Start LoRa DSR receiver"));
}

void loop()
{
  long cur_time = millis();
  DSR.check_timer(cur_time);

  String received = DSR.checkPacket();
  if (received != ""){
    Serial.print(F("Received -> "));
    Serial.println(received);
  }
}