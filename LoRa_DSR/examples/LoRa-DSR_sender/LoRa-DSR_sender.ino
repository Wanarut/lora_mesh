#include <LoRa_DSR.h>

#define LORA_BAND   920E6 // LoRa Band (Thailand)
#define PABOOST     true

byte currentID      = 91;        // This node address
byte destinationID  = 93;        // Original destination (0xFF broadcast)

LoRa_DSR DSR(currentID, destinationID, 1/*, true*/); // (_currentID, _destinationID, DeviceType = 1, _enableDebug = false)
                                    // 0 = End device low battery
                                    // 1 = Cluster station
                                    // 2 = Internet cluster station
                                    // 3 = Not connect

int interval = 30000;       // interval between sends

int counter = 0;
int success = 0;
void setup()
{
  Serial.begin(115200);

  DSR.configForLoRaWAN(14, 12, 125000, 5, 8, 0x34); // (_TXPOWER = 14, _SPREADING_FACTOR = 12, _BANDWIDTH = 125000, _CODING_RATE = 5, _PREAMBLE_LENGTH = 8, _SYNC_WORD = 0x34)
  delay(1000);
  DSR.begin(LORA_BAND); // (_LORA_BAND = 868E6, _PABOOST = true)

  Serial.println(F("Start LoRa DSR sender"));
}

long lastsent = -interval;
void loop()
{
  long cur_time = millis();
  DSR.check_timer(cur_time);

  if (cur_time - lastsent > interval) {
    lastsent = cur_time;
    counter++;
    // String message = String(DSR.currentID) + ":Hello World:" + String(counter);
    String message = "{\"id\":91,\"ty\":1,\"sta\":0,\"lat\":98.1234567,\"lon\":18.1234567,\"spd\":1,\"sat\":4,\"dtp\":53,\"tmp\":25,\"hum\":47,\"prs\":973}";
    DSR.sendDATA(message, DSR.destinationID);
    Serial.println(message);
  }

  String received = DSR.checkPacket();
  if (received != ""){
    Serial.print(F("Received -> "));
    Serial.println(received);
    if (received == "SENT_SUCCESS") {
      Serial.println("Send: " + String(counter));
      Serial.println("Success: " + String(++success));
    }else{

    }
  }
}