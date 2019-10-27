#include <SPI.h>      // include libraries
#include <LoRa.h>
#include <SSD1306.h>
#include <ArduinoJson.h>

//////////////////////CONFIG NODE///////////////////////////
String nodeFunction[4] = {"DEVICE", "CLUSTER", "GATEWAY", "NONE"};
// This node is cluster?
// 0 = End device low battery
// 1 = Cluster station
// 2 = Internet cluster station
// 3 = Not connect
byte DeviceType = 1;
byte currentID = 91;         // This node address
byte destinationID = 93;    // Original destination (0xFF broadcast)

int interval = 10000;       // interval between sends
//String msg = "Hello World";    // send a message

//Pinout! Customized for TTGO LoRa32 V2.0 Oled Board!
#define SX1278_SCK  5    // GPIO5  -- SX1278's SCK
#define SX1278_MISO 19   // GPIO19 -- SX1278's MISO
#define SX1278_MOSI 27   // GPIO27 -- SX1278's MOSI
#define SX1278_CS   18   // GPIO18 -- SX1278's CS
#define SX1278_RST  14   // GPIO14 -- SX1278's RESET
#define SX1278_DI0  26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define OLED_ADDR   0x3c  // OLED's ADDRESS
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16

#define LORA_BAND   920E6 // LoRa Band (Thailand)
#define PABOOST     true

// LoRaWAN Parameters
#define TXPOWER 14
#define RF_PACONFIG_PASELECT_PABOOST 0x80
#define RF_PACONFIG_PASELECT_RFO 0x00
#define SPREADING_FACTOR 12
#define BANDWIDTH 125000
#define CODING_RATE 5
#define PREAMBLE_LENGTH 8
#define SYNC_WORD 0x34
SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL, OLED_RST);

#define LEDPIN 25
uint64_t chipid;
String MAC;

//DSR algorithm
uint16_t msgID = 0;   // Unique packet ID at source node
String packets[6] = {"RREQ", "RREP", "DATA", "RERR", "UACK", "MACK"};
byte const maxDestinationRow = 4; // number of destination nodes
byte const maxPathListColumn = 4; // number of path or hop nodes from destination node
byte* routingTable[maxDestinationRow] = {}; // Routing Table
byte const maxProcess = 16; // number of destination nodes
uint32_t myprocesses[maxProcess] = {};
bool timer_state = false;
long prev_timer = 0;


long lastSendTime = 0;  // last send time


void setup() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(OLED_RST, HIGH); // while OLED running, must set GPIO16 in high
  delay(1000);

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);

  display.init();
  //  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();

  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("GAINT LoRa DSR V0.1");
  display.drawString(0, 00, "GAINT LoRa DSR V0.1");
  display.display();

  LoRa.setPins(SX1278_CS, SX1278_RST, SX1278_DI0);// set CS, reset, IRQ pin

  // should be done before LoRa.begin
  configForLoRaWAN();

  if (!LoRa.begin(LORA_BAND, PABOOST))
  { // initialize ratio at 920 MHz
    Serial.println("LoRa init failed. Check your connections.");
    display.drawString(0, 10, "LoRa init failed");
    display.drawString(0, 20, "Check connections");
    display.display();

    while (true);                       // if failed, do nothing
  }

  //LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("LoRa init succeeded.");
  display.drawString(0, 10, "LoRa init succeeded.");
  display.drawString(0, 20, "LoRa Mesh init...");
  display.display();
  delay(1500);
  //  display.clear();
  //  display.display();

  // MAC do LoRa
  chipid = ESP.getEfuseMac();
  //Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  //Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  MAC = String((uint16_t)(chipid >> 32), HEX);
  MAC += String((uint32_t)chipid, HEX);
  Serial.println(MAC);


  timer_state = true;
  byte path[maxPathListColumn] = {};
  path[0] = currentID;
  Serial.println("Send RREQ");
  sendRREQ(destinationID, path, 1);
//  printTable();
  prev_timer = millis();
}
int k = 10000;
void loop() {
  long cur_mil = millis();
  if (timer_state && cur_mil - prev_timer > k) {
    prev_timer = cur_mil;
    // re-initiating RREQ.
    byte path[maxPathListColumn] = {};
    path[0] = currentID;
    Serial.println("Send RREQ again");
    sendRREQ(destinationID, path, 1);
//    printTable();

    LoRa.receive();                     // go back into receive mode
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }
}


void sendRREQ(byte destination, byte pathlist[], byte pathlength) {
  LoRa.beginPacket();         // start packet
  LoRa.write(0);              // RREQ packet type
  LoRa.write(currentID);      // add source address
  LoRa.write(msgID);          // add Unique packet ID
  LoRa.write(destination);  // add destination address
  LoRa.write(pathlength);     // add path length
  for (int i = 0; i < pathlength; i++) {
    LoRa.write(pathlist[i]);  // add path list from source to destination
  }
  LoRa.endPacket();           // finish packet and send it
  msgID++;
}


void sendRREP(byte destination, byte pathlist[], byte pathlength) {
  LoRa.beginPacket();         // start packet
  LoRa.write(1);              // RREP packet type
  LoRa.write(currentID);      // add source address
  LoRa.write(msgID);          // add Unique packet ID
  LoRa.write(destination);  // add destination address
  LoRa.write(pathlength);     // add path length
  for (int i = 0; i < pathlength; i++) {
    LoRa.write(pathlist[i]);  // add path list from source to destination
  }
  LoRa.endPacket();           // finish packet and send it
  msgID++;
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;        // if there's no packet, return

  // read packet header bytes:
  byte packetType = LoRa.read();      // incoming packet type
  byte source;
  uint16_t UID;
  uint32_t process;
  byte destination;
  byte pathlength;
  byte path[maxPathListColumn];

  String message;         // DATA

  byte original_source;   // UACK, MACK
  uint16_t original_UID;  // UACK, MACK

  int i;
  int index;
  switch (packetType) {
    case 0:
      // statements RREQ processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      destination = LoRa.read();  // incoming destination address
      pathlength = LoRa.read();   // incoming path length
      for (i = 0; i < pathlength; i++) {
        path[i] = LoRa.read();;   // add path list from incoming path
      }

      process = UID * 100;
      process += source;
      Serial.println("Find Process: " + String(process));

      if (arrayIncludeProcess(myprocesses, process, maxProcess)) {  // check by comparing it with elements in processed list
        return;   // THEN skip packet.
      } else {
        Serial.println("Add Process");
        arrayAddProcess(myprocesses, process, maxProcess);  // add <Source IP, UID > to the processed list.
        if (currentID == destination) {
          Serial.println("currentID == destination");
          path[i] = currentID;    // append current node IP to path
          Serial.println("Send RREP");
          sendRREP(path[i - 1], path, i + 1);   // send RREP to previous node using path with new UID

          // the path to destination from current node is in routing table of the current node
        } else if (tableIncludeDest(routingTable, destination, maxDestinationRow)) {
          Serial.println("destination in table");
          path[i] = currentID;  // append Current IP to path
          Serial.println("Send RREP");
          sendRREP(path[i - 1], path, i + 1);   // send RREP to previous node through path and with new UID
        } else {
          path[i] = currentID;  // append current IP to path
          Serial.println("rebroadcast");
          // rebroadcast the packet
          LoRa.beginPacket();         // start packet
          LoRa.write(packetType);     // RREP packet type
          LoRa.write(source);         // add source address
          LoRa.write(UID);            // add Unique packet ID
          LoRa.write(destination);    // add destination address
          LoRa.write(pathlength);     // add path length
          for (int j = 0; j < i + 1; j++) {
            LoRa.write(path[j]);  // add path list from source to destination
          }
          LoRa.endPacket();           // finish packet and send it
        }
      }
      break;
    case 1:
      // statements RREP processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      destination = LoRa.read();  // incoming destination address
      pathlength = LoRa.read();   // incoming path length
      for (i = 0; i < pathlength; i++) {
        path[i] = LoRa.read();;   // add path list from incoming path
      }
      index = arrayIncludeElement(path, currentID, pathlength);
      // FOR each node that appear to the left of the current node in the path field of RREP packet
      for (int j = 0; j < index; j++) {
        byte routing[] = {path[j], path[index - 1]};
        /*
          make an entry in routing table for that node with predecessor node in
          path field of RREP being the next hop in routing table
        */
        tableAddList(routingTable, routing, maxDestinationRow);
      }
      // FOR each node that appear to the right of the current node in the path field of RREP packet
      for (int j = pathlength - 1; j > index; j--) {
        byte routing[] = {path[j], path[index + 1]};
        /*
          make an entry in routing table for that node with successor node in
          path field of RREP being the next hop in routing table
        */
        tableAddList(routingTable, routing, maxDestinationRow);
      }

      if (currentID != path[0]) {
        sendRREP(path[index - 1], path, index + 1);   // send the RREP packet to the previous node in the path
      } else {
        timer_state = false;
        Serial.println("Start Send Data");
      }
      break;
    case 2:
      // statements

      break;
    default:
      // statements
      break;
  }
}


bool arrayIncludeProcess(uint32_t array[], uint32_t process, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == process) {
      return true;
    }
  }
  return false;
}
byte cur_process = 0;
void arrayAddProcess(uint32_t array[], uint32_t process, byte max) {
  if (cur_process == max) {
    cur_process = 0;
  }
  array[cur_process] = process;
  cur_process++;
}


bool tableIncludeDest(byte** array, byte destination, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][0] == destination) {
      return true;
    }
  }
  return false;
}
void tableAddList(byte** array, byte list[], byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][0] == 0) {
      array[i] = list;
      return;
    }
  }
}


int arrayIncludeElement(byte array[], byte element, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == element) {
      return i;
    }
  }
  return -1;
}


void arrayAddElement(byte array[], byte element, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == 0) {
      array[i] = element;
      return;
    }
  }
}


void configForLoRaWAN()
{
  if (PABOOST)
    LoRa.setTxPower(TXPOWER, RF_PACONFIG_PASELECT_PABOOST);
  else
    LoRa.setTxPower(TXPOWER, RF_PACONFIG_PASELECT_RFO);
  //  LoRa.setTxPower(TXPOWER);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.crc();
}


void printTable() {
  Serial.print("Table:");
  for (int i = 0; i < maxDestinationRow; i++) {
    for (int j = 0; j < maxPathListColumn; j++) {
      Serial.print(String(routingTable[i][j]));  Serial.print("\t");
    }
    Serial.println();
  }
}
