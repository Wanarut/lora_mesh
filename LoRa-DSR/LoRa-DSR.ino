#include <SPI.h>      // include libraries
#include <LoRa.h>
#include <SSD1306.h>

//////////////////////CONFIG NODE///////////////////////////
String nodeFunction[4] = {"DEVICE", "CLUSTER", "GATEWAY", "NONE"};
// This node is cluster?
// 0 = End device low battery
// 1 = Cluster station
// 2 = Internet cluster station
// 3 = Not connect
byte DeviceType = 1;
byte currentID = 94;        // This node address
byte destinationID = 93;    // Original destination (0xFF broadcast)
String code_version = "1.0";

int interval = 30000;       // interval between sends

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
#define PABOOST     false

// LoRaWAN Parameters
#define TXPOWER 0
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

// DSR algorithm
byte msgID = 1;   // Unique packet ID at source node
String packets[6] = {"RREQ", "RREP", "DATA", "RERR", "UACK", "MACK"};
byte const maxDestinationRow = 4; // number of destination nodes
byte routingTable[maxDestinationRow][2] = {}; // Routing Table
byte const maxProcess = 2; // number of destination nodes
byte cur_process = 0;
uint16_t myprocesses[maxProcess] = {};
byte const maxPathListLength = 16; // number of path or hop nodes from destination node

long rreq_timer = 0;
byte rreq_K = 0;
long uack_timer = 0;
byte uack_K = 0;
long mack_timer = 0;
byte mack_K = 0;
byte original_source = 0;
byte original_destination = 0;

long timedout = 4000;

// byte const maxQueue = 4; // number of data buffer
// String data_queue[maxQueue] = {};
String message_data = "";

int counter = 0;
int success = 0;
void setup() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(OLED_RST, HIGH); // while OLED running, must set GPIO16 in high
  delay(1000);

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);

  display.init();
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();

  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("GIANT LoRa DSR v" + code_version);
  display.drawString(0, 00, "GAINT LoRa DSR v" + code_version);
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

  //  LoRa.onReceive(onReceive);
  LoRa.receive();

  Serial.println("LoRa init succeeded. LoRa DSR init...");
  display.drawString(0, 10, "LoRa init succeeded.");
  display.drawString(0, 20, "LoRa DSR init...");
  display.display();
  delay(1500);
  display.clear();
  display.display();

  // MAC do LoRa
  chipid = ESP.getEfuseMac();
  //Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
  //Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  MAC = String((uint16_t)(chipid >> 32), HEX);
  MAC += String((uint32_t)chipid, HEX);
  Serial.println(MAC);

  displayData();
//  if (DeviceType != 2) {
//    message_data = String(currentID) + ":Hello World:" + String(counter++);
//    sendDATA(message_data, destinationID);
//  }
}
byte Kmax = 5;
long lastsent = -interval;
void loop() {
  long cur_time = millis();
  check_timer(cur_time);

  if (DeviceType != 2 && cur_time - lastsent > interval) {
    lastsent = cur_time;
    message_data = String(currentID) + ":Hello World:" + String(counter++);
    sendDATA(message_data, destinationID);
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }
}

void check_timer(long cur_time) {

  // If timer timed out then try for maximum K times by re-initiating RREQ.
  if (rreq_timer) {
    if (rreq_K < 5) {
      if (cur_time - rreq_timer > timedout) {
        rreq_timer = cur_time;
        Serial.println("rreq_timer time out");
        sendRREQ(original_destination);
        rreq_K++;
      }
    } else {
      //if timed out for K times then show error and drop all the packets
      rreq_timer = 0;
      rreq_K = 0;
      Serial.println("tx RREQ Fail");
      displayStatus("RREQ Fail");
    }
  }

  // IF UACK is received
  if (uack_timer == 0) {
    // THEN drop the copy of the packet from the queue.
    // data_queue[destinationID % maxQueue] = "";
  } else if (cur_time - uack_timer > timedout) {
    uack_timer = cur_time;
    uack_K++;
    // IF Number of time out is equal to K
    if (uack_K == Kmax) {
      uack_timer = 0;
      uack_K = 0;
      // THEN initiate RREQ
      Serial.println("uack_timer max time out");
      sendRREQ(destinationID);
    } else {
      // restart DATA initiation.
      Serial.println("uack_timer time out");
      sendDATA(message_data, destinationID);
    }
  }

  // IF MACK is received
  if (mack_timer == 0) {
    // THEN drop the copy of the packet from the queue.
    // data_queue[original_destination % maxQueue] = "";
  } else if (cur_time - mack_timer > timedout/4) {
    mack_timer = cur_time;
    mack_K++;
    // IF Number of time out is equal to K
    if (mack_K == Kmax) {
      mack_timer = 0;
      mack_K = 0;
      // Send RERR to original source.
      sendRERR(original_source);
      // Remove entry for route to destination from the routing table.
      //      tableDelList(routingTable, original_source, maxDestinationRow);
      byte nexthop = tableIncludeDest(routingTable, original_destination, maxDestinationRow);
      tableDelHop(routingTable, nexthop, maxDestinationRow);
      // Send RREQ for destination, for sending the data.
      sendRREQ(original_destination);
    }
  }

}


void sendRREQ(byte destination) {
  Serial.println();
  // Initiated by the source node which wish to send data and do not have path in its routing table.
  tableDelList(routingTable, destination, maxDestinationRow);

  uint16_t process = msgID * 100;
  process += currentID;
  Serial.print("Add Process: " + String(process) + " ");
  arrayAddProcess(myprocesses, process, maxProcess);

  // starting the timer
  rreq_timer = millis();
  original_destination = destination;

  // create the RREQ packet
  byte path[maxPathListLength] = {};
  arrayAddElement(path, currentID, maxPathListLength);
  byte pathlength = 1;

  // broadcast
  LoRa.beginPacket();         // start packet
  LoRa.write(0);              // RREQ packet type
  LoRa.write(currentID);      // add source address
  LoRa.write(msgID);          // add Unique packet ID
  LoRa.write(destination);    // add destination address
  LoRa.write(pathlength);     // add path length

  String tx_data = String(currentID) + ":" + String(msgID) + ":" + String(destination) + ":" + String(pathlength);
  for (int i = 0; i < pathlength; i++) {
    LoRa.write(path[i]);      // add path list from source to destination
    tx_data += ":" + String(path[i]);
  }
  LoRa.endPacket();           // finish packet and send it
  msgID++;
  msgID %= 256;

  Serial.println("tx RREQ: " + tx_data);
  displayTX(tx_data);
  displayStatus("TX RREQ");
  //  printTable();
  //  displayTable();

  LoRa.receive();
}


void sendRREP(byte path[], byte pathlength, byte nexthop) {
  int cur_pos = arrayIncludeElement(path, currentID, pathlength);
  if (cur_pos == -1) return;
  pathToTable(routingTable, path, pathlength, cur_pos);

  // Initiated by a node when either path to destination is found in the routing table or
  // when a node itself is destination for which RREQ was intended.
  if (tableIncludeDest(routingTable, nexthop, maxDestinationRow) || currentID == path[pathlength - 1]) {
    Serial.println();
    uint16_t process = msgID * 100;
    process += currentID;
    Serial.print("Add Process: " + String(process) + " ");
    arrayAddProcess(myprocesses, process, maxProcess);

    LoRa.beginPacket();         // start packet
    LoRa.write(1);              // RREP packet type
    LoRa.write(currentID);      // add source address
    LoRa.write(msgID);          // add Unique packet ID
    LoRa.write(nexthop);        // add next hop address
    LoRa.write(pathlength);     // add path length

    String tx_data = String(currentID) + ":" + String(msgID) + ":" + String(nexthop) + ":" + String(pathlength);
    for (int i = 0; i < pathlength; i++) {
      LoRa.write(path[i]);  // add path list from source to destination
      tx_data += ":" + String(path[i]);
    }
    LoRa.endPacket();           // finish packet and send it
    msgID++;
    msgID %= 256;

    Serial.println("tx RREP: " + tx_data);
    displayTX(tx_data);
    displayStatus("TX RREP");
    //    printTable();
    //    displayTable();

    LoRa.receive();
  }
}


void sendDATA(String payload, byte destination) {
  // Store the data packet in the queue corresponding to the destination.
  // data_queue[msgID % maxQueue] = payload;
  // IF path to destination is available in routing table.
  // Send packet to the node found in the routing table to the destination.
  // (i.e. Next Hop to the destination).
  byte nexthop = tableIncludeDest(routingTable, destination, maxDestinationRow);
  if (nexthop > 0) {
    Serial.println();
    uint16_t process = msgID * 100;
    process += currentID;
    Serial.print("Add Process: " + String(process) + " ");
    arrayAddProcess(myprocesses, process, maxProcess);

    // Start the timer for (UACK).
    uack_timer = millis();

    byte checksum = payload.length();

    LoRa.beginPacket();       // start packet
    LoRa.write(2);            // DATA packet type
    LoRa.write(currentID);    // add source address
    LoRa.write(msgID);        // add Unique packet ID
    LoRa.write(nexthop);      // add next hop address
    LoRa.write(destination);  // add destination address
    LoRa.write(checksum);     // add payload length
    LoRa.print(payload);      // add payload
    LoRa.endPacket();         // finish packet and send it

    String tx_data = String(currentID) + ":" + String(msgID) + ":" + String(nexthop) + ":" + String(destination) + ":" + String(checksum);

    msgID++;
    msgID %= 256;

    Serial.println("tx DATA: " + tx_data);
    Serial.println("--------send DATA to destination--------");
    Serial.println("DATA: " + payload);
    Serial.println("----------------------------------------");
    displayTX(tx_data);
    displayStatus("TX DATA");
    //    printTable();
    //    displayTable();

    LoRa.receive();
  } else {
    sendRREQ(destination);
  }
}


void sendRERR(byte destination) {
  Serial.println();
  uint16_t process = msgID * 100;
  process += currentID;
  Serial.print("Add Process: " + String(process) + " ");
  arrayAddProcess(myprocesses, process, maxProcess);

  byte path[maxPathListLength] = {};
  arrayAddElement(path, currentID, maxPathListLength);
  byte pathlength = 1;

  LoRa.beginPacket();         // start packet
  LoRa.write(3);              // RERR packet type
  LoRa.write(currentID);      // add source address
  LoRa.write(msgID);          // add Unique packet ID
  LoRa.write(destination);    // add destination address
  LoRa.write(pathlength);     // add path length

  String tx_data = String(currentID) + ":" + String(msgID) + ":" + String(destination) + ":" + String(pathlength);
  for (int i = 0; i < pathlength; i++) {
    LoRa.write(path[i]);      // add path list from source to destination
    tx_data += ":" + String(path[i]);
  }
  LoRa.endPacket();           // finish packet and send it
  msgID++;
  msgID %= 256;

  Serial.println("tx RERR: " + tx_data);
  displayTX(tx_data);
  displayStatus("TX RERR");
  //  printTable();
  //  displayTable();

  LoRa.receive();
}


void sendUACK(byte original_source, byte original_UID) {
  byte nexthop = tableIncludeDest(routingTable, original_source, maxDestinationRow);
  if (nexthop != 0) {
    Serial.println();
    LoRa.beginPacket();           // start packet
    LoRa.write(4);                // RERR packet type
    LoRa.write(currentID);        // add node which initiated the UACK packet address
    LoRa.write(original_UID);     // add Unique packet ID
    LoRa.write(nexthop);          // add next hop address
    LoRa.write(original_source);  // add original source address
    LoRa.endPacket();             // finish packet and send it

    String tx_data = String(currentID) + ":" + String(original_UID) + ":" + String(nexthop) + ":" + String(original_source);

    msgID++;
    msgID %= 256;

    Serial.println("tx UACK: " + tx_data);
    displayTX(tx_data);
    displayStatus("TX UACK");
    //    printTable();
    //    displayTable();

    LoRa.receive();
  }
}


void sendMACK(byte original_source, byte original_UID) {
  byte nexthop = tableIncludeDest(routingTable, original_source, maxDestinationRow);
  if (nexthop != 0) {
    Serial.println();
    LoRa.beginPacket();           // start packet
    LoRa.write(5);                // RERR packet type
    LoRa.write(original_UID);     // add Unique packet ID
    LoRa.write(nexthop);          // add next hop address
    LoRa.write(original_source);  // add original source address
    LoRa.endPacket();             // finish packet and send it

    String tx_data = String(original_UID) + ":" + String(nexthop) + ":" + String(original_source);

    msgID++;
    msgID %= 256;

    Serial.println("tx MACK: " + tx_data);
    displayTX(tx_data);
    displayStatus("TX MACK");
    //    printTable();
    //    displayTable();

    LoRa.receive();
  }
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;        // if there's no packet, return
  Serial.println();

  // read packet header bytes:
  byte packetType = LoRa.read();      // incoming packet type
  byte source;
  byte UID;
  uint16_t process;
  byte nexthop;
  byte destination;
  byte pathlength;
  byte path[maxPathListLength];

  int cur_pos;
  String rx_data;
  String message;         // DATA
  switch (packetType) {
    case 0:
      // statements RREQ processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      destination = LoRa.read();  // incoming destination address
      pathlength = LoRa.read();   // incoming path length

      rx_data = String(source) + ":" + String(UID) + ":" + String(destination) + ":" + String(pathlength);
      for (int i = 0; i < pathlength; i++) {
        path[i] = LoRa.read();    // add path list from incoming path
        rx_data += ":" + String(path[i]);
      }

      Serial.println("rx RREQ: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX RREQ");

      process = UID * 100;
      process += source;

      if (arrayIncludeProcess(myprocesses, process, maxProcess)) {  // check by comparing it with elements in processed list
        Serial.println("Skip Process: " + String(process));
        return;   // THEN skip packet.
      } else {
        Serial.print("Add Process: " + String(process) + " ");
        arrayAddProcess(myprocesses, process, maxProcess);

        if (currentID == destination) {
          Serial.println("RREQ reach destination");
          path[pathlength] = currentID;
          pathlength++;
          sendRREP(path, pathlength, path[pathlength - 2]);
          //        } else if (tableIncludeDest(routingTable, destination, maxDestinationRow)) {
          //          Serial.println("destination in table");
          //          path[pathlength] = currentID;
          //          pathlength++;
          //          sendRREP(path, pathlength, path[pathlength - 2]);
        } else {
          Serial.println("rebroadcast");
          original_destination = destination;
          path[pathlength] = currentID;
          pathlength++;
          LoRa.beginPacket();         // start packet
          LoRa.write(0);              // RREQ packet type
          LoRa.write(source);      // add source address
          LoRa.write(UID);          // add Unique packet ID
          LoRa.write(destination);    // add destination address
          LoRa.write(pathlength);     // add path length
          for (int i = 0; i < pathlength; i++) {
            LoRa.write(path[i]);      // add path list from source to destination
          }
          LoRa.endPacket();           // finish packet and send it

          LoRa.receive();
        }
      }
      break;
    case 1:
      // statements RREP processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      nexthop = LoRa.read();      // incoming next hop address
      pathlength = LoRa.read();   // incoming path length

      rx_data = String(source) + ":" + String(UID) + ":" + String(nexthop) + ":" + String(pathlength);

      for (int i = 0; i < pathlength; i++) {
        path[i] = LoRa.read();    // add path list from incoming path
        rx_data += ":" + String(path[i]);
      }

      Serial.println("rx RREP: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX RREP");

      // uni-cast
      if (currentID != nexthop) {
        return;
      }

      cur_pos = arrayIncludeElement(path, currentID, pathlength);
      if (cur_pos == -1) return;

      // when RREP packet is received by the node in the network follow these steps.
      //      pathToTable(routingTable, path, pathlength, cur_pos);
      //      printTable();
      //      displayTable();

      // IF Current node is not equal to destination node in RREP
      // (i.e. original source which initiated RREQ).
      if (currentID != path[0]) {
        sendRREP(path, pathlength, path[cur_pos - 1]);
      } else {
        // stop the timer.
        rreq_timer = 0;
        pathToTable(routingTable, path, pathlength, cur_pos);
        Serial.print("\nInitiate data transmission process.\n");
        sendDATA(message_data, destinationID);
      }
      break;
    case 2:
      // statements DATA processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      nexthop = LoRa.read();      // incoming nexthop address
      destination = LoRa.read();  // incoming destination address
      pathlength = LoRa.read();   // incoming checksum

      message = "";               // payload of packet

      while (LoRa.available()) {            // can't use readString() in callback, so
        message += (char)LoRa.read();      // add bytes one by one
      }

      // unicast
      if (currentID != nexthop) {
        Serial.println("Drop Packet");
        return;
      }

      rx_data = String(source) + ":" + String(UID) + ":" + String(nexthop) + ":" + String(destination) + ":" + String(pathlength);

      Serial.println("rx DATA: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX DATA");

      if (pathlength != message.length()) {   // check length for error
        Serial.println("error: message length does not match length");
        return;                             // skip rest of function
      }
      // Send MACK to the node from which DATA packet arrived.
      sendMACK(source, UID);

      process = UID * 100;
      process += source;

      // IF Packet received for not the first time.
      if (arrayIncludeProcess(myprocesses, process, maxProcess)) {  // check by comparing it with elements in processed list
        Serial.println("Skip Process: " + String(process));
        return;   // THEN skip packet.
      }
      // IF Current node is the destination node.
      if (currentID == destination) {
        // Consume packet.
        Serial.println();
        Serial.println("---------DATA reach destination---------");
        Serial.println("DATA: " + message);
        Serial.println("----------------------------------------");
        // Send UACK.
        sendUACK(source, UID);
      } else {
        // Start the timer (MACK).
        mack_timer = millis();
        original_source = source;
        original_destination = destination;
        nexthop = tableIncludeDest(routingTable, destination, maxDestinationRow);
        // Forward the data to the next node towards the destination.
        LoRa.beginPacket();       // start packet
        LoRa.write(2);            // DATA packet type
        LoRa.write(source);       // add source address
        LoRa.write(UID);          // add Unique packet ID
        LoRa.write(nexthop);      // add next hop address
        LoRa.write(destination);  // add destination address
        LoRa.write(pathlength);   // add payload length
        LoRa.print(message);      // add payload
        LoRa.endPacket();         // finish packet and send it

        LoRa.receive();
      }
      break;
    case 3:
      // statements RERR processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      destination = LoRa.read();  // incoming destination address
      pathlength = LoRa.read();   // incoming path length

      rx_data = String(source) + ":" + String(UID) + ":" + String(destination) + ":" + String(pathlength);

      for (int i = 0; i < pathlength; i++) {
        path[i] = LoRa.read();    // add path list from incoming path
        rx_data += ":" + String(path[i]);
      }

      Serial.println("rx RERR: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX RERR");

      process = UID * 100;
      process += source;

      if (arrayIncludeProcess(myprocesses, process, maxProcess)) {  // check by comparing it with elements in processed list
        Serial.println("Skip Process: " + String(process));
        return;   // THEN skip packet.
      }
      // From the Routing table of the node, remove all those entries where
      // next node entry in the routing table is equal to the successor of the
      // current node in the path field of the RERR packet.
      //      for (int i = 0; i < pathlength; i++) {
      //        nexthop = tableIncludeDest(routingTable, path[i], maxDestinationRow);
      //        if (nexthop > 0) {
      //          tableDelHop(routingTable, nexthop, maxDestinationRow);
      //        }
      //      }

      nexthop = tableIncludeDest(routingTable, destination, maxDestinationRow);
      if (nexthop > 0) {
        tableDelHop(routingTable, nexthop, maxDestinationRow);
        byte routing[] = {destination, path[pathlength - 1]};
        tableAddList(routingTable, routing, maxDestinationRow);
      }
      // Add the entries in the routing table for all the nodes that are successor
      // of the current node in the path field of the RERR packet.

      for (int i = 0; i < pathlength; i++) {
        if (!tableIncludeDest(routingTable, path[i], maxDestinationRow)) {
          byte routing[] = {path[i], path[pathlength - 1]};
          tableAddList(routingTable, routing, maxDestinationRow);
        }
      }
      // pathToTable(routingTable, path, pathlength, cur_pos);
      printTable();
      displayTable();

      // IF current node is not equal to destination node
      if (currentID != destination) {
        // prepend the node IP address and send the packet towards the destination
        nexthop = tableIncludeDest(routingTable, destination, maxDestinationRow);
        path[pathlength] = currentID;
        pathlength++;
        LoRa.beginPacket();         // start packet
        LoRa.write(3);              // RREQ packet type
        LoRa.write(source);      // add source address
        LoRa.write(UID);          // add Unique packet ID
        LoRa.write(destination);    // add destination address
        LoRa.write(pathlength);     // add path length
        for (int i = 0; i < pathlength; i++) {
          LoRa.write(path[i]);      // add path list from source to destination
        }
        LoRa.endPacket();           // finish packet and send it

        LoRa.receive();
      } else {
        Serial.println("RERR reached source");
      }
      break;
    case 4:
      // statements UACK processing
      source = LoRa.read();       // incoming sender address
      UID = LoRa.read();          // incoming message ID
      nexthop = LoRa.read();      // incoming nexthop address
      destination = LoRa.read();  // incoming destination address

      rx_data = String(source) + ":" + String(UID) + ":" + String(nexthop) + ":" + String(destination);

      // unicast
      if (currentID != nexthop) {
        return;
      }

      Serial.println("rx UACK: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX UACK");

      if (currentID != destination) {
        sendUACK(destination, UID);
      } else {
        uack_timer = 0;
        uack_K = 0;
        Serial.println("Sending: " + String(counter));
        Serial.println("Success: " + String(++success));
      }
      break;
    case 5:
      // statements MACK processing
      UID = LoRa.read();          // incoming message ID
      nexthop = LoRa.read();      // incoming nexthop address
      destination = LoRa.read();  // incoming destination address

      rx_data = String(UID) + ":" + String(nexthop) + ":" + String(destination);

      // unicast
      if (currentID != nexthop) {
        return;
      }

      Serial.println("rx MACK: " + rx_data);
      Serial.println("RSSI: " + String(LoRa.packetRssi()));
      Serial.println("Snr: " + String(LoRa.packetSnr()));

      displayRX(rx_data);
      displayStatus("RX MACK");

      mack_timer = 0;
      mack_K = 0;
      break;
    default:
      // statements
      break;
  }
}


bool arrayIncludeProcess(uint16_t array[], uint16_t process, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == process) {
      return true;
    }
  }
  return false;
}
void arrayAddProcess(uint16_t array[], uint16_t process, byte max) {
  array[cur_process] = process;
  cur_process++;
  if (cur_process == max) {
    cur_process = 0;
  }
  printProcess();
  displayProcess();
}
void printProcess() {
  Serial.print("Process: {");
  for (int i = 0; i < maxProcess; i++) {
    Serial.print(myprocesses[i]);  Serial.print(" ");
  }  Serial.println("}");
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
void printArray(byte array[], byte max) {
  Serial.print("Array: {");
  for (int i = 0; i < max; i++) {
    Serial.print(array[i]);  Serial.print(" ");
  }  Serial.println("}");
}


void configForLoRaWAN() {
  if (PABOOST)
    LoRa.setTxPower(TXPOWER, RF_PACONFIG_PASELECT_PABOOST);
  else
    LoRa.setTxPower(TXPOWER, RF_PACONFIG_PASELECT_RFO);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(PREAMBLE_LENGTH);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.crc();
}

void pathToTable(byte routingTable[maxDestinationRow][2], byte path[], byte pathlength, byte cur_pos) {
  for (int i = 0; i < cur_pos; i++) {
    //    if (!tableIncludeDest(routingTable, path[i], maxDestinationRow)) {
    byte routing[] = {path[i], path[cur_pos - 1]};
    /*
      make an entry in routing table for that node with predecessor node in
      path field of RREP being the next hop in routing table
    */
    tableAddList(routingTable, routing, maxDestinationRow);
    //    }
  }
  // FOR each node that appear to the right of the current node in the path field of RREP packet
  for (int i = pathlength - 1; i > cur_pos; i--) {
    //    if (!tableIncludeDest(routingTable, path[i], maxDestinationRow)) {
    byte routing[] = {path[i], path[cur_pos + 1]};
    /*
      make an entry in routing table for that node with successor node in
      path field of RREP being the next hop in routing table
    */
    tableAddList(routingTable, routing, maxDestinationRow);
    //    }
  }
  printTable();
  displayTable();
}
byte tableIncludeDest(byte array[maxDestinationRow][2], byte destination, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][0] == destination) {
      return array[i][1];
    }
  }
  return 0;
}
void tableAddList(byte array[maxDestinationRow][2], byte list[], byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][0] == list[0]) {
      array[i][1] = list[1];
      return;
    }
    if (array[i][0] == 0) {
      array[i][0] = list[0];
      array[i][1] = list[1];
      return;
    }
  }
}
void tableDelList(byte array[maxDestinationRow][2], byte destination, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][0] == destination) {
      array[i][0] = 0;
      array[i][1] = 0;
      printTable();
      displayTable();
      return;
    }
  }
}
void tableDelHop(byte array[maxDestinationRow][2], byte nexthop, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i][1] == nexthop) {
      array[i][0] = 0;
      array[i][1] = 0;
    }
  }
  printTable();
  displayTable();
}
void printTable() {
  Serial.print("Table:");
  for (int i = 0; i < maxDestinationRow; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print("\t");
      Serial.print(routingTable[i][j]);
    }
    Serial.println();
  }
}


void displayData() {
  display.setColor(BLACK);
  display.fillRect(50, 0, 128, 26);
  display.display();
  display.setColor(WHITE);

  display.drawString(50, 0, "LoRa " + String(int(LORA_BAND / 1000000)) + " MHz");
  display.drawString(50, 8, nodeFunction[DeviceType]);
  display.drawString(50, 16, "src: " + String(currentID) + " dst: " + String(destinationID));
  display.display();
}
void displayProcess() {
  display.setColor(BLACK);
  display.fillRect(50, 27, 128, 35);
  display.display();
  display.setColor(WHITE);

  display.drawString(50, 24, "[ " + String(myprocesses[0]) + ", " + String(myprocesses[1]) + " ]");
  display.display();
}
void displayTable() {
  display.setColor(BLACK);
  display.fillRect(0, 0, 50, 36);
  display.drawLine(0, 36, 128, 36);
  display.display();
  display.setColor(WHITE);

  for (int i = 0; i < maxDestinationRow; i++) {
    for (int j = 0; j < 2; j++) {
      display.drawString(j * 25, i * 8, String(routingTable[i][j]));
    }
  }
  display.drawLine(0, 36, 128, 36);
  display.display();
}
void displayTX(String transmit) {
  display.setColor(BLACK);
  display.fillRect(0, 37, 128, 45);
  display.display();
  display.setColor(WHITE);

  display.drawString(0, 36, "tx:" + transmit);
  display.display();
}
void displayRX(String receive) {
  display.setColor(BLACK);
  display.fillRect(0, 46, 128, 54);
  display.display();
  display.setColor(WHITE);

  display.drawString(0, 45, "rx:" + receive);
  display.display();
}
void displayStatus(String status) {
  display.setColor(BLACK);
  display.fillRect(0, 55, 128, 64);
  display.display();
  display.setColor(WHITE);

  display.drawString(0, 54, "Stat: " + status);
  display.display();
}
