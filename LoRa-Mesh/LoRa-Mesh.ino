                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   /*
  LoRa Duplex communication Mesh - Sending multiple sensor Values to SYNC
  Note: while sending, LoRa radio is not listening for incoming messages.
  IPG - MCM 2019
  SENDING NODE
  0xFF como BroadCast
*/
#include <SPI.h>      // include libraries
#include <LoRa.h>
#include <SSD1306.h>
#include <ArduinoJson.h>

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

//////////////////////CONFIG 1///////////////////////////
// This node is server?
// 0 = Internet server
// 1 = Neighbor of Internet Server
// 2 = Neighbor with a neighbor from an internet server
byte isServer = 0;
String nodeFunction[4] = {"GATEWAY", "ROUTER", "DEVICE", "SLEEP"};

byte const maxTableArrayNeighbours = 32; // number of neighbors can be increased as memory is available
byte myNeighbours[maxTableArrayNeighbours] = {}; // address of direct neighbors

byte const maxTableArrayServers = 4; // number of servers I have access to can be increased
byte myServers[maxTableArrayServers]     = {}; // address of the servers I found

byte localAddress = 92;     // This node address
byte destination = 94;     // Original destination (0xFF broadcast)

int interval = 5000;       // interval between sends
String message = "Hello World";    // send a message
String otherValues = "";
///////////////////////////////////////////////////////

byte msgCount     = 0;        // count of outgoing messages
byte tableTime    = 5;       // when msgCount equal tableTime send the table
long lastSendTime = 0;        // last send time

/*
  //////////////////////CONFIG 2///////////////////////////
  byte localAddress = 18;    // address of this device
  byte destination = 8;      // destination to send to
  int interval = 2000;       // interval between sends
  String message = "Pong!"; // send a message
  ///////////////////////////////////////////////////////
*/


boolean arrayIncludeElement(byte array[], byte element, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == element) {
      return true;
    }
  }
  return false;
}


void arrayAddElement(byte array[], byte element, byte max) {
  for (int i = 0; i < max; i++) {
    if (array[i] == 0) {
      array[i] = element;
      return;
    }
  }
}


void printNeighbours() {
  Serial.print("Neighbours: {");
  for (int i = 0; i < sizeof(myNeighbours); i++) {
    Serial.print(String(myNeighbours[i]));  Serial.print(" ");
  }  Serial.println("}");

  Serial.print("Sync: {");
  for (int i = 0; i < sizeof(myServers); i++) {
    Serial.print(String(myServers[i]));     Serial.print(" ");
  }  Serial.println("}");
}


void printScreen() {
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.setColor(BLACK);
  //  display.clear();
  display.fillRect(0, 0, 128, 32);
  display.display();
  display.setColor(WHITE);
  // display.drawString(0, 00, String(LORA_BAND/1000000)+" LoRa sender " + String(localAddress));
  display.drawString(0, 00, String(int(LORA_BAND / 1000000)) + " MHz LoRa " + nodeFunction[isServer]);
  display.drawString(0, 10, "src: " + String(localAddress)
                     + " dst: " + String(destination)
                     + " #: " + String(msgCount));
  display.drawString(0, 20, "Tx: " + message);
  display.display();
}


void sendMessage(String outgoing, byte destination) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(isServer);                 // add server ID
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  printScreen();

  //  Serial.println("Send Message ID: " + String(msgCount) + " to Node: " + String(destination));
  //  Serial.println("Message: " + message);
  //  Serial.println();
  //  delay(1000);
  //  msgCount++;                           // increment message ID
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // incoming recipient address
  byte sender = LoRa.read();            // incoming sender address
  byte incomingMsgHand = LoRa.read();   // incoming server ID
  byte incomingMsgId = LoRa.read();     // incoming message ID
  byte incomingLength = LoRa.read();    // incoming payload length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    incoming = "message length error";
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me");
    incoming = "message is not for me";
    message = incoming;
    printScreen();
    //    delay(150);
    return;                             // skip rest of function
  }

  display.setColor(BLACK);
  display.fillRect(0, 32, 128, 61);
  display.display();

  display.setColor(WHITE);
  display.drawLine(0, 32, 128, 32);
  display.drawString(0, 32, "Rx: " + incoming);

  //New neighbor
  if (!arrayIncludeElement(myNeighbours, sender, maxTableArrayNeighbours)) {
    arrayAddElement(myNeighbours, sender, maxTableArrayNeighbours);
    display.drawString(0, 32, "NEW: " + String(sender));
    Serial.println("I found Node: " + String(sender));
  }
  display.drawString(0, 42, "FR:"  + String(sender)
                     + " TO:" + String(recipient)
                     + " L:" + String(incomingLength)
                     + " ID:" + String(incomingMsgId));
  display.drawString(0, 52, "RSSI: " + String(LoRa.packetRssi())
                     + " SNR: " + String(LoRa.packetSnr()));

  display.display();
  printNeighbours();

  // if message is for this device, or broadcast, print details:
  Serial.println("Handshake: " + String(incomingMsgHand));
  Serial.println("Received from: " + String(sender));
  Serial.println("Send to: " + String(recipient));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Msg: " + incoming);
  //--->> Save Values--->>
  otherValues += incoming;
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  //  delay(1000);


  // Posicionamento dos servidores na mesh
  switch (incomingMsgHand) {
    case 0:
      // statements
      if (!arrayIncludeElement(myServers, sender, maxTableArrayServers)) {
        Serial.println("I found a SYNC! " + String(sender));
        arrayAddElement(myServers, sender, maxTableArrayServers);
        display.drawString(0, 32, "NEW: " + String(sender));
      }
      destination = sender;
      break;
    case 1:
      // statements
      if (!arrayIncludeElement(myNeighbours, sender, maxTableArrayNeighbours)) {
        Serial.println("I found AUTOESTRADA for SYNC! " + String(sender));
        arrayAddElement(myNeighbours, sender, maxTableArrayNeighbours);
        display.drawString(0, 32, "NEW: " + String(sender));
      }
      if (isServer != 0) {
        destination = sender;
      }
      break;
    case 2:
      // statements
      Serial.println("I found CAMINHO to SYNC!");
      break;
    default:
      // statements
      break;
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


const size_t CAPACITY = JSON_ARRAY_SIZE(3);
StaticJsonDocument<CAPACITY> doc;
JsonArray array = doc.to<JsonArray>();
String Values;

void makeData() {
  // add some values
  array.add(MAC); //<- Lora MAC
  //  array.add(1556969160); //<-- Timestamp
  array.add(123);
  //  array.add(456);
  //  array.add(789);
  //  array.add(0);
  //  array.add(0);
  //  array.add(0);
  //  array.add(0);
  //  array.add(0);
  //  array.add(0);

  // serialize the array and send the result to Serial
  // serialize the array and send the result to Serial
  Values = "";
  serializeJson(doc, Values);
  serializeJson(doc, Serial);
  Serial.println("");
}


void setup() {
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);  // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(OLED_RST, HIGH); // while OLED running, must set GPIO16 in high
  delay(1000);

  //makeData();
  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN, OUTPUT);


  display.init();
  //  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();

  Serial.begin(115200);                   // initialize serial
  while (!Serial);
  Serial.println("IPG SFarm LoRa V0.5");
  display.drawString(0, 00, "IPG SFarm MeshLoRa V0.5");
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

  //  if (!bme.begin(0x76))
  //  {
  //    display.clear();
  //    display.drawString(0, 0, "Sensor não encontrado");
  //    display.display();
  //    while (1);
  //  }

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
}

String sendTable() {
  const size_t CAPACITY = JSON_ARRAY_SIZE(maxTableArrayServers);
  StaticJsonDocument<CAPACITY> doc;
  JsonArray array = doc.to<JsonArray>();
  for (int i = 0; i < sizeof(myServers); i++) {
    array.add(myServers[i]);
  }
  String Values;
  serializeJson(doc, Values);
  return Values;
}


void printSensor() {
  //  Serial.print("Temperature = ");
  //  Serial.print(bme.readTemperature());
  //  Serial.println("*C");
  //
  //  Serial.print("Pressure = ");
  //  Serial.print(bme.readPressure() / 100.0F);
  //  Serial.println("hPa");
  //
  //  Serial.print("Humidity = ");
  //  Serial.print(bme.readHumidity());
  //  Serial.println("%");
}

void loop() {
  long curTime = millis();
  if (curTime - lastSendTime > interval) {
    lastSendTime = curTime;            // timestamp the message

    //  Serial.print("Destination = ");
    //  Serial.println(destination);

    Serial.println("msgCount: " + String(msgCount));
    if (msgCount % tableTime == 0)
    {
      message = sendTable();
      Serial.println("table: " + message);
      sendMessage(message, 0xFF);
      //<<<--- send all values from all nodes
      //SendValues(otherValues);
      otherValues = "";
      //      msgCount = 0;    // reset message ID

    } else {
      if (isServer == 0) {
        // send to TTN
        digitalWrite(LEDPIN, HIGH);
        // printSensor();
        //        otherValues += Values;
        Serial.println("otherValues: " + otherValues);
        Serial.println();

      } else {
        // enviar para mais próximo do TTN em random
        digitalWrite(LEDPIN, LOW);
        destination = myNeighbours[0];
        Serial.print("message: ");
        makeData();
        message = Values;
        sendMessage(message, destination);
        printSensor();
      }
    }
    msgCount++;                           // increment message ID

    //    interval = random(interval) + 20000;     // 20-30 seconds
    LoRa.receive();                     // go back into receive mode
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    onReceive(packetSize);
  }
}
