/*
  LoRa_DSR.h - LoRa_DSR library for Wiring - description
  Copyright (c) 2020 Wanarut Boonyung.  All right reserved.
*/

#ifndef LoRa_DSR_h
#define LoRa_DSR_h

#include <SPI.h>
#include <SSD1306Wire.h>

class LoRa_DSR
{
  public:
    byte currentID      = 255;        // This node address
    byte destinationID  = 255;        // Original destination (0xFF(255) broadcast)
    byte DeviceType     = 1;          // This node is cluster?
    long LORA_BAND      = 920E6;      // LoRa Band (Thailand)
    bool PABOOST        = true;       // Power Amplify Boost
    bool enableDebug    = false;      // DSR Display

    LoRa_DSR(byte _currentID, byte _destinationID, byte DeviceType = 1, bool _enableDebug = false);
    int begin(long _LORA_BAND = 868E6, bool _PABOOST = true);
    String checkPacket();
    void check_timer(long cur_time = millis());
    void sendDATA(String payload, byte destination);
    void configForLoRaWAN(byte _TXPOWER = 20, byte _SPREADING_FACTOR = 12, long _BANDWIDTH = 500E3, byte _CODING_RATE = 5);
  
  private:
    String nodeFunction[4] = {"DEVICE", "CLUSTER", "GATEWAY", "NONE"};
    // This node is cluster?
    // 0 = End device low battery
    // 1 = Cluster station
    // 2 = Internet cluster station
    // 3 = Not connect
    String code_version = "1.0";

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
    SSD1306Wire *display;

    // LoRaWAN Parameters
    #define TXPOWER 20
    #define RF_PACONFIG_PASELECT_PABOOST 0x80
    #define RF_PACONFIG_PASELECT_RFO 0x00
    #define SPREADING_FACTOR 12
    #define BANDWIDTH 500E3
    #define CODING_RATE 5
    #define PREAMBLE_LENGTH 8
    #define SYNC_WORD 0x34

    #define LEDPIN 25
    uint64_t chipid;
    String MAC;

    // DSR algorithm
    #define maxDestinationRow 4   // number of destination nodes
    #define maxProcess 2          // number of destination nodes
    byte msgID = 0;               // Unique packet ID at source node
    String packets[6] = {"RREQ", "RREP", "DATA", "RERR", "UACK", "MACK"};
    byte routingTable[maxDestinationRow][2] = {}; // Routing Table
    byte cur_process = 0;
    uint16_t myprocesses[maxProcess] = {};
    byte const maxPathListLength = 16; // number of path or hop nodes from destination node

    #define timedout 4000//millisecond
    String UACK_Sign = "4O6XtlTMhy";
    long rreq_timer = 0;
    byte rreq_K = 0;
    long uack_timer = 0;
    byte uack_K = 0;
    long mack_timer = 0;
    byte mack_K = 0;
    byte Kmax = 5;
    byte original_sourceID = 0;
    byte original_destination = 0;

    // byte const maxQueue = 4; // number of data buffer
    // String temp_data[maxQueue] = {};
    String temp_data = "";
    String recieved_data = "";

    void sendRREQ(byte destination);
    void sendRREP(byte path[], byte pathlength, byte nexthop);
    void sendRERR(byte destination);
    void sendUACK(byte original_source, byte original_UID);
    void sendMACK(byte original_source, byte original_UID);

    void onReceive(int packetSize);

    bool arrayIncludeProcess(uint16_t array[], uint16_t process, byte max);
    void arrayAddProcess(uint16_t array[], uint16_t process, byte max);
    void printProcess();

    int arrayIncludeElement(byte array[], byte element, byte max);
    void arrayAddElement(byte array[], byte element, byte max);
    void printArray(byte array[], byte max);

    void pathToTable(byte array[maxDestinationRow][2], byte path[], byte pathlength, byte cur_pos);
    byte tableIncludeDest(byte array[maxDestinationRow][2], byte destination, byte max);
    void tableAddList(byte array[maxDestinationRow][2], byte list[], byte max);
    void tableDelList(byte array[maxDestinationRow][2], byte destination, byte max);
    void tableDelHop(byte array[maxDestinationRow][2], byte nexthop, byte max);
    void printTable();

    void displayData();
    void displayProcess();
    void displayTable();
    void displayTX(String transmit);
    void displayRX(String receive);
    void displayStatus(String status);
};

#endif

