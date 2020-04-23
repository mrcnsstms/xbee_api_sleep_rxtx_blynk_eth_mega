//#include "blynk_ethernet_defs.h"
#include "millisDelay.h"
#include "version.h"
#include "Xbee_lib.h"
#include "Xbee_lib_defs.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SoftwareSerial.h>
#include <Print_lib.h>

SoftwareSerial ss(7,8);  // (rx,tx)
#define BLYNK_PRINT Serial // must be before Blynk include
#include <BlynkSimpleEthernet.h>

char auth[] = "ed2b10f6ec7a46b1aef3c363778c5973";

#define W5100_CS   10
#define SDCARD_CS   4
#define LED_PIN    13
#define WAKE_PIN    3
#define DS180_TEMP  4

Xbee_lib m_xbee(&ss);
Print_lib m_printer(&ss);

millisDelay m_wireless_timer;
millisDelay m_remote_status_timer;

uint8_t m_usb_state = 0;
uint8_t m_usb_count = 0;
struct Msg_data m_rx_blynk_msg;
uint8_t m_tx_addr = 0;
uint8_t j = 0;

struct Msg_data m_tx_msg[5];
struct Msg_data m_rx_msg[5];

OneWire oneWire(DS180_TEMP);
DallasTemperature sensor(&oneWire);

uint8_t m_tx_count[6] = {}; // number of xbees in network

//////////////////////////////////////////////////////////////////////

void setup()
{
  // allow time to switch to xbee mode on pcb
  delay(2000);

  Serial.begin(19200);
  Serial.println("**** SERIAL ****");

  m_printer.Print();

  ss.begin(19200);   //m_xbee.Begin(19200);
  ss.print("xbee_api_sleep_txrx_usb_coordinator : ");
  ss.println(version);

  // pin definitions
  pinMode(LED_PIN, OUTPUT);
  pinMode(WAKE_PIN, INPUT);
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); // Deselect the SD card

  // delay for handling wireless interface
  m_wireless_timer.start(1);

  // delay for reporting status
  m_remote_status_timer.start(14000);

  // callback for when valid data received
  m_xbee.Set_callback(Message_received);

  // Start up the library for dallas temp
  sensor.begin();

  //setup blynk parameters
  Blynk.begin(auth);   // must be last call
}

//////////////////////////////////////////////////////////////////////

void loop()
{
  //Run blynk
  Blynk.run();

  if(m_wireless_timer.justFinished())
  {
    m_wireless_timer.repeat();
    handle_wireless();
  }

  if(m_remote_status_timer.justFinished())
  {
    m_remote_status_timer.repeat();
    handle_remote_status();
  }
}

////////////////////////////////////////
//Write data to blynk
void Post_blynk_data(const struct Msg_data blynk_data)
{
  ss.println("Posting to Blynk");

  WidgetMap myMap1(V13);
  int index1 = 1;
  float lat1 = 44.484306;
  float lon1 = -73.209325;
  myMap1.location(index1, lat1, lon1, "matt, parilla");

  int index2 = 2;
  float lat2 = 44.487366;
  float lon2 = -73.211969;
  myMap1.location(index2, lat2, lon2, "em");
  //Blynk.setProperty(V13, "color", "#D3435C");


  WidgetTable table(V14);
  static bool add = true;
  if(add)
  {
    table.addRow(1, "Xbee1", millis() / 1000);
    table.addRow(2, "Xbee2", millis() / 1000);
    table.addRow(3, "Xbee3", millis() / 1000);
    add = false;
  }
  else
  {
    table.updateRow(1, "Xbee1 new", "UpdatedValue1");
    table.updateRow(2, "Xbee2 new", "UpdatedValue2");
    table.updateRow(3, "Xbee3 new", "UpdatedValue3");
  }

  table.pickRow(2);

  switch(blynk_data.address)
  {
    case 1 :
    {
      float battery = blynk_data.payload[2] * 8 * 0.003157; // 0.0032258 @ 3.3V
      float temp = blynk_data.payload[1];
      uint8_t light = map(blynk_data.payload[0], 0, 255, 0, 100);

      Blynk.virtualWrite(V0, battery);
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V2, light);
      break;
    }
    case 2 :
    {
      float battery = blynk_data.payload[2] * 8 * 0.003157; // 0.0032258 @ 3.3V
      float temp = blynk_data.payload[1];
      uint8_t light = map(blynk_data.payload[0], 0, 255, 0, 100);

      Blynk.virtualWrite(V5, battery);
      Blynk.virtualWrite(V6, temp);
      Blynk.virtualWrite(V7, light);
      break;
    }
    case 3 :
    {
      float battery = blynk_data.payload[2] * 8 * 0.003157; // 0.0032258 @ 3.3V
      float temp = blynk_data.payload[1];
      uint8_t light = map(blynk_data.payload[0], 0, 255, 0, 100);

      Blynk.virtualWrite(V10, battery);
      Blynk.virtualWrite(V11, temp);
      Blynk.virtualWrite(V12, light);
      break;
    }
    case 4 :
      break;

    case 5 :
      break;

    case 6 :
      break;
  }
}

BLYNK_WRITE(V14)
{
 String cmd = param[0].asStr();
 if (cmd == "select")
 {
   //row in table was selected.
   int rowId = param[1].asInt();
   ss.print("Row ID: ");
   ss.print(rowId);
   ss.println(" selected");
 }

 if (cmd == "deselect")
 {
   //row in table was selected.
   int rowId = param[1].asInt();
   ss.print("Row ID: ");
   ss.print(rowId);
   ss.println(" de-selected");
 }
 if (cmd == "order")
 {
   //rows in table where reodered
   int oldRowIndex = param[1].asInt();
   int newRowIndex = param[2].asInt();
 }
}


template <typename T>
T Post_blynk(T pin, T value)
{

}

uint8_t Convert_payload(uint8_t payload_id[], uint8_t length)
{
  switch(payload_id[0])
  {
    case 'i' :
    break;
  }
}

////////////////////////////////////////
//Get Value from blynk V10
BLYNK_WRITE(XBEE2_DIG_6)
{
  ss.println("Blynk command RX'd");
  m_rx_blynk_msg.address = 3;
  m_rx_blynk_msg.payload_id = CMD_ID::IO_OUT;
  m_rx_blynk_msg.payload_len = 3;
  m_rx_blynk_msg.payload[0] = 6; // pin 6
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  if(pinValue)
  {
    m_rx_blynk_msg.payload[1] = 1; // state
    m_rx_blynk_msg.payload[2] = 1; // duration (0 is permanent)
  }

  else
  {
    m_rx_blynk_msg.payload[1] = 0; // state
    m_rx_blynk_msg.payload[2] = 0; // duration (0 is permanent)
  }
  update_tx_msg(m_rx_blynk_msg);
  m_xbee.Clear_msg(m_rx_blynk_msg);
}

BLYNK_WRITE(V5)
{
  ss.println("Blynk command RX'd");
  m_rx_blynk_msg.address = 3;
  m_rx_blynk_msg.payload_id = 'o';
  m_rx_blynk_msg.payload_len = 3;
  m_rx_blynk_msg.payload[0] = 6; // pin 6
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable

  if(pinValue)
  {
    m_rx_blynk_msg.payload[1] = 1; // state
    m_rx_blynk_msg.payload[2] = 1; // duration (0 is permanent)
  }

  else
  {
    m_rx_blynk_msg.payload[1] = 0; // state
    m_rx_blynk_msg.payload[2] = 0; // duration (0 is permanent)
  }
  update_tx_msg(m_rx_blynk_msg);
  m_xbee.Clear_msg(m_rx_blynk_msg);
}


void handle_remote_status()
{
  ss.println("Handle remote status");
  for(int i = 0; i < 5; i++)
  {
    if(m_rx_msg[i].valid)
    {
      ss.print("Remote Address: ");
      ss.println(m_rx_msg[i].address);
      ss.print("Light[%]: ");
      uint8_t light = map(m_rx_msg[i].payload[0], 0, 255, 0, 100);
      ss.println(light);
      ss.print("Temperature[F]: ");
      ss.println(m_rx_msg[i].payload[1]);
      ss.print("Battery[V]: ");
      float battery = m_rx_msg[i].payload[2] * 8 * 0.003157; // 0.0032258 @ 3.3V
      ss.println(battery);
      ss.println();
      Post_blynk_data(m_rx_msg[i]);
    }
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_tx_msg(const struct Msg_data blynk_data)
{
  ss.println("Updating TX msg");
  m_xbee.Print_msg(blynk_data, false);
  m_tx_msg[blynk_data.address] = blynk_data;
}

//////////////////////////////////////////////////////////////////////

void handle_wireless()
{
  while(Serial.available())
  {
    m_xbee.Process_byte(Serial.read());
  }
}

//////////////////////////////////////////////////////////////////////

void Message_received(const struct Msg_data rx_data)
{
  ss.println("Message_received");
  m_xbee.Print_msg(rx_data);

  switch(rx_data.payload_id)
  {
    case CMD_ID::NO_ACK :
      ss.println("Rx'd No Ack");
      // do nothing, no response
      break;

    case CMD_ID::ACK :
      ss.println("Rx'd Ack");
      // build response
      break;

    case CMD_ID::IO_IN :
      ss.println("Rx'd IO in");

      // update rx data container
      for(int i = 0; i < sizeof(rx_data.payload); i++)
      {
        m_rx_msg[rx_data.address] = rx_data;
      }
      //build response

      break;

    default :
      ss.print("Unknown CMD::ID: ");
      ss.println(rx_data.payload_id);
  }


  // build message, insert payloads
  struct Msg_data tx_msg;
  tx_msg.length = 23;
  tx_msg.frame_type = 0x10;
  tx_msg.address = rx_data.address;
  tx_msg.payload_len = m_tx_msg[tx_msg.address].payload_len;
  tx_msg.payload_cnt = m_tx_count[rx_data.address];
  tx_msg.payload_id = m_tx_msg[tx_msg.address].payload_id;;

  // payload
  tx_msg.payload[0] = m_tx_msg[tx_msg.address].payload[0];
  tx_msg.payload[1] = m_tx_msg[tx_msg.address].payload[1];
  tx_msg.payload[2] = m_tx_msg[tx_msg.address].payload[2];

  // use enum from transmit status
  uint8_t tx_ok = m_xbee.Transmit_data(tx_msg);
  if(tx_ok == 1)
  {
    m_tx_count[rx_data.address]++;
  }

  // if only meant to send this once
  if(true)
  {
    m_xbee.Clear_msg(m_tx_msg[tx_msg.address]);
  }
}

//////////////////////////////////////////////////////////////////////

uint8_t getDallasTemp()
{
  delay(10);
  sensor.requestTemperatures(); // Send the command to get temperatures
  float tempF = (sensor.getTempCByIndex(0) * 9.0 / 5.0) + 32;

  if(tempF < 0)
  {
    tempF = 0;
  }

  return tempF;
}
