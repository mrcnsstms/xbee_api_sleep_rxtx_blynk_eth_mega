//#include "blynk_ethernet_defs.h"
#include "millisDelay.h"
#include "version.h"
#include "Xbee_lib.h"
#include "Xbee_lib_defs.h"
#include <SPI.h>
#include <Ethernet.h>
#include <Print_lib.h>

#define BLYNK_PRINT Serial // must be before Blynk include
#include <BlynkSimpleEthernet.h>

#define W5100_CS   10
#define SDCARD_CS   4
#define LED_PIN    13

const char auth[] = "v1KVwnFGjFTRMiuZUeZyL9XAs7CGB43r";

Print_lib m_hw(&Serial);
Xbee_lib m_xbee(&m_hw);

millisDelay m_remote_status_timer;

struct Msg_data m_rx_blynk_msg;
struct Msg_data m_tx_msg[5];
struct Msg_data m_rx_msg[5];

uint8_t m_tx_count[6] = {}; // number of xbees in network

//////////////////////////////////////////////////////////////////////

void setup()
{
  // allow time to switch to xbee mode on pcb
  delay(2000);
  Serial1.begin(19200);
  m_hw.Begin(19200);   //m_xbee.Begin(19200);
  m_hw.Print("xbee_api_sleep_txrx_usb_coordinator : ");
  m_hw.Println(version);

  // pin definitions
  pinMode(LED_PIN, OUTPUT);
  pinMode(WAKE_PIN, INPUT);
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH); // Deselect SD card
  pinMode(W5100_CS, OUTPUT);
  digitalWrite(W5100_CS, HIGH);

  // delay for reporting status
  m_remote_status_timer.start(14000);

  // callback for when valid data received
  m_xbee.Set_callback(Message_received);

  //setup blynk parameters
  Blynk.begin(auth);   // must be last call
}

//////////////////////////////////////////////////////////////////////

void loop()
{
  //Run blynk
  Blynk.run();

  handle_wireless();

  if(m_remote_status_timer.justFinished())
  {
    m_remote_status_timer.repeat();
    handle_remote_status();
  }
}

//////////////////////////////////////////////////////////////////////

void Transmit_frame(const uint8_t* frame, const uint8_t length)
{
  // doesn't work properly, tx's huge frame
  //m_hw.Print_array("TX frame: ", frame, length);
  m_hw.Print_array(frame, length, HEX);
  delay(10);
  Serial1.write(frame, length);
  delay(10);
}


////////////////////////////////////////
//Write data to blynk
void Post_blynk_data(const struct Msg_data blynk_data)
{
  m_hw.Println("Posting to Blynk");

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
   m_hw.Print("Row ID: ");
   m_hw.Print(rowId);
   m_hw.Println(" selected");
 }

 if (cmd == "deselect")
 {
   //row in table was selected.
   int rowId = param[1].asInt();
   m_hw.Print("Row ID: ");
   m_hw.Print(rowId);
   m_hw.Println(" de-selected");
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
  m_hw.Println("Blynk command RX'd");
  m_rx_blynk_msg.address = 3;
  m_rx_blynk_msg.payload_id = CMD_ID::IO_OUT;
  m_rx_blynk_msg.payload_len = 3;
  m_rx_blynk_msg.payload[0] = 6; // pin 6
  int pinValue = param.asInt(); // am_hwigning incoming value from pin V1 to a variable

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
  m_hw.Println("Blynk command RX'd");
  m_rx_blynk_msg.address = 3;
  m_rx_blynk_msg.payload_id = 'o';
  m_rx_blynk_msg.payload_len = 3;
  m_rx_blynk_msg.payload[0] = 6; // pin 6
  int pinValue = param.asInt(); // am_hwigning incoming value from pin V1 to a variable

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
  m_hw.Println("Handle remote status");
  for(int i = 0; i < 5; i++)
  {
    if(m_rx_msg[i].valid)
    {
      m_hw.Print("Remote address: ");
      m_hw.Println(m_rx_msg[i].address);
      m_hw.Print("Light[%]: ");
      uint8_t light = map(m_rx_msg[i].payload[0], 0, 255, 0, 100);
      m_hw.Println(light);
      m_hw.Print("Temperature[F]: ");
      m_hw.Println(m_rx_msg[i].payload[1]);
      m_hw.Print("Battery[V]: ");
      float battery = m_rx_msg[i].payload[2] * 8 * 0.003157; // 0.0032258 @ 3.3V
      m_hw.Println(battery);
      m_hw.Println();
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
  m_hw.Println("Updating TX msg");
  m_hw.Print_msg(blynk_data);
  m_tx_msg[blynk_data.address] = blynk_data;
}

//////////////////////////////////////////////////////////////////////

void handle_wireless()
{
  while(Serial1.available())
  {
    m_xbee.Process_byte(Serial1.read());
  }
}

//////////////////////////////////////////////////////////////////////

void Message_received(const struct Msg_data rx_data)
{
//  m_hw.Println("Mem_hwage_received");
//  m_hw.Print_msg(rx_data, false);

  switch(rx_data.payload_id)
  {
    case CMD_ID::NO_ACK :
      m_hw.Println("Rx'd No Ack");
      // do nothing, no response
      break;

    case CMD_ID::ACK :
      m_hw.Println("Rx'd Ack");
      // build response
      break;

    case CMD_ID::IO_IN :
      m_hw.Println("Rx'd IO in");

      // update rx data container
      for(int i = 0; i < sizeof(rx_data.payload); i++)
      {
        m_rx_msg[rx_data.address] = rx_data;
      }
      //build response

      break;

    default :
      m_hw.Print("Unknown CMD::ID: ");
      m_hw.Println(rx_data.payload_id);
  }


  // build mem_hwage, insert payloads
  struct Msg_data tx_msg;
  tx_msg.frame_type = 0x10;
  tx_msg.address = rx_data.address;
  tx_msg.payload_len = m_tx_msg[tx_msg.address].payload_len;
  tx_msg.payload_cnt = m_tx_count[rx_data.address];
  tx_msg.payload_id = m_tx_msg[tx_msg.address].payload_id;;

  // payload
  tx_msg.payload[0] = m_tx_msg[tx_msg.address].payload[0];
  tx_msg.payload[1] = m_tx_msg[tx_msg.address].payload[1];
  tx_msg.payload[2] = m_tx_msg[tx_msg.address].payload[2];

  uint8_t tx_array[sizeof(tx_msg.payload) + 20];
  bool tx_ok = m_xbee.Build_frame(tx_msg, tx_array);
  if(tx_ok)
  {
    Transmit_frame(tx_array, sizeof(tx_array));
    m_tx_count[rx_data.address]++;
  }

  // if only meant to send this once
  if(true)
  {
    m_xbee.Clear_msg(m_tx_msg[tx_msg.address]);
  }
}

