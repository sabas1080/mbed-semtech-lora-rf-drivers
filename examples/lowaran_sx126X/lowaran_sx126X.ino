#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"

// Application helpers
#include "SX126X_LoRaRadio.h"
#include "lorawan/lorastack/phy/LoRaPHYEU868.h"
#include "lorawan/lorastack/phy/LoRaPHYAS923.h"
#include "lorawan/lorastack/phy/LoRaPHYAU915.h"
#include "lorawan/lorastack/phy/LoRaPHYCN470.h"
#include "lorawan/lorastack/phy/LoRaPHYCN779.h"
#include "lorawan/lorastack/phy/LoRaPHYEU433.h"
#include "lorawan/lorastack/phy/LoRaPHYIN865.h"
#include "lorawan/lorastack/phy/LoRaPHYKR920.h"
#include "lorawan/lorastack/phy/LoRaPHYUS915.h"

static uint8_t LORAWAN_DEV_EUI[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t LORAWAN_APP_EUI[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t LORAWAN_APP_KEY[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*
  SX1276_LoRaRadio radio(MBED_CONF_APP_LORA_SPI_MOSI,
                       MBED_CONF_APP_LORA_SPI_MISO,
                       MBED_CONF_APP_LORA_SPI_SCLK,
                       MBED_CONF_APP_LORA_CS,
                       MBED_CONF_APP_LORA_RESET,
                       MBED_CONF_APP_LORA_DIO0,
                       MBED_CONF_APP_LORA_DIO1,
                       MBED_CONF_APP_LORA_DIO2,
                       MBED_CONF_APP_LORA_DIO3,
                       MBED_CONF_APP_LORA_DIO4,
                       MBED_CONF_APP_LORA_DIO5,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL1,
                       MBED_CONF_APP_LORA_RF_SWITCH_CTL2,
                       MBED_CONF_APP_LORA_TXCTL,
                       MBED_CONF_APP_LORA_RXCTL,
                       MBED_CONF_APP_LORA_ANT_SWITCH,
                       MBED_CONF_APP_LORA_PWR_AMP_CTL,
                       MBED_CONF_APP_LORA_TCXO);
*/
SX126X_LoRaRadio radio(P1_12,
                       P1_13,
                       P1_11,
                       P1_10,
                       P1_6,
                       P1_15,
                       P1_14,
                       P1_7,
                       NC,
                       NC,
                       NC);

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
   Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
*/
#define TX_TIMER                        10000

/**
   Maximum number of events for the event queue.
   10 is the safe number for the stack events, however, if application
   also uses the queue for whatever purposes, this number should be increased.
*/
#define MAX_NUMBER_OF_EVENTS            10

/**
   Maximum number of retries for CONFIRMED messages before giving up
*/
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
   Dummy pin for dummy sensor
*/
#define PC_9                            0

/**
   Dummy sensor class object
*/
DS1820  ds1820(PC_9);

/**
  This event queue is the global event queue for both the
  application and stack. To conserve memory, the stack is designed to run
  in the same thread as the application and the application is responsible for
  providing an event queue to the stack that will be used for ISR deferment as
  well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
   Event handler.

   This will be passed to the LoRaWAN stack to queue events for the
   application which in turn drive the application.
*/
static void lora_event_handler(lorawan_event_t event);

static LoRaPHYUS915 phy;
static LoRaWANInterface lorawan(radio, phy);

/**
   Application specific callbacks
*/
static lorawan_app_callbacks_t callbacks;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize LoRaWAN stack
  if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  Serial.println("Mbed LoRaWANStack initialized");

}

void loop() {
  // stores the status of a call to LoRaWAN protocol
  lorawan_status_t retcode;

  // prepare application callbacks
  callbacks.events = mbed::callback(lora_event_handler);
  lorawan.add_app_callbacks(&callbacks);

  // Set number of retries in case of CONFIRMED messages
  if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
      != LORAWAN_STATUS_OK) {
    Serial.println("set_confirmed_msg_retries failed!");
    //return -1;
  }

  Serial.print("CONFIRMED message retries:");
  Serial.println(CONFIRMED_MSG_RETRY_COUNTER);

  // Enable adaptive data rate
  if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
    Serial.println("enable_adaptive_datarate failed!");
    //return false;
  }

  Serial.println("Adaptive data  rate (ADR) - Enabled");

  lorawan_connect_t connect_params;
  connect_params.connect_type = LORAWAN_CONNECTION_OTAA;

  connect_params.connection_u.otaa.dev_eui = LORAWAN_DEV_EUI;
  connect_params.connection_u.otaa.app_eui = LORAWAN_APP_EUI;
  connect_params.connection_u.otaa.app_key = LORAWAN_APP_KEY;
  connect_params.connection_u.otaa.nb_trials = 10;

  retcode = lorawan.connect(connect_params);

  if (retcode == LORAWAN_STATUS_OK || retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
        Serial.println("Connection - In Progress ...");
  } else {
    Serial.print("Connection error, code = ");
    Serial.println(retcode);
    //return false;
  }

  Serial.println("Connection - In Progress ...");

  // make your event queue dispatching events forever
  ev_queue.dispatch_forever();

  //return false;
}

/**
   Sends a message to the Network Server
*/
static void send_message()
{
  uint16_t packet_len;
  int16_t retcode;
  float sensor_value;

  if (ds1820.begin()) {
    ds1820.startConversion();
    sensor_value = ds1820.read();
    Serial.print("Dummy Sensor Value =");
    Serial.println(sensor_value);
    ds1820.startConversion();
  } else {
    Serial.println("No sensor found");
    return;
  }

  packet_len = sprintf((char *) tx_buffer, "Dummy Sensor Value is %3.1f",
                       sensor_value);

  retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                         MSG_UNCONFIRMED_FLAG);

  if (retcode < 0) {
    if (retcode == LORAWAN_STATUS_WOULD_BLOCK)
    {
      Serial.println("send - WOULD BLOCK");
    }
    else {
      Serial.println("send() - Error code ");
      Serial.println(retcode);
    }

    if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
      //retry in 3 seconds
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        ev_queue.call_in(3000, send_message);
      }
    }
    return;
  }

  Serial.print("bytes scheduled for transmission");
  Serial.println(retcode);
  memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
   Receive a message from the Network Server
*/
static void receive_message()
{
  uint8_t port;
  int flags;
  int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

  if (retcode < 0) {
    Serial.print("receive() - Error code");
    Serial.println(retcode);
    return;
  }

  Serial.print("RX Data on port %u (%d bytes): ");
  Serial.print(port);
  Serial.println(retcode);
  for (uint8_t i = 0; i < retcode; i++) {
    Serial.println(rx_buffer[i]);
  }
  Serial.println();

  memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
   Event handler
*/
static void lora_event_handler(lorawan_event_t event)
{
  switch (event) {
    case CONNECTED:
      Serial.println("Connection - Successful");
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        send_message();
      } else {
        ev_queue.call_every(TX_TIMER, send_message);
      }

      break;
    case DISCONNECTED:
      ev_queue.break_dispatch();
      Serial.println("Disconnected Successfully");
      break;
    case TX_DONE:
      Serial.println("Message Sent to Network Server");
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        send_message();
      }
      break;
    case TX_TIMEOUT:
    case TX_ERROR:
    case TX_CRYPTO_ERROR:
    case TX_SCHEDULING_ERROR:
      Serial.print("Transmission Error - EventCode =");
      Serial.println(event);
      // try again
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        send_message();
      }
      break;
    case RX_DONE:
      Serial.println("Received message from Network Server");
      receive_message();
      break;
    case RX_TIMEOUT:
    case RX_ERROR:
      Serial.print("Error in reception - Code = ");
      Serial.println(event);
      break;
    case JOIN_FAILURE:
      Serial.println("OTAA Failed - Check Keys");
      break;
    case UPLINK_REQUIRED:
      Serial.println("Uplink required by NS");
      if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
        send_message();
      }
      break;
    default:
      MBED_ASSERT("Unknown Event");
  }
}
