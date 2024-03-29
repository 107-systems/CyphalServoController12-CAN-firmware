/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/CyphalServoController12-CAN-firmware/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <limits>

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>
#include <107-Arduino-Servo-RP2040.h>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const MCP2515_CS_PIN  = 17;
static int const MCP2515_INT_PIN = 20;
static int const LED2_PIN        = 21; /* GP21 */
static int const LED3_PIN        = 22; /* GP22 */

static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static unsigned long const SERVO_PULSE_WIDTH_MESSAGE_TIMEOUT_ms = 1000;

static uint16_t const SERVO_PULSE_WIDTH_VALVE_OFF_ms = 1500;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame);
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

ArduinoMCP2515 mcp2515([]() { digitalWrite(MCP2515_CS_PIN, LOW); },
                       []() { digitalWrite(MCP2515_CS_PIN, HIGH); },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr,
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnError, error code = \"%s\"", MCP2515::toStr(err_flag)); },
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnWarning, warning code = \"%s\"", MCP2515::toStr(err_flag)); });

cyphal::Node::Heap<cyphal::Node::DEFAULT_O1HEAP_SIZE> node_heap;
cyphal::Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

cyphal::Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);

cyphal::Subscription servo_pulse_width_sub;
static unsigned long last_servo_pulse_width_msg_timestamp = 0;

cyphal::ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

/* SERVOS *****************************************************************************/

class ServoInterface
{
public:
  virtual ~ServoInterface() { }
  virtual void attach(pin_size_t const pin) = 0;
  virtual void writeMicroseconds(uint16_t const pulse_width_us) = 0;
};

class ServoAdapter_PIO final : public ServoInterface
{
private:
  Servo & _servo;
public:
  ServoAdapter_PIO(Servo & servo) : _servo{servo} { }
  virtual ~ServoAdapter_PIO() { }
  void attach(pin_size_t const pin) override { _servo.attach(pin); }
  void writeMicroseconds(uint16_t const pulse_width_us) { _servo.writeMicroseconds(pulse_width_us); }
};
static Servo servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, servo_6, servo_7;

class ServoAdapter_RP2040 final : public ServoInterface
{
private:
  _107_::Servo & _servo;
public:
  ServoAdapter_RP2040(_107_::Servo & servo) : _servo{servo} { }
  virtual ~ServoAdapter_RP2040() { }
  void attach(pin_size_t const pin) override { _servo.attach(pin); }
  void writeMicroseconds(uint16_t const pulse_width_us) { _servo.writeMicroseconds(pulse_width_us); }
};
static _107_::Servo servo_8, servo_9, servo_10, servo_11;

static size_t constexpr NUM_SERVOS = 12;
static std::array<std::shared_ptr<ServoInterface>, NUM_SERVOS> servo_ctrl
{
  std::make_shared<ServoAdapter_PIO>   (servo_0),
  std::make_shared<ServoAdapter_PIO>   (servo_1),
  std::make_shared<ServoAdapter_PIO>   (servo_2),
  std::make_shared<ServoAdapter_PIO>   (servo_3),
  std::make_shared<ServoAdapter_PIO>   (servo_4),
  std::make_shared<ServoAdapter_PIO>   (servo_5),
  std::make_shared<ServoAdapter_PIO>   (servo_6),
  std::make_shared<ServoAdapter_PIO>   (servo_7),
  std::make_shared<ServoAdapter_RP2040>(servo_8),
  std::make_shared<ServoAdapter_RP2040>(servo_9),
  std::make_shared<ServoAdapter_RP2040>(servo_10),
  std::make_shared<ServoAdapter_RP2040>(servo_11)
};
static std::array<pin_size_t, NUM_SERVOS> SERVO_PINS =
{
   2, /* GP2  = SERVO00 */
   3, /* GP3  = SERVO01 */
   6, /* GP6  = SERVO02 */
   7, /* GP7  = SERVO03 */
   8, /* GP8  = SERVO04 */
   9, /* GP9  = SERVO05 */
  10, /* GP10 = SERVO06 */
  11, /* GP11 = SERVO07 */
  12, /* GP12 = SERVO08 */
  13, /* GP13 = SERVO09 */
  14, /* GP14 = SERVO10 */
  15  /* GP15 = SERVO11 */
};

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id             = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_pulse_width = std::numeric_limits<CanardPortID>::max();

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id              = node_registry->expose("cyphal.node.id", {true}, node_id);
const auto reg_ro_cyphal_node_description     = node_registry->route ("cyphal.node.description", {true}, []() { return "L3X-Z VALVE CONTROLLER"; });
const auto reg_rw_cyphal_sub_pulse_width_id   = node_registry->expose("cyphal.sub.pulse_width.id", {true}, port_id_pulse_width);
const auto reg_ro_cyphal_pub_pulse_width_type = node_registry->route ("cyphal.sub.pulse_width.type", {true}, []() { return "uavcan.primitive.array.Natural16.1.0"; });

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while (!Serial) { } /* Only for debug. */
  delay(1000);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));

  if (port_id_pulse_width != std::numeric_limits<CanardPortID>::max())
    servo_pulse_width_sub = node_hdl.create_subscription<uavcan::primitive::array::Natural16_1_0>(
      port_id_pulse_width,
      [](uavcan::primitive::array::Natural16_1_0 const & msg)
      {
        last_servo_pulse_width_msg_timestamp = millis();

        for (size_t sid = 0; sid < msg.value.size(); sid++)
        {
          if (sid >= NUM_SERVOS) {
            DBG_WARNING("Servo pulse width message contains more than %d entries", NUM_SERVOS);
            return;
          }

          uint16_t const pulse_width_us = msg.value[sid];

          if (pulse_width_us < DEFAULT_MIN_PULSE_WIDTH) {
            DBG_ERROR("Servo %d pulse width (%d) below minimum valid value (%d)", sid, pulse_width_us, DEFAULT_MIN_PULSE_WIDTH);
            continue;
          }

          if (pulse_width_us > DEFAULT_MAX_PULSE_WIDTH) {
            DBG_ERROR("Servo %d pulse width (%d) above maximum valid value (%d)", sid, pulse_width_us, DEFAULT_MAX_PULSE_WIDTH);
            continue;
          }

          servo_ctrl[sid]->writeMicroseconds(pulse_width_us);
        }
      });

  DBG_INFO("Node ID: %d\n\r\tPULSE WIDTH ID = %d",
           node_id, port_id_pulse_width);

  /* NODE INFO ************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* cyphal.node.Version.1.0 protocol_version */
    1, 0,
    /* cyphal.node.Version.1.0 hardware_version */
    1, 0,
    /* cyphal.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.CyphalServoController12/CAN"
  );

  /* Setup all servos. */
  for (size_t s = 0; s < NUM_SERVOS; s++)
    servo_ctrl[s]->attach(SERVO_PINS[s]);

  /* Turn all valves off. */
  for (size_t sid = 0; sid < NUM_SERVOS; sid++)
    servo_ctrl[sid]->writeMicroseconds(SERVO_PULSE_WIDTH_VALVE_OFF_ms);

  /* Setup LED pins and initialize */
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  /* Setup SPI access */
  SPI.begin();
  SPI.beginTransaction(MCP2515x_SPI_SETTING);
  pinMode(MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MCP2515_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);

  CanardFilter const CAN_FILTER_SERVICES = canardMakeFilterForServices(node_id);
  DBG_INFO("CAN Filter #1\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           CAN_FILTER_SERVICES.extended_mask,
           CAN_FILTER_SERVICES.extended_can_id);

  /* Only pass service requests/responses for this node ID through to receive buffer #0. */
  uint32_t const RXMB0_MASK = CAN_FILTER_SERVICES.extended_mask;
  size_t const RXMB0_FILTER_SIZE = 2;
  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | CAN_FILTER_SERVICES.extended_can_id,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);

  /* Only allow messages containing servo pulse width information to pass. */
  CanardFilter const CAN_FILTER_PULSE_WIDTH = canardMakeFilterForSubject(port_id_pulse_width);
  DBG_INFO("CAN Filter #2\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           CAN_FILTER_PULSE_WIDTH.extended_mask,
           CAN_FILTER_PULSE_WIDTH.extended_can_id);

  uint32_t const RXMB1_MASK = CAN_FILTER_PULSE_WIDTH.extended_mask;
  size_t const RXMB1_FILTER_SIZE = 4;
  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | CAN_FILTER_PULSE_WIDTH.extended_can_id,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration and enable MCP2515. */
  mcp2515.setNormalMode();

  /* Enable watchdog. */
  rp2040.wdt_begin(WATCHDOG_DELAY_ms);
  rp2040.wdt_reset();

  DBG_INFO("Init complete.");
}

void loop()
{
  /* Deal with all pending events of the MCP2515 -
   * signaled by the INT pin being driven LOW.
   */
  while(digitalRead(MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;

  unsigned long const now = millis();

  /* Publish the heartbeat once/second */
  if((now - prev_heartbeat) > UPDATE_PERIOD_HEARTBEAT_ms)
  {
    prev_heartbeat = now;

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    digitalWrite(LED2_PIN, !digitalRead(LED2_PIN));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  /* If we have not received a servo pulse width message for
   * more than SERVO_PULSE_WIDTH_MESSAGE_TIMEOUT_ms than
   * we shut all valves off.
   */
  if ((now - last_servo_pulse_width_msg_timestamp) > SERVO_PULSE_WIDTH_MESSAGE_TIMEOUT_ms) {
    for (size_t sid = 0; sid < NUM_SERVOS; sid++)
      servo_ctrl[sid]->writeMicroseconds(SERVO_PULSE_WIDTH_VALVE_OFF_ms);
  }

  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    rp2040.wdt_reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  digitalWrite(LED3_PIN, !digitalRead(LED3_PIN));
  node_hdl.onCanFrameReceived(frame);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { rp2040.wdt_reset(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    /* Feed the watchdog. */
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
