#include "i2c_simulatedsensor.h"
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_cmu.h"

// ---------------------------------------------------------------------------
// Counter state
// ---------------------------------------------------------------------------

typedef enum { COUNTING_UP, COUNTING_DOWN } Direction_t;

static volatile uint32_t    counter   = COUNTER_MIN;
static volatile Direction_t direction = COUNTING_UP;

// Response frame: [status, P_hi, P_lo, T_hi, T_lo]
static volatile uint8_t tx_byte[5];

// ---------------------------------------------------------------------------
// I2C state machine
// ---------------------------------------------------------------------------

typedef enum {
  STATE_IDLE,
  STATE_RECEIVE_CMD,
  STATE_CONVERSION_READY,
  STATE_SEND_DATA,
} SensorState_t;

static volatile SensorState_t sensor_state = STATE_IDLE;
static volatile uint8_t       tx_index     = 0;

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

void i2c_simulatedsensor_init(void)
{
  CMU_ClockEnable(cmuClock_HFPER, true); // enable clocks
  CMU_ClockEnable(cmuClock_I2C0,  true);
  CMU_ClockEnable(cmuClock_GPIO,  true);

  // SDA = PC0 (loc 4), SCL = PC1 (loc 4)
  GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUp, 1);

  I2C0->ROUTEPEN  = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
  I2C0->ROUTELOC0 = (4 << _I2C_ROUTELOC0_SDALOC_SHIFT)
                  | (4 << _I2C_ROUTELOC0_SCLLOC_SHIFT);

  I2C0->SADDR     = SENSOR_I2C_ADDR << 1; // programs address 0x40 into hardware, bit shift to make room for R/W bit
  I2C0->SADDRMASK = 0xFE;
  I2C0->CTRL      = I2C_CTRL_SLAVE; // puts i2C0 into mode
  I2C0->CTRL     |= I2C_CTRL_AUTOACK; // enables AUTOACK so hardwareACKs received bytes automatically
  I2C0->CMD       = I2C_CMD_CLEARTX;

  I2C_IntClear(I2C0, _I2C_IFC_MASK); // clears pending interrupts flags
  I2C0->IEN = I2C_IEN_ADDR // enables address
            | I2C_IEN_RXDATAV
            | I2C_IEN_ACK
            | I2C_IEN_SSTOP
            | I2C_IEN_BUSERR
            | I2C_IEN_ARBLOST;

  I2C0->CTRL |= I2C_CTRL_EN; // enables i2c0 peripheral

  NVIC_ClearPendingIRQ(I2C0_IRQn); // clears any pending NVIC interrupts
  NVIC_EnableIRQ(I2C0_IRQn); // tells ARM CPU to start responding to I2C0 interrupts
}

// ---------------------------------------------------------------------------
// IRQ handler
// ---------------------------------------------------------------------------

void I2C0_IRQHandler(void)
{
  uint32_t pending = I2C_IntGetEnabled(I2C0);

  // ---- Address match ----
  if (pending & I2C_IF_ADDR) { // IDLE to RECEIVE CMD, master writes to 0x40)
    I2C_IntClear(I2C0, I2C_IFC_ADDR);  // check if address match fired
    I2C_IntClear(I2C0, I2C_IFC_ACK); // clear ADDR and ACK flags
    pending &= ~I2C_IF_ACK;

    uint8_t addr_byte = I2C0->RXDATA; // reads address byte out of RXDATA
    pending &= ~I2C_IF_RXDATAV;

    if (addr_byte & 0x01) { // checks bit 0 to see of 0 for write or 1 for read
      // Read transaction: master wants the conversion result
      sensor_state = STATE_SEND_DATA;
      I2C0->TXDATA = tx_byte[0];
      tx_index     = 1;
    } else {
      // Write transaction: master is sending command (0xAC)
      sensor_state = STATE_RECEIVE_CMD; // if bit is 0 for write then -> write transaction so state = state_receive_cmd
    }
    I2C0->CMD = I2C_CMD_ACK;
  }

  // ---- Command byte received ----
  if (pending & I2C_IF_RXDATAV) { //checks if data byte has been received
    uint8_t cmd = I2C0->RXDATA; // reads command byte out of RXDATA

    if (sensor_state == STATE_RECEIVE_CMD && cmd == 0xAC) { // checks 0xAC and in state_receive_cmd
      // Advance counter (simulate new measurement being triggered)
      if (direction == COUNTING_UP) {
        if (counter >= COUNTER_MAX) {
          direction = COUNTING_DOWN;
        } else {
          counter += COUNTER_STEP;
        }
      } else {
        if (counter <= COUNTER_MIN) {
          direction = COUNTING_UP;
        } else {
          counter -= COUNTER_STEP;
        }
      }

      uint32_t snap = counter; // snapshots counter into local variable as a safe copy
      tx_byte[0] = 0x40;                     // status: normal (bit 6 always 1, matches real Keller sensor)
      tx_byte[1] = (uint8_t)(snap >> 8);     // pressure high byte
      tx_byte[2] = (uint8_t)(snap & 0xFF);   // pressure low byte
      tx_byte[3] = 0x5F;                     // temperature high byte (~25 °C raw)
      tx_byte[4] = 0x40;                     // temperature low byte

      sensor_state = STATE_CONVERSION_READY;
    }
    I2C0->CMD = I2C_CMD_ACK; // acknowledges command byte
  }

  // ---- Master ACKed our byte — send the next ----
  if (pending & I2C_IF_ACK) {  // check if master ACKed last byte
    I2C_IntClear(I2C0, I2C_IFC_ACK);
    if (sensor_state == STATE_SEND_DATA && tx_index < 5) { // if still sending and bytes remain, then
      I2C0->TXDATA = tx_byte[tx_index++]; // send next byte
    }
  }

  // ---- Stop ----
  if (pending & I2C_IF_SSTOP) {
    I2C_IntClear(I2C0, I2C_IFC_SSTOP);
    // Keep CONVERSION_READY across the stop between write and read transactions
    if (sensor_state != STATE_CONVERSION_READY) {
      sensor_state = STATE_IDLE;
      tx_index     = 0;
    }
  }

  // ---- Errors ----
  if (pending & (I2C_IF_BUSERR | I2C_IF_ARBLOST)) {
    I2C_IntClear(I2C0, I2C_IFC_BUSERR | I2C_IFC_ARBLOST);
    I2C0->CMD    = I2C_CMD_ABORT;
    sensor_state = STATE_IDLE;
  }
}
