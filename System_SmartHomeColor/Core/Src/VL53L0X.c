// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <stdint.h>
#include "stm32f4xx_hal.h" // Change it for your requirements.
#include <string.h> // For memcpy
#include <stdbool.h> // For bool type
#include "VL53L0X.h"

//---------------------------------------------------------
// Module-level (static) variables
//---------------------------------------------------------
static uint8_t g_i2cAddr = ADDRESS_DEFAULT;
static uint16_t g_ioTimeout = 0;  // no timeout by default
static bool g_isTimeout = false; // Flag to indicate if timeout occurred
static uint16_t g_timeoutStartMs; // Variable for timeout measurement
static uint8_t g_stopVariable; // read by init and used when starting measurement
static uint32_t g_measTimBudUs; // Measurement Timing Budget in microseconds

// Static I2C Handle variable - Copy the handle passed during init here
static I2C_HandleTypeDef vl53l0x_i2c_handler;

// Buffer for I2C messages
static uint8_t msgBuffer[4];

// I2C Defines
#define I2C_TIMEOUT 100 // I2C timeout in ms
#define I2C_READ 1
#define I2C_WRITE 0
// HAL Status variable
static HAL_StatusTypeDef i2cStat;


//---------------------------------------------------------
// Internal function prototypes (static)
//---------------------------------------------------------
static bool getSpadInfo(uint8_t *count, bool *type_is_aperture);
static void getSequenceStepEnables(SequenceStepEnables * enables);
static void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
static bool performSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static void startTimeoutInternal(); // Renamed to avoid conflict with macro in header
static bool checkTimeoutExpiredInternal(); // Renamed to avoid conflict with macro in header

//---------------------------------------------------------
// Timeout Helper Functions (Internal)
//---------------------------------------------------------
static void startTimeoutInternal() {
    g_timeoutStartMs = HAL_GetTick();
}

static bool checkTimeoutExpiredInternal() {
    return (g_ioTimeout > 0 && ((uint16_t)HAL_GetTick() - g_timeoutStartMs) > g_ioTimeout);
}


//---------------------------------------------------------
// I2C communication Functions (using static handler)
//---------------------------------------------------------
// Write an 8-bit register
static void writeReg(uint8_t reg, uint8_t value) {
  msgBuffer[0] = value; // Assign the value to the buffer.
  // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Write(&vl53l0x_i2c_handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
  // Optional: Check i2cStat for errors HAL_OK
}

// Write a 16-bit register
static void writeReg16Bit(uint8_t reg, uint16_t value){
  // Handle endianness explicitly (VL53L0X expects big-endian)
  msgBuffer[0] = (value >> 8) & 0xFF; // MSB
  msgBuffer[1] = value & 0xFF;        // LSB
  // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Write(&vl53l0x_i2c_handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
}

// Write a 32-bit register
static void writeReg32Bit(uint8_t reg, uint32_t value){
  // Handle endianness explicitly (VL53L0X expects big-endian)
  msgBuffer[0] = (value >> 24) & 0xFF;
  msgBuffer[1] = (value >> 16) & 0xFF;
  msgBuffer[2] = (value >> 8) & 0xFF;
  msgBuffer[3] = value & 0xFF;
  // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Write(&vl53l0x_i2c_handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
}

// Read an 8-bit register
static uint8_t readReg(uint8_t reg) {
  uint8_t value;
  // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Read(&vl53l0x_i2c_handler, g_i2cAddr | I2C_READ, reg, 1, &value, 1, I2C_TIMEOUT);
  return value;
}

// Read a 16-bit register
static uint16_t readReg16Bit(uint8_t reg) {
  uint16_t value;
  uint8_t buffer[2];
  // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Read(&vl53l0x_i2c_handler, g_i2cAddr | I2C_READ, reg, 1, buffer, 2, I2C_TIMEOUT);
  // Combine bytes (big-endian)
  value = (uint16_t)buffer[0] << 8 | buffer[1];
  return value;
}

// Read a 32-bit register
static uint32_t readReg32Bit(uint8_t reg) {
  uint32_t value;
  uint8_t buffer[4];
   // *** MODIFIED: Use static handler ***
  i2cStat = HAL_I2C_Mem_Read(&vl53l0x_i2c_handler, g_i2cAddr | I2C_READ, reg, 1, buffer, 4, I2C_TIMEOUT);
  // Combine bytes (big-endian)
  value = (uint32_t)buffer[0] << 24 | (uint32_t)buffer[1] << 16 | (uint32_t)buffer[2] << 8 | buffer[3];
  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
static void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count){
   // *** MODIFIED: Use static handler ***
  // Note: The source buffer 'src' is passed directly
  i2cStat = HAL_I2C_Mem_Write(&vl53l0x_i2c_handler, g_i2cAddr | I2C_WRITE, reg, 1, (uint8_t*)src, count, I2C_TIMEOUT);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
static void readMulti(uint8_t reg, uint8_t * dst, uint8_t count) {
    // *** MODIFIED: Use static handler ***
	i2cStat = HAL_I2C_Mem_Read(&vl53l0x_i2c_handler, g_i2cAddr | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT);
}


// Public Methods //////////////////////////////////////////////////////////////

void setAddress_VL53L0X(uint8_t new_addr) {
  writeReg( I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F );
  g_i2cAddr = new_addr;
}

uint8_t getAddress_VL53L0X() {
  return g_i2cAddr;
}

// *** MODIFIED: Accepts I2C_HandleTypeDef pointer ***
bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler){
  // VL53L0X_DataInit() begin

  // *** MODIFIED: Copy the provided handler to the static variable ***
  if (handler == NULL) {
      return false; // Must provide a valid I2C handle
  }
  memcpy(&vl53l0x_i2c_handler, handler, sizeof(I2C_HandleTypeDef));

  // Reset the message buffer.
  memset(msgBuffer, 0, sizeof(msgBuffer));

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  writeReg(0x88, 0x00);

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  g_stopVariable = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);

  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);

  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);

  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);

  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);

  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);

  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);

  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);

  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  g_measTimBudUs = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(g_measTimBudUs);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). Returns true if limit is valid.
bool setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(limit_Mcps * (1 << 7)));
  return true;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(void)
{
  return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds. Returns true if valid.
bool setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    if (used_budget_us > budget_us)
    {
      return false; // Requested timeout too big.
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;
    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    g_measTimBudUs = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
uint32_t getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) { budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead); }
  if (enables.dss) { budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead); }
  else if (enables.msrc) { budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead); }
  if (enables.pre_range) { budget_us += (timeouts.pre_range_us + PreRangeOverhead); }
  if (enables.final_range) { budget_us += (timeouts.final_range_us + FinalRangeOverhead); }

  g_measTimBudUs = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL pulse period. Returns true if valid.
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
    // Validate period_pclks based on type (must be even)
    if ((period_pclks % 2) != 0) { return false; } // Must be even

    if (type == VcselPeriodPreRange) {
        if (period_pclks < 12 || period_pclks > 18) { return false; }
    } else if (type == VcselPeriodFinalRange) {
        if (period_pclks < 8 || period_pclks > 14) { return false; }
    } else {
        return false; // Invalid type
    }

    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    // Apply specific settings and recalculate timeouts
    if (type == VcselPeriodPreRange) {
        // Set phase check limits
        switch (period_pclks) {
            case 12: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false; // Should already be caught, but defensive check
        }
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // Update timeouts
        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
        writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
        // MSRC timeout register is only 8 bits
        writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
    } else { // VcselPeriodFinalRange
        switch (period_pclks) {
            case 8:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x30); writeReg(0xFF, 0x00);
                break;
            case 10:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00);
                break;
            case 12:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00);
                break;
            case 14:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                writeReg(0xFF, 0x01); writeReg(ALGO_PHASECAL_LIM, 0x20); writeReg(0xFF, 0x00);
                break;
            default: return false; // Should already be caught
        }
        writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // Update timeouts
        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
        if (enables.pre_range) {
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;
        }
        writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    }

    // Re-apply timing budget and perform phase calibration
    setMeasurementTimingBudget(g_measTimBudUs);
    uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(0x0);
    writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

    return true;
}


// Get the VCSEL pulse period in PCLKs for the given period type.
uint8_t getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; } // Indicate error
}

// Start continuous ranging measurements.
void startContinuous(uint32_t period_ms)
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, g_stopVariable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode
    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);
    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }
    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
void stopContinuous(void)
{
  writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, 0x00);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
uint16_t readRangeContinuousMillimeters( statInfo_t_VL53L0X *extraStats ) {
  uint8_t tempBuffer[12];
  uint16_t temp;
  startTimeoutInternal(); // Use internal function
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpiredInternal()) // Use internal function
    {
      g_isTimeout = true;
      if (extraStats != 0) memset(extraStats, 0xFF, sizeof(statInfo_t_VL53L0X)); // Indicate error in stats
      return 65535; // Indicate timeout
    }
     // Add a small delay or yield if using RTOS to prevent busy-waiting hogging CPU
     #ifdef CMSIS_OS_RTX // Check if using CMSIS-RTOS
     osDelay(1); // Minimal delay for RTOS context switch
     #endif
  }
  if( extraStats == 0 ){
    // Read only the distance value
    temp = readReg16Bit(RESULT_RANGE_STATUS + 10);
  } else {
    // Read all statistics
    readMulti(0x14, tempBuffer, 12);
    extraStats->rangeStatus =  tempBuffer[0x00] >> 3; // Extract bits 7:3 for status
    extraStats->spadCnt     = ((uint16_t)tempBuffer[0x02] << 8) | tempBuffer[0x03];
    extraStats->signalCnt   = ((uint16_t)tempBuffer[0x06] << 8) | tempBuffer[0x07];
    extraStats->ambientCnt  = ((uint16_t)tempBuffer[0x08] << 8) | tempBuffer[0x09];
    temp                    = ((uint16_t)tempBuffer[0x0A] << 8) | tempBuffer[0x0B];
    extraStats->rawDistance = temp;
  }
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return temp;
}

// Performs a single-shot range measurement
uint16_t readRangeSingleMillimeters( statInfo_t_VL53L0X *extraStats ) {
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, g_stopVariable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(SYSRANGE_START, 0x01);
  // "Wait until start bit has been cleared"
  startTimeoutInternal();
  while (readReg(SYSRANGE_START) & 0x01){
    if (checkTimeoutExpiredInternal()){
      g_isTimeout = true;
      if (extraStats != 0) memset(extraStats, 0xFF, sizeof(statInfo_t_VL53L0X));
      return 65535;
    }
     // Add a small delay or yield if using RTOS
     #ifdef CMSIS_OS_RTX
     osDelay(1);
     #endif
  }
  return readRangeContinuousMillimeters( extraStats );
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()? Clears the timeout flag.
bool timeoutOccurred()
{
  bool tmp = g_isTimeout;
  g_isTimeout = false;
  return tmp;
}

void setTimeout(uint16_t timeout){
  g_ioTimeout = timeout;
}

uint16_t getTimeout(void){
  return g_ioTimeout;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
static bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  startTimeoutInternal();
  while (readReg(0x83) == 0x00)
  {
    if (checkTimeoutExpiredInternal()) { return false; }
     #ifdef CMSIS_OS_RTX
     osDelay(1);
     #endif
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
static void getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
static void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);
  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->pre_range_mclks = decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);
  timeouts->final_range_mclks = decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (enables->pre_range) {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }
  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}


// Decode sequence step timeout in MCLKs from register value
static uint16_t decodeTimeout(uint16_t reg_val)
{
  return (uint16_t)((reg_val & 0x00FF) << (uint8_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;
  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  } else {
    return 0;
  }
}

// Convert sequence step timeout from MCLKs to microseconds
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Perform single reference calibration
static bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
  startTimeoutInternal();
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpiredInternal()) { return false; }
     #ifdef CMSIS_OS_RTX
     osDelay(1);
     #endif
  }
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(SYSRANGE_START, 0x00);
  return true;
}
