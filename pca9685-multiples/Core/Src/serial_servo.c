#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>

/*──────────────────────────────────────────────────────────────────
 * PCA9685 registers (7-bit address 0x40 → 8-bit 0x80)
 *─────────────────────────────────────────────────────────────────*/
#define PCA9685_I2C_ADDR_7BIT   0x40
#define PCA9685_ADDR            (PCA9685_I2C_ADDR_7BIT << 1)   /* 0x80 */
#define PCA9685_REG_MODE1       0x00
#define PCA9685_REG_PRESCALE    0xFE
#define PCA9685_REG_LED0_ON_L   0x06
#define PCA9685_BIT_MODE1_SLEEP  4
#define PCA9685_BIT_MODE1_AI     5
#define PCA9685_BIT_MODE1_RESTART 7

#define NUM_JOINTS    4
static const uint8_t SERVO_CH[NUM_JOINTS] = {0, 1, 2, 3};

/* Servo pulse limits in MICROSECONDS (adjust to your servo) */
#define SERVO_MIN_US  500   /* 0°   */
#define SERVO_MAX_US  2500  /* 180° */
#define SERVO_FREQ_HZ 50    /* 20 ms period */

static I2C_HandleTypeDef  *s_i2c;
static UART_HandleTypeDef *s_uart;

/*──────────────────────────────────────────────────────────────────
 * Convert µs pulse → PCA9685 12-bit count (0-4095)
 * At 50 Hz: period = 20 ms = 20000 µs, 4096 steps → 1 step ≈ 4.88 µs
 *─────────────────────────────────────────────────────────────────*/
static uint16_t us_to_count(uint16_t us) {
  return (uint16_t)((float)us * 4096.0f / (1000000.0f / SERVO_FREQ_HZ));
}

/*──────────────────────────────────────────────────────────────────
 * Single-byte register write with error checking
 *─────────────────────────────────────────────────────────────────*/
static HAL_StatusTypeDef pca_write8(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(s_i2c, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1, 100);
}

/*──────────────────────────────────────────────────────────────────
 * Multi-byte register write with error checking
 *─────────────────────────────────────────────────────────────────*/
static HAL_StatusTypeDef pca_write16(uint8_t reg, const uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Write(s_i2c, PCA9685_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, 100);
}

static void uart_puts(const char *s) {
  HAL_UART_Transmit(s_uart, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

/*──────────────────────────────────────────────────────────────────
 * Initialise PCA9685 for 50 Hz servo operation.
 * Returns HAL_OK if the device responded on the bus.
 *─────────────────────────────────────────────────────────────────*/
static HAL_StatusTypeDef pca_init(void) {
  uint8_t prescale;
  uint8_t mode1;

  /* 1. Probe: is the PCA9685 actually on the bus? */
  if (HAL_I2C_IsDeviceReady(s_i2c, PCA9685_ADDR, 3, 100) != HAL_OK) {
    uart_puts("PCA9685: NOT DETECTED at 0x40 - check wiring/power/pull-ups\r\n");
    return HAL_ERROR;
  }
  uart_puts("PCA9685: detected at 0x40\r\n");

  /* 2. Compute prescale for 50 Hz:
       prescale = round(25 MHz / (4096 * 50)) - 1 = 121 = 0x79   */
  prescale = (uint8_t)(25000000.0f / (4096.0f * (float)SERVO_FREQ_HZ) - 1.0f);

  /* 3. Enter sleep (required to write PRESCALE) */
  mode1 = (1 << PCA9685_BIT_MODE1_SLEEP);   /* 0x10 */
  if (pca_write8(PCA9685_REG_MODE1, mode1) != HAL_OK) {
    uart_puts("PCA9685: write MODE1(sleep) FAILED\r\n");
    return HAL_ERROR;
  }

  /* 4. Write prescale */
  if (pca_write8(PCA9685_REG_PRESCALE, prescale) != HAL_OK) {
    uart_puts("PCA9685: write PRESCALE FAILED\r\n");
    return HAL_ERROR;
  }

  /* 5. Wake: clear SLEEP, enable auto-increment */
  mode1 = (1 << PCA9685_BIT_MODE1_AI);      /* 0x20 */
  if (pca_write8(PCA9685_REG_MODE1, mode1) != HAL_OK) {
    uart_puts("PCA9685: write MODE1(wake) FAILED\r\n");
    return HAL_ERROR;
  }

  /* Oscillator needs ≥500 µs to stabilise */
  HAL_Delay(2);

  /* 6. Set restart + auto-increment */
  mode1 = (1 << PCA9685_BIT_MODE1_RESTART) | (1 << PCA9685_BIT_MODE1_AI);  /* 0xA0 */
  if (pca_write8(PCA9685_REG_MODE1, mode1) != HAL_OK) {
    uart_puts("PCA9685: write MODE1(restart) FAILED\r\n");
    return HAL_ERROR;
  }

  uart_puts("PCA9685: initialised at 50 Hz\r\n");
  return HAL_OK;
}

/*──────────────────────────────────────────────────────────────────
 * Set one servo channel to an angle (0–180°).
 *─────────────────────────────────────────────────────────────────*/
static HAL_StatusTypeDef pca_set_angle(uint8_t ch, float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;

  /* Map angle → µs pulse → PCA9685 count */
  uint16_t us = (uint16_t)(SERVO_MIN_US + (deg / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US));
  uint16_t off = us_to_count(us);

  uint8_t reg = (uint8_t)(PCA9685_REG_LED0_ON_L + 4 * ch);
  uint8_t buf[4] = {
    0x00,                                                  /* ON_L  = 0 */
    0x00,                                                  /* ON_H  = 0 */
    (uint8_t)(off & 0xFF),                                 /* OFF_L */
    (uint8_t)((off >> 8) & 0x0F)                           /* OFF_H (top 4 bits) */
  };

  return pca_write16(reg, buf, 4);
}

/*──────────────────────────────────────────────────────────────────
 * Startup self-test: centre all servos, then sweep 0→180→0 once.
 * This confirms the PCA9685 + servos are alive before the serial
 * command loop takes over.
 *─────────────────────────────────────────────────────────────────*/
static void pca_selftest(void) {
  uart_puts("PCA9685: self-test – centering all servos at 90°\r\n");
  for (int i = 0; i < NUM_JOINTS; i++) {
    pca_set_angle(SERVO_CH[i], 90.0f);
  }
  HAL_Delay(600);

  uart_puts("PCA9685: self-test – sweeping 0° → 180° → 0°\r\n");
  for (int deg = 0; deg <= 180; deg += 10) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      pca_set_angle(SERVO_CH[i], (float)deg);
    }
    HAL_Delay(20);
  }
  for (int deg = 180; deg >= 0; deg -= 10) {
    for (int i = 0; i < NUM_JOINTS; i++) {
      pca_set_angle(SERVO_CH[i], (float)deg);
    }
    HAL_Delay(20);
  }
  /* Return to center */
  for (int i = 0; i < NUM_JOINTS; i++) {
    pca_set_angle(SERVO_CH[i], 90.0f);
  }
  uart_puts("PCA9685: self-test done – ready for serial commands\r\n");
}

/*──────────────────────────────────────────────────────────────────
 * Main entry: initialise PCA9685, run self-test, then parse serial
 * commands of the form "J<n> <rad>\n" forever.
 *──────────────────────────────────────────────────────────────────*/
void serial_servo_run(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
  s_i2c = hi2c;
  s_uart = huart;

  uart_puts("\r\n=== STM32F429ZI + PCA9685 servo controller ===\r\n");
  uart_puts("I2C1: PB8=SCL, PB9=SDA | USART3: 115200 8N1\r\n");
  uart_puts("Protocol: J<n> <rad>  (n=0..3, rad relative to 90° center)\r\n");

  if (pca_init() != HAL_OK) {
    uart_puts("PCA9685: init FAILED – servos will not move!\r\n");
    uart_puts("Check: V+ = 5V to servos, OE = GND, common GND, I2C pull-ups\r\n");
  } else {
    pca_selftest();
  }

  char line[64];
  uint8_t idx = 0;
  while (1) {
    uint8_t c;
    HAL_StatusTypeDef rx = HAL_UART_Receive(s_uart, &c, 1, 10);
    if (rx != HAL_OK) {
      /* Clear any latched framing/overrun/noise/parity error flags so a
         single glitch on the line can't permanently stall or flood the
         receiver with spurious bytes on every subsequent poll. */
      __HAL_UART_CLEAR_OREFLAG(s_uart);
      __HAL_UART_CLEAR_FEFLAG(s_uart);
      __HAL_UART_CLEAR_NEFLAG(s_uart);
      __HAL_UART_CLEAR_PEFLAG(s_uart);
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        line[idx] = '\0';
        int j;
        float v;
        /* The USB virtual COM port on this board duplicates the first byte
           of a message whenever it follows an idle gap (normal here, since
           the bridge only sends short bursts every ~100ms). Collapse any
           run of leading 'J' characters to a single one before parsing so
           a glitched "JJ0 0.000" still parses the same as "J0 0.000". */
        char *p = line;
        while (p[0] == 'J' && p[1] == 'J') p++;
        if (sscanf(p, "J%d %f", &j, &v) == 2 && j >= 0 && j < NUM_JOINTS) {
          /* Map radians → degrees (0 rad = 90°, ±1.57 rad = 0°/180°) */
          float deg = 90.0f + (v * 57.29578f);
          if (deg < 0.0f) deg = 0.0f;
          if (deg > 180.0f) deg = 180.0f;
          if (pca_set_angle(SERVO_CH[j], deg) != HAL_OK) {
            uart_puts("PCA9685: I2C write failed\r\n");
          } else {
            char resp[48];
            snprintf(resp, sizeof(resp), "J%d -> %.1f deg\r\n", j, deg);
            uart_puts(resp);
          }
        } else {
          /* Diagnostic: show exactly what bytes were received (hex) so a
             wiring/noise problem is visible instead of just "ERR". */
          char hexline[3 * sizeof(line)];
          size_t hpos = 0;
          for (uint8_t k = 0; k < idx && hpos < sizeof(hexline) - 4; k++) {
            hpos += (size_t)snprintf(hexline + hpos, sizeof(hexline) - hpos, "%02X ", (uint8_t)line[k]);
          }
          char resp[64 + sizeof(hexline)];
          snprintf(resp, sizeof(resp), "ERR raw[%u]: %s(\"%s\")\r\n", (unsigned)idx, hexline, line);
          uart_puts(resp);
        }
        idx = 0;
      }
    } else if (c >= 0x20 && c < 0x7F) {
      /* Only accept printable ASCII into the line buffer - electrical
         noise/garbage bytes on the line are silently dropped instead of
         being treated as command characters that later fail to parse
         and flood the port with ERR replies. */
      if (idx < (uint8_t)(sizeof(line) - 1)) {
        line[idx++] = (char)c;
      } else {
        /* Line too long (garbage) - reset without printing an error. */
        idx = 0;
      }
    }
  }
}
