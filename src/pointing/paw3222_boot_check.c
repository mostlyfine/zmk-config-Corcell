/*
 * Copyright (c) 2026 The Corcell contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * The PAW3222 driver probes the sensor once, at boot. If that probe fails the
 * device stays "not ready" and the trackball is dead until the next boot,
 * while the keys keep working. This has been seen after waking from deep
 * sleep, and a plain reset brought the trackball back.
 *
 * So: shortly after boot, if the sensor did not come up, note what it answers
 * now and reboot, a limited number of times.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include <nrf.h>

#define PAW_NODE DT_INST(0, pixart_paw3222)

#if DT_NODE_HAS_STATUS(PAW_NODE, okay)

#define PAW_SPI_MODE                                                                               \
    (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define REG_PRODUCT_ID1 0x00
#define REG_PRODUCT_ID2 0x01
#define REG_OPERATION_MODE 0x05
#define REG_CONFIGURATION 0x06
#define REG_WRITE_PROTECT 0x09

#define SPI_WRITE BIT(7)
#define WRITE_PROTECT_DISABLE 0x5a
#define CONFIGURATION_RESET BIT(7)

/* GPREGRET2 survives a soft reset and is cleared by power-on. */
#define RETRY_MAGIC 0xA0
#define RETRY_MAGIC_MASK 0xF0
#define RETRY_MAX 2

#define NOTE_KIND_PAW_INIT_FAILED 0x50

static const struct device *const paw = DEVICE_DT_GET(PAW_NODE);
static const struct spi_dt_spec paw_spi = SPI_DT_SPEC_GET(PAW_NODE, PAW_SPI_MODE, 0);

#if IS_ENABLED(CONFIG_CORCELL_CRASH_LOG)
void corcell_crash_log_note(uint8_t kind, const uint32_t *words, size_t count);
#else
static inline void corcell_crash_log_note(uint8_t kind, const uint32_t *words, size_t count) {}
#endif

/* Register value in the low byte, or the negative errno in the upper bits. */
static uint32_t read_reg(uint8_t addr) {
    uint8_t value = 0;
    const struct spi_buf tx_buf = {.buf = &addr, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf rx_buf[] = {{.buf = NULL, .len = 1}, {.buf = &value, .len = 1}};
    const struct spi_buf_set rx = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};

    int ret = spi_transceive_dt(&paw_spi, &tx, &rx);

    return ret < 0 ? (uint32_t)ret << 8 : value;
}

static void write_reg(uint8_t addr, uint8_t value) {
    uint8_t buf[] = {addr | SPI_WRITE, value};
    const struct spi_buf tx_buf = {.buf = buf, .len = sizeof(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    spi_write_dt(&paw_spi, &tx);
}

static uint8_t retry_count(void) {
    uint32_t reg = NRF_POWER->GPREGRET2;

    return (reg & RETRY_MAGIC_MASK) == RETRY_MAGIC ? reg & ~RETRY_MAGIC_MASK : 0;
}

static void check_work_cb(struct k_work *work) {
    uint8_t attempt = retry_count();

    if (device_is_ready(paw)) {
        NRF_POWER->GPREGRET2 = 0;
        return;
    }

    uint32_t note[12] = {0};

    note[0] = NRF_POWER->RESETREAS;
    note[1] = attempt;
    note[2] = k_uptime_get_32();

    if (spi_is_ready_dt(&paw_spi)) {
        /* What the sensor answers now, a moment after the driver gave up. */
        note[3] = read_reg(REG_PRODUCT_ID1);
        note[4] = read_reg(REG_PRODUCT_ID2);
        note[5] = read_reg(REG_OPERATION_MODE);
        note[6] = read_reg(REG_CONFIGURATION);

        /* And after asking it to reset itself, without relying on reads. */
        write_reg(REG_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
        write_reg(REG_CONFIGURATION, CONFIGURATION_RESET);
        k_msleep(10);

        note[7] = read_reg(REG_PRODUCT_ID1);
        note[8] = read_reg(REG_PRODUCT_ID2);
        note[9] = read_reg(REG_OPERATION_MODE);
        note[10] = read_reg(REG_CONFIGURATION);
    } else {
        note[3] = UINT32_MAX;
    }

    corcell_crash_log_note(NOTE_KIND_PAW_INIT_FAILED, note, ARRAY_SIZE(note));

    if (attempt >= RETRY_MAX) {
        /* Give up for this boot. The next wake or reset starts counting afresh. */
        NRF_POWER->GPREGRET2 = 0;
        return;
    }

    NRF_POWER->GPREGRET2 = RETRY_MAGIC | (attempt + 1);
    sys_reboot(SYS_REBOOT_COLD);
}

static K_WORK_DELAYABLE_DEFINE(check_work, check_work_cb);

static int paw3222_boot_check_init(void) {
    k_work_schedule(&check_work, K_MSEC(500));
    return 0;
}

SYS_INIT(paw3222_boot_check_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_NODE_HAS_STATUS(PAW_NODE, okay) */
