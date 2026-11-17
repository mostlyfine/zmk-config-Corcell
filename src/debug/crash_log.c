/*
 * Copyright (c) 2026 The Corcell contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Zephyr's default reaction to a fatal error is to lock interrupts and spin,
 * which on a keyboard looks like "Bluetooth dropped and only the reset button
 * brings it back", with nothing left behind to say why.
 *
 * This records where the firmware died into a spare flash page and reboots,
 * so the keyboard reconnects by itself and the reason can be read back later
 * over the USB serial port. It also catches the other way to go dead: the
 * system work queue (which scans the keys) not running for a long time.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fatal.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/util.h>

#include <cmsis_core.h>
#include <nrf.h>

#include <zmk/usb.h>

/* Last page of the application partition. The image ends far below it. */
#define CODE_PARTITION DT_CHOSEN(zephyr_code_partition)
#define LOG_PAGE_SIZE 4096
#define LOG_PAGE_ADDR (DT_REG_ADDR(CODE_PARTITION) + DT_REG_SIZE(CODE_PARTITION) - LOG_PAGE_SIZE)

#define RECORD_WORDS 16
#define RECORD_COUNT (LOG_PAGE_SIZE / (RECORD_WORDS * sizeof(uint32_t)))
#define RECORD_MAGIC 0xC0C5E000u
#define RECORD_MAGIC_MASK 0xFFFFF000u
#define RECORD_EMPTY 0xFFFFFFFFu

/* Record kinds beyond the K_ERR_* reasons Zephyr passes to the fatal handler. */
#define KIND_WORKQ_STALL 0x80
/* Marks which firmware owns the page; not printed. */
#define KIND_FW_MARK 0x7F

#define STALL_SECONDS 15
#define TRACE_WORDS 8

enum {
    W_HEADER,
    W_PC,
    W_LR,
    W_STACK,  /* start of the thread's stack: identifies the thread */
    W_ICSR,   /* active / pending exception numbers */
    W_UPTIME, /* seconds since boot */
    W_CFSR,
    W_FAULT_ADDR,
    W_TRACE, /* TRACE_WORDS return addresses found on the stack */
};

BUILD_ASSERT(W_TRACE + TRACE_WORDS == RECORD_WORDS);

static const uint32_t *record_at(size_t i) {
    return (const uint32_t *)(LOG_PAGE_ADDR + i * RECORD_WORDS * sizeof(uint32_t));
}

static bool record_is_valid(const uint32_t *rec) {
    return (rec[W_HEADER] & RECORD_MAGIC_MASK) == RECORD_MAGIC;
}

static bool is_fw_mark(const uint32_t *rec) {
    return rec[W_HEADER] == (RECORD_MAGIC | KIND_FW_MARK);
}

/* Runs with interrupts locked and the system about to reboot. */
static void nvmc_write_words(const uint32_t *dst, const uint32_t *src, size_t words) {
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while (!NRF_NVMC->READY) {
    }

    for (size_t i = 0; i < words; i++) {
        *(volatile uint32_t *)&dst[i] = src[i];
        while (!NRF_NVMC->READY) {
        }
    }

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (!NRF_NVMC->READY) {
    }
}

static bool looks_like_return_address(uint32_t value) {
    uintptr_t addr = value & ~1u;

    return (value & 1u) && addr >= (uintptr_t)__text_region_start &&
           addr < (uintptr_t)__text_region_end;
}

static void collect_trace(uint32_t *rec, const struct k_thread *thread, const uint32_t *sp) {
    uintptr_t start = thread->stack_info.start;
    uintptr_t end = start + thread->stack_info.size;
    size_t found = 0;

    if ((uintptr_t)sp < start || (uintptr_t)sp >= end) {
        return;
    }

    for (; (uintptr_t)sp < end && found < TRACE_WORDS; sp++) {
        if (looks_like_return_address(*sp)) {
            rec[W_TRACE + found++] = *sp;
        }
    }
}

static void store_record(uint32_t *rec) {
    for (size_t i = 0; i < RECORD_COUNT; i++) {
        if (record_at(i)[W_HEADER] == RECORD_EMPTY) {
            nvmc_write_words(record_at(i), rec, RECORD_WORDS);
            return;
        }
    }
    /* Page full: keep the oldest records, they are the ones not yet read. */
}

/* Lets other code leave a note in the same log. Kinds 0x50 and up are notes. */
void corcell_crash_log_note(uint8_t kind, const uint32_t *words, size_t count) {
    uint32_t rec[RECORD_WORDS];

    memset(rec, 0xFF, sizeof(rec));
    rec[W_HEADER] = RECORD_MAGIC | kind;
    memcpy(&rec[1], words, MIN(count, RECORD_WORDS - 1) * sizeof(uint32_t));

    unsigned int key = irq_lock();
    store_record(rec);
    irq_unlock(key);
}

static void fill_common(uint32_t *rec, uint8_t kind, const struct k_thread *thread) {
    memset(rec, 0xFF, RECORD_WORDS * sizeof(uint32_t));

    rec[W_HEADER] = RECORD_MAGIC | kind;
    rec[W_STACK] = thread != NULL ? thread->stack_info.start : 0;
    rec[W_ICSR] = SCB->ICSR;
    rec[W_UPTIME] = (uint32_t)(k_uptime_get() / 1000);
    rec[W_CFSR] = SCB->CFSR;
    rec[W_FAULT_ADDR] = (SCB->CFSR & SCB_CFSR_BFARVALID_Msk)    ? SCB->BFAR
                        : (SCB->CFSR & SCB_CFSR_MMARVALID_Msk) ? SCB->MMFAR
                                                                : 0;
}

/*
 * Optional snapshot of the Bluetooth controller's event state, for chasing
 * asserts inside it. The variables are file-static in Zephyr, so their
 * addresses come from the linked image: build once, look them up with nm,
 * pass them as CONFIG_CORCELL_CRASH_LOG_ADDR_* and build again. With the
 * addresses left at 0 nothing extra is stored.
 */
#define KIND_CTLR_STATE 0x41
#define KIND_CTLR_PIPELINE 0x42 /* 0x42..0x44: three pipeline slots each */

#define PIPELINE_SLOTS 8
#define PIPELINE_SLOT_BYTES 32
#define PIPELINE_SLOTS_PER_RECORD 3

static void store_controller_state(void) {
#if CONFIG_CORCELL_CRASH_LOG_ADDR_LLL_EVENT != 0 && CONFIG_CORCELL_CRASH_LOG_ADDR_PREP_FIFO != 0
    static uint32_t rec[RECORD_WORDS];
    const uint32_t *curr = (const uint32_t *)CONFIG_CORCELL_CRASH_LOG_ADDR_LLL_EVENT;
    const uint8_t *fifo = (const uint8_t *)CONFIG_CORCELL_CRASH_LOG_ADDR_PREP_FIFO;
    const uint8_t *fifo_idx = fifo + PIPELINE_SLOTS * PIPELINE_SLOT_BYTES;

    memset(rec, 0xFF, sizeof(rec));
    rec[0] = RECORD_MAGIC | KIND_CTLR_STATE;
    rec[1] = curr[0]; /* event.curr.param */
    rec[2] = curr[1]; /* event.curr.is_abort_cb */
    rec[3] = curr[2]; /* event.curr.abort_cb */
    rec[4] = fifo_idx[0] | (fifo_idx[1] << 8); /* first (read) | last (write) */
#if CONFIG_CORCELL_CRASH_LOG_ADDR_PREEMPT_FLAGS != 0
    memcpy(&rec[5], (const void *)CONFIG_CORCELL_CRASH_LOG_ADDR_PREEMPT_FLAGS, 8);
#endif
    rec[7] = NRF_RADIO->STATE;
    rec[8] = NRF_RADIO->INTENSET;
    rec[9] = (NRF_RADIO->EVENTS_DISABLED ? BIT(0) : 0) | (NRF_RADIO->EVENTS_END ? BIT(1) : 0) |
             (NRF_RADIO->EVENTS_READY ? BIT(2) : 0) | (NRF_RADIO->EVENTS_ADDRESS ? BIT(3) : 0);
    rec[10] = NRF_RADIO->SHORTS;
    rec[11] = NVIC->ISER[0];
    rec[12] = NVIC->ISPR[0];
    rec[13] = NVIC->IABR[0];
    rec[14] = NRF_RTC0->COUNTER;
    rec[15] = NRF_RTC0->INTENSET;
    store_record(rec);

    for (size_t slot = 0; slot < PIPELINE_SLOTS; slot += PIPELINE_SLOTS_PER_RECORD) {
        memset(rec, 0xFF, sizeof(rec));
        rec[0] = RECORD_MAGIC | (KIND_CTLR_PIPELINE + slot / PIPELINE_SLOTS_PER_RECORD);

        for (size_t i = 0; i < PIPELINE_SLOTS_PER_RECORD && slot + i < PIPELINE_SLOTS; i++) {
            const uint8_t *e = fifo + (slot + i) * PIPELINE_SLOT_BYTES;
            uint32_t words[8];

            /* struct lll_event: ticks_at_expire, remainder, lazy:16 force:8, param,
             * prepare_cb, is_abort_cb, abort_cb, is_resume:1 is_aborted:1
             */
            memcpy(words, e, sizeof(words));
            rec[1 + i * 4] = words[0];
            rec[2 + i * 4] = words[3];
            rec[3 + i * 4] = words[4];
            rec[4 + i * 4] = (words[7] & 0x3) | ((words[2] & 0xFFFFFF) << 8);
        }

        store_record(rec);
    }
#endif
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) {
    static uint32_t rec[RECORD_WORDS];
    const struct k_thread *thread = _current;

    (void)irq_lock();

    fill_common(rec, reason & 0x7F, thread);

    if (esf != NULL) {
        rec[W_PC] = esf->basic.pc;
        rec[W_LR] = esf->basic.lr;
    }

    if (thread != NULL) {
        collect_trace(rec, thread, (const uint32_t *)__get_PSP());
    }

    store_record(rec);
    store_controller_state();

    sys_reboot(SYS_REBOOT_COLD);
    CODE_UNREACHABLE;
}

/* Work queue stall detection */

static uint32_t heartbeat;

static void heartbeat_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(heartbeat_work, heartbeat_work_cb);

static void heartbeat_work_cb(struct k_work *work) {
    heartbeat++;
    k_work_schedule(&heartbeat_work, K_SECONDS(1));
}

/* Timer expiry runs in interrupt context, so it still fires when threads do not. */
static void stall_timer_cb(struct k_timer *timer) {
    static uint32_t rec[RECORD_WORDS];
    static uint32_t last_heartbeat;
    static uint8_t stale_seconds;

    if (heartbeat != last_heartbeat) {
        last_heartbeat = heartbeat;
        stale_seconds = 0;
        return;
    }

    if (++stale_seconds < STALL_SECONDS) {
        return;
    }

    (void)irq_lock();

    const struct k_thread *workq = &k_sys_work_q.thread;
    /* A thread that is switched out has its exception frame at its saved PSP. */
    const uint32_t *frame = workq == _current ? (const uint32_t *)__get_PSP()
                                                     : (const uint32_t *)workq->callee_saved.psp;

    fill_common(rec, KIND_WORKQ_STALL, workq);

    uintptr_t start = workq->stack_info.start;
    uintptr_t end = start + workq->stack_info.size;

    if ((uintptr_t)frame >= start && (uintptr_t)(frame + 8) <= end) {
        rec[W_LR] = frame[5];
        rec[W_PC] = frame[6];
        collect_trace(rec, workq, frame + 8);
    }

    store_record(rec);

    sys_reboot(SYS_REBOOT_COLD);
}

static K_TIMER_DEFINE(stall_timer, stall_timer_cb, NULL);

/* Reporting */

static void print_records(void) {
    size_t total = 0;

    printk("CORCELL-FW %s\n", CONFIG_CORCELL_FW_TAG);

    for (size_t i = 0; i < RECORD_COUNT; i++) {
        const uint32_t *rec = record_at(i);

        total += record_is_valid(rec) && !is_fw_mark(rec) ? 1 : 0;
    }

    if (total == 0) {
        printk("CORCELL-CRASH none\n");
        return;
    }

    size_t n = 0;

    for (size_t i = 0; i < RECORD_COUNT; i++) {
        const uint32_t *rec = record_at(i);

        if (!record_is_valid(rec) || is_fw_mark(rec)) {
            continue;
        }

        printk("CORCELL-CRASH %u/%u", (unsigned int)++n, (unsigned int)total);
        for (size_t w = 0; w < RECORD_WORDS; w++) {
            printk(" %08x", (unsigned int)rec[w]);
        }
        printk("\n");
        /* The USB serial buffer is small; let it drain between lines. */
        k_msleep(20);
    }
}

static void report_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(report_work, report_work_cb);

static void report_work_cb(struct k_work *work) {
    if (zmk_usb_is_powered()) {
        print_records();
    }

    k_work_schedule(&report_work, K_SECONDS(5));
}

static uint32_t fw_tag_hash(void) {
    uint32_t hash = 2166136261u; /* FNV-1a */

    for (const char *c = CONFIG_CORCELL_FW_TAG; *c != '\0'; c++) {
        hash = (hash ^ (uint8_t)*c) * 16777619u;
    }

    return hash;
}

static bool has_fw_mark(uint32_t hash) {
    for (size_t i = 0; i < RECORD_COUNT; i++) {
        const uint32_t *rec = record_at(i);

        if (rec[W_HEADER] == (RECORD_MAGIC | KIND_FW_MARK) && rec[1] == hash) {
            return true;
        }
    }

    return false;
}

/*
 * A page that holds anything other than our records or erased flash is
 * stale, and so is one written by a firmware with a different tag.
 */
static bool page_needs_erase(void) {
    if (!has_fw_mark(fw_tag_hash())) {
        return true;
    }

    for (size_t i = 0; i < RECORD_COUNT; i++) {
        const uint32_t *rec = record_at(i);

        if (record_is_valid(rec)) {
            continue;
        }

        for (size_t w = 0; w < RECORD_WORDS; w++) {
            if (rec[w] != RECORD_EMPTY) {
                return true;
            }
        }
    }

    return false;
}

static void prepare_work_cb(struct k_work *work) {
    const struct device *flash = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));

    if (page_needs_erase() && device_is_ready(flash)) {
        uint32_t hash = fw_tag_hash();

        flash_erase(flash, LOG_PAGE_ADDR, LOG_PAGE_SIZE);
        corcell_crash_log_note(KIND_FW_MARK, &hash, 1);
    }

    k_work_schedule(&report_work, K_NO_WAIT);
}

static K_WORK_DELAYABLE_DEFINE(prepare_work, prepare_work_cb);

static int crash_log_init(void) {
    k_work_schedule(&heartbeat_work, K_SECONDS(1));
    k_timer_start(&stall_timer, K_SECONDS(1), K_SECONDS(1));
    /* Leave boot and reconnection alone before touching flash. */
    k_work_schedule(&prepare_work, K_SECONDS(10));

    return 0;
}

SYS_INIT(crash_log_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
