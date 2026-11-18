/*
 * Copyright (c) 2026 The Corcell contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Backport of two Zephyr Bluetooth controller fixes for the nRF radio,
 * applied at link time (-Wl,--wrap) so the Zephyr fork in use stays as is.
 * Both landed in Zephyr 4.3; the Zephyr this firmware builds against is 4.1.
 *
 * 355648a69f "Bluetooth: Controller: Fix missing radio tmr status reset"
 *   radio_disable() did not disable the PPI channels that start the radio,
 *   so under a race the radio could still enter Tx or Rx after being
 *   disabled, putting the controller out of sync with the radio. Upstream's
 *   symptoms: the central not transmitting in the next interval, and events
 *   piling up in the prepare pipeline. That matches the hardware records from
 *   this keyboard: a split-link central event left current with the radio
 *   idle, the pipeline full behind it, and LL_ASSERT(next) in
 *   lll_prepare_resolve().
 *
 * fe7675e209 "Bluetooth: Controller: Fix assertion due to cleared pending
 *   radio IRQ"
 *   radio_isr_set() cleared a pending radio interrupt in the NVIC, which
 *   under a race loses the interrupt that would have ended the event.
 *   Upstream removed the clear (and enables the DISABLED interrupt once at
 *   init). The callback variables are private to radio.c, so here the real
 *   function still runs and the interrupt it may have cleared is put back.
 *
 * Remove this file once the Zephyr in use contains both commits.
 */

#include <stdbool.h>

#include <zephyr/kernel.h>

#include <cmsis_core.h>
#include <nrf.h>

typedef void (*radio_isr_cb_t)(void *param);

void radio_tmr_status_reset(void);

void __real_radio_disable(void);
void __real_radio_isr_set(radio_isr_cb_t cb, void *param);

void __wrap_radio_disable(void) {
    /* Reset/disable PPI before disabling, as upstream radio_disable() now does. */
    radio_tmr_status_reset();
    __real_radio_disable();
}

void __wrap_radio_isr_set(radio_isr_cb_t cb, void *param) {
    bool was_pending = NVIC_GetPendingIRQ(RADIO_IRQn) != 0;

    __real_radio_isr_set(cb, param);

    /*
     * Upstream leaves the NVIC alone, so a radio interrupt pending before the
     * call, or raised by a DISABLED event during it, is still delivered.
     */
    if (was_pending || NRF_RADIO->EVENTS_DISABLED != 0) {
        NVIC_SetPendingIRQ(RADIO_IRQn);
    }
}
