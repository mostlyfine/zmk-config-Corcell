target_sources_ifdef(CONFIG_ZMK_INPUT_PROCESSOR_VERTICAL_SCROLL app PRIVATE
  src/pointing/input_processor_vertical_scroll.c
)

target_sources_ifdef(CONFIG_CORCELL_BATTERY_VOLTAGE_DIVIDER app PRIVATE
  src/battery/corcell_battery_voltage_divider.c
)

target_sources_ifdef(CONFIG_CORCELL_POWER_ON_LED app PRIVATE
  src/status/power_on_led.c
)

target_sources_ifdef(CONFIG_CORCELL_CRASH_LOG app PRIVATE
  src/debug/crash_log.c
)

target_sources_ifdef(CONFIG_CORCELL_PAW3222_BOOT_CHECK app PRIVATE
  src/pointing/paw3222_boot_check.c
)

if(CONFIG_CORCELL_BT_RADIO_FIXES_BACKPORT)
  target_sources(app PRIVATE src/bluetooth/radio_fixes_backport.c)
  zephyr_ld_options(-Wl,--wrap=radio_disable -Wl,--wrap=radio_isr_set)
endif()
