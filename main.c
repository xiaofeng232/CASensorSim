/*
 * CASensorSim -- CA (crank angle) sensor signal simulator firmware.
 *
 * App layer for the FK407M2-ZGT6 (STM32F407ZGT6) bench board: starts
 * the bit-exact virtual CA engine (cas/ library) with the X1+ golden
 * preset and drives it from a 10 us TIM8 update tick. Each tick runs
 * cas_step() + cas_fault_apply() and pushes the 8 digital levels to
 * the output pins CH00..CH07 = PE0..PE7 (KiCad pin partition):
 * driven output while oe=1, high-Z input while oe=0 (FR-30/HR-02).
 *
 * The LED (PC13, active low) keeps a heartbeat in the main thread; the
 * whole signal path runs from the timer ISR.
 *
 * SSI: both absolute-angle words are computed every tick, but their
 * pin layer (PA8/PC0 + RS-485 DIR, TIM6 protocol timing) is a later
 * step -- the words are only visible to a debugger for now.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>

#include "cas_core.h"
#include "cas_fault.h"
#include "cas_preset.h"

#define BLINK_INTERVAL_MS 500u

/* Signal tick: 10 us update events (pin-partition plan, TIM8). */
#define CAS_TICK_HZ 100000u

/* CH00..CH07 sit on PE0..PE7. */
#define CAS_DIG_MASK 0xFFu

static const struct gpio_dt_spec led =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct device *const dig_port =
	DEVICE_DT_GET(DT_NODELABEL(gpioe));
static const struct device *const tick_cnt =
	DEVICE_DT_GET(DT_CHILD(DT_NODELABEL(timers8), counter));

static cas_state_t cas_st;
static cas_fault_t cas_faults;

/* oe[] bits currently driven as outputs (mode change only on toggle). */
static gpio_port_pins_t oe_driven;

static void tick_isr(const struct device *dev, void *user_data)
{
	gpio_port_pins_t oe = 0;
	uint32_t level = 0;

	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/* One engine sample + fault application per tick. */
	cas_step(&cas_st);
	cas_fault_apply(&cas_faults, &cas_st);

	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		if (cas_st.oe[ch] != 0u) {
			oe |= BIT(ch);
		}
		if (cas_st.dig[ch] != 0u) {
			level |= BIT(ch);
		}
	}

	if (oe != oe_driven) {
		/* oe changed: driven output vs high-Z input (FR-30). */
		gpio_port_pins_t changed = oe ^ oe_driven;

		for (int ch = 0; ch < CAS_N_DIG; ch++) {
			if ((changed & BIT(ch)) == 0u) {
				continue;
			}
			(void)gpio_pin_configure(dig_port, ch,
						 (oe & BIT(ch)) != 0u
							 ? GPIO_OUTPUT
							 : GPIO_INPUT);
		}
		oe_driven = oe;
	}

	/* One BSRR write for all 8 levels; input-mode pins ignore it. */
	gpio_port_set_masked_raw(dig_port, CAS_DIG_MASK, level);
}

/* Wire the signal path: PE0..7 as high-Z inputs, then the 10 us tick. */
static int signal_path_start(void)
{
	if (!device_is_ready(dig_port)) {
		printk("GPIOE not ready\n");
		return -ENODEV;
	}
	if (!device_is_ready(tick_cnt)) {
		printk("TIM8 counter not ready\n");
		return -ENODEV;
	}

	oe_driven = 0;
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		int rc = gpio_pin_configure(dig_port, ch, GPIO_INPUT);

		if (rc != 0) {
			printk("PE%d configure failed: %d\n", ch, rc);
			return rc;
		}
	}

	uint32_t freq = counter_get_frequency(tick_cnt);
	uint32_t ticks = freq / CAS_TICK_HZ;

	if (freq < CAS_TICK_HZ || (freq % CAS_TICK_HZ) != 0u ||
	    ticks == 0u || ticks > 0x10000u) {
		printk("TIM8 clock %u Hz cannot tick at %u Hz\n", freq,
		       CAS_TICK_HZ);
		return -EINVAL;
	}

	struct counter_top_cfg top = {
		.ticks = ticks - 1u, /* ARR: period = ticks + 1 clocks */
		.callback = tick_isr,
	};

	int rc = counter_set_top_value(tick_cnt, &top);

	if (rc != 0) {
		printk("TIM8 top value failed: %d\n", rc);
		return rc;
	}
	rc = counter_start(tick_cnt);
	if (rc != 0) {
		printk("TIM8 start failed: %d\n", rc);
	}
	return rc;
}

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("LED device not ready\n");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
		printk("LED configure failed\n");
		return -EIO;
	}

	cas_cfg_t cfg;
	/* Power-up state: engine stopped (const 0 rpm). The speed comes
	 * from the runtime parameter interface once it exists. */
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 0.0};
	cas_ssi_cfg_t ssi = {0};

	cas_preset_x1plus(&cfg);
	cas_fault_reset(&cas_faults); /* FR-34: all faults off at power-up */
	cas_init(&cas_st, &cfg, &prof, &ssi, 1.0 / (double)CAS_TICK_HZ);

	printk("CASensorSim: X1+ preset (186 teeth), const 0 rpm, "
	       "tick %u Hz, CH00..07 = PE0..7\n", CAS_TICK_HZ);

	int rc = signal_path_start();

	if (rc != 0) {
		printk("signal path start failed: %d\n", rc);
		return rc;
	}

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(BLINK_INTERVAL_MS);
	}
	return 0;
}
