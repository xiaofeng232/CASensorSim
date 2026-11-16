/*
 * CASensorSim -- CA (crank angle) sensor signal simulator firmware.
 *
 * Bring-up skeleton for the NUCLEO-F429ZI test-bench board: LED heartbeat
 * to prove the target is alive. The actual product generates 8 digital
 * pickoff channels (TDC1/REF1/A/B/REF2/C/D/TDC2) + 2 SSI absolute-angle
 * streams in real time from a virtual flywheel; the bit-exact core port
 * (cas_core.c / cas_fault.c per HANDOFF.md #7) is the active task and
 * will be driven from a timer here.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define BLINK_INTERVAL_MS 500

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		printk("CASensorSim: LED device not ready\n");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) {
		printk("CASensorSim: LED configure failed\n");
		return -EIO;
	}

	printk("CASensorSim firmware skeleton (%s)\n", CONFIG_BOARD_TARGET);
	printk("cas_core port pending -- see HANDOFF.md #7\n");

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(BLINK_INTERVAL_MS);
	}
	return 0;
}
