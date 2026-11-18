/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * CASensorSim -- CA (crank angle) sensor signal simulator firmware.
 *
 * App layer for the FK407M2-ZGT6 (STM32F407ZGT6) bench board. Two
 * concurrent activities:
 *
 *   1. Signal path (100 kHz TIM8 update ISR, 10 us tick per the
 *      pin-partition plan): cas_step() -> cas_fault_apply() -> GPIO
 *      output stage CH00..CH07 = PE0..PE7 (dig[] levels with a single
 *      BSRR write; oe=0 switches the pin to input = high-Z, FR-30) and
 *      the telemetry mirror refresh (below).
 *   2. Fibre protocol over the native USB bulk interface (`fibre-usb`
 *      alias; on boards without a usable device controller, over the UART
 *      selected by `fibre-uart`): exposes the CasIntf interface
 *      (interface/cas_interface.yaml) -- readonly telemetry attributes +
 *      set_rpm / set_dig_fault / set_ssi_fault / clear_faults. The Fibre
 *      stack runs in its own event-loop thread; protocol calls never run
 *      in ISR context. The transport backend comes from the platform
 *      fibre-transport module; the build picks USB or UART depending on
 *      which alias the board overlay defines.
 *
 * Concurrency model:
 *   - The ISR writes the CasObject telemetry members every tick (plain
 *     32-bit aligned stores; sample_count is 64 bit and may tear when
 *     read -- acceptable for a monotonic counter). The Fibre property
 *     layer reads the members through a pointer at request time, so
 *     host reads always see the latest tick's values.
 *   - Parameter writes (set_rpm / set_*_fault / clear_faults) run on the
 *     event-loop thread and are guarded with irq_lock() so the ISR can
 *     never observe a half-updated profile or fault set.
 *
 * SSI: both absolute-angle words are computed every tick; their pin
 * layer (PA8/PC0 + RS-485 DIR, TIM6 protocol timing) is a later step.
 *
 * Console and shell are disabled (the Fibre transport owns the protocol
 * interface); build with `-DEXTRA_CONF_FILE=rtt.conf` for logs over SEGGER
 * RTT (SWD). The host-side counterpart is host/cas_fibre_test.py.
 */

#include <cmath>

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>

#include <fibre/fibre.hpp>
#include <fibre/zephyr_event_loop.hpp>

#include "app_version.h"
#include "banner.h"

#if DT_NODE_EXISTS(DT_ALIAS(fibre_usb))
#include <fibre_transport/usb_backend.hpp>
#else
#include <fibre_transport/uart_backend.hpp>
#endif
#include "interfaces.hpp"

#include "cas_core.h"
#include "cas_fault.h"
#include "cas_preset.h"

LOG_MODULE_REGISTER(cas_sim);

namespace {

/* --------------------------------------------------------------------------
 * Signal path constants
 */

/* Signal tick: 10 us update events (pin-partition plan, TIM8). */
constexpr uint32_t CAS_TICK_HZ = 100000u;

/* CH00..CH07 sit on PE0..PE7. */
constexpr uint32_t CAS_DIG_MASK = 0xFFu;

/* Firmware version reported over Fibre, encoded (major<<16)|(minor<<8)|patch.
 * Derived at build time from this repo's git state (the generated
 * app_version.h; see CMakeLists.txt) instead of a hardcoded constant: it
 * follows the vX.Y.Z tag / version.txt target of the CASensorSim repo. */
constexpr uint32_t CASIM_FW_VERSION =
    (static_cast<uint32_t>(APP_VERSION_MAJOR) << 16) |
    (static_cast<uint32_t>(APP_VERSION_MINOR) << 8) |
    static_cast<uint32_t>(APP_VERSION_PATCH);

/* --------------------------------------------------------------------------
 * Engine + output stage state (ISR context)
 */

cas_state_t cas_st;
cas_fault_t cas_faults;

/* oe[] bits currently driven as outputs (mode change only on toggle). */
gpio_port_pins_t oe_driven;

const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
const struct device *const dig_port = DEVICE_DT_GET(DT_NODELABEL(gpioe));
const struct device *const tick_cnt =
	DEVICE_DT_GET(DT_CHILD(DT_NODELABEL(timers8), counter));

/* --------------------------------------------------------------------------
 * Published Fibre object
 *
 * Member naming is dictated by the generated CasIntfWrapper: every YAML
 * attribute maps to a member `<name>_` (read live through a pointer when
 * the host reads the property), every function to a method.
 */
class CasObject {
public:
    /* -- telemetry (written by the signal ISR every tick) -- */
    uint32_t firmware_version_ = CASIM_FW_VERSION;
    uint32_t tick_hz_ = CAS_TICK_HZ;
    float rpm_ = 0.0f;
    float theta_deg_ = 0.0f;
    uint8_t dig_mask_ = 0;
    uint8_t oe_mask_ = 0;
    int32_t ssi1_ = 0;
    int32_t ssi2_ = 0;
    uint64_t sample_count_ = 0;

    /* -- parameters (event-loop thread, irq_lock() against the ISR) --
     *
     * The legacy Fibre protocol only carries fixed-size int/uint function
     * payloads: enable/pfb/hold/fail flags are uint8 (0/1), the return
     * value is uint8 (1 = accepted, 0 = rejected) and set_rpm takes the
     * speed in milli-rpm (60000 = 60.000 rpm, negative = reverse). */

    uint8_t set_rpm(int32_t rpm_mrpm) {
        double rpm = (double)rpm_mrpm / 1000.0;

        if (!std::isfinite(rpm)) {
            return 0;
        }
        unsigned key = irq_lock();

        cas_st.prof.type = CAS_PROF_CONST;
        cas_st.prof.rpm = rpm;
        irq_unlock(key);
        return 1;
    }

    uint8_t set_dig_fault(uint8_t channel, uint8_t enable, uint8_t trigger,
                          uint32_t angle_mdeg, uint32_t every_n) {
        if (channel >= CAS_N_DIG || trigger > CAS_FT_EVERY_N_SSI) {
            return 0;
        }
        unsigned key = irq_lock();
        cas_fault_cond_t *cond = &cas_faults.dig[channel].cond;

        cond->enable = (enable != 0u);
        cond->trigger = (cas_fault_trigger_t)trigger;
        cond->angle_mdeg = angle_mdeg;
        cond->every_n = every_n;
        cond->active = false; /* re-arm the latch */
        irq_unlock(key);
        return 1;
    }

    uint8_t set_ssi_fault(uint8_t channel, uint8_t enable, uint8_t trigger,
                          uint32_t angle_mdeg, uint32_t every_n, uint8_t pfb,
                          int32_t offset_mdeg, uint8_t hold,
                          int32_t single_frame_delta, uint8_t fail) {
        if (channel >= CAS_N_SSI || trigger > CAS_FT_EVERY_N_SSI) {
            return 0;
        }
        unsigned key = irq_lock();
        cas_ssi_fault_t *f = &cas_faults.ssi[channel];
        cas_fault_cond_t *cond = &f->cond;

        cond->enable = (enable != 0u);
        cond->trigger = (cas_fault_trigger_t)trigger;
        cond->angle_mdeg = angle_mdeg;
        cond->every_n = every_n;
        cond->active = false; /* re-arm the latch */
        f->pfb = (pfb != 0u);
        f->offset_mdeg = offset_mdeg;
        f->hold = (hold != 0u);
        f->single_frame_delta = single_frame_delta;
        f->fail = (fail != 0u);
        irq_unlock(key);
        return 1;
    }

    void clear_faults() {
        unsigned key = irq_lock();

        cas_fault_reset(&cas_faults); /* FR-34 all-off state */
        irq_unlock(key);
    }
};

CasObject cas_object_;
CasIntfWrapper<CasObject> cas_object_wrapper_{cas_object_};

/* Stable node ID so the host recognizes the device across reboots. */
const uint8_t kNodeId[16] = {
    'C', 'A', 'S', 'S', 'I', 'M', 'F', 'W',
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
};

/* Consistency between the devicetree alias and the enabled fibre-transport
 * backend: exactly one of the two transports must be wired up. */
#if DT_NODE_EXISTS(DT_ALIAS(fibre_usb)) && !defined(CONFIG_FIBRE_TRANSPORT_USB)
#error "fibre-usb devicetree alias present but CONFIG_FIBRE_TRANSPORT_USB is not enabled"
#endif
#if !DT_NODE_EXISTS(DT_ALIAS(fibre_usb)) && !defined(CONFIG_FIBRE_TRANSPORT_UART)
#error "no fibre-usb alias; enable CONFIG_FIBRE_TRANSPORT_UART and define a fibre-uart alias"
#endif

#if DT_NODE_EXISTS(DT_ALIAS(fibre_usb))
fibre_transport::UsbBackend transport_backend_;
#else
fibre_transport::UartBackend transport_backend_;
#endif
fibre::ZephyrEventLoop event_loop_;

/* --------------------------------------------------------------------------
 * Signal path (ISR context)
 */

/* Telemetry mirror: copy the tick's engine state into the published
 * Fibre object. Plain stores; see the concurrency note in the header
 * comment. */
void telemetry_refresh(void)
{
	uint8_t dig = 0;
	uint8_t oe = 0;

	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		if (cas_st.dig[ch] != 0u) {
			dig |= BIT(ch);
		}
		if (cas_st.oe[ch] != 0u) {
			oe |= BIT(ch);
		}
	}
	cas_object_.rpm_ = (float)cas_st.rpm;
	cas_object_.theta_deg_ = (float)cas_st.theta;
	cas_object_.dig_mask_ = dig;
	cas_object_.oe_mask_ = oe;
	cas_object_.ssi1_ = cas_st.ssi1;
	cas_object_.ssi2_ = cas_st.ssi2;
	cas_object_.sample_count_++;
}

void tick_isr(const struct device *dev, void *user_data)
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

	telemetry_refresh();
}

/* Wire the signal path: PE0..7 as high-Z inputs, then the 10 us tick. */
int signal_path_start(void)
{
	if (!device_is_ready(dig_port)) {
		LOG_ERR("GPIOE not ready");
		return -ENODEV;
	}
	if (!device_is_ready(tick_cnt)) {
		LOG_ERR("TIM8 counter not ready");
		return -ENODEV;
	}

	oe_driven = 0;
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		int rc = gpio_pin_configure(dig_port, ch, GPIO_INPUT);

		if (rc != 0) {
			LOG_ERR("PE%d configure failed: %d", ch, rc);
			return rc;
		}
	}

	uint32_t freq = counter_get_frequency(tick_cnt);
	uint32_t ticks = freq / CAS_TICK_HZ;

	if (freq < CAS_TICK_HZ || (freq % CAS_TICK_HZ) != 0u ||
	    ticks == 0u || ticks > 0x10000u) {
		LOG_ERR("TIM8 clock %u Hz cannot tick at %u Hz", freq,
			CAS_TICK_HZ);
		return -EINVAL;
	}

	struct counter_top_cfg top = {};

	top.ticks = ticks - 1u; /* ARR: period = ticks + 1 clocks */
	top.callback = tick_isr;

	int rc = counter_set_top_value(tick_cnt, &top);

	if (rc != 0) {
		LOG_ERR("TIM8 top value failed: %d", rc);
		return rc;
	}
	rc = counter_start(tick_cnt);
	if (rc != 0) {
		LOG_ERR("TIM8 start failed: %d", rc);
	}
	return rc;
}

/* --------------------------------------------------------------------------
 * Fibre -> Zephyr log bridge
 *
 * Only produces output in builds with a log backend (rtt.conf); compiled out
 * otherwise.
 */

void fibre_log_bridge(void * /*ctx*/, const char *file, unsigned line,
		      int level, uintptr_t /*info0*/, uintptr_t /*info1*/,
		      const char *text)
{
	if (level >= 4) { /* kDebug / kTrace */
		LOG_DBG("fibre: %s:%u: %s", file, line, text ? text : "");
	} else if (level == 1) { /* kError */
		LOG_ERR("fibre: %s:%u: %s", file, line, text ? text : "");
	} else {
		LOG_INF("fibre: %s:%u: %s", file, line, text ? text : "");
	}
}

fibre::Logger fibre_logger_{fibre::log_fn_t{&fibre_log_bridge, nullptr},
			    fibre::LogLevel::kDebug};

/* --------------------------------------------------------------------------
 * Event loop startup
 *
 * Everything that touches the Fibre stack must run on the event loop thread,
 * so the whole setup happens in the on_started callback. The functor lives at
 * namespace scope because Callback stores a pointer to it.
 */

struct OnStarted {
	void operator()() const
	{
		fibre::Fibre *ctx = nullptr;

		if (fibre::open(&event_loop_, fibre_logger_, &ctx).is_error()) {
			LOG_ERR("failed to open Fibre context");
			return;
		}

#if DT_NODE_EXISTS(DT_ALIAS(fibre_usb))
		if (ctx->init_backend("usb", &transport_backend_).is_error()) {
			LOG_ERR("failed to initialize USB backend");
			return;
		}

		/* Domain path "<transport>:<specs>": the USB backend ignores
		 * the specs part (single fixed interface). */
		fibre::Domain *domain = ctx->create_domain("usb:", kNodeId, {});
		if (!domain) {
			LOG_ERR("failed to create Fibre domain");
			return;
		}

		LOG_INF("fibre: domain ready, serving cas_object over USB bulk");
#else
		if (ctx->init_backend("uart", &transport_backend_).is_error()) {
			LOG_ERR("failed to initialize UART backend");
			return;
		}

		/* Domain path "<transport>:<specs>": the UART backend ignores
		 * the specs part (single fixed port). */
		fibre::Domain *domain = ctx->create_domain("uart:", kNodeId, {});
		if (!domain) {
			LOG_ERR("failed to create Fibre domain");
			return;
		}

		LOG_INF("fibre: domain ready, serving cas_object over UART");
#endif
	}
};

const OnStarted on_started_{};

} // namespace

/* Consumed by the generated static_exports.cpp ("must be defined by the
 * application"). The generated interface classes live in the global
 * namespace. */
CasIntfIntf *cas_object_ptr = &cas_object_wrapper_;

int main(void)
{
	banner_print();

	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("LED device not ready");
		return -ENODEV;
	}
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE) < 0) {
		LOG_ERR("LED configure failed");
		return -EIO;
	}

	/* Power-up state: engine stopped (const 0 rpm), all faults off
	 * (FR-34). The speed and fault set come in over Fibre. */
	cas_cfg_t cfg;
	cas_profile_t prof = {};
	cas_ssi_cfg_t ssi = {};

	prof.type = CAS_PROF_CONST;
	prof.rpm = 0.0;

	cas_preset_x1plus(&cfg);
	cas_fault_reset(&cas_faults);
	cas_init(&cas_st, &cfg, &prof, &ssi, 1.0 / (double)CAS_TICK_HZ);

	LOG_INF("CASensorSim: X1+ preset (186 teeth), const 0 rpm, "
		"tick %u Hz, CH00..07 = PE0..7",
		CAS_TICK_HZ);

	int rc = signal_path_start();

	if (rc != 0) {
		LOG_ERR("signal path start failed: %d", rc);
		return rc;
	}

	if (event_loop_.start(fibre_logger_, fibre::Callback<void>{on_started_})
		    .is_error()) {
		LOG_ERR("failed to start Fibre event loop");
		return 1;
	}

	/* Heartbeat: blink the on-board LED (PC13, active low). */
	for (;;) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}
	return 0;
}
