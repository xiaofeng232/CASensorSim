/*
 * Host golden test for cas_core + cas_fault (HANDOFF.md 7.5-4). Plain
 * C11, no Zephyr:
 *
 *     make -C test test
 *
 * Layers:
 *   1. unit asserts   -- cas_rne (banker's rounding), cas_fmod_pos
 *                        (Python % semantics), speed profiles
 *   2. wave boundary  -- dyadic-exact square-wave configuration: every
 *                        theta/center/width value is exactly representable,
 *                        so the < boundary and the mod-wrap are checked
 *                        against an integer oracle
 *   3. SSI words      -- 0/90/180/270 deg, offset and PFB bit
 *   4. golden baseline-- X1+ preset, const 60 rpm, dt = 20 us, 5 s
 *                        (N = 250001): per-channel h31 hashes and
 *                        high-level counts must equal HANDOFF.md 6 exactly
 *                        (bit-level parity with the casim golden model)
 *   5. fault vectors  -- HANDOFF.md 7.4: FR-34 all-off bit-identity,
 *                        FR-30 digital break, FR-31 PFB / offset / hold /
 *                        single frame / failure, FR-33 triggers
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cas_core.h"
#include "cas_fault.h"
#include "cas_preset.h"

static int failures;

#define CHECK(cond, ...)                                                       \
	do {                                                                   \
		if (!(cond)) {                                                 \
			failures++;                                            \
			printf("FAIL: ");                                      \
			printf(__VA_ARGS__);                                   \
			printf("\n");                                          \
		}                                                              \
	} while (0)

/* ------------------------------------------------------------------ 1 */

static void test_helpers(void)
{
	/* np.round round-half-to-even (HANDOFF.md 7.3-7) */
	CHECK(cas_rne(0.5) == 0.0, "rne(0.5)=%f", cas_rne(0.5));
	CHECK(cas_rne(1.5) == 2.0, "rne(1.5)=%f", cas_rne(1.5));
	CHECK(cas_rne(2.5) == 2.0, "rne(2.5)=%f", cas_rne(2.5));
	CHECK(cas_rne(3.5) == 4.0, "rne(3.5)=%f", cas_rne(3.5));
	CHECK(cas_rne(67.5) == 68.0, "rne(67.5)=%f", cas_rne(67.5));

	/* Python float % (HANDOFF.md 7.3-6) */
	CHECK(cas_fmod_pos(-1.0, 360.0) == 359.0, "fmod_pos(-1,360)=%f",
	      cas_fmod_pos(-1.0, 360.0));
	CHECK(cas_fmod_pos(720.5, 360.0) == 0.5, "fmod_pos(720.5,360)=%f",
	      cas_fmod_pos(720.5, 360.0));
	CHECK(cas_fmod_pos(0.0, 360.0) == 0.0, "fmod_pos(0,360)=%f",
	      cas_fmod_pos(0.0, 360.0));
}

static void test_profiles(void)
{
	cas_profile_t p;
	double v;

	p = (cas_profile_t){.type = CAS_PROF_RAMP, .start_rpm = 0.0,
			    .target_rpm = 60.0, .accel_rpm = 10.0};
	CHECK(cas_profile_rpm(&p, 0.5) == 5.0, "ramp up t=0.5 -> %f",
	      cas_profile_rpm(&p, 0.5));
	CHECK(cas_profile_rpm(&p, 100.0) == 60.0, "ramp clip high");

	p = (cas_profile_t){.type = CAS_PROF_RAMP, .start_rpm = 60.0,
			    .target_rpm = 0.0, .accel_rpm = -10.0};
	CHECK(cas_profile_rpm(&p, 1.0) == 50.0, "ramp down t=1 -> %f",
	      cas_profile_rpm(&p, 1.0));

	/* d_up = d_down = 6 s, period 12 s */
	p = (cas_profile_t){.type = CAS_PROF_RAMP_CYCLE, .start_rpm = 0.0,
			    .target_rpm = 60.0, .accel_rpm = 10.0,
			    .ret_rpm = 0.0};
	CHECK(cas_profile_rpm(&p, 3.0) == 30.0, "cycle up t=3");
	CHECK(cas_profile_rpm(&p, 6.0) == 60.0, "cycle top t=6");
	CHECK(cas_profile_rpm(&p, 9.0) == 30.0, "cycle down t=9");
	v = cas_profile_rpm(&p, 12.1);
	CHECK(fabs(v - 1.0) < 1e-9, "cycle wrap t=12.1 -> %f", v);

	/* sin() is libm-dependent: tolerance, not equality (VR-01) */
	p = (cas_profile_t){.type = CAS_PROF_FLUCT, .rpm = 60.0,
			    .amp_rpm = 5.0, .freq_hz = 1.0};
	v = cas_profile_rpm(&p, 0.0);
	CHECK(fabs(v - 60.0) < 1e-9, "fluct t=0 -> %f", v);
	v = cas_profile_rpm(&p, 0.25);
	CHECK(fabs(v - 65.0) < 1e-9, "fluct peak -> %f", v);
}

/* ------------------------------------------------------------------ 2 */

static void test_wave_boundary(void)
{
	/* Dyadic-exact configuration (all values exactly representable):
	 * teeth=45 -> tooth period P = 8.0 deg; A at pos 0 (center 0),
	 * width 5.625 deg (= 45/8); rpm=60, dt=2^-10 s
	 * -> inc = 360*2^-10 = 45/128 deg/sample (exact), one revolution
	 * = 1024 samples. In 1/128 deg units, u = (45*k) mod 1024 is theta:
	 *   rel = (u + 360) mod 1024, high iff rel < 720 (= width 5.625)
	 *       iff  u < 360  or  u >= 664    (integer oracle).
	 * u = 360 (theta = 2.8125 = center + w/2) must be LOW (boundary
	 * excluded); u = 664 (theta = center - w/2) must be HIGH. */
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	const int n = 1024; /* exactly one revolution */

	cas_preset_default100(&cfg);
	cfg.teeth = 45;
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		cfg.pos_mdeg[ch] = 0;
		cfg.width_mdeg[ch] = 0;
	}
	cfg.pos_mdeg[2] = 0;      /* CH02 A: center at 0 deg */
	cfg.width_mdeg[2] = 5625; /* width 5.625 deg */

	cas_init(&st, &cfg, &prof, &ssi, 1.0 / 1024.0);

	for (int k = 0; k < n; k++) {
		uint32_t u = (uint32_t)(45 * k) % 1024u;
		int expect = (u < 360u) || (u >= 664u);

		if (k > 0) {
			cas_step(&st);
		}
		CHECK(st.dig[2] == expect, "A k=%d: got %u expect %d", k,
		      st.dig[2], expect);
	}
}

/* ------------------------------------------------------------------ 3 */

static void test_ssi(void)
{
	/* dt=0.25 s, rpm=60 -> inc = 90 deg/sample (exact). */
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;

	cas_preset_default100(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 0.25);

	CHECK(st.ssi1 == 0, "ssi(0 deg)=%d", st.ssi1);
	cas_step(&st);
	CHECK(st.ssi1 == 1024, "ssi(90)=%d", st.ssi1);
	cas_step(&st);
	CHECK(st.ssi1 == 2048, "ssi(180)=%d", st.ssi1);
	cas_step(&st);
	CHECK(st.ssi1 == 3072, "ssi(270)=%d", st.ssi1);
	cas_step(&st);
	CHECK(st.ssi1 == 0, "ssi(360 wrap)=%d", st.ssi1);

	/* angle offset 500 mDeg: a=0.5 deg -> round(0.5/360*4096)=6 */
	ssi = (cas_ssi_cfg_t){.ssi1_off_mdeg = 500, .ssi2_off_mdeg = 0,
			      .pfb1 = false, .pfb2 = true};
	cas_init(&st, &cfg, &prof, &ssi, 0.25);
	CHECK(st.ssi1 == 6, "ssi offset 500 mDeg -> %d", st.ssi1);
	CHECK(st.ssi2 == (1 << CAS_SSI_PFB_SHIFT), "ssi2 pfb=%d", st.ssi2);
}

/* ------------------------------------------------------------------ 4/5 */

#define GOLD_N 250001u /* X1+ baseline: 5 s @ dt = 20 us */

/* Baseline samples, shared by the golden-hash and fault-vector tests. */
static uint8_t base_dig[GOLD_N][CAS_N_DIG];
static int32_t base_ssi[GOLD_N][CAS_N_SSI];
static bool base_ran;

static void run_baseline(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;

	if (base_ran) {
		return;
	}
	base_ran = true;

	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);
	for (uint32_t i = 0; i < GOLD_N; i++) {
		int32_t pair[CAS_N_SSI];

		if (i > 0) {
			cas_step(&st);
		}
		pair[0] = st.ssi1;
		pair[1] = st.ssi2;
		memcpy(base_dig[i], st.dig, sizeof(st.dig));
		memcpy(base_ssi[i], pair, sizeof(pair));
	}
}

static void test_golden_baseline(void)
{
	static const struct {
		uint32_t h;
		uint32_t ones;
	} exp_dig[CAS_N_DIG] = {
		{3205541500u, 696},   /* CH00 TDC1 */
		{1802195195u, 695},   /* CH01 REF1 */
		{3446791681u, 77501}, /* CH02 A    */
		{2104567553u, 77501}, /* CH03 B    */
		{1802195195u, 695},   /* CH04 REF2 */
		{3446791681u, 77501}, /* CH05 C    */
		{2104567553u, 77501}, /* CH06 D    */
		{3205541500u, 696},   /* CH07 TDC2 */
	};
	static const uint32_t exp_ssi[CAS_N_SSI] = {1136600576u,
						    1136600576u};
	uint32_t h[CAS_N_DIG] = {0};
	uint32_t ones[CAS_N_DIG] = {0};
	uint32_t hssi[CAS_N_SSI] = {0};

	run_baseline();

	for (uint32_t i = 0; i < GOLD_N; i++) {
		for (int ch = 0; ch < CAS_N_DIG; ch++) {
			h[ch] = h[ch] * 31u + base_dig[i][ch];
			ones[ch] += base_dig[i][ch];
		}
		for (int s = 0; s < CAS_N_SSI; s++) {
			hssi[s] = hssi[s] * 31u + (uint32_t)base_ssi[i][s];
		}
	}

	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		CHECK(h[ch] == exp_dig[ch].h,
		      "CH%02d hash %u != %u (ones %u vs %u)", ch, h[ch],
		      exp_dig[ch].h, ones[ch], exp_dig[ch].ones);
		CHECK(ones[ch] == exp_dig[ch].ones, "CH%02d ones %u != %u", ch,
		      ones[ch], exp_dig[ch].ones);
	}
	for (int s = 0; s < CAS_N_SSI; s++) {
		CHECK(hssi[s] == exp_ssi[s], "SSI%d hash %u != %u", s + 1,
		      hssi[s], exp_ssi[s]);
	}
}

/* Run the X1+ baseline through an (enabled) fault set, comparing every
 * sample against the stored fault-free baseline. Returns the h31 hash
 * of dig[ch] and the high-level count of dig[ch]. */
static void run_fault(const cas_fault_t *fault_in, int ch_probe,
		      uint32_t *h_out, uint32_t *ones_out)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault = *fault_in;
	uint32_t h = 0;
	uint32_t ones = 0;

	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);

		h = h * 31u + st.dig[ch_probe];
		ones += st.dig[ch_probe];
	}
	*h_out = h;
	*ones_out = ones;
}

/* FR-34: with every fault disabled the output must be bit-identical. */
static void test_fault_all_off(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;

	cas_fault_reset(&fault);
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		for (int ch = 0; ch < CAS_N_DIG; ch++) {
			CHECK(st.dig[ch] == base_dig[i][ch],
			      "FR-34 dig[%d] differs at k=%u", ch, i);
			CHECK(st.oe[ch] == 1u, "FR-34 oe[%d]=%u at k=%u", ch,
			      st.oe[ch], i);
		}
		CHECK(st.ssi1 == base_ssi[i][0] && st.ssi2 == base_ssi[i][1],
		      "FR-34 ssi differs at k=%u", i);
		CHECK(st.ssi_fail[0] == 0u && st.ssi_fail[1] == 0u,
		      "FR-34 ssi_fail set at k=%u", i);
	}
	CHECK(1, "FR-34 all-off run is bit-identical (no FAIL lines above)");
}

/* FR-30: CH00 (TDC1) break, trigger EVERY_N_REV n=2 -> latches at rev 2
 * (t = 2.0 s, k = 100000). Samples k >= 100000: dig=0, oe=0; before that
 * bit-identical to baseline. Expected ones = both intact rev windows =
 * 2 * 139 = 278 (HANDOFF 7.4 vector 2: "2/5 x 696 ~ 278"). */
static void test_fault_dig_break(void)
{
	cas_fault_t fault;
	uint32_t h = 0;
	uint32_t ones = 0;
	uint32_t h_oracle = 0;

	cas_fault_reset(&fault);
	fault.dig[0] = (cas_dig_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_EVERY_N_REV,
			 .every_n = 2},
	};
	run_fault(&fault, 0, &h, &ones);

	/* oracle: baseline hash with dig[0] forced to 0 from k=100000 */
	for (uint32_t i = 0; i < GOLD_N; i++) {
		h_oracle = h_oracle * 31u + (i >= 100000u ? 0u : base_dig[i][0]);
	}

	CHECK(ones == 278u, "FR-30 ones %u != 278", ones);
	CHECK(h == h_oracle, "FR-30 hash %u != oracle %u", h, h_oracle);
}

/* FR-31: SSI2 PFB (IMMEDIATE): bit12 set on every word, low 12 bits
 * identical to baseline; SSI1 untouched. */
static void test_fault_ssi_pfb(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;

	cas_fault_reset(&fault);
	fault.ssi[1] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_IMMEDIATE},
		.pfb = true,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		CHECK(st.ssi2 == (base_ssi[i][1] | (1 << CAS_SSI_PFB_SHIFT)),
		      "pfb k=%u: %d vs %d", i, st.ssi2, base_ssi[i][1]);
		CHECK(st.ssi1 == base_ssi[i][0], "ssi1 touched at k=%u", i);
	}
}

/* FR-31: SSI2 permanent offset +500 mDeg (IMMEDIATE): every word gets
 * round(500/360000*4096) = 6 counts added (mod 4096), HANDOFF 7.4
 * vector 4. */
static void test_fault_ssi_offset(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;

	cas_fault_reset(&fault);
	fault.ssi[1] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_IMMEDIATE},
		.offset_mdeg = 500,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		int32_t expect = (base_ssi[i][1] + 6) % 4096;

		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		CHECK(st.ssi2 == expect, "offset k=%u: %d vs %d", i, st.ssi2,
		      expect);
	}
}

/* FR-31: SSI2 angle hold, trigger AT_ANGLE=180 deg -> word frozen at
 * the activation frame on; before that bit-identical to baseline
 * (HANDOFF 7.4 vector 5: activation word = 2048 at 180 deg). The
 * activation sample is located dynamically (first theta >= 180 deg):
 * cumulated theta is not exactly representable, so hardcoding k would
 * be off by one sample depending on rounding. */
static void test_fault_ssi_hold(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;
	uint32_t k_act = GOLD_N;
	int32_t held = 0;
	uint32_t mismatches = 0;

	cas_fault_reset(&fault);
	fault.ssi[1] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_AT_ANGLE,
			 .angle_mdeg = 180000},
		.hold = true,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		if (k_act == GOLD_N && st.theta >= 180.0) {
			k_act = i;
			held = st.ssi2; /* activation frame word */
		}
		if (i < k_act) {
			if (st.ssi2 != base_ssi[i][1]) {
				mismatches++;
			}
		} else if (st.ssi2 != held) {
			mismatches++;
		}
	}
	CHECK(k_act < GOLD_N, "hold: never activated");
	CHECK(base_ssi[k_act][1] == 2048, "hold: activation word %d != 2048",
	      base_ssi[k_act][1]);
	CHECK(mismatches == 0, "hold: %u mismatched samples", mismatches);
}

/* FR-31: SSI2 single-frame perturbation +800 counts, trigger AT_ANGLE=
 * 90 deg -> exactly the activation word (1024 at 90 deg) becomes
 * (1024+800) mod 4096 = 1824, every other sample bit-identical. */
static void test_fault_ssi_single(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;
	uint32_t k_act = GOLD_N;
	uint32_t mismatches = 0;
	int32_t got_at_act = 0;

	cas_fault_reset(&fault);
	fault.ssi[1] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_AT_ANGLE,
			 .angle_mdeg = 90000},
		.single_frame_delta = 800,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		if (k_act == GOLD_N && st.theta >= 90.0) {
			k_act = i;
			got_at_act = st.ssi2;
		}
		if (i == k_act) {
			continue; /* checked after the loop */
		}
		if (st.ssi2 != base_ssi[i][1]) {
			mismatches++;
		}
	}
	CHECK(k_act < GOLD_N, "single: never activated");
	CHECK(base_ssi[k_act][1] == 1024, "single: activation word %d != 1024",
	      base_ssi[k_act][1]);
	CHECK(got_at_act == 1824, "single: k_act word %d != 1824", got_at_act);
	CHECK(mismatches == 0, "single: leaked into %u other samples",
	      mismatches);
}

/* FR-31 fail -> ssi_fail flags only: SSI1 IMMEDIATE (flag from k=0 on),
 * SSI2 EVERY_N_REV n=1 (flag from k=50000, first rev wrap, on). Words
 * themselves must stay bit-identical to baseline. */
static void test_fault_ssi_fail(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;

	cas_fault_reset(&fault);
	fault.ssi[0] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_IMMEDIATE},
		.fail = true,
	};
	fault.ssi[1] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_EVERY_N_REV,
			 .every_n = 1},
		.fail = true,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		CHECK(st.ssi_fail[0] == 1u, "fail1 k=%u: %u", i,
		      st.ssi_fail[0]);
		CHECK(st.ssi_fail[1] == (i >= 50000u ? 1u : 0u),
		      "fail2 k=%u: %u", i, st.ssi_fail[1]);
		CHECK(st.ssi1 == base_ssi[i][0] && st.ssi2 == base_ssi[i][1],
		      "fail touched words at k=%u", i);
	}
}

/* FR-33: EVERY_N_SSI n=100 -> first modified frame is k=99 (apply #100). */
static void test_fault_ssi_every_n_frame(void)
{
	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	cas_fault_t fault;

	cas_fault_reset(&fault);
	fault.ssi[0] = (cas_ssi_fault_t){
		.cond = {.enable = true, .trigger = CAS_FT_EVERY_N_SSI,
			 .every_n = 100},
		.pfb = true,
	};
	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < GOLD_N; i++) {
		int32_t expect = (i >= 99u)
					 ? (base_ssi[i][0] |
					    (1 << CAS_SSI_PFB_SHIFT))
					 : base_ssi[i][0];

		if (i > 0) {
			cas_step(&st);
		}
		cas_fault_apply(&fault, &st);
		CHECK(st.ssi1 == expect, "every-n-frame k=%u: %d vs %d", i,
		      st.ssi1, expect);
	}
}

/* ------------------------------------------------------------------ */

int main(void)
{
	test_helpers();
	test_profiles();
	test_wave_boundary();
	test_ssi();
	test_golden_baseline();
	test_fault_all_off();
	test_fault_dig_break();
	test_fault_ssi_pfb();
	test_fault_ssi_offset();
	test_fault_ssi_hold();
	test_fault_ssi_single();
	test_fault_ssi_fail();
	test_fault_ssi_every_n_frame();

	if (failures == 0) {
		printf("cas_core: ALL TESTS PASSED\n");
		return 0;
	}
	printf("cas_core: %d FAILURE(S)\n", failures);
	return 1;
}
