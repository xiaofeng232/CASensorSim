/*
 * Host golden test for cas_core (HANDOFF.md 7.5-4). Plain C11, no Zephyr:
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
 *                        (bit-level parity with the casim golden model).
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cas_core.h"
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

/* ------------------------------------------------------------------ 4 */

static void test_golden_baseline(void)
{
	/* X1+ preset, const 60 rpm, dt=20 us, 5 s -> N=250001 samples.
	 * Expected h31/ones from HANDOFF.md 6 (casim golden model). */
	static const uint32_t N = 250001;
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

	cas_cfg_t cfg;
	cas_profile_t prof = {.type = CAS_PROF_CONST, .rpm = 60.0};
	cas_ssi_cfg_t ssi = {0};
	cas_state_t st;
	uint32_t h[CAS_N_DIG] = {0};
	uint32_t ones[CAS_N_DIG] = {0};
	uint32_t hssi[CAS_N_SSI] = {0};

	cas_preset_x1plus(&cfg);
	cas_init(&st, &cfg, &prof, &ssi, 20e-6);

	for (uint32_t i = 0; i < N; i++) {
		if (i > 0) {
			cas_step(&st);
		}
		for (int ch = 0; ch < CAS_N_DIG; ch++) {
			h[ch] = h[ch] * 31u + st.dig[ch];
			ones[ch] += st.dig[ch];
		}
		hssi[0] = hssi[0] * 31u + (uint32_t)st.ssi1;
		hssi[1] = hssi[1] * 31u + (uint32_t)st.ssi2;
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

int main(void)
{
	test_helpers();
	test_profiles();
	test_wave_boundary();
	test_ssi();
	test_golden_baseline();

	if (failures == 0) {
		printf("cas_core: ALL TESTS PASSED\n");
		return 0;
	}
	printf("cas_core: %d FAILURE(S)\n", failures);
	return 1;
}
