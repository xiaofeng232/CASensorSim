/*
 * cas_core implementation -- see cas_core.h for the bit-exactness rules.
 *
 * Every formula below mirrors one casim source line (file/line references
 * in comments). Operation order is normative: changing it changes the
 * low bits and breaks the golden-hash verification (HANDOFF.md 6).
 */

#include <math.h>

#include "cas_core.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Channel index map (NI 9474 wiring): CH00 TDC1, CH01 REF1, CH02 A,
 * CH03 B, CH04 REF2, CH05 C, CH06 D, CH07 TDC2.
 * Tooth-pickup channels share the flywheel gear; the rest read marks. */
#define CH_TDC1 0
#define CH_REF1 1
#define CH_A    2
#define CH_B    3
#define CH_REF2 4
#define CH_C    5
#define CH_D    6
#define CH_TDC2 7

static bool ch_is_tooth(int ch)
{
	return ch == CH_A || ch == CH_B || ch == CH_C || ch == CH_D;
}

double cas_fmod_pos(double x, double m)
{
	/* Python float % m: result in [0, m) with the divisor's sign. */
	double r = fmod(x, m);
	if (r < 0.0) {
		r += m;
	}
	return r;
}

double cas_rne(double x)
{
	/* np.round: round-half-to-even (banker's rounding). */
	double f = floor(x);
	double d = x - f;
	if (d > 0.5) {
		return f + 1.0;
	}
	if (d < 0.5) {
		return f;
	}
	/* d == 0.5: tie -> even mantissa */
	if (fmod(f, 2.0) == 0.0) {
		return f;
	}
	return f + 1.0;
}

double cas_profile_rpm(const cas_profile_t *prof, double t)
{
	switch (prof->type) {
	case CAS_PROF_CONST:
		/* profile.py ConstantSpeed.speed() */
		return prof->rpm;

	case CAS_PROF_RAMP: {
		/* profile.py RampSpeed.speed(), cycle=False:
		 *   clip(start + accel*t, min(start, target), max(start, target)) */
		double lo = fmin(prof->start_rpm, prof->target_rpm);
		double hi = fmax(prof->start_rpm, prof->target_rpm);
		double v = prof->accel_rpm * t;

		/* Multiply and add live in separate statements on purpose:
		 * no FMA contraction may reorder the rounding. */
		return fmin(fmax(prof->start_rpm + v, lo), hi);
	}

	case CAS_PROF_RAMP_CYCLE: {
		/* profile.py RampSpeed.speed(), cycle=True */
		double a = fabs(prof->accel_rpm);
		if (a == 0.0) {
			a = 1e-9; /* abs(accel) or 1e-9 */
		}
		double d_up = (prof->target_rpm - prof->start_rpm) / a;
		double d_down = (prof->target_rpm - prof->ret_rpm) / a;
		double period = d_up + d_down;
		double tc = cas_fmod_pos(t, period);
		double rise = prof->accel_rpm * fmin(tc, d_up);
		double fall = prof->accel_rpm *
			      fmin(fmax(tc - d_up, 0.0), d_down);
		double up = prof->start_rpm + rise;
		double down = prof->target_rpm - fall;

		return (tc <= d_up) ? up : down;
	}

	case CAS_PROF_FLUCT: {
		/* profile.py Fluctuation.speed(): base + amp*sin(2*pi*f*t).
		 * Note: sin() is libm-dependent -- cross-platform results may
		 * differ in the last ulp (VR-01 tolerates +-1 sample). */
		double wave = prof->amp_rpm *
			      sin(2.0 * M_PI * prof->freq_hz * t);

		return prof->rpm + wave;
	}

	default:
		return prof->rpm;
	}
}

/* Compute dig[]/oe[]/ssi1/ssi2 for the current theta (sensor.py signals(),
 * square_wave() and ssi.py words()). */
static void cas_compute(cas_state_t *st)
{
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		/* square_wave(): rel = (angle - center + 0.5*width) % period
		 * (multiply and add split across statements: no FMA). */
		double half = 0.5 * st->width[ch];
		double rel = cas_fmod_pos((st->theta - st->center[ch]) + half,
					  st->period[ch]);

		st->dig[ch] = (rel < st->width[ch]) ? 1u : 0u;
		st->oe[ch] = 1u; /* healthy default; fault module clears */
	}

	for (int s = 0; s < CAS_N_SSI; s++) {
		/* ssi.py _counts(): a = (angle + offset) % 360;
		 * counts = round(a/360*resolution) % resolution */
		double a = cas_fmod_pos(st->theta + st->ssi_off[s],
					CAS_REV_DEG);
		double c = cas_rne((a / CAS_REV_DEG) *
				   (double)CAS_SSI_RESOLUTION);
		int32_t word = (int32_t)fmod(c, (double)CAS_SSI_RESOLUTION);
		if (st->pfb[s]) {
			word |= 1 << CAS_SSI_PFB_SHIFT;
		}
		if (s == 0) {
			st->ssi1 = word;
		} else {
			st->ssi2 = word;
		}
	}
}

void cas_init(cas_state_t *st, const cas_cfg_t *cfg,
	      const cas_profile_t *prof, const cas_ssi_cfg_t *ssi, double dt)
{
	double tooth_period = CAS_REV_DEG / (double)cfg->teeth;

	st->dt = dt;
	st->start = 0.0; /* start angle fixed at 0 deg for now */
	st->cum = 0.0;
	st->prof = *prof;

	/* Precompute per-channel geometry: center = (pos - mark) mod period
	 * (sensor.py centers()); tooth channels use period P and tooth0 as
	 * the mark, mark channels use 360 deg and their own mark. */
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		double pos = (double)cfg->pos_mdeg[ch] / 1000.0;
		double mark;

		if (ch_is_tooth(ch)) {
			mark = (double)cfg->tooth0_mdeg / 1000.0;
			st->period[ch] = tooth_period;
		} else if (ch == CH_REF1) {
			mark = (double)cfg->ref1_mark_mdeg / 1000.0;
			st->period[ch] = CAS_REV_DEG;
		} else if (ch == CH_REF2) {
			mark = (double)cfg->ref2_mark_mdeg / 1000.0;
			st->period[ch] = CAS_REV_DEG;
		} else { /* TDC1 / TDC2 share the unique TDC mark */
			mark = (double)cfg->tdc_mark_mdeg / 1000.0;
			st->period[ch] = CAS_REV_DEG;
		}

		st->center[ch] = cas_fmod_pos(pos - mark, st->period[ch]);
		st->width[ch] = (double)cfg->width_mdeg[ch] / 1000.0;
	}

	st->ssi_off[0] = (double)ssi->ssi1_off_mdeg / 1000.0;
	st->ssi_off[1] = (double)ssi->ssi2_off_mdeg / 1000.0;
	st->pfb[0] = ssi->pfb1;
	st->pfb[1] = ssi->pfb2;

	/* Sample k=0: t = 0*dt, theta = start + 0 (generator.py run()). */
	st->k = 0;
	st->t = (double)st->k * st->dt;
	st->rpm = cas_profile_rpm(&st->prof, st->t);
	st->rpm_prev = st->rpm;
	st->theta = st->start + st->cum;
	cas_compute(st);
}

void cas_step(cas_state_t *st)
{
	/* generator.py run(): inc = 0.5*(rpm[1:] + rpm[:-1])*6.0*dt;
	 * angle = start + cumsum(inc). Order per HANDOFF.md 7.3-2; the
	 * product is materialized in `inc` first so no FMA contraction
	 * can fuse it into the accumulation. */
	double inc;
	double rpm_new;

	st->k = st->k + 1;
	st->t = (double)st->k * st->dt;
	rpm_new = cas_profile_rpm(&st->prof, st->t);
	inc = ((0.5 * (st->rpm_prev + rpm_new)) * 6.0) * st->dt;
	st->cum += inc;
	st->theta = st->start + st->cum;
	st->rpm_prev = rpm_new;
	st->rpm = rpm_new;

	cas_compute(st);
}
