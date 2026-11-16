/*
 * cas_fault implementation -- see cas_fault.h for the FR-30..FR-34
 * semantics (HANDOFF.md 7.4). Platform-independent C, no FMA-prone
 * multiply-add chains (the rne() input is materialized in x first).
 */
#include "cas_fault.h"

#include <math.h>
#include <string.h>

/* Angle-domain trigger test: did theta cross `angle_mdeg` (mod 360)
 * between the previous and the current sample (FR-33 AT_ANGLE)? */
static bool crossed_at_angle(const cas_fault_t *f, double theta,
			     uint32_t angle_mdeg)
{
	double prev_a = cas_fmod_pos(f->prev_theta, CAS_REV_DEG);
	double cur_a = cas_fmod_pos(theta, CAS_REV_DEG);
	double ang = (double)angle_mdeg / 1000.0;

	if (!f->have_prev) {
		return false; /* no previous sample yet */
	}
	if (theta >= f->prev_theta) { /* forward rotation */
		if (cur_a >= prev_a) {
			return (prev_a < ang) && (ang <= cur_a);
		}
		/* wrapped through 0 this step */
		return (ang > prev_a) || (ang <= cur_a);
	}
	/* reverse rotation */
	if (cur_a <= prev_a) {
		return (cur_a <= ang) && (ang < prev_a);
	}
	/* wrapped through 0 downwards this step */
	return (ang < prev_a) || (ang >= cur_a);
}

/* FR-33 trigger state machine ("active-and-stay" latching). */
static void cond_eval(cas_fault_cond_t *c, const cas_fault_t *f,
		      double theta)
{
	uint32_t n = (c->every_n != 0u) ? c->every_n : 1u;

	if (!c->enable) {
		c->active = false; /* disabled -> inert (FR-34) */
		return;
	}

	switch (c->trigger) {
	case CAS_FT_IMMEDIATE:
		c->active = true;
		break;
	case CAS_FT_AT_ANGLE:
		if (crossed_at_angle(f, theta, c->angle_mdeg)) {
			c->active = true;
		}
		break;
	case CAS_FT_EVERY_N_REV:
		if (f->rev_count > 0u && (f->rev_count % n) == 0u) {
			c->active = true;
		}
		break;
	case CAS_FT_EVERY_N_SSI:
		if (f->ssi_frame_count > 0u &&
		    (f->ssi_frame_count % n) == 0u) {
			c->active = true;
		}
		break;
	default:
		break;
	}
}

/* Wrap a 12 bit SSI count into [0, 4096) (Python % semantics). */
static int32_t ssi_wrap(int32_t c)
{
	return ((c % CAS_SSI_RESOLUTION) + CAS_SSI_RESOLUTION) %
	       CAS_SSI_RESOLUTION;
}

/* Apply one stream's effects to the word cas_step() just produced.
 * Order per HANDOFF 7.4: offset/hold -> single frame -> PFB -> fail. */
static void ssi_fault_apply(cas_ssi_fault_t *sf, cas_fault_t *f,
			    cas_state_t *st, int s)
{
	int32_t *word_ptr = (s == 0) ? &st->ssi1 : &st->ssi2;
	int32_t mask = (int32_t)(CAS_SSI_RESOLUTION - 1);
	int32_t word;

	if (!sf->cond.active) {
		f->hold_captured[s] = false; /* re-arm one-shot effects */
		f->single_done[s] = false;
		return;
	}

	word = *word_ptr;

	if (sf->hold) {
		/* FR-31 angle hold: freeze at the activation-frame word. */
		if (!f->hold_captured[s]) {
			f->held_word[s] = word;
			f->hold_captured[s] = true;
		}
		word = f->held_word[s];
	} else if (sf->offset_mdeg != 0) {
		/* FR-31 angle offset, count domain per HANDOFF vector 4:
		 * word += round(offset_mdeg / 360000 * 4096) (mod 4096). */
		double x = (double)sf->offset_mdeg / 360000.0;
		int32_t delta;
		int32_t low;

		x = x * (double)CAS_SSI_RESOLUTION;
		delta = (int32_t)cas_rne(x);
		low = ssi_wrap((word & mask) + delta);
		word = (word & ~mask) | low;
	}

	if (sf->single_frame_delta != 0 && !f->single_done[s]) {
		/* FR-31 single-frame perturbation: one frame only. */
		int32_t low = ssi_wrap((word & mask) + sf->single_frame_delta);

		word = (word & ~mask) | low;
		f->single_done[s] = true;
	}

	if (sf->pfb) {
		word |= (int32_t)1 << CAS_SSI_PFB_SHIFT;
	}
	if (sf->fail) {
		st->ssi_fail[s] = 1u; /* HAL drives the stream high-Z */
	}

	*word_ptr = word;
}

void cas_fault_reset(cas_fault_t *f)
{
	memset(f, 0, sizeof(*f)); /* enable=false everywhere (FR-34) */
}

void cas_fault_apply(cas_fault_t *f, cas_state_t *st)
{
	/* Frame / revolution bookkeeping (FR-33 counters). The frame
	 * counter increments per sample: one SSI word per cas_step(). */
	f->ssi_frame_count++;
	if (f->have_prev) {
		double drev = floor(st->theta / CAS_REV_DEG) -
			      floor(f->prev_theta / CAS_REV_DEG);

		if (drev > 0.0) {
			f->rev_count += (uint32_t)drev;
		} else if (drev < 0.0) {
			uint32_t back = (uint32_t)(-drev);

			f->rev_count =
				(f->rev_count > back) ? f->rev_count - back : 0;
		}
	} else {
		f->have_prev = true;
	}

	/* Trigger evaluation must see the PREVIOUS sample's angle in
	 * f->prev_theta (AT_ANGLE crossing test) -- prev_theta is only
	 * overwritten after the triggers ran. */
	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		cond_eval(&f->dig[ch].cond, f, st->theta);
	}
	for (int s = 0; s < CAS_N_SSI; s++) {
		cond_eval(&f->ssi[s].cond, f, st->theta);
	}
	f->prev_theta = st->theta;

	for (int ch = 0; ch < CAS_N_DIG; ch++) {
		if (f->dig[ch].cond.active) {
			/* FR-30: OFF = high-Z (HR-02), line reads 0. */
			st->dig[ch] = 0u;
			st->oe[ch] = 0u;
		}
	}
	for (int s = 0; s < CAS_N_SSI; s++) {
		ssi_fault_apply(&f->ssi[s], f, st, s);
	}
}
