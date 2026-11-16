/*
 * cas_fault -- sensor fault injection on top of the cas_core outputs.
 *
 * Implements HANDOFF.md 7.4 / requirements FR-30..FR-34 (FR-32 missing
 * teeth is phase 2 and not implemented here):
 *
 *   FR-30 digital channel break: any channel can open in real time,
 *       OFF = high impedance (oe[ch] = 0, dig[ch] = 0). The HAL drives
 *       the pin as input while oe = 0.
 *   FR-31 SSI faults, combinable: PFB bit set; permanent / temporary
 *       angle offset; angle hold (frozen word); single-frame
 *       perturbation; total / temporary failure (high-Z, reported via
 *       st->ssi_fail[]).
 *   FR-33 trigger conditions: immediate / at a CA angle / every n-th
 *       revolution / every n-th SSI frame.
 *   FR-34 power-up default: everything disabled; with all faults
 *       disabled the output is bit-identical to the fault-free run.
 *
 * Effect order per HANDOFF 7.4: angle offset / hold first, then the
 * single-frame perturbation, then the PFB bit; `fail` only raises the
 * high-Z status flag.
 *
 * Call protocol: cas_fault_reset() once, then cas_fault_apply() after
 * cas_init() and after every cas_step(). The trigger state machine
 * observes the engine angle through those calls, so apply() must be
 * called every step.
 *
 * Trigger semantics (fixed "active-and-stay" per HANDOFF 7.4):
 *   IMMEDIATE     active from the first apply() while enable = true.
 *   AT_ANGLE      latches when the flywheel angle crosses angle_mdeg
 *                 (mod 360) between two samples, forward or reverse.
 *   EVERY_N_REV   latches when rev_count reaches a positive multiple
 *                 of every_n (revs counted on theta wraps).
 *   EVERY_N_SSI   latches when ssi_frame_count reaches a positive
 *                 multiple of every_n (one frame per sample).
 *
 * Platform-independent: no MCU/Zephyr headers (host-tested with the
 * same sources as cas_core).
 */
#ifndef CAS_FAULT_H
#define CAS_FAULT_H

#include <stdbool.h>
#include <stdint.h>

#include "cas_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	CAS_FT_IMMEDIATE = 0, /* active as soon as enable */
	CAS_FT_AT_ANGLE,      /* latch at angle_mdeg (CA cross) */
	CAS_FT_EVERY_N_REV,   /* latch at rev_count multiple of every_n */
	CAS_FT_EVERY_N_SSI,   /* latch at frame count multiple of every_n */
} cas_fault_trigger_t;

typedef struct {
	bool enable;               /* FR-34: gate, default false */
	cas_fault_trigger_t trigger;
	uint32_t angle_mdeg;       /* AT_ANGLE trigger angle */
	uint32_t every_n;          /* EVERY_N_REV / EVERY_N_SSI */
	bool active;               /* internal latched state (readable) */
} cas_fault_cond_t;

/* FR-30: digital channel break -> dig = 0 and oe = 0 (high-Z). */
typedef struct {
	cas_fault_cond_t cond;
} cas_dig_fault_t;

/* FR-31: SSI fault set (effects combinable while cond is active). */
typedef struct {
	cas_fault_cond_t cond;
	bool pfb;                   /* set the PFB bit */
	int32_t offset_mdeg;        /* angle offset (count domain, mod 4096) */
	bool hold;                  /* freeze the word at the activation frame */
	int32_t single_frame_delta; /* one-frame perturbation [counts] */
	bool fail;                  /* total/temporary failure: high-Z flag */
} cas_ssi_fault_t;

typedef struct {
	cas_dig_fault_t dig[CAS_N_DIG];
	cas_ssi_fault_t ssi[CAS_N_SSI];
	uint32_t rev_count;       /* completed revolutions (theta wraps) */
	uint32_t ssi_frame_count; /* frames seen (= apply calls) */

	/* -- internal state, managed by cas_fault_apply() -- */
	bool have_prev;           /* prev_theta valid */
	double prev_theta;        /* engine angle at the previous sample */
	int32_t held_word[CAS_N_SSI];   /* hold: captured word */
	bool hold_captured[CAS_N_SSI];  /* hold: captured */
	bool single_done[CAS_N_SSI];    /* single-frame: consumed */
} cas_fault_t;

/* All-off power-up state (FR-34): every fault disabled and cleared. */
void cas_fault_reset(cas_fault_t *f);

/*
 * Apply the fault set to one sample, after cas_init()/cas_step() wrote
 * the fault-free outputs. Must be called once per sample, every step.
 */
void cas_fault_apply(cas_fault_t *f, cas_state_t *st);

#ifdef __cplusplus
}
#endif

#endif /* CAS_FAULT_H */
