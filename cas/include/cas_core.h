/*
 * cas_core -- virtual crank-angle (CA) sensor signal generator core.
 *
 * Bit-exact C port of the casim Python golden model
 * (platform workspace: doc/ca sensor sim/casim/):
 *
 *   generator.py  virtual flywheel: trapezoidal integration of rpm
 *   sensor.py     pickup square waves (WinGD "CA Generator Model")
 *   ssi.py        SSI absolute-angle word (12 bit + PFB)
 *   profile.py    speed profiles (const / ramp / ramp-cycle / fluct)
 *
 * Bit-exactness rules (HANDOFF.md 7.3, verified by tools/precheck_c_port.py):
 *   1. t = (double)k * dt            (never t += dt accumulation)
 *   2. cum += ((0.5*(rpm_prev+rpm_new))*6.0)*dt;  theta = start + cum;
 *   3. center[ch] = fmod_pos(pos_mdeg/1000.0 - mark_mdeg/1000.0, period)
 *   4. rel = fmod_pos((theta - center) + 0.5*width, period);  level = rel < width
 *   5. SSI: a = fmod_pos(theta + off, 360.0); c = rne((a/360.0)*4096.0);
 *          word = (int32)fmod(c, 4096.0) | (pfb ? 1<<12 : 0)
 *   6. fmod_pos / rne = Python float % / np.round (banker's) semantics
 *
 * The core is platform-independent (no MCU/Zephyr headers): the same
 * source runs on the target and as the host golden test (test/test_golden.c).
 * Keep expressions free of multiply-add chains in one statement so no
 * FMA contraction can reorder the rounding.
 *
 * Channel order = NI 9474 wiring: 0=TDC1 1=REF1 2=A 3=B 4=REF2 5=C 6=D 7=TDC2
 */
#ifndef CAS_CORE_H
#define CAS_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAS_N_DIG 8 /* digital pickup channels (CH00..CH07) */
#define CAS_N_SSI 2 /* redundant SSI streams (SSI1/SSI2) */

#define CAS_SSI_RESOLUTION 4096 /* 12 bit single-turn */
#define CAS_SSI_PFB_SHIFT  12   /* bit12 = power fail bit */

/* Total angle per revolution, deg. */
#define CAS_REV_DEG 360.0

/*
 * Sensor/flywheel geometry. All angles are milli-degree (1 deg = 1000 mDeg,
 * 0..360000 per rev) exactly like the X1+ GUI and web PRESETS.
 *
 * Single flywheel, single gear: A/B/C/D share `teeth` and `tooth0`;
 * TDC1/TDC2 are two redundant sensors reading the one `tdc_mark`;
 * REF1/REF2 each read their own mark, 1 pulse per rev.
 */
typedef struct {
	uint16_t teeth;       /* gear teeth count (A/B/C/D shared) */
	uint32_t tooth0_mdeg;     /* center of tooth #0 on the flywheel */
	uint32_t ref1_mark_mdeg;  /* REF1 mark position on the flywheel */
	uint32_t ref2_mark_mdeg;  /* REF2 mark position on the flywheel */
	uint32_t tdc_mark_mdeg;   /* TDC mark position on the flywheel (unique) */
	uint32_t pos_mdeg[CAS_N_DIG];   /* sensor mounting angle, CH00..CH07 */
	uint32_t width_mdeg[CAS_N_DIG]; /* high-level width, CH00..CH07 */
} cas_cfg_t;

/* Speed profile types (casim/profile.py). */
typedef enum {
	CAS_PROF_CONST = 0,     /* constant rpm */
	CAS_PROF_RAMP,          /* start_rpm -> target_rpm at accel_rpm rpm/s */
	CAS_PROF_RAMP_CYCLE,    /* target_rpm <-> ret_rpm sawtooth cycle */
	CAS_PROF_FLUCT,         /* rpm + amp_rpm*sin(2*pi*freq_hz*t) */
} cas_prof_type_t;

typedef struct {
	cas_prof_type_t type;
	double rpm;        /* CONST / FLUCT base speed [rpm] */
	double start_rpm;  /* RAMP / RAMP_CYCLE */
	double target_rpm; /* RAMP / RAMP_CYCLE */
	double accel_rpm;  /* RAMP / RAMP_CYCLE [rpm/s] */
	double ret_rpm;    /* RAMP_CYCLE return speed [rpm] */
	double amp_rpm;    /* FLUCT amplitude [rpm] */
	double freq_hz;    /* FLUCT frequency [Hz] */
} cas_profile_t;

/* SSI encoder configuration (fault injection extends this later). */
typedef struct {
	uint32_t ssi1_off_mdeg; /* SSI1 angle offset (fault simulation) */
	uint32_t ssi2_off_mdeg; /* SSI2 angle offset (fault simulation) */
	bool pfb1;              /* SSI1 power-fail bit */
	bool pfb2;              /* SSI2 power-fail bit */
} cas_ssi_cfg_t;

/*
 * Simulator state. The struct is public so callers can embed it, but
 * everything below the private marker is managed by cas_init()/cas_step().
 */
typedef struct {
	/* -- current sample (valid after cas_init / each cas_step) -- */
	uint64_t k;        /* sample index, t = k * dt */
	double t;          /* sample time [s] */
	double theta;      /* virtual flywheel angle [deg], unwrapped */
	double rpm;        /* profile speed at t [rpm] */
	uint8_t dig[CAS_N_DIG];  /* digital levels 0/1, CH00..CH07 */
	uint8_t oe[CAS_N_DIG];   /* output enable (1 = driven, 0 = high-Z) */
	int32_t ssi1;      /* SSI1 word: 12 bit angle | pfb<<12 */
	int32_t ssi2;      /* SSI2 word: 12 bit angle | pfb<<12 */

	/* -- private: precomputed geometry / integration state -- */
	double start;      /* start angle [deg] (fixed 0.0 for now) */
	double cum;        /* trapezoidal integral of rpm*6*dt [deg] */
	double dt;         /* sample period [s] */
	double rpm_prev;   /* profile speed at sample k-1 */
	double center[CAS_N_DIG]; /* pulse center [deg] (mod period) */
	double period[CAS_N_DIG]; /* pulse period [deg] (tooth P or 360) */
	double width[CAS_N_DIG];  /* pulse width [deg] */
	double ssi_off[CAS_N_SSI]; /* SSI angle offsets [deg] */
	bool pfb[CAS_N_SSI];
	cas_profile_t prof; /* profile copy (cas_step() is profile-less) */
} cas_state_t;

/*
 * Initialize the simulator and compute sample k=0 (t=0).
 * `dt` is the sample period in seconds (e.g. 20e-6).
 */
void cas_init(cas_state_t *st, const cas_cfg_t *cfg,
	      const cas_profile_t *prof, const cas_ssi_cfg_t *ssi,
	      double dt);

/* Advance one sample: recompute t/theta/rpm and the output words. */
void cas_step(cas_state_t *st);

/* Speed profile value at time t [s] (casim/profile.py speed()). */
double cas_profile_rpm(const cas_profile_t *prof, double t);

/* Python float % m: fmod with the sign of the divisor (result in [0, m)). */
double cas_fmod_pos(double x, double m);

/* np.round semantics: round-half-to-even (banker's rounding). */
double cas_rne(double x);

#ifdef __cplusplus
}
#endif

#endif /* CAS_CORE_H */
