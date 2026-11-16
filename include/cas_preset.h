/*
 * Preset sensor/flywheel configurations, all angles in milli-degree
 * (1 deg = 1000 mDeg, 1 rev = 360000 mDeg) -- the same unit and field
 * names as the X1+ CA Generator GUI and the web dashboard PRESETS
 * (doc/ca sensor sim/configs/x1plus.json).
 *
 * These are parameters, not constants: the firmware fills a cas_cfg_t
 * from whichever preset (or a user-provided one) and passes it to
 * cas_init(); re-initializing with a different cfg switches the
 * simulated engine. Channel order = NI 9474 wiring:
 * CH00 TDC1, CH01 REF1, CH02 A, CH03 B, CH04 REF2, CH05 C, CH06 D, CH07 TDC2.
 */
#ifndef CAS_PRESET_H
#define CAS_PRESET_H

#include "cas_core.h"

/*
 * X1+ test-rig CA Generator GUI preset (gui_config.py gui_to_spec on
 * configs/x1plus.json): REF1/REF2 pulse center 180.0 deg (width 1.0),
 * TDC1/TDC2 center 0.2 deg (width 1.0), tooth width 0.6 deg,
 * B/D offset +300 mDeg against A/C. This is the bit-exact golden
 * baseline configuration (HANDOFF.md 6).
 */
static inline void cas_preset_x1plus(cas_cfg_t *cfg)
{
	*cfg = (cas_cfg_t){
		.teeth = 186,
		.tooth0_mdeg = 0,
		.ref1_mark_mdeg = 0,
		.ref2_mark_mdeg = 0,
		.tdc_mark_mdeg = 0,
		.pos_mdeg = {200,    /* CH00 TDC1  (center 0.2 deg)  */
			     180000, /* CH01 REF1  (center 180 deg)  */
			     180000, /* CH02 A     (offset 0 vs REF1)*/
			     180300, /* CH03 B     (offset +300)     */
			     180000, /* CH04 REF2  (center 180 deg)  */
			     180000, /* CH05 C     (offset 0 vs REF2)*/
			     180300, /* CH06 D     (offset +300)     */
			     200},   /* CH07 TDC2  (center 0.2 deg)  */
		.width_mdeg = {1000, 1000, 600, 600, 1000, 600, 600, 1000},
	};
}

/*
 * casim SensorSpec defaults: 100-tooth flywheel, A/B and C/D mounted
 * 0.9 / 2.7 deg, REF sensors at 0 deg, all marks at 0 deg.
 */
static inline void cas_preset_default100(cas_cfg_t *cfg)
{
	*cfg = (cas_cfg_t){
		.teeth = 100,
		.pos_mdeg = {0,    /* CH00 TDC1 */
			     0,    /* CH01 REF1 */
			     900,  /* CH02 A    */
			     2700, /* CH03 B    */
			     0,    /* CH04 REF2 */
			     900,  /* CH05 C    */
			     2700, /* CH06 D    */
			     0},   /* CH07 TDC2 */
		.width_mdeg = {500, 600, 1800, 1800, 600, 1800, 1800, 500},
	};
}

#endif /* CAS_PRESET_H */
