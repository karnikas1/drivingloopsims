# drivingloopsims
The intent behind this repo is sim different driving loop algorithms for a FSAE EV car.

What is going on here? 
- Pedal and torque map with speed based limits
- One Pedal Drive: Linear (fixed regen v lift) and Adpative (target decel that fades at low speeds)
- Regen gate with hysteresis: no regen above X% throttle, can come back when you lift foot
- Paddles should tweak regen, early lift to feel it on da fly
- Limit Stacks (i'll let u read that for urself)
- Temp Derate whihc filters, latches, and cuts values. slow recovery so we don't have that torque ripple
- Slip Guard to kill regen if we slippin
- Plant is torque then accel with drag then roll
- Energy Tally for recovery
- Plot stuff 
