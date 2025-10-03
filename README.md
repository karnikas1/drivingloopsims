# drivingloopsims
The intent behind this repo is sim different driving loop algorithms for a FSAE EV car.

Here’s my quick list of what the driving-loop sims cover (in my words):
	• Pedal → torque map with speed-based ceiling + curve so tip-in feels sane.
	• Two OPD flavors:
A) Linear = fixed regen vs lift (simple, strongest).
B) Adaptive = target decel that fades at low speed (smoother).
	• Regen gate w/ hysteresis: no regen above X% throttle; comes back when I fully lift.
	• Paddles: tweak regen strength and early-lift feel on the fly.
	• Limit stack: min(P/ω, DC V·I/ω, μFzR) then multiply soft derates (temp/SOC/track).
	• Temp derate block: filter → ramp → latch → hard cut, with slow recovery so no torque ripple.
	• Slip guard: kills regen if slip goes past a threshold.
	• Simple plant: axle torque → accel with drag + rolling, good enough to compare ideas.
	• Energy tally: integrates recovered kJ so I can A/B tunes.
	• Plots/diags: speed/torque/energy, limiter “what’s binding,” and pedal-map speed slices.
Why: lets me tune pedal feel + safety limits in software before touching hardware.
