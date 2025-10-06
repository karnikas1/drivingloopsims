# drivingloopsims
The intent behind this repo is sim different driving loop algorithms for a FSAE EV car.

The primary objective behind modeling the driving loop algorithm is to anticipate control behaviors before hardware testing, ensuring safety and performance. Key goals include:
1.	Decide which parameters should be tunable and evaluate their influence on vehicle behavior.
2.	Compare different pedal maps (linear vs curved).
3.	Investigate derating strategies (stacking vs multiplicative).
4.	Compare linear vs adaptive one-pedal driving (OPD) algorithms.
5.	Ensure regenerative braking is suppressed below a set throttle threshold.
6.	Formulate methods for dynamic torque limiting (power, current, thermal).
7.	Introduce hysteresis to prevent oscillatory torque recovery.
8.	Incorporate state-of-charge (SOC) into torque/regen computation.
9.	Implement traction control strategies and compare P-controller vs fuzzy logic.
10.	Determine the required number of driver inputs (pedal, regen paddles, buttons) for practical implementation.

