# Changelog

## 0.4.0
- **The allocator weighs the lateral residual in the velocity frame** (`SlipFrameAlloc`): the error about the velocity normal (dp sin a - dr cos a) is the sideslip acceleration itself, the error about the velocity vector is stability-axis roll. The roll weight becomes the velocity-axis weight and the yaw weight the velocity-normal one; the body-frame high-AoA roll fade is bypassed while it is on. Identical below ~10 deg AoA, swapped by 70. Bench: full-aft reversal at 320 km/h 4.1 -> 1.8 deg of sideslip, the 65 deg stick-release cases 5.4/5.7 -> 1.8/1.8, steady rolls below 45 deg unchanged.
- **The lateral governor's rate cap is a forward model of the reversal** (`RevSim`): trajectory, rate loops, a least-squares allocation over every effector, the probe's stiffness and damping, beta-dot kinematics, bisected for the largest rate that stays inside the margin. Replaces the kinematic formula over a hand-picked yaw-effector set (which left out the trailing edge - the dominant yaw axis above ~45 deg on this jet). 22 us/tick.
- **`RollStopSideslip` 4 -> 8**, once (`GovernorRevision`; a hand-set value is kept).
- Yaw-damping credit from the probe (`slipDamp`, `dampCredit`), `PaceStiffCredit`, `SweepMarginGate` (were to be 0.3.1, never released).
- Bench: falling leaf at 70 deg AoA 82 deg/s at 2.5 deg of sideslip (0.2.22: 71 at 11.3; 0.3.0: 41 at 3.0). Corkscrews 62-71 (0.2.22: 72-81, 0.3.0: 34-51).
- Bench-only. Not flown.

## 0.2.22
- **Knee-preserving elevon stall stagger.** The stagger now moves only the plateau above the wing foil's 17.2° knee; below it the curve is unchanged and the peak CL (1.10) is kept. 0.2.20–0.2.21 compressed the whole curve, which dragged the knee down to 12–14° and cost the inboard pair roll authority at 10–15° plus some lift.
- **Inboard reversal 37 → 26°** (`InnerElevonReversalAoA`, migrated once from exactly 37). Outboard stays 45°. Model (both pairs + camber, measured arms): worst-case velocity-vector roll authority over 10–55° AoA 0.26 / 0.18 (small / full deflection) vs 0.15 / 0.07 in 0.2.21 and 0.03 / 0.01 unstaggered; unchanged at 5–15°.
- `ElevonStaggerLiftSlope` retired (the peak CL is always kept). Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.21
- **Inboard elevon reversal 32 → 37°** (`InnerElevonReversalAoA` 37; a dev file holding exactly 32 is moved once). Outboard stays 45. Reversal: inboard ±37.2 (±36.0 at full deflection), outboard ±44.8 (±40.7 at full). Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.20
- **Staggered elevon stall:** the inboard elevons now reverse for velocity-vector roll at ±32° AoA and the outboard at ±45°, instead of both at ±45° together (`ElevonStallStagger`, `InnerElevonReversalAoA` 32, `OuterElevonReversalAoA` 45, `ElevonStaggerLiftSlope` 1). Each pair flies its own copy of the wing foil with the peak moved (game aero and FCS models alike); the inboard pair's camber share on wing2 is now a force on the inboard curve so it reverses with the elevon. Combined elevon roll authority at 45° AoA goes from 0.016 to 0.059 (×3.7, small-deflection model); the low point anywhere 25–50° goes from 0.016 to 0.059. Takes effect on the next spawn.
- Compiled only. Not flown.

## 0.2.19
- **Canard travel +60/−45°:** leading edge down 70 → 60° (`StabilatorRangeTEUp` 60; a dev file holding exactly 70 is moved once). On the runway full push and the rollout brake each go to the 60° stop.
- Compiled only. Not flown.

## 0.2.18
- **Runway canards reach their stops:** full push takes them to their full leading-edge-down travel (70°), full pull to full leading-edge-up (45°), instead of stick × the stock pitch range (~35°). The rollout aero-brake alone also takes them all the way down (`GroundBrakeCanard` 45 → 80, capped by the 70° stop; migrated once). Before, only push + brake together reached the stop.
- The canards' +70/−45 travel now applies from a standstill (geometry sign), not only after 10 m/s of airflow.
- Compiled only. Not flown.

## 0.2.17
- **Canard travel +70/−45°:** leading edge down 80 → 70° (`StabilatorRangeTEUp` 70; a dev file holding 0.2.16's 80 / 45 is moved once). Leading edge up stays 45°.
- Compiled only. Not flown.

## 0.2.16
- **Canard travel the right way round:** +80/−45 — leading edge **down** to 80°, leading edge up back to 45° (`StabilatorRangeTEUp` 80, `StabilatorRangeTEDown` 45; dev files holding 0.2.14/0.2.15's 45 / 80 are swapped once). 0.2.14 had extended the leading-edge-up side. Sign checked from flight data: at +45..70° AoA a +40° canard sits at ~+8° local AoA, at −45..−56° a −40° canard at ~−14°.
- Compiled only. Not flown.

## 0.2.15
- **Push + roll sideslip governor:** looks ahead as far as the lateral loops take to answer (0.33–0.47 s, the same lag model as the sideslip-hold gain) instead of a fixed 0.8 s (`SlipGovLeadFromLoops`, on). In the 30 Oct 23:45 log (0.2.14) the sideslip crossing zero at +11 deg/s on its way back from −3.3° read as +9° 0.8 s out and cut the roll target 79 → 46 deg/s with the yaw nozzle and rudder on their stops; the release 0.35 s later re-built it 46 → 85 and the yaw axis went round again. New telemetry columns `latLag`, `slipLead`.
- Compiled only. Not benched, not flown.

## 0.2.14
- **Push + roll (negative corkscrew) entry:** the sideslip-hold gain now comes from what the rate loops carrying the correction can follow (`SlipGainFromLoops`, on): 1.3 at 0° AoA, ~1.85 at −50°, ~2.3 at 70°, instead of the fixed ramp to 4. In the 30 Oct 22:48 log the fixed 4 through the 0.4 s yaw loop ran the sideslip at ~1 Hz (−3.4 ↔ +3.6°), the yaw rate ±12 deg/s about its command and the roll governor cut the roll twice — the hitch. The ramp is still there under `SlipGainFromLoops = false`. New dev key `ActuatorLagT` (0.07 s).
- **Canards:** travel −80/+45° (leading edge up to 80°; `StabilatorRangeTEDown` 45 → 80, migrated once) — the wrong side, corrected in 0.2.16.
- Compiled only. Not benched, not flown.

## 0.2.10
- In-flight brake surfaces close with AoA the way the F-22E's speedbrake does. They are fully out to 15° |AoA|, close linearly until they are stowed at 35°, and reopen as the AoA comes back. Dev key `TrailingEdgeBrakeAoAFade`, on by default.

## 0.2.9
- **In-flight brake surfaces:** move as fast as the actuators allow. The outboard elevon share is solved every frame for zero added lift, with the canards' re-trim counted, so deploying and retracting doesn't upset pitch, roll or yaw. Bench, 400–1200 km/h: nz 0.97–1.11 while deploying, pitch rate under 0.2 deg/s, sideslip 0°.
- **Take-off:** the nose wheel now lifts 0.14–0.2 s after a pull instead of 0.7–0.85 s (`GroundRotationRamp` 60 → 600, migrated once). A short pull no longer fails to rotate the jet.

## 0.2.8
- **`TrailingEdgeBrakeConfig`** (player config, on by default): F-22E-style brake surface sets.
  - **In the air:** inboard elevons down, outboard up.
  - **On the runway:** every trailing edge up, canards leading edge fully down.
  - **Braked touchdown:** the nose is lowered at 4 deg/s, then the full rollout set comes in.
- Needs `ThrottleIndependentAirbrake` on.

## 0.2.7
- **No-TVC rolls at the AoA cap:** smoother. Roll weight 1, and the canards are kept in their attached-flow range.
- **Canards and negative AoA:** canard travel ±45°, negative AoA limit 55°.
- **Fin lost:** 30° AoA limit without nozzle yaw. With TVC, the nozzle keeps vectoring above the game's 250 m/s cut-off. The pedal's sideslip is limited.
- **MPO:** the stick is velocity-vector roll and the pedals are pure body yaw, at every AoA.

## 0.2.6
- **ThrottleIndependentAirbrake off:** the airbrake is fully stock, so the brake key no longer opens it. This fixes Brake and Countermeasures sharing one controller button.

## 0.2.5
- **FcsApi v4:** HUD and visualizer auto-detection.
- **No TVC:** faster roll rate; AoA cap 50° on afterburner.

## 0.2.4
- **Corkscrew:** smoother yaw-rate curve.
- **Aileron visuals:** drawn about the measured hinge line.

## 0.2.3
- Aileron hinge-line visuals (render only).

## 0.2.2
- `MaxYawRate` 90.
- Yaw-nozzle hold gated on rudder headroom.

## 0.2.1
- Ground elevon pitch.
- Yaw-TVC sideslip gain ramp.

## 0.2.0
- AoA limit 70°.
- Inboard elevons as hinge effectors.
- `ElevonPitch` on.

## 0.1.0
- First release. A fork of KR-67 Ifrit FCS 0.5.4, retargeted to the FS-12 Revoker.
