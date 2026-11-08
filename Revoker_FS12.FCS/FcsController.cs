using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal sealed class FcsController
	{
		// StickCancelsLimit (ported from F-22E FCS 2.44.0, Tom 4 Nov 20:26): "if i hold stick all the way back at max aoa, and the jet still can
		// do positive pitch rate, then nothing happens; as soon as the only pitch rate that can hold that aoa is 0, that is the point the stick
		// can cancel the protection". At the AoA limit the nose-down part of the limit's own command (the capture bound below the limit,
		// protection above it) scales with 1 - stick / 0.9, as protection's push always did; a positive holding rate is never scaled. Before,
		// below limit + 0.5 the aft-stick floor held the command at 0 and above it 1.9.5's 'pilot command' (the stick clamped by the capture
		// bound) brought the bound's nose-down back through full aft: slowing with the flight path falling, the command cycled 0 <-> ~-12
		// deg/s at ~2 Hz (F-22E 20:07 log). 1.9.5's rule keeps its purpose: a stick asking for more nose-down than protection still gets it.
		internal static bool StickCancelsLimit = true;
		private bool capAtLimU, capAtLimD;

		public readonly Aircraft ac;

		public readonly ControlInputs inputs;

		public readonly List<SurfaceEffector> surfaces = new List<SurfaceEffector>();

		public readonly TvcEffector tvc;

		public readonly FlapSystem flaps;

		private readonly List<Effector> effs = new List<Effector>();

		private readonly Allocator alloc = new Allocator();

		private readonly List<LandingGear> gears = new List<LandingGear>();

		private readonly List<Rigidbody> bodies = new List<Rigidbody>();

		private readonly List<AeroPart> aeroParts = new List<AeroPart>();

		private readonly Dictionary<AeroPart, AirfoilTable> foils = new Dictionary<AeroPart, AirfoilTable>();

		public bool Engaged;

		public bool Failed;

		public FcsMode Mode;

		private float mass = 20000f;

		private Vector3 cg;

		private readonly float[,] Ia = new float[3, 3];

		private readonly float[,] IaInv = new float[3, 3];

		private float massTimer = 99f;

		private float nAlphaTimer = 99f;

		private Vector3 omega;

		private Vector3 omegaPrev;

		private Lpf3 omegaDotF;

		private Lpf3 mNowF;

		private Lpf3 accF;

		// F-22E 2.30.0 port: air data at the CG. vCg = the mass-weighted velocity of every body (the same weighting as cg); the AoA,
		// sideslip, TAS / IAS and the flight-path rates (omegaV, vDot) are read from it instead of the root rigidbody's velocity.
		// nz stays the root body's. The root sits ahead of the CG on these jets (Ifrit ~0.62 m, Revoker ~0.95 m), so a yaw / pitch
		// rate put a false sideslip / AoA of rate x arm / V on every read.
		// Revoker 0.5.1: off by default. Bench (CG-to-CG comparison, the root sits 0.93 m BEHIND the CG on this jet): 2 of 20
		// regression cases worse by more than the ~0.5 deg noise, both 65 deg AoA releases (peak sideslip 1.7 -> 2.8 and 1.1 -> 1.8
		// deg), and 5 of 32 damage-suite cases. Logged both ways (alphaRoot/betaRoot, alphaCg/betaCg) for the flight to decide.
		internal static bool AirDataAtCg = false;

		internal static bool FlightPathAtCg = false;

		public float alphaRoot, betaRoot, alphaCgDbg, betaCgDbg;

		private Vector3 vCg;

		private Vector3 vCgPrev;

		private Lpf3 accCgF;

		// F-22E 3.0.0 capacity estimator, stock-jet port (every effector grouped, sideslip-axis read): telemetry only
		internal readonly CapacityEstimator capEst = new CapacityEstimator();

		private Vector3 velPrev;

		private bool havePrev;

		private Lpf alphaDotF;

		private Lpf aUpF;

		private Lpf aDnF;

		private Lpf qTdotF;

		private float alphaPrev;

		public float alphaU;

		public bool protHold;

		private float protHoldSide;

		public float alpha;

		public float beta;

		public float tas;

		public float ias;

		public float nz;

		public float omegaV;

		public float eLy;

		public float phi;

		public float theta;

		public float alphaDot;

		public float nAlpha = 0.1f;

		public float tw;

		public float nMaxAero = 9f;

		// 1.9.0: wing lift-to-weight ratio at the lift peak, thrust not counted
		public float nLiftMax = 9f;

		// 1.9.0 command shaping (AoA/G pre-filter)
		public float alphaCmdRaw;

		public float nCmdRaw = 1f;

		private float aRefV;

		private float nRefV;

		private bool shapeInit;

		// 1.9.0 AoA -> pitch-rate hand-over
		public float rateBlendAoA = 36f;

		// 1.9.7 negative side (magnitude), which side the rate command / floating hold belongs to, wing negative-lift capacity
		public float rateBlendAoANeg = 36f;

		private int hiSide = 1;

		private int floatSide = 1;

		public float nLiftMin = 9f;

		private bool rbInit;

		public bool hiRate;

		public float rateW;

		public float zoneX;

		private bool hiAnchor;

		public bool floatOn;

		public float floatS;

		internal static float FloatKnee = 2f;

		private float hiAS;

		private float hiAQ;

		// 1.9.0 low-thrust (nose-down authority) AoA cap
		public float authCap = 999f;

		public float authCapEff = 999f;

		// 2.0.9: negative AoA limit while the nozzles do not cover pitch (DisableTVC or low thrust), and the test-switch state
		public float authCapNegEff = -999f;

		private bool noTvcOn;

		private string testSwitchKey;

		private bool authInit;

		internal static float AuthCapLead = 1f;

		internal static float ReleaseTauG = 0.3f;

		internal static float AuthCapTauUp = 1f;

		internal static float AuthCapTauDown = 0.3f;

		internal static float AuthCapRise = 4f;

		internal static float AuthCapRiseAtCap = 1f;

		internal static float AuthCapThrustStep = 1f;   // deg/s^2 of nozzle nose-down authority gained between sweeps

		internal static float AuthCapFastHold = 1f;     // s the fast rise stays open after the last such gain

		private float authTvPrev;

		private float authFastT;

		public float alphaAtNMax = 30f;

		public float gAvail = 9f;

		public float gInt;

		public float vDot;

		public float alphaDotHoldG;

		private Lpf vDotF;

		public float aUp = 60f;

		public float aDn = 60f;

		internal static float RollLoadLead = 0.2f;

		// 2.1.10: allocator yaw weight above ~40 deg AoA while sideslip builds (YawAxisPriority below 20 deg, blended between).
		// The sideslip rate is p sin(AoA) - r cos(AoA): at 45 deg a body-roll error counts as much as a yaw-rate error, but
		// the allocator weighted yaw 8 : roll 0.35, so it let the stabs' yaw (their roll side-effect uncancelled, flaperons at
		// their stops) run the body roll 30 deg/s past its command (13 Oct log, 45 deg limit: sideslip 20-28 deg in rolls
		// and reversals at 70-100 m/s). With |sideslip + 0.3 s lead| over 2 deg the yaw weight comes down toward this value
		// (full at 8 deg); <= 0 = 2.1.9 (flat YawAxisPriority).
		internal static float YawPrioHi = 2f;

		// sideslip gate for YawPrioHi: 0 below YawHiSlip0 of |sideslip + lead|, 1 at YawHiSlip0 + YawHiSlipSpan (0 span = always 1)
		internal static float YawHiSlip0 = 2f;

		internal static float YawHiSlipSpan = 6f;

		internal static float YawHiSlipLead = 0.3f;

		internal static float YawHiSat0 = 0.6f;

		internal static float YawHiSatSpan = 0f;


		private float YawHiSlipGate()
		{
			if (YawHiSlipSpan <= 0f)
			{
				return 1f;
			}
			float bl = beta + slipRateDbg * YawHiSlipLead;
			float g = Mathf.Clamp01((Mathf.Abs(bl) - YawHiSlip0) / YawHiSlipSpan);
			if (YawHiSatSpan > 0f)
			{
				g *= 1f - Mathf.Clamp01((yawSat - YawHiSat0) / YawHiSatSpan);
			}
			return g;
		}

		internal static float RollStickRate = 10f;

		internal static float PreLag = 0.01f;

		internal static bool LegacyLag;

		internal static float RollJerkServo = 1f;

		internal static float RollJerkTau = 0.18f;

		internal static bool BrakeUseAll = true;

		internal static float RollEndJerk = 0.35f;



		// 2.1.0 sideslip-rate lead for the lateral guard (see LateralLaw)
		internal static float SlipLeadT = 0.8f;

		internal static float SlipRateTau = 0.05f;

		internal static float GuardCutTau = 0.08f;

		internal static float SlipGovStart = 4f;

		internal static float SlipGovSpan = 10f;

		public float slipGovDbg = 1f;

		private float latLagEff = 0.47f;

		private float latLagYawNom = 0.47f;

		public float slipLeadDbg;

		public static bool GLimitAccelLead = true;

		public float rateCmdG;

		private float stickLagF;

		public bool capOn;

		private float capSp;

		private float capQ;

		private float capSpPrev;

		private float capSteadyT;

		internal static float CapStickRate = 0.15f;

		internal static float CapSteadyTime = 0.15f;

		internal static float CapAoABand = 0.5f;

		internal static float CapAlphaDot = 2f;

		internal static float CapStickBand = 0.02f;

		private bool leadInit;

		public float stickLeadDbg;

		public bool latRevDbg;

		private float latRevSign;

		internal static float NegGateA0 = 0f;

		internal static bool LowCapLatFix = true;

		internal static bool AsymSharedLat = true;

		public float sharedLatDbg = 1f;

		internal static float LoadRevYawK = 0.4f;

		public float loadRevDbg;

		internal static float LowCapCoupBand = 4f;

		internal static bool CoupGateAtLimit = true;

		internal static float CoupLimBand = 4f;

		internal static float CoupGateRelease = 0.6f;

		internal static float CoupGateLoaded = 0f;

		private float coupGateHeld;

		private bool LowThrustCapOn => authFixedOn || authCapEff < Plugin.AoAPos - 0.5f;

		internal static float NegGateSpan = 3f;

		public float negGateDbg;

		internal static float YawAnchorK = 0.5f;

		internal static float YawAnchorT = 0.25f;

		internal static float YawAnchorFloor = 10f;

		internal static bool YawAnchorGrav = true;

		public float yawAnchorDbg = 999f;

		private Lpf bdkF;

		// Ifrit 0.3.5: sideslip predictor history (commanded rS less gravity, washed out) and its frame times
		private readonly float[] slipHistV = new float[256];

		private readonly float[] slipHistDt = new float[256];

		private int slipHistN;

		private int slipHistHead;

		private float slipHistLp;

		public float slipPredDbg;

		public float slipHoldDbg = 1f;

		private void SlipHistPush(float x, float dt)
		{
			float tw = Plugin.SlipPredictWashout;
			slipHistLp = (tw > 0f) ? (slipHistLp + (x - slipHistLp) * (dt / (tw + dt))) : 0f;
			slipHistHead = (slipHistHead + 1) % slipHistV.Length;
			slipHistV[slipHistHead] = x - slipHistLp;
			slipHistDt[slipHistHead] = dt;
			if (slipHistN < slipHistV.Length)
			{
				slipHistN++;
			}
		}

		private float SlipHistSum(float window)
		{
			float sum = 0f;
			float tt = 0f;
			int idx = slipHistHead;
			for (int k = 0; k < slipHistN && tt < window; k++)
			{
				float d = Mathf.Min(slipHistDt[idx], window - tt);
				sum += slipHistV[idx] * d;
				tt += slipHistDt[idx];
				idx = (idx - 1 + slipHistV.Length) % slipHistV.Length;
			}
			return sum;
		}

		private void SlipHistClear()
		{
			slipHistN = 0;
			slipHistLp = 0f;
		}

		public float slipRateDbg;

		// 2.23.0: stays 0.5. The sweep cap is the kinematic demand (a coordinated rotation pS needs pS cos AoA of body yaw per unit
		// AoA, see LateralLaw) against half the yaw capacity; it is the binder of every released push-over from 65 deg once the AoA
		// falls through ~55 deg (3 Nov 11:19 log, 47.5-48.1 s: 15-21 deg/s of AoA rate allowed against 20-23 of flight-path rotation
		// = a pitch command of +4..+9). 1 removes that and doubles the bench's bottom sideslip (clean released push 600: 9.2 -> 21.3;
		// 0.65: 13.3): the larger pitch demand takes the stabs as they return to pitch and the lateral axes lose the coordination.
		internal static float SweepYawK = 0.7f;   // 2.27.0: 0.5 -> 0.7. Since 2.26.0 the kinematic cap acts on the HELD push only (released: scaled out, the follow pace paces). Tom (14:28): "for the held push-over the pitch-down is too slow, the sideslip is being kept too tight" - 14:23 log held pushes 135-201 kt: sideslip <= 2 deg of the 4 deg margin, AoA rate 15-20 deg/s against +12..+23 of flight-path rotation. Bench (its held pushes do not reach this cap): one-fin held 560 5.7 -> 9.3, push+roll 300 2.7 -> 3.2, pull-then-pushover 350 4.2 -> 4.8, push-then-pullthrough 450 7.5 -> 8.3; the 2.23.0 bisect at 1.0 doubled the released push's sideslip when this cap still paced it.
		// 2.24.0: a cap on the sweep toward zero AoA may hold the nose, never turn it back: the pitch-rate command it leaves is bounded
		// at zero (the AoA still falls at the flight-path rate). Where the deficit is real (11:19 log, 223-244 kt with 70-80 deg/s on,
		// sideslip 5-6 deg and climbing) the honest AoA-rate cap of 14-19 deg/s sat under +20..+24 of flight-path rotation = a nose-UP
		// command of +5..+18 with the stick full forward - Tom's hitch, whatever the pace behind it.
		internal static bool SweepNoReverse = true;
		internal static float SweepNoReverseStick = 0.2f;   // lateral stick fraction over which 'released' fades to 'held' (the floor and the delivered term are released-push devices; the held push keeps 2.10.0's accepted pace)
		// 2.25.0: the floor on the held push too (Tom, 3 Nov 13:17: "sometimes it steps, sometimes it goes all the way to the hard stop then
		// goes again, for both the released push-over and the held push-over" - 13:12 log, 122.0 s held push: qCmd -25 -> -13 -> 0 -> +7 ->
		// -9 -> -36, body pitch rate +4 with the stick full forward). A positive pitch command with the stick forward is a hitch whatever
		// the rotation behind it; what the held push loses in the bottom sideslip the shed cap's pace has to buy back.
		internal static bool SweepNoReverseHeld = true;
		// 2.34.0 (the push governor): the push side of the reference-capacity governor replaces the 2.5.8-2.27.0 push-over stack (the kinematic
		// sweep cap and its delivered term, the shed cap and its follow pace, the crossing lead, the cap filters and slews, the released
		// follow latch, its roll target and its yaw brake). Tom: "it limits the lateral rate when only the yaw is the bottleneck and the
		// pitch rate when both yaw and roll are". A sweep toward zero AoA under a velocity roll pS drives the stability-axis yaw rate
		// rS = r cos a - p sin a at pS x AoA-rate; the yaw (through cos a) and the roll (through sin a) turn it back. The yaw sheds at
		// most capY: the velocity roll may come down no faster than (pS cos a AoA-rate - capY) / sin a (and must GROW when that is
		// positive: the roll follows the yaw) - the lateral limit. Where the roll cannot grow that fast either (capX), the AoA rate is
		// limited, and over the whole sweep the AoA may come no lower than atan(yaw left / body-roll ceiling) (released: 1.5 x the body
		// roll at the push; held: the stick's target), closed on at PushGovDecel. Capacities: one 0.2 s filter, TradeCapShare of them for
		// the pitch, all of the yaw for the lateral edge. Bench record: claude/Pushover_governor_2.34_dev_findings.md
		// 2.35.3: the stack it replaced (and the PushGov switch back to it) is deleted; it had not run since 2.34.0.
		internal static float PushGovCapTau = 0.2f;   // s: the governor's one capacity filter (the pace is one curve, not the estimator's tick-to-tick read - 2.25.0)
		internal static float PushGovTau = 0.5f;      // s: the pace closes on the yaw-limited AoA over this time (when PushGovDecel = 0)
		internal static bool PushGovNoStab = true;   // 2.34.1: plan the push on the yaw capacity without the stabs (see LateralLaw)
		internal static float PushGovSlipK = 0.5f;    // x the PushoverSideslip allowance the pace may leave to the bottom
		internal static float PushGovDecel = 40f;     // deg/s^2: the pace closes on a_min at this deceleration (sqrt(2 D gap)); 0 = over PushGovTau
		internal static float PushGovRollK = 1.5f;    // the body roll may rise to this x the body roll it had when the sweep began
		// 2.35.0 (PushPlan, Tom 22:19: "velocity vector roll bleeds while the pitch is smoothly slowed down for the push over"). The push
		// is PLANNED once, when it starts, not re-paced every tick from the capacity read (2.34.0/2.34.1: a pace that follows a read
		// swinging +-30 % in 0.3 s, sitting at the flight path's own 15-20 deg/s of rotation, hesitated). At the push: the yaw on board
		// (less what PushPlanSlip deg of sideslip absorbs at the bottom) over the yaw the push will shed - from the logs, 77 push-overs
		// 14:45-22:06, the mean shed from the push to 10 deg AoA was 28.6e-4 x IAS^2 (r 0.81 with IAS; the capacity reads 0.53-0.64)
		// - gives the push time T; the body pitch rate is set to bring the AoA to 10 deg in T (less the flight path's rotation), ramped
		// in at PushPlanJerk and then held; it may rise (never fall) at PushPlanRise when the yaw goes faster than planned. With the
		// lateral stick released the velocity-vector roll bleeds to zero over the same T (not the stop law's brake). One number to tune
		// the trade: PushPlanK (higher = faster push, more sideslip at the bottom).
		internal static bool PushPlan = true;
		internal static bool PushPlanStopsRoll = true;
		internal static bool RudderReach = true;   // Revoker 0.7.0: the allocator's reach pass on the rudder (see the reachTau loop)   // Revoker 0.7.0: the plan bleeds a held roll as well as a released one
		// Revoker 0.6.0: shed = PushPlanK0 + PushPlanK x IAS^PushPlanExp (deg/s^2, IAS m/s). The F-22E's shed is all aero (K0 0, K 0.028 from
		// 93 push-overs). This jet's yaw at the top of a push is the yaw nozzle (thrust, no fade with speed) and the elevons on the sideslip
		// axis: the 4 Nov 23:03 / 23:13 logs' 4 push-overs shed about the same at every IAS, so the fit is a constant: K0 40, K 0 (residual
		// 3.9 deg/s^2 rms). Provisional: 4 samples, flown on 0.5.1. Refit as more push-overs are flown.
		internal static float PushPlanK0 = 40f;
		internal static float PushPlanK = 0f;
		internal static float PushPlanExp = 1.5f;
		// Revoker 0.6.0: the plan's sideslip allowance at the bottom is the push-over allowance Tom set for the stock jets (PushoverSideslip, 8;
		// the F-22E plan uses 4). -1 = follow it.
		internal static float PushPlanSlip = -1f;
		internal static float PushPlanA0 = 20f;       // deg: the push is planned only from above this AoA
		internal static float PushPlanQMin = 8f;      // deg/s: the nose always visibly moves
		internal static float PushPlanJerk = 120f;    // deg/s^2: the planned rate is ramped in at this
		internal static float PushPlanRise = 15f;     // deg/s^2: how fast it may rise when the yaw is ahead of the plan
		internal static float PushPlanTMin = 0.6f;
		internal static float PushPlanR0 = 15f;       // deg/s of body yaw on board at the push for it to be planned
		internal static float PushPlanWvK = 1f;       // the flight path's rotation at the push counted against the planned AoA rate
		internal static float PushPlanRelease = 60f;  // deg/s^2: the bound ramps out below 10 deg AoA
		internal static bool PushPlanOwnsPitch = true;    // 2.36.0: under a plan the sweep caps (the 60 deg/s AoA-rate ceiling) stand aside; with PushPlanClipKey the allocator's pitch priority and the stabs' window read the plan's bound (2.35.1's try without ClipKey: 4.8 -> 11.2)
		internal static bool PushPlanWinOff = true;
		internal static bool PushPlanClipKey = true;      // 2.36.0: pitchNeedDn / sweepHoldWin read the plan's bound (see the pitch law)
		// 2.40.0 (PushPlanHeld; 2.36.0's held-stick rule): a planned push owns the held lateral stick too, in place of the push governor's
		// lateral band. The band paced the roll toward the stick's target by L = (pS cos a a' - capY) / sin a with capY the measured yaw
		// capacity read, which swings +-30 % in 0.3 s (the 2.34.1 noise). Under a plan the same rule reads the plan's own yaw budget D,
		// fixed at the push's start - D = max(capPlan, (Y0 - Yend) / T): the coordinated yaw of the rotation, Y = pS sin a, drained to its
		// end (the held roll's coordinated yaw at 10 deg AoA) over the planned push time - and this tick's commanded AoA rate a'.
		// Released stick: unchanged (the plan's linear bleed). 2.36.0 flown 09:03: "held push over is smooth".
		internal static bool PushPlanHeld = true;
		// 2.41.0 (PushPlanReachTau): through a planned push the allocator places the nozzles and wing surfaces against where each stab will
		// be within this many seconds (its servo's reach toward its command), not against the command (Allocator.ReachPass). 11:01 log
		// (2.40.0), 34.1 s: the window sent the left stab -25 -> +35 deg (0.45 s, through its lift peak); the allocator, seeing the stab
		// already at +35 (nose-up), drove the nozzles -4 -> -9 deg nose-down while the stab gave its peak nose-down on the way: -34 deg/s
		// flown against -26.8 asked. Hybrid replays, 59 released pushes: overshoot 6.8 -> 2.8 deg/s median, the mid-push step 3.9 -> 1.3,
		// the body-roll rise 9.3 -> 4.8; sideslip 5.0 -> 7.4 median (the stab's transit roll pulse had been holding it down). 0 = off.
		internal static float PushPlanReachTau = 0.1f;
		// 2.44.0 (StickCancelsLimit, Tom 4 Nov 20:26): "if i hold stick all the way back at max aoa, and the jet still can do positive pitch
		// rate, then nothing happens; as soon as the only pitch rate that can hold that aoa is 0, that is the point the stick can cancel the
		// protection". At the AoA limit the nose-down part of the limit's own command (the capture bound below the limit, protection
		// above it) scales with 1 - stick / 0.9, as protection's push always did; a positive holding rate is never scaled. Before, two
		// rules disagreed at the limit: below limit + 0.5 the aft-stick floor held the command at 0, above it 1.9.5's 'pilot command'
		// (the stick clamped by the capture bound) brought the bound's nose-down back through full aft. Slowing at 22-50 IAS with the
		// flight path falling 1-8 deg/s (20:07 log, 143-154 s, 173-182 s) the command cycled 0 <-> -11..-13 deg/s at ~2 Hz, AoA
		// 64.5-66.6, nozzles -13 <-> +16 deg. Now full aft holds zero rate there and the AoA rises with the falling flight path (bench,
		// those segments: 112 / 85 deg at their ends); release and protection pushes. 1.9.5's rule keeps its purpose: a stick asking for
		// more nose-down than protection (forward stick above the limit) still gets it.

		internal static float SweepPs0 = 10f;


		internal static float SweepFloor = 6f;





		internal static bool SweepPosPullFree = true;



		internal static float SweepPosK = 0.25f;

		internal static float SlipGovPs0 = 10f;





		internal static float PullRollTradeFinLoss = 0.7f;









		public float decelDbg;

		private float decelF;

		// Ifrit 0.4.0: split-deceleron yaw tier gate (0..1), engine out / damaged yaw axis only
		private float splitTierF;

		public float splitTierDbg;

		public float tierDemDbg, tierReachDbg, tierSplitDbg;

		internal static float SplitTierRise = 3f;

		internal static float SplitTierFall = 1.5f;

		internal static float SplitYawClose = 0.6f;

		internal static float SplitYawSatOpen = 0.05f;

		internal static float SplitYawSatSpan = 0.6f;

		private float decelYieldF;

		internal static float DecelYieldFall = 1f;

		internal static float DecelFightRelax = 1f;

		internal static float DecelStabRelax = 0f;




		internal static float DecelRise = 3f;

		internal static float DecelFall = 1.5f;

		private float pullGovF = 1f;

		// 2.2.24 adaptive feed-forward in the rolling-pull governor
		private float pullFFBoost = 1f;

		public float pullServoDbg;

		private float pullServoEx;






		public float pullServoWDbg;







		internal static float RelLatRollPrio = 3f;

		public float relLatPrioDbg = 1f;






		public float pullXsDbg;

		public float pullLeadDbg;





		internal static float PullFFBoostIdle = 2f;




		// 2.2.8 governor state
		internal static float PullGovIdleRise = 4f;

		internal static float PullTradeFloor = 30f;

		// 2.4.17 roll-stop sideslip governor (see LateralLaw)
		internal static float StopSlipYawUse = 1f;



		internal static float YawSlewLeadK = 1f;

		public float yawLeadDbg;

		private float rCmdRateF;



		private bool rCmdLeadInit;

		// 2.5.0 reference governor on the commanded roll rate (see LateralLaw). The stop / reversal sideslip model fitted
		// on the 30 Oct flight logs (200 stops and reversals, 3-40 deg AoA): peak sideslip after the stop = StopSlipBase +
		// |pS| sin(AoA) cos(AoA) x (StopSlipTau0 + StopSlipTau1 x rudder load above StopSlipKnee), with the rudder load the share
		// of the rudders' yaw on the roll's side (StopSlipHoldTau filter).
		internal static float StopSlipBase = 2.0f;

		internal static float StopSlipTau0 = 0.054f;

		internal static float StopSlipTau1 = 0.22f;

		internal static float StopSlipKnee = 0.7f;

		// 2.5.2 yaw-reserve governor: rudder load (share of their yaw on the roll's side) above which the roll reference is
		// trimmed, and below which it is released; the release rises at StopSlipReserveRiseK x the fall rate
		internal static float StopSlipLoad = 0.8f;


		internal static float StopSlipReserveRiseK = 0.5f;


		// 2.21.0: the AoA-rise cap (StopSlipPitchK / Start / Lead / Close, StopSlipPinAB0 / 1) is deleted - see LateralLaw. The
		// floor stays as the trade's pitch-share floor (deg/s of AoA rate)
		internal static float StopSlipPitchFloor = 6f;

		private float paceRDotLast;

		// 2.6.4: the trade's split. PullRollTrade is the share of a rolling pull's yaw shortfall the roll gives up (the roll limit
		// goes that far from pSmax toward the trade's target); the pitch gives up the rest - the AoA-rate ask is scaled so that the
		// pair's sweep demand (pS cos AoA x AoA rate) fits the share: with the roll at R_r of pSmax and the demand needing R of it,
		// the AoA rate is held to R / R_r of the ask. 1 = all roll (2.5.1-2.6.3: the remainder was simply uncovered - at 170-210 kt
		// the roll sat at the 30-35 deg/s floor with the pitch at full rate, Tom: "the slower, the more pitch and less roll; not the
		// way the old governor divided pitch and roll with the 0.7 trade"; 22:17 log); 0 = all pitch; your 0.7 = 70 / 30.
		private float pullTradeAsk;
		internal float tradeAlphaRate = -1f;

		// 2.8.0: the trade from the measured capacity (CapacityEstimator) against the roll the stick asks, not the roll limit
		// (15:02 log, 210.3 / 215.0 / 224.1 s: stick released, roll braking to zero, yaw idle, and the pitch still held to 30 deg/s
		// for ~0.35 s while the slewed roll limit climbed back - the re-pull hitch). 2.10.0: the 2.7.x arithmetic is deleted.

		// share of the measured yaw capacity a rolling sweep may spend before the trade cuts in
		internal static float TradeCapShare = 0.7f;

		// the AoA-rate cap the measured trade asks of the pitch this tick (-1: none), and the roll the stick asks before any cap
		internal float tradeAlphaNew = -1f;

		internal float pSAskStick;
		public float tradeAlphaDbg;


		internal static float StopSlipPitchFree = 60f;

		private float stopSlipAbsF;

		internal float yawSatAlphaRate = -1f;

		// 2.5.5: the lateral governors (margin cap, yaw reserve, pace, pitch caps, trade, no-chase) act in the AoA/G and rate laws
		// and, with the gear down, in the air (no wheel on the ground; GearDownSlipProtect as for the 2.4.13 limiters). 10:10 log:
		// 51 s gear-down airborne, 20.9 deg of sideslip, every governor column idle.
		private bool LatGovMode => Mode == FcsMode.AoAG || Mode == FcsMode.Rate || (Mode == FcsMode.GearDownRate && Plugin.GearDownSlipProtect && !gsAnyContact);

		private float yawSatCapQ = 120f;

		private float yawSatCapQLo = -120f;

		internal float yawSatAlphaRateDn = -1f;

		private bool yawSatCapOn;

		public float yawSatPitchDbg;
		private float sweepClipDbg;   // 2.36.0: the push's clip (ask - command) by the caps or the plan (see PushPlanClipKey)
		private bool sweepClipOn;

		// margin schedule: RollStopSideslip up to StopSlipAoALo, RollStopSideslip50 at 50, RollStopSideslip65 at 65, off past StopSlipAoAEnd
		internal static float StopSlipAoALo = 10f;

		internal static float StopSlipAoAEnd = 70f;

		// the limit comes down at StopSlipFallK x (the rudders' braking yaw acceleration / sin AoA), within StopSlipFallMin..Max
		// deg/s per s, so a falling limit is a roll the yaw can coordinate, never a stop of its own
		internal static float StopSlipFallK = 0.35f;

		internal static float StopSlipFallMin = 20f;


		// while the trajectory is still accelerating into the roll (faster than this, deg/s^2) the limit does not come down:
		// the rudders pin in every fast entry and unpin as the yaw catches up

		// yaw-saturation hold (see LateralLaw)
		internal static float StopSlipHoldTau = 0.08f;

		public float stopSlipUDbg;

		public float stopSlipPredDbg;

		public float stopSlipMarginDbg;

		private float rCmdCoordPrev;


		public float noChaseDbg;

		// no-chase and the yaw slew lead fade out over this AoA band (the one-axis high-AoA law owns the yaw there)
		internal static float StopSlipA0 = 36f;

		internal static float StopSlipA1 = 45f;


		internal static float StopSlipRiseK = 1.5f;


		public float stopSlipLimDbg = 999f;

		public float stopSlipABDbg;

		// 2.5.3 yaw-reach (pace) governor, RollGovernorReplaces >= 2 (see LateralLaw, after the roll trajectory step). The lateral
		// pair's body-yaw demand (pS sin AoA + the sideslip correction's cos AoA share) may not get further from the measured yaw
		// rate than the yaw axis can close within PaceHorizon: the yaw acceleration the jet is showing now (PaceAccelTau filter)
		// times the horizon, plus what the rudders can add within it from where they are (their tables, slew-limited, at
		// PaceRudderUse), plus PaceFloor deg/s for the loop's normal tracking error. The roll reference is clamped to that window.
		internal static float PaceHorizon = 0.15f;

		internal static float PaceFloor = 4f;

		internal static float PaceRudderUse = 1f;

		internal static float PaceAccelTau = 0.08f;

		// below this |sin AoA| the roll's yaw share is too small for the window to mean anything
		internal static float PaceSinMin = 0.05f;

		internal static float PaceRollUse = 1f;

		// Revoker 0.3.0: with the yaw and roll windows disjoint, the window of the axis that can move less toward the target
		// stands (true); false = the F-22E 2.6.6 rule (the yaw window stands)
		internal static bool PaceWeakAxis = true;

		// Revoker 0.3.0: the roll window's reach from the roll effectors' tables and slews (true); false = F-22E 2.6.6 (aRollPos x horizon)
		internal static bool PaceRollSlew = true;

		// Revoker 0.3.0: the roll effectors' slew in use opens the pace window (true; telemetry rollGate)
		internal static bool PaceRollGate = true;

		private float[] rollEffPrev;

		private bool rollEffInit;

		private float rollLoadF;

		public float rollLoadDbg;

		// open-loop: share of the rudders' available yaw the trajectory's acceleration / braking may ask for (level 3)
		internal static float PaceYawUse = 0.85f;

		private float paceAccelCap = -1f;

		// 1 = the open-loop cap / the rudder-usage gate fade out over the rudder fade (StopSlipA0..A1); 0 = they stand at every AoA


		// 1 = the open-loop cap hands over to the one-axis law as the rudders fade (rudderAuth); 0 = 2.5.5 (cap at every AoA)

		// false = keep the margin cap and the yaw-reserve governor under the pace governor (2.5.6 and before), for A/B
		// 2.5.15 sideslip-tolerance probe: the airframe's own aero (every AeroPart at the game's per-part law, the surfaces at
		// their present deflection) evaluated at the present AoA and a sweep of hypothetical sideslip, every SlipProbePeriod.
		// Two bounds come out: the sideslip where the restoring yaw moment stops growing (directional stiffness gone - the
		// departure boundary) and the sideslip where the rudders' or the roll surfaces' authority has fallen to SlipAuthFrac of
		// its zero-sideslip value. The smaller, x SlipTolUse, caps the margin. Nothing entered: the tables are the craft's.
		internal static float SlipProbePeriod = 0.5f;
		internal static float SlipProbeStep = 4f;
		internal static float SlipProbeMax = 20f;
		internal static float SlipTolUse = 0.7f;
		internal static float SlipAuthFrac = 0.5f;
		internal static float SlipAuthUse = 0.5f;
		private float slipSignR;
		internal static float SlipTolTau = 0.5f;
		private float slipProbeTimer;
		private float slipTolF = 25f;
		public float slipTolDbg = 25f;
		public float slipDepDbg = 25f;
		public float slipAuthDbg = 25f;
		public float slipStiffDbg;

		// 2.31.2 (telemetry only, drives nothing): the passive airframe, evaluated per part as the game does (AirframeMomentAt).
		// psA*: the airframe's own roll / yaw acceleration now (surfaces where they are, no thrust; deg/s^2, x roll right, z yaw right)
		// psD*: how that changes if the body rates go from (p, r) to this tick's (pCmd, rCmd) - the help (or opposition) the
		//       measured term does not contain yet; cePass*: psD as capacity toward each side, and along the velocity-roll axis
		// psL* / psN*: roll / yaw derivatives at the present state, per deg of sideslip (b) and per deg/s of p and r (0.1 s cadence)
		internal static bool PassiveProbeOn = true;
		internal static float PassiveDerivPeriod = 0.1f;
		public float psAX, psAZ, psDX, psDZ, psLb, psNb, psLp, psNp, psLr, psNr;
		public float cePassXP, cePassXN, cePassZP, cePassZN, cePassSP, cePassSN;
		private float psDerivTimer;
		private List<(ControlSurface cs, AeroPart host, AirfoilTable table)> psCamber;
		private readonly Dictionary<AeroPart, float> psCdMin = new Dictionary<AeroPart, float>();
		// 2.5.8 rate mode (RollPaceRateMode): the lever is the commanded roll rate, not the trajectory's acceleration or braking.
		// The roll-rate target is capped at the rate whose stick reversal, run at the tuned acceleration / braking, the yaw can
		// follow within the sideslip margin: the reversal swings the yaw demand by 2 pS sin AoA; the rudders first cross from
		// where they are (slew) and then pull at their full yaw on the new side; the yaw-rate error that accumulates meanwhile,
		// x cos AoA, is sideslip. PaceRateK scales the predicted sideslip (1 = kinematic, no credit for the airframe's own
		// directional stiffness); PaceRateFloor is the least rate the cap leaves (deg/s).
		internal static float PaceRateK = 1f;
		internal static float PaceRateFloor = 10f;

		// 2.6.9: the rate cap's blend weighted by rudderAuth (its model's validity); false = 2.6.8 (blend at every AoA)


		// 2.6.9: the rate cap models the ramped reversal the law flies (false = the 2.5.8 step model)
		internal static bool PaceRateRamp = true;

		// bench: scale on the cap's rudder-crossing dead time (1 = 2.6.9)
		internal static float RateCapTauK = 1f;

		// 2.6.9: the cap's dead time is the travel to the deflection the reversal needs, not to the far stop
		internal static bool RateCapNeedTravel = true;

		// 2.6.11: the pace window reads the whole airframe's yaw - its usage for the gate, every effector's travel for the reach - at every AoA (false = rudders)

		internal static bool PaceMarginReach = true;

		// bench: false = no open-loop acceleration cap at any blend (the trajectory always runs at its tuned acceleration / braking)
		internal static bool PaceAccelCapOn = true;

		// 2.7.0 measured pace window (PaceMeasured). One governor on the commanded roll rate, from measured state and the
		// allocator's own output - no effector tables beyond the deflections the allocator has actually commanded, no surface
		// list, no rate cap, no blend, no open-loop acceleration cap. Each tick, for the yaw axis and the roll axis:
		//   what the axis will have done by the end of PaceHorizon = its measured acceleration now (everything acting on the
		//   airframe, passive and active) plus the increment still in flight toward the effectors' commands, as far as their
		//   servos get within the horizon (the tables at the commanded deflection: the allocator's decision, not a hypothesis);
		//   the sideslip margin still unused, spent at that acceleration (sqrt(2 a (margin - |beta|) / cos AoA) of yaw-rate lead);
		//   PaceFloor for the loop's ordinary tracking error.
		// The roll target may not ask the axis for more than that - but only while the axis measurably cannot keep up: the
		// allocator reports the target unmet at the commands it gave (allocUnmet), or the commanded increment is further away
		// than the servos reach within the horizon (slew). In a coordinated roll, a corkscrew, the falling leaf, nothing binds
		// (no steady-state tax). Where an axis is out - rudders crossing their travel in a reversal, the stabs busy with the
		// pull - the target is held to what the axis delivers and the trajectory still runs at its tuned acceleration / braking
		// toward it (Tom's rule: the commanded rate is the only lever).

		// deg/s^2 of unmet-plus-undelivered acceleration at which the gate starts (PaceSatDead) and is fully on (PaceSatDead + PaceSatSpan)
		internal static float PaceSatDead = 5f;

		// 2.7.2: the unmet acceleration, as a share of the axis's authority, at which it counts as fully saturated
		internal static float PaceSatFrac = 0.25f;

		internal static float PaceSatSpan = 40f;

		// filter on the allocator's unmet report (s)
		internal static float PaceSatTau = 0.05f;

		// share of the axis's demonstrated acceleration (a decaying peak of |omega-dot|, PaceSeenTau) credited where nothing is
		// yet committed: the margin lead and the release pace after a hold
		internal static float PaceSeenUse = 0.5f;

		internal static float PaceSeenTau = 3f;

		// 2.7.1: lead allowed over the axis as a time at its capacity (s) - the sideslip rate accepted while the roll waits for it
		internal static float PaceLeadT = -1f;   // 2.17.0: < 0 = 1 / KYaw (0.4 s at the default gain); 0.1 = the 2.7.1 value

		private float paceRatePrev;

		private bool paceRateInit;

		// 2.7.0: the trajectory's acceleration law reads the stick's intent (a reversal is a reversal even while the window
		// holds the target on the near side), so a governed reversal brakes at the reversal's tuned acceleration, never the stop's
		internal static bool PaceRevIntent = true;

		public Vector3 allocUnmet;

		private Lpf3 allocUnmetF;

		public float aYawSeen;

		private float aRollSeen;

		public float paceGateZDbg;

		public float paceGateXDbg;

		public float paceDzHDbg;

		public float paceDzAllDbg;

		public float paceDxHDbg;

		public float paceDxAllDbg;

		public float paceRHDbg;

		public float pacePHDbg;

		public float paceTgtRaw;

		public float paceSeenZDbg;

		private float paceBetaG;

		public float paceMLeftDbg;

		public float paceLoDbg;

		public float paceHiDbg;

		public float paceClampDbg;

		// 2.20.0: the window's no-fin stop (the roll target held at the stopped-body-roll rotation r sin AoA; the yaw loop asked for its brake)
		// Revoker 0.6.0: off on this jet. The F-22E device assumes that with the fins gone there is no yaw axis left but the stabs; here
		// yawHealth reads the rudder and the fins, while the yaw nozzle and the elevons (the yaw axis at high AoA: elevon pair +-68 deg/s^2,
		// rudder +-12 at 350 km/h / 70 deg) are untouched. Bench damage suite, rudder or fins+rudder detached: with it on, 4 held / released
		// push-overs departed (sideslip 41-52 deg, AoA to -80..-93 after the release at -50) and the leaf stops made 13-17 deg against 0.5.1's
		// 2-3; off, every damaged case is back to 0.5.1's (worst +0.8 deg), clean cases identical. Inert on the Ifrit's bench either way.
		internal static bool PaceStopNoFin = false;

		private float paceStopNF;

		public float paceStopNFDbg;

		public float paceCapZPDbg;

		public float paceCapZNDbg;

		public float paceCapXPDbg;

		public float paceCapXNDbg;

		public float paceHdZPDbg;

		public float paceHdZNDbg;

		// 2.7.2: passive directional stiffness credited to the reversal cap, as a share of the restoring yaw at the margin sideslip
		internal static float PaceStiffUse = 1f;

		// 2.7.1's target rate limit at the capacity (Tom, 10:19: felt as slowed deceleration - off)


		internal static float PaceGateOn = 0.5f;

		internal static float PaceGateOff = 0.25f;

		private bool paceLatchZ;

		private bool paceLatchX;

		public float paceNEffDbg;

		public float paceDdDbg;


		private float revBlendF;

		internal static float PaceMarginK = 1f;

		public float paceMarginLeadDbg;

		// 2.6.9: the pace window's gate reads the whole airframe's yaw usage past the rudder fade (false = rudder load only)

		private float yawUseAllF;

		private float rateBlendEffDbg;
		// 2.5.12: the yaw the cap credits is derated by the allocator's own unmet-yaw report (yawSat, 0..1 over 10-50 deg/s^2
		// unmet, 0.25 s filter): by PaceSatDerateHi of it past the rudder fade (the stabs are the yaw axis there and the tables
		// overstate them 3-4x: 13:31 log, 55 s - a 64 deg/s roll allowed at 65 deg AoA, the release left 60 deg/s of yaw the
		// axis shed at 25 deg/s^2, 11 deg as the AoA fell through 30), PaceSatDerateLo below it. "Not too much" (Tom).
		internal static float PaceSatDerateHi = 0.5f;
		internal static float PaceSatDerateLo = 0.25f;




		// 2.6.6: 1 - cos AoA at which the shed cap stands at full strength (0.5 = 60 deg); the pace eases below it as holding buys less
		internal static float SweepShedCos = 0.05f;   // 2.8.0: 0.5 -> 0.05, the cap stands at every AoA (Tom, 2 Nov: smooth over fast in the push-over)

		// weight of the allocator's unmet yaw (yawSat) in the window's gate

		// the window's gate releases over this (s); the rudders read zero usage for a few ticks as they swing through center
		internal static float PaceGateTau = 0.3f;

		private float paceGateF;

		private float paceGateXF;
		// 2.6.1: the window's release is paced by the yaw. The window holds the target while the rudders are loaded (PaceLoad0..1 of
		// their yaw, 0.08 s filter) and let it go in one tick when they unloaded, into the full trade cap at the blended acceleration
		// (300-720 deg/s^2 at blend 0.41): the yaw demand jumped, the rudders reloaded, the window held again - a 1.9 Hz hold /
		// release cycle, +-5 deg/s on the target and +-5 deg on the rudders, in every corkscrew entry at 20-50 deg AoA once the
		// 2.5.14 trade share gave the target room above the window (bench; 14:55 / 15:10 logs: pSTarget chatter 0.7 -> 3.0-3.7
		// deg/s from 2.5.11). The hold may still come on in one tick (that is the protection); the climb back out of it may not
		// ask the yaw for more than PaceYawUse of what the rudders can add toward the move's side (their tables, from where they
		// are): roll-target rise x sin AoA = yaw acceleration. The trajectory's own acceleration and braking are untouched; this
		// is the commanded rate, climbing out of a hold at the yaw's pace instead of in a step. PaceReleasePace 0 = 2.6.0.
		// Only a hold with the roll already going the target's way is paced out (paceWinSame): a stick reversal's hold (the
		// window keeps the new target at zero while the roll brakes through) is released as before, so the reversal still runs at
		// the tuned acceleration (pacing it too took 450 km/h / 12-19 deg AoA reversals from 0.3 to 0.7-1.0 s on the bench).
		private bool paceWinHeld;
		private bool paceWinSame;
		private float paceWinPrev;
		public float paceRelDbg;
		internal static float PaceReleasePace = 1f;

		// the window applies as the rudders' usage (either side) runs from PaceLoad0 to PaceLoad1 of their yaw
		internal static float PaceLoad0 = 0.5f;

		internal static float PaceLoad1 = 0.75f;

		private Lpf paceRDotF;

		private Lpf pacePDotF;

		private float paceOmegaZPrev;

		private float paceOmegaXPrev;

		private bool paceInit;

		public float paceLeadDbg;

		public float paceHoldDbg;
		public float dbgWinLo, dbgWinHi, dbgWinLoR, dbgWinHiR, dbgReachPos, dbgReachNeg, dbgYawCorr, dbgWSat, dbgRateCap, dbgWin, dbgRollPos, dbgRDot, dbgPDot;

		internal static float PullTradeFFK = 0.3f;
		// 2.5.14: the share of the yaw authority an AoA sweep under a roll may spend, of the part the rudders are not using
		internal static float PullTradeFreeK = 0.35f;

		internal static float PullTradeFFIas0 = 93f;

		internal static float PullTradeFFIasSpan = 25f;

		internal static float PullTradeFall = 300f;

		internal static float PullTradeRise = 120f;

		internal static float PullTradeCutA = 15f;








		private bool pullGovTouched;


		internal static float GovCapTau = 0.3f;



		private float pullTradePs = 999f;

		private float pullTradeTgt = 999f;

		public bool pullTradeHoldDbg;

		internal static float PullTradeRiseHold = 0f;

		public float pullTradeDbg;

		public float slipGovPullDbg = 1f;






		internal static float ToeLatRelease = 0.5f;

		private float toeLatGate;

		public float toeTgtF;


		public float coordJerkDbg;

		internal static float AsymRollMax = 120f;

		internal static float AsymRollU0 = 0.1f;

		internal static bool AsymRollHardOnly = true;

		internal static float EndJerkHi = 2f;

		internal static float CoupGateA0 = 12f;

		// 2.9.0 SweepHoldPitchPrio: the nose-down counterpart of OvershootPitchPriority. While the AoA-sweep cap holds the pitch-rate
		// command above the stick's push (a push-over under a rotation the yaw cannot shed in time) and the nose drops faster than
		// that command by SweepHoldErr0..+SweepHoldErrSpan deg/s, the pitch hold owns the stabilators (allocator pitch weight up,
		// the released-roll priority off, the honest stab model, the nozzles free), as the run-away case does. 15:59 log (2.8.1),
		// 46.3 / 87.9 s: push-over from 65 deg AoA at 225 kt with 72-79 deg/s of rotation; the cap asked -12 then +9 deg/s, the
		// nose went down at 31 deg/s with the right stab at its +35 peak and the left at -8 (both on the roll stop, 240-280 deg/s^2
		// of roll unmet at weight 1.3 against pitch's 7.5), AoA 40 -> 20 in 0.4 s against a nose-up command, 19.5 / 21.2 deg of
		// sideslip at 25 -> 5 deg AoA from the 50 deg/s of yaw still on board. Tom: slow the pitch, not the rotation, at high AoA.
		internal static float SweepHoldErr0 = 2f;

		internal static float SweepHoldErrSpan = 8f;

		// 2.9.2 SweepHoldStallWindow: during the capped sweep toward zero AoA, once the stabilators have pitch range back
		// (stabPitchFrac > 0), they are held inside their attached window (local AoA <= lift peak + RpMainStallMargin), the
		// 1.8.40 StallWindow. 16:37 log (2.9.1), 53.7 s: the right stab was parked at -30 (45 deg of local AoA) by the roll
		// stop through 65 -> 50 deg while pitch was TVC-only; when the hold came (pitch weight 8, rate-bounded) the allocator
		// had no nose-up gradient on a stab that deep past its stall - the honest model reads TE-up as un-stalling it (more
		// tail lift, nose-down) - so it drove it further TE-down (-25) and put all the nose-up on the nozzles (+20, the stop);
		// the nose went 37 -> 22 deg at -24 deg/s against +5..+10 commanded until the stab had been walked through (0.5 s).
		// Inside the window the stabs have gradient the moment the hold needs them; the roll stop keeps what they give there.
		internal static float CoupHeldGate = 1f;

		internal static float CoupHeldDy0 = 0.04f;

		internal static float CoupHeldDySpan = 0.08f;

		internal static float AsymPitchPrio = 1f;

		internal static float AsymPitchPrioLead = 3f;

		internal static float AsymStabAntiCost = 0f;

		internal static float AsymPitchCapLead = 2f;

		internal static float AsymPitchCapFloor = 10f;

		public float asymCapPitch = 999f;


		internal static float EndJerkLp0 = 1.5f;

		internal static float EndJerkLp1 = 2.5f;

		public float lpDbg;

		internal static float AIRollDirectAccel = 3000f;

		public bool aiGunTrack;

		public float aiRollFade = 1f;

		private float rollSlew = 100f;

		public static bool DebugYaw;

		public float dbgWVe;

		public float dbgYawRud;

		public float dbgYawWing;

		public float dbgYawStab;

		public float dbgYawTvc;

		public float dbgTargetZ;

		public float dbgMfZ;

		public float dbgTgtY;

		public float dbgMfY;

		public float aRoll = 300f;

		public float aYaw = 60f;

		private Lpf aRollF;

		private Lpf aYawF;

		private Lpf aRollWF;

		public float aRollWing = 200f;

		public float pS;

		private RateTraj pTraj;

		// Ifrit 0.2.0: MPO separate-axes yaw-rate trajectory and the state of the last frame (for the hand-over)
		private RateTraj mpoRTraj;

		private RateTraj mpoPTraj;

		private float mpoCoord;

		private bool mpoSepPrev;

		public bool mpoSepDbg;

		private RateTraj qTraj;

		private Lpf rSF;

		public float rudderAuth = 1f;

		// 1.8.35 sideslip rudder: signed yaw-command travel allowed beyond the normal rudder fade (deg), and the pedal
		// sideslip command it corrects toward (from the previous lateral-law pass)
		public float rudSideDbg;

		private float betaPedalCmd;

		private float rudSideF;

		// 1.8.36 ground steering
		public bool gsActive;

		public float gsNwsDeg;

		public float gsRCmd;

		public float gsRud;

		public bool gsHold;

		public float gsLineY;

		public float gsChiRef;

		private Vector3 gsLineP;

		// the captured line's point in the game's global coordinates: NO shifts its Unity world origin as the jet travels, so a
		// Unity world position stored at the capture jumps (27 Oct F-22E log: 242 m and 58 m in one tick)
		private GlobalPosition gsLinePG;

		// GroundStockMatch: the jet's own ControlsFilter (stock fly-by-wire parameters), the stock yaw sent to the gear and
		// rudders, and the sideslip rate for the drift limit
		public ControlsFilter stockCf;

		private float gsStockYaw;

		public bool gsStockPhase;

		private float gsBetaPrev = float.NaN;

		private float gsBetaDot;

		private float gsInt;

		// the raw pedal (inp.yaw), as LandingGear reads it: no stick smoothing or rate limit
		public float gsPedRaw;

		private float gsRudInt;

		private float gsBrakeT;

		private float gsLead = -1f;

		public float gsChiAir;

		private float gsSinceContact = 99f;

		public float gsAirT;

		public float aInt;

		// 2.0.14: AoA limit capture. The capture bound used to stop at zero once the AoA was past the limit (the
		// flight-path rate only: nothing brought it back) and protection only began at limit + 0.5 with zero onset, so
		// the jet sat 0.3-0.6 deg over the limit in rolls. The bound now continues through the limit (CapKp per deg,
		// nose-down past it) and an integrator (CapKi) trims the measured residual while the pilot asks for more than
		// the limit. Protection is never less nose-down than this bound, so there is no step at limit + 0.5.
		public float capIUp;

		public float capIDn;

		public float capBoundU;

		public float capBoundD;

		// nose-down (U) / nose-up (D) AoA rate the capture asks for past the flight-path rate, for protection's onset
		public float capPushU;

		public float capPushD;

		internal static float CapKi = 3f;

		internal static float CapIMax = 4f;

		internal static float CapIAlphaDot = 3f;

		internal static float CapIBand = 0.6f;

		internal static float CapIHoldT = 0.3f;

		public float capHoldTU;

		public float capHoldTD;

		public float yawSat;

		private Lpf yawSatF;

		private Lpf guardF;

		public float stabPitchFrac = 1f;

		public float tvcAnchor;

		public float tvcBudgetGate;

		private float tvcBudgetLat;

		private bool tvcAnchorInit;

		private float pCmdPrev;

		private float rCmdPrev;

		private Lpf pDotF;

		private Lpf rDotF;

		public float pFull = 300f;

		public float pStockRef;

		public float pMaxDbg;

		// Ifrit 0.3.0: roll acceleration available from the differential TVC (deg/s^2), and the body-roll budget the sideslip
		// correction may use (deg/s)
		public float aRollTvc;

		private Lpf aRollTvcF;

		public float pSlipMax;

		public float slipGainDbg;

		public float rMaxDbg;

		public float rollCtrlHalf = 100000f;

		public float guardDbg = 1f;

		public float pSmaxDbg;

		public float pSTgtDbg;

		public float latAccDbg;

		public float latJerkDbg;

		public float rSDbg;

		public bool revFlow;

		private bool revFlowLogged;

		public float sp;

		public float sr;

		public float sy;

		private float spF;

		private float srF;

		private float syF;

		public float qB;

		public float qRaw;

		public float qCmd;

		public float qT;

		public float qTprev;

		public float betaG;

		public float alphaCmd;

		public float nCmd;

		public bool unload;

		public bool prot;

		public float qMaxDbg;

		public float qMinDbg;

		public float gUnderDbg;

		public float pCmd;

		public float rCmd;

		public float pDotFF;

		public float rDotFF;

		public Vector3 omegaDotDes;

		public Vector3 mTarget;

		// 2.0.8: DisableTVC (debug) locks the nozzles at 0 whatever the FCS computes
		private float tvcPitchRaw;

		public readonly List<SplitEffector> splits = new List<SplitEffector>();

		// Ifrit 0.2.2: the splits are still coming in after an airbrake release
		private bool splitRetract;

		// Ifrit 0.2.2: the split effectors' coordinates this frame (true = retraction from full split, brake out); the surface
		// hook feeds the job the matching throttle so the two never disagree
		public bool splitBrakeMode;

		public TvcDiffEffector tvcd;

		// Revoker 0.1.0: the nozzle's yaw axis
		public TvcYawEffector tvcy;

		// Revoker 0.3.0 (F-22E 2.6.6 reference governor, ported): the yaw axis the lateral governor counts on - the surfaces whose
		// job is yaw (the rudder) and the yaw nozzle, each with its own yaw table, stops and slew. The F-22E's governor reads
		// "the rudders" everywhere (the tables, the crossing time, the load); here that set is these effectors. The rudder's
		// share fades with the fin (rudderAuth; 75-85 deg on this jet); the nozzle's yaw is thrust and holds at any AoA and
		// IAS (speedFade above the vectoring speed, off under 3 m). The allocator's aYaw is not used as the capacity: it sums
		// every effector's yaw table (canards, elevons, splits included) and read 1.5-2.2x what the rudder and nozzle at their
		// stops delivered in the 30-31 Oct logs (calibrated plant: 58-91 deg/s^2 from the two, against aYaw 101-168; the plant's
		// total yaw acceleration matched the logs to 2-9 deg/s^2 RMS, so the two are the yaw there).
		private struct YawEff
		{
			public Effector e;
			public float lo;
			public float hi;
			public float auth;
			public bool rudder;
		}

		private readonly List<YawEff> yawEffs = new List<YawEff>(4);

		public float yawCapDbg = 60f;

		// Revoker 0.4.0: 1 = the probe's directional stiffness x margin / 2 is credited to the yaw capacity (see yawCapDbg); 0 = kinematic (F-22E)
		internal static float PaceStiffCredit = 1f;

		private float stiffCredit;

		// Revoker 0.4.0: the airframe's yaw damping from the probe (SlipMomentAt with a yaw rate) credited to the yaw a reversal has to shed:
		// a rotation r is braked by |Nr| r on its own, half of that on average over the swing to zero. The rate cap and the shed
		// cap (the two that brake the yaw) take it; the pull and sweep caps (a growing demand, which the damping opposes) do not.
		// 1 = credited; 0 = the F-22E's effectors-only model. SlipDampProbeRate: the yaw rate the probe perturbs by (rad/s).
		internal static float PaceDampCredit = 1f;

		internal static float SlipDampProbeRate = 0.35f;

		private float slipDampF;

		public float slipDampDbg;

		private float dampCredit;
		
		// Revoker 0.4.0: the rate cap from a forward model of the reversal instead of the kinematic yaw-swing formula. Each tick, for a
		// candidate velocity-vector roll rate pS, the governor reverses the jet in its head: the roll trajectory swings the demand
		// from +pS to -pS at its own reversal acceleration; the rate loops (KRoll / KYaw, the sideslip hold at the loop's own
		// gain) ask the effectors for roll and yaw; a small least-squares allocation over EVERY allocator effector (each from where
		// it is, at its own slew, within its allocator window, its tables at the present flow, pitch held) delivers what it can;
		// the body rates integrate with the airframe's sideslip stiffness in yaw and roll and its yaw damping (all three from the
		// probe's tables); the sideslip follows the kinematics beta' = p sin a - r cos a. The largest pS whose reversal keeps the
		// peak sideslip inside the margin is the cap. No effector set to pick by hand (the ailerons' yaw at 65 deg AoA counts as
		// the rudder's at 10), and where the roll axis is as slow as the yaw the model says so instead of charging the yaw's lag.
		// false = the kinematic formula (F-22E 2.5.8).
		// Revoker 0.4.0: 1 = the allocator weighs the lateral residual in the velocity frame (Allocator.slipFrame); 0 = body frame (2.6.6)
		internal static bool SlipFrameAlloc = true;

		// Revoker 0.8.0: off - the steady-rate cap is the F-22E 3.0.0 reversal model, as on the Raptor (Tom, 5 Nov 01:21: "I thought the gov is
		// airframe agnostic outside of the model"). RevSim, fed the estimator, sat at its 300 deg/s ceiling (01:23 log 54.1 s: 200 deg/s reversal
		// with the pull 8 -> 27 deg AoA, RevSim predicted 1.3-5 deg of sideslip, the jet made 14.5). Bench with the 3.0.0 cap at the jet's
		// RollPaceRateBlend 0.4: every regression case 0.3-3 deg less, sweep reversal worst 8.5 -> 6.2, stop 5.4 -> 3.9, corkscrews -0.7..-3.3
		// (one +0.6), damage suite better or equal; the price is rate: sweep mean roll 113 -> 106, corkscrew velocity roll 10-50 deg/s lower,
		// a clean leaf stop 1.73 -> 1.98 s. Replay of the 54.1 s reversal: 11.1 -> 6.7 deg.
		internal static bool RevSim = false;
		
		// Revoker 0.4.1: the yaw-reach window's reach is computed over EVERY allocator effector, not the hand-picked yaw set
		// (CollectYawEffs). The window holds the commanded roll to what the measured yaw can follow within PaceHorizon, so
		// its reach is a statement about the airframe's yaw - and the hand-picked set leaves out the trailing edge, which is
		// the yaw axis at moderate-to-high AoA on this jet (per-surface on the calibrated plant at 150 km/h / 65 deg AoA the
		// aileron pair makes +-12.5 deg/s^2 of yaw against the tails' +-2.2). With the old set the reach read 5-6 deg/s in
		// flight at 15-45 deg AoA and the window held the roll 12-14 deg/s under the stick while the sideslip sat at 2-5 deg
		// of an 8 deg margin (1 Nov Ifrit 0.7.0 log). Each effector from where it is, at its own slew, inside its allocator
		// window, its tables at the present flow, the same PaceRudderUse derate and one-side cap as before. The crossing
		// time, the probe's authority and the yaw slew lead keep the dedicated set. false = the pre-Revoker 0.4.1 reach.
		internal static bool PaceReachAll = true;
		
		public float paceReachDbg;
		
		// yaw moment (deg/s^2) every allocator effector can still add toward each side from where it is within the horizon,
		// in the same two forms the window needs: the steady one-side yaw (nAv) and the horizon-integrated reach (g)
		private void ReachAll(out float gPosA, out float gNegA, out float nAvPosA, out float nAvNegA)
		{
			gPosA = 0f;
			gNegA = 0f;
			nAvPosA = 0f;
			nAvNegA = 0f;
			foreach (Effector eR in effs)
			{
				if (eR == null || !eR.active || eR.M == null || eR.N < 2 || eR.rate <= 0f || eR.hi - eR.lo < 0.5f)
				{
					continue;
				}
				float nowR = Mathf.Clamp(eR.now, eR.lo, eR.hi);
				float zNowR = AccYaw(eR.MAt(nowR));
				float zHiR = AccYaw(eR.MAt(eR.hi));
				float zLoR = AccYaw(eR.MAt(eR.lo));
				float nSideR = 0.5f * Mathf.Abs(zHiR - zLoR);
				nAvPosA += Mathf.Min(nSideR, Mathf.Max(0f, Mathf.Max(zHiR, zLoR) - zNowR));
				nAvNegA += Mathf.Min(nSideR, Mathf.Max(0f, zNowR - Mathf.Min(zHiR, zLoR)));
				for (int dR = -1; dR <= 1; dR += 2)
				{
					float tgtR = ((zHiR >= zLoR) == (dR > 0)) ? eR.hi : eR.lo;
					float ddR = Mathf.Abs(tgtR - nowR);
					float tsR = Mathf.Min(PaceHorizon, ddR / Mathf.Max(1f, eR.rate));
					float dhR = nowR + Mathf.Sign(tgtR - nowR) * Mathf.Max(1f, eR.rate) * tsR;
					float aSR = Mathf.Max(0f, (float)dR * (AccYaw(eR.MAt(dhR)) - zNowR));
					float gR = 0.5f * aSR * tsR + aSR * (PaceHorizon - tsR);
					if (dR > 0)
					{
						gPosA += gR;
					}
					else
					{
						gNegA += gR;
					}
				}
			}
		}
		
		internal static float RevSimDt = 0.02f;
		
		internal static float RevSimTmax = 3f;
		
		internal static float RevSimMax = 300f;
		
		public float revSimBetaDbg;
		
		public float revSimTDbg;
		
		public float revSimAtDbg;
		
		private struct RevEff
		{
			public float pHi;
			public float qHi;
			public float rHi;
			public float pLo;
			public float qLo;
			public float rLo;
			public float tHi;
			public float tLo;
		}
		
		private readonly List<RevEff> revEffs = new List<RevEff>(16);
		
		private float[] revX = new float[16];
		
		// the probe's signed sideslip slopes (deg/s^2 per deg of sideslip, in the tables' own sign): yaw and roll
		public float slipYawSlopeDbg;
		
		public float slipRollSlopeDbg;
		
		private float slipYawSlopeF;
		
		private float slipRollSlopeF;
		
		internal static bool RevSimDump = false;

		private void CollectRevEffs()
		{
			revEffs.Clear();
			foreach (Effector e in effs)
			{
				if (e == null || !e.active || e.M == null || e.N < 2 || e.rate <= 0f || e.hi - e.lo < 0.5f)
				{
					continue;
				}
				float now = Mathf.Clamp(e.now, e.lo, e.hi);
				Vector3 m0 = e.MAt(now);
				Vector3 mH = e.MAt(e.hi) - m0;
				Vector3 mL = e.MAt(e.lo) - m0;
				RevEff x = default(RevEff);
				x.pHi = AccRoll(mH);
				x.qHi = IaInv[1, 1] * mH.y * 57.29578f;
				x.rHi = AccYaw(mH);
				x.pLo = AccRoll(mL);
				x.qLo = IaInv[1, 1] * mL.y * 57.29578f;
				x.rLo = AccYaw(mL);
				x.tHi = Mathf.Max(0.01f, (e.hi - now) / Mathf.Max(1f, e.rate));
				x.tLo = Mathf.Max(0.01f, (now - e.lo) / Mathf.Max(1f, e.rate));
				revEffs.Add(x);
				if (RevSimDump)
				{
					System.Console.WriteLine($"   reveff {e.name} now {now:F1} [{e.lo:F1},{e.hi:F1}] rate {e.rate:F0} hi p{x.pHi:F1} q{x.qHi:F1} r{x.rHi:F1} t{x.tHi:F2} | lo p{x.pLo:F1} q{x.qLo:F1} r{x.rLo:F1} t{x.tLo:F2}");
				}
			}
			if (revX.Length < revEffs.Count)
			{
				revX = new float[revEffs.Count + 8];
			}
		}
		
		// peak |sideslip| (deg) of a full reversal from velocity-vector roll pS at the trajectory's reversal acceleration aT
		private float RevSimBeta(float pS, float aT, float jT, float c, float sn, float kY, float kL, float nR, float g, out float tRev)
		{
			int n = revEffs.Count;
			float dt = Mathf.Max(0.005f, RevSimDt);
			float p = pS * c;
			float r = pS * sn;
			float r0 = r;
			float beta = 0f;
			float wd = pS;
			float aRamp = 0f;
			float peak = 0f;
			tRev = RevSimTmax;
			for (int i = 0; i < n; i++)
			{
				revX[i] = 0f;
			}
			float kRoll = Plugin.KRoll;
			float kYaw = Plugin.KYaw;
			for (float t = 0f; t < RevSimTmax; t += dt)
			{
				bool ramp = wd > 0f - pS;
				// Revoker 0.4.1: the trajectory ramps its acceleration at the roll jerk limit and never reaches aT on a short swing -
				// stepping straight to aT made the modelled reversal 3-4x more violent than the one the law actually flies
				// (bench, Ifrit at 150 km/h / 35 deg AoA: model 127 deg/s^2 against a realized 27-35), and the cap came out
				// 14-29 deg/s where the real reversal at 42 made 1.9 deg of sideslip against an 8 deg margin.
				aRamp = Mathf.Min(aT, aRamp + jT * dt);
				wd = Mathf.Max(0f - pS, wd - aRamp * dt);
				// the law: rate commands (the sideslip hold rotates about the velocity normal: -g beta sin a of roll, +g beta cos a of yaw)
				float pd = wd * c - g * beta * sn;
				float rd = wd * sn + g * beta * c;
				float ap = kRoll * (pd - p) + (ramp ? (0f - aRamp * c) : 0f);
				float ar = kYaw * (rd - r) + (ramp ? (0f - aRamp * sn) : 0f);
				// allocation: least squares over roll, pitch (held at zero) and yaw, each effector within [-1, 1] of its two moves,
				// three Gauss-Seidel sweeps, then its slew
				float sp = 0f;
				float sq = 0f;
				float sr = 0f;
				for (int i = 0; i < n; i++)
				{
					RevEff e = revEffs[i];
					float xi = revX[i];
					if (xi >= 0f)
					{
						sp += xi * e.pHi;
						sq += xi * e.qHi;
						sr += xi * e.rHi;
					}
					else
					{
						sp -= xi * e.pLo;
						sq -= xi * e.qLo;
						sr -= xi * e.rLo;
					}
				}
				for (int sweep = 0; sweep < 3; sweep++)
				{
					for (int i = 0; i < n; i++)
					{
						RevEff e = revEffs[i];
						float xi = revX[i];
						// residual without this effector
						float ep;
						float eq;
						float er;
						if (xi >= 0f)
						{
							ep = ap - (sp - xi * e.pHi);
							eq = 0f - (sq - xi * e.qHi);
							er = ar - (sr - xi * e.rHi);
						}
						else
						{
							ep = ap - (sp + xi * e.pLo);
							eq = 0f - (sq + xi * e.qLo);
							er = ar - (sr + xi * e.rLo);
						}
						float nH = e.pHi * e.pHi + e.qHi * e.qHi + e.rHi * e.rHi;
						float nL = e.pLo * e.pLo + e.qLo * e.qLo + e.rLo * e.rLo;
						float xH = (nH > 1E-06f) ? Mathf.Clamp01((ep * e.pHi + eq * e.qHi + er * e.rHi) / nH) : 0f;
						float xL = (nL > 1E-06f) ? Mathf.Clamp01((ep * e.pLo + eq * e.qLo + er * e.rLo) / nL) : 0f;
						float cH = (ep - xH * e.pHi) * (ep - xH * e.pHi) + (eq - xH * e.qHi) * (eq - xH * e.qHi) + (er - xH * e.rHi) * (er - xH * e.rHi);
						float cL = (ep - xL * e.pLo) * (ep - xL * e.pLo) + (eq - xL * e.qLo) * (eq - xL * e.qLo) + (er - xL * e.rLo) * (er - xL * e.rLo);
						float xNew = (cH <= cL) ? xH : (0f - xL);
						// slew: from where it is, at its own rate
						float lim = (xNew > xi) ? (dt / ((xi >= 0f) ? e.tHi : e.tLo)) : (dt / ((xi > 0f) ? e.tHi : e.tLo));
						xNew = Mathf.Clamp(xNew, xi - lim, xi + lim);
						if (xNew >= 0f)
						{
							sp += xNew * e.pHi - ((xi >= 0f) ? (xi * e.pHi) : (0f - xi * e.pLo));
							sq += xNew * e.qHi - ((xi >= 0f) ? (xi * e.qHi) : (0f - xi * e.qLo));
							sr += xNew * e.rHi - ((xi >= 0f) ? (xi * e.rHi) : (0f - xi * e.rLo));
						}
						else
						{
							sp += (0f - xNew * e.pLo) - ((xi >= 0f) ? (xi * e.pHi) : (0f - xi * e.pLo));
							sq += (0f - xNew * e.qLo) - ((xi >= 0f) ? (xi * e.qHi) : (0f - xi * e.qLo));
							sr += (0f - xNew * e.rLo) - ((xi >= 0f) ? (xi * e.rHi) : (0f - xi * e.rLo));
						}
						revX[i] = xNew;
					}
				}
				// the airframe: effectors + sideslip stiffness (yaw and roll) + yaw damping on the yaw-rate change
				float accP = sp + kL * beta;
				float accR = sr + kY * beta + nR * (r - r0);
				p += accP * dt;
				r += accR * dt;
				beta += (p * sn - r * c) * dt;
				if (RevSimDump && pS == RevSimMax && ((int)(t / dt + 0.5f)) % 10 == 0)
				{
					System.Console.WriteLine($"   revsim t{t:F2} wd{wd:F0} p{p:F1} r{r:F1} beta{beta:F2} ap{ap:F0} ar{ar:F0} sp{sp:F0} sq{sq:F0} sr{sr:F0} x[{string.Join(",", System.Linq.Enumerable.Select(System.Linq.Enumerable.Take(revX, n), v => v.ToString("F2")))}]");
				}
				float ab = Mathf.Abs(beta);
				if (ab > peak)
				{
					peak = ab;
				}
				if (peak > 60f)
				{
					break;
				}
				if (p * c + r * sn <= -0.9f * pS)
				{
					tRev = t;
					break;
				}
			}
			return peak;
		}
		
		// the largest velocity-vector roll rate whose full reversal stays inside the sideslip margin (bisection over the forward model)
		private float RevSimCap(float margin, float c, float sn, float aPace)
		{
			CollectRevEffs();
			if (revEffs.Count == 0)
			{
				return PaceRateFloor;
			}
			float cA = Mathf.Max(Mathf.Abs(c), 0.05f);
			// the trajectory's reversal acceleration, as the roll law sets it (RevFloorK x roll authority, in velocity-roll terms)
			float aT = Mathf.Max(20f, ((ReversalFloor && RevConst) ? (RevFloorK * aRoll) : (Plugin.RollBrakeUse * (BrakeUseAll ? Mathf.Min(aRollPos, aRollNeg) : aRollWing))) / cA);
			// Revoker 0.6.0 (as Ifrit 0.7.2): the reversal the governor flies is paced by the yaw the effectors can add (the estimator's,
			// PaceYawUse of it over sin AoA); simulating an unpaced full-stick reversal and capping on that double-counts the protection
			if (aPace > 0f)
			{
				aT = Mathf.Min(aT, aPace);
			}
			revSimAtDbg = aT;
			// Revoker 0.4.1: the trajectory's own roll-jerk limit (the a2 of the lateral law), in velocity-roll terms
			float jT = Mathf.Max(1f, Mathf.Min(aT / Mathf.Max(0.01f, RollJerkTau), RollJerkServo * aRoll * rollSlew / cA));
			float kY = slipYawSlopeF;
			float kL = slipRollSlopeF;
			float nR = (PaceDampCredit > 0f) ? Mathf.Min(0f, slipDampF) : 0f;
			float g = Mathf.Max(0f, slipGainDbg);
			float lo = 0f;
			float hi = RevSimMax;
			float bHi = RevSimBeta(hi, aT, jT, c, sn, kY, kL, nR, g, out float tH);
			if (bHi * Mathf.Max(0.1f, PaceRateK) <= margin)
			{
				revSimBetaDbg = bHi;
				revSimTDbg = tH;
				return hi;
			}
			float bLo = 0f;
			float tLo = 0f;
			for (int it = 0; it < 8; it++)
			{
				float mid = 0.5f * (lo + hi);
				float b = RevSimBeta(mid, aT, jT, c, sn, kY, kL, nR, g, out float tM);
				if (b * Mathf.Max(0.1f, PaceRateK) <= margin)
				{
					lo = mid;
					bLo = b;
					tLo = tM;
				}
				else
				{
					hi = mid;
				}
			}
			revSimBetaDbg = bLo;
			revSimTDbg = tLo;
			return Mathf.Max(PaceRateFloor, lo);
		}

		// Revoker 0.4.0: the sweep cap fades in with the forecast sideslip over the margin (true); false = F-22E 2.6.6 (at every sideslip)
		internal static bool SweepMarginGate = true;

		private void CollectYawEffs()
		{
			yawEffs.Clear();
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Rudder && se.active && se.M != null && se.N >= 2 && se.rate > 0f)
				{
					yawEffs.Add(new YawEff { e = se, lo = se.min, hi = se.max, auth = Mathf.Max(0.05f, rudderAuth), rudder = true });
				}
			}
			if (tvcy != null && tvcy.active && tvcy.M != null && tvcy.N >= 2 && tvcy.hi - tvcy.lo > 0.5f && tvcy.rate > 0f)
			{
				yawEffs.Add(new YawEff { e = tvcy, lo = tvcy.lo, hi = tvcy.hi, auth = 1f, rudder = false });
			}
		}

		// the yaw moment (aero frame z) a yaw effector can reach each way within its usable travel
		private static void YawRange(YawEff y, out float zHi, out float zLo)
		{
			zHi = float.MinValue;
			zLo = float.MaxValue;
			for (int i = 0; i < y.e.N; i++)
			{
				float g = y.e.grid[i];
				if (g < y.lo - 0.001f || g > y.hi + 0.001f)
				{
					continue;
				}
				zHi = Mathf.Max(zHi, y.e.M[i].z);
				zLo = Mathf.Min(zLo, y.e.M[i].z);
			}
			float zA = y.e.MAt(y.lo).z;
			float zB = y.e.MAt(y.hi).z;
			zHi = Mathf.Max(zHi, Mathf.Max(zA, zB));
			zLo = Mathf.Min(zLo, Mathf.Min(zA, zB));
		}

		// Revoker 0.2.0: the inboard trailing-edge flaps as flaperons
		public readonly List<HingeFlapEffector> hinges = new List<HingeFlapEffector>();

		internal static float HingeSymCostK = 1f;

		internal static int HingeRollGroup = 3;

		public float tvcYawInput;

		public float tvcRollInput;

		public float tvcPitchInput
		{
			get
			{
				return Plugin.DisableTVC ? 0f : tvcPitchRaw;
			}
			set
			{
				tvcPitchRaw = value;
			}
		}

		private int groundFrames;

		public bool rotating;

		internal static float RotLeadT = 0.1f;

		internal static float RotLeadMax = 3f;

		public bool mainsOnGround;

		// 2.0.6 ground rate law state
		public bool groundRateOn;

		public bool noseContactLaw;

		private float gNeutralT;

		private float gContactAir;

		private float rotMomLim;

		private bool rotMomOn;

		public float groundBrake;

		public float airBrake;

		// 2.2.16 (AirbrakeAoAFade): the in-flight brake's share of the drag and its opening target
		private readonly List<Airbrake> airbrakes = new List<Airbrake>();

		private static readonly System.Reflection.FieldInfo fABDrag = HarmonyLib.AccessTools.Field(typeof(Airbrake), "dragAmount");

		private static readonly System.Reflection.FieldInfo fABPart = HarmonyLib.AccessTools.Field(typeof(Airbrake), "part");

		private readonly List<Rigidbody> abRb = new List<Rigidbody>();

		private readonly List<float> abDrag = new List<float>();

		private readonly List<UnitPart> abPart = new List<UnitPart>();

		public float airbrakeShare = 1f;

		public float airbrakeFade = 1f;

		public float airbrakeOpenNow;

		// the opening the brake is driven to: 0 when not commanded, else 1 or less with AirbrakeAoAFade
		public float AirbrakeOpenTarget => AirbrakeOut ? (Plugin.AirbrakeAoAFade ? airbrakeFade : 1f) : 0f;

		public bool onGround;

		private float[] brakeBias;

		// Revoker 0.2.8: TrailingEdgeBrakeConfig, only with ThrottleIndependentAirbrake on (off: the airbrake is fully stock)
		public bool TeBrakeOn => Plugin.TeBrakeConfig && Plugin.ThrottleIndependentAirbrake;

		public float teBrakeAir;

		public float teBrakeAirF;

		public float teBrakeLandF;

		public bool derotGate;

		public bool teBrakeMov;

		internal static float DerotationRate = 4f;

		internal static float TeBrakeAirRate = 0f;

		// Revoker 0.2.9: the outboard elevons' trailing-edge-up share of the in-flight set is solved each frame so that the set's
		// normal force, including the canards' re-trim of its pitch moment, stays at zero (TeBrakeLiftNeutral); at most TeBrakeOutMax
		internal static bool TeBrakeLiftNeutral = true;

		internal static float TeBrakeOutMax = 30f;

		public float teBrakeOutDbg;

		public float teBrakeFnDbg;

		// Revoker 0.2.9 (TeBrakeLiftNeutral): 0.2.8 fixed the outboard share at 20 deg (inboard 25 TE down) and moved the set at
		// 0.5 /s, because faster it bumped the g: the two sets' lift does not rise in proportion (bench, 900 km/h, set in 0.4 s:
		// nz 1.09 -> 1.25 at 0.2 s, 0.84 at 0.5 s, still 0.94 after 1 s with the AoA working back up at the held attitude).
		// Here, after the effectors are evaluated, the outboard common deflection is the one whose normal force cancels the
		// inboard set's plus the canards' normal force from re-trimming both sets' pitch moment (canard normal force per unit
		// of its pitch moment, from its own model at its present deflection). Scanned over 0..TeBrakeOutMax in 0.5 deg steps.
		internal static float TeBrakeSymK = 50f;

		// Revoker 0.2.10 (TrailingEdgeBrakeAoAFade): the in-flight set closes with AoA as the F-22E's speedbrake does. The F-22E
		// opens it by drag share (AirbrakeAoAFade, 2.2.16: full at 40 % of the total drag, stowed at 15 %), which on the clean
		// F-22E is full to ~15 deg AoA, half at ~25 and stowed past ~35. The same share rule does not carry over: the Revoker's
		// elevon set is a smaller share of a cleaner airframe (bench, model drag, 400 km/h: 0.41 at 6 deg, 0.23 at 9, 0.13-0.17
		// flat from 12 to 36 deg), so it would close by 12 deg and barely reopen (at most 9 % open up to 36 deg). The F-22E's resulting AoA schedule is used
		// instead: full to TeFadeAoA0, closing linearly to stowed at TeFadeAoA1 (|AoA|), at most 2 /s as the F-22E's filter.
		internal static float TeFadeAoA0 = 15f;

		internal static float TeFadeAoA1 = 35f;

		public float teFade = 1f;

		private void TeBrakeFadeStep(float dt)
		{
			float tgt = 1f - Mathf.InverseLerp(TeFadeAoA0, Mathf.Max(TeFadeAoA0 + 1f, TeFadeAoA1), Mathf.Abs(alpha));
			teFade = Mathf.MoveTowards(teFade, tgt, 2f * dt);
		}

		private void TeBrakeLiftBalance()
		{
			teBrakeOutDbg = 0f;
			teBrakeFnDbg = 0f;
			// the pairs' common deflection held to the set this much harder while it is in (1 = the normal ElevonSymCost)
			float kS = 1f + (Mathf.Max(1f, TeBrakeSymK) - 1f) * Mathf.Max(teBrakeAirF, teBrakeLandF);
			foreach (SurfaceEffector sK in surfaces)
			{
				if (sK.partner != null && sK.kind == SurfaceKind.Aileron)
				{
					sK.symCost = Plugin.ElevonSymCost * kS;
				}
			}
			foreach (HingeFlapEffector hK in hinges)
			{
				if (hK.partner != null)
				{
					hK.symCost = Plugin.ElevonSymCost * HingeSymCostK * kS;
				}
			}
			if (!TeBrakeLiftNeutral || teBrakeAirF <= 0.001f || Plugin.AirBrakeElevonUp <= 0f)
			{
				return;
			}
			Transform root = ((Component)(object)ac).transform;
			SurfaceEffector ob = null;
			foreach (SurfaceEffector sB in surfaces)
			{
				if (sB.partner != null && sB.kind == SurfaceKind.Aileron && sB.active && sB.GeoTeUp(root) != 0f)
				{
					ob = sB;
					break;
				}
			}
			if (ob == null)
			{
				return;
			}
			float tuO = ob.GeoTeUp(root);
			float dF = 0f;
			float dM = 0f;
			foreach (HingeFlapEffector hB in hinges)
			{
				float tuH = hB.GeoTeUp(root);
				if (hB.active && tuH != 0f)
				{
					float dI = 0f - tuH * Plugin.AirBrakeInboardDown * teBrakeAirF;
					dF += hB.FNAt(dI) - hB.FNAt(0f);
					dM += hB.MAt(dI).y - hB.MAt(0f).y;
				}
			}
			// canards: normal force per unit pitch moment
			float kC = 0f;
			int nC = 0;
			foreach (SurfaceEffector sC in surfaces)
			{
				if (sC.kind == SurfaceKind.Stabilator && sC.active)
				{
					float a0 = Mathf.Clamp(sC.now - 1f, sC.min, sC.max - 2f);
					float dMc = sC.MAt(a0 + 2f).y - sC.MAt(a0).y;
					if (Mathf.Abs(dMc) > 1f)
					{
						kC += (sC.FNAt(a0 + 2f) - sC.FNAt(a0)) / dMc;
						nC++;
					}
				}
			}
			kC = (nC > 0) ? (kC / (float)nC) : 0f;
			int nO = (ob.partner is SurfaceEffector obP && obP.active) ? 2 : 1;
			float bestU = 0f;
			float bestE = float.MaxValue;
			float uMax = Mathf.Min(TeBrakeOutMax, ob.max, 0f - ob.min);
			for (float u = 0f; u <= uMax + 0.001f; u += 0.5f)
			{
				float dO = tuO * u;
				float fO = (float)nO * (ob.FNAt(dO) - ob.FNAt(0f));
				float mO = (float)nO * (ob.MAt(dO).y - ob.MAt(0f).y);
				float e = Mathf.Abs(dF + fO - kC * (dM + mO));
				if (e < bestE)
				{
					bestE = e;
					bestU = u;
				}
			}
			teBrakeOutDbg = bestU;
			teBrakeFnDbg = bestE;
			foreach (SurfaceEffector sB in surfaces)
			{
				float tuB = sB.GeoTeUp(root);
				if (sB.partner != null && sB.kind == SurfaceKind.Aileron && tuB != 0f)
				{
					sB.symTarget += tuB * bestU;
				}
			}
		}

		private float gbLogTimer;

		private float noseDownTime;

		private float noseUpTime;

		private float airTime;

		private LandingGear noseGearCached;

		private bool noseGearSearched;

		private FcsMode lastLoggedMode;

		private Telemetry telemetry;

		private float time;

		private static readonly float[] envAlphas = new float[25]
		{
			-1f, 0f, 1f, 5f, 10f, 15f, 20f, 25f, 30f, 35f,
			40f, 45f, 50f, 55f, 60f, 65f, -5f, -10f, -15f, -20f,
			-25f, -30f, -35f, -40f, -45f
		};

		private const int EnvNegStart = 16; // 1.9.7: negative-lift points for the negative hand-over threshold

		private readonly float[] envLift = new float[25];

		private static readonly float[] asymAlphas = BuildAsymAlphas();

		private readonly float[] asymU = new float[22];

		private readonly float[] asymAr = new float[22];

		private readonly float[] asymAy = new float[22];

		private readonly float[] asymCr = new float[22];

		private readonly float[] asymCy = new float[22];

		private readonly float[] asymNetDn = new float[22];

		// 1.9.0: nose-down authority incl. the flaperons at their present deflection (asymNetDn leaves their lift out)
		private readonly float[] authNetDn = new float[22];

		private readonly float[] asymNetUp = new float[22];

		public float aBrakeTgt;

		internal static int CaptureMode = 1;

		internal static bool RollCapture = true;

		internal static float CapLagT = 0.3f;

		// share of the nose-down (nose-up at negative AoA) pitch authority that the nozzles carry
		private float CapTvcShare()
		{
			if (tvc == null || !tvc.active || tvc.M == null || tvc.N < 2 || Ia[1, 1] <= 1f)
			{
				return 0f;
			}
			float lo = float.MaxValue;
			float hi = float.MinValue;
			for (int k = 0; k < tvc.N; k++)
			{
				lo = Mathf.Min(lo, tvc.M[k].y);
				hi = Mathf.Max(hi, tvc.M[k].y);
			}
			float half = 0.5f * (hi - lo) * IaInv[1, 1] * 57.29578f;
			return Mathf.Clamp01(half / Mathf.Max(20f, (alpha >= 0f) ? aDn : aUp));
		}


		internal static float RollCaptureCoupK = 1f;

		internal static float PitchLeadT = 0.25f;

		internal static float StabsPitchOnlyCost = 1e6f;

		internal static float HiAoAStabRollCost = 0f;

		// Revoker 0.2.7: only a live rudder keeps the canards out of the first yaw tier
		internal static bool RudderTierLiveOnly = true;

		// Revoker 0.2.7 (FinLossTvcHold)
		internal static float FinLossTvcSpeed = 1000f;

		// Revoker 0.2.7 (CanardAttachedWindow)
		internal static float CanWinA0 = 30f;

		internal static bool CanWinLatOnly = true;

		internal static bool CanWinNegOnly = true;

		// Revoker 0.2.7: canard travel the asymmetric-load sweep counts (deg)
		internal static float AsymStabTravel = 40f;

		private float hiRollWF;

		internal static float CanWinRate = 120f;

		public float canWinDbg;

		internal static float TvcNeutralQdot = 20f;

		internal static float TvcNeutralTau = 1f;

		internal static float TvcNeutralBand = 1f;

		private float tvcNeutralW;

		// Revoker 0.1.0: yaw-TVC neutral hold state
		private float tvcYawNeutralW;

		private float srHeldY;

		private float syHeldY;

		public float tvcNeutralDbg;

		public float fightGateDbg;

		// 0.4.3: the 0.4.1 roll-fight exemption with an engine down on thrust (not only out); false (see the fight gate)
		internal static bool FightExemptAsym = false;

		// bench/debug: local AoA now of the k-th stabilator (order of the surfaces list)
		public float StabLocalAoA(int k)
		{
			int n = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Stabilator)
				{
					if (n == k)
					{
						return se.localAoANow;
					}
					n++;
				}
			}
			return 0f;
		}

		// Ifrit 0.3.0: roll-TVC neutral hold weight
		public float tvcRollNeutralW;

		internal static float HiAoAStabRollA0 = 22f;

		internal static float HiAoAStabRollSpan = 10f;

		public float pitchLeadDbg;



		private float asymTimer = 99f;

		private bool asymValid;

		public float asymCapPos = 999f;

		public float asymCapNeg = -999f;

		public float asymCapPosEff = 999f;

		public float asymCapNegEff = -999f;

		public float asymDy;

		public float asymArNow;

		public float asymAyNow;

		public float asymUNow;

		public float asymCrNow;

		public float asymCyNow;

		public Vector3 asymMNow;

		private Lpf3 asymMF;

		public Vector3 asymFF;

		public float asymFFr;

		public float asymFFy;

		public float asymFFGate;

		public float rollPitchFF;

		private float tvcPin;

		private bool tvcPinned;

		private readonly float[] pinLo = new float[16];

		private readonly float[] pinHi = new float[16];

		private readonly float[] symSave = new float[16];

		public float dMeasR;

		public float dMeasY;

		private Lpf dMeasRF;

		private Lpf dTrimRF;

		public float dTrimR;

		public float aRollPos = 300f;

		public float aRollNeg = 300f;

		internal static bool RollTrimHeadroom = true;

		private Lpf dMeasYF;

		private float asymLogTimer = 99f;

		private float asymLoggedCap = 999f;

		private float asymLoggedCapN = -999f;

		private readonly HashSet<AeroPart> surfaceParts = new HashSet<AeroPart>();

		private int asymK = -1;

		private bool asymCycleDone;

		private bool asymFirstLogged;

		internal static int AsymPointsPerStep = 3;

		private Vector3 asymMT;

		// bench/debug: the sweep's thrust moment as yaw accel (deg/s^2), and whether the pitch / differential TVC effectors are active
		public float AsymMTYaw => AccYaw(asymMT);

		public int TvcActiveDbg => (tvc != null && tvc.active ? 1 : 0) + (tvcd != null && tvcd.active ? 2 : 0);

		private float[] asymDsym;

		public float Lp = -100000f;

		private const float KCapture = 3f;

		internal static float YawBoostFixed = -1f;

		internal static bool RudderRevBack = true;

		internal static float YawFFAoA0 = 15f;

		internal static float YawFFAoA1 = 28f;

		internal static bool ReversalFloor = true;

		internal static float RevFloorK = 1f;

		internal static bool RevConst = true;

		internal static bool RevCarry = true;

		private float revA;

		private float revTarget;

		internal static float WashK = -1f;

		internal static float WashTau = -1f;

		internal static float WashUtilHi = 0.75f;

		internal static bool GUnderAboveOnly = true;

		private float washApplied;

		internal static float MLeadTau = 0.04f;

		internal static float IndiTau = 0.04f;

		// 1.9.0 hybrid INDI (pitch): the incremental law takes the airframe's own moment from filtered measurements, so
		// it lags the true one by the filter (IndiTau). At high dynamic pressure the non-control-surface moment moves fast
		// with AoA and pitch rate (the fuselage/LERX nose-up grows with AoA), and that lag let the pitch rate overshoot its
		// command by ~2 deg/s for 0.3 s at 1300 km/h - ~1.4 g at that speed, the G-command overshoot/undershoot cycle.
		// The lag is predicted from the airframe model: dM = dM/dAoA x (AoA - AoA filtered) + dM/dq x (q - q filtered),
		// slopes of the non-surface parts' moment evaluated every 0.1 s at the present flow.
		public float mAlphaO;

		public float mQO;

		public float hybridDM;

		private float dHatPrevY;

		private bool dHatInit;

		internal float dDotYF;

		private float distLeadGate;

		internal float distLeadAcc;

		internal static float DistLeadTau = 0.06f;

		internal static float DistLeadMax = 15f;

		internal static bool DistLeadNoDamp = true;

		internal static float DistLeadTw0 = 0.2f;

		internal static float DistLeadTw1 = 0.4f;

		internal static float NewtonMinTw = 0.25f;

		internal static bool NewtonReleasedOnly = true;

		private Lpf alphaIndiF;

		private Lpf qIndiF;

		private float mPrevY;

		private bool mLeadInit;

		private float mDotYF;

		public float mLeadGate;

		public float mLeadAgree = 1f;

		internal static bool PairedTrim = true;

		public float latReserve;

		public bool isAI;

		public float aiGLimit = 9.5f;

		public float aiCornerSpeed = 140f;

		private bool telemetryOpened;

		private float aiLogT;

		private float aiLogAMax;

		private float aiLogAMin;

		private float aiLogGMax;

		private float aiLogGMin;

		private float aiLogIasMin = 9999f;

		private float aiLogIasMax;

		private float aiLogProtT;

		private int aiLogN;

		private float aiLogP2;

		private int aiLogRev;

		private int aiLogRevSign;

		private float aiLogGunT;

		private float aiLogFadeT;

		private void AILogStep(float dt)
		{
			if (aiLogN == 0)
			{
				aiLogAMax = (aiLogAMin = alpha);
				aiLogGMax = (aiLogGMin = nz);
			}
			aiLogN++;
			aiLogT += dt;
			aiLogAMax = Mathf.Max(aiLogAMax, alpha);
			aiLogAMin = Mathf.Min(aiLogAMin, alpha);
			aiLogGMax = Mathf.Max(aiLogGMax, nz);
			aiLogGMin = Mathf.Min(aiLogGMin, nz);
			aiLogIasMin = Mathf.Min(aiLogIasMin, ias * 3.6f);
			aiLogIasMax = Mathf.Max(aiLogIasMax, ias * 3.6f);
			if (prot)
			{
				aiLogProtT += dt;
			}
			float pDeg = omega.x * 57.29578f;
			aiLogP2 += pDeg * pDeg * dt;
			int num = ((pDeg > 15f) ? 1 : ((pDeg < -15f) ? (-1) : 0));
			if (num != 0)
			{
				if (aiLogRevSign != 0 && num != aiLogRevSign)
				{
					aiLogRev++;
				}
				aiLogRevSign = num;
			}
			if (aiGunTrack)
			{
				aiLogGunT += dt;
				if (aiRollFade < 0.99f)
				{
					aiLogFadeT += dt;
				}
			}
			if (aiLogT >= 20f)
			{
				Plugin.Log.LogInfo($"Revoker FCS AI jet #{((UnityEngine.Object)(object)ac).GetInstanceID()} last {aiLogT:0}s: mode {Mode}, IAS {aiLogIasMin:0}-{aiLogIasMax:0} km/h, AoA {aiLogAMin:0.0}..{aiLogAMax:0.0} deg, g {aiLogGMin:0.0}..{aiLogGMax:0.0}, AoA protection active {aiLogProtT:0.0}s, roll rate rms {Mathf.Sqrt(aiLogP2 / Mathf.Max(aiLogT, 0.01f)):0} deg/s with {aiLogRev} left/right reversals (>15 deg/s), gun-tracking a moving target {aiLogGunT:0.0}s (roll faded {aiLogFadeT:0.0}s).");
				aiLogT = 0f;
				aiLogN = 0;
				aiLogProtT = 0f;
				aiLogP2 = 0f;
				aiLogRev = 0;
				aiLogGunT = 0f;
				aiLogFadeT = 0f;
				aiLogIasMin = 9999f;
				aiLogIasMax = 0f;
			}
		}

		public int stabBranch;

		public float pitchNeed;

		// 2.9.0: the two halves of pitchNeed - the nose-up run-away (1.8.23) and the capped nose-down sweep running ahead of its cap
		public float pitchNeedUp;

		public float pitchNeedDn;

		public bool sweepHoldWinDbg;

		// Revoker 0.2.5: 20 (F-22E / Ifrit: 5). Without TVC the modelled net nose-down at the 45 deg cap is under 10 deg/s^2,
		// so the floor set the velocity-vector roll at 25 deg/s at every IAS (in-game 28 Oct 19:56, DisableTVC). Bench, TVC
		// off, loaded rolls at the cap 220-600 km/h: 23-24 -> 32-46 deg/s mean, AoA overshoot 0.3 deg, sideslip 4.5-7.3 deg,
		// push recovery unchanged (1.56 -> 1.66 s at 300 km/h). 30 gave little more (32-52) for more sideslip.
		internal static float CoupRollFloor = 20f;

		// 2.6.9: the coupling roll limit from the pitch axis's measured nose-down headroom (false = the table cap + gates)

		// bench: split the blend's two effects (target mix with the rate cap / relaxation of the acceleration cap)
		internal static float PaceBlendTargetK = 1f;

		internal static float PaceBlendAccelK = 1f;

		public float coupHeadDbg;

		public float rcAOppDbg;

		public float rcTcDbg;

		public float rcSinHDbg;

		public float coupRollLim;

		private bool branchGateOn;

		internal static float BranchFailTime = 0.25f;

		internal static bool BranchFreeTvc = true;


		internal static bool BranchScaleToPeak = true;

		internal static bool BranchFullTravel = true;

		internal static float BranchDeepMargin = 5f;

		internal static float BranchCoolTime = 0.25f; // 2.0.1: 1 s -> 0.25 s (a stab could park deep in stall during the wait)

		internal static float BranchCoolTimeStop = 0.25f;

		internal static float LatTrimGain = 100f;

		internal static bool LatReservePitchFrac = true;
		public float latFloorDbg;
		internal static bool ReleaseCoordinate = true;   // 2.18.0: released roll under a capped nose-down sweep, fins gone: target = r / sin AoA
		internal static bool SweepHoldFade = true;       // 2.18.0: the nose waits above the rudder fade while excess yaw remains, fins gone
		internal static float SweepHoldRate = 12f;       // 2.19.0: velocity-roll rotation (deg/s) above which the nose waits, fins gone
		internal static float SweepHoldRateBlend = 6f;   // deg/s of blend above it
		internal static float SweepHoldStabFrac = 0.25f; // the hold stands while stabPitchFrac is below this (stabs off pitch)
		internal static bool PaceHoldPacedByRudder = true;   // 2.19.0: the window paces where the rudders are not the yaw axis (rudW < 1)
		public float sweepHoldDbg;
		public float pSStickRaw;
		internal static bool PaceNoRetreat = true;   // 2.18.0: the latched window never moves the target back toward the rotation
		internal static float PaceHoldCapFloor = 5f;   // deg/s^2


		public float capSweepDbg, capShedDbg;   // 2.23.0: the two AoA-rate caps on the nose-down sweep this tick (deg/s; 60 = open) - which one binds
		public float releaseBrakeTdDbg;   // 2.35.3: the device is deleted; the CSV column is kept (always 0)
		public float sweepDelF;   // 2.24.0: the delivered wind-yaw deceleration the sweep cap credited (deg/s^2, low-passed)
		public float stickRelW;   // 2.24.0: lateral stick released (1) .. held (0), over SweepNoReverseStick of stick travel
		public float relFollowDbg;   // 2.35.3: the device is deleted; the CSV column is kept (always 0)
		// 2.34.0 (PushGov): the lateral limit this tick (deg/s^2 of velocity-roll change allowed: < 0 the most the rotation may come down,
		// > 0 what it must grow; NaN = not in force), the rotation side, the roll capacity read (3-tick, TradeCapTau)
		public float pushGovL = float.NaN, pushGovU, pushGovA;
		private float pushGovSide, pushGovCapXF, pushGovCapYF, pushGovP0, pushGovGrowF;
		private bool pushGovXInit, pushGovActive;
		public float pushGovCeil, pushGovP0Dbg, pushGovAMin;
		private int reachLogPrev;
		public bool pushPlanOn; public float pushPlanQ, pushPlanQNow, pushPlanBleed, pushPlanT, pushPlanSide, pushPlanD, pushHeldB;

		public float aYawShedDbg;   // 2.22.0: the shed the cap paced to (deg/s^2)
		internal static bool LatLimitYawFloor = true;   // 2.16.0: the lateral limit's coordination floor (see num11)
		internal static float LatFloorSinMin = 0.26f;    // sin 15 deg

		public float aUpPitch;

		// Revoker 0.2.12: see PitchLaw (lateral reserve). false = 0.2.11.
		internal static bool LatReserveTvcFull = true;

		public float aDnPitch;


		internal static bool LeadAgreeGate = true;

		public float pitchPrio = 1f;

		public bool stabRateBound;

		private float tvcTrimRef;

		private float tvcSlow;

		private float[] cmdSave = new float[64];

		public bool SplitDeploy
		{
			get
			{
				return (Plugin.AirbrakeSplit || !Plugin.ThrottleIndependentAirbrake) && AirbrakeOut;
			}
		}

		private LandingGear NoseGear()
		{
			if (noseGearSearched)
			{
				return noseGearCached;
			}
			noseGearSearched = true;
			LandingGear landingGear = null;
			float num = float.MinValue;
			float num2 = float.MinValue;
			int num3 = 0;
			foreach (LandingGear gear in gears)
			{
				if (!(gear == null))
				{
					num3++;
					float num4 = Vector3.Dot(gear.transform.position - ((ac.rb != null) ? ac.rb.worldCenterOfMass : ((Component)(object)ac).transform.position), ((Component)(object)ac).transform.forward);
					if (num4 > num)
					{
						num2 = num;
						num = num4;
						landingGear = gear;
					}
					else if (num4 > num2)
					{
						num2 = num4;
					}
				}
			}
			noseGearCached = ((num3 >= 2 && num - num2 > 1.5f) ? landingGear : null);
			Plugin.Log.LogInfo((noseGearCached != null) ? $"Revoker FCS: nose gear '{noseGearCached.name}' ({num - num2:F1} m ahead of the next leg), {num3} gear legs." : $"Revoker FCS: no distinct nose gear among {num3} gear legs; rotation law disabled.");
			return noseGearCached;
		}

		public FcsController(Aircraft aircraft)
		{
			ac = aircraft;
			inputs = aircraft.GetInputs();
			AircraftParameters aircraftParameters = aircraft.GetAircraftParameters();
			foreach (ControlSurface item in Acc.Collect<ControlSurface>(aircraft))
			{
				AeroPart aeroPart = Acc.CS_attached(item) as AeroPart;
				if (!(aeroPart == null))
				{
					SurfaceKind kind = Classify(item);
					surfaces.Add(new SurfaceEffector(item, aeroPart, kind, FoilFor(aeroPart, aircraftParameters)));
				}
			}
			if (surfaces.Count < 5)
			{
				throw new InvalidOperationException("Revoker FCS: expected 5 control surfaces, found " + surfaces.Count);
			}
			// 2.2.0 flap-camber coupling: each rudder / flaperon / aileron knows the fixed part ahead of it
			Dictionary<string, AeroPart> partsByName = new Dictionary<string, AeroPart>();
			foreach (AeroPart apH in Acc.Collect<AeroPart>(aircraft))
			{
				partsByName[apH.gameObject.name.Replace("Aryx_KingRaptor_", "")] = apH;
			}
			foreach (SurfaceEffector seH in surfaces)
			{
				string hn = (seH.cs != null) ? Camber.HostOf(seH.cs.gameObject.name) : null;
				if (hn != null && partsByName.TryGetValue(hn, out var hp))
				{
					seH.host = hp;
					seH.hostFoil = FoilFor(hp, aircraftParameters);
					// Revoker 0.2.20: the camber increment on the host follows the elevon's own (staggered) curve
					StallStagger.Fam fH = StallStagger.Enabled ? StallStagger.FamilyOf(seH.cs.gameObject.name) : StallStagger.Fam.None;
					if (fH != StallStagger.Fam.None)
					{
						seH.hostFoil = StallStagger.TableForFamily(AirfoilOf(hp, aircraftParameters), fH);
					}
				}
			}
			foreach (SurfaceEffector surface in surfaces)
			{
				if (surface.kind == SurfaceKind.Stabilator)
				{
					surface.mu = 0.0002f;
					surface.linearPitch = true;
				}
				else
				{
					// 2.0.6: with RudderPitchAid the rudders keep their true pitch moment in the allocator's model (toe-in /
					// toe-out of the canted fins), so a symmetric rudder split is available as a pitch effector at a cost
					surface.noPitch = !(surface.kind == SurfaceKind.Rudder && Plugin.RudderPitchAid);
					// Revoker 0.1.0 (ElevonPitch): the delta's trailing-edge flaperons with their true pitch moment
					if (surface.kind == SurfaceKind.Aileron && Plugin.ElevonPitch)
					{
						surface.noPitch = false;
					}
				}
				surface.rollGroup = ((surface.kind == SurfaceKind.Stabilator) ? 1 : ((surface.kind == SurfaceKind.Aileron) ? 2 : ((surface.kind == SurfaceKind.Flaperon) ? 3 : 0)));
				surface.pitchGroup = ((surface.kind == SurfaceKind.Stabilator) ? 1 : 0);
				effs.Add(surface);
			}
			// 2.0.0: the roll-time nozzle budget and the roll-pitch pass fade out by body AoA at points chosen for Aryx's stab
			// foil (peak ~34 deg local). They stand for "the roll's down-going stab is reaching the flat top of its lift
			// curve", so they move with the stab's own modelled peak (StallStagger): 25 deg stabs -> ~8.5 deg lower.
			float stabPeak = float.MaxValue;
			foreach (SurfaceEffector sp0 in surfaces)
			{
				if (sp0.kind == SurfaceKind.Stabilator)
				{
					stabPeak = Mathf.Min(stabPeak, sp0.stallAoA);
				}
			}
			stabStallShift = (StabPeakShift && stabPeak < 90f) ? Mathf.Max(0f, StabPeakRef - stabPeak) : 0f;
			foreach (SurfaceEffector l in surfaces)
			{
				if (!l.name.EndsWith("_L"))
				{
					continue;
				}
				SurfaceEffector surfaceEffector = surfaces.Find((SurfaceEffector o) => o.kind == l.kind && o.name == l.name.Substring(0, l.name.Length - 2) + "_R");
				if (surfaceEffector == null)
				{
					continue;
				}
				bool num = l.kind == SurfaceKind.Rudder;
				float num2 = (num ? Mathf.Sign(l.origYawRange) : Mathf.Sign(l.origRollRange));
				float num3 = (num ? Mathf.Sign(surfaceEffector.origYawRange) : Mathf.Sign(surfaceEffector.origRollRange));
				if (num2 != 0f && num3 != 0f && num2 != num3)
				{
					l.partner = surfaceEffector;
					l.pairSign = num2;
					surfaceEffector.pairSign = num3;
					if (l.kind == SurfaceKind.Stabilator)
					{
						l.symCost = Plugin.StabTrimCost;
						l.antiCost = 0.01f;
					}
					else
					{
						l.symCost = (l.kind == SurfaceKind.Rudder && Plugin.RudderPitchAid) ? Plugin.RudderToeCost : ((l.kind == SurfaceKind.Aileron) ? Plugin.ElevonSymCost : 2f);
						l.antiCost = 0f;
					}
				}
			}
			tvc = new TvcEffector(Acc.Collect<Turbojet>(aircraft));
			effs.Insert(0, tvc);
			tvc.pitchGroup = 2;
			// Ifrit: split-flap decelerons and differential (roll) TVC as allocator effectors
			if (Plugin.IfritSplitYaw)
			{
				foreach (SurfaceEffector sf in surfaces)
				{
					if (Acc.CS_maxSplit(sf.cs) > 0f && sf.part != null)
					{
						SplitEffector spe = new SplitEffector(sf.cs, sf.part);
						spe.host = sf;
						effs.Add(spe);
						splits.Add(spe);
					}
				}
			}
			if (Plugin.IfritTvcRoll && tvc.engines.Count >= 2)
			{
				tvcd = new TvcDiffEffector(tvc);
				if (tvcd.max > 0.5f)
				{
					effs.Add(tvcd);
				}
				else
				{
					tvcd = null;
				}
			}
			// Revoker 0.1.0: yaw TVC (the single nozzle's yaw axis) as an allocator effector
			if (Plugin.YawTvc)
			{
				float vyMax = 0f;
				foreach (Turbojet ey in tvc.engines)
				{
					if (ey != null)
					{
						vyMax = Mathf.Max(vyMax, Mathf.Abs(Acc.TF_vec(ey).y));
					}
				}
				if (vyMax > 0.5f)
				{
					tvc.yawSeparate = true;
					tvcy = new TvcYawEffector(tvc);
					effs.Add(tvcy);
				}
			}
			foreach (AeroPart apF in Acc.Collect<AeroPart>(aircraft))
			{
				string fn = apF.gameObject.name.ToLowerInvariant();
				if (fn.StartsWith("vstab") || fn == "rudder")
				{
					finParts.Add(apF);
					finAreaTotal += Mathf.Max(0f, Acc.AP_wingArea(apF));
				}
			}
			gears.AddRange(Acc.Collect<LandingGear>(aircraft));
			try
			{
				foreach (Airbrake ab in Acc.Collect<Airbrake>(aircraft))
				{
					float dA = (fABDrag != null) ? Convert.ToSingle(fABDrag.GetValue(ab)) : 0f;
					Component pc = (fABPart != null) ? (fABPart.GetValue(ab) as Component) : null;
					Rigidbody rbA = (pc is UnitPart up) ? up.rb : null;
					if (rbA != null && dA > 0f)
					{
						airbrakes.Add(ab);
						abRb.Add(rbA);
						abDrag.Add(dA);
						abPart.Add(pc as UnitPart);
					}
				}
			}
			catch (Exception exAB)
			{
				Plugin.Log.LogWarning("Revoker FCS airbrake drag lookup: " + exAB.Message);
			}
			foreach (UnitPart item2 in aircraft.partLookup)
			{
				if (item2 is AeroPart aeroPart2)
				{
					aeroParts.Add(aeroPart2);
					if (!foils.ContainsKey(aeroPart2))
					{
						foils[aeroPart2] = FoilFor(aeroPart2, aircraftParameters);
					}
				}
			}
			RebuildBodies();
			flaps = new FlapSystem(aircraft, surfaces);
			// Revoker 0.2.0 (FlapFlaperons): flap_L/R, driven by the HLD only in the game, become allocator effectors (pitch and
			// lateral) with wing2_X as their camber host; the FCS's FlapSystem then leaves their two transforms alone
			if (Plugin.FlapFlaperons)
			{
				foreach (AeroPart apF in aeroParts)
				{
					string fn = apF.gameObject.name;
					if (!(fn == "flap_L" || fn == "flap_R"))
					{
						continue;
					}
					Transform visF = Acc.AP_liftNormal(apF);
					if (visF == null || visF == apF.transform)
					{
						continue;
					}
					AeroPart hostF = aeroParts.Find((AeroPart o) => o.gameObject.name == "wing2_" + fn.Substring(5));
					Transform hostLN = (hostF != null) ? Acc.AP_liftNormal(hostF) : null;
					if (hostLN == hostF?.transform)
					{
						hostLN = null;
					}
					// Revoker 0.2.20: with ElevonStallStagger the camber share on wing2 uses the inboard elevon's curve (wing2's airfoil, remapped)
					AirfoilTable hostTbl = (hostF != null && foils.TryGetValue(hostF, out var tH)) ? tH : null;
					StallStagger.Fam fF = StallStagger.Enabled ? StallStagger.FamilyOf(apF.gameObject.name) : StallStagger.Fam.None;
					if (hostF != null && fF != StallStagger.Fam.None)
					{
						hostTbl = StallStagger.TableForFamily(AirfoilOf(hostF, aircraftParameters), fF);
					}
					HingeFlapEffector hf = new HingeFlapEffector(apF, visF, hostF, hostLN, foils.TryGetValue(apF, out var tF) ? tF : FoilFor(apF, aircraftParameters), hostTbl, Plugin.FlapHingeRange, flaps.RestOf(visF), (hostLN != null) ? flaps.RestOf(hostLN) : null);
					if (hostTbl != null && fF != StallStagger.Fam.None)
					{
						CamberForce.AddHinge(ac, hf, hostTbl);
					}
					hf.servo = (Plugin.SurfaceServoSpeed > 0f) ? Plugin.SurfaceServoSpeed : 70f;
					hf.rollGroup = HingeRollGroup;
					hinges.Add(hf);
					effs.Add(hf);
					flaps.Own(visF);
					if (hostLN != null)
					{
						flaps.Own(hostLN);
					}
				}
				HingeFlapEffector hL = hinges.Find((HingeFlapEffector h) => h.left);
				HingeFlapEffector hR = hinges.Find((HingeFlapEffector h) => !h.left);
				if (hL != null && hR != null)
				{
					hL.partner = hR;
					hL.mate = hR;
					hR.mate = hL;
					hL.pairSign = -1f;
					hR.pairSign = 1f;
					hL.symCost = Plugin.ElevonSymCost * HingeSymCostK;
					hL.antiCost = 0f;
				}
			}
			Plugin.Log.LogInfo(string.Format("Revoker FCS attached: {0} surfaces [{1}], ", surfaces.Count, string.Join(", ", surfaces.ConvertAll((SurfaceEffector s) => s.name + ":" + s.kind.ToString() + ((s.min == -s.max) ? (" ±" + s.max.ToString("0")) : (" " + s.min.ToString("0") + "/+" + s.max.ToString("0")))))) + $"TVC engines {tvc.engines.Count} (±{tvc.max:0} deg pitch, roll {(tvcd != null ? "±" + tvcd.max.ToString("0") : "off")}, yaw {(tvcy != null ? "±" + tvcy.max.ToString("0") : "off")}), fin area {finAreaTotal:F1} m², splits {splits.Count}, inboard elevons {hinges.Count} (±{(hinges.Count > 0 ? hinges[0].max : 0f):0}), {bodies.Count} rigidbodies, {flaps.Describe()}.");
		}

		private static SurfaceKind Classify(ControlSurface cs)
		{
			string name = cs.gameObject.name;
			// Ifrit: the stock surfaces are named elevator_L / aileron_L / flap_L; match case-insensitively
			string lower = name.ToLowerInvariant();
			name = lower.Contains("elevator") ? "Elevator" : lower.Contains("rudder") ? "Rudder" : lower.Contains("flap") ? "Flap" : lower.Contains("aileron") ? "Aileron" : name;
			if (Mathf.Abs(Acc.CS_pitchRange(cs)) > 0.01f || name.Contains("Elevator") || name.Contains("Stab"))
			{
				return SurfaceKind.Stabilator;
			}
			if (name.Contains("Rudder"))
			{
				return SurfaceKind.Rudder;
			}
			if (name.Contains("Flap"))
			{
				return SurfaceKind.Flaperon;
			}
			if (name.Contains("Aileron"))
			{
				return SurfaceKind.Aileron;
			}
			if (Mathf.Abs(Acc.CS_yawRange(cs)) > Mathf.Abs(Acc.CS_rollRange(cs)))
			{
				return SurfaceKind.Rudder;
			}
			return SurfaceKind.Other;
		}

		private static Airfoil AirfoilOf(AeroPart part, AircraftParameters prm)
		{
			int k = Acc.AP_airfoil(part);
			return (k >= 0 && prm != null && prm.airfoils != null && k < prm.airfoils.Length) ? prm.airfoils[k] : null;
		}

		private static AirfoilTable FoilFor(AeroPart part, AircraftParameters prm)
		{
			Airfoil af = null;
			int num = Acc.AP_airfoil(part);
			if (num >= 0 && prm != null && prm.airfoils != null && num < prm.airfoils.Length)
			{
				af = prm.airfoils[num];
			}
			return StallStagger.TableFor(part, af);
		}

		private void RebuildBodies()
		{
			bodies.Clear();
			foreach (UnitPart item in ac.partLookup)
			{
				if (!(item == null) && !(item.rb == null) && !item.IsDetached() && !bodies.Contains(item.rb))
				{
					bodies.Add(item.rb);
				}
			}
			if (bodies.Count == 0 && ac.rb != null)
			{
				bodies.Add(ac.rb);
			}
		}

		public void Dispose()
		{
			telemetry?.Close();
			telemetry = null;
		}

		private void UpdateMassProps(bool full, Quaternion rootInv)
		{
			float num = 0f;
			Vector3 zero = Vector3.zero;
			Vector3 vSum = Vector3.zero;
			for (int num2 = bodies.Count - 1; num2 >= 0; num2--)
			{
				Rigidbody rigidbody = bodies[num2];
				if (rigidbody == null)
				{
					bodies.RemoveAt(num2);
				}
				else
				{
					num += rigidbody.mass;
					zero += rigidbody.mass * rigidbody.worldCenterOfMass;
					vSum += rigidbody.mass * rigidbody.velocity;
				}
			}
			if (num < 1f)
			{
				return;
			}
			mass = num;
			cg = zero / num;
			vCg = vSum / num;
			if (!full)
			{
				return;
			}
			double[,] array = new double[3, 3];
			foreach (Rigidbody body in bodies)
			{
				Matrix4x4 matrix4x = Matrix4x4.Rotate(body.rotation * body.inertiaTensorRotation);
				Vector3 inertiaTensor = body.inertiaTensor;
				Vector3 vector = body.worldCenterOfMass - cg;
				float sqrMagnitude = vector.sqrMagnitude;
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						double num3 = matrix4x[i, 0] * inertiaTensor.x * matrix4x[j, 0] + matrix4x[i, 1] * inertiaTensor.y * matrix4x[j, 1] + matrix4x[i, 2] * inertiaTensor.z * matrix4x[j, 2];
						num3 += (double)(body.mass * (((i == j) ? sqrMagnitude : 0f) - vector[i] * vector[j]));
						array[i, j] += num3;
					}
				}
			}
			Matrix4x4 matrix4x2 = Matrix4x4.Rotate(rootInv);
			double[,] array2 = new double[3, 3];
			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					double num4 = 0.0;
					for (int m = 0; m < 3; m++)
					{
						for (int n = 0; n < 3; n++)
						{
							num4 += (double)matrix4x2[k, m] * array[m, n] * (double)matrix4x2[l, n];
						}
					}
					array2[k, l] = num4;
				}
			}
			int[] array3 = new int[3] { 2, 0, 1 };
			float[] array4 = new float[3] { -1f, -1f, 1f };
			for (int num5 = 0; num5 < 3; num5++)
			{
				for (int num6 = 0; num6 < 3; num6++)
				{
					Ia[num5, num6] = (float)((double)(array4[num5] * array4[num6]) * array2[array3[num5], array3[num6]]);
				}
			}
			Invert3(Ia, IaInv);
			alloc.SetInverseInertia(IaInv);
		}

		private static void Invert3(float[,] a, float[,] o)
		{
			float num = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0]) + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
			if (Mathf.Abs(num) < 1E-06f)
			{
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						o[i, j] = ((i == j) ? (1f / Mathf.Max(1f, a[i, i])) : 0f);
					}
				}
				return;
			}
			float num2 = 1f / num;
			o[0, 0] = (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1]) * num2;
			o[0, 1] = (a[0, 2] * a[2, 1] - a[0, 1] * a[2, 2]) * num2;
			o[0, 2] = (a[0, 1] * a[1, 2] - a[0, 2] * a[1, 1]) * num2;
			o[1, 0] = (a[1, 2] * a[2, 0] - a[1, 0] * a[2, 2]) * num2;
			o[1, 1] = (a[0, 0] * a[2, 2] - a[0, 2] * a[2, 0]) * num2;
			o[1, 2] = (a[0, 2] * a[1, 0] - a[0, 0] * a[1, 2]) * num2;
			o[2, 0] = (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]) * num2;
			o[2, 1] = (a[0, 1] * a[2, 0] - a[0, 0] * a[2, 1]) * num2;
			o[2, 2] = (a[0, 0] * a[1, 1] - a[0, 1] * a[1, 0]) * num2;
		}

		private Vector3 MulI(Vector3 v)
		{
			return new Vector3(Ia[0, 0] * v.x + Ia[0, 1] * v.y + Ia[0, 2] * v.z, Ia[1, 0] * v.x + Ia[1, 1] * v.y + Ia[1, 2] * v.z, Ia[2, 0] * v.x + Ia[2, 1] * v.y + Ia[2, 2] * v.z);
		}

		private static float[] BuildAsymAlphas()
		{
			float[] array = new float[22];
			for (int i = 0; i < 22; i++)
			{
				array[i] = -40f + 5f * (float)i;
			}
			return array;
		}

		// 1.8.35: past the normal rudder fade (RudderFadeStartAoA-EndAoA) the canted rudders' "inverse roll" is what
		// moves sideslip: beta-dot = p sin(a) - r cos(a), and a yaw-right rudder command rolls the jet left, so both terms
		// push beta the same way. Bench sweep: from ~55 deg AoA their sideslip effect is >3x their velocity-vector roll
		// (6x at 75), while the flaperons' and stabs' sideslip authority collapses near 60 and reverses past ~62.
		// So above the fade the rudders get a limited range that can only REDUCE the sideslip error: deflection only in
		// the beta-reducing direction (taken from the live surface model, so it follows the sign flip at negative AoA),
		// scaled in by the error (0 below 0.5 deg, full at 3 deg), and capped so the body yaw acceleration the rudders make
		// stays within RudderSideslipYawCap. Their yaw per degree falls with AoA (about 0 by 80 deg), so the travel opens
		// up as AoA rises. Uses the previous tick's surface model and pedal-sideslip command.
		private void SideslipRudder(float dt)
		{
			rudSideDbg = 0f;
			float cap = Plugin.RudderSideslipYawCap;
			float gate = (1f - rudderAuth) * ((Mathf.Abs(alpha) < 100f) ? 1f : 0f);
			SurfaceEffector rl = null;
			SurfaceEffector rr = null;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Rudder)
				{
					if (se.name.EndsWith("_L"))
					{
						rl = se;
					}
					else if (se.name.EndsWith("_R"))
					{
						rr = se;
					}
				}
			}
			if (cap <= 0f || gate <= 0.001f || !Engaged || !havePrev || Mode == FcsMode.Ground || Mode == FcsMode.Off || rl == null || rr == null || !rl.active || !rr.active || rl.N < 2 || rr.N < 2)
			{
				rudSideF = Mathf.MoveTowards(rudSideF, 0f, 60f * dt);
				return;
			}
			float sL = Mathf.Sign(rl.origYawRange);
			float sR = Mathf.Sign(rr.origYawRange);
			const float probe = 10f;
			Vector3 m = rl.MAt(sL * probe) + rr.MAt(sR * probe) - rl.MAt(0f) - rr.MAt(0f);
			float pDot = AccRoll(m) / probe;
			float rDot = AccYaw(m) / probe;
			float ar = alpha * (MathF.PI / 180f);
			float bdd = pDot * Mathf.Sin(ar) - rDot * Mathf.Cos(ar);
			float err = beta - betaPedalCmd;
			float y = 0f;
			if (Mathf.Abs(bdd) > 1E-4f)
			{
				float span = Mathf.Min(Mathf.Max(rl.max, 0f - rl.min), Mathf.Max(rr.max, 0f - rr.min));
				float lim = Mathf.Min(span, cap / Mathf.Max(Mathf.Abs(rDot), 1E-4f));
				float use = Mathf.Clamp01((Mathf.Abs(err) - 0.5f) / 2.5f);
				y = (0f - Mathf.Sign(err) * Mathf.Sign(bdd)) * lim * use * gate;
			}
			// servo-rate-limited so the allowed range cannot jump across zero in one frame when the error changes sign
			rudSideF = Mathf.MoveTowards(rudSideF, y, Mathf.Max(1f, rl.rate) * dt);
			rudSideDbg = rudSideF;
			float yLo = Mathf.Min(0f, rudSideF);
			float yHi = Mathf.Max(0f, rudSideF);
			ApplyYawInterval(rl, sL, yLo, yHi);
			ApplyYawInterval(rr, sR, yLo, yHi);
		}

		private static void ApplyYawInterval(SurfaceEffector se, float sgn, float yLo, float yHi)
		{
			float a = sgn * yLo;
			float b = sgn * yHi;
			float lo = Mathf.Min(a, b);
			float hi = Mathf.Max(a, b);
			se.lo = Mathf.Clamp(Mathf.Min(se.lo, lo), se.min, se.max);
			se.hi = Mathf.Clamp(Mathf.Max(se.hi, hi), se.min, se.max);
		}

		// 1.8.36 fly-by-wire ground steering. Outer loop: pedal -> yaw-rate command (GroundYawRateMax, limited to
		// GroundLateralAccel / V); pedal released -> yaw braking (zero-rate command) until |r| < 0.7 deg/s, then capture the
		// ground track (or the heading when nearly stopped) and the line through the CG, and hold that line: desired track =
		// captured track minus atan(cross-track / max(4 s x V, 25 m)) (<= 5 deg), desired heading = desired track (the heading
		// is lined up with the track: a touchdown crab or a weathervaning crosswind is steered out, the mains' side force
		// removes the drift). Inner loop: nose-wheel angle = kinematic feed-forward atan(r L / V) + yaw-rate P+I, limited to
		// what the tyre can use at that speed (NO's tyre saturates at 1/(0.2 + 0.01 v) deg of slip); in the ground roll the
		// rudders run the same yaw-rate error. Below 1.5 m/s the pedal steers the wheel directly.
		private void GroundSteer(float dt, bool mainsWow, bool noseWowWeight)
		{
			// contact rather than weight: in the ground roll the F-22E's nose leg carries only ~15-20 kN (about 0.02 m of
			// its 0.7 m travel), below WeightOnWheel(0.05), yet the tyre still steers with that load
			bool noseWow = false;
			bool anyContact = mainsWow;
			foreach (LandingGear g in gears)
			{
				if (g != null && g.WeightOnWheel(0.002f))
				{
					anyContact = true;
					if (g == NoseGear())
					{
						noseWow = true;
					}
				}
			}
			gsAnyContact = anyContact;
			// human pilots only: AI taxi logic is written for the stock pedal-to-wheel-angle steering
			bool on = Plugin.NoseWheelFbw && !isAI && Engaged && Mode != FcsMode.Off && anyContact;
			if (!anyContact && ac.rb != null)
			{
				// airborne: remember the ground track (0.5 s filter) so a touchdown captures the approach track (the runway),
				// not the track the mains have already dragged toward a crabbed heading
				Vector3 va = ac.rb.velocity;
				if (va.x * va.x + va.z * va.z > 400f)
				{
					float ca = Mathf.Atan2(va.x, va.z) * 57.29578f;
					gsChiAir = ((gsAirT > 0f) ? (gsChiAir + Mathf.DeltaAngle(gsChiAir, ca) * (dt / (0.5f + dt))) : ca);
					gsAirT += dt;
				}
				else
				{
					gsAirT = 0f;
				}
			}
			if (!on)
			{
				gsActive = false;
				gsStockPhase = false;
				gsBetaPrev = float.NaN;
				gsRPrev = float.NaN;
				gsRdotF = 0f;
				gsFyF = 0f;
				gsIndiOn = false;
				gsInt = 0f;
				gsSinceContact += dt;
				if (gsSinceContact < 1.5f && Plugin.NoseWheelFbw && !isAI)
				{
					// a bounce: keep the captured line and track
					gsNwsDeg = 0f;
					gsRud = 0f;
					gsMainsContact = false;
					return;
				}
				gsHold = false;
				gsRudInt = 0f;
				gsBrakeT = 0f;
				gsRCmdMains = 0f;
				gsSurfV = Vector3.zero;
				gsSurfR = 0f;
				gsNwsDeg = 0f;
				gsRud = 0f;
				gsMainsContact = false;
				return;
			}
			LandingGear nose = NoseGear();
			float steerLock = ((nose != null) ? Mathf.Max(1f, Mathf.Abs(Acc.LG_steerLock(nose))) : 45f);
			if (gsLead < 0f)
			{
				gsLead = 5.9f;
				if (nose != null)
				{
					float best = float.MinValue;
					foreach (LandingGear g in gears)
					{
						if (g != null && g != nose)
						{
							best = Mathf.Max(best, Vector3.Dot(g.transform.position - nose.transform.position, -((Component)(object)ac).transform.forward));
						}
					}
					if (best > 1f && best < 30f)
					{
						gsLead = best;
					}
				}
			}
			Rigidbody rb = ac.rb;
			Vector3 v = ((rb != null) ? rb.velocity : Vector3.zero);
			// 0.5.4: everything on the ground relative to the surface under the wheels, as the game's tyres see it (a moving
			// carrier deck: its rigidbody's point velocity plus an AnimatedPhysicsSurface's velocity). With world velocity the
			// line hold captured the ship's track and steered the jet back toward the ship's heading on every release, and the
			// drift limit (on the aerodynamic sideslip) read the wind over the deck as a skid.
			GsSurface(nose);
			v -= gsSurfV;
			Vector3 vh = new Vector3(v.x, 0f, v.z);
			float V = vh.magnitude;
			Vector3 fwd = ((Component)(object)ac).transform.forward;
			float psi = Mathf.Atan2(fwd.x, fwd.z) * 57.29578f;
			float chi = ((V > 0.5f) ? (Mathf.Atan2(vh.x, vh.z) * 57.29578f) : psi);
			float r = ((rb != null) ? (Vector3.Dot(rb.angularVelocity, Vector3.up) * 57.29578f) : 0f);
			// slip over the surface (+ = moving right of the nose), for the drift limit
			gsSlip = ((V > 1f) ? Vector3.SignedAngle(Vector3.ProjectOnPlane(fwd, Vector3.up), vh, Vector3.up) : 0f);
			Vector3 pos = ((rb != null) ? rb.worldCenterOfMass : ((Component)(object)ac).transform.position);
			float rLim = Mathf.Min(Plugin.GroundYawRateMax, Plugin.GroundLateralAccel / Mathf.Max(V, 0.5f) * 57.29578f);
			float ped = sy;
			float rc;
			// GroundStockMatch (0.5.2): stock steering, a drift limit, and the straight-line correction. While the pedal is in,
			// and after release until the yaw has stopped, the nose leg, the rudders and the main legs' differential brake get
			// the stock fly-by-wire's yaw - clamp(yawTightness x remap x (pedal - yaw rate) + yawWeathervaning x sideslip) with
			// the jet's own ControlsFilter parameters, raw pedal below its minSpeed - exactly as stock steers. On top of it the
			// drift limit lets the sideslip reach GroundDriftAllow and counters what goes past it (sideslip led by its rate,
			// faded in over GroundDriftFadeSpeed .. +10 m/s, so tight taxi turns are untouched). Once the yaw has stopped
			// (|r| < 0.7 deg/s, or 2 s after release) the FCS holds the straight line along the track until the pedal moves;
			// the touchdown capture holds the approach track the same way.
			bool sm = Plugin.GroundStockMatch;
			if (sm)
			{
				ped = gsPedRaw;
			}
			if (float.IsNaN(gsBetaPrev) || dt <= 0f)
			{
				gsBetaDot = 0f;
			}
			else
			{
				gsBetaDot += (Mathf.DeltaAngle(gsBetaPrev, gsSlip) / dt - gsBetaDot) * Mathf.Clamp01(dt / 0.08f);
			}
			gsBetaPrev = gsSlip;
			// 0.5.4 INDI: measured yaw acceleration and the nose tyre's lateral force now, through the same low-pass so
			// the increment is taken between synchronous values
			float aIndi = (dt > 0f) ? (dt / (Mathf.Max(0.005f, Plugin.GroundIndiTau) + dt)) : 0f;
			gsRdotF += (((float.IsNaN(gsRPrev) || dt <= 0f) ? 0f : ((r - gsRPrev) / dt)) - gsRdotF) * aIndi;
			gsRPrev = r;
			bool indiOk = GsNoseTyre(nose, noseWow, out float fyNow, out float alphaC, out float kTyre, out float muN, out float yawPerN);
			gsFyF += (fyNow - gsFyF) * aIndi;
			bool indi = Plugin.GroundIndi && indiOk && V >= 1.5f;
			gsIndiOn = false;
			gsSinceContact = 0f;
			if (!gsActive && !gsHold && gsAirT > 0.3f && V > 20f && Mathf.Abs(ped) <= 0.05f)
			{
				// touchdown: hold the approach track through the touchdown point straight away (drift / crab correction)
				gsHold = true;
				gsChiRef = ((gsSurfV.sqrMagnitude > 1f) ? chi : gsChiAir);
				gsLineY = 0f;
				gsLineP = pos;
				gsLinePG = ac.GlobalPosition();
				gsRCmd = r;
			}
			if (anyContact)
			{
				gsAirT = 0f;
			}
			if (Mathf.Abs(ped) > 0.05f || V < 1.5f)
			{
				rc = (sm ? r : (ped * rLim));
				gsHold = false;
				gsBrakeT = 0f;
			}
			else
			{
				rc = 0f;
				if (!gsHold)
				{
					gsBrakeT += dt;
					if (Mathf.Abs(r - gsSurfR) < 0.7f || gsBrakeT > 2f)
					{
						gsHold = true;
						gsChiRef = ((V > 3f) ? chi : psi);
						gsLineY = 0f;
						gsLineP = pos;
						gsLinePG = ac.GlobalPosition();
					}
				}
				if (gsHold)
				{
					gsChiRef += gsSurfR * dt;
					float cr = gsChiRef * (MathF.PI / 180f);
					Vector3 right = new Vector3(Mathf.Cos(cr), 0f, 0f - Mathf.Sin(cr));
					// cross-track from the surface-relative velocity (2.4.12-2.4.13: from the global position, which on a moving
					// deck is the ship's own travel)
					gsLineY += Vector3.Dot(vh, right) * dt;
					float chiDes = gsChiRef - Mathf.Clamp(Mathf.Atan2(gsLineY * Mathf.Clamp01(Plugin.GroundLineHold), Mathf.Max(4f * V, 25f)) * 57.29578f, -5f, 5f);
					float e = Mathf.DeltaAngle(psi, chiDes);
					// corrections are not limited by the pedal's comfort limit (GroundLateralAccel), only by GsHoldRate
					float rHold = Mathf.Min(GsHoldRate, Plugin.GroundYawRateMax);
					rc = Mathf.Clamp(GsKpsi * e, 0f - rHold, rHold) + gsSurfR;
				}
			}
			// yaw-acceleration limit on the command so the nose tyre is not asked for more than it can grip at the entry
			// stock-matched: the release braking asks for zero rate at once (no yaw-acceleration limit); the line hold and the
			// touchdown capture keep the limit
			float rCmdPrev = gsRCmd;
			gsRCmd = ((sm && !gsHold) ? r : Mathf.MoveTowards(gsRCmd, rc, GsYawAccel * dt));
			float rCmdDot = ((sm && !gsHold) || dt <= 0f) ? 0f : ((gsRCmd - rCmdPrev) / dt);
			// 0.5.4: the yaw-rate command for the nose-up roll on the mains (gear-down law, below). 2.4.12-2.4.13 handed it
			// gsRCmd, which in the stock phase is the measured yaw rate, so the rate error was zero and the pedal did nothing
			// (27 Oct 12:58 log, 381-386 s: 71 m/s on the mains, theta 3.8, full left / right pedal, yaw rate 1.9 throughout).
			// Pedal: the capped rate of 1.8.36-2.4.11 (GroundLateralAccel / V); released: stop the yaw; holding: the hold's.
			gsRCmdMains = ((sm && !gsHold) ? Mathf.MoveTowards(gsRCmdMains, ((Mathf.Abs(ped) > 0.05f) ? (ped * rLim) : 0f) + gsSurfR, GsYawAccel * dt) : gsRCmd);
			rc = gsRCmd;
			float err = rc - r;
			if (sm && (Mathf.Abs(ped) > 0.05f || V < 1.5f || !gsHold))
			{
				// stock steering (pedal in, and after release until the line is captured) with the drift limit on top
				float ys = GsStockYaw(ped, V);
				gsStockYaw = ys;
				gsStockPhase = true;
				gsInt = 0f;
				gsRudInt = 0f;
				gsNwsDeg = (noseWow ? (ys * steerLock) : 0f);
				if (indi && Mathf.Abs(ped) <= 0.05f)
				{
					// 0.5.4 release braking: the nose wheel stops the yaw by INDI (rudders and differential brake stay stock)
					gsNwsDeg = GsIndiWheel((0f - Plugin.GroundIndiGain) * (r - gsSurfR), alphaC, kTyre, muN, yawPerN, steerLock, ped, V);
				}
				gsRud = ys;
				gsMainsContact = mainsContactNow(nose);
				gsActive = true;
				return;
			}
			gsStockPhase = false;
			if (indi)
			{
				// 0.5.4: INDI on the nose wheel - wanted yaw acceleration = GroundIndiGain x rate error + the command's slope
				gsInt = 0f;
				gsNwsDeg = GsIndiWheel(Plugin.GroundIndiGain * err + rCmdDot, alphaC, kTyre, muN, yawPerN, steerLock, ped, V);
			}
			else if (noseWow && V >= 1.5f)
			{
				float ff = Mathf.Atan(rc * (MathF.PI / 180f) * gsLead / Mathf.Max(V, 1f)) * 57.29578f;
				float kp = Mathf.Clamp(GsKp0 / Mathf.Max(V, 3f), GsKpMin, GsKpMax);
				float lim = Mathf.Min(steerLock, GsLimK / Mathf.Max(V, 1f));
				// NO's tyre: lateral force = clamp(slip x (0.2 + 0.01 v), +-1) x muN, so it saturates at 1 / (0.2 + 0.01 v) deg of
				// slip. Feedback beyond that only drives the wheel into a skid and winds the integrator up.
				float sat = GsSatMargin / (0.2f + 0.01f * V);
				// stock-matched release braking: proportional only (bench: the integrator swung the yaw back 1-2.5 deg/s)
				gsInt = ((sm && !gsHold) ? 0f : Mathf.Clamp(gsInt + GsKi * kp * err * dt, 0f - sat, sat));
				float fb = Mathf.Clamp(kp * err + gsInt, 0f - sat, sat);
				float blend = Mathf.InverseLerp(1.5f, 3f, V);
				gsNwsDeg = Mathf.Lerp(ped * steerLock, Mathf.Clamp(ff + fb, 0f - lim, lim), blend);
			}
			else if (noseWow)
			{
				gsInt = 0f;
				gsNwsDeg = ped * steerLock;
			}
			else
			{
				gsInt = 0f;
				gsNwsDeg = 0f;
			}
			float rq = Mathf.InverseLerp(10f, 25f, V);
			gsRudInt = ((rq > 0f && !(sm && !gsHold)) ? Mathf.Clamp(gsRudInt + GsKri * err * dt * rq, -0.5f, 0.5f) : 0f);
			// below taxi speed the rudders have no authority: they show the pedal, as stock
			// stock-matched, pedal released: the rudders on the (released) pedal with the yaw-rate loop trimming on top
			gsRud = (sm ? Mathf.Clamp(ped + rq * (GsKr * err + gsRudInt), -1f, 1f) : Mathf.Lerp(ped, Mathf.Clamp(GsKr * err + gsRudInt, -1f, 1f), rq));
			gsMainsContact = mainsContactNow(nose);
			gsActive = true;
		}

		public bool gsMainsContact;

		// 0.5.4 INDI nose wheel state: filtered yaw acceleration (deg/s^2) and nose-tyre lateral force (N), the yaw-rate
		// command for the nose-up-on-the-mains phase, debug values
		private float gsRPrev = float.NaN;

		private float gsRdotF;

		private float gsFyF;

		public float gsRCmdMains;

		private bool gsIndiOn;

		private float gsNuDbg;

		private float gsWheelDbg;

		// any wheel touching the ground this tick (set by GroundSteer, which runs every tick)
		public bool gsAnyContact;

		// 0.5.4 INDI nose wheel. The game's nose tyre (LandingGear.FixedUpdate): lateral force = clamp(clamp(slip, +-10) x
		// response x (0.2 + 0.01 |wheelSpeed|), +-1) x mu N, perpendicular to the contact velocity, slip = angle from the wheel to
		// the contact velocity. Its yaw moment about the CG divided by the yaw inertia is the wheel's yaw effectiveness; all
		// of it is read from the game each tick (leg load, friction, the FCS's own CG and inertia), so loads, fuel and CG
		// move it with them. The virtual control is that force: wanted = force now (filtered) + (wanted yaw acceleration -
		// measured, filtered) / effectiveness, so the mains, the rudders, the brakes, gusts and model errors all show up in the
		// measured yaw acceleration rather than having to be modelled.
		private bool GsNoseTyre(LandingGear nose, bool noseWow, out float fy, out float alphaC, out float k, out float muN, out float yawPerN)
		{
			fy = 0f;
			alphaC = 0f;
			k = 0.2f;
			muN = 0f;
			yawPerN = 0f;
			Rigidbody body = ac.rb;
			if (nose == null || body == null || !noseWow)
			{
				return false;
			}
			Transform tr = ((Component)(object)ac).transform;
			Vector3 up = tr.up;
			Vector3 p = nose.transform.position;
			RaycastHit hit = Acc.LG_hit(nose);
			// the contact point when it is current (a floating-origin shift leaves last frame's hit point behind)
			if (hit.collider != null && (hit.point - p).sqrMagnitude < 9f)
			{
				p = hit.point;
			}
			Vector3 vc = body.GetPointVelocity(p) - SurfaceVelAt(hit, p, out float _);
			Vector3 vp = Vector3.ProjectOnPlane(vc, up);
			float sp = vp.magnitude;
			if (sp < 0.5f)
			{
				return false;
			}
			// + = the contact moving right of the nose
			alphaC = Vector3.SignedAngle(Vector3.ProjectOnPlane(tr.forward, up), vp, up);
			float delta = Acc.LG_steerAngle(nose);
			k = Mathf.Max(0.05f, Acc.LG_response(nose)) * (0.2f + 0.01f * Mathf.Abs(Acc.LG_wheelSpeed(nose)));
			muN = Mathf.Max(0f, Acc.LG_compForce(nose) + Acc.LG_dampForce(nose)) * Acc.LG_friction(nose);
			// + = force to the right of the contact velocity
			fy = Mathf.Clamp(Mathf.Clamp(delta - alphaC, -10f, 10f) * k, -1f, 1f) * muN;
			Vector3 right = Vector3.Cross(up, vp / sp);
			float iz = Ia[2, 2];
			yawPerN = ((iz > 1f) ? (Vector3.Dot(Vector3.Cross(p - cg, right), up) / iz * 57.29578f) : 0f);
			gsWheelDbg = delta;
			return muN > 1f && yawPerN > 1E-06f;
		}

		// nose-wheel angle for a wanted yaw acceleration nu (deg/s^2): the INDI force increment, turned into a slip angle through the
		// tyre's slope and kept inside GsSatMargin x its saturation slip (more slip only skids and scrubs), plus the contact
		// velocity's angle. Below 1.5-3 m/s it blends to the pedal, as before.
		private float GsIndiWheel(float nu, float alphaC, float k, float muN, float yawPerN, float steerLock, float ped, float V)
		{
			float fDes = gsFyF + (nu - gsRdotF) / yawPerN;
			float sMax = GsSatMargin / k;
			float slip = Mathf.Clamp(fDes / (muN * k), 0f - sMax, sMax);
			float d = Mathf.Clamp(alphaC + slip, 0f - steerLock, steerLock);
			gsNuDbg = nu;
			gsIndiOn = true;
			return Mathf.Lerp(ped * steerLock, d, Mathf.InverseLerp(1.5f, 3f, V));
		}

		// 0.5.4: the velocity (and yaw rate, deg/s) of the surface the wheels stand on - a moving deck's rigidbody plus an
		// AnimatedPhysicsSurface, the same two the game's LandingGear takes off the contact velocity. Zero on fixed ground.
		public Vector3 gsSurfV;

		public float gsSurfR;

		private float gsSlip;

		private Collider gsSurfCol;

		private AnimatedPhysicsSurface gsSurfAps;

		private Vector3 SurfaceVelAt(RaycastHit hit, Vector3 p, out float yawRate)
		{
			yawRate = 0f;
			Collider col = hit.collider;
			if (col == null)
			{
				return Vector3.zero;
			}
			Vector3 vs = Vector3.zero;
			Rigidbody srb = col.attachedRigidbody;
			if (srb != null)
			{
				vs += srb.GetPointVelocity(p);
				yawRate = Vector3.Dot(srb.angularVelocity, Vector3.up) * 57.29578f;
			}
			if (col != gsSurfCol)
			{
				gsSurfCol = col;
				gsSurfAps = col.gameObject.GetComponent<AnimatedPhysicsSurface>();
			}
			if (gsSurfAps != null)
			{
				vs += gsSurfAps.GetVelocity();
			}
			return vs;
		}

		// the surface under the wheels this tick: a main leg in contact first, else the nose; kept through a bounce
		private void GsSurface(LandingGear nose)
		{
			LandingGear pick = null;
			foreach (LandingGear g in gears)
			{
				if (g != null && g.WeightOnWheel(0.002f) && (pick == null || pick == nose))
				{
					pick = g;
				}
			}
			if (pick == null)
			{
				return;
			}
			RaycastHit hit = Acc.LG_hit(pick);
			Vector3 p = pick.transform.position;
			if (hit.collider != null && (hit.point - p).sqrMagnitude < 16f)
			{
				p = hit.point;
			}
			gsSurfV = SurfaceVelAt(hit, p, out gsSurfR);
		}

		private bool mainsContactNow(LandingGear nose)
		{
			foreach (LandingGear g in gears)
			{
				if (g != null && g != nose && g.WeightOnWheel(0.002f))
				{
					return true;
				}
			}
			return false;
		}

		private static System.Reflection.FieldInfo fCfFbw, fCfMinSpeed, fCfMinAlt, fFbwEnabled, fFbwYawT, fFbwYawWv, fFbwCorner;

		private static bool fCfFailed;

		// the stock fly-by-wire's yaw (FlyByWire.Filter, read from the game's IL) for pedal ped, plus the drift limit
		private float GsStockYaw(float ped, float V)
		{
			float y = ped;
			try
			{
				if (stockCf != null && !fCfFailed)
				{
					if (fCfFbw == null)
					{
						Type t = typeof(ControlsFilter);
						fCfFbw = HarmonyLib.AccessTools.Field(t, "flyByWire");
						fCfMinSpeed = HarmonyLib.AccessTools.Field(t, "minSpeed");
						fCfMinAlt = HarmonyLib.AccessTools.Field(t, "minAlt");
						Type ft = fCfFbw.FieldType;
						fFbwEnabled = HarmonyLib.AccessTools.Field(ft, "Enabled");
						fFbwYawT = HarmonyLib.AccessTools.Field(ft, "yawTightness");
						fFbwYawWv = HarmonyLib.AccessTools.Field(ft, "yawWeathervaning");
						fFbwCorner = HarmonyLib.AccessTools.Field(ft, "cornerSpeed");
					}
					float spd = ac.speed;
					bool below = spd < (float)fCfMinSpeed.GetValue(stockCf) && ac.radarAlt < (float)fCfMinAlt.GetValue(stockCf);
					object fbw = fCfFbw.GetValue(stockCf);
					if (!below && fbw != null && (bool)fFbwEnabled.GetValue(fbw))
					{
						float T = (float)fFbwYawT.GetValue(fbw);
						float wv = (float)fFbwYawWv.GetValue(fbw);
						float corner = Mathf.Max(1f, (float)fFbwCorner.GetValue(fbw));
						float x = spd * spd * ac.airDensity / (corner * corner * 1.225f);
						float remap = 1f / Mathf.Max(x, 1f);
						Rigidbody body = ac.rb;
						float wy = (body != null) ? body.transform.InverseTransformDirection(body.angularVelocity).y : 0f;
						float k = T * remap * (wy - ped);
						if (wv > 0f && body != null)
						{
							Transform tr = ((Component)(object)ac).transform;
							k -= wv * Vector3.SignedAngle(Vector3.ProjectOnPlane(tr.forward, tr.up), Vector3.ProjectOnPlane(body.velocity, tr.up), tr.up);
						}
						y = Mathf.Clamp(0f - k, -1f, 1f);
					}
				}
			}
			catch (Exception e)
			{
				fCfFailed = true;
				Plugin.Log.LogWarning("ground steering: stock fly-by-wire parameters not readable, using the raw pedal (" + e.Message + ")");
				y = ped;
			}
			// drift limit: sideslip (led by its rate) past GroundDriftAllow is countered; beta > 0 = velocity right of the nose,
			// so a nose-right skid (beta < 0) is met with left yaw
			float fade = Mathf.InverseLerp(Plugin.GroundDriftFadeSpeed, Plugin.GroundDriftFadeSpeed + 10f, V);
			if (fade > 0f && Plugin.GroundDriftGain > 0f)
			{
				float bp = gsSlip + Plugin.GroundDriftLead * gsBetaDot;
				float allow = Mathf.Max(0f, Plugin.GroundDriftAllow);
				float over = bp - Mathf.Clamp(bp, 0f - allow, allow);
				y = Mathf.Clamp(y + fade * Plugin.GroundDriftGain * over, -1f, 1f);
			}
			return y;
		}

		// the main legs' differential brake reads the stock fly-by-wire's yaw in stock, not the pedal
		public bool StockBrakeYaw(out float yaw)
		{
			yaw = gsStockYaw;
			return gsActive && gsStockPhase;
		}

		public bool NoseSteerOverride(out float yaw)
		{
			LandingGear nose = NoseGear();
			float steerLock = ((nose != null) ? Mathf.Max(1f, Mathf.Abs(Acc.LG_steerLock(nose))) : 45f);
			yaw = Mathf.Clamp(gsNwsDeg / steerLock, -1f, 1f);
			return gsActive;
		}

		internal static float GsKpsi = 1f;

		internal static float GsHoldRate = 4f;

		internal static float GsYawAccel = 20f;

		internal static float GsSatMargin = 1.2f;

		internal static float GsKp0 = 15f;

		internal static float GsKpMin = 0.15f;

		internal static float GsKpMax = 3f;

		internal static float GsKi = 1f;

		internal static float GsLimK = 300f;

		internal static float GsKr = 0.1f;

		internal static float GsKphi = 3f;

		internal static float GsPhiFull = 10f;

		internal static float GsKri = 0.1f;

		private static float RudderAuthAt(float a)
		{
			float num = 1f - Mathf.Clamp01((Mathf.Abs(a) - Plugin.RudderFadeStart) / Mathf.Max(1f, Plugin.RudderFadeEnd - Plugin.RudderFadeStart));
			if (RudderRevBack)
			{
				num = Mathf.Max(num, Mathf.InverseLerp(120f, 140f, Mathf.Abs(a)));
			}
			return num;
		}

		private float AccRoll(Vector3 m)
		{
			return (IaInv[0, 0] * m.x + IaInv[0, 2] * m.z) * 57.29578f;
		}

		private float AccYaw(Vector3 m)
		{
			return (IaInv[2, 0] * m.x + IaInv[2, 2] * m.z) * 57.29578f;
		}

		private void AsymStep(float dt, float rho, Quaternion rootRot, ref EvalContext ctx, Vector3 wdF, Vector3 mF)
		{
			DecelStep(dt);
			if (havePrev)
			{
				Vector3 m = MulI(wdF) + Vector3.Cross(omega, MulI(omega)) - mF;
				// 2.0.12: on the wheels the runway takes whatever roll moment the surfaces make, so the 'unexplained' roll
				// is the gear's reaction, not an asymmetry: the estimates bleed to zero there (10:37 log: 0 -> 25 deg/s^2
				// on the take-off roll, carried into the climb)
				float rollIn = wheelsTouch ? 0f : AccRoll(m);
				dMeasR = dMeasRF.Step(rollIn, 0.3f, dt);
				if (Mathf.Abs(omega.x * 57.29578f) < 20f)
				{
					dTrimR = dTrimRF.Step(wheelsTouch ? 0f : (AccRoll(m) - Lp * omega.x * IaInv[0, 0] * 57.29578f), 0.5f, dt);
				}
				dMeasY = dMeasYF.Step(AccYaw(m), 0.3f, dt);
			}
			if (!Plugin.AsymLimiter)
			{
				asymCapPos = (asymCapPosEff = 999f);
				asymCapNeg = (asymCapNegEff = -999f);
				asymCapFilt = 999f;
				asymCapFiltNeg = -999f;
				asymValid = false;
				asymFF = Vector3.zero;
				return;
			}
			AsymPrepSym(rootRot);
			FlapSymStep(dt);
			// 0.4.1: entering the engine-out / damaged-engine state, the cap from the last sweep was judged with the thrust
			// term in (and below AsymMinIAS there is no next sweep to replace it): it is replaced by the fixed cap at once
			// and a fresh sweep (thrust term out) is started, so a damage at high AoA and low speed gets the fixed cap and
			// not a stale 12 deg.
			bool fixedNow = (engOut || engAsym) && EngineOutFixedCap;
			if (fixedNow && !engFixedPrev && asymValid)
			{
				float capF = engOut ? Plugin.EngineOutAoACap : Plugin.EngineDamagedAoACap;
				asymCapPos = capF;
				asymCapNeg = 0f - capF;
				PushCapSample(asymCapPos, asymCapNeg, reset: true);
				asymK = -1;
				asymTimer = 1f;
			}
			// leaving it because the thrust match took the split away: the limit is lifted at once to the aero-only cap
			// (the sweep's arrays were made without the thrust term) instead of relaxing up over 1.5 s - the "hitch" at 29 deg
			// on the way to the full AoA (23 Oct 00:00 / 00:06 logs: capEff 30 -> 55 in 1.5 s after each engagement)
			if (!fixedNow && engFixedPrev && asymValid && matchFull)
			{
				asymCapPos = AsymCap.Find(asymAlphas, asymU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax);
				asymCapNeg = AsymCap.Find(asymAlphas, asymU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax);
				PushCapSample(asymCapPos, asymCapNeg, reset: true);
				asymCapPosEff = asymCapPos;
				asymCapNegEff = asymCapNeg;
				tKHoldT = 1f;
			}
			if (!fixedNow)
			{
				tKHoldT = Mathf.Max(0f, tKHoldT - dt);
			}
			engFixedPrev = fixedNow;
			asymMNow = AsymMomentAt(alpha, rho, rootRot, ref ctx);
			SlipProbeStep_(dt, rho, rootRot, ref ctx);
			Vector3 vector = asymMF.Step(asymMNow, Mathf.Max(0.05f, Plugin.AsymFFWashout), dt);
			float num = Mathf.Max(0.01f, Plugin.AsymDeadband);
			float num2 = Mathf.Clamp01(Mathf.Max(Mathf.Abs(AccRoll(asymMNow)) / num, Mathf.Abs(AccYaw(asymMNow)) / (0.25f * num)) - 1f);
			float num3 = 1f - Mathf.Clamp01((Mathf.Abs(beta) - Plugin.AsymFFMaxBeta) / 12f);
			float num4 = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.AsymFFMaxAoA) / 15f);
			asymFFGate = num2 * num3 * num4;
			asymFF = (0f - Mathf.Clamp01(Plugin.AsymFeedForward)) * asymFFGate * new Vector3(asymMNow.x - vector.x, 0f, asymMNow.z - vector.z);
			asymFFr = AccRoll(asymFF);
			asymFFy = AccYaw(asymFF);
			asymTimer += dt;
			if (asymK < 0 && asymTimer > 0.2f && ias > Plugin.AsymMinIas)
			{
				asymTimer = 0f;
				AsymSweepPrep(rootRot);
				asymK = 0;
			}
			if (asymK >= 0)
			{
				int num5 = Mathf.Min(asymAlphas.Length, asymK + AsymPointsPerStep);
				AsymSweepPoints(asymK, num5, rho, rootRot, ref ctx);
				asymK = ((num5 < asymAlphas.Length) ? num5 : (-1));
			}
			if (asymK < 0 && asymCycleDone)
			{
				asymCycleDone = false;
				asymCapPos = AsymCap.Find(asymAlphas, asymU, asymAlphas.Length, 1, Plugin.AoAPos, Mathf.Min(Plugin.AsymCapFloor, Plugin.AoAPos), AsymCapUMax);
				asymCapNeg = AsymCap.Find(asymAlphas, asymU, asymAlphas.Length, -1, Plugin.AoANeg, 0f - Mathf.Min(Plugin.AsymCapFloorNeg, 0f - Plugin.AoANeg), AsymCapUMax);
				if (engOut && EngineOutFixedCap)
				{
					asymCapPos = Mathf.Min(asymCapPos, Plugin.EngineOutAoACap);
					asymCapNeg = Mathf.Max(asymCapNeg, 0f - Plugin.EngineOutAoACap);
				}
				else if (engAsym && EngineOutFixedCap)
				{
					// Ifrit 0.4.1: a weak (damaged, running) engine gets its own fixed cap, not the dynamic thrust cap
					asymCapPos = Mathf.Min(asymCapPos, Plugin.EngineDamagedAoACap);
					asymCapNeg = Mathf.Max(asymCapNeg, 0f - Plugin.EngineDamagedAoACap);
				}
				asymCapPitch = 999f;
				float capGate = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan);
				if (Plugin.AsymPitchMargin > 0f && capGate > 0.5f && asymCapPos < Plugin.AoAPos - 0.5f && Ia[1, 1] > 1f)
				{
					float tvDn = 0f;
					if (tvc.active && tvc.M != null)
					{
						float tvLo = float.MaxValue;
						for (int tk = 0; tk < tvc.N; tk++)
						{
							tvLo = Mathf.Min(tvLo, tvc.M[tk].y);
						}
						if (tvLo < float.MaxValue)
						{
							tvDn = tvLo * IaInv[1, 1] * 57.29578f;
						}
					}
					float prevA = 0f;
					float prevM = float.NaN;
					for (int ia = 0; ia < asymAlphas.Length; ia++)
					{
						float aA = asymAlphas[ia];
						if (aA < 0f)
						{
							continue;
						}
						float mDn = 0f - (asymNetDn[ia] + tvDn);
						if (mDn < Plugin.AsymPitchMargin)
						{
							float aP = (float.IsNaN(prevM) || prevM <= mDn) ? aA : Mathf.Lerp(prevA, aA, (prevM - Plugin.AsymPitchMargin) / (prevM - mDn));
							asymCapPitch = Mathf.Max(AsymPitchCapFloor, aP - AsymPitchCapLead);
							break;
						}
						prevA = aA;
						prevM = mDn;
					}
					asymCapPos = Mathf.Min(asymCapPos, asymCapPitch);
				}
				AuthCapFromSweep();
				PushCapSample(asymCapPos, asymCapNeg, !asymValid);
				if (!asymValid)
				{
					asymCapPosEff = asymCapPos;
					asymCapNegEff = asymCapNeg;
					asymValid = true;
				}
				if (!asymFirstLogged)
				{
					asymFirstLogged = true;
					Plugin.Log.LogInfo($"Revoker FCS: asymmetric-load check at {ias:F0} m/s IAS - lateral CG offset {asymDy * 100f:F1} cm, lateral usage at 30/45/60 deg AoA " + $"{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 30f):F2}/{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 45f):F2}/{AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 60f):F2} " + $"(cap engages above 1.00); roll/yaw authority there {AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 30f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 30f):F0}, " + $"{AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 45f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 45f):F0}, " + $"{AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 60f):F0}/{AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 60f):F0} deg/s^2; " + $"load accel at 45 deg {AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, 45f):F1}/{AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, 45f):F1} deg/s^2 (deadband {Plugin.AsymDeadband:F1}); AoA cap {asymCapPos:F1}/{asymCapNeg:F1} deg.");
				}
			}
			if (asymValid)
			{
				CapFilterStep(dt);
				float upRate = Plugin.AsymRelax * (1f + Mathf.Max(0f, Plugin.AsymRelaxBoost) * (1f - Mathf.Clamp01(AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, asymCapPosEff))));
				float tightR = Plugin.AsymTighten * ((engOut || engAsym) ? Mathf.Max(1f, EngOutTightenK) : 1f);
				asymCapPosEff = AsymCap.Slew(asymCapPosEff, asymCapFilt, 1, tightR, upRate, dt);
				asymCapNegEff = AsymCap.Slew(asymCapNegEff, asymCapFiltNeg, -1, tightR, upRate, dt);
				asymUNow = AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, alpha);
				asymArNow = AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, alpha);
				asymAyNow = AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, alpha);
				asymCrNow = AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, alpha);
				asymCyNow = AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, alpha);
				FuelTrimStep(dt, rootRot);
				asymLogTimer += dt;
				bool flag = asymCapPos < Plugin.AoAPos - 0.1f || asymCapNeg > Plugin.AoANeg + 0.1f;
				bool flag2 = asymLoggedCap < Plugin.AoAPos - 0.1f || asymLoggedCapN > Plugin.AoANeg + 0.1f;
				if (asymLogTimer > 2f && (flag != flag2 || (flag && (Mathf.Abs(asymCapPos - asymLoggedCap) > 3f || Mathf.Abs(asymCapNeg - asymLoggedCapN) > 3f))))
				{
					asymLogTimer = 0f;
					asymLoggedCap = asymCapPos;
					asymLoggedCapN = asymCapNeg;
					Plugin.Log.LogInfo(flag ? $"Revoker FCS: asymmetric load - lateral CG offset {asymDy * 100f:F1} cm, AoA cap {asymCapPos:F1}/{asymCapNeg:F1} deg at {ias:F0} m/s IAS (lateral demand at 30 deg: {AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 30f):F2}, at 60: {AsymCap.Interp(asymAlphas, asymU, asymAlphas.Length, 60f):F2}; load accel at 45 deg {AsymCap.Interp(asymAlphas, asymAr, asymAlphas.Length, 45f):F1} roll / {AsymCap.Interp(asymAlphas, asymAy, asymAlphas.Length, 45f):F1} yaw, authority there {AsymCap.Interp(asymAlphas, asymCr, asymAlphas.Length, 45f):F0} / {AsymCap.Interp(asymAlphas, asymCy, asymAlphas.Length, 45f):F0} deg/s^2)" : $"Revoker FCS: asymmetric-load AoA cap released (lateral CG offset {asymDy * 100f:F1} cm)");
				}
			}
		}

		// ---- 1.8.45: IAS-decay prediction and asymmetric-cap filter ----
		public float iasDotF;

		public float qPredK = 1f;

		public float latPredK = 1f;

		public float asymCapFilt = 999f;

		public float asymCapFiltNeg = -999f;

		private float iasPrevD = -1f;

		private readonly float[] capHist = new float[3];

		private readonly float[] capHistN = new float[3];

		private int capHistI;

		private float capMed = 999f;

		private float capMedNeg = -999f;

		internal static float BrakeRefIas => Plugin.YawBrakeRefKmh / 3.6f;   // 1.9.2: 270 km/h (was 280): the reference manoeuvre's IAS

		// 1.9.2: below YawMarginIasHi the yaw braking margin grows smoothly (smoothstep) to YawMarginLow x at YawMarginIasLo
		internal static float YawMarginIasHi => Plugin.YawMarginStartKmh;

		internal static float YawMarginIasLo => Plugin.YawMarginFullKmh;

		internal static float YawMarginLow => Plugin.YawMarginLowIas;

		internal static float BrakeSoftMin = 0.12f;      // soft-min width, fraction of the limits' mean

		public float yawMarginDbg = 1f;

		public float yawMidDbg = 1f;

		internal static float RollStopTau = 0.5f;

		internal static float ProtOnsetGain = 8f;

		internal static float ProtFlagBand = 1.5f;

		private float rollStopAF = -1f;

		// C1-smooth minimum: equals min(a, b) once they differ by more than k, at most k/4 below it where they cross
		private static float SoftMin(float a, float b, float k)
		{
			if (k <= 1e-4f)
			{
				return Mathf.Min(a, b);
			}
			float h = Mathf.Clamp01(0.5f + 0.5f * (b - a) / k);
			return Mathf.Lerp(b, a, h) - k * h * (1f - h);
		}

		internal static float DecelTau = 0.25f;          // s, IAS-rate filter

		internal static float DecelLook => Plugin.DecelLookahead;   // s, how far ahead the decay is projected

		internal static float Decel0 => Plugin.DecelStart;          // m/s^2: below this, no anticipation

		internal static float Decel1 => Plugin.DecelFull;           // m/s^2: full weight from here (quadratic in between)

		internal static float DecelKMin = 0.35f;

		internal static float AsymResMaxFrac = 0.8f;

		// 1.8.45: demand at the cap (was 1.0). In-game, with the light mixed load (6.5 cm lateral CG offset, 60-110 m/s)
		// the demand rises ~0.025 per degree above the cap, so 1.08 is ~3 deg more AoA there; heavier loads rise faster.
		internal static float AsymCapUMax => Plugin.AsymCapDemand;

		internal static float AsymCapTauUp => Plugin.AsymCapSmoothing;

		internal static float AsymCapTauDown = 0.3f;

		internal static bool LatBrakeFloor => Plugin.HighAoABrakeFloor;

		internal static float LatBrakeA0 = 25f;

		internal static float LatBrakeA1 = 35f;

		internal static float LatBrakeMinRate = 8f;

		private void DecelStep(float dt)
		{
			if (dt <= 0f)
			{
				return;
			}
			if (iasPrevD < 0f || !Engaged)
			{
				iasPrevD = ias;
				iasDotF = 0f;
				qPredK = 1f;
				return;
			}
			float d = (ias - iasPrevD) / dt;
			iasPrevD = ias;
			iasDotF += (Mathf.Clamp(d, -80f, 80f) - iasDotF) * Mathf.Clamp01(dt / Mathf.Max(0.02f, DecelTau));
			float dec = Mathf.Max(0f, 0f - iasDotF);
			float w = Mathf.Clamp01((dec - Decel0) / Mathf.Max(0.1f, Decel1 - Decel0));
			float iasP = Mathf.Max(ias - dec * w * w * DecelLook, 0.1f * ias);
			float r = (ias > 1f) ? (iasP / ias) : 1f;
			qPredK = Mathf.Clamp(r * r, DecelKMin, 1f);
		}

		private void PushCapSample(float pos, float neg, bool reset)
		{
			if (reset)
			{
				for (int i = 0; i < 3; i++)
				{
					capHist[i] = pos;
					capHistN[i] = neg;
				}
				capMed = (asymCapFilt = pos);
				capMedNeg = (asymCapFiltNeg = neg);
				return;
			}
			capHist[capHistI] = pos;
			capHistN[capHistI] = neg;
			capHistI = (capHistI + 1) % 3;
			capMed = Med3(capHist[0], capHist[1], capHist[2]);
			capMedNeg = Med3(capHistN[0], capHistN[1], capHistN[2]);
		}

		private static float Med3(float a, float b, float c)
		{
			return Mathf.Max(Mathf.Min(a, b), Mathf.Min(Mathf.Max(a, b), c));
		}

		private void CapFilterStep(float dt)
		{
			// tightening passes quickly, relaxing slowly: noise can't lift the cap, a real drop is followed in ~0.3 s
			float kU = Mathf.Clamp01(dt / Mathf.Max(0.05f, AsymCapTauUp));
			float kD = Mathf.Clamp01(dt / Mathf.Max(0.05f, AsymCapTauDown));
			asymCapFilt += (capMed - asymCapFilt) * ((capMed < asymCapFilt) ? kD : kU);
			asymCapFiltNeg += (capMedNeg - asymCapFiltNeg) * ((capMedNeg > asymCapFiltNeg) ? kD : kU);
		}

		private void AsymPrepSym(Quaternion rootRot)
		{
			Vector3 rhs = rootRot * Vector3.right;
			Vector3 zero = Vector3.zero;
			int num = 0;
			foreach (SurfaceEffector surface in surfaces)
			{
				if (surface.partner != null && surface.active && surface.partner.active && surface.partner is SurfaceEffector surfaceEffector && !(surface.part.rb == null) && !(surfaceEffector.part.rb == null))
				{
					zero += 0.5f * (surface.part.rb.worldCenterOfMass + surfaceEffector.part.rb.worldCenterOfMass);
					num++;
				}
			}
			// Revoker 0.2.1: with no intact pair left the reference is the airframe's centreline (the root transform; the
			// Revoker's parts are mirrored about it); 0.2.0 reported 0
			asymDy = ((num > 0) ? Vector3.Dot(cg - zero / num, rhs) : Vector3.Dot(cg - ((Component)(object)ac).transform.position, rhs));
			float[] array = asymDsym;
			if (array == null || array.Length != surfaces.Count)
			{
				array = (asymDsym = new float[surfaces.Count]);
			}
			for (int i = 0; i < surfaces.Count; i++)
			{
				array[i] = 0f;
			}
			for (int j = 0; j < surfaces.Count; j++)
			{
				SurfaceEffector surfaceEffector2 = surfaces[j];
				if (surfaceEffector2.partner != null)
				{
					int num2 = ((surfaceEffector2.partner is SurfaceEffector item) ? surfaces.IndexOf(item) : (-1));
					if (num2 >= 0)
					{
						float num3 = surfaceEffector2.pairSign * surfaceEffector2.now;
						float num4 = surfaceEffector2.partner.pairSign * surfaceEffector2.partner.now;
						float num5 = 0.5f * (num3 + num4);
						array[j] = surfaceEffector2.pairSign * (num3 - num5);
						array[num2] = surfaceEffector2.partner.pairSign * (num4 - num5);
					}
				}
			}

			if (surfaceParts.Count != 0)
			{
				return;
			}
			foreach (SurfaceEffector surface2 in surfaces)
			{
				if (surface2.part != null)
				{
					surfaceParts.Add(surface2.part);
				}
			}
			// Revoker 0.2.0: the inboard elevons are modelled by their effector too
			foreach (HingeFlapEffector hP in hinges)
			{
				if (hP.part != null)
				{
					surfaceParts.Add(hP.part);
				}
			}
		}

		// Revoker 0.2.0: a part's lift-normal rotation for the airframe models: an inboard elevon's camber host without the
		// camber the elevon's effector already models
		private Quaternion AirframeRot(AeroPart ap, Transform ln)
		{
			for (int i = 0; i < hinges.Count; i++)
			{
				if (hinges[i].host == ap && hinges[i].hostLN == ln)
				{
					return hinges[i].HostRot0;
				}
			}
			return ln.rotation;
		}

		// Revoker 0.2.4: gravity's sideslip feed-forward fade (TAS, m/s) and cap (deg/s); F-22E / Ifrit: 40, 20, none
		// Revoker 0.2.5: 0.6 (see the authority sweep)
		internal static float ElevonAuthShare = 0.6f;

		// 0.2.4 defaults: down to 18 -> 28 m/s TAS, capped at 30 deg/s (bench corkscrews: sideslip 7.4-9.0 -> 4.0-5.7 deg)
		internal static float GravFFTas0 = 18f;

		internal static float GravFFSpan = 10f;

		internal static float GravFFMax = 30f;

		internal static float LimTau = 0.4f;

		private float pLimF = -1f;

		private float rLimF;

		internal static float CorkYawOmega = 0f;

		internal static float CorkYawAccel = 120f;

		internal static float CorkShapeA0 = 30f;

		private float corkR;

		private float corkRd;

		public float corkWDbg;

		internal static float CorkAoA0 = 35f;

		internal static float CorkTolClose = 8f;

		public float corkTolNow;

		private float RudderHoldGate()
		{
			float use = -1f;
			foreach (SurfaceEffector rS in surfaces)
			{
				if (rS.kind == SurfaceKind.Rudder && rS.active)
				{
					float lim = (rS.cmdOut >= 0f) ? Mathf.Max(1f, rS.hi) : Mathf.Max(1f, 0f - rS.lo);
					use = Mathf.Max(use, Mathf.Abs(rS.cmdOut) / lim);
				}
			}
			return (use < 0f) ? 0f : (1f - Mathf.Clamp01((use - 0.55f) / 0.3f));
		}

		// 2.5.15: the whole airframe's aero moment (aero frame: x roll, z yaw) at a hypothetical AoA + sideslip, the surfaces at
		// their present deflection. Same per-part law as AsymMomentAt (the game's AeroJob_Math per AeroPart, the FCS's foil tables).
		private Vector3 SlipMomentAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			return SlipMomentAt(aDeg, bDeg, rho, rootRot, ref ctx, 0f);
		}

		// Revoker 0.4.0: the same with a body yaw rate rYaw (rad/s, aero z): every part and surface sees its own velocity, v + omega x r,
		// so the difference to rYaw = 0 is the airframe's yaw damping at this flow - what a nose-high fuselage does to a rotation by
		// itself (the rate cap's reversal model had only the effectors braking it). rYaw = 0 is the 2.5.15 function exactly.
		private Vector3 SlipMomentAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx, float rYaw)
		{
			float num = Mathf.Max(tas, 1f);
			float fa = aDeg * (MathF.PI / 180f);
			float fb = bDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(Mathf.Sin(fb) * num, (0f - Mathf.Sin(fa)) * Mathf.Cos(fb) * num, Mathf.Cos(fa) * Mathf.Cos(fb) * num);
			Vector3 omegaW = (rYaw != 0f) ? (rootRot * new Vector3(0f, rYaw, 0f)) : Vector3.zero;
			Vector3 zero = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				if (!(transform == null))
				{
					Quaternion rotation = transform.rotation;
					Vector3 lhs = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 vP = (rYaw != 0f) ? (vector + Vector3.Cross(omegaW, lhs)) : vector;
					float vPm = Mathf.Max(vP.magnitude, 1f);
					Vector3 rhs = -(vP / vPm) * (0.5f * (0.5f * rho * vPm * vPm) * Acc.AP_dragArea(aeroPart));
					if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
					{
						rhs += PartForce(vP, rotation, rotation * Vector3.right, value, 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart));
					}
					zero += Vector3.Cross(lhs, rhs);
				}
			}
			Vector3 vector3 = Quaternion.Inverse(rootRot) * zero;
			Vector3 result = new Vector3(0f - vector3.z, 0f - vector3.x, vector3.y);
			for (int i = 0; i < surfaces.Count; i++)
			{
				if (surfaces[i].active)
				{
					Vector3 vS = vector;
					if (rYaw != 0f && surfaces[i].part != null && surfaces[i].part.rb != null)
					{
						vS = vector + Vector3.Cross(omegaW, surfaces[i].part.rb.worldCenterOfMass - cg);
					}
					result += surfaces[i].EvalFlow(surfaces[i].now, vS, rho, ref ctx);
				}
			}
			return result;
		}

		// 2.31.2: the whole airframe's aero moment (aero frame: x roll, z yaw) at a hypothetical CG air velocity vAir and body rotation
		// w (world, rad/s), every aero part evaluated as the game's AeroJob_Math does it (the bench plant's Forces): the flow at the part
		// vAir + w x arm, lift and drag from the part's table, the parasitic drag, the transonic factor, the stabs' downwash on the root
		// body's flow, the per-part force clamp (30 x mass x |v|), and the flap-camber increment as CamberForce applies it. Control
		// surfaces sit where their transforms are now; no thrust. SlipMomentAt (2.5.15) differs: uniform flow, surfaces through their
		// effector model, no clamp - it over-reads the sideslip stiffness 2-15x above 40 deg AoA (claude/Passive_lateral_stiffness_finding.md).
		private Vector3 AirframeMomentAt(Vector3 vAir, Vector3 w, float rho, Quaternion rootRot)
		{
			Vector3 vRootRef = vAir + Vector3.Cross(w, ac.rb.worldCenterOfMass - cg);
			Vector3 m = Vector3.zero;
			foreach (AeroPart ap in aeroParts)
			{
				if (ap == null || ap.rb == null || ap.IsDetached())
				{
					continue;
				}
				Transform lt = Acc.AP_liftNormal(ap);
				if (lt == null)
				{
					continue;
				}
				Vector3 com = ap.rb.worldCenterOfMass;
				Vector3 v = vAir + Vector3.Cross(w, com - cg);
				float v2 = v.sqrMagnitude;
				if (v2 < 1f)
				{
					continue;
				}
				float vmag = Mathf.Sqrt(v2);
				Quaternion lr = lt.rotation;
				Vector3 vf = Downwash.IsStabName(ap.name) ? Downwash.Apply(v, rootRot, vRootRef) : v;
				Vector3 v6 = Quaternion.Inverse(lr) * vf;
				float a = Mathf.Atan2(v6.y, v6.z);
				float cl, cd;
				if (foils.TryGetValue(ap, out var foil) && foil != null)
				{
					foil.Get(a, out cl, out cd);
				}
				else
				{
					cl = 1.8f * Mathf.Sin(5f * a);
					cd = 1.5f * (1f - Mathf.Cos(2f * a)) + 0.02f;
				}
				float area = Acc.AP_wingArea(ap), eff = Acc.AP_wingEff(ap), dA = Acc.AP_dragArea(ap);
				float n8 = (dA + area * 0.1f) * (1f - eff);
				float n9 = 0.5f * rho * v2 * 0.5f * (dA + n8);
				float n10 = cd * rho * v2 * 0.5f * area * eff;
				float tr = DragReliefMath.Transonic(vmag / DragReliefMath.SpeedOfSound(com.y));
				Vector3 vfHat = vf.normalized, vHat = v / vmag;
				Vector3 f = (-vfHat * n10 - vHat * n9) * tr;
				// the buffed profile's drag relief (DragRelief hook): part of the foil drag above its minimum, by part AoA and Mach
				if (Plugin.DragReliefAoA > 0f || Plugin.DragReliefAoAHigh > 0f || Plugin.DragReliefTransonic > 0f)
				{
					if (!psCdMin.TryGetValue(ap, out float cdMin))
					{
						cdMin = float.MaxValue;
						if (foil != null)
						{
							for (float aa = -0.35f; aa <= 0.35f; aa += 0.005f)
							{
								foil.Get(aa, out _, out var cdv);
								cdMin = Mathf.Min(cdMin, cdv);
							}
						}
						if (cdMin == float.MaxValue)
						{
							cdMin = 0f;
						}
						psCdMin[ap] = cdMin;
					}
					float dRel = DragReliefMath.Relief(n9, n10, Mathf.Min(cdMin, cd) * rho * v2 * 0.5f * area * eff, vmag / DragReliefMath.SpeedOfSound(com.y), a * 57.29578f);
					if (dRel > 0f)
					{
						f += vHat * dRel;
					}
				}
				Vector3 nrm = Vector3.Cross(vf, lr * Vector3.right);
				float nm = nrm.magnitude;
				if (nm > 0.0001f)
				{
					f += -nrm / nm * (cl * rho * v2 * 0.5f * area * eff);
				}
				f = Vector3.ClampMagnitude(f, vmag * 0.5f * ap.mass * 60f);
				m += Vector3.Cross(com + lr * Acc.AP_centerOfLift(ap) - cg, f);
			}
			// flap-camber coupling, as CamberForce: the host's force increment for its surface's present deflection, unclamped
			if (psCamber == null)
			{
				psCamber = new List<(ControlSurface, AeroPart, AirfoilTable)>();
				AircraftParameters prm = ac.GetAircraftParameters();
				Dictionary<string, AeroPart> byName = new Dictionary<string, AeroPart>();
				foreach (AeroPart ap in aeroParts)
				{
					if (ap != null)
					{
						byName[ap.gameObject.name.Replace("Aryx_KingRaptor_", "")] = ap;
					}
				}
				foreach (ControlSurface cs in Acc.Collect<ControlSurface>(ac))
				{
					string hn = Camber.HostOf(cs.gameObject.name);
					if (hn == null || !byName.TryGetValue(hn, out var host))
					{
						continue;
					}
					int idx = Acc.AP_airfoil(host);
					Airfoil af = (idx >= 0 && prm != null && prm.airfoils != null && idx < prm.airfoils.Length) ? prm.airfoils[idx] : null;
					if (af == null || af.liftCoef == null || af.dragCoef == null || Acc.CS_visible(cs) == null)
					{
						continue;
					}
					psCamber.Add((cs, host, StallStagger.TableFor(host, af)));
				}
			}
			foreach (var (cs, host, table) in psCamber)
			{
				if (cs == null || host == null || host.rb == null || host.IsDetached() || !Acc.JobCreated(cs))
				{
					continue;
				}
				ref NuclearOption.Jobs.ControlSurfaceFields job = ref Acc.Job(cs);
				if (job.IsDetached)
				{
					continue;
				}
				float k = Camber.KForName(cs.gameObject.name);
				Transform lt = Acc.AP_liftNormal(host);
				if (k <= 0f || lt == null)
				{
					continue;
				}
				Transform vis = Acc.CS_visible(cs).transform;
				float delta = Camber.DeflectionOf(vis, job.restingRotation);
				Vector3 com = host.rb.worldCenterOfMass;
				Vector3 v = vAir + Vector3.Cross(w, com - cg);
				float v2 = v.sqrMagnitude;
				if (v2 < 25f)
				{
					continue;
				}
				float qS = 0.5f * rho * v2 * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				Vector3 dF = Camber.Delta(v, lt.rotation, vis.rotation * Vector3.right, k, delta, table, qS);
				m += Vector3.Cross(com + lt.rotation * Acc.AP_centerOfLift(host) - cg, dF);
			}
			Vector3 mb = Quaternion.Inverse(rootRot) * m;
			return new Vector3(0f - mb.z, 0f - mb.x, mb.y);
		}

		// 2.31.2: the passive airframe telemetry (see the psA / psD / cePass fields). Body rates in the aero frame (x roll, z yaw),
		// world rotation w = rootRot * (-qa.y, qa.z, -qa.x) for aero rates qa = (p, q, r) - the inverse of the omega mapping in Tick.
		private void PassiveStep(float dt, float rho, Quaternion rootRot)
		{
			if (!PassiveProbeOn || isAI || tas < 30f)
			{
				return;
			}
			Vector3 vAir = vCgAir;
			Vector3 W(float pa, float qa, float ra)
			{
				return rootRot * new Vector3(0f - qa, ra, 0f - pa);
			}
			float pNow = omega.x, qNow = omega.y, rNow = omega.z;   // rad/s
			Vector3 m0 = AirframeMomentAt(vAir, W(pNow, qNow, rNow), rho, rootRot);
			psAX = AccRoll(m0);
			psAZ = AccYaw(m0);
			Vector3 m1 = AirframeMomentAt(vAir, W(pCmd * (MathF.PI / 180f), qNow, rCmd * (MathF.PI / 180f)), rho, rootRot);
			psDX = AccRoll(m1) - psAX;
			psDZ = AccYaw(m1) - psAZ;
			cePassXP = Mathf.Max(0f, psDX);
			cePassXN = Mathf.Max(0f, 0f - psDX);
			cePassZP = Mathf.Max(0f, psDZ);
			cePassZN = Mathf.Max(0f, 0f - psDZ);
			float ar = alpha * (MathF.PI / 180f);
			float dS = psDX * Mathf.Cos(ar) + psDZ * Mathf.Sin(ar);   // along the velocity-roll axis (body roll : yaw = cos a : sin a)
			cePassSP = Mathf.Max(0f, dS);
			cePassSN = Mathf.Max(0f, 0f - dS);
			psDerivTimer += dt;
			if (psDerivTimer < PassiveDerivPeriod)
			{
				return;
			}
			psDerivTimer = 0f;
			// sideslip: the CG flow at (alpha, beta +- db), same TAS (the parametrization of SlipMomentAt and the bench), the rates held
			const float db = 2f, dw = 5f;
			Vector3 VAt(float bDeg)
			{
				float fa = alpha * (MathF.PI / 180f), fb = bDeg * (MathF.PI / 180f);
				return rootRot * new Vector3(Mathf.Sin(fb), (0f - Mathf.Sin(fa)) * Mathf.Cos(fb), Mathf.Cos(fa) * Mathf.Cos(fb)) * tas;
			}
			Vector3 wNow = W(pNow, qNow, rNow);
			Vector3 mbP = AirframeMomentAt(VAt(beta + db), wNow, rho, rootRot);
			Vector3 mbN = AirframeMomentAt(VAt(beta - db), wNow, rho, rootRot);
			psLb = (AccRoll(mbP) - AccRoll(mbN)) / (2f * db);
			psNb = (AccYaw(mbP) - AccYaw(mbN)) / (2f * db);
			float dwr = dw * (MathF.PI / 180f);
			Vector3 mpP = AirframeMomentAt(vAir, W(pNow + dwr, qNow, rNow), rho, rootRot);
			Vector3 mpN = AirframeMomentAt(vAir, W(pNow - dwr, qNow, rNow), rho, rootRot);
			psLp = (AccRoll(mpP) - AccRoll(mpN)) / (2f * dw);
			psNp = (AccYaw(mpP) - AccYaw(mpN)) / (2f * dw);
			Vector3 mrP = AirframeMomentAt(vAir, W(pNow, qNow, rNow + dwr), rho, rootRot);
			Vector3 mrN = AirframeMomentAt(vAir, W(pNow, qNow, rNow - dwr), rho, rootRot);
			psLr = (AccRoll(mrP) - AccRoll(mrN)) / (2f * dw);
			psNr = (AccYaw(mrP) - AccYaw(mrN)) / (2f * dw);
		}

		// the rudders' one-side yaw and the other surfaces' roll authority at a hypothetical flow (deg/s^2)
		private void SlipAuthorityAt(float aDeg, float bDeg, float rho, Quaternion rootRot, ref EvalContext ctx, out float yawA, out float rollA)
		{
			float num = Mathf.Max(tas, 1f);
			float fa = aDeg * (MathF.PI / 180f);
			float fb = bDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(Mathf.Sin(fb) * num, (0f - Mathf.Sin(fa)) * Mathf.Cos(fb) * num, Mathf.Cos(fa) * Mathf.Cos(fb) * num);
			yawA = 0f;
			rollA = 0f;
			for (int i = 0; i < surfaces.Count; i++)
			{
				SurfaceEffector se = surfaces[i];
				if (!se.active)
				{
					continue;
				}
				Vector3 mMax = se.EvalFlow(se.max, vector, rho, ref ctx);
				Vector3 mMin = se.EvalFlow(se.min, vector, rho, ref ctx);
				if (se.kind == SurfaceKind.Rudder)
				{
					yawA += 0.5f * Mathf.Abs(mMax.z - mMin.z) * IaInv[2, 2] * 57.29578f * Mathf.Max(0.05f, rudderAuth);
				}
				else
				{
					rollA += 0.5f * Mathf.Abs(mMax.x - mMin.x) * IaInv[0, 0] * 57.29578f;
				}
			}
			// Revoker 0.3.0: the yaw nozzle is part of the yaw axis at every sideslip (thrust; its table does not move with the flow)
			if (tvcy != null && tvcy.active && tvcy.M != null && tvcy.N >= 2 && tvcy.hi - tvcy.lo > 0.5f)
			{
				float zA = tvcy.MAt(tvcy.lo).z;
				float zB = tvcy.MAt(tvcy.hi).z;
				yawA += 0.5f * Mathf.Abs(zB - zA) * IaInv[2, 2] * 57.29578f;
			}
		}

		private void SlipProbeStep_(float dt, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			if (!Plugin.RollStopSlipProbe || isAI)
			{
				slipTolF = 25f;
				slipTolDbg = 25f;
				return;
			}
			slipProbeTimer += dt;
			if (slipProbeTimer >= SlipProbePeriod && tas > 30f)
			{
				slipProbeTimer = 0f;
				// the sign of a restoring yaw moment, latched once at low AoA where any conventional airframe is directionally
				// stable (the probe's sweep gives it: the yaw moment grows with the sideslip one way or the other)
				if (slipSignR == 0f && Mathf.Abs(alpha) < 10f && tas > 60f)
				{
					float n0 = AccYaw(SlipMomentAt(alpha, 0f, rho, rootRot, ref ctx));
					float nP = AccYaw(SlipMomentAt(alpha, 8f, rho, rootRot, ref ctx));
					float nN = AccYaw(SlipMomentAt(alpha, -8f, rho, rootRot, ref ctx));
					float dP = nP - n0;
					float dN = n0 - nN;
					slipSignR = ((dP + dN) >= 0f) ? 1f : (-1f);
				}
				if (slipSignR == 0f)
				{
					slipTolDbg = 25f;
				}
				else
				{
					float dep = SlipProbeMax + 5f;
					float auth = SlipProbeMax + 5f;
					float stiff = 0f;
					float n0 = AccYaw(SlipMomentAt(alpha, 0f, rho, rootRot, ref ctx));
					SlipAuthorityAt(alpha, 0f, rho, rootRot, ref ctx, out float y0, out float r0);
					for (int sgn = -1; sgn <= 1; sgn += 2)
					{
						for (float b = SlipProbeStep; b <= SlipProbeMax + 0.01f; b += SlipProbeStep)
						{
							float bb = sgn * b;
							// restoring yaw at this sideslip (deg/s^2; positive = toward zero sideslip)
							float rest = (AccYaw(SlipMomentAt(alpha, bb, rho, rootRot, ref ctx)) - n0) * slipSignR * sgn;
							if (b <= SlipProbeStep + 0.01f)
							{
								stiff += 0.5f * rest / SlipProbeStep;
							}
							SlipAuthorityAt(alpha, bb, rho, rootRot, ref ctx, out float yB, out float rB);
							// departure: the airframe pushes the sideslip open harder than SlipAuthUse of the yaw axis can pull back - the
							// rudders' yaw at that sideslip, or past the rudder fade the allocator's yaw authority (the stabs'): 2.5.16,
							// the 14:55 log read a 4 deg bound at -20..-60 deg AoA where the rudders' tables are faded to nothing
							// and any destabilizing moment at all tripped the test (the margin never moved, the episodes were short)
							// Revoker 0.3.0: the yaw axis at this sideslip is the rudder (faded) + the nozzle, as SlipAuthorityAt sums them;
							// no allocator-wide credit (see yawEffs)
							float yHold = yB;
							if (rest < 0f - SlipAuthUse * yHold && b < dep)
							{
								dep = b;
							}
							// authority: the rudders' yaw or the surfaces' roll down to SlipAuthFrac of their zero-sideslip value
							if ((yB < SlipAuthFrac * y0 || rB < SlipAuthFrac * r0) && b < auth)
							{
								auth = b;
							}
							if (dep <= b && auth <= b)
							{
								break;
							}
						}
					}
					slipDepDbg = dep;
					slipAuthDbg = auth;
					slipStiffDbg = stiff;
					slipTolDbg = Mathf.Min(dep, auth);
					// Revoker 0.4.0: the airframe's yaw damping at the present AoA and sideslip, deg/s^2 per deg/s of body yaw rate (negative = damping)
					float nR0 = AccYaw(SlipMomentAt(alpha, beta, rho, rootRot, ref ctx, 0f));
					float nR1 = AccYaw(SlipMomentAt(alpha, beta, rho, rootRot, ref ctx, SlipDampProbeRate));
					slipDampDbg = (nR1 - nR0) / (SlipDampProbeRate * 57.29578f);
					// Revoker 0.4.0: the signed sideslip slopes for the reversal model (tables' own sign; the kinematics decide what restores)
					Vector3 mP = SlipMomentAt(alpha, SlipProbeStep, rho, rootRot, ref ctx);
					Vector3 mN = SlipMomentAt(alpha, 0f - SlipProbeStep, rho, rootRot, ref ctx);
					slipYawSlopeDbg = (AccYaw(mP) - AccYaw(mN)) / (2f * SlipProbeStep);
					slipRollSlopeDbg = (AccRoll(mP) - AccRoll(mN)) / (2f * SlipProbeStep);
				}
			}
			slipTolF += (slipTolDbg - slipTolF) * (dt / (SlipTolTau + dt));
			slipDampF += (slipDampDbg - slipDampF) * (dt / (SlipTolTau + dt));
			slipYawSlopeF += (slipYawSlopeDbg - slipYawSlopeF) * (dt / (SlipTolTau + dt));
			slipRollSlopeF += (slipRollSlopeDbg - slipRollSlopeF) * (dt / (SlipTolTau + dt));
		}

		private void AsymSweepPrep(Quaternion rootRot)
		{
			asymSweepIas = Mathf.Max(ias, 1f);
			AsymPrepSym(rootRot);
			asymMT = ((tvc.active || Plugin.DisableTVC) ? (((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : Vector3.zero);
		}

		private Vector3 AsymMomentAt(float aDeg, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			float[] array = asymDsym;
			if (array == null)
			{
				return Vector3.zero;
			}
			float num = Mathf.Max(tas, 1f);
			float f = aDeg * (MathF.PI / 180f);
			Vector3 vector = rootRot * new Vector3(0f, (0f - Mathf.Sin(f)) * num, Mathf.Cos(f) * num);
			Vector3 vector2 = vector / num;
			float num2 = 0.5f * rho * num * num;
			Vector3 zero = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				if (!(transform == null))
				{
					Quaternion rotation = AirframeRot(aeroPart, transform);
					Vector3 lhs = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 rhs = -vector2 * (0.5f * num2 * Acc.AP_dragArea(aeroPart));
					if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
					{
						rhs += PartForce(vector, rotation, rotation * Vector3.right, value, 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart));
					}
					zero += Vector3.Cross(lhs, rhs);
				}
			}
			Vector3 vector3 = Quaternion.Inverse(rootRot) * zero;
			Vector3 result = new Vector3(0f - vector3.z, 0f - vector3.x, vector3.y) + (tvc.active ? (((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : Vector3.zero);
			for (int i = 0; i < surfaces.Count; i++)
			{
				if (surfaces[i].active)
				{
					result += surfaces[i].EvalFlow(array[i], vector, rho, ref ctx);
				}
			}
			foreach (HingeFlapEffector hA in hinges)
			{
				if (hA.active)
				{
					result += hA.EvalFlow(hA.SymNow, vector, rho, ref ctx);
				}
			}
			return result;
		}

		private void AsymSweepPoints(int k0, int k1, float rho, Quaternion rootRot, ref EvalContext ctx)
		{
			float[] array = asymDsym;
			if (array == null)
			{
				return;
			}
			float num = Mathf.Max(tas, 1f);
			Vector3 vector = asymMT;
			for (int i = k0; i < k1; i++)
			{
				float f = asymAlphas[i] * (MathF.PI / 180f);
				Vector3 vector2 = rootRot * new Vector3(0f, (0f - Mathf.Sin(f)) * num, Mathf.Cos(f) * num);
				Vector3 vector3 = vector2 / num;
				float num2 = 0.5f * rho * num * num;
				Vector3 zero = Vector3.zero;
				foreach (AeroPart aeroPart in aeroParts)
				{
					if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached())
					{
						continue;
					}
					Transform transform = Acc.AP_liftNormal(aeroPart);
					if (!(transform == null))
					{
						Quaternion rotation = AirframeRot(aeroPart, transform);
						Vector3 lhs = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
						Vector3 rhs = -vector3 * (0.5f * num2 * Acc.AP_dragArea(aeroPart));
						if (!surfaceParts.Contains(aeroPart) && foils.TryGetValue(aeroPart, out var value) && Acc.AP_wingArea(aeroPart) > 0f)
						{
							rhs += PartForce(vector2, rotation, rotation * Vector3.right, value, 0.5f * rho * Acc.AP_wingArea(aeroPart) * Acc.AP_wingEff(aeroPart));
						}
						zero += Vector3.Cross(lhs, rhs);
					}
				}
				Vector3 vector4 = Quaternion.Inverse(rootRot) * zero;
				Vector3 m = new Vector3(0f - vector4.z, 0f - vector4.x, vector4.y) + vector;
				float pAero = m.y - vector.y;
				float pStabMin = 0f;
				float pStabMax = 0f;
				float pFlapNow = 0f;
				float pRud0 = 0f;
				float pRudP = 0f;
				float pRudM = 0f;
				float toe = 0f;
				float num3 = 0f;
				float num4 = 0f;
				float t = Mathf.InverseLerp(30f, 45f, Mathf.Abs(asymAlphas[i]));
				float num5 = RudderAuthAt(asymAlphas[i]);
				// Revoker 0.2.0: the inboard elevon pair at its symmetric deflection (the roll it makes for the pilot is no load)
				float pHingeAuth = 0f;
				foreach (HingeFlapEffector hA in hinges)
				{
					if (hA.active)
					{
						Vector3 mH = hA.EvalFlow(hA.SymNow, vector2, rho, ref ctx);
						m += mH;
						// Revoker 0.2.5: and its nose-down travel for the authority cap (ElevonAuthShare of it)
						float pLoH = mH.y;
						for (int lH = 0; lH < hA.N; lH++)
						{
							pLoH = Mathf.Min(pLoH, hA.EvalFlow(hA.grid[lH], vector2, rho, ref ctx).y);
						}
						pHingeAuth += Mathf.Lerp(mH.y, pLoH, Plugin.ElevonPitch ? ElevonAuthShare : 0f);
					}
				}
				for (int j = 0; j < surfaces.Count; j++)
				{
					SurfaceEffector surfaceEffector = surfaces[j];
					if (!surfaceEffector.active)
					{
						continue;
					}
					m += surfaceEffector.EvalFlow(array[j], vector2, rho, ref ctx);
					if (surfaceEffector.kind == SurfaceKind.Flaperon || surfaceEffector.kind == SurfaceKind.Aileron)
					{
						float pRef = surfaceEffector.EvalFlow((flapSymRef != null && j < flapSymRef.Length) ? flapSymRef[j] : 0f, vector2, rho, ref ctx).y;
						// Revoker 0.2.5 (ElevonAuthShare): the delta's elevons are pitch effectors (ElevonPitch), so their nose-down
						// travel counts toward the authority the low-thrust / no-TVC cap judges by, at ElevonAuthShare of it (the
						// rest stays for roll at the cap); the F-22E counted its flaperons only at their present trim
						if (surfaceEffector.kind == SurfaceKind.Aileron && Plugin.ElevonPitch && ElevonAuthShare > 0f)
						{
							float pLo = pRef;
							for (int lE = 0; lE < surfaceEffector.N; lE++)
							{
								pLo = Mathf.Min(pLo, surfaceEffector.EvalFlow(surfaceEffector.grid[lE], vector2, rho, ref ctx).y);
							}
							pRef = Mathf.Lerp(pRef, pLo, ElevonAuthShare);
						}
						pFlapNow += pRef;
					}
					if (surfaceEffector.kind == SurfaceKind.Rudder && toe > 0f)
					{
						pRud0 += surfaceEffector.EvalFlow(0f, vector2, rho, ref ctx).y;
						pRudP += surfaceEffector.EvalFlow(toe, vector2, rho, ref ctx).y;
						pRudM += surfaceEffector.EvalFlow(0f - toe, vector2, rho, ref ctx).y;
					}
					float num6;
					float num7;
					switch (surfaceEffector.kind)
					{
					case SurfaceKind.Flaperon:
					case SurfaceKind.Aileron:
						num6 = 1f;
						num7 = 1f;
						break;
					case SurfaceKind.Stabilator:
						num6 = (num7 = Mathf.Lerp(Plugin.StabRollShare, 1f, t));
						break;
					case SurfaceKind.Rudder:
						num6 = 0f;
						num7 = num5;
						break;
					default:
						num6 = (num7 = 0f);
						break;
					}
					if (num6 != 0f || num7 != 0f)
					{
						float num8 = float.MaxValue;
						float num9 = float.MinValue;
						float num10 = float.MaxValue;
						float num11 = float.MinValue;
						float pMinS = float.MaxValue;
						float pMaxS = float.MinValue;
						for (int l = 0; l < surfaceEffector.N; l++)
						{
							// Revoker 0.2.7: the asymmetric-load cap judges the canards on their stock 40 deg; the 5 deg added in 0.2.7
							// stay a reserve it does not spend. Counted, they raised the cap with a canard shot off from 30-47 to 49-64
							// deg (bench, canard_L + wingtip_R + aileron_R + flap_L gone, 350 km/h reversal), where the stop-side roll
							// authority was down to 11-13 deg/s^2 (0.2.6: 33-65) and the roll error doubled.
							// Sampled on the stock grid too (31 points over +-40), not the +-45 grid with points over 40 skipped: the cap is
							// sensitive to where the sweep samples, and the coarser grid alone raised it 25-38 -> 51-53 deg in the full
							// pull at 290-300 km/h with that damage.
							float gS = (surfaceEffector.kind == SurfaceKind.Stabilator && AsymStabTravel > 0f && surfaceEffector.N > 1) ? Mathf.Lerp(0f - AsymStabTravel, AsymStabTravel, (float)l / (float)(surfaceEffector.N - 1)) : surfaceEffector.grid[l];
							Vector3 m2 = surfaceEffector.EvalFlow(gS, vector2, rho, ref ctx);
							pMinS = Mathf.Min(pMinS, m2.y);
							pMaxS = Mathf.Max(pMaxS, m2.y);
							float b = AccRoll(m2);
							float b2 = AccYaw(m2);
							num8 = Mathf.Min(num8, b);
							num9 = Mathf.Max(num9, b);
							num10 = Mathf.Min(num10, b2);
							num11 = Mathf.Max(num11, b2);
						}
						num3 += num6 * 0.5f * (num9 - num8);
						num4 += num7 * 0.5f * (num11 - num10);
						if (surfaceEffector.kind == SurfaceKind.Stabilator)
						{
							pStabMin += pMinS;
							pStabMax += pMaxS;
						}
					}
				}
				asymNetDn[i] = (pAero + pStabMin) * IaInv[1, 1] * 57.29578f;
				// 2.0.9: the symmetric rudder toe (2.0.7 pitch aid, up to RudderToeMax each way) counts toward the nose-down
				// authority the low-thrust cap judges by (the rudders' pitch was left out before, neutral or toed)
				float pRud = (toe > 0f) ? Mathf.Min(pRud0, Mathf.Min(pRudP, pRudM)) : 0f;
				authNetDn[i] = (pAero + pStabMin + pFlapNow + pRud + pHingeAuth) * IaInv[1, 1] * 57.29578f;
				asymNetUp[i] = (pAero + pStabMax) * IaInv[1, 1] * 57.29578f;
				asymAr[i] = AccRoll(m);
				asymAy[i] = AccYaw(m);
				asymCr[i] = num3;
				asymCy[i] = num4;
				// 1.8.45: the demand is judged at the dynamic pressure predicted from a rapid IAS decay. Load and authority
				// both scale with q and the absolute reserves (AsymRollReserveAccel / AsymYawReserveAccel) do not, so where
				// those bind, and through the fixed deadband everywhere, a fast deceleration brings the cap down ahead of
				// the authority loss. The share of the authority the reserve may claim is AsymResMaxFrac (0.8, as before).
				float kq = qPredK;
				float crK = kq * num3;
				float cyK = kq * num4;
				float num12 = Mathf.Clamp01(Plugin.AsymReserve);
				float tauRes = (Plugin.AsymRollTau > 0.01f) ? (Mathf.Max(0f, pStockRef) / Plugin.AsymRollTau) : 0f;
				float num13 = Mathf.Min(Mathf.Max(num12 * crK, Mathf.Max(tauRes, Plugin.AsymRollReserveAccel)), AsymResMaxFrac * crK);
				float num14 = Mathf.Min(Mathf.Max(num12 * cyK, Plugin.AsymYawReserveAccel), AsymResMaxFrac * cyK);
				float num15 = Mathf.Max(0f, Plugin.AsymDeadband);
				float num16 = 0.25f * num15;
				// 2.0.12: the thrust part of the load (one engine out: tvc.Mnow) does not fall with q the way the aero load
				// and the authority do, so only the aero part is scaled by the predicted q. 2.0.11 scaled both: the engine-out
				// cap was judged at today's IAS however fast the pull was bleeding it (10:37 log: AoA 64.7 deg at 101 m/s before
				// the cap came down, 3 s after the pull).
				float arT = AsymThrustSplit ? AccRoll(vector) : 0f;
				float ayT = AsymThrustSplit ? AccYaw(vector) : 0f;
				// 2.0.13: with an engine out the thrust part is left out of the cap altogether (the cap is the fixed
				// EngineOutAoACap plus whatever an aerodynamic asymmetry needs): the thrust yaw is the throttle's business
				// (VMC protection), and judging it here made the cap follow speed and throttle, 12 <-> 65 deg in seconds
				float tK = (((engOut || engAsym) && EngineOutFixedCap) || tKHoldT > 0f) ? 0f : 1f;
				float arL = kq * (asymAr[i] - arT) + tK * arT;
				float ayL = kq * (asymAy[i] - ayT) + tK * ayT;
				asymU[i] = Mathf.Max(0f, Mathf.Abs(arL) - num15) / Mathf.Max(crK - num13, 1f) + Mathf.Max(0f, Mathf.Abs(ayL) - num16) / Mathf.Max(cyK - num14, 1f);
			}
			if (k1 >= asymAlphas.Length)
			{
				asymCycleDone = true;
			}
		}

		private float NonSurfacePitch(Vector3 wind, float rho, Quaternion rootInv, Vector3 rootRight, float dAlphaDeg, float qRad)
		{
			Quaternion rot = Quaternion.AngleAxis(dAlphaDeg, rootRight);
			Vector3 w = -rootRight * qRad;
			Vector3 m = Vector3.zero;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null || aeroPart.IsDetached() || surfaceParts.Contains(aeroPart))
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				float area = Acc.AP_wingArea(aeroPart);
				if (transform == null || area <= 0f || !foils.TryGetValue(aeroPart, out var foil))
				{
					continue;
				}
				Quaternion rotation = AirframeRot(aeroPart, transform);
				Vector3 arm = aeroPart.rb.worldCenterOfMass - cg;
				Vector3 r = arm + rotation * Acc.AP_centerOfLift(aeroPart);
				// flow at the part: the CG's air velocity turned by dAlpha, plus the pitch rotation qRad
				Vector3 v = rot * (vCgAir) + Vector3.Cross(w, arm);
				m += Vector3.Cross(r, PartForce(v, rotation, rotation * Vector3.right, foil, 0.5f * rho * area * Acc.AP_wingEff(aeroPart)));
			}
			return 0f - (rootInv * m).x;
		}

		private Vector3 vCgAir;

		private void ComputeHybridSlopes(Vector3 wind, float rho, Vector3 rootRight)
		{
			if (surfaceParts.Count == 0)
			{
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.part != null)
					{
						surfaceParts.Add(se.part);
					}
				}
			}
			Quaternion rootInv = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
			vCgAir = ((AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : ac.rb.velocity) - wind;
			float q0 = omega.y;
			float mP = NonSurfacePitch(wind, rho, rootInv, rootRight, 1f, q0);
			float mM = NonSurfacePitch(wind, rho, rootInv, rootRight, -1f, q0);
			float mQ1 = NonSurfacePitch(wind, rho, rootInv, rootRight, 0f, q0 + 0.05f);
			float mQ0 = NonSurfacePitch(wind, rho, rootInv, rootRight, 0f, q0 - 0.05f);
			mAlphaO = 0.5f * (mP - mM);
			mQO = (mQ1 - mQ0) / (0.1f * 57.29578f);
		}

		private void ComputeEnvelope(Vector3 wind, float rho, Vector3 rootRight, Vector3 eL)
		{
			for (int i = 0; i < envLift.Length; i++)
			{
				envLift[i] = 0f;
			}
			Quaternion quaternion = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
			Vector3 lhs = -((Component)(object)ac).transform.forward * 0.2f;
			float num = 0f;
			float num2 = 0f;
			float num3 = mass * 9.81f;
			Vector3 vCgHat = (((AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : ac.rb.velocity) - wind).normalized;
			float dragAf = 0f;
			foreach (AeroPart aeroPart in aeroParts)
			{
				if (aeroPart == null || aeroPart.rb == null)
				{
					continue;
				}
				Transform transform = Acc.AP_liftNormal(aeroPart);
				float num4 = Acc.AP_wingArea(aeroPart);
				if (transform == null || aeroPart.IsDetached() || num4 <= 0f || !foils.TryGetValue(aeroPart, out var value))
				{
					continue;
				}
				Vector3 vector = aeroPart.rb.velocity - wind;
				float sqrMagnitude = vector.sqrMagnitude;
				bool dwPart = Downwash.IsStab(aeroPart);
				if (dwPart)
				{
					vector = Downwash.Apply(vector, ((Component)(object)ac).transform.rotation, ac.rb.velocity - wind);
				}
				if (!(sqrMagnitude < 25f))
				{
					float qS = 0.5f * rho * sqrMagnitude * num4 * Acc.AP_wingEff(aeroPart);
					Quaternion rotation = AirframeRot(aeroPart, transform);
					Vector3 right = rotation * Vector3.right;
					Vector3 lhs2 = aeroPart.rb.worldCenterOfMass + rotation * Acc.AP_centerOfLift(aeroPart) - cg;
					Vector3 v = vector + Vector3.Cross(lhs, aeroPart.rb.worldCenterOfMass - cg);
					num += 0f - (quaternion * Vector3.Cross(lhs2, PartForce(vector, rotation, right, value, 0.5f * rho * num4 * Acc.AP_wingEff(aeroPart)))).z;
					num2 += 0f - (quaternion * Vector3.Cross(lhs2, PartForce(v, rotation, right, value, 0.5f * rho * num4 * Acc.AP_wingEff(aeroPart)))).z;
					// 2.2.16: airframe drag along the flight path (airfoil + parasitic, AeroJob_Math terms), for the brake's share
					float effD = Acc.AP_wingEff(aeroPart);
					float dArea = Acc.AP_dragArea(aeroPart);
					dragAf += 0f - Vector3.Dot(PartForce(vector, rotation, right, value, 0.5f * rho * num4 * effD), vCgHat) + 0.25f * rho * sqrMagnitude * (dArea + (dArea + num4 * 0.1f) * (1f - effD));
					for (int j = 0; j < envAlphas.Length; j++)
					{
						Quaternion quaternion2 = Quaternion.AngleAxis((j < 3) ? envAlphas[j] : (envAlphas[j] - alpha), rootRight);
						Vector3 vj = quaternion2 * (aeroPart.rb.velocity - wind);
						if (dwPart)
						{
							vj = Downwash.Apply(vj, ((Component)(object)ac).transform.rotation, quaternion2 * (ac.rb.velocity - wind));
						}
						envLift[j] += PartLift(vj, rotation, right, value, qS, eL);
					}
				}
			}
			// 2.2.16 (AirbrakeAoAFade): the brake's drag (game Airbrake.FixedUpdate: dragAmount x rho x v^2 x opening, on each
			// carrying part) against the airframe's own. Full open while the brake is AirbrakeFadeShareFull of the total drag or
			// more, stowed at AirbrakeFadeShareZero or less. Clean jet, bench: share 81 % at 0 deg AoA, 56 % at 10, 42 % at 15,
			// 32 % at 20, 19 % at 30, 12 % at 40, 9 % at 50.
			float dragAb = 0f;
			for (int iA = 0; iA < abRb.Count; iA++)
			{
				if (abRb[iA] != null && !(abPart[iA] != null && abPart[iA].IsDetached()))
				{
					dragAb += abDrag[iA] * rho * (abRb[iA].velocity - wind).sqrMagnitude;
				}
			}
			airbrakeShare = (dragAb > 0f) ? (dragAb / (dragAb + Mathf.Max(0f, dragAf))) : 1f;
			float abTgt = Mathf.Clamp01((airbrakeShare - Plugin.AirbrakeFadeShareZero) / Mathf.Max(0.01f, Plugin.AirbrakeFadeShareFull - Plugin.AirbrakeFadeShareZero));
			airbrakeFade = Mathf.MoveTowards(airbrakeFade, abTgt, 0.2f);
			nAlpha = Mathf.Clamp((envLift[2] - envLift[0]) * 0.5f / num3, 0.005f, 12f);
			Lp = Mathf.Min((num2 - num) / 0.2f, -1f);
			float num5 = float.MinValue;
			float num6 = 30f;
			for (int k = 3; k < EnvNegStart; k++)
			{
				float num7 = envLift[k] / num3 + tw * Mathf.Sin(envAlphas[k] * (MathF.PI / 180f));
				if (num7 > num5)
				{
					num5 = num7;
					num6 = envAlphas[k];
				}
			}
			nMaxAero = Mathf.Max(1f, num5);
			float nl = 0f;
			for (int kl = 3; kl < EnvNegStart; kl++)
			{
				nl = Mathf.Max(nl, envLift[kl] / num3);
			}
			nLiftMax = nl;
			float nlN = 0f;
			for (int kn = EnvNegStart; kn < envAlphas.Length; kn++)
			{
				nlN = Mathf.Max(nlN, (0f - envLift[kn]) / num3);
			}
			nLiftMin = nlN;
			alphaAtNMax = num6;
		}

		private static Vector3 PartForce(Vector3 v, Quaternion lr, Vector3 right, AirfoilTable t, float qSr)
		{
			float sqrMagnitude = v.sqrMagnitude;
			if (sqrMagnitude < 1f)
			{
				return Vector3.zero;
			}
			Vector3 vector = Quaternion.Inverse(lr) * v;
			t.Get(Mathf.Atan2(vector.y, vector.z), out var CL, out var CD);
			Vector3 vector2 = Vector3.Cross(v, right);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return Vector3.zero;
			}
			float num = qSr * sqrMagnitude;
			return -vector2 / magnitude * (CL * num) - v / Mathf.Sqrt(sqrMagnitude) * (CD * num);
		}

		private float StockRollRef()
		{
			float num = 0.01868f * tas;
			float b = tas * tas * Mathf.Max(0.01f, ac.airDensity) / 24010f;
			float num2 = 1f / Mathf.Max(1f, b);
			float num3 = Mathf.Min(num, num * (1f + 4.5f * num2) / (1f + 1.1f * num2 * num)) * 57.29578f;
			float num4 = Mathf.Lerp(Plugin.RollGainLowIAS, Plugin.RollGainHighIAS, Mathf.InverseLerp(120f, 170f, ias));
			return num3 * num4;
		}

		private float NPath(float a)
		{
			float W = mass * 9.81f;
			if (a <= 5f)
			{
				return Nk(3) + nAlpha * (a - 5f);
			}
			if (a >= 65f)
			{
				return Nk(15);
			}
			float num = (a - 5f) / 5f;
			int num2 = Mathf.Clamp((int)num, 0, 11);
			float num3 = num - (float)num2;
			return Nk(3 + num2) + (Nk(4 + num2) - Nk(3 + num2)) * num3;
			float Nk(int k)
			{
				return envLift[k] / W + tw * Mathf.Sin(envAlphas[k] * (MathF.PI / 180f));
			}
		}

		private static float PartLift(Vector3 v, Quaternion lr, Vector3 right, AirfoilTable t, float qS, Vector3 eL)
		{
			Vector3 vector = Quaternion.Inverse(lr) * v;
			float a = Mathf.Atan2(vector.y, vector.z);
			t.Get(a, out var CL, out var _);
			Vector3 vector2 = Vector3.Cross(v, right);
			float magnitude = vector2.magnitude;
			if (magnitude < 0.0001f)
			{
				return 0f;
			}
			return Vector3.Dot(-vector2 / magnitude * (CL * qS), eL);
		}

		// Ifrit 0.2.1: the pilot's throttle as it came into the FCS this frame (the stock airbrake rule needs it)
		public float rawThrottle = 1f;

		// Ifrit 0.2.1: whether the in-flight speedbrake is commanded out now (brake held, or with
		// ThrottleIndependentAirbrake off, throttle at zero too), in the air and on the ground.
		public bool AirbrakeOut
		{
			get
			{
				// Ifrit 0.2.1: the split decelerons are the airbrake in the air and on the ground (the stock Ifrit's behaviour)
				return airBrake > 0.01f;
			}
		}

		public void Tick(ControlInputs inp, bool flightAssist)
		{
			rawThrottle = (inp != null) ? inp.throttle : 1f;
			float num = Mathf.Max(0.001f, Time.fixedDeltaTime);
			time += num;
			Rigidbody rb = ac.rb;
			Transform transform = ((Component)(object)ac).transform;
			if (rb == null || transform == null)
			{
				return;
			}
			Quaternion rotation = transform.rotation;
			Quaternion quaternion = Quaternion.Inverse(rotation);
			massTimer += num;
			bool flag = massTimer > 0.5f;
			if (flag)
			{
				massTimer = 0f;
				RebuildBodies();
			}
			UpdateMassProps(flag || !havePrev, quaternion);
			Vector3 vector = Acc.AC_wind(ac);
			Vector3 vRef = (AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : rb.velocity;
			Vector3 vector2 = vRef - vector;
			tas = vector2.magnitude;
			float num2 = Mathf.Max(0.01f, ac.airDensity);
			ias = tas * Mathf.Sqrt(num2 / 1.225f);
			Vector3 vector3 = quaternion * vector2;
			alpha = ((tas > 1f) ? (Mathf.Atan2(0f - vector3.y, vector3.z) * 57.29578f) : 0f);
			beta = ((tas > 1f) ? (Mathf.Asin(Mathf.Clamp(vector3.x / tas, -1f, 1f)) * 57.29578f) : 0f);
			{
				Vector3 vRootB = quaternion * (rb.velocity - vector);
				float tasR = vRootB.magnitude;
				alphaRoot = (tasR > 1f) ? (Mathf.Atan2(0f - vRootB.y, vRootB.z) * 57.29578f) : 0f;
				betaRoot = (tasR > 1f) ? (Mathf.Asin(Mathf.Clamp(vRootB.x / tasR, -1f, 1f)) * 57.29578f) : 0f;
				Vector3 vCgB = quaternion * (((vCg.sqrMagnitude > 0f) ? vCg : rb.velocity) - vector);
				float tasC = vCgB.magnitude;
				alphaCgDbg = (tasC > 1f) ? (Mathf.Atan2(0f - vCgB.y, vCgB.z) * 57.29578f) : 0f;
				betaCgDbg = (tasC > 1f) ? (Mathf.Asin(Mathf.Clamp(vCgB.x / tasC, -1f, 1f)) * 57.29578f) : 0f;
			}
			Vector3 vector4 = quaternion * rb.angularVelocity;
			omega = new Vector3(0f - vector4.z, 0f - vector4.x, vector4.y);
			Vector3 x = (havePrev ? ((rb.velocity - velPrev) / num) : Vector3.zero);
			Vector3 vector5 = accF.Step(x, 0.05f, num);
			Vector3 xCg = (havePrev ? ((vRef - vCgPrev) / num) : Vector3.zero);
			Vector3 accCg = accCgF.Step(xCg, 0.05f, num);
			if (!FlightPathAtCg)
			{
				accCg = vector5;
			}
			Vector3 lhs = vector5 - Physics.gravity;
			nz = Vector3.Dot(lhs, transform.up) / 9.81f;
			nxDbg = Vector3.Dot(lhs, transform.forward) / 9.81f;   // 0.4.6 telemetry: body-axis specific force along the nose (g)
			Vector3 vector6 = ((tas > 1f) ? Vector3.Cross(vector2, transform.right).normalized : transform.up);
			eLy = vector6.y;
			float vc = Mathf.Max(tas, 30f);
			omegaV = Vector3.Dot(accCg, vector6) / Mathf.Max(tas, 5f) * 57.29578f;
			vDot = vDotF.Step((tas > 1f) ? Vector3.Dot(accCg, vector2 / tas) : 0f, 0.2f, num);
			theta = Mathf.Asin(Mathf.Clamp(transform.forward.y, -1f, 1f)) * 57.29578f;
			phi = Mathf.Atan2(0f - transform.right.y, transform.up.y) * 57.29578f;
			float x2 = (havePrev ? (Mathf.DeltaAngle(alphaPrev, alpha) / num) : 0f);
			alphaDot = alphaDotF.Step(x2, 0.05f, num);
			alphaPrev = alpha;
			alphaU = ((Mathf.Abs(alpha) < 90f || !havePrev) ? alpha : Mathf.Clamp(alphaU + Mathf.DeltaAngle(alphaU, alpha), -300f, 300f));
			bool flag2 = false;
			foreach (LandingGear gear in gears)
			{
				if (gear != null && gear.WeightOnWheel(0.05f))
				{
					flag2 = true;
					break;
				}
			}
			flaps.Update(num, this, flag2 && tas < 110f, ac.speed, Mathf.Clamp(inp.roll, -1f, 1f), groundBrake);
			// Ifrit 0.3.0 (IfritFlaperons): the decelerons (inner) and ailerons (outer) droop on the F-22E flap schedule (AoA,
			// load and the high-lift speed schedule, stall-capped on each surface's own section flow). Each left/right pair's
			// common deflection is held at the droop through its pair cost (symCost, as for rudder toe); the differential
			// stays free for roll. TE-down is -teUp in the effector's deflection sign.
			foreach (SurfaceEffector sF in surfaces)
			{
				if (sF.partner != null && (sF.kind == SurfaceKind.Flaperon || sF.kind == SurfaceKind.Aileron))
				{
					float droop = (sF.kind == SurfaceKind.Flaperon) ? flaps.innerDroop : flaps.outerDroop;
					sF.symTarget = (sF.teUp != 0f) ? (0f - sF.teUp * droop) : 0f;
				}
			}
			// Revoker 0.2.0: the inboard elevons are drawn toward FlapHighLiftDroop x the HLD position (default 0: a canard-delta
			// has no flap schedule); no manoeuvre droop
			foreach (HingeFlapEffector hS in hinges)
			{
				hS.symTarget = (hS.teUp != 0f) ? ((0f - hS.teUp) * Plugin.FlapHighLiftDroop * flaps.hlPos) : 0f;
			}
			// Revoker 0.2.8 (TrailingEdgeBrakeConfig): in the air, with the airbrake out, the elevon pairs' common deflections are
			// drawn to the F-22E's speedbrake set: inboard trailing edge down (its flaperons), outboard up (its ailerons). Their
			// lift and pitch largely cancel (the canards trim what is left); the differentials stay free for roll. Scaled by the
			// airbrake's AoA fade, off on the wheels (the rollout aero-brake takes over there).
			teBrakeAir = (TeBrakeOn && !flag2 && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (airBrake * (Plugin.TeBrakeAoAFade ? teFade : 1f)) : 0f;
			// Revoker 0.2.9: as fast as the actuators move it (the inboard elevons' full set at their servo rate; 0.2.8: 0.5 /s)
			float teRate = (TeBrakeAirRate > 0f) ? TeBrakeAirRate : (((hinges.Count > 0) ? Mathf.Max(1f, hinges[0].servo) : 70f) / Mathf.Max(1f, Mathf.Max(Plugin.AirBrakeInboardDown, TeBrakeOutMax)));
			teBrakeAirF = Mathf.MoveTowards(teBrakeAirF, teBrakeAir, teRate * num);
			// the derotation after a braked touchdown (derotGate, last frame's): the elevons go to the rollout's trailing-edge-up
			// set already, held through the pair costs, and the rate law flies the nose down with the canards
			teBrakeLandF = Mathf.MoveTowards(teBrakeLandF, (derotGate && Mode == FcsMode.GearDownRate) ? 1f : 0f, 1.5f * num);
			if (teBrakeAirF > 0.001f || teBrakeLandF > 0.001f)
			{
				Transform rootB = ((Component)(object)ac).transform;
				foreach (SurfaceEffector sB in surfaces)
				{
					float tuB = sB.GeoTeUp(rootB);
					if (sB.partner != null && sB.kind == SurfaceKind.Aileron && tuB != 0f)
					{
						sB.symTarget += tuB * (((TeBrakeLiftNeutral && Plugin.AirBrakeElevonUp > 0f) ? 0f : (Plugin.AirBrakeElevonUp * teBrakeAirF)) + Plugin.GroundBrakeFlap * teBrakeLandF);
					}
				}
				foreach (HingeFlapEffector hB in hinges)
				{
					float tuH = hB.GeoTeUp(rootB);
					if (tuH != 0f)
					{
						hB.symTarget += tuH * (Plugin.GroundBrakeInboard * teBrakeLandF - Plugin.AirBrakeInboardDown * teBrakeAirF);
					}
				}
			}
			rudderAuth = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.RudderFadeStart) / Mathf.Max(1f, Plugin.RudderFadeEnd - Plugin.RudderFadeStart));
			if (RudderRevBack)
			{
				rudderAuth = Mathf.Max(rudderAuth, Mathf.InverseLerp(120f, 140f, Mathf.Abs(alpha)));
			}
			foreach (SurfaceEffector surface in surfaces)
			{
				float num3 = ((surface.kind == SurfaceKind.Rudder) ? rudderAuth : 1f);
				surface.lo = surface.min * num3;
				surface.hi = surface.max * num3;
				if (StabPeakClamp && surface.kind == SurfaceKind.Stabilator)
				{
					ClampStabToPeak(surface);
				}
			}
			SideslipRudder(num);
			EvalContext ctx = new EvalContext
			{
				cg = cg,
				rootInv = quaternion,
				rho = num2,
				wind = vector,
				vAir = ac.rb.velocity - vector
			};
			// Ifrit 0.2.2: split effectors in retraction coordinates while the brake is out; the pair cost keeps the brake
			// symmetric (common retraction costs SplitBrakeHoldCost) and leaves one-sided retraction free for yaw
			splitBrakeMode = SplitDeploy;
			SplitEffector spL = null;
			SplitEffector spR = null;
			foreach (SplitEffector sm in splits)
			{
				sm.SetBrakeMode(splitBrakeMode);
				sm.partner = null;
				if (sm.name.EndsWith("_L"))
				{
					spL = sm;
				}
				else
				{
					spR = sm;
				}
			}
			if (splitBrakeMode && spL != null && spR != null)
			{
				spL.partner = spR;
				spL.pairSign = 1f;
				spR.pairSign = 1f;
				spL.symCost = 0f;
				spL.symTarget = 0f;
				spL.antiCost = Plugin.SplitBrakeHoldCost;
			}
			Vector3 zero = Vector3.zero;
			RollPitchGateStep(num);
			foreach (Effector eff in effs)
			{
				if (eff is SurfaceEffector seH)
				{
					seH.honestGate = Mathf.Max(Plugin.OvershootPitchPriority ? pitchNeed : 0f, HonestRollGate * rpLat);
				}
				eff.Evaluate(ref ctx);
				if (eff.active)
				{
					zero += eff.Mnow;
					if (!Engaged || !havePrev)
					{
						eff.cmd = eff.now;
						eff.cmdOut = eff.now;
						eff.cmdLag = eff.now;
						eff.outTraj.Reset(eff.now);
					}
				}
			}
			TeBrakeFadeStep(num);
			TeBrakeLiftBalance();
			// Ifrit 0.2.2: airbrake. The game holds the splits at full travel while the brake is out (SurfacePostfix feeds the job
			// zero throttle); 0.2.1 modelled them as still under the allocator's control, so each frame it 'retracted' splits
			// that never moved to meet part of the pitch target. The INDI increment went into that phantom, the stabs made only
			// the rest, and the brake's nose-down drag moment (flaps 2.5 m aft and 0.15 m below the CG, ~q) left a steady
			// pitch-rate error instead of being integrated out (bench, neutral stick, brake held: q -0.9 / -1.6 / -2.6 deg/s at
			// 400 / 700 / 1000 km/h; pedal with the brake out did nothing). The split effectors are now in retraction
			// coordinates while the brake is out (see SplitEffector), and each split is bounded to where its servo gets it
			// next frame while the brake is out and while the splits come back in after it, so deploy and release are fed
			// forward to the stabs and nozzles.
			foreach (SplitEffector sb in splits)
			{
				if (sb.active && (SplitDeploy || splitRetract))
				{
					sb.lo = Mathf.Max(sb.min, sb.now - sb.rate * num);
					sb.hi = Mathf.Min(sb.max, sb.now + sb.rate * num);
					sb.cmd = Mathf.Clamp(sb.cmd, sb.lo, sb.hi);
				}
			}
			// Ifrit 0.3.1 (DeceleronSplitBudget): the split is bounded to the travel the deceleron's deflection leaves
			// (|deflection| + split <= travel, deflection first); SurfacePostfix applies the same cap to the job.
			foreach (SplitEffector sbB in splits)
			{
				if (!sbB.active)
				{
					continue;
				}
				float sAllow = sbB.BudgetSplit();
				if (sbB.brakeMode)
				{
					sbB.lo = Mathf.Min(Mathf.Max(sbB.lo, sbB.max - sAllow), sbB.hi);
				}
				else
				{
					sbB.hi = Mathf.Max(Mathf.Min(sbB.hi, sAllow), sbB.lo);
				}
				sbB.cmd = Mathf.Clamp(sbB.cmd, sbB.lo, sbB.hi);
			}
			if (SplitDeploy)
			{
				splitRetract = true;
			}
			else if (splitRetract)
			{
				bool anyOut = false;
				foreach (SplitEffector sb2 in splits)
				{
					if (sb2.active && sb2.now > 0.5f)
					{
						anyOut = true;
					}
				}
				splitRetract = anyOut;
			}
			Vector3 x3 = (havePrev ? ((omega - omegaPrev) / num) : Vector3.zero);
			Vector3 vector7 = omegaDotF.Step(x3, IndiTau, num);
			Vector3 vector8 = mNowF.Step(zero, IndiTau, num);
			float aIndiF = alphaIndiF.Step(alpha, IndiTau, num);
			float qIndiFv = qIndiF.Step(omega.y * 57.29578f, IndiTau, num);
			// faded out from 25 to 40 deg AoA: the lag matters at high dynamic pressure, and the stalled-foil slopes up there cost roll (bench, full pull + roll at 350 km/h, 64 deg: bank at 1 s 60 -> 48 deg without the fade)
			float hybLo = 1f - Mathf.InverseLerp(25f, 40f, Mathf.Abs(alpha));
			hybridDM = hybLo * ((Mathf.Abs(alpha) < 80f && tas > 30f) ? (mAlphaO * Mathf.Clamp(alpha - aIndiF, -5f, 5f) + mQO * Mathf.Clamp(omega.y * 57.29578f - qIndiFv, -20f, 20f)) : 0f);
			// Ifrit 0.4.9 (F-22E 2.4.7/2.4.8) disturbance-rate lead; the bench figures below are the F-22E's. INDI's pitch target is I(wdot_des - d), d = the filtered measured pitch acceleration
			// less the effectors' modelled share (the airframe's own moment, gravity-driven AoA change, coupling, model error:
			// everything the effectors are not doing). INDI uses d as it was IndiTau ago and the nozzles need ~0.1 s more to
			// deliver, so while d falls steadily the effectors trail it by d' x that lag and the rate hold sags. After a
			// corkscrew release the fuselage loses ~15 deg/s^2 of nose-up in half a second as the AoA drops 57 -> 48 deg
			// (bench: d' about -30 deg/s^3; 24 Oct 22:26 log, 118.3 s: q -3.7). Most of that dip was the allocator leaving the
			// nozzles short (Allocator, CoordNewton); this covers the lag that remains (bench, corkscrew release at 1 km:
			// q min -1.38 -> -0.68). The lead takes d forward by d' x HybridIndiHighAoALead. Model-free on purpose:
			// the stalled-foil slopes the 1.9.0 hybrid lead uses have the wrong sign up here (the fuselage's moment falls as
			// the AoA falls through 57-41 deg). Above the 25-40 deg fade only, with the pitch stick released (zero-rate hold
			// or AoA-limit ride with the stick centred); with the stick in, the previous law (a held pull + roll at 64 deg lost
			// roll rate to a high-AoA lead: bench, 350 km/h, bank at 1 s 74 -> 58 deg).
			float dHatY = vector7.y - IaInv[1, 1] * vector8.y;
			float dDotRaw = (dHatInit && num > 1E-05f) ? ((dHatY - dHatPrevY) / num) : 0f;
			dHatPrevY = dHatY;
			dHatInit = havePrev;
			// the airframe's pitch damping is in d too; its rate (mQ x the pitch acceleration) is the rate loop's own doing, not
			// a disturbance, and leading it fought the capture (bench: q 1.4 -> -0.5 deg/s at the moment the hold took over)
			dDotRaw -= DistLeadNoDamp ? (IaInv[1, 1] * mQO * vector7.y * 57.29578f) : 0f;
			dDotYF += (Mathf.Clamp(dDotRaw, -10f, 10f) - dDotYF) * (num / (DistLeadTau + num));
			// its horizon is the nozzles' response: with their thrust near idle the stabs carry the pitch and the lead only
			// shuffled them (bench, idle release at 380 km/h: q min -0.9 -> -2.0), so it fades out with T/W 0.4 -> 0.2
			float hiGate = (1f - hybLo) * (1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f)) * ((tas > 30f && Mathf.Abs(alpha) < 80f) ? 1f : 0f) * Mathf.InverseLerp(DistLeadTw0, DistLeadTw1, tw);
			distLeadGate = Mathf.MoveTowards(distLeadGate, hiGate, num / 0.15f);
			distLeadAcc = distLeadGate * Mathf.Clamp(dDotYF * Plugin.HybridIndiHighAoALead, -DistLeadMax * (MathF.PI / 180f), DistLeadMax * (MathF.PI / 180f));
			omegaPrev = omega;
			velPrev = rb.velocity;
			vCgPrev = (AirDataAtCg && vCg.sqrMagnitude > 0f) ? vCg : rb.velocity;
			havePrev = true;
			float num4 = 0f;
			float num5 = 0f;
			Vector3 zero2 = Vector3.zero;
			Vector3 zero3 = Vector3.zero;
			float zNowAll = 0f;
			foreach (Effector eff2 in effs)
			{
				if (eff2.active)
				{
					if (eff2.M != null && eff2.N >= 2)
					{
						zNowAll += eff2.MAt(eff2.now).z;
					}
					Vector3 vector9 = new Vector3(float.MinValue, float.MinValue, float.MinValue);
					Vector3 vector10 = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
					for (int i = 0; i < eff2.N; i++)
					{
						vector9 = Vector3.Max(vector9, eff2.M[i]);
						vector10 = Vector3.Min(vector10, eff2.M[i]);
					}
					zero2 += vector9;
					zero3 += vector10;
				}
			}
			num4 = zero2.y;
			num5 = zero3.y;
			float num6 = IaInv[1, 1];
			aUp = Mathf.Clamp(aUpF.Step((vector7.y + (num4 - vector8.y) * num6) * 57.29578f, 0.15f, num), 5f, 1500f);
			aDn = Mathf.Clamp(aDnF.Step((0f - (vector7.y + (num5 - vector8.y) * num6)) * 57.29578f, 0.15f, num), 5f, 1500f);
			float num7 = 0f;
			float num8 = 0f;
			rollSlew = 100f;
			foreach (SurfaceEffector surface2 in surfaces)
			{
				if (!surface2.active)
				{
					continue;
				}
				float num9 = ((surface2.kind == SurfaceKind.Flaperon || surface2.kind == SurfaceKind.Aileron) ? 1f : ((surface2.kind == SurfaceKind.Stabilator) ? Mathf.Lerp(Plugin.StabRollShare, 0.25f, Mathf.InverseLerp(160f, 240f, ias)) : 0f));
				if (num9 != 0f)
				{
					float num10 = float.MinValue;
					float num11 = float.MaxValue;
					for (int j = 0; j < surface2.N; j++)
					{
						num10 = Mathf.Max(num10, surface2.M[j].x);
						num11 = Mathf.Min(num11, surface2.M[j].x);
					}
					num7 += num9 * 0.5f * (num10 - num11);
					if (surface2.kind != SurfaceKind.Stabilator)
					{
						num8 += 0.5f * (num10 - num11);
					}
					if (surface2.kind != SurfaceKind.Stabilator && surface2.rate > 0f)
					{
						rollSlew = Mathf.Min(rollSlew, surface2.rate / Mathf.Max(1f, 0.5f * (surface2.max - surface2.min)));
					}
				}
			}
			aRoll = Mathf.Clamp(aRollF.Step(num7 * IaInv[0, 0] * 57.29578f, 0.2f, num), 10f, 5000f);
			// Ifrit 0.3.0: roll acceleration the differential TVC can add (half its roll range), scaled by the nozzle travel the
			// pitch TVC leaves free (both share the same +-15 deg). Thrust-based, so it holds at any IAS: at 80 km/h and 65 deg
			// AoA it is ~58 deg/s^2 against ~5-9 from the wing surfaces. Used only for the sideslip correction (SlipTvcAssist).
			float tvcRollHalf = 0f;
			if (tvcd != null && tvcd.active && tvcd.N > 1)
			{
				float txMax = float.MinValue;
				float txMin = float.MaxValue;
				for (int iT = 0; iT < tvcd.N; iT++)
				{
					txMax = Mathf.Max(txMax, tvcd.M[iT].x);
					txMin = Mathf.Min(txMin, tvcd.M[iT].x);
				}
				float tvFree = (tvc != null && tvc.max > 0.1f) ? Mathf.Clamp01(1f - Mathf.Abs(tvc.now) / tvc.max) : 1f;
				tvcRollHalf = 0.5f * (txMax - txMin) * tvFree;
			}
			aRollTvc = Mathf.Clamp(aRollTvcF.Step(tvcRollHalf * IaInv[0, 0] * 57.29578f, 0.2f, num), 0f, 5000f);
			rollCtrlHalf = aRoll * (MathF.PI / 180f) / Mathf.Max(IaInv[0, 0], 1E-09f);
			aRollWing = Mathf.Clamp(aRollWF.Step(num8 * IaInv[0, 0] * 57.29578f, 0.2f, num), 10f, 5000f);
			aYaw = Mathf.Clamp(aYawF.Step(0.5f * (zero2.z - zero3.z) * IaInv[2, 2] * 57.29578f, 0.2f, num), 5f, 3000f);
			// 2.6.9: how much of the airframe's yaw range all the effectors together are using now (0 = centred, 1 = at the stops):
			// the same effectors and tables as aYaw, read from where they are - whatever carries the yaw at this AoA
			float yawUseRaw = Mathf.Abs(zNowAll - 0.5f * (zero2.z + zero3.z)) / Mathf.Max(1E-06f, 0.5f * (zero2.z - zero3.z));
			yawUseAllF += (Mathf.Clamp01(yawUseRaw) - yawUseAllF) * (num / (0.05f + num));
			float t = Mathf.Clamp01((Mathf.Abs(alpha) - 20f) / 20f);
			float num12 = ((!flightAssist) ? Plugin.MpoGPos : Plugin.GPos);
			float num13 = ((!flightAssist) ? Plugin.MpoGNeg : Plugin.GNeg);
			float t2 = Mathf.Clamp01(Mathf.Max((nz - 0.85f * num12) / (0.15f * num12), (0.85f * num13 - nz) / (0.15f * (0f - num13))));
			stabPitchFrac = 1f;
			foreach (SurfaceEffector surface3 in surfaces)
			{
				if (surface3.kind == SurfaceKind.Stabilator && surface3.active)
				{
					stabPitchFrac = Mathf.Min(stabPitchFrac, surface3.pitchFrac);
				}
			}
			float t3 = 1f - stabPitchFrac;
			pitchNeed = 0f;
			pitchNeedUp = 0f;
			pitchNeedDn = 0f;
			if (Plugin.OvershootPitchPriority && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				float num47 = Mathf.Sign(alpha) * (omega.y * 57.29578f - qT);
				float pnA0 = 15f;
				float pnRamp = 10f;
				if (Plugin.AsymRollCoupling && AsymPitchPrio > 0f && alpha > 0f && asymCapPosEff < 60f)
				{
					float pnG = AsymPitchPrio * Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan);
					pnA0 = Mathf.Lerp(15f, Mathf.Min(15f, asymCapPosEff - AsymPitchPrioLead), pnG);
					pnRamp = Mathf.Lerp(10f, 4f, pnG);
				}
				pitchNeed = Mathf.Clamp01((num47 - 2f) / 8f) * Mathf.Clamp01((Mathf.Abs(alpha) - pnA0) / pnRamp);
				if (prot)
				{
					pitchNeed = 1f;
				}
				pitchNeedUp = pitchNeed;
				// 2.9.0: the capped sweep toward zero AoA (last tick's cap state; the cap runs later in the tick). Only while the cap
				// clipped the command on the toward-zero side (positive AoA: the stick asked for more nose-down than the cap gave,
				// yawSatPitchDbg < 0; negative AoA: more nose-up, > 0) and the nose is moving toward zero faster than the capped
				// command. Not into the coupling roll gate (gateC reads pitchNeedUp): the rotation is not cut, the stabs are taken
				// back for the hold. (Bench: gated on the raw sign it also fired on a push to -20 deg AoA under a roll, where the cap
				// clipped the sweep that grows |AoA| and the pitch merely lagged it: pitch weight 8 -> 16 for 0.2 s, 1.3 deg more
				// sideslip in the pull-through after.)
				pitchNeedDn = 0f;
				if (sweepClipOn && Mathf.Sign(alpha) * sweepClipDbg < -1f)
				{
					float dnOver = Mathf.Sign(alpha) * (qT - omega.y * 57.29578f);
					pitchNeedDn = Mathf.Clamp01((dnOver - SweepHoldErr0) / Mathf.Max(0.5f, SweepHoldErrSpan)) * Mathf.Clamp01((Mathf.Abs(alpha) - 15f) / 10f);
				}
				pitchNeed = Mathf.Max(pitchNeed, pitchNeedDn);
				t3 *= 1f - pitchNeed;
			}
			// Revoker 0.2.7 (HighAoARollWeight): with the nozzles not covering pitch (TVC off / low thrust) the surfaces carry
			// every axis and the 0.35 high-AoA roll weight let pitch and yaw take the roll away under saturation; with the nozzles
			// working the F-22E's 0.35 stays (bench: 1 there loosened the damaged-jet reversal, roll error x3). Faded at 1/s.
			hiRollWF = Mathf.MoveTowards(hiRollWF, noTvcOn ? 1f : 0f, num);
			// Revoker 0.4.0: the allocator's lateral residual in the velocity frame (see Allocator.slipFrame). The AoA is the measured
			// one; on the ground, with the stick in the ground modes and in MPO (separate axes) the body frame stands.
			alloc.slipFrame = SlipFrameAlloc && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off;
			alloc.slipSin = Mathf.Sin(alpha * (MathF.PI / 180f));
			alloc.slipCos = Mathf.Cos(alpha * (MathF.PI / 180f));
			alloc.axisWeight = new Vector3(Mathf.Lerp(alloc.slipFrame ? 1f : Mathf.Lerp(1f, Mathf.Lerp(0.35f, Plugin.HighAoARollWeight, hiRollWF), t), 1.5f, t3), Mathf.Lerp(Mathf.Lerp(8f, 15f, t2), 1f, t3), Mathf.Lerp(Mathf.Lerp(Plugin.YawPriority, (YawPrioHi > 0f && Plugin.SlipYawWeightOn) ? Mathf.Lerp(Mathf.Max(2f, Plugin.YawPriority), Mathf.Min(Plugin.YawPriority, YawPrioHi), YawHiSlipGate()) : Mathf.Max(2f, Plugin.YawPriority), t), Mathf.Max(2.5f, Plugin.YawPriority), t3));
			alloc.PairedTrimSearch = PairedTrim && Plugin.StabTvcPairedSearch;
			// 2.0.6 rudder pitch aid: the symmetric rudder split is cheap only while no roll or yaw is asked for (stick,
			// pedal, or a roll still running); with lateral input the rudders go back to yaw, as the stabs do.
			if (!Plugin.RudderPitchAid)
			{
				foreach (SurfaceEffector seT0 in surfaces)
				{
					if (seT0.kind == SurfaceKind.Rudder && seT0.partner != null)
					{
						seT0.symCost = 2f;
						seT0.symTarget = 0f;
						seT0.noPitch = true;
					}
				}
			}
			else
			{
				float latNow = Mathf.Max(Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.03f) / 0.12f), Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.12f)), Mathf.Clamp01((Mathf.Abs(pS) - 5f) / 20f));
				toeLatGate = Mathf.Max(latNow, toeLatGate - num / ToeLatRelease);
				float toeCost = Mathf.Lerp(Plugin.RudderToeCost, 2f, toeLatGate);
				// 2.0.7 toe schedule: the rudders split with the pitch demand (the pitch loop's desired pitch acceleration as a
				// share of the authority in that direction, x RudderToeGain), up to RudderToeMax each way, in every airborne
				// mode and at any AoA (a symmetric split makes no net roll, so the rudder fade does not apply to it); the
				// allocator pulls the pair toward it (and still uses more split when the stabs and nozzles run short). Zero
				// with lateral input. Pair value +d (both rudders at +d) is nose-up across the whole AoA range in the surface
				// model; nose-down authority is about 40 % of nose-up.
				float nuY = omegaDotDes.y * 57.29578f;
				float share = (nuY >= 0f) ? (nuY / Mathf.Max(20f, aUp)) : (nuY / Mathf.Max(20f, aDn));
				float toeT = (Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Plugin.RudderToeMax * Mathf.Clamp(share * Plugin.RudderToeGain, -1f, 1f) * (1f - toeLatGate)) : 0f;
				toeTgtF = Mathf.MoveTowards(toeTgtF, toeT, 100f * num);
				foreach (SurfaceEffector seT in surfaces)
				{
					if (seT.kind == SurfaceKind.Rudder && seT.partner != null)
					{
						seT.symCost = toeCost;
						seT.symTarget = toeTgtF;
						// in a roll the allocator does not see the rudders' pitch at all (2.0.5 model), so it has no reason to
						// split them; takes effect from the next tick's surface model
						seT.noPitch = toeLatGate > 0.5f;
					}
					if (seT.kind == SurfaceKind.Rudder && Mathf.Abs(toeTgtF) > 0.01f)
					{
						// open the window to the toe value past the rudder fade too (both rudders the same value = no net roll)
						seT.lo = Mathf.Clamp(Mathf.Min(seT.lo, toeTgtF), seT.min, seT.max);
						seT.hi = Mathf.Clamp(Mathf.Max(seT.hi, toeTgtF), seT.min, seT.max);
					}
				}
			}
			// active while the pedals are in (and FightPedalRelease s after): rolls without pedal allocate as in 1.8.40
			float pedAct = Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.15f);
			fightPed = Mathf.Max(pedAct, fightPed - num / Mathf.Max(0.05f, FightPedalRelease));
			float rollIn = Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f);
			fightRollHold = Mathf.Max(rollIn, fightRollHold - num / Mathf.Max(0.05f, FightRollRelease));
			// 2.2.16: RollFightReleased (default on) = FightMode 2: the roll-fight cost also acts while the lateral stick is
			// centred (and FightRollRelease s after). Without it, a crossed set (one stab split against the flaperons, ailerons
			// and rudders at their stops) could hold with the stick centred: 19 Oct 18:05 / 18:32 logs, TVC off.
			int fightMode = (FightMode == 0 && Plugin.RollFightReleased) ? 2 : FightMode;
			// 2.2.16b: with the stick released the cost waits for the pitch to settle (pitch-rate command steady, pitch-rate
			// error under 2-6 deg/s): on a release from a rolling pull the stabs carry the nose-down and their split against
			// the wing is the price of that; pricing it there moved the second stab nose-down too (bench 600 km/h, release
			// at 30 deg AoA: pitch-rate dip -4 -> -8 deg/s). The locked sets formed and held with the pitch settled.
			// 2.2.21: not with a fin lost or the deceleron split working: the surfaces are crossed on purpose there (wing for
			// yaw, stabs for the roll it makes), and pricing it froze the stabs for pitch (19 Oct 22:05 log, fins gone)
			// Ifrit 0.4.1: nor with an engine out or down on thrust: the thrust yaw is held by the tails' differential with
			// the wing taking the roll it makes. Priced, the allocator sat on the cliff of the sign-dependent cost (the wing
			// may cancel the tails' roll only once it over-cancels it) and flipped between two corner sets: tails -10/+21
			// with the inboard flaperon on its stop, or the left tail on its stop at 47 deg local AoA (16 kN of drag) with the
			// nozzle at 15 deg for the pitch (22 Oct 22:15 log, right engine destroyed, right banks at 90-110 m/s).
			// The cost comes back as the yaw saturates (yawSat 0.05 -> 0.35): on a hard pull with the tails spent it keeps the
			// crossed flaperon sets and the splits to what the yaw needs (bench, 300 km/h mil pull to 22 deg: 27 km/h slower
			// after 14 s with the cost simply off).
			// 0.4.3: with an engine OUT only. With an engine down on thrust but running (a third to a half of the split), the
			// cost being off is what let the crossed set hold: 23 Oct 00:51 log, 112.5-122.4 s, stabs +45/+27 (the left on
			// its stop at 41-47 deg of local AoA) against the nozzle at -15 and the crossed wing for 10 s with the stick
			// centred, fightGate 0 - and it collapsed within 0.3 s of the gate coming back at 122.3 s (yawSat 0.5). Of the
			// two corner sets the 0.4.1 exemption was made for, the stab on its stop is now excluded by the settled stall
			// window (0.4.3, StabParkEscape). Bench, the log's push and release at 55 m/s: with the exemption the wing stays
			// crossed (ailerons -30/+26) for 10 s; without it the wing is symmetric within 3 s.
			float relFin = (yawHealth > 0.99f && decelF <= 0.01f) ? 1f : 0f;
			if (engOut || (engAsym && FightExemptAsym))
			{
				relFin *= Mathf.Clamp01((yawSat - 0.05f) / 0.3f);
			}
			float relQuiet = (Plugin.RollFightReleased && FightMode == 0) ? relFin * ((RelQuietHold ? (1f - pfQHold) : 1f) * (1f - Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - RelQuietErr0) / Mathf.Max(0.1f, RelQuietErrSpan)))) : 1f;
			float fightGate = (fightMode == 1) ? 1f : ((fightMode == 2) ? Mathf.Max(fightPed, (1f - fightRollHold) * relQuiet) : fightPed);
			fightGateDbg = fightGate;
			alloc.fightK = ((Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Mathf.Max(0f, Plugin.RollFightCost) * fightGate) : 0f);
			// 1.8.42: stab-vs-nozzle pitch fight, while a roll is commanded (and RpRelease after)
			// not while the pitch-rate command is moving (captures, releases: the nozzles are the fast pitch effector there),
			// in AoA protection or on an AoA run-away
			pfQHold = Mathf.Max(Mathf.Clamp01(Mathf.Abs(qTraj.v) / Mathf.Max(1f, PitchFightQdot)), pfQHold - num / Mathf.Max(0.05f, PitchFightQHold));
			// and not while the nose is coming up faster than commanded: the nozzles have to be free to stop it
			float pfErr = Mathf.Sign(alpha) * (omega.y * 57.29578f - qT);
			if (PitchFightAbsErr) pfErr = Mathf.Abs(pfErr);
			float pfGate = rpLat * (1f - pfQHold) * (prot ? 0f : (1f - pitchNeed)) * (1f - Mathf.Clamp01((pfErr - PitchFightErr0) / Mathf.Max(0.5f, PitchFightErrSpan)));
			alloc.pitchFightSign = ((alpha >= 0f) ? 1f : (-1f));
			alloc.pitchFightK = ((Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (Mathf.Max(0f, Plugin.RollPitchFightCost) * pfGate) : 0f);
			float num14 = 1f + Mathf.Max(0f, Plugin.AsymAxisBoost) * Mathf.Clamp01((asymUNow - 0.5f) / 0.5f);
			if (num14 > 1.001f)
			{
				alloc.axisWeight = new Vector3(alloc.axisWeight.x * num14, alloc.axisWeight.y, alloc.axisWeight.z * num14);
			}
			foreach (SurfaceEffector surface4 in surfaces)
			{
				if (surface4.kind == SurfaceKind.Stabilator && surface4.partner != null)
				{
					surface4.antiCost = Mathf.Lerp(0.0002f, Mathf.Lerp(Plugin.StabRollCost, 0.01f, Mathf.InverseLerp(160f, 240f, ias)), stabPitchFrac);
					surface4.antiCost = Mathf.Lerp(surface4.antiCost, Mathf.Max(surface4.antiCost, AsymStabAntiCost), pitchNeed * Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan));
					// 2.15.0: the lateral reserve prices the stabs' pitch use only where they have pitch use (stabPitchFrac). Past their
					// pitch stall (65 deg AoA, stabPitchFrac 0) the symmetric movement the allocator needs for yaw and roll is not pitch
					// trim, and x 71 on it held the stabs still for 0.5 s at a no-fin reversal while the yaw loop was 60 deg/s^2 short
					// (19:54 log, 164.8 s: symCost 0 -> 0.28 at the stick, stabs parked until the roll target fell away).
					// 2.15.0: the lateral reserve keeps symmetric stab travel for anti-symmetric use. Where the estimator counts the
					// stabs' SYMMETRIC motion as the yaw capacity (rudW -> 0: fins gone or rudders faded) that reserve starves the
					// yaw it is meant to serve (19:54 log, 164.8 s: stabs parked 0.5 s on symCost 0.28), so it goes with the pitch
					// range the stabs still have. With the rudders present it is unchanged.
					float rudWsym = Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f);
					surface4.symCost = Plugin.StabTrimCost * (1f + LatTrimGain * latReserve * (LatReservePitchFrac ? Mathf.Lerp(stabPitchFrac, 1f, rudWsym) : 1f));
					// 2.2.14 test: extra cost on the stab split at high AoA (roll moved to the rudders / flaperons / sideslip)
					if (HiAoAStabRollCost > 0f)
					{
						surface4.antiCost = Mathf.Max(surface4.antiCost, HiAoAStabRollCost * Mathf.Clamp01((Mathf.Abs(alpha) - HiAoAStabRollA0) / Mathf.Max(1f, HiAoAStabRollSpan)));
					}
					// 2.2.5 test switch: stabs pitch only (any antisymmetric deflection made prohibitively expensive)
					if (Plugin.StabsPitchOnly && !isAI)
					{
						surface4.antiCost = StabsPitchOnlyCost;
					}
				}
			}
			// 2.2.29 (RelLatRollPrio): with the lateral stick and pedals released, the roll axis is weighted up by up to this
			// factor as the body roll runs ahead of its command (2 -> 10 deg/s). Stopping a high-AoA rotation brakes the yaw
			// with the rudders, whose inverse roll then rolled the jet toward the yaw with the ailerons already at their stops
			// (20 Oct 10:35 log, falling-leaf stops at 198 / 285 s: body roll 14 -> 34 deg/s against a 19 command)
			if (RelLatRollPrio > 1f)
			{
				float relLat = 1f - Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.03f) / 0.12f), Mathf.Clamp01((Mathf.Abs(sy) - 0.03f) / 0.12f));
				float pErr = Mathf.Abs(omega.x * 57.29578f - pCmd);
				// 2.9.0: not while the capped sweep's pitch hold has the stabs (pitchNeedDn)
				float rlp = Mathf.Lerp(1f, RelLatRollPrio, relLat * Mathf.Clamp01((pErr - 2f) / 8f) * (1f - pitchNeedDn));
				relLatPrioDbg = rlp;
				if (rlp > 1.001f)
				{
					alloc.axisWeight = new Vector3(alloc.axisWeight.x * rlp, alloc.axisWeight.y, alloc.axisWeight.z);
				}
			}
			float relStick = 1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f);
			float relErr = Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - 1f) / 8f);
			pitchPrio = Mathf.Lerp(1f, Mathf.Max(1f, Plugin.ReleasedPitchPriority), stabPitchFrac * relStick * relErr);
			if (pitchPrio > 1.001f)
			{
				alloc.axisWeight = new Vector3(alloc.axisWeight.x, alloc.axisWeight.y * pitchPrio, alloc.axisWeight.z);
			}
			// Revoker 0.2.8: not while the TrailingEdgeBrakeConfig set moves in or out: the elevons' common deflection needs the
			// canards to re-trim with it, and bounded to their own (near-zero) rate they held both still for 0.7 s mid-way and
			// let go in one step (bench, 400 km/h: elevons 11 -> 18 deg in 0.2 s, nz 1.07 -> 0.81)
			bool teBrakeMoving = Mathf.Abs(teBrakeAir - teBrakeAirF) > 0.001f || (teBrakeLandF > 0.001f && teBrakeLandF < 0.999f);
			teBrakeMov = teBrakeMoving;
			// 2.4.6 telemetry: the allocator's axis weights after the priority terms (roll, pitch, yaw)
			axWDbg = alloc.axisWeight;
			// 2.9.1: also while the capped sweep's pitch hold has the stabs (pitchNeedDn). 16:29 log (2.9.0), 75.0 s: the hold took
			// the weights (pitch 8, roll 0.5) and the allocator still left the left stab at -6 while it sent the right from -30 to
			// +35 - in command space one stab's travel covered the pitch increment, so the other stayed on the roll; the servo
			// took 0.9 s over the 65 deg and the nose dropped 40 -> 20 deg AoA meanwhile, then the same stab swung back +35 -> -30
			// for the push once the hold let go: the "hitch" at 17 deg AoA (q -2 for 0.3 s, then -47). Bounded to what the servos
			// reach in 0.1 s, the increment is spread over both stabs and the nozzles and no stab swings stop to stop.
			// 2.41.0 (PushPlanReach): through a planned push the allocator balances the nozzles and wings against where the stabs
			// will be within PushPlanReachTau s, not where they are sent (see Allocator.ReachPass)
			// 2.43.0: the stab rate bound (ReleasedStickStabRateBound, 1.8.14 / 2.9.1) is replaced by the allocator's reach model, in the same
			// places (pitch stick released, or the capped sweep's pitch hold, scaled by the stabs' pitch share) and under a planned push.
			// The bound clipped each stab command to 0.1 s of servo travel. The reach model keeps the command and places the other surfaces
			// against where the stab will be in PushPlanReachTau (Allocator.ReachPass), then compares that plan with the stab moved only
			// as far as it gets in that time, over both horizons and the stab's whole transit (Allocator.TwoHorizon): the bound is the
			// second plan, chosen when the first one's transit costs more than its destination saves. Not under a plan (2.41.0 unchanged).
			// The two-horizon pick matters: with the reach pass alone a roll entry at -43 deg AoA (23:37 log, 50.6 s) sent the right stab to
			// the far stop the wrong way through its range, the roll lagged 0.4 s (pinned command dip 2.8 -> 5.8 deg/s, sideslip 5.7 -> 7.4).
			float relG = Mathf.Max(relStick, pitchNeedDn) * stabPitchFrac;
			// Revoker: not while the TrailingEdgeBrakeConfig set moves in or out (see teBrakeMoving: the 0.2.8 exception the stab rate bound had)
			bool reachHere = ((PushPlan && pushPlanOn) || relG > 0.01f) && !teBrakeMoving;
			stabRateBound = false;
			foreach (SurfaceEffector seR in surfaces)
			{
				seR.reachTau = (seR.kind == SurfaceKind.Stabilator && reachHere && PushPlanReachTau > 0f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? PushPlanReachTau : 0f;
				// Revoker 0.7.0 (RudderReach): the rudder too, always (in flight). It crosses its +-25 deg in ~0.35 s and the solve placed the
				// yaw nozzle against where the rudder was SENT: 00:44 log, falling-leaf reversals at 73.0 / 76.9 s (69 deg AoA): the rudder sent
				// -2 -> -25 at the yaw crossing, the nozzle backed off -15 -> -2 as if the rudder were there, the rudder then flipped -25 <-> +25
				// every few ticks until the nozzle came back for the new side (11-13 deg of nozzle given up, 0.9-1.1 s to the roll's zero).
				// Placed against where the rudder will be in PushPlanReachTau (0.1 s), the nozzle keeps braking. Bench suite: no case worse by
				// more than 0.4 deg, roll-then-pull-rev 400 7.6 -> 4.6, sweep reversal worst 9.5 -> 8.5. The leaf dither itself is not on the
				// bench (its fin sidewash differs): flight-validated.
				if (RudderReach && seR.kind == SurfaceKind.Rudder && PushPlanReachTau > 0f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
				{
					seR.reachTau = PushPlanReachTau;
				}
				seR.twoHorizon = !(PushPlan && pushPlanOn);
			}
			// 1.8.40: while rolling at low/moderate AoA, the stabs are not driven past their lift peak in any pass: beyond
			// it the down-going stab's lift falls off and its nose-down turns into pitch-up exactly when it rolls hardest.
			// 2.9.2: also during the capped sweep toward zero AoA with the stabs' pitch range back (see SweepHoldStallWindow)
			sweepHoldWinDbg = !isAI && sweepClipOn && Mathf.Sign(alpha) * sweepClipDbg < -1f && stabPitchFrac > 0.05f && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off;
			if (sweepHoldWinDbg)
			{
				foreach (SurfaceEffector sw in surfaces)
				{
					if (sw.kind == SurfaceKind.Stabilator && sw.active)
					{
						StallWindow(sw, sw.stallAoA + RpMainStallMargin);
					}
				}
			}
			// 2.2.15: with the lateral stick and pedals centred and the AoA under StabParkAoA, a stab past its lift peak is walked
			// back to the attached side. On the stall plateau its modelled moment hardly changes with deflection, so the
			// allocator had no gradient to bring it back and held every other surface against it: 19 Oct 18:05 log (TVC off),
			// 212-227 s, left stab at -30 (local AoA 31-35 deg against a 25 deg peak), flaperons and rudders at their stops
			// against it, for 15 s with the stick centred and the AoA falling 45 -> 0.
			if (Plugin.StabParkEscape && !isAI && Mathf.Abs(sr) < 0.05f && Mathf.Abs(sy) < 0.05f && Mathf.Abs(alpha) < StabParkAoA && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				foreach (SurfaceEffector sw in surfaces)
				{
					if (sw.kind == SurfaceKind.Stabilator && sw.active)
					{
						StallWindow(sw, sw.stallAoA + RpMainStallMargin);
					}
				}
			}
			// 0.4.3 (the F-22E's 2.3.0/2.3.1 settled stall window, ported, on top of the 2.2.15 one - which stays: it keeps a
			// stab from running past the 45 deg peak into falling lift, and the engine-out trims lean on it (bench, engine-out
			// pull at 250 km/h without it: 9.9 deg of sideslip and the VMC cutting the throttle, against 2.9 and none).
			// The 2.2.15 window only acts below 30 deg AoA and allows the peak itself. On this jet the stabilator foil has no
			// stall before 45 deg: its lift keeps
			// rising from 20 to 45 deg of local AoA at a fifth of the linear slope and less (0.17 /rad at 40), so a stab parked
			// out there is "attached" to the 2.2.15 window and the allocator has next to no gradient to bring it back, while it
			// makes its maximum lift: 23 Oct 00:51 log, 112.5-122.4 s, after a full push through zero AoA at 58 m/s with an
			// engine down on thrust - left stab on its stop at +45 (local AoA -41 to -47) for 10 s with the stick centred, the
			// right at +27, the nozzle at -15 against them, the wing crossed, 3-7 deg of sideslip at 70-130 m/s, released all at
			// once at 128 m/s. Bench, same push at 55 m/s: right stab at -37.5/-40.5 (local 47-49) for 10 s.
			// Now: with the lateral stick and pedals centred and the lateral rates settled (the stop finished), the stabilators
			// and rudders may not rest past the smaller of (peak - SettledStallMargin) and the foil's lift-slope knee (where
			// the slope has fallen to SettledSlopeFrac of its steepest, 26 deg on this foil at 0.15) of local AoA, at any AoA
			// where such an attached range exists (none: the window is left open); a surface outside is walked in at
			// SettledWalkRate. The gate latches on below SettledRateOn deg/s of body roll and yaw and off above SettledRateOff,
			// or with lateral input. Telemetry: settledWin.
			// Not while the thrust yaw is loading the fins (|asymAy| / asymCy above SettledYawUse): this jet has no rudders -
			// its yaw is the canted stabilators (10 deg of yaw per 35 of pitch), the nozzles and the deceleron split - and with
			// an engine out a stab run out to 47-63 deg of local AoA is its drag rudder. Bench, engine-out pull at 250 km/h to
			// 22 deg: windowed, the yaw went to the deceleron split, saturated (7.6 deg of sideslip) and the VMC cut the
			// throttle to 0.5 for 3 s; the thrust yaw there is 1.2 x the fins' authority, against 0.29 in the 00:51 log's set.
			// (the usage filtered 0.5 s, with 0.1 of hysteresis: the raw value dips through 0.5 in the engine-out pulls as the
			// AoA moves, and the window flickering on there cost 1-2 deg of sideslip)
			float yawUse = (asymValid && asymCyNow > 1f) ? (Mathf.Abs(asymAyNow) / asymCyNow) : 0f;
			settledUseF += (yawUse - settledUseF) * Mathf.Clamp01(num / 0.5f);
			if (settledUseF > Plugin.SettledYawUse)
			{
				settledUseHi = true;
			}
			else if (settledUseF < Plugin.SettledYawUse - 0.1f)
			{
				settledUseHi = false;
			}
			if (Plugin.StabParkEscape && !isAI && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
			{
				float latRate = Mathf.Max(Mathf.Abs(omega.x), Mathf.Abs(omega.z)) * 57.29578f;
				bool latIn = Mathf.Abs(sr) >= 0.05f || Mathf.Abs(sy) >= 0.05f;
				if (latIn || latRate > Plugin.SettledRateOff || settledUseHi)
				{
					settledWin = false;
				}
				else if (latRate < Plugin.SettledRateOn)
				{
					settledWin = true;
				}
				if (settledWin)
				{
					foreach (SurfaceEffector sw in surfaces)
					{
						if ((sw.kind == SurfaceKind.Stabilator || (SettledWindowRudders && sw.kind == SurfaceKind.Rudder)) && sw.active)
						{
							float lim = Mathf.Min(sw.stallAoA - Plugin.SettledStallMargin, sw.KneeAoA(Plugin.SettledSlopeFrac));
							SettledWindow(sw, Mathf.Max(5f, lim), Plugin.SettledWalkRate, num);
						}
					}
				}
			}
			else
			{
				settledWin = false;
			}
			// Revoker 0.2.7 (CanardAttachedWindow): at high AoA either sign, the canards are walked into the deflections where their
			// own flow still has lift slope (|local AoA| up to the foil's knee + CanardWindowMargin), whenever such deflections exist in
			// their travel. Left alone, past the knee the allocator sees no roll / yaw gradient on them and parks them stalled
			// (28 Oct 21:18 log, full push + roll at -50 deg: both canards at -1 deg, local AoA -48..-50, for 20 s).
			// Revoker 0.2.7: with a canard shot off, the one left goes back to its stock 40 deg. The extra 5 deg let it trim the jet
			// to 51-53 deg against the asymmetric-load cap (bench: canard_L + wingtip_R + aileron_R + flap_L gone, full pull at
			// 290-300 km/h; 0.2.6: 25-38 deg), with the roll error up from 1.4-1.7 to 7.7-8.6 deg/s.
			if (AsymStabTravel > 0f)
			{
				bool stabGone = false;
				foreach (SurfaceEffector sg in surfaces)
				{
					if (sg.kind == SurfaceKind.Stabilator && !sg.Valid)
					{
						stabGone = true;
					}
				}
				if (stabGone)
				{
					foreach (SurfaceEffector sg in surfaces)
					{
						if (sg.kind == SurfaceKind.Stabilator && sg.active)
						{
							sg.lo = Mathf.Max(sg.lo, 0f - AsymStabTravel);
							sg.hi = Mathf.Min(sg.hi, AsymStabTravel);
							if (sg.lo > sg.hi)
							{
								sg.lo = (sg.hi = Mathf.Clamp(sg.cmd, 0f - AsymStabTravel, AsymStabTravel));
							}
						}
					}
				}
			}
			canWinDbg = 0f;
			if (Plugin.CanardAttachedWindow && !settledWin && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off && (alpha < 0f - CanWinA0 || (alpha > CanWinA0 && (noTvcOn || !CanWinNegOnly))))
			{
				bool latInC = Mathf.Abs(sr) >= 0.05f || Mathf.Abs(sy) >= 0.05f || Mathf.Max(Mathf.Abs(omega.x), Mathf.Abs(omega.z)) * 57.29578f > Plugin.SettledRateOff;
				if (latInC || !CanWinLatOnly)
				{
					foreach (SurfaceEffector sw in surfaces)
					{
						if (sw.kind == SurfaceKind.Stabilator && sw.active)
						{
							SettledWindow(sw, sw.KneeAoA(Plugin.SettledSlopeFrac) + Plugin.CanardWindowMargin, CanWinRate, num);
							canWinDbg = 1f;
						}
					}
				}
			}
			settledWinDbg = settledWin;
			nAlphaTimer += num;
			if (nAlphaTimer > 0.1f)
			{
				nAlphaTimer = 0f;
				ComputeEnvelope(vector, num2, transform.right, vector6);
				ComputeHybridSlopes(vector, num2, transform.right);
			}
			AsymStep(num, num2, rotation, ref ctx, vector7, vector8);
			tw = tvc.totalThrust / (mass * 9.81f);
			float num15 = 0.5f * num2 * tas * tas;
			float num16 = 13781.25f;
			float num17 = Mathf.Clamp01(num15 / num16);
			num17 *= num17;
			float t4 = Mathf.Clamp01(Plugin.LowQRegRelief) * Mathf.InverseLerp(40f, 60f, ias);
			alloc.regScale = Mathf.Lerp(1f, Mathf.Max(0.02f, num17), t4);
			if (!revFlow && Mathf.Abs(alpha) > 100f)
			{
				revFlow = true;
			}
			else if (revFlow && Mathf.Abs(alpha) < 80f)
			{
				revFlow = false;
			}
			if (revFlow != revFlowLogged)
			{
				revFlowLogged = revFlow;
				Plugin.Log.LogInfo(revFlow ? "Revoker FCS: reversed flow (flying backwards) - roll sense flipped to body axes." : "Revoker FCS: forward flow.");
			}
			bool flag3 = false;
			bool flag4 = false;
			bool flag5 = false;
			LandingGear landingGear = NoseGear();
			foreach (LandingGear gear2 in gears)
			{
				if (!(gear2 == null) && gear2.WeightOnWheel(0.05f))
				{
					flag3 = true;
					if (gear2 == landingGear)
					{
						flag5 = true;
					}
					else
					{
						flag4 = true;
					}
				}
			}
			groundFrames = (flag3 ? Mathf.Min(groundFrames + 1, 1000) : 0);
			onGround = flag3;
			airTime = (flag3 ? 0f : (airTime + num));
			bool flag6 = flag3 || airTime < 0.2f;
			noseUpTime = ((landingGear != null && flag4 && !flag5 && tas > Plugin.RotationMinSpeed) ? (noseUpTime + num) : 0f);
			if (noseUpTime > 0.15f)
			{
				rotating = true;
				noseDownTime = 0f;
			}
			else if (rotating)
			{
				noseDownTime = (flag5 ? (noseDownTime + num) : 0f);
				if (!flag3 || noseDownTime > 0.3f || tas < Plugin.RotationMinSpeed - 8f)
				{
					rotating = false;
				}
			}
			mainsOnGround = flag4;
			// 2.0.6: pitch-rate command in every airborne mode on request (MPO stays MPO: it is rate command already)
			FcsMode airMode = (!flightAssist) ? FcsMode.MPO : ((!ac.gearDeployed) ? (isAI ? FcsMode.AIRate : (Plugin.RateCommandOnly ? FcsMode.Rate : FcsMode.AoAG)) : FcsMode.GearDownRate);
			FcsMode fcsMode;
			if (Plugin.GroundRateLaw && landingGear != null)
			{
				// 2.0.6 ground pitch: the gear-down rate law flies the pitch axis on the runway as soon as the pilot pulls
				// above RotationMinSpeed, and keeps it through the rotation and the lift-off. 2.0.5 flew an open-loop stick
				// map until the nose wheel had been off for 0.15 s, then an 8 deg/s rotation law, and dropped back to the
				// open-loop map for 0.2 s every time the mains' weight-on-wheel flickered as the wing took the weight: full
				// stab and nozzle deflection for those frames, the spike at lift-off. Contact here is 'touching' (0.2 % of
				// the leg travel) rather than 'carrying weight' (5 %), so a lightly loaded leg still counts.
				bool noseC = landingGear.WeightOnWheel(0.002f);
				bool mainC = false;
				foreach (LandingGear gC in gears)
				{
					if (gC != null && gC != landingGear && gC.WeightOnWheel(0.002f))
					{
						mainC = true;
					}
				}
				bool anyC = noseC || mainC;
				wheelsTouch = anyC;
				gContactAir = anyC ? 0f : (gContactAir + num);
				// Revoker 0.2.8 (TrailingEdgeBrakeConfig), touchdown with the brake held and the stick not pulled back:
				// - mains only: the gear-down rate law stays on and lowers the nose at DerotationRate (derotGate, pitch law),
				//   with the elevons already trailing edge up (lift dump), so the canards go leading edge down to fly it;
				// - nose wheel down too: straight to the ground mode and its full rollout aero-brake. Before, the rate law held
				//   the attitude until the stick had been neutral for 0.3 s with all wheels down.
				// Forcing the open-loop ground mode at the mains contact instead let the nose fall on its own (bench, 260 km/h,
				// 10 deg touchdown: -13.6 deg/s at the nose-wheel contact, 210 kN on the nose leg against 13 kN static).
				derotGate = TeBrakeOn && mainC && !noseC && airBrake > 0.05f && sp <= 0.08f && tas >= Plugin.RotationMinSpeed;
				bool brakeLand = TeBrakeOn && mainC && noseC && airBrake > 0.05f && sp <= 0.08f;
				if (anyC && tas >= Plugin.RotationMinSpeed && brakeLand)
				{
					groundRateOn = false;
				}
				else if (anyC && tas >= Plugin.RotationMinSpeed)
				{
					if (!groundRateOn)
					{
						if (!noseC || sp > 0.08f)
						{
							groundRateOn = true;
							gNeutralT = 0f;
						}
					}
					else
					{
						gNeutralT = (noseC && mainC && sp < 0.03f) ? (gNeutralT + num) : 0f;
						if (gNeutralT > 0.3f)
						{
							groundRateOn = false;
						}
					}
				}
				else if (anyC && tas < Plugin.RotationMinSpeed - 8f)
				{
					groundRateOn = false;
				}
				else if (!anyC && gContactAir > 0.5f)
				{
					groundRateOn = false;
				}
				noseContactLaw = noseC && groundRateOn;
				if (anyC)
				{
					fcsMode = groundRateOn ? FcsMode.GearDownRate : FcsMode.Ground;
				}
				else
				{
					// a bounce in the open-loop ground mode stays there for 0.2 s, as before, unless the pilot pulls
					fcsMode = (Mode == FcsMode.Ground && gContactAir < 0.2f && sp <= 0.08f && tas < 110f) ? FcsMode.Ground : (groundRateOn ? FcsMode.GearDownRate : airMode);
				}
			}
			else
			{
				noseContactLaw = false;
				derotGate = false;
				fcsMode = ((rotating && flag4) ? FcsMode.GearDownRate : ((flag6 && (tas < 110f || groundFrames > 10)) ? FcsMode.Ground : airMode));
			}
			if (fcsMode != Mode)
			{
				if (Mode == FcsMode.Ground || Mode == FcsMode.Off)
				{
					qT = omega.y * 57.29578f;
					qTprev = qT;
					qTraj.Reset(qT);
					pTraj.Reset(0f);
					mpoRTraj.Reset(0f);
					mpoPTraj.Reset(0f);
					pS = 0f;
					aInt = 0f;
					capIUp = (capIDn = 0f);
					gInt = 0f;
				}
				Mode = fcsMode;
			}
			if (Mode != lastLoggedMode)
			{
				Plugin.Log.LogInfo("Revoker FCS mode: " + Mode);
				lastLoggedMode = Mode;
			}
			gbLogTimer += num;
			if (Plugin.GroundBrakeLog && flag3 && gbLogTimer > 1f)
			{
				gbLogTimer = 0f;
				LandingGear landingGear2 = NoseGear();
				Plugin.Log.LogInfo(string.Format("Revoker FCS ground brake: mode {0}, mainsWow {1}, noseWow {2} (nose gear {3}), ", Mode, flag4, flag5, (landingGear2 != null) ? landingGear2.name : "none") + $"tas {tas:F1} (min {Plugin.GroundBrakeMinSpeed:F0}), brake {inp.brake:F2}, throttle {inp.throttle:F2} (< {Plugin.GroundBrakeThrottle:F2}), " + string.Format("rotating {0}, blend {1:F2}, airbrake {2:F2} (split {3})", rotating, groundBrake, airBrake, SplitDeploy ? "out" : "stowed"));
			}
			float num18 = num / (Plugin.StickSmoothing + num);
			spF += (Mathf.Clamp(0f - inp.pitch, -1f, 1f) - spF) * num18;
			srF += (Mathf.Clamp(inp.roll, -1f, 1f) - srF) * num18;
			syF += (Mathf.Clamp(inp.yaw, -1f, 1f) - syF) * num18;
			sp = Mathf.MoveTowards(sp, spF, 8f * num);
			sr = Mathf.MoveTowards(sr, srF, RollStickRate * num);
			sy = Mathf.MoveTowards(sy, syF, 8f * num);
			// Ifrit 0.2.1: the brake command: the brake key, or with ThrottleIndependentAirbrake off zero throttle instead (stock rule).
			// In the air it is the speedbrake (AirbrakeOut), on the main wheels the rollout aero-brake below.
			// F-22E 2.4.15 / Ifrit 0.5.5 / Revoker 0.2.6: with ThrottleIndependentAirbrake off the brake key no longer opens it: the stock
			// rule only (Airbrake.Update and the split surfaces open at throttle == 0; the stock Brake action is the wheel brake alone).
			// Before, the brake key opened it as well, so a controller with Brake and Countermeasures on one button popped the
			// airbrake with every flare in the air.
			bool abCmd = Plugin.ThrottleIndependentAirbrake ? (Mathf.Clamp01(inp.brake) > 0.05f) : (rawThrottle <= 0f);
			airBrake = Mathf.MoveTowards(airBrake, abCmd ? 1f : 0f, 2f * num);
			gsPedRaw = Mathf.Clamp(inp.yaw, -1f, 1f);
			GroundSteer(num, flag4, flag5);
			Vector3 vector11;
			tvc.anchorK = 0f;
			if (Mode == FcsMode.Ground)
			{
				float num19 = 0f - sp;
				// Revoker 0.2.18: canard travel known at a standstill (see SurfaceEffector.EnsureAsymRange)
				foreach (SurfaceEffector seA in surfaces)
				{
					if (seA.kind == SurfaceKind.Stabilator && seA.active)
					{
						seA.EnsureAsymRange(((Component)(object)ac).transform);
					}
				}
				float num20 = sr;
				if (gsActive && gsMainsContact)
				{
					num20 = Mathf.Lerp(Mathf.Clamp((0f - phi) / GsPhiFull, -1f, 1f), sr, Mathf.Clamp01(Mathf.Abs(sr) / 0.3f));
				}
				float num21 = (gsActive ? gsRud : sy);
				float target = 0f;
				bool flag7 = flag4;
				bool flag8 = inp.brake > 0.05f || (!Plugin.ThrottleIndependentAirbrake && rawThrottle <= 0f) || (Plugin.GroundBrakeThrottle >= 0f && inp.throttle < Plugin.GroundBrakeThrottle);
				if (TeBrakeOn && flag7 && flag8 && tas >= Plugin.GroundBrakeMinSpeed)
				{
					target = Plugin.GroundAeroBrake * airBrake;
				}
				groundBrake = Mathf.MoveTowards(groundBrake, target, 1.5f * num);
				if (brakeBias == null || brakeBias.Length != surfaces.Count)
				{
					brakeBias = new float[surfaces.Count];
				}
				for (int k = 0; k < surfaces.Count; k++)
				{
					brakeBias[k] = 0f;
				}
				if (groundBrake > 0.01f)
				{
					for (int l = 0; l < surfaces.Count; l++)
					{
						SurfaceEffector surfaceEffector = surfaces[l];
						if (surfaceEffector.active && surfaceEffector.kind != SurfaceKind.Rudder)
						{
							// Revoker 0.2.8: geometry sign as a fallback (teUp is learned from the flow only above 10 m/s). Canards:
							// trailing edge up = leading edge down, full GroundBrakeCanard (the nose-down sense on a canard).
							float teUp = surfaceEffector.GeoTeUp(((Component)(object)ac).transform);
							if (teUp != 0f)
							{
								brakeBias[l] = groundBrake * teUp * ((surfaceEffector.kind == SurfaceKind.Stabilator) ? Plugin.GroundBrakeStab : Plugin.GroundBrakeFlap);
							}
						}
					}
					for (int m = 0; m < surfaces.Count; m++)
					{
						SurfaceEffector surfaceEffector2 = surfaces[m];
						if (surfaceEffector2.kind == SurfaceKind.Rudder && surfaceEffector2.partner is SurfaceEffector surfaceEffector3 && surfaceEffector2.active && surfaceEffector3.active)
						{
							int num22 = surfaces.IndexOf(surfaceEffector3);
							if (num22 >= 0)
							{
								float num23 = groundBrake * Plugin.GroundBrakeRudder;
								brakeBias[m] = surfaceEffector2.pairSign * num23;
								brakeBias[num22] = (0f - surfaceEffector3.pairSign) * num23;
							}
						}
					}
				}
				for (int n = 0; n < surfaces.Count; n++)
				{
					SurfaceEffector surfaceEffector4 = surfaces[n];
					// 2.0.7: on the runway the rudders split with the pitch stick too (pull = the nose-up split), as for rotation
					float gToe = (Plugin.RudderPitchAid && surfaceEffector4.kind == SurfaceKind.Rudder) ? ((0f - num19) * Plugin.RudderToeMax) : 0f;
					// Ifrit 0.3.6 (DeceleronHldFold): on the runway the decelerons carry the HLD's flap share as droop (TE-down = -teUp)
					float gDroop = (surfaceEffector4.kind == SurfaceKind.Flaperon && flaps != null && Plugin.DeceleronHldFold) ? ((0f - surfaceEffector4.teUp) * flaps.decHl) : 0f;
					// Revoker 0.2.1 (GroundElevonPitch): on the runway the trailing-edge elevons follow the pitch stick too (pull = TE up),
					// as fractions of their own travel; with roll on top the up-going side saturates first
					float gElev = (surfaceEffector4.kind == SurfaceKind.Aileron && Plugin.ElevonPitch) ? ((0f - num19) * surfaceEffector4.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundElevonPitch * Mathf.Abs(surfaceEffector4.origRollRange)) : 0f;
					// Revoker 0.2.18: on the runway full pitch stick takes the canards to their full travel that way (push: leading edge
					// down 70, pull: leading edge up 45), not stick x the stock pitch range (~35 deg): with the +70 travel a full push
					// reached only half of it (30 Oct logs: stick 0.8 -> 27.7 deg), and only push + brake together reached the stop.
					float gPitch = num19 * surfaceEffector4.origPitchRange;
					if (surfaceEffector4.kind == SurfaceKind.Stabilator && Mathf.Abs(gPitch) > 0.0001f)
					{
						gPitch = Mathf.Abs(num19) * Mathf.Sign(gPitch) * surfaceEffector4.LimitFor(gPitch);
					}
					surfaceEffector4.cmd = Mathf.Clamp(gPitch + num20 * surfaceEffector4.origRollRange + num21 * surfaceEffector4.origYawRange + brakeBias[n] + gToe + gDroop + gElev, surfaceEffector4.min, surfaceEffector4.max);
				}
				foreach (HingeFlapEffector hG in hinges)
				{
					float gElevH = (0f - num19) * hG.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundElevonPitch * hG.max;
					// Revoker 0.2.8 (TrailingEdgeBrakeConfig): the inboard elevons go trailing edge up in the rollout aero-brake too
					float gBrkH = groundBrake * hG.GeoTeUp(((Component)(object)ac).transform) * Plugin.GroundBrakeInboard;
					hG.cmd = Mathf.Clamp(num20 * hG.pairSign * hG.max + hG.symTarget + gElevH + gBrkH, hG.min, hG.max);
				}
				tvc.cmd = Mathf.Clamp((0f - num19) * tvc.max + groundBrake * Plugin.GroundBrakeTvc, tvc.min, tvc.max);
				// Ifrit 0.2.2: no allocator on the ground, so with the brake out the splits are held at zero retraction (full
				// brake) here; a command left over from the air would otherwise hold the split in on the runway
				if (splitBrakeMode)
				{
					foreach (SplitEffector sg in splits)
					{
						sg.cmd = 0f;
					}
				}
				qCmd = (qT = omega.y * 57.29578f);
				qTprev = qT;
				qTraj.Reset(qT);
				pTraj.Reset(0f);
				mpoRTraj.Reset(0f);
				mpoPTraj.Reset(0f);
				pCmd = omega.x * 57.29578f;
				rCmd = omega.z * 57.29578f;
				omegaDotDes = Vector3.zero;
				unload = (prot = false);
				pS = 0f;
				gInt = 0f;
				aInt = 0f;
				capIUp = (capIDn = 0f);
				vector11 = vector8;
				tvcPitchInput = Mathf.Clamp(num19, -1f, 1f);
			}
			else
			{
				PitchLaw(num, vc);
				LateralLaw(num);
				if (gsActive && gsMainsContact)
				{
					// rolling on the mains with the nose up (rotation / landing flare-out): the ground yaw-rate command replaces
					// the airborne sideslip law, so a crosswind cannot weathervane the jet off the line
					rCmd = gsRCmdMains;
					rDotFF = 0f;
					// on the mains the roll axis is a bank hold in the air law; a crabbed touchdown's tyre side force (2 m below
					// the CG) tips the jet onto one main and the neutral stick would then hold that bank. Level the wings instead,
					// fading out with roll stick.
					pCmd = Mathf.Lerp(Mathf.Clamp((0f - GsKphi) * phi, -30f, 30f), pCmd, Mathf.Clamp01(Mathf.Abs(sr) / 0.3f));
				}
				qTprev = qT;
				Vector3 vector12 = new Vector3(pCmd, qT, rCmd) * (MathF.PI / 180f) - omega;
				// 2.2.5 pitch lead with lateral stick in a pull: the stabs go to the roll (latReserve) and the nozzles, slewing
				// ~70 deg/s, carry the pitch. The rate loop then reads the pitch-rate error ~0.3 s late, drives the nozzles
				// stop to stop and the pitch rate swings 20-40 deg/s about its command (14 Oct 18:36 log, 666-671 s: full aft
				// from a 525 km/h dive, full roll at 37 deg AoA, q 61 against 48, then 10 against 46, AoA 66.6 at the 65 limit).
				// The measured pitch rate is taken PitchLeadT x nozzle share ahead along the measured pitch acceleration.
				pitchLeadDbg = 0f;
				// pulls only (positive pitch stick, positive AoA): the push + roll corkscrews are left as they were
				float leadT = PitchLeadT * Plugin.RollPitchLead / 0.25f;
				if (leadT > 0f && Plugin.LateralStabReserve > 0.01f && latReserve > 0.001f && sp > 0f && alpha > 0f)
				{
					float gL = Mathf.Clamp01(latReserve / Plugin.LateralStabReserve) * CapTvcShare();
					pitchLeadDbg = leadT * gL;
					// (lead on q itself; taking the trajectory's own acceleration out doubled the bench hitch, 6 -> 12 deg/s)
					vector12.y -= pitchLeadDbg * vector7.y;
				}
				// 2.4.18 yaw slew lead: the rudders reach the other side of their travel no faster than their servo allows (35 deg
				// at 100 deg/s), so the yaw loop reads the yaw-rate error late by the time they need to swing back through neutral,
				// keeps accelerating the yaw past its command and then cannot brake it in time. 30 Oct 22:36 log, 51.6-52.4 s:
				// rudders at +35, yaw rate 42 against a command of 28 -> 7, sideslip -12.4 on the reversal that followed (the
				// 2.4.17 roll limit held the roll to 30-51 deg/s and did not prevent it). The measured yaw rate is taken ahead
				// along the measured yaw acceleration by that time (antisymmetric rudder deflection / servo rate), as the 2.2.5
				// pitch lead does for the nozzles. Rudders near neutral = no lead, so a yaw axis that is keeping up is unchanged.
				yawLeadDbg = 0f;
				if (Plugin.YawSlewLead && YawSlewLeadK > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.Rate))
				{
					// Revoker 0.3.0: the time each yaw effector (rudder, yaw nozzle) needs to swing back through neutral, weighted by
					// its one-side yaw (the yaw the loop will have to wait for is each one's share)
					CollectYawEffs();
					float tLeadSum = 0f;
					float wLeadSum = 0f;
					foreach (YawEff yL in yawEffs)
					{
						YawRange(yL, out float zH, out float zL);
						float wY = 0.5f * Mathf.Abs(zH - zL) * yL.auth;
						tLeadSum += wY * Mathf.Abs(yL.e.now) / Mathf.Max(1f, yL.e.rate);
						wLeadSum += wY;
					}
					if (wLeadSum > 1E-06f)
					{
						float gYL = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - StopSlipA0) / Mathf.Max(1f, StopSlipA1 - StopSlipA0));
						float tauY = YawSlewLeadK * gYL * tLeadSum / wLeadSum;
						// only where the yaw is carried onto or past its command within that time (the error closing by at least
						// half of itself); a yaw rate still building toward a far command is left alone, so roll entries keep their
						// yaw build-up
						float travel = tauY * (vector7.z - rCmdRateF * (MathF.PI / 180f));
						float kY = (Mathf.Abs(vector12.z) > 1E-04f) ? (travel / vector12.z) : ((Mathf.Abs(travel) > 1E-04f) ? 9f : 0f);
						float gO = Mathf.Clamp01((kY - 0.5f) / 0.5f);
						gO = gO * gO * (3f - 2f * gO);
						yawLeadDbg = tauY * gO;
						vector12.z -= yawLeadDbg * vector7.z;
					}
				}
				omegaDotDes = new Vector3(Plugin.KRoll * vector12.x + pDotFF * (MathF.PI / 180f), Plugin.KPitch * vector12.y + 0.5f * qTraj.v * (MathF.PI / 180f), Plugin.KYaw * vector12.z + rDotFF * (MathF.PI / 180f));
				float mDotY = (mLeadInit ? ((vector8.y - mPrevY) / num) : 0f);
				mPrevY = vector8.y;
				mLeadInit = true;
				mDotYF = mDotYF + (mDotY - mDotYF) * (num / (MLeadTau + num));
				vector11 = vector8 + MulI(omegaDotDes - vector7);
				vector11.y -= Plugin.HybridIndiGain * hybridDM;
				vector11.y -= MulI(new Vector3(0f, distLeadAcc, 0f)).y;
				if (Plugin.ReleasedMomentLead > 0f)
				{
					float leadGate = (1f - Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f)) * (1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / 20f)) * ((Mathf.Abs(qT) < 1f && !prot && !unload) ? 1f : 0f) * stabPitchFrac;
					mLeadGate = Mathf.MoveTowards(mLeadGate, leadGate, num / 0.2f);
					float leadM = Plugin.ReleasedMomentLead * mLeadGate * mDotYF * Mathf.Clamp01(1f - Plugin.HybridIndiGain); // 1.9.0: the hybrid INDI term covers this lag
					float leadErr = vector12.y * 57.29578f;
					mLeadAgree = ((LeadAgreeGate && leadM * leadErr < 0f) ? (1f - Mathf.Clamp01((Mathf.Abs(leadErr) - 1f) / 2f)) : 1f);
					vector11.y += leadM * mLeadAgree;
				}
				Vector3 vector13 = new Vector3(pCmd, qT, rCmd) * (MathF.PI / 180f);
				vector11 += Plugin.CouplingFF * (Vector3.Cross(vector13, MulI(vector13)) - Vector3.Cross(omega, MulI(omega)));
				vector11 += asymFF;
				// 2.0.12: on the wheels the incremental (INDI) roll target is an integrator with nothing to stop it: the gear
				// holds the wings, the measured roll acceleration stays 0, and any roll moment the surfaces make (the mode
				// switch at the rotation speed left one) is kept for the whole roll. Take-off logs: stabs split +7 / -6 deg
				// from 46 m/s to lift-off. While a wheel touches, the surfaces' roll moment is the proportional roll law
				// alone (I x the wanted roll acceleration), with the airframe's own roll moment from the model.
				if (GroundRollP && wheelsTouch)
				{
					float restX = vector8.x;
					foreach (Effector eX in effs)
					{
						if (eX.active)
						{
							restX -= eX.MAt(eX.now).x;
						}
					}
					vector11.x = restX + MulI(new Vector3(omegaDotDes.x, 0f, 0f)).x;
				}
				// 2.0.12: with a rudder gone or an engine out the yaw target can sit far beyond what the jet has (10:29 log,
				// fin shot off: -50..-106 deg/s^2 asked, -12 available). Chasing it, the allocator bought small yaw gains with
				// the stabs and flaperons at any roll / pitch cost (one stab swapping +35 <-> -30 at 0.6 Hz, roll +18 <-> -68
				// with the stick held, pitch rate +-40). Here the yaw target is limited to what the rudders reach with every
				// other surface where it is now, so the rest of the allocation is not traded away for yaw it cannot get.
				decelDbg = 0f;
				yawClampStabDbg = 0f;
				alloc.fightW[1] = (alloc.fightW[2] = (alloc.fightW[3] = 1f));
				if (YawTargetClamp && (yawHealth < 0.999f || engOut || engAsym))
				{
					bool hasRudders = false;
					foreach (SurfaceEffector sR in surfaces)
					{
						// Revoker 0.2.7: a live rudder only. With the fin shot off the rudder's effector stays in the list
						// (inactive), so the canards never joined the first yaw tier, and once the nozzle's yaw was gone too (TVC
						// off, or past its 250 m/s cut-off) the yaw target was clamped to what nothing reached: 28 Oct 21:22 log,
						// fins and wings gone, 247 m/s, the nozzles cut out at 211.0 s and the yaw diverged (r -0.8 -> -128 deg/s
						// in 2 s, sideslip to 85 deg) with both canards held at +1..2 deg.
						if (sR.kind == SurfaceKind.Rudder && (sR.active || !RudderTierLiveOnly))
						{
							hasRudders = true;
							break;
						}
					}
					float restZ = vector8.z;
					float rudLo = 0f;
					float rudHi = 0f;
					float decelLo = 0f;
					float decelHi = 0f;
					float splitLo = 0f;
					float splitHi = 0f;
					foreach (Effector eY in effs)
					{
						if (!eY.active)
						{
							continue;
						}
						SurfaceEffector sY2 = eY as SurfaceEffector;
						float zNow = eY.MAt(eY.now).z;
						restZ -= zNow;
						if ((eY is SurfaceEffector sF && (sF.kind == SurfaceKind.Aileron || sF.kind == SurfaceKind.Flaperon)) || eY is HingeFlapEffector)
						{
							float fl = float.MaxValue;
							float fh = float.MinValue;
							for (int iF = 0; iF < eY.N; iF++)
							{
								fl = Mathf.Min(fl, eY.M[iF].z);
								fh = Mathf.Max(fh, eY.M[iF].z);
							}
							decelLo += Mathf.Min(0f, fl - zNow);
							decelHi += Mathf.Max(0f, fh - zNow);
						}
						// Ifrit: no rudders. The yaw sources are the canted tails and the split decelerons, so the clamp is to
						// what they reach (with the rest where it is now); without this the yaw target froze at the moment
						// the engine quit (engine-out pull at 300 km/h: sideslip 36-65 deg, splits idle).
						// Ifrit 0.4.0 (SplitYawTier): the splits are a tier of their own, after the tails (see below).
						if (!hasRudders && eY is SplitEffector && Plugin.SplitYawTier)
						{
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iY = 0; iY < eY.N; iY++)
							{
								zl = Mathf.Min(zl, eY.M[iY].z);
								zh = Mathf.Max(zh, eY.M[iY].z);
							}
							splitLo += Mathf.Min(0f, zl);
							splitHi += Mathf.Max(0f, zh);
						}
						// Revoker 0.1.0: the nozzle's yaw axis is in the first tier with the rudder (least drag: it costs only the
						// thrust's cosine), so the split flaperons come in only past what rudder + yaw TVC reach
						else if ((eY is SurfaceEffector sY && (sY.kind == SurfaceKind.Rudder || (!hasRudders && sY.kind == SurfaceKind.Stabilator))) || (!hasRudders && eY is SplitEffector) || eY is TvcYawEffector || (sY2 != null && sY2.kind == SurfaceKind.Stabilator && Plugin.CanardYawTier))
						{
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iY = 0; iY < eY.N; iY++)
							{
								zl = Mathf.Min(zl, eY.M[iY].z);
								zh = Mathf.Max(zh, eY.M[iY].z);
							}
							rudLo += zl;
							rudHi += zh;
						}
						else if (YawClampStabReach && eY is SurfaceEffector sS && sS.kind == SurfaceKind.Stabilator && stabPitchFrac < YawClampStabFree)
						{
							// 2.20.0: where the stabs are off the pitch (nozzles carry it, stabPitchFrac -> 0: ~60 deg and up) their
							// differential IS the yaw axis without fins - the reach includes their yaw range, scaled by how free
							// they are. 00:25 log (2.19.0, no fins): the clamp held the yaw target at the flaperons' drag-yaw (+8..+20
							// deg/s^2) against a ~100 demand for 4 s per right-going reversal, stabs idle at -7/+2 with 22-27 deg/s^2
							// of yaw in their tables; the left-going ones were quick only because the roll loop's stab differential
							// happened to yaw the right way. On the pitch (2.0.12's one-fin mid-AoA case) the reach is unchanged.
							float zl = float.MaxValue;
							float zh = float.MinValue;
							for (int iS = 0; iS < eY.N; iS++)
							{
								zl = Mathf.Min(zl, eY.M[iS].z);
								zh = Mathf.Max(zh, eY.M[iS].z);
							}
							float wFree = Mathf.Clamp01((YawClampStabFree - stabPitchFrac) / YawClampStabFree);
							rudLo += zNow + wFree * Mathf.Min(0f, zl - zNow);
							rudHi += zNow + wFree * Mathf.Max(0f, zh - zNow);
							yawClampStabDbg = wFree;
						}
						else
						{
							rudLo += zNow;
							rudHi += zNow;
						}
					}
					// Ifrit 0.4.0 (SplitYawTier, dev 3. Lateral-directional): least drag first. With an engine out (or a damaged
					// yaw axis) the canted tails carry the yaw as long as they can, the split decelerons take the rest, the
					// flaperon crossing (decelF) and the VMC throttle limit come after. 22 Oct 00:35 log, right engine alone at
					// AB, 140 m/s, 3 deg AoA: the left split sat at 15-24 deg for 30 s while the tails held +-2 deg of difference
					// (the split was the allocator's cheapest pure yaw); at the 30 deg cap, 50-85 m/s, the inner/outer flaperons
					// crossed at +-30 with the splits shut (the deflection had taken the split's travel) and the live engine was
					// cut to 30-40 %.
					if (!hasRudders && Plugin.SplitYawTier)
					{
						// the tails' yaw comes with roll (they are canted), which the wing must take back: that is the
						// roll-fight cost's "stabs against the wing" case, so the stab group's fight weight is relaxed by
						// StabYawRelax here. Without it the allocator leaves the yaw to sideslip rather than pay the fight
						// (bench, right engine alone at 500 km/h: 2.5 deg of sideslip held with the tails at +-0.1 deg).
						alloc.fightW[1] = 1f - Mathf.Clamp01(Plugin.StabYawRelax);
						// The split gate opens on measured yaw saturation (the yaw target out of reach with the splits as bounded
						// now, after the tails and the flaperon crossing the allocator finds on its own) and leaks shut slowly
						// (SplitTierFall) once the yaw is met, so it settles on the least split that keeps the yaw met. Bench, yaw
						// per kN of added drag at 250 km/h, 20 deg AoA: tails +-10 deg 4.8 deg/s^2 per kN, inner/outer crossing
						// 1.1-1.5, one split at 30 deg 0.4 (17.7 kN for 7.8 deg/s^2), the VMC throttle cut 0.17 (thrust given up).
						float stYawNow = 0f;
						foreach (SurfaceEffector sS in surfaces)
						{
							if (sS.kind == SurfaceKind.Stabilator && sS.active)
							{
								stYawNow += sS.MAt(sS.now).z;
							}
						}
						float sTgt = Mathf.Clamp01((yawSat - SplitYawSatOpen) / SplitYawSatSpan);
						splitTierF = Mathf.MoveTowards(splitTierF, sTgt, ((sTgt > splitTierF) ? SplitTierRise : SplitTierFall) * num);
						rudLo += splitTierF * splitLo;
						rudHi += splitTierF * splitHi;
						float demSign = Mathf.Sign(stYawNow);
						foreach (SplitEffector sT in splits)
						{
							if (!sT.active || sT.brakeMode)
							{
								continue;
							}
							// gate on the split's travel; with the gate open the split has priority over the host deceleron's
							// deflection for their shared travel (the reverse of the normal DeceleronSplitBudget rule), so the
							// allocator can always open it: the crossing had taken the deceleron's travel first and left the split
							// nothing (22 Oct 00:35 log, 30 deg cap)
							float gateHi = Mathf.Min(sT.max, Mathf.Max(sT.now - sT.rate * num, splitTierF * sT.max));
							sT.hi = Mathf.Max(sT.lo, Mathf.Min(Mathf.Max(sT.hi, gateHi), gateHi));
							sT.cmd = Mathf.Clamp(sT.cmd, sT.lo, sT.hi);
							float sReachSide = (demSign > 0f) ? sT.M[sT.N - 1].z : (0f - sT.M[sT.N - 1].z);
							if (sT.host != null && sT.host.active && splitTierF > 0f && sReachSide > 0f)
							{
								float travel = Mathf.Max(sT.host.max, 0f - sT.host.min);
								float keep = Mathf.Max(0f, travel - Mathf.Max(sT.cmdOut, sT.now));
								sT.host.hi = Mathf.Max(sT.host.lo, Mathf.Min(sT.host.hi, keep));
								sT.host.lo = Mathf.Min(sT.host.hi, Mathf.Max(sT.host.lo, 0f - keep));
								sT.host.cmd = Mathf.Clamp(sT.host.cmd, sT.host.lo, sT.host.hi);
							}
						}
						tierDemDbg = yawSat; tierReachDbg = stYawNow * IaInv[2, 2] * 57.29578f; tierSplitDbg = (splitHi - splitLo) * IaInv[2, 2] * 57.29578f;
					}
					else
					{
						splitTierF = 0f;
					}
					splitTierDbg = splitTierF;
					float zMargin = YawTargetMargin * (rudHi - rudLo);
					float zLo = restZ + rudLo - zMargin;
					float zHi = restZ + rudHi + zMargin;
					// 2.2.7 split flaperons as decelerons: when the yaw asked for is past what the rudders reach (both fins
					// gone, one fin gone, or the live engine's thrust more than the rudders hold), the reach is extended by
					// SplitFlaperonYawUse x the drag yaw the four flaperons have left. The allocator then buys it by opening the
					// flaperons on one wing against each other (inner and outer pair cancelling each other's roll, their drag
					// yawing the jet), with the roll-fight cost relaxed for the inner/outer split in proportion. Only the part
					// of the demand the rudders cannot cover; the VMC throttle limit (which acts on the remaining yaw
					// saturation) comes after it.
					// 2.2.8: lateral stick takes the flaperons back (the split's reach shrinks by SplitFlaperonRollYield x |lateral
					// stick|, held for 1 s after the stick comes back), the roll-fight cost is relaxed for the wing groups only
					// (the stabs still may not roll against the wing), and both follow a rate-limited gate (up 3/s, down 1.5/s)
					// instead of the frame's own saturation. 19 Oct log, both fins gone: the split held all four flaperons at
					// +-25 through rolls, the gate flicked 0.3 <-> 1 frame to frame and the fight cost with it; roll rate
					// swung 5 <-> 40 deg/s against a 35 command, stab difference -46..+3, nozzles chasing the pitch.
					zLoDbg = zLo; zHiDbg = zHi; zTgtDbg = vector11.z;
					float dTgt = 0f;
					decelYieldF = Mathf.Max(Mathf.Clamp01(Mathf.Abs(sr)), Mathf.MoveTowards(decelYieldF, 0f, DecelYieldFall * num));
					if (Plugin.SplitFlaperonYaw && (vector11.z < zLo || vector11.z > zHi))
					{
						float k = Plugin.SplitFlaperonYawUse * (1f - Plugin.SplitFlaperonRollYield * decelYieldF);
						float span = Mathf.Max(1E-06f, k * ((vector11.z < zLo) ? (0f - decelLo) : decelHi));
						float over = (vector11.z < zLo) ? (zLo - vector11.z) : (vector11.z - zHi);
						dTgt = Mathf.Clamp01(over / span);
					}
					decelF = Mathf.MoveTowards(decelF, dTgt, ((dTgt > decelF) ? DecelRise : DecelFall) * num);
					if (Plugin.SplitFlaperonYaw && decelF > 0f)
					{
						float k = Plugin.SplitFlaperonYawUse * (1f - Plugin.SplitFlaperonRollYield * decelYieldF);
						zLo += k * decelF * decelLo;
						zHi += k * decelF * decelHi;
						alloc.fightW[2] = (alloc.fightW[3] = 1f - DecelFightRelax * decelF);
						// 2.2.19 (SplitFlaperonStabRoll): the stabs may take up the wing's roll too, so the flaperons can be set for drag
						// yaw without having to cancel their own roll inside each wing
						alloc.fightW[1] = 1f - Mathf.Clamp01(DecelStabRelax) * decelF;
					}
					decelDbg = decelF;
					// 2.2.22: what the clamp takes away still counts as yaw saturation (the VMC throttle limit watches it). With
					// both fins gone the clamped target was always "reached", yawSat stayed 0 and the limiter never cut the
					// live engine (bench, one engine out + fins gone: departures at 250-600 km/h with the throttle untouched).
					yawClampExcess = Mathf.Abs(vector11.z - Mathf.Clamp(vector11.z, zLo, zHi));
					vector11.z = Mathf.Clamp(vector11.z, zLo, zHi);
				}
				else
				{
					decelF = 0f;
					splitTierF = 0f;
					splitTierDbg = 0f;
				}
				// 2.0.6: with the nose wheel on the runway the measured pitch acceleration stays ~0 whatever the surfaces do,
				// so the incremental (INDI) pitch target would integrate to full nose-up deflection before the nose moves and
				// then have to be unwound at servo rate after it lifts (the over-rotation). While the nose wheel touches, the
				// nose-up pitch moment target may only grow at GroundRotationRamp (deg/s^2 per s), starting from what the
				// surfaces make now; the nose comes up when that moment unloads the leg, with little left over.
				if (noseContactLaw)
				{
					float rampM = Plugin.GroundRotationRamp * (MathF.PI / 180f) / Mathf.Max(1E-06f, IaInv[1, 1]) * num;
					if (!rotMomOn)
					{
						rotMomLim = vector8.y;
						rotMomOn = true;
					}
					rotMomLim = Mathf.Min(rotMomLim + rampM, Mathf.Max(rotMomLim, vector11.y));
					vector11.y = Mathf.Min(vector11.y, rotMomLim);
				}
				else
				{
					rotMomOn = false;
				}
				tvc.lam = Mathf.Lerp(2E-05f, Mathf.Max(2E-05f, Plugin.TvcMoveCost), stabPitchFrac);
				// 2.0.3: with little thrust the nozzles make almost no moment per degree, so the allocator swung them across
				// their travel chasing pitch errors they could not fix (idle at the AoA limit: +-10-13 deg sd). Their movement
				// costs more as the thrust they vector falls.
				tvc.lam *= 1f + TvcLowThrustLam * (1f - Mathf.Clamp01(tw / TvcLowThrustTw));
				// (with the nozzles' thrust near idle their moment per degree is small and the grid's coarse steps were the
				// only thing holding them still; the Newton step had them chase the hold: bench, idle release at 450 km/h,
				// q held +1 deg/s for 2 s)
				// Ifrit 0.4.9: with the pitch stick released only, where the dip is, so a held pull runs 0.4.8's allocation. (Held,
				// it flipped single bench runs between the two stab roll-split branches of the full-pull + roll approach to 65 deg;
				// over a 350-560 km/h sweep the slow branch came up 10 times against 0.4.8's 11, so not a trend either way.)
				Allocator.CoordNewton = (Plugin.AllocNozzleNewton && tw >= NewtonMinTw && (!NewtonReleasedOnly || Mathf.Abs(sp) < 0.18f)) ? 1 : 0;
				TvcBudgetStep(num, vector11);
				// 2.2.14 (TvcNeutralHold, Dev): while the pitch rate is held (pitch-rate command steady, no pitch error, no roll
				// budget in force), the nozzles are drawn back to neutral and the stabs take the steady pitch moment; the
				// pull toward neutral drops at once when the pitch-rate command moves or a pitch error opens, so the nozzles
				// still take the changes. It returns over TvcNeutralTau.
				tvcNeutralDbg = 0f;
				// Revoker 0.2.8: not while the TrailingEdgeBrakeConfig set moves (teBrakeMov): held at neutral the nozzle froze the
				// whole pitch allocation through the retraction (bench, 900 km/h: canards, elevons and nozzle still for 1.6 s with
				// the pitch target unmet, nz down to 0.33); the hold comes back once the set has settled
				if (Plugin.TvcNeutralHold > 0f && tvc.anchorK <= 0f && !prot && !isAI && Mode == FcsMode.AoAG && !teBrakeMov)
				{
					float steadyW = (1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / TvcNeutralQdot)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.y * 57.29578f - qT) - 1f) / 4f)) * stabPitchFrac;
					tvcNeutralW = (steadyW < tvcNeutralW) ? steadyW : Mathf.MoveTowards(tvcNeutralW, steadyW, num / Mathf.Max(0.05f, TvcNeutralTau));
					if (tvcNeutralW > 0.001f)
					{
						tvc.anchor = 0f;
						tvc.band = TvcNeutralBand;
						tvc.anchorK = Plugin.TvcNeutralHold * tvcNeutralW;
						tvcNeutralDbg = tvcNeutralW;
					}
				}
				else
				{
					tvcNeutralW = 0f;
				}
				// Ifrit 0.3.0 (TvcRollNeutralHold): the same for the differential (roll) TVC. With the lateral stick and pedals
				// centred, the roll on its command and the sideslip under 2 deg, the nozzles are drawn back to symmetric and the
				// wing surfaces carry the steady roll trim; any roll command, roll error or sideslip lets go of them at once. Seen
				// after lift-off (11:51 log): with the ground law off, the nozzles took the roll trim for a few seconds.
				if (tvcd != null)
				{
					tvcd.anchorK = 0f;
					if (Plugin.TvcRollNeutralHold && Plugin.TvcRollNeutralWeight > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
					{
						float rollSteady = (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.08f)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.x * 57.29578f - pCmd) - 2f) / 6f)) * (1f - Mathf.Clamp01((Mathf.Abs(beta) - 2f) / 2f));
						tvcRollNeutralW = (rollSteady < tvcRollNeutralW) ? rollSteady : Mathf.MoveTowards(tvcRollNeutralW, rollSteady, num / Mathf.Max(0.05f, TvcNeutralTau));
						if (tvcRollNeutralW > 0.001f)
						{
							tvcd.anchor = 0f;
							tvcd.band = TvcNeutralBand;
							tvcd.anchorK = Plugin.TvcRollNeutralWeight * tvcRollNeutralW;
						}
					}
					else
					{
						tvcRollNeutralW = 0f;
					}
				}
				// Revoker 0.1.0 (YawTvcNeutralHold): the same for the nozzle's yaw axis. With the lateral stick and pedals steady,
				// the roll on its command and the sideslip under 2 deg (a held turn), the nozzle is drawn back to neutral in yaw
				// and the rudder and canards carry the steady yaw; any lateral command, roll error or sideslip lets go of it at once.
				if (tvcy != null)
				{
					tvcy.anchorK = 0f;
					if (Plugin.YawTvcNeutralHold > 0f && !isAI && (Mode == FcsMode.AoAG || Mode == FcsMode.GearDownRate))
					{
						float yawSteady = (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr - srHeldY), Mathf.Abs(sy - syHeldY)) - 0.02f) / 0.08f)) * (1f - Mathf.Clamp01((Mathf.Abs(omega.x * 57.29578f - pCmd) - 2f) / 6f)) * (1f - Mathf.Clamp01((Mathf.Abs(beta) - 2f) / 2f)) * (1f - Mathf.Clamp01((Mathf.Max(Mathf.Abs(pCmd), Mathf.Abs(omega.x * 57.29578f)) - 5f) / 10f));
						// Revoker 0.2.1: only as far as the rudder has travel to take the steady yaw. With a steady yaw load beyond the
						// rudder (in-game log 28 Oct 18:57, right wingtip and a missile gone, ~-10 deg/s^2 at 60-100 m/s) the hold pulled
						// the nozzle to neutral, the rudder sat on its stop, the sideslip grew past 2 deg and let go, the nozzle took the
						// load back and the hold came in again: a 2.3 s cycle, sideslip 1-4.4 deg, rudder +-25. Fades out from 55 to 85 %
						// of the rudder's travel in the direction it is deflected; none with no rudder left.
						yawSteady *= RudderHoldGate();
						srHeldY = Mathf.MoveTowards(srHeldY, sr, num / 0.5f);
						syHeldY = Mathf.MoveTowards(syHeldY, sy, num / 0.5f);
						tvcYawNeutralW = (yawSteady < tvcYawNeutralW) ? yawSteady : Mathf.MoveTowards(tvcYawNeutralW, yawSteady, num / Mathf.Max(0.05f, TvcNeutralTau));
						if (tvcYawNeutralW > 0.001f)
						{
							tvcy.anchor = 0f;
							tvcy.band = TvcNeutralBand;
							tvcy.anchorK = Plugin.YawTvcNeutralHold * tvcYawNeutralW;
						}
					}
					else
					{
						tvcYawNeutralW = 0f;
					}
				}
				float num24 = Mathf.Clamp01(Mathf.Abs(pCmd) / 40f) * stabPitchFrac * (1f - Mathf.InverseLerp(12f, 20f, Mathf.Abs(alpha))) * (1f - Mathf.Clamp01(Mathf.Abs(sp) / 0.15f)) * (1f - Mathf.Clamp01(Mathf.Abs(qT) / 8f));
				float num25 = ((WashK >= 0f) ? WashK : Plugin.RollNozzleHold);
				if (num24 < 0.05f)
				{
					tvcTrimRef += (tvc.cmdOut - tvcTrimRef) * (num / (0.3f + num));
				}
				tvc.lam = Mathf.Max(tvc.lam, 0.05f * Mathf.Clamp01(num25) * num24 * Mathf.InverseLerp(125f, 150f, ias));
				if (num25 > 0f && num24 > 0.02f)
				{
					for (int num26 = 0; num26 < effs.Count; num26++)
					{
						cmdSave[num26] = effs[num26].cmd;
					}
					alloc.Solve(effs, vector11);
					float b = ((WashTau >= 0f) ? WashTau : Plugin.RollNozzleWashout);
					tvcSlow += (tvc.cmd - tvcTrimRef - tvcSlow) * (num / (Mathf.Max(0.02f, b) + num));
					float num27 = 0f;
					foreach (SurfaceEffector surface5 in surfaces)
					{
						if (surface5.kind == SurfaceKind.Stabilator && surface5.active)
						{
							num27 = Mathf.Max(num27, Mathf.Abs(surface5.cmd) / Mathf.Max(1f, surface5.LimitFor(surface5.cmd)));
						}
					}
					float num28 = Mathf.Clamp01((WashUtilHi - num27) / 0.2f);
					float washTarget = Mathf.Clamp01(num25) * num24 * num28 * tvcSlow;
					washApplied = ((Plugin.RollNozzleWashoutSlew > 0f) ? Mathf.MoveTowards(washApplied, washTarget, Plugin.RollNozzleWashoutSlew * num) : washTarget);
					float hi = Mathf.Clamp(tvc.cmd - washApplied, tvc.lo, tvc.hi);
					for (int num29 = 0; num29 < effs.Count; num29++)
					{
						effs[num29].cmd = cmdSave[num29];
					}
					float lo = tvc.lo;
					float hi2 = tvc.hi;
					tvc.lo = (tvc.hi = hi);
					alloc.Solve(effs, vector11);
					tvc.lo = lo;
					tvc.hi = hi2;
					tvcPin = hi;
					tvcPinned = true;
				}
				else
				{
					tvcSlow += (0f - tvcSlow) * (num / (0.2f + num));
					washApplied = ((Plugin.RollNozzleWashoutSlew > 0f) ? Mathf.MoveTowards(washApplied, 0f, Plugin.RollNozzleWashoutSlew * num) : 0f);
					tvcPinned = false;
					alloc.Solve(effs, vector11);
				}
				stabBranch = 0;
				bool branchGate = Plugin.StabBranchSwitch && Plugin.StabBranchMaxAoA > 0f;
				if (branchGate)
				{
					float aAbs = Mathf.Abs(alpha);
					branchGateOn = branchGateOn ? (aAbs <= Plugin.StabBranchMaxAoA + 2f) : (aAbs <= Plugin.StabBranchMaxAoA);
					branchGate = branchGateOn;
				}
				foreach (SurfaceEffector sb in surfaces)
				{
					if (sb.kind != SurfaceKind.Stabilator)
					{
						continue;
					}
					sb.branchCool = Mathf.Max(0f, sb.branchCool - num);
					if (Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f)
					{
						sb.branchCool = Mathf.Min(sb.branchCool, BranchCoolTimeStop);
					}
					if (!branchGate || !sb.active)
					{
						sb.branchOn = false;
						continue;
					}
					if (!sb.branchOn && sb.branchCool > 0f)
					{
						continue;
					}
					// 2.0.1: "deep past the peak" and the target are scaled to the stab's own peak (10 and 20 deg were set
					// for Aryx's 33.75 deg foil), and the attached-side target is searched over the stab's whole travel, not
					// this tick's rate-limited window: from -30 deg that window never reached the attached side, so the
					// switch never fired.
					// (with lateral input only: without it the 2.0.0 behaviour is kept)
					bool latIn = Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) >= 0.1f;
					float peakK = (BranchScaleToPeak && latIn) ? Mathf.Clamp(sb.stallAoA / StabPeakRef, 0.3f, 1.5f) : 1f;
					if (Mathf.Abs(sb.LocalAoAAt(sb.cmd)) < sb.stallAoA + (latIn ? BranchDeepMargin : 10f) * peakK)
					{
						continue;
					}
					float bestD = sb.cmd;
					float bestE = float.MaxValue;
					float gLo = (BranchFullTravel && latIn) ? sb.min : sb.lo;
					float gHi = (BranchFullTravel && latIn) ? sb.max : sb.hi;
					for (int gi = 0; gi < sb.N; gi++)
					{
						float gd = sb.grid[gi];
						if (gd < gLo - 0.0001f || gd > gHi + 0.0001f)
						{
							continue;
						}
						float la = Mathf.Abs(sb.LocalAoAAt(gd));
						if (la >= sb.stallAoA)
						{
							continue;
						}
						float e = Mathf.Abs(la - Plugin.StabBranchTarget * peakK);
						if (e < bestE)
						{
							bestE = e;
							bestD = gd;
						}
					}
					if (bestE == float.MaxValue)
					{
						sb.branchOn = false;
						continue;
					}
					float y0 = 0f;
					for (int ci = 0; ci < effs.Count; ci++)
					{
						cmdSave[ci] = effs[ci].cmd;
						if (effs[ci].active)
						{
							y0 += effs[ci].MAt(effs[ci].cmd).y;
						}
					}
					float slo = sb.lo;
					float shi = sb.hi;
					// the stab is walked there through this tick's reachable window, so the allocator plans the nozzles and
					// the other surfaces around where it actually is on the way across its lift peak
					float pinD = (BranchFullTravel && latIn) ? Mathf.Clamp(bestD, sb.lo, sb.hi) : bestD;
					sb.lo = (sb.hi = (sb.cmd = pinD));
					// 2.0.1: the nozzles are free in the branch solve. With the roll-time nozzle budget on, they could not take the
					// stab's nose-down, the pitch check failed every time and the stab stayed parked deep in stall until the AoA
					// was low (in game: right stab at -30 from 29 to 3 deg AoA with full roll, then a roll-rate jump).
					float akSave = tvc.anchorK;
					if (BranchFreeTvc)
					{
						tvc.anchorK = 0f;
					}
					alloc.Solve(effs, vector11);
					tvc.anchorK = akSave;
					sb.lo = slo;
					sb.hi = shi;
					float y1 = 0f;
					for (int ci2 = 0; ci2 < effs.Count; ci2++)
					{
						if (effs[ci2].active)
						{
							y1 += effs[ci2].MAt(effs[ci2].cmd).y;
						}
					}
					float kP = IaInv[1, 1] * 57.29578f;
					bool pitchOk = Mathf.Abs(y1 - vector11.y) * kP <= Mathf.Abs(y0 - vector11.y) * kP + Plugin.StabBranchPitchTol;
					// 2.0.4: with lateral input, nozzles at the end of their travel no longer block the switch when it does not
					// make the modelled pitch error worse: at idle they sit pinned holding against the parked stab, so the test
					// failed every time and the stab stayed in stall (in game, 156-161 s: right stab -30, nozzles +20, AoA 36 -> 0).
					bool tvcRoom = tvc == null || !tvc.active || Mathf.Abs(tvc.cmd) <= 0.8f * Mathf.Max(1f, tvc.max) || (latIn && BranchFreeTvc && Mathf.Abs(y1 - vector11.y) <= Mathf.Abs(y0 - vector11.y));
					if (sb.branchOn)
					{
						sb.branchFail = (pitchOk ? 0f : (sb.branchFail + num));
						if (sb.branchFail > BranchFailTime)
						{
							for (int ci3 = 0; ci3 < effs.Count; ci3++)
							{
								effs[ci3].cmd = cmdSave[ci3];
							}
							sb.branchOn = false;
							sb.branchCool = ((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f) ? BranchCoolTimeStop : BranchCoolTime);
						}
						else
						{
							stabBranch++;
						}
					}
					else if (pitchOk && tvcRoom)
					{
						sb.branchOn = true;
						sb.branchFail = 0f;
						stabBranch++;
					}
					else
					{
						for (int ci4 = 0; ci4 < effs.Count; ci4++)
						{
							effs[ci4].cmd = cmdSave[ci4];
						}
						sb.branchCool = ((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.1f) ? BranchCoolTimeStop : BranchCoolTime);
					}
				}
				rollPitchFF = 0f;
				float num30 = Plugin.RollPitchFF * (1f - Mathf.Clamp01((Mathf.Abs(alpha) - (Plugin.RollPitchFFAoA - 15f)) / 15f));
				if (num30 > 0.01f)
				{
					foreach (SurfaceEffector surface6 in surfaces)
					{
						if (surface6.active && surface6.noPitch)
						{
							float c = Mathf.MoveTowards(surface6.now, surface6.cmd, Mathf.Max(1f, surface6.rate) * num);
							rollPitchFF += surface6.PitchRawAt(c) - surface6.PitchRawAt(surface6.now);
						}
					}
					rollPitchFF *= num30;
					float num31 = Plugin.RollPitchFFMax / Mathf.Max(IaInv[1, 1] * 57.29578f, 1E-06f);
					rollPitchFF = Mathf.Clamp(rollPitchFF, 0f - num31, num31);
					if (Mathf.Abs(rollPitchFF) * IaInv[1, 1] * 57.29578f > 0.5f)
					{
						float lam = tvc.lam;
						int num32 = 0;
						if (Plugin.RollPitchFFStabCost > 1.01f)
						{
							tvc.lam = 2E-05f;
							foreach (SurfaceEffector surface7 in surfaces)
							{
								if (surface7.kind == SurfaceKind.Stabilator && surface7.partner != null && num32 < symSave.Length)
								{
									symSave[num32] = surface7.symCost;
									surface7.symCost *= Plugin.RollPitchFFStabCost;
									num32++;
								}
							}
						}
						float lo2 = tvc.lo;
						float hi3 = tvc.hi;
						if (tvcPinned)
						{
							tvc.lo = (tvc.hi = tvcPin);
						}
						int num33 = 0;
						foreach (SurfaceEffector surface8 in surfaces)
						{
							if (surface8.active && surface8.noPitch && num33 < pinLo.Length)
							{
								pinLo[num33] = surface8.lo;
								pinHi[num33] = surface8.hi;
								surface8.lo = (surface8.hi = Mathf.Clamp(surface8.cmd, surface8.lo, surface8.hi));
								num33++;
							}
						}
						alloc.Solve(effs, vector11 - new Vector3(0f, rollPitchFF, 0f));
						num33 = 0;
						foreach (SurfaceEffector surface9 in surfaces)
						{
							if (surface9.active && surface9.noPitch && num33 < pinLo.Length)
							{
								surface9.lo = pinLo[num33];
								surface9.hi = pinHi[num33];
								num33++;
							}
						}
						tvc.lo = lo2;
						tvc.hi = hi3;
						tvc.lam = lam;
						if (Plugin.RollPitchFFStabCost > 1.01f)
						{
							num32 = 0;
							foreach (SurfaceEffector surface10 in surfaces)
							{
								if (surface10.kind == SurfaceKind.Stabilator && surface10.partner != null && num32 < symSave.Length)
								{
									surface10.symCost = symSave[num32];
									num32++;
								}
							}
						}
					}
				}
				RollPitchPass(vector11, num);
				Vector3 zero4 = Vector3.zero;
				dbgYawRud = (dbgYawWing = (dbgYawStab = (dbgYawTvc = 0f)));
				foreach (Effector eff3 in effs)
				{
					if (!eff3.active)
					{
						continue;
					}
					Vector3 vector14 = eff3.MAt(eff3.cmd);
					zero4 += vector14;
					if (DebugYaw)
					{
						float num34 = vector14.z * IaInv[2, 2] * 57.29578f;
						if (eff3.name.Contains("Rudder"))
						{
							dbgYawRud += num34;
						}
						else if (eff3.name.Contains("Flap") || eff3.name.Contains("Aileron"))
						{
							dbgYawWing += num34;
						}
						else if (eff3.name.Contains("Elevator"))
						{
							dbgYawStab += num34;
						}
						else
						{
							dbgYawTvc += num34;
						}
					}
				}
				dbgTargetZ = vector11.z * IaInv[2, 2] * 57.29578f;
				dbgMfZ = vector8.z * IaInv[2, 2] * 57.29578f;
				dbgTgtY = vector11.y * IaInv[1, 1] * 57.29578f;
				dbgMfY = vector8.y * IaInv[1, 1] * 57.29578f;
				float num35 = Mathf.Max((vector11.z - zero4.z) * Mathf.Sign(vector11.z - vector8.z), YawSatCountClamp ? yawClampExcess : 0f);
				yawClampExcess = 0f;
				yawSat = yawSatF.Step(Mathf.Clamp01((num35 * IaInv[2, 2] * 57.29578f - 10f) / 40f), 0.25f, num);
				// 2.7.0: the allocator's own unmet report per axis, signed with the increment it was asked for (deg/s^2): what the
				// effectors at their commands fall short of the target by, toward the side the demand went. Read by the measured
				// pace window as its saturation gate (with the slew still in flight), no surface list, no filter beyond PaceSatTau.
				Vector3 unmetRaw = new Vector3(Mathf.Max(0f, (vector11.x - zero4.x) * Mathf.Sign(vector11.x - vector8.x)) * IaInv[0, 0] * 57.29578f * Mathf.Sign(vector11.x - vector8.x), Mathf.Max(0f, (vector11.y - zero4.y) * Mathf.Sign(vector11.y - vector8.y)) * IaInv[1, 1] * 57.29578f * Mathf.Sign(vector11.y - vector8.y), Mathf.Max(0f, num35) * IaInv[2, 2] * 57.29578f * Mathf.Sign(vector11.z - vector8.z));
				allocUnmet = allocUnmetF.Step(unmetRaw, PaceSatTau, num);
				// the stock-jet capacity estimator (every effector grouped, sideslip-axis read), from this tick's commands (restores them).
				// It drives the governor from this version on, so it runs every tick, logging or not. The last argument is the F-22E's
				// rudder-fade weight, kept for the telemetry; the estimator's own rudW comes from its probes.
				// Revoker 0.8.2: what the allocation left unmade of the INDI target (deg/s^2), before the estimator's probes re-solve
				allocResMain = alloc.Accel(alloc.resDbg);
				capEst.Step(effs, alloc, vector7, IaInv, zero4, num, unmetRaw, alpha, Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f));
				float num36 = 10f;
				float num37 = 1f;
				if (tvc.engines.Count > 0 && tvc.engines[0] != null)
				{
					num36 = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(tvc.engines[0]).x));
					float x4 = Acc.TF_vecGain(tvc.engines[0]).x;
					num37 = ((Mathf.Abs(x4) > 0.001f) ? x4 : 1f);
				}
				tvcPitchInput = Mathf.Clamp((0f - tvc.cmdOut) / (num36 * num37), -1f, 1f);
			}
			mTarget = vector11;
			float actuatorDamping = Plugin.ActuatorDamping;
			foreach (Effector eff4 in effs)
			{
				if (Mode == FcsMode.Ground || actuatorDamping <= 0.0001f)
				{
					eff4.cmdOut = eff4.cmd;
					eff4.cmdLag = eff4.cmd;
					eff4.outTraj.Reset(eff4.cmd);
				}
				else if (LegacyLag)
				{
					eff4.cmdOut += (eff4.cmd - eff4.cmdOut) * (num / (actuatorDamping + num));
					eff4.outTraj.Reset(eff4.cmdOut);
				}
				else
				{
					float num38 = Mathf.Max(1f, eff4.rate);
					eff4.cmdLag += (eff4.cmd - eff4.cmdLag) * (num / (PreLag + num));
					eff4.outTraj.Step(eff4.cmdLag, num38, num38 / actuatorDamping, num);
					eff4.cmdOut = eff4.outTraj.x;
				}
			}
			tvcRollInput = (tvcd != null && tvcd.active && Mode != FcsMode.Ground) ? tvcd.RollInput(tvcd.cmdOut) : 0f;
			tvcYawInput = (tvcy != null && tvcy.active && Mode != FcsMode.Ground) ? tvcy.YawInput(tvcy.cmdOut) : 0f;
			foreach (HingeFlapEffector hD in hinges)
			{
				if (hD.Valid)
				{
					hD.Drive(hD.active ? hD.cmdOut : hD.symTarget, num);
				}
			}
			if (Mode == FcsMode.Ground)
			{
				float num39 = 10f;
				float num40 = 1f;
				if (tvc.engines.Count > 0 && tvc.engines[0] != null)
				{
					num39 = Mathf.Max(0.1f, Mathf.Abs(Acc.TF_vec(tvc.engines[0]).x));
					float x5 = Acc.TF_vecGain(tvc.engines[0]).x;
					num40 = ((Mathf.Abs(x5) > 0.001f) ? x5 : 1f);
				}
				float num41 = groundBrake * Plugin.GroundBrakeTvc / (num39 * num40);
				tvcPitchInput = Mathf.Clamp(0f - sp - num41, -1f, 1f);
			}
			foreach (SurfaceEffector surface11 in surfaces)
			{
				if (surface11.active)
				{
					surface11.localAoACmd = surface11.LocalAoAAt(surface11.cmd);
				}
			}
			Engaged = true;
			if (isAI && Plugin.AIFlightLog)
			{
				AILogStep(num);
			}
			if (!telemetryOpened && Plugin.TelemetryEnabled && !isAI)
			{
				telemetryOpened = true;
				telemetry = new Telemetry(this);
			}
			telemetry?.Write(time, sp, sr, sy, num2);
		}

		// Ifrit 0.3.1 (FullStickPitchRate): the pitch-rate cap rises from MaxPitchRate to FullStickPitchRate over the last 20 %
		// of aft stick, so full aft pulls faster while every stick position up to 0.8 (and the push side) maps as before
		public float PitchRateCap
		{
			get
			{
				float m = Plugin.MaxPitchRate;
				if (Plugin.FullStickPitchRate > m && sp > 0f && Mode != FcsMode.MPO)
				{
					m = Mathf.Lerp(m, Plugin.FullStickPitchRate, Mathf.InverseLerp(0.8f, 1f, sp));
				}
				return m;
			}
		}

		private float RateBudget()
		{
			float cap = PitchRateCap;
			float num = Mathf.Clamp(5f + 0.15f * ias + 40f * tw, 5f, cap);
			float b = Mathf.Clamp((ias - 22f) * 1.1f, 0f, cap);
			float t = Mathf.Clamp01((Mathf.Abs(alpha) - (Plugin.CoupleAoA - 6f)) / 12f);
			return Mathf.Clamp(Mathf.Lerp(Mathf.Max(num, b), num, t), 6f, cap);
		}

		private float AIStickRate()
		{
			float num = Mathf.Max(1f, aiCornerSpeed);
			float num2 = tas * tas * Mathf.Max(0.01f, ac.airDensity) / (num * num * 1.225f);
			float num3 = aiGLimit * 9.81f / Mathf.Max(tas, 0.75f * num) * 57.29578f;
			if (num2 < 1f)
			{
				num3 *= Mathf.Clamp(num2, 0.3f, 1f);
			}
			return num3;
		}

		// 1.8.40: roll-pitch priority. While a roll is commanded (and for RpRelease s after), the flaperons' pitch moment
		// is in the allocator's model instead of being fed forward: the roll's pitch side-effect is then balanced in the
		// same solve that picks the roll deflections, with pitch weighted ~8-15x roll, so the solver takes it off the roll
		// (less flaperon/stab differential) before it pushes the stabs past their lift peak or the nozzles off their trim.
		// The flaperon pair's collective costs RpSymCost x more meanwhile, so they are not used as pitch surfaces.
		internal static float RpEnable = 1f;

		internal static float RpSymCost = 50f;

		internal static float RpRelease = 0.5f;

		internal static float RpAoA0 = 18f;

		internal static float RpAoA1 = 26f;

		private float rpLat;

		public float rpGate;

		internal static float RpPitchW = 10f;

		internal static float RpPitchTol = 4f;

		internal static float RpYawW = 10f;

		internal static float RpErrBand = 0f;

		internal static float RpHpTau = 0.4f;

		internal static float RpHpK = 0f;

		private float rpTvcLp;

		private bool rpLpInit;

		internal static float RpErr0 = 1.5f;

		internal static float RpBandMin = 0.5f;

		public float rpBand = -1f;

		public float rpPitchErr;

		// After the normal allocation: if the nozzles were sent more than RollTvcBudget off their pre-roll trim while a
		// roll is commanded, solve again with the nozzles held inside that band, every surface's true pitch curve
		// (flaperon pitch moment in, stab lift past its peak honest) and pitch weighted RpPitchW x more. The roll is what
		// gives. Accepted only if its modelled pitch error stays within RpPitchTol deg/s^2 of the normal solution's;
		// otherwise the band is widened (x3, x8) and finally the normal solution kept.
		private void RollPitchPass(Vector3 target, float dt)
		{
			rpBand = -1f;
			rpPitchErr = 0f;
			if (tvc == null || !tvc.active)
			{
				return;
			}
			// the band is centred on the pre-roll trim plus the fast (high-passed) part of what the normal solution wants
			// from the nozzles: they keep doing the quick pitch corrections the surfaces cannot follow (reversals, actuator
			// lag) and hand the steady part of the roll's pitch moment to the stabs within RpHpTau.
			float dtR = Mathf.Max(0.001f, dt);
			float tvcMain = tvc.cmd;
			if (!rpLpInit || rpGate <= 0.01f)
			{
				rpTvcLp = tvcMain;
				rpLpInit = true;
			}
			else
			{
				rpTvcLp += (tvcMain - rpTvcLp) * (dtR / (Mathf.Max(0.01f, RpHpTau) + dtR));
			}
			if (rpGate <= 0.01f)
			{
				return;
			}
			float center = tvcAnchor + RpHpK * (tvcMain - rpTvcLp);
			// the band opens with the pitch-rate error: the nozzles stay the fast pitch effector for transients the
			// surfaces cannot follow (roll reversals: rate limits and actuator lag), only the steady roll trim is held off them
			float qErrNow = Mathf.Abs(omega.y * 57.29578f - qT);
			float band0 = Mathf.Max(RpBandMin, Plugin.RollTvcBudget) + (1f - rpGate) * (1f - rpGate) * Mathf.Max(tvc.max, 0f - tvc.min) * RpGateSpan + RpErrBand * Mathf.Max(0f, qErrNow - RpErr0);
			// with afterburner-class thrust the nozzles are the stronger and faster pitch effector: holding them off the
			// trim moves the stabs' reserve for reversal transients into the trim (bench: asym-load AB roll spam overshot
			// the AoA cap by 3-4 deg more). The band widens by RpTwBand between T/W 0.7 and 1.1.
			band0 += RpTwBand * Mathf.Clamp01((tw - TvcBudgetTw0) / Mathf.Max(0.05f, TvcBudgetTwFade));
			// roll stop / reversal (stick against the roll rate): the roll's pitch moment is about to swing the other way
			// faster than the nozzles slew, so they are let go for it
			float pNow = omega.x * 57.29578f;
			float rev = Mathf.Clamp01(-Mathf.Sign(pNow) * sr * 2f) * Mathf.Clamp01((Mathf.Abs(pNow) - 20f) / 40f);
			band0 += RpRevBand * rev;
			rpRevNow = Mathf.Max(rev, rpRevNow - dtR / Mathf.Max(0.05f, RpRevHold));
			// AoA protection / overshoot past the AoA (or asymmetric-load) limit: the nozzles are free again
			band0 += Mathf.Max(prot ? 1f : 0f, pitchNeed) * Mathf.Max(tvc.max, 0f - tvc.min) * 2f;
			float dev = tvc.cmd - center;
			if (Mathf.Abs(dev) <= band0 + 0.05f)
			{
				return;
			}
			int n = effs.Count;
			if (cmdSave.Length < n)
			{
				cmdSave = new float[n * 2];
			}
			for (int i = 0; i < n; i++)
			{
				cmdSave[i] = effs[i].cmd;
			}
			float kP = IaInv[1, 1] * 57.29578f;
			int ns = 0;
			int nl = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				// stabs stay on the attached side of their lift peak in this pass: past it, their pitch moment turns with the
				// roll (the pitch-up this pass exists to avoid) and lags the plan by a stall's worth of travel
				if (RpStallLimit && se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
				{
					pinLo[nl] = se.lo;
					pinHi[nl] = se.hi;
					nl++;
					StallWindow(se, se.stallAoA + RpStallMargin);
					// and keep a reserve short of the travel stops for the pitch transients to come (a roll reversal
					// swings the roll's pitch moment the other way faster than the nozzles slew)
					float rlo = se.min + RpStabReserve;
					float rhi = se.max - RpStabReserve;
					if (se.lo < rlo)
					{
						se.lo = Mathf.Min(rlo, se.hi);
					}
					if (se.hi > rhi)
					{
						se.hi = Mathf.Max(rhi, se.lo);
					}
				}
				se.HonestPitchOn();
				if (se.noPitch && se.partner != null && se.kind != SurfaceKind.Rudder && ns < symSave.Length)
				{
					symSave[ns++] = se.symCost;
					se.symCost *= 1f + RpSymCost;
				}
			}
			Vector3 w0 = alloc.axisWeight;
			alloc.axisWeight = new Vector3(w0.x, w0.y * RpPitchW, w0.z * RpYawW);
			float lo0 = tvc.lo;
			float hi0 = tvc.hi;
			// reference: the same solve with the nozzles free - the pitch error the surfaces cannot avoid anyway
			alloc.Solve(effs, target);
			float eA = Mathf.Abs(PitchSum() - target.y) * kP;
			bool ok = false;
			float e = 0f;
			for (int k = 0; k < 3 && !ok; k++)
			{
				float band = band0 * ((k == 0) ? 1f : ((k == 1) ? 3f : 8f));
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
				float lo = Mathf.Max(lo0, center - band);
				float hi = Mathf.Min(hi0, center + band);
				if (lo > hi)
				{
					lo = (hi = ((center - band > hi0) ? hi0 : lo0));
				}
				tvc.lo = lo;
				tvc.hi = hi;
				alloc.Solve(effs, target);
				e = Mathf.Abs(PitchSum() - target.y) * kP;
				if (e <= eA + RpPitchTol + RpRevTol * rpRevNow)
				{
					ok = true;
					rpBand = band;
				}
			}
			tvc.lo = lo0;
			tvc.hi = hi0;
			alloc.axisWeight = w0;
			ns = 0;
			nl = 0;
			foreach (SurfaceEffector se2 in surfaces)
			{
				if (RpStallLimit && se2.kind == SurfaceKind.Stabilator && se2.active && nl < pinLo.Length)
				{
					se2.lo = pinLo[nl];
					se2.hi = pinHi[nl];
					nl++;
				}
				se2.HonestPitchOff();
				if (se2.noPitch && se2.partner != null && se2.kind != SurfaceKind.Rudder && ns < symSave.Length)
				{
					se2.symCost = symSave[ns++];
				}
			}
			rpPitchErr = e;
			if (!ok)
			{
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
			}
		}

		internal static bool RpStallLimit = true;

		internal static float RpStallMargin = 0f;

		internal static float RpStabReserve = 0f;

		internal static float RpTwBand = 0f;

		// 1.8.42: on roll stops/reversals the nozzles are no longer let go (RpRevBand 12 -> 0); the pass is instead accepted
		// with up to RpRevTol deg/s^2 more modelled pitch error than free nozzles would leave, so the stabs and roll rate
		// take the reversal's pitch swing and the nozzles stay near trim (2026-09-28 21:18 log: -20 deg and +1.7..3.3 deg
		// over the 12 deg cap on reversals at 600 km/h).
		internal static float RpRevBand = 12f;

		internal static float RpRevTol = 0f;

		internal static float RpRevHold = 0.6f;

		internal static float HonestRollGate = 0f;

		private float rpRevNow;

		internal static float RpGateSpan = 1f;



		internal static float RpMainStallMargin = 3f;


		internal static float StabParkAoA = 30f;

		// 0.4.3 settled stall window (F-22E 2.3.0): like StallWindow, but a surface outside the attached range is brought in at
		// 'rate' deg/s: the window's near edge starts at the surface's own command and moves toward the range, so the
		// allocator re-solves around a small step each frame instead of a 30 deg jump
		private static void SettledWindow(SurfaceEffector se, float lim, float rate, float dt)
		{
			float a = float.MaxValue;
			float b = float.MinValue;
			for (int i = 0; i < se.N; i++)
			{
				float g = se.grid[i];
				if (Mathf.Abs(se.LocalAoAAt(g)) <= lim)
				{
					a = Mathf.Min(a, g);
					b = Mathf.Max(b, g);
				}
			}
			if (a > b)
			{
				return;
			}
			float h = 0.5f * se.Step;
			float lo = Mathf.Max(se.lo, a - h);
			float hi = Mathf.Min(se.hi, b + h);
			if (lo > hi)
			{
				return;
			}
			float c = se.cmd;
			float step = Mathf.Max(0.01f, rate * dt);
			if (c < lo)
			{
				lo = Mathf.Min(lo, c + step);
				hi = Mathf.Max(hi, lo);
			}
			else if (c > hi)
			{
				hi = Mathf.Max(hi, c - step);
				lo = Mathf.Min(lo, hi);
			}
			se.lo = Mathf.Max(se.lo, lo);
			se.hi = Mathf.Min(se.hi, hi);
			if (se.lo > se.hi)
			{
				se.lo = (se.hi = Mathf.Clamp(c, se.min, se.max));
			}
		}

		internal static bool SettledWindowRudders = true;

		private bool settledWin;

		public Vector3 allocResMain;

		public bool settledWinDbg;

		// 0.4.6 telemetry: the thrust moment's yaw about the CoM, which dMeasY leaves out (the nozzles are an effector, so their
		// whole moment, the CoM-offset part included, is subtracted there); asymLoadY = dMeasY + asymThrY is the measured load
		// yaw comparable with asymMy (the model's, thrust included). Plus the mass and the body-axis specific force.
		public float nxDbg;

		private float settledUseF;

		private bool settledUseHi;
		internal static float CoordStopErr0 = 3f;





		// 2.3.3 rate-capture effector lag: servo/actuator lag added to the unwind time (s)
		internal static float RateCaptureLagServo = 0.06f;

		public float rateCapLagDbg;
		public Vector3 axWDbg;   // 0.6.0: the allocator axis weights this tick (telemetry)
		public float sweepLimDbg, stopSlipHoldDbg;   // 0.6.0: retired with the 2.6.6 sweep cap and the AoA-rise cap (F-22E 3.0.0 replaced them); kept as columns, always 0

		private static void StallWindow(SurfaceEffector se, float lim)
		{
			float a = float.MaxValue;
			float b = float.MinValue;
			// 2.2.18: the attached window is searched over the surface's whole travel, not only this frame's reach. Searched
			// within the reach, a stab more than one frame's travel past its stall found no attached point and was left
			// alone, so it stayed stalled until the AoA brought it back (19 Oct 20:11 log, 31.5-34.3 s: left stab at -30,
			// local AoA 47 -> 38, for 2.8 s after the release while the jet kept rolling at 3-17 deg/s).
			for (int i = 0; i < se.N; i++)
			{
				float g = se.grid[i];
				if (Mathf.Abs(se.LocalAoAAt(g)) <= lim)
				{
					a = Mathf.Min(a, g);
					b = Mathf.Max(b, g);
				}
			}
			if (a > b)
			{
				return;
			}
			float h = 0.5f * se.Step;
			float lo = Mathf.Max(se.lo, a - h);
			float hi = Mathf.Min(se.hi, b + h);
			if (lo > hi)
			{
				// the attached window is out of this frame's reach: go toward it as far as the surface can
				lo = (hi = ((b + h < se.lo) ? se.lo : se.hi));
			}
			se.lo = lo;
			se.hi = hi;
		}

		// 2.14.0 telemetry: the stabilators' allocator windows and pair costs (index 0 = first stab found, 1 = second), for the no-fin stop diagnosis
		private float StabWin(int which, bool lo)
		{
			int k = 0;
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind != SurfaceKind.Stabilator) continue;
				if (k == which) return lo ? se.lo : se.hi;
				k++;
			}
			return 0f;
		}

		private float StabCost(bool anti)
		{
			foreach (SurfaceEffector se in surfaces)
			{
				if (se.kind == SurfaceKind.Stabilator) return anti ? se.antiCost : se.symCost;
			}
			return 0f;
		}

		private float PitchSum()
		{
			float y = 0f;
			foreach (Effector ef in effs)
			{
				if (ef.active)
				{
					y += ef.MAt(ef.cmd).y;
				}
			}
			return y;
		}

		private void RollPitchGateStep(float dt)
		{
			float g = (Plugin.RollPitchPriority && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? RpEnable : 0f;
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.2f), Mathf.Clamp01(Mathf.Abs(pCmd) / 30f));
			rpLat = Mathf.Max(latNow, rpLat - dt / Mathf.Max(0.05f, RpRelease));
			g *= rpLat * (1f - Mathf.Clamp01((Mathf.Abs(alpha) + stabStallShift - RpAoA0) / Mathf.Max(1f, RpAoA1 - RpAoA0)));
			rpGate = Mathf.Clamp01(g);
		}

		// 1.8.32: during roll/yaw the nozzles keep within RollTvcBudget of where they were before the lateral input
		// (the pitch trim they held for the pilot's own pitch command); the pitch moment the roll itself makes is
		// taken by the stabilators beyond that. A cost in the allocator rather than a clamp: when the stabs cannot
		// supply it (on a stop, stalled) the nozzles still go past the budget, as far as the pitch error needs.
		// The band opens to full travel between 14 and 20 deg AoA, between T/W 0.7 and 1.1, in AoA protection / run-away
		// and while the pitch-rate command is moving (AoA/G capture braking).
		private void TvcBudgetStep(float dt, Vector3 target)
		{
			tvc.anchorK = 0f;
			float budget = Plugin.RollTvcBudget;
			if (!tvcAnchorInit)
			{
				tvcAnchor = tvc.cmd;
				tvcAnchorInit = true;
			}
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) - 0.02f) / 0.2f), Mathf.Clamp01(Mathf.Abs(pCmd) / 30f));
			tvcBudgetLat = Mathf.Max(latNow, tvcBudgetLat - dt / Mathf.Max(0.05f, TvcBudgetRelease));
			// Where the stabs can take the pitch (g = 1) the band is RollTvcBudget; it opens toward the full nozzle travel
			// as they lose that ability or the nozzles need the room: AoA from 14 to 20 deg (the stab going trailing-edge
			// down for the roll is then at ~30 deg local AoA, where the F22E_Wing foil's lift curve flattens, and nose-down from it costs roll rate:
			// 13-22 % in 250 km/h roll spam on the bench), AoA protection / run-away, and while the pitch-rate command is
			// moving (AoA/G capture braking keeps full authority).
			float g = 1f;
			g *= 1f - Mathf.Clamp01((Mathf.Abs(alpha) + stabStallShift - TvcBudgetAoA) / Mathf.Max(1f, TvcBudgetAoAFade));
			g *= prot ? 0f : (1f - pitchNeed);
			g *= 1f - Mathf.Clamp01(Mathf.Abs(qTraj.v) / TvcBudgetQdot);
			g *= stabPitchFrac;
			// With afterburner-class thrust the nozzles' roll trim is only a few degrees anyway (1.8.31 bench: -2..-7 mean)
			// and they are the fast pitch effector; holding them there moved that fast work onto the stabs, which rang
			// with 3 frames of control delay (450 km/h AB roll spam: q sd 2.1 -> 3.8 deg/s). Full budget up to T/W 0.7,
			// open by 1.1.
			// Revoker 0.1.0 (CanardPitchPriority): the canards are the faster pitch effector (100 deg/s vs the nozzle's 70)
			// and sit far ahead of the CG, so the budget holds at every thrust
			if (!Plugin.CanardPitchPriority)
			{
				g *= 1f - Mathf.Clamp01((tw - TvcBudgetTw0) / Mathf.Max(0.05f, TvcBudgetTwFade));
			}
			float active = tvcBudgetLat;
			if (budget <= 0f || Plugin.RollTvcBudgetCost <= 0f)
			{
				active = 0f;
			}
			tvcBudgetGate = active * g;
			// the anchor (pre-roll nozzle trim for the pilot's own pitch command) follows the nozzles only while no lateral input is held
			float follow = 1f - Mathf.Clamp01(active / 0.3f);
			tvcAnchor += (tvc.cmd - tvcAnchor) * follow * (dt / (TvcAnchorTau + dt));
			// 2.0.0: with lateral input held, the anchor no longer freezes at the trim from before the roll. It follows the
			// nozzle position the same allocation gives for this tick's pitch demand with no roll or yaw demand: the trim for
			// the pilot's own (possibly changing) pitch command, so only the roll's pitch moment is held off the nozzles. A
			// frozen anchor made the stabs carry every change of pitch trim during a held roll (slow ease from high to low
			// AoA): with the 25 deg stab peak they sat on it for nose-down, the roll stab parked deep in stall, the nozzles
			// pushed against them, and roll rate spiked when the stabs came free (bench, 400 km/h: 141 vs 186 deg/s mean).
			if (AnchorPitchOnly && active > 0.001f && budget > 0f && Plugin.RollTvcBudgetCost > 0f)
			{
				int n = effs.Count;
				if (cmdSave.Length < n)
				{
					cmdSave = new float[n * 2];
				}
				for (int i = 0; i < n; i++)
				{
					cmdSave[i] = effs[i].cmd;
				}
				// the stabs are kept on the attached side of their peak here: a stab parked past it (and the nose-up the
				// nozzles then hold against it) is not the pilot's trim
				int nl = 0;
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
					{
						pinLo[nl] = se.lo;
						pinHi[nl] = se.hi;
						nl++;
						float g0 = se.cmd;
						StallWindow(se, se.stallAoA);
						se.cmd = Mathf.Clamp(g0, se.lo, se.hi);
					}
				}
				alloc.Solve(effs, new Vector3(0f, target.y, 0f));
				nl = 0;
				foreach (SurfaceEffector se in surfaces)
				{
					if (se.kind == SurfaceKind.Stabilator && se.active && nl < pinLo.Length)
					{
						se.lo = pinLo[nl];
						se.hi = pinHi[nl];
						nl++;
					}
				}
				float trimNow = tvc.cmd;
				for (int i = 0; i < n; i++)
				{
					effs[i].cmd = cmdSave[i];
				}
				tvcAnchor += (trimNow - tvcAnchor) * (1f - follow) * (dt / (TvcAnchorTau + dt));
			}
			if (active <= 0.001f)
			{
				return;
			}
			float span = Mathf.Max(tvc.max, 0f - tvc.min);
			// Revoker 0.1.0 (CanardPitchPriority): the band is about neutral, not the pitch-only trim. That trim is the
			// nozzle angle of a pitch-only solve, where the nozzle is the cheapest pitch effector; held there, the nozzle went
			// on carrying the roll's pitch (bench, 550 km/h 0.5-stick roll: anchor drifting to -14, nozzle pinned at -15).
			// With it at neutral the canards take the roll's pitch moment and the pilot's trim; the band still opens where
			// they cannot (their stall band, AoA protection, capture braking).
			tvc.anchor = Plugin.CanardPitchPriority ? 0f : tvcAnchor;
			tvc.band = Mathf.Lerp(span * 2f, budget, active * g);
			tvc.anchorK = Plugin.RollTvcBudgetCost;
		}

		internal static float TvcBudgetRelease = 0.5f;

		// Revoker 0.1.0: 28 / 10 (F-22E 14 / 6). The canard foil's lift keeps rising to 45 deg local AoA (CL 0.9 at 17, 1.0 at
		// 45), so the canard going TE-down for a roll still has nose-down to give well past the F-22E stab's 14-20 deg band.
		// Bench, 0.5-stick rolls at 350-450 km/h (AoA 13-25): nozzle -15 -> -7 deg, pitch-rate error <= 3.8 deg/s.
		internal static float TvcBudgetAoA = 28f;

		internal static bool StabPeakShift = true;

		internal static float StabPeakRef = 33.75f;

		public float stabStallShift;

		internal static float TvcBudgetAoAFade = 10f;

		internal static float TvcBudgetQdot = 25f;

		internal static float TvcBudgetTw0 = 0.7f;

		internal static float TvcBudgetTwFade = 0.4f;

		internal static float TvcAnchorTau = 0.25f;

		internal static bool AnchorPitchOnly = true;

		internal static bool AuthRateLimit = true;

		internal static float TvcLowThrustLam = 0f;

		internal static float TvcLowThrustTw = 0.3f;

		internal static float AuthRateStopAngle = 15f;

		internal static float AuthRateLook = 5f;

		internal static float AuthRateMin = 6f;

		internal static float AuthBrakeFloor = 4f;

		public float authRateUp = 999f;

		public float authRateDn = 999f;

		internal static bool AuthCapFixed = true;

		internal static float AuthCapFixedAoA = 36f;


		private bool authFixedOn;

		// 1.9.0: highest AoA at which the nose-down authority available now (stabs full nose-down, flaperons where they
		// are, nozzles at the present thrust - none above the vectoring cut-off speed) still leaves LowThrustCapMargin.
		private void AuthCapFromSweep()
		{
			if (!Plugin.LowThrustAoACap || Ia[1, 1] <= 1f)
			{
				authCap = 999f;
				return;
			}
			float tvDn = 0f;
			if (tvc.active && tvc.M != null)
			{
				float tvLo = float.MaxValue;
				for (int tk = 0; tk < tvc.N; tk++)
				{
					tvLo = Mathf.Min(tvLo, tvc.M[tk].y);
				}
				if (tvLo < float.MaxValue)
				{
					tvDn = Mathf.Min(0f, tvLo * IaInv[1, 1] * 57.29578f);
				}
			}
			// 1.9.5: nozzle nose-down authority growing (throttle up / spool-up): the cap may follow at once
			if (0f - tvDn > authTvPrev + AuthCapThrustStep)
			{
				authFastT = AuthCapFastHold;
			}
			authTvPrev = 0f - tvDn;
			float margin = Plugin.AuthCapMargin;
			float cap = 999f;
			float prevA = 0f;
			float prevM = float.NaN;
			for (int ia = 0; ia < asymAlphas.Length; ia++)
			{
				float aA = asymAlphas[ia];
				if (aA < 0f)
				{
					continue;
				}
				float mDn = 0f - (authNetDn[ia] + tvDn);
				if (mDn < margin)
				{
					float aP = (float.IsNaN(prevM) || prevM <= mDn) ? aA : Mathf.Lerp(prevA, aA, (prevM - margin) / (prevM - mDn));
					cap = Mathf.Max(Plugin.AuthCapFloor, aP - AuthCapLead);
					break;
				}
				prevA = aA;
				prevM = mDn;
			}
			authCap = cap;
		}

		// 2.0.3: nose-down pitch acceleration (deg/s^2) the nozzles give at the present thrust below which they count as
		// not covering the pitch authority (fixed AuthCapFixedAoA limit). 2.0.4: 12 -> 20 (~T/W 0.2; idle is ~9): between T/W 0.13
		// and 0.2 a pull from high IAS overshot the swept cap by 13 deg as it came down with the speed (bench, in game 102-135 s).
		internal static float AuthFixedTvcMin = 20f;

		// 1.9.0: the flaperons' symmetric deflection for the low-thrust cap, followed only while no roll/yaw is commanded:
		// during a roll one flaperon runs into its stop and the pair's mean shifts, which is not a trim the cap should judge by
		private float[] flapSymRef;

		private bool flapSymInit;

		private void FlapSymStep(float dt)
		{
			float[] anti = asymDsym;
			if (anti == null || anti.Length != surfaces.Count)
			{
				return;
			}
			if (flapSymRef == null || flapSymRef.Length != surfaces.Count)
			{
				flapSymRef = new float[surfaces.Count];
				flapSymInit = false;
			}
			bool quiet = Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy)) < 0.05f && Mathf.Abs(omega.x * 57.29578f) < 15f;
			float k = Mathf.Clamp01(dt / 0.3f);
			for (int i = 0; i < surfaces.Count; i++)
			{
				SurfaceEffector se = surfaces[i];
				float sym = se.now - anti[i];
				if (!flapSymInit)
				{
					flapSymRef[i] = sym;
				}
				else if (quiet)
				{
					flapSymRef[i] += (sym - flapSymRef[i]) * k;
				}
			}
			flapSymInit = true;
		}

		// 2.0.10: below the high-AoA regime a stab is not commanded past its lift peak (local AoA within the peak +
		// StabPeakMargin, either sign). Past the peak the lift curve is flat or falling, so further travel buys nothing,
		// and the allocator could not get back: in Tom's 10:00 flight a left roll at ~5 deg AoA drove the right stab to
		// -30 (local 32, peak 25) and it stayed there for 10 s, the rolling-back direction looking worse until it had
		// crossed the peak, with every other surface saturated against it. At 36 deg AoA (stab at its peak by design)
		// the same search swung the stabs +-30 every 1.5 s. The margin opens up from 40 to 50 deg body AoA, so the deep-
		// stall use above that (falling leaf, post-stall roll) is unchanged.
		internal static bool StabPeakClamp = true;


		private void ClampStabToPeak(SurfaceEffector se)
		{
			float x = Mathf.Abs(alpha);
			float lim = se.stallAoA + Plugin.StabPeakMargin + Mathf.Max(0f, x - 40f) * 8f;
			if (lim > 89f || !se.Valid)
			{
				return;
			}
			float l0 = se.LocalAoAAt(0f);
			float slope = (se.LocalAoAAt(10f) - l0) / 10f;
			if (float.IsNaN(slope) || Mathf.Abs(slope) < 0.5f)
			{
				return;
			}
			float d1 = (lim - l0) / slope;
			float d2 = (0f - lim - l0) / slope;
			float dLo = Mathf.Min(d1, d2);
			float dHi = Mathf.Max(d1, d2);
			float lo = Mathf.Max(se.lo, dLo);
			float hi = Mathf.Min(se.hi, dHi);
			if (lo < hi - 1f)
			{
				se.lo = lo;
				se.hi = hi;
			}
		}

		// 2.2.2: the positive no-TVC cap. With DisableTVC the stabs alone hold 45 deg with the thrust up (bench, 160-750 km/h,
		// rolls included); at part throttle, rolling as the speed bled through ~200-250 km/h, 45 departed where 36 held. So
		// NoTvcAoACapPositive (36) up to T/W NoTvcThrustCapTW0, NoTvcThrustAoACapPositive (45) from NoTvcThrustCapTW1.
		internal static float NoTvcThrustCapTW0 = 0.8f;

		internal static float NoTvcThrustCapTW1 = 1f;

		private float NoTvcCapPosNow()
		{
			if (!Plugin.DisableTVC)
			{
				return Plugin.NoTvcAoACapPos;
			}
			float k = Mathf.Clamp01((tw - NoTvcThrustCapTW0) / Mathf.Max(0.05f, NoTvcThrustCapTW1 - NoTvcThrustCapTW0));
			return Mathf.Lerp(Plugin.NoTvcAoACapPos, Mathf.Max(Plugin.NoTvcAoACapPos, Plugin.NoTvcThrCapPos), k);
		}

		private void AuthCapStep(float dt)
		{
			// 2.0.9: after a change of the airframe test switches the cap follows its new target as fast as it comes down
			// (2.0.8 let it creep up from 36 at the at-cap rise rate, which looked like the old limit was still on)
			string key = Plugin.DisableTVC + "|" + Plugin.AoAFull65 + "|" + Plugin.AoACap36 + "|" + Plugin.NoTvcAoACapPos.ToString("F1") + "|" + Plugin.NoTvcAoACapNeg.ToString("F1") + "|" + Plugin.NoTvcThrCapPos.ToString("F1") + "|" + Plugin.NoTvcThrCapNeg.ToString("F1");
			if (testSwitchKey != key)
			{
				if (testSwitchKey != null)
				{
					// a looser cap is followed as fast as a tighter one (the 1.9.5 thrust-step path)
					authFastT = AuthCapFastHold;
				}
				testSwitchKey = key;
			}
			if (!Plugin.LowThrustAoACap)
			{
				authCapEff = 999f;
				authCapNegEff = -999f;
				authInit = false;
				return;
			}
			// 2.0.9: nozzles not covering pitch: low thrust (T/W under LowThrustTW, released 0.03 above) or DisableTVC
			noTvcOn = Plugin.DisableTVC || (noTvcOn ? (tw < Plugin.LowThrustTW + 0.03f) : (tw < Plugin.LowThrustTW));
			bool negCapOn = noTvcOn && !Plugin.AoAFull65;
			// 2.2.2: with DisableTVC the negative cap is NoTvcThrustAoACapNegative (45) at every thrust
			float tgtNeg = negCapOn ? Mathf.Max(Plugin.AoANeg, 0f - (Plugin.DisableTVC ? Plugin.NoTvcThrCapNeg : Plugin.NoTvcAoACapNeg)) : -999f;
			if (!authInit || tgtNeg <= authCapNegEff)
			{
				// first step, or released / loosened: straight to the new limit
				authCapNegEff = tgtNeg;
			}
			else
			{
				// tightening toward 0: from the limit that applied so far, as fast as the positive cap comes down
				float from = Mathf.Max(authCapNegEff, Plugin.AoANeg);
				authCapNegEff = from + (tgtNeg - from) * Mathf.Clamp01(dt / AuthCapTauDown);
			}
			float tgt = Mathf.Min(authCap, Plugin.AoAPos);
			// 2.2.19 (FinLossAoACap): with the fins gone the positive AoA is capped where the sideslip stays bounded in a held
			// roll (bench, both fins detached: 35-45 deg holds, 25-30 departs with full roll held, 50-65 lets the sideslip
			// grow to 15-22 deg with the nozzles on). Phased in from one fin lost (yawHealth 0.5) to both (0).
			if (Plugin.FinLossAoACap > 0f && yawHealth < 0.5f)
			{
				float kF = Mathf.Clamp01((0.5f - yawHealth) / 0.5f);
				tgt = Mathf.Min(tgt, Mathf.Lerp(Plugin.AoAPos, Plugin.FinLossAoACap, kF));
			}
			// Revoker 0.2.7 (FinLossNoTvcAoACap): with the fin damaged and no nozzle yaw (TVC off or below the low-thrust T/W), the
			// yaw left is the canards' and the elevons' drag, a few deg/s^2 at high AoA, against a fuselage that is directionally
			// unstable there. 28 Oct 20:31 log (TVC off, fin gone at 685 s): at the 45 deg cap, 50-60 m/s, the sideslip ran
			// -1 -> -75 deg in 10 s with every surface on its stop, then the jet flipped. Bench, fin gone, TVC off, held pull +
			// roll: 45-50 deg departs at 250-300 km/h, 40 departs with the game-like command delay, 35 and below hold (sideslip
			// <= 3.3 deg). Phased in from no damage (yawHealth 1) to the fin gone (0).
			if (Plugin.FinLossNoTvcAoACap > 0f && yawHealth < 0.999f && (noTvcOn || tvcy == null || !tvcy.active))
			{
				float kF2 = Mathf.Clamp01((1f - yawHealth) / 0.5f);
				tgt = Mathf.Min(tgt, Mathf.Lerp(Plugin.AoAPos, Plugin.FinLossNoTvcAoACap, kF2));
			}
			// 2.0.3: when the nozzles can no longer cover the nose-down need (a cap exists at all), the limit is a fixed
			// AuthCapFixedAoA instead of a value sliding with IAS, as long as the authority sweep allows it there;
			// below that the swept cap still rules. Engages when a cap appears below 62 deg, releases above 64.5.
			if (AuthCapFixed)
			{
				// nozzles short of nose-down authority (decided by the thrust you have, so a pull at idle from high IAS is
				// limited from the start instead of the cap coming down onto the jet as the speed bleeds), or a swept cap
				// 2.0.5: judged by thrust-to-weight (Tom: floor T/W 0.3), release 0.03 above
				bool tvShort = authFixedOn ? (tw < Plugin.LowThrustTW + 0.03f) : (tw < Plugin.LowThrustTW);
				// 2.0.8: with the nozzles locked (DisableTVC) they never cover pitch
				tvShort |= Plugin.DisableTVC;
				authFixedOn = tvShort || (authFixedOn ? (authCap < Plugin.AoAPos - 0.5f) : (authCap < Plugin.AoAPos - 3f));
				// 2.0.8: AoALimit Full65 (debug) turns the fixed 36 deg rule off; the swept authority cap still applies
				if (Plugin.AoAFull65)
				{
					authFixedOn = false;
				}
				if (authFixedOn)
				{
					// 2.0.9: the fixed value is the debug NoTvcAoACapPositive (36 by default)
					tgt = Mathf.Min(tgt, NoTvcCapPosNow());
				}
			}
			if (!authInit)
			{
				authCapEff = tgt;
				authInit = true;
				return;
			}
			if (tgt < authCapEff)
			{
				authCapEff += (tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauDown);
				return;
			}
			// 1.9.1: rising, the cap is also rate-limited, and nearly frozen while the jet sits at or above it. In 1.9.0 the
			// exponential rise (up to ~10 deg/s just after a drop) let the capture re-accelerate the nose into the still-
			// rising cap and the recovery push it back: a 1 Hz, 2-18 deg/s pitch-rate cycle at 55-60 deg AoA, half thrust.
			authFastT = Mathf.Max(0f, authFastT - dt);
			if (authFastT > 0f)
			{
				// 1.9.5: the rise comes from more thrust (the nozzles' nose-down authority grew at the last sweeps), not from
				// the airframe - follow it as fast as a drop. 1.9.1-1.9.4 held it to 1 deg/s while the jet sat at the cap, so
				// idle -> max AB with the stick held took ~40 s to give back the full AoA.
				authCapEff += (tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauDown);
				return;
			}
			float riseMax = ((alpha > authCapEff - 1f) ? AuthCapRiseAtCap : AuthCapRise) * dt;
			authCapEff += Mathf.Min((tgt - authCapEff) * Mathf.Clamp01(dt / AuthCapTauUp), riseMax);
		}

		private static float SmoothMin(float a, float b, float k)
		{
			float h = Mathf.Clamp01(0.5f + 0.5f * (b - a) / Mathf.Max(0.01f, k));
			return Mathf.Lerp(b, a, h) - k * h * (1f - h);
		}

		private static float Smooth01(float x)
		{
			x = Mathf.Clamp01(x);
			return x * x * (3f - 2f * x);
		}

		// 1.9.0: AoA/G command pre-filter. The reference moves toward the stick's command at a speed that grows with the
		// step (small steps gently, large ones as before), its rate building up over an onset time that also shrinks with
		// the step. Released stick: the reference sits on the jet's present AoA/G, so the next step is measured from there.
		private static float ShapeStep(ref float x, ref float v, ref float mag, float target, float span, float kSmall, float kLarge, float onsetSmall, float dt)
		{
			float d = target - x;
			// the step's size is held (decaying at one span per second) so a large step keeps its speed all the way in
			// instead of slowing down on its last few degrees / tenths of a g as if it were a small one
			mag = Mathf.Max(Mathf.Abs(d), mag - span * dt);
			float r = Mathf.Clamp01(mag / span);
			float s = r * r;
			float k = Mathf.Lerp(kSmall, kLarge, s);
			float ton = Mathf.Lerp(onsetSmall, 0.03f, s);
			float vDes = k * d;
			if (vDes * v < 0f)
			{
				v = 0f;
			}
			float aLim = Mathf.Max(Mathf.Abs(vDes) / Mathf.Max(0.01f, ton), k * k * Mathf.Abs(d));
			v = Mathf.MoveTowards(v, vDes, aLim * dt);
			float xn = x + v * dt;
			if ((target - xn) * d <= 0f)
			{
				xn = target;
				v = 0f;
			}
			x = xn;
			return x;
		}

		private void ShapeCommands(float dt, bool held)
		{
			if (!Plugin.CmdShaping)
			{
				alphaCmd = alphaCmdRaw;
				nCmd = nCmdRaw;
				shapeInit = false;
				return;
			}
			if (!held || !shapeInit)
			{
				aRefX = alpha;
				nRefX = nz;
				aRefV = 0f;
				nRefV = 0f;
				aRefMag = 0f;
				nRefMag = 0f;
				shapeInit = true;
			}
			if (held)
			{
				ShapeStep(ref aRefX, ref aRefV, ref aRefMag, alphaCmdRaw, Plugin.ShapeAoASpan, Plugin.ShapeAoAGainSmall, Plugin.ShapeAoAGainLarge, Plugin.ShapeOnsetSmall, dt);
				ShapeStep(ref nRefX, ref nRefV, ref nRefMag, nCmdRaw, Plugin.ShapeGSpan, Plugin.ShapeGGainSmall, Plugin.ShapeGGainLarge, Plugin.ShapeOnsetSmall, dt);
				alphaCmd = aRefX;
				nCmd = nRefX;
			}
			else
			{
				alphaCmd = alphaCmdRaw;
				nCmd = nCmdRaw;
			}
		}

		private float aRefX;

		private float nRefX = 1f;

		private float aRefMag;

		private float nRefMag;

		private void PitchLaw(float dt, float Vc)
		{
			qB = RateBudget();
			AuthCapStep(dt);
			latReserve = Mathf.Clamp01(Plugin.LateralStabReserve) * Mathf.Clamp01(Mathf.Max(Mathf.Abs(sr), Mathf.Abs(sy))) * Mathf.Clamp01((Mathf.Abs(sp) - 0.03f) / 0.15f);
			float aUpP = aUp;
			float aDnP = aDn;
			if (latReserve > 0.001f)
			{
				float tvUp = 0f;
				float tvDn = 0f;
				if (tvc.active && tvc.M != null && tvc.N > 1)
				{
					float tvMax = float.MinValue;
					float tvMin = float.MaxValue;
					for (int tvI = 0; tvI < tvc.N; tvI++)
					{
						tvMax = Mathf.Max(tvMax, tvc.M[tvI].y);
						tvMin = Mathf.Min(tvMin, tvc.M[tvI].y);
					}
					// Revoker 0.2.12 (LatReserveTvcFull): the nozzles' share is their whole travel either side of mid-travel, not
					// what is left from where they are now. Measured from the current nozzle angle, the share shrank as the
					// nozzles moved to brake the pitch (none left at the stop), so the reserve took 70 % of everything and the
					// authority the capture planned with fell 120 -> 42 deg/s^2 mid-stop. The capture then cut the pitch command
					// (48 -> 21 deg/s), the nozzles came off the stop, the authority came back and the command rose again: the
					// AoA stepped in to the negative limit in two or three surges (29 Oct 11:40 log, 90-93 s and 120-123 s, full push +
					// full roll at 150 kt, capture from -34 deg against the -55 limit).
					float tvMid = 0.5f * (tvMax + tvMin);
					if (LatReserveTvcFull)
					{
						tvUp = Mathf.Max(0f, (tvMax - tvMid) * IaInv[1, 1] * 57.29578f);
						tvDn = Mathf.Max(0f, (tvMid - tvMin) * IaInv[1, 1] * 57.29578f);
					}
					else
					{
						tvUp = Mathf.Max(0f, (tvMax - tvc.Mnow.y) * IaInv[1, 1] * 57.29578f);
						tvDn = Mathf.Max(0f, (tvc.Mnow.y - tvMin) * IaInv[1, 1] * 57.29578f);
					}
				}
				aUpP = Mathf.Max(20f, aUp - latReserve * Mathf.Max(0f, aUp - tvUp));
				aDnP = Mathf.Max(20f, aDn - latReserve * Mathf.Max(0f, aDn - tvDn));
			}
			aUpPitch = aUpP;
			aDnPitch = aDnP;
			bool flag = Mode != FcsMode.MPO;
			float num = (flag ? Plugin.AoAPos : 999f);
			float num2 = (flag ? Plugin.AoANeg : (-999f));
			float num3 = (flag ? Mathf.Min(Mathf.Min(num, asymCapPosEff), authCapEff) : num);
			float num4 = (flag ? Mathf.Max(Mathf.Max(num2, asymCapNegEff), authCapNegEff) : num2);
			float num5 = ((Mode == FcsMode.MPO) ? Plugin.MpoGPos : Plugin.GPos);
			float num6 = ((Mode == FcsMode.MPO) ? Plugin.MpoGNeg : Plugin.GNeg);
			float num7 = 9.81f / Vc * 57.29578f;
			float num8 = (dbgWVe = omega.y * 57.29578f - alphaDot);
			alphaDotHoldG = Mathf.Clamp(2f * Mathf.Abs(nz) * Mathf.Max(0f, 0f - vDot) / (Vc * Mathf.Max(nAlpha, 0.02f)), 0f, 15f);
			unload = false;
			prot = false;
			float num9 = 1f;
			float num10 = 0f;
			alphaCmd = 0f;
			nCmd = 1f;
			betaG = 0f;
			if (Mode == FcsMode.AoAG)
			{
				// 1.9.0: hand-over AoA (lower when the wings can barely lift the weight), the stick breakpoint that keeps the
				// AoA scale below it, and how far into the rate-command travel the stick is
				float aTgtRb = Plugin.RateBlendAoA;
				if (nLiftMax < Plugin.RateBlendLiftG)
				{
					aTgtRb = Mathf.Lerp(Mathf.Min(Plugin.RateBlendAoAMin, Plugin.RateBlendAoA), Plugin.RateBlendAoA, Mathf.Clamp01((nLiftMax - 1f) / (Plugin.RateBlendLiftG - 1f)));
				}
				// 1.9.7: the negative side's threshold (magnitude) from the wings' negative-lift capacity
				float aTgtRbN = Plugin.RateBlendAoANeg;
				if (nLiftMin < Plugin.RateBlendLiftG)
				{
					aTgtRbN = Mathf.Lerp(Mathf.Min(Plugin.RateBlendAoAMinNeg, Plugin.RateBlendAoANeg), Plugin.RateBlendAoANeg, Mathf.Clamp01((nLiftMin - 1f) / (Plugin.RateBlendLiftG - 1f)));
				}
				rateBlendAoA = (rbInit ? (rateBlendAoA + (aTgtRb - rateBlendAoA) * Mathf.Clamp01(dt / 0.5f)) : aTgtRb);
				rateBlendAoANeg = (rbInit ? (rateBlendAoANeg + (aTgtRbN - rateBlendAoANeg) * Mathf.Clamp01(dt / 0.5f)) : aTgtRbN);
				rbInit = true;
				// 1.9.1 stick map (fixed AoA per stick: RateBlendAoA at RateBlendStick, 72 deg per full stick by default).
				// The threshold's stick point moves with the threshold (24 deg -> 0.333). Up to it: AoA command on that line.
				// Past it: AoA command on from the threshold to the envelope limit at full stick (full stick = the limit, so
				// the jet goes through the threshold at the rate budget); once the jet crosses the threshold with the stick
				// there, pitch-rate command. With the envelope limit (asymmetric / low-thrust cap) at or below the
				// threshold the whole travel maps onto the limit as before; blended over the 4 deg above it.
				// 1.9.7: the same on the negative side, mirrored. Everything below runs in side space (s = +1 pull, -1 push):
				// stick s*sp, AoA s*alpha, limit s*limit, threshold as a magnitude; the outputs are mapped back with s.
				bool blendP = Plugin.HighAoARateBlend;
				bool blendN = Plugin.HighAoARateBlendNeg;
				float aTP = rateBlendAoA;
				float aTN = rateBlendAoANeg;
				float limP = num3;
				float limN = 0f - num4;
				float gAP = Plugin.RateBlendAoA / Plugin.RateBlendStick;
				StickCurveSetup();
				float gAN = Plugin.RateBlendAoANeg / Plugin.RateBlendStick;
				float zwP = (blendP ? Mathf.Clamp01((limP - aTP) / 4f) : 0f);
				float zwN = (blendN ? Mathf.Clamp01((limN - aTN) / 4f) : 0f);
				float aBP = (blendP ? Mathf.Min(aTP, limP) : limP); // 1.9.5: with the blend off full stick is the AoA limit again (1.9.1-1.9.4 stopped at the threshold)
				float aBN = (blendN ? Mathf.Min(aTN, limN) : limN);
				float sBP = Mathf.Lerp(1f, Mathf.Clamp(StickCurveInv(aTP), 0.05f, 0.95f), zwP);
				float sBN = Mathf.Lerp(1f, Mathf.Clamp(aTN / gAN, 0.05f, 0.95f), zwN);
				// floating hold point: the jet came back down to the threshold out of rate command with the stick past the
				// threshold's point - that stick position is the threshold now; easing below it holds the (live) threshold
				// with a soft knee until the stick meets the threshold's own point on the line, where it is caught
				if (floatOn)
				{
					float fsp = (float)floatSide * sp;
					bool fzone = ((floatSide > 0) ? zwP : zwN) > 0.001f;
					if (!fzone || fsp <= ((floatSide > 0) ? sBP : sBN) || Mathf.Abs(sp) < 0.03f || fsp > 0.98f)
					{
						floatOn = false;
					}
				}
				int sm = ((sp >= 0f) ? 1 : (-1));
				float spS = (float)sm * sp;
				float aT = ((sm > 0) ? aTP : aTN);
				float zw = ((sm > 0) ? zwP : zwN);
				bool zone = zw > 0.001f;
				float aB = ((sm > 0) ? aBP : aBN);
				float sB = ((sm > 0) ? sBP : sBN);
				float limS = ((sm > 0) ? limP : limN);
				float sH = ((floatOn && floatSide == sm) ? Mathf.Max(floatS, sB) : sB);
				float aCmdS;
				if (spS <= sH || sH > 0.999f)
				{
					// 2.0.12: the pull side follows the stick curve (StickFor24degAoA / StickFor36degAoA), scaled so the
					// breakpoint still lands on aB; on the default points it is the 1.9.1 straight line
					aCmdS = ((sm > 0) ? (StickCurve(Mathf.Min(spS, 1f)) * aB / Mathf.Max(StickCurve(Mathf.Max(sB, 0.01f)), 0.01f)) : (aB * Mathf.Min(spS, 1f) / Mathf.Max(sB, 0.01f)));
					if (floatOn && floatSide == sm)
					{
						aCmdS = SmoothMin(aCmdS, aT, FloatKnee);
					}
					aCmdS = Mathf.Min(aCmdS, limS);
				}
				else
				{
					aCmdS = aB + (spS - sH) / (1f - sH) * Mathf.Max(0f, limS - aB);
				}
				alphaCmdRaw = (float)sm * aCmdS;
				nCmdRaw = ((sp >= 0f) ? (1f + sp * (num5 - 1f)) : (1f + sp * (1f - num6)));
				zoneX = ((zone && spS > sH) ? Mathf.Clamp01((spS - sH) / Mathf.Max(0.02f, 1f - sH)) : 0f);
				// 2.1.0 G + pitch-rate law (PitchCommandLaw = GAndPitchRate): no AoA command at all. The AoA branch below is
				// replaced by a pitch-rate command; the G branch and the betaG blend are unchanged. The AoA envelope (limits,
				// low-thrust / asymmetric-load caps, AoA protection) still bounds the rate further down as a limit only.
				bool gRate = Plugin.GRateLaw;
				if (gRate)
				{
					alphaCmdRaw = alpha;
					zoneX = 0f;
					hiRate = false;
					floatOn = false;
				}
				num9 = Mathf.Clamp01((Mathf.Abs(sp) - 0.01f) / 0.06f);
				ShapeCommands(dt, num9 > 0.5f);
				float a = 1f + qB * (MathF.PI / 180f) * Vc / 9.81f;
				gAvail = Mathf.Min(a, nMaxAero);
				betaG = Mathf.Clamp01((gAvail - num5) / 3f + 0.5f);
				if (gRate)
				{
				}
				else if (!hiRate)
				{
					if (zone && zoneX > 0f && (float)sm * alpha >= aT && betaG < 0.5f && num9 > 0.5f)
					{
						hiRate = true;
						hiSide = sm;
						hiAnchor = true;
						hiAS = spS;
						hiAQ = Mathf.Clamp((float)sm * qTraj.x, 0f, qB);
						floatOn = false;
					}
				}
				else
				{
					// 1.9.5: only a pilot ease latches it - not a full pull (full stick always means all the way), and not the
					// jet being pushed down through the threshold by an envelope cap coming down (throttle to idle)
					float hs = hiSide;
					float aTh = ((hiSide > 0) ? aTP : aTN);
					float sBh = ((hiSide > 0) ? sBP : sBN);
					bool zoneh = ((hiSide > 0) ? zwP : zwN) > 0.001f;
					bool capPush = ((hiSide > 0) ? (num3 < num - 0.05f && alphaU > num3 - 1f) : (num4 > num2 + 0.05f && alphaU < num4 + 1f));
					if (hs * alpha < aTh && hs * sp > sBh && hs * sp < 0.95f && !capPush && !floatOn && zoneh)
					{
						floatOn = true;
						floatS = hs * sp;
						floatSide = hiSide;
					}
					if (!zoneh || hs * alpha < aTh - Plugin.RateBlendBand || betaG > 0.6f)
					{
						hiRate = false;
					}
				}
				// 2.1.8 AoAThenRateHold: AoA command until the stick is steady and the AoA has settled on the command, then hold
				// the pitch rate the jet has at that moment (the AoA is left to drift with IAS); a stick movement past
				// CapStickBand returns to AoA command (shaping restarts from the present AoA), a release ends it too.
				bool hyb = !gRate && Plugin.AoAThenRateLaw;
				float stickRate = Mathf.Abs(sp - capSpPrev) / Mathf.Max(dt, 1E-4f);
				capSpPrev = sp;
				capSteadyT = (stickRate < CapStickRate) ? (capSteadyT + dt) : 0f;
				if (!hyb || hiRate || num9 < 0.5f || betaG >= 0.5f)
				{
					capOn = false;
				}
				else if (capOn)
				{
					if (Mathf.Abs(sp - capSp) > CapStickBand)
					{
						capOn = false;
						shapeInit = false;
					}
				}
				else if (capSteadyT >= CapSteadyTime && Mathf.Abs(alphaCmd - alpha) < CapAoABand && Mathf.Abs(alphaDot) < CapAlphaDot && Mathf.Abs(sp) > 0.03f)
				{
					capOn = true;
					capSp = sp;
					capQ = omega.y * 57.29578f;
				}
				bool flag2 = !gRate && !hiRate && !capOn && sp * alpha >= 0f && Mathf.Abs(alpha) > 3f && Mathf.Abs(alphaCmd) < Mathf.Abs(alpha) - 1f;
				bool flag3 = sp * (nz - 1f) >= 0f && Mathf.Abs(nCmd - 1f) < Mathf.Abs(nz - 1f) - 0.3f;
				unload = ((betaG < 0.5f) ? flag2 : flag3);
				if (nz > num5 || nz < num6 || alpha > num || alpha < num2 || num9 < 0.5f)
				{
					unload = false;
				}
				num10 = ((betaG < 0.5f) ? Mathf.Sign(alpha) : Mathf.Sign(nz - 1f));
				float num11 = alphaCmd - alpha;
				float num12 = Mathf.Sign(num11) * Mathf.Min(Plugin.KpAoA * Mathf.Abs(num11), Mathf.Sqrt(240f * Mathf.Abs(num11)));
				if (!gRate && !capOn && betaG < 0.5f && num9 > 0.5f && !flag2 && !hiRate && Mathf.Abs(num11) < 4f && Mathf.Abs(qT) < 0.95f * qB)
				{
					aInt = Mathf.Clamp(aInt + Plugin.KiAoA * num11 * dt, -10f, 10f);
				}
				else
				{
					aInt = Mathf.MoveTowards(aInt, 0f, 20f * dt);
				}
				float a2 = num8 + num12 + aInt;
				if (flag2)
				{
					float num13 = num8 + (NPath(alphaCmd) - NPath(alpha)) * num7;
					float num14 = Mathf.Max(0f, num10 * alphaDot);
					a2 = num13 + (0.15f + 0.45f * (1f - Mathf.Clamp01(Mathf.Abs(num11) / 10f))) * num11 - num10 * Plugin.UnloadRiseGain * num14;
				}
				rateW = 0f;
				if (hiRate)
				{
					// past the threshold: pitch-rate command. The map runs through (0, 0), the entry point (stick, rate at
					// the crossing) and (full stick, budget); a release drops the entry point (zero-rate hold, and the next
					// pull maps 0..full stick straight onto 0..budget).
					// a release or a full pull drops the entry point: from then on 0..full stick = 0..budget
					// 1.9.7: in side space (hiSide), so a push past the negative threshold is the same nose-down rate command
					float hs = hiSide;
					float spH = hs * sp;
					if (Mathf.Abs(sp) < 0.03f || spH > 0.98f)
					{
						hiAnchor = false;
					}
					float qMap;
					if (spH <= 0f)
					{
						qMap = spH * qB;
					}
					else if (hiAnchor && hiAS > 0.05f)
					{
						qMap = ((spH <= hiAS || hiAS > 0.999f) ? (hiAQ * spH / hiAS) : Mathf.Lerp(hiAQ, Mathf.Max(hiAQ, qB), (spH - hiAS) / (1f - hiAS)));
					}
					else
					{
						qMap = spH * qB;
					}
					qMap = Mathf.Min(qMap, qB);
					float aTh = ((hiSide > 0) ? aTP : aTN);
					rateW = Smooth01((hs * alpha - (aTh - Plugin.RateBlendBand)) / Plugin.RateBlendBand);
					a2 = Mathf.Lerp(a2, hs * qMap, rateW);
				}
				if (capOn)
				{
					a2 = capQ;
				}
				if (gRate)
				{
					// stick -> body pitch rate, 0..full stick = 0..the pitch-rate budget. Pull side keeps the pilot's stick
					// shape (section 5 curve, normalised to full stick), push side linear as on the AoA law. Released stick =
					// zero-rate hold (num9), as before.
					float spAbs = Mathf.Min(Mathf.Abs(sp), 1f);
					float frac = (sp >= 0f) ? (StickCurve(spAbs) / Mathf.Max(1f, StickCurve(1f))) : spAbs;
					rateCmdG = Mathf.Sign(sp) * Mathf.Clamp01(frac) * qB;
					// 2.1.7 stick lead (StickLeadGain, default 0 = off): a washed-out term on stick movement, so a quick small
					// input moves the nose a little at once (as an AoA step would) and then settles into the plain rate
					// response. Extra nose movement ~ gain x stick change x budget x tau; slow, smooth stick motion barely
					// triggers it; a held stick gets none.
					float sfr = Mathf.Sign(sp) * Mathf.Clamp01(frac);
					if (!leadInit)
					{
						stickLagF = sfr;
						leadInit = true;
					}
					float tauL = Mathf.Max(0.02f, Plugin.StickLeadTau);
					stickLagF += (sfr - stickLagF) * Mathf.Clamp01(dt / tauL);
					stickLeadDbg = Plugin.StickLeadGain * (sfr - stickLagF) * qB;
					a2 = rateCmdG + stickLeadDbg;
				}
				float num15 = Mathf.Max(nAlpha, 0.1f);
				float b;
				if (flag3)
				{
					float num16 = Mathf.Max(0f, num10 * alphaDot);
					b = (nCmd - eLy) * num7 + 0.3f * Plugin.KpG * (nCmd - nz) / num15 - num10 * Plugin.UnloadRiseGain * num16;
				}
				else
				{
					b = (nCmd - eLy) * num7 + Plugin.KpG * (nCmd - nz) / num15 + gInt + Mathf.Sign(nCmd - 1f) * alphaDotHoldG;
				}
				float num17 = Mathf.Lerp(a2, b, betaG);
				if (betaG > 0.5f && num9 > 0.5f && !unload && Mathf.Abs(qT) < 0.95f * qB && Mathf.Abs(nCmd - nz) < 1.5f)
				{
					gInt = Mathf.Clamp(gInt + Plugin.KiG * (nCmd - nz) / Mathf.Max(nAlpha, 0.1f) * dt, -10f, 10f);
				}
				else
				{
					gInt = Mathf.MoveTowards(gInt, 0f, 10f * dt);
				}
				qRaw = num17 * num9;
			}
			else
			{
				qRaw = sp * (isAI ? Mathf.Min(qB, AIStickRate()) : qB);
				hiRate = false;
				floatOn = false;
				shapeInit = false;
				leadInit = false;
				stickLeadDbg = 0f;
				rateW = 0f;
				zoneX = 0f;
				if (rotating && mainsOnGround && !Plugin.GroundRateLaw)
				{
					qRaw = sp * Mathf.Min(qB, Plugin.RotationRate);
				}
				gInt = 0f;
				aInt = 0f;
				capIUp = (capIDn = 0f);
			}
			float num18 = qB;
			float num19 = 0f - qB;
			// 2.0.3: the pitch-rate budget is IAS/T-W based and knows nothing about what can stop the rate. With little
			// nozzle thrust and the stabs near the end of their authority (idle, high AoA) it allowed rates the surfaces
			// could not stop or regulate. The AoA rate (body pitch rate less the flight-path rate) is now also held to what
			// the braking authority available a little further on can stop within AuthRateStopAngle: nose-down authority
			// (stabs full nose-down, flaperons, nozzles at the present thrust) AuthRateLook deg above the present AoA for
			// a nose-up rate, nose-up authority below it for a nose-down rate. Measured state, no schedule. Only while the
			// low-thrust AoA cap is in force (the nozzles cannot cover the nose-down need); otherwise the old budget.
			authRateUp = authRateDn = 999f;
			if (AuthRateLimit && flag && asymValid && tvc != null && tvc.M != null && tvc.N > 1 && (authFixedOn || authCapEff < Plugin.AoAPos - 0.5f))
			{
				float tvLo = float.MaxValue;
				float tvHi = float.MinValue;
				for (int tvJ = 0; tvJ < tvc.N; tvJ++)
				{
					tvLo = Mathf.Min(tvLo, tvc.M[tvJ].y);
					tvHi = Mathf.Max(tvHi, tvc.M[tvJ].y);
				}
				float tvLoA = (tvc.active ? (tvLo * IaInv[1, 1] * 57.29578f) : 0f);
				float tvHiA = (tvc.active ? (tvHi * IaInv[1, 1] * 57.29578f) : 0f);
				float brDnA = 0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, alpha + AuthRateLook) + tvLoA);
				float brUpA = AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, alpha - AuthRateLook) + tvHiA;
				authRateUp = Mathf.Max(AuthRateMin, Mathf.Sqrt(2f * Mathf.Max(0f, brDnA) * AuthRateStopAngle));
				authRateDn = Mathf.Max(AuthRateMin, Mathf.Sqrt(2f * Mathf.Max(0f, brUpA) * AuthRateStopAngle));
				if (authRateUp < num18 - num8)
				{
					num18 = Mathf.Max(num8 + authRateUp, Mathf.Min(num18, AuthRateMin));
				}
				if (authRateDn < num8 - num19)
				{
					num19 = Mathf.Min(num8 - authRateDn, Mathf.Max(num19, 0f - AuthRateMin));
				}
			}
			capAtLimU = capAtLimD = false;
			capPushU = (capPushD = 0f);
			if (!flag)
			{
				capIUp = (capIDn = 0f);
			}
			if (flag)
			{
				float num20 = num3;
				float num21 = num4;
				if (Mode == FcsMode.AoAG && !Plugin.GRateLaw && !capOn && !unload && num9 > 0.5f && betaG < 0.5f && !hiRate)
				{
					if (alphaCmdRaw > 5f)
					{
						num20 = Mathf.Min(num3, alphaCmdRaw + 0.2f);
					}
					if (alphaCmdRaw < -5f)
					{
						num21 = Mathf.Max(num4, alphaCmdRaw - 0.2f);
					}
				}
				float eCapU = (num20 - alpha) * 0.9f - Mathf.Max(0f, alphaDot) * 0.15f;
				float eCapD = (alpha - num21) * 0.9f - Mathf.Max(0f, 0f - alphaDot) * 0.15f;
				float num22 = Mathf.Max(0f, eCapU);
				float num23 = Mathf.Max(0f, eCapD);
				// 2.0.14: at the envelope limit itself (not the 0.2 deg shelf above an ordinary AoA command) the bound goes
				// on through the limit with the slope it has at zero (3 per unit = 2.7 deg/s per deg), and the integrator
				// trims what the flight-path-rate estimate misses. Only while the pilot asks for more than the limit.
				bool capAtU = num20 >= num3 - 0.01f && Mode == FcsMode.AoAG && num9 > 0.5f;
				bool capAtD = num21 <= num4 + 0.01f && Mode == FcsMode.AoAG && num9 > 0.5f;
				capAtLimU = capAtU && alpha > num20 - 2f;
				capAtLimD = capAtD && alpha < num21 + 2f;
				float capNegU = (capAtU && eCapU < 0f) ? (3f * eCapU) : 0f;
				float capNegD = (capAtD && eCapD < 0f) ? (3f * eCapD) : 0f;
				aBrakeTgt = aDn;
				if (CaptureMode == 1 && Plugin.AoACaptureThrustBrake && asymValid && tvc.M != null && tvc.N > 1)
				{
					float tvLo = float.MaxValue;
					float tvHi = float.MinValue;
					for (int tvJ = 0; tvJ < tvc.N; tvJ++)
					{
						tvLo = Mathf.Min(tvLo, tvc.M[tvJ].y);
						tvHi = Mathf.Max(tvHi, tvc.M[tvJ].y);
					}
					float tvLoA = (tvc.active ? (tvLo * IaInv[1, 1] * 57.29578f) : 0f);
					float tvHiA = (tvc.active ? (tvHi * IaInv[1, 1] * 57.29578f) : 0f);
					float brDn = 0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, num20) + tvLoA);
					float brUp = AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, num21) + tvHiA;
					// 2.0.3: floor 20 -> AuthBrakeFloor: with little authority the capture planned with braking it did not have
					aBrakeTgt = Mathf.Max(AuthBrakeFloor, Mathf.Min(aDn, brDn));
					float aBrakeUp = Mathf.Max(AuthBrakeFloor, Mathf.Min(aUp, brUp));
					aBrakeTgt += Mathf.Clamp01(Plugin.AoACaptureBrakeBlend) * Mathf.Max(0f, aDn - aBrakeTgt);
					aBrakeUp += Mathf.Clamp01(Plugin.AoACaptureBrakeBlend) * Mathf.Max(0f, aUp - aBrakeUp);
					// 2.1.10 rolling capture: in a roll the capture planned with braking the jet did not have - the stabs are
					// partly spent on the roll (aDnPitch) and the roll's inertial pitch-up, (Iyaw - Iroll) / Ipitch x p r, works
					// against the stop. At the 45 deg limit the AoA overshot 3-6 deg, protection cut the rate to zero, the AoA fell
					// 5-10 deg below and the cycle repeated every ~1.2 s (bench 520 km/h entry; 13 Oct log 92-95 s). While a
					// velocity-vector roll runs, the braking is taken as what is left for pitch less the coupling, and the
					// capture's near-limit gain comes down with it.
					float kU = 3f;
					float kD = 3f;
					if (RollCapture && Plugin.CoupGateAtLimitOn && Ia[1, 1] > 1f)
					{
						float saR = Mathf.Sin(alpha * (MathF.PI / 180f));
						float caR = Mathf.Cos(alpha * (MathF.PI / 180f));
						float pSr = Mathf.Abs((omega.x * caR + omega.z * saR) * 57.29578f);
						float lg = Mathf.Clamp01((pSr - 10f) / 20f);
						if (lg > 0f)
						{
							float coupA = (Ia[2, 2] - Ia[0, 0]) / Ia[1, 1] * omega.x * omega.z * 57.29578f;
							float bU = Mathf.Max(AuthBrakeFloor, Mathf.Min(aBrakeTgt, aDnPitch) - RollCaptureCoupK * Mathf.Max(0f, coupA));
							float bD = Mathf.Max(AuthBrakeFloor, Mathf.Min(aBrakeUp, aUpPitch) - RollCaptureCoupK * Mathf.Max(0f, 0f - coupA));
							bU = Mathf.Lerp(aBrakeTgt, Mathf.Min(aBrakeTgt, bU), lg);
							bD = Mathf.Lerp(aBrakeUp, Mathf.Min(aBrakeUp, bD), lg);
							kU = 3f * Mathf.Sqrt(bU / Mathf.Max(1f, aBrakeTgt));
							kD = 3f * Mathf.Sqrt(bD / Mathf.Max(1f, aBrakeUp));
							aBrakeTgt = bU;
							aBrakeUp = bD;
						}
					}
					// 2.2.1: the stop also takes the pitch effectors' lag (CapLagT, scaled by the nozzles' share of the authority:
					// they slew ~70 deg/s across their travel and carry most of it at low IAS). Stopping distance q tau + q^2 / 2a
					// instead of q^2 / 2a: a re-pull at ~200 km/h from 38 deg came in at 30 deg/s against a bound that assumed the
					// jet could stop at once, and overshot the 45 deg limit by 1.5 deg (13 Oct 10:46 log, 107 s; bench 46.4).
					float sqU = Mathf.Sqrt(1.4f * aBrakeTgt * num22);
					float sqD = Mathf.Sqrt(1.4f * aBrakeUp * num23);
					if (CapLagT > 0f && Plugin.CapLagOn)
					{
						float tauC = CapLagT * CapTvcShare();
						if (tauC > 0.001f)
						{
							float aU = 0.7f * Mathf.Max(1f, aBrakeTgt);
							float aD = 0.7f * Mathf.Max(1f, aBrakeUp);
							sqU = aU * (Mathf.Sqrt(tauC * tauC + 2f * num22 / aU) - tauC);
							sqD = aD * (Mathf.Sqrt(tauC * tauC + 2f * num23 / aD) - tauC);
						}
					}
					capBoundU = num8 + Mathf.Min(sqU, kU * num22);
					capBoundD = num8 - Mathf.Min(sqD, kD * num23);
				}
				else
				{
					capBoundU = num8 + Mathf.Min(Mathf.Sqrt(2f * aDn * num22), 3f * num22);
					capBoundD = num8 - Mathf.Min(Mathf.Sqrt(2f * aUp * num23), 3f * num23);
				}
				capBoundU += capNegU;
				capBoundD -= capNegD;
				// integrators: while the pilot's command is held back by this bound near the limit. Past the limit they
				// learn whenever the AoA rate is small. Short of it only after CapIHoldT of continuous hold within CapIBand:
				// the proportional capture converges from below, and learning on that approach banked error that carried
				// the jet over the limit on arrival. They keep their value through transients and bleed off otherwise.
				bool bindU = capAtU && Mathf.Abs(num20 - alpha) < 2f && qRaw >= capBoundU + capIUp - 0.5f;
				bool bindD = capAtD && Mathf.Abs(alpha - num21) < 2f && qRaw <= capBoundD + capIDn + 0.5f;
				bool capSettled = Mathf.Abs(alphaDot) < CapIAlphaDot;
				capHoldTU = (bindU && Mathf.Abs(num20 - alpha) < CapIBand) ? (capHoldTU + dt) : 0f;
				capHoldTD = (bindD && Mathf.Abs(alpha - num21) < CapIBand) ? (capHoldTD + dt) : 0f;
				if (bindU)
				{
					float eI = num20 - alpha;
					if (capSettled && capHoldTU > 0f && (eI < 0f || capHoldTU > CapIHoldT))
					{
						capIUp = Mathf.Clamp(capIUp + CapKi * eI * dt, 0f - CapIMax, CapIMax);
					}
				}
				else
				{
					capIUp = Mathf.MoveTowards(capIUp, 0f, 8f * dt);
				}
				if (bindD)
				{
					float eI = num21 - alpha;
					if (capSettled && capHoldTD > 0f && (eI > 0f || capHoldTD > CapIHoldT))
					{
						capIDn = Mathf.Clamp(capIDn + CapKi * eI * dt, 0f - CapIMax, CapIMax);
					}
				}
				else
				{
					capIDn = Mathf.MoveTowards(capIDn, 0f, 8f * dt);
				}
				capBoundU += capIUp;
				capBoundD += capIDn;
				capPushU = Mathf.Max(0f, num8 - capBoundU);
				capPushD = Mathf.Max(0f, capBoundD - num8);
				num18 = Mathf.Min(num18, capBoundU);
				num19 = Mathf.Max(num19, capBoundD);
			}
			if (tas > 25f)
			{
				float num24 = Mathf.Max(nAlpha, 0.02f);
				float num25 = ((nz > 1f) ? alphaDotHoldG : 0f);
				float num26 = ((nz < 0f) ? alphaDotHoldG : 0f);
				float num27 = 0.15f + RollLoadLead * Mathf.Clamp01(Mathf.Abs(omega.x * 57.29578f) / 150f);
				float num28 = (num5 - nz) / num24 - Mathf.Max(0f, alphaDot - num25) * num27;
				float num29 = (nz - num6) / num24 - Mathf.Max(0f, 0f - alphaDot - num26) * num27;
				float num30 = ((num28 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aDn * num28), 3f * num28) : (3f * num28));
				float num31 = ((num29 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aUp * num29), 3f * num29) : (3f * num29));
				// Ifrit 0.3.1 (GLimitAccelLead): the hold terms above cover a jet slowing down (the AoA has to keep rising to
				// hold the load); speeding up, the load grows at a fixed AoA, so the AoA has to come back toward zero by
				// 2 |n| vDot / (V n_alpha) just to stay at the limit. Without it a push held in an accelerating dive sat
				// 0.3-0.4 g past GLimitNegative (20 Oct 14:03 log 438-446 s: -3.8 g, -4.4 g peak; bench full push from
				// 360 km/h: -3.87 g steady).
				float accHold = GLimitAccelLead ? Mathf.Clamp(2f * Mathf.Abs(nz) * Mathf.Max(0f, vDot) / (Vc * num24), 0f, 15f) : 0f;
				num18 = Mathf.Min(num18, num8 + num25 + num30 - ((nz > 1f) ? accHold : 0f));
				num19 = Mathf.Max(num19, num8 - num26 - num31 + ((nz < 0f) ? accHold : 0f));
				// 1.9.0: only while the load is being taken off (the stick eased, or the shaped G command still coming down to
				// it). In a steady hold its 0.15 g dead zone pinned the load 0.15-0.2 g above the command once the hybrid
				// pitch loop removed the oscillation that used to hide it.
				bool gUnderGate = unload || nCmdRaw < nCmd - 0.05f || !Plugin.CmdShaping;
				if (Mode == FcsMode.AoAG && betaG > 0.5f && Plugin.GUndershootCapture > 0f && gUnderGate)
				{
					float num32 = (nz - nCmd - 0.15f) / num24;
					float num33 = ((num32 > 0f) ? Mathf.Min(Mathf.Sqrt(2f * aUp * num32), 3f * num32) : (3f * num32));
					if (num32 > 0f || !GUnderAboveOnly)
					{
						num19 = Mathf.Max(num19, num8 - num33 / Mathf.Clamp01(Plugin.GUndershootCapture));
					}
					gUnderDbg = num33;
				}
				else
				{
					gUnderDbg = 0f;
				}
			}
			if (protHold && (!flag || Mode != FcsMode.AoAG || Mathf.Abs(sp) < 0.7f || (protHoldSide > 0f && alphaU < num - 5f) || (protHoldSide < 0f && alphaU > num2 + 5f)))
			{
				protHold = false;
			}
			if (flag && Mode == FcsMode.AoAG && !protHold)
			{
				if (alphaU > num + 5f && sp > 0.85f)
				{
					protHold = true;
					protHoldSide = 1f;
				}
				else if (alphaU < num2 - 5f && sp < -0.85f)
				{
					protHold = true;
					protHoldSide = -1f;
				}
			}
			float num34;
			// 1.9.5: the pilot's own command within the normal bounds - the AoA protection may not hold it back from a
			// command that is already beyond protection's own (more nose-down above the limit, more nose-up below the
			// negative one). Before, protection set both bounds to its command, and from 1.9.3 (soft onset) that command is
			// ~the flight-path rate just past the limit: full forward stick at 65.5 deg still got +8..+12 deg/s nose-up.
			float qPilot = ((!(num19 > num18)) ? Mathf.Clamp(qRaw, num19, num18) : ((alpha >= 0f) ? num18 : num19));
			if (protHold)
			{
				num34 = 0f;
				num18 = (num19 = num34);
				prot = true;
			}
			else if (flag && alphaU > num + 0.5f)
			{
				// 1.9.3: continuous onset. The protection branch starts at alpha > limit + 0.5, where the capture branch commands
				// the flight-path rate; it used to subtract sqrt(2 a e) + 4 there at once (~14 deg/s), a step that pitched the
				// jet 3-4.5 deg below the limit in max-AoA turns (the 'pitch hitch'). Now 0 at the threshold, linear
				// (ProtOnsetGain /s per deg) until it meets the braking curve, and the +4 ramps in over 2 deg.
				float eP = alphaU - num - 0.5f;
				float num35 = Mathf.Min(Plugin.ProtRate, Mathf.Min(ProtOnsetGain * eP, Mathf.Sqrt(2f * Mathf.Max(aUp, 40f) * (alphaU - num))) + 4f * Mathf.Clamp01(eP / 2f));
				num35 = Mathf.Min(Plugin.ProtRate, Mathf.Max(num35, capPushU)); // 2.0.14: never weaker than the capture just below
				float num36 = Mathf.Max(num8 - num35, 0f - Plugin.ProtRate);
				float num37 = 1f - Mathf.Clamp01(sp / 0.9f);
				num34 = ((num36 < 0f) ? (num36 * num37) : num36);
				if (!StickCancelsLimit || qRaw < num34) num34 = Mathf.Min(num34, qPilot);
				num18 = (num19 = num34);
				// 1.9.3: the protection state (pitch priority in the allocator, fight costs off, ...) only past ProtFlagBand,
				// so a jet holding the limit in a turn does not toggle it every few frames
				prot = eP > ProtFlagBand;
			}
			else if (flag && alphaU < num2 - 0.5f)
			{
				float eN = num2 - alphaU - 0.5f;
				float num38 = Mathf.Min(Plugin.ProtRate, Mathf.Min(ProtOnsetGain * eN, Mathf.Sqrt(2f * Mathf.Max(aDn, 40f) * (num2 - alphaU))) + 4f * Mathf.Clamp01(eN / 2f));
				num38 = Mathf.Min(Plugin.ProtRate, Mathf.Max(num38, capPushD)); // 2.0.14
				float num39 = Mathf.Min(num8 + num38, Plugin.ProtRate);
				float num40 = 1f - Mathf.Clamp01((0f - sp) / 0.9f);
				num34 = ((num39 > 0f) ? (num39 * num40) : num39);
				if (!StickCancelsLimit || qRaw > num34) num34 = Mathf.Max(num34, qPilot);
				num18 = (num19 = num34);
				prot = eN > ProtFlagBand;
			}
			else
			{
				num34 = ((!(num19 > num18)) ? Mathf.Clamp(qRaw, num19, num18) : ((alpha >= 0f) ? num18 : num19));
				if (unload)
				{
					if (num10 > 0f)
					{
						num34 = Mathf.Max(num34, 0f);
					}
					else if (num10 < 0f)
					{
						num34 = Mathf.Min(num34, 0f);
					}
				}
				if (Mode == FcsMode.AoAG)
				{
					// 2.44.0 (StickCancelsLimit): at the AoA limit the limit's own nose-down (the capture bound) scales with 1 - stick / 0.9
					float flU = (StickCancelsLimit && capAtLimU) ? Mathf.Min(0f, num18) * (1f - Mathf.Clamp01(sp / 0.9f)) : 0f;
					float flD = (StickCancelsLimit && capAtLimD) ? Mathf.Max(0f, num19) * (1f - Mathf.Clamp01((0f - sp) / 0.9f)) : 0f;
					if (sp > 0.02f && alpha > -2f)
					{
						num34 = Mathf.Max(num34, (nz > num5) ? ((0f - Plugin.ProtRate) * (1f - sp)) : flU);
					}
					else if (sp < -0.02f && alpha < 2f)
					{
						num34 = Mathf.Min(num34, (nz < num6) ? (Plugin.ProtRate * (1f + sp)) : flD);
					}
				}
			}
			if (flag && !prot)
			{
				if (num3 < num - 0.05f && alphaU > num3)
				{
					float num41 = alphaU - num3;
					float num42 = Mathf.Min(Plugin.AsymRecoveryRate, Mathf.Min(3f * num41, Mathf.Sqrt(2f * Mathf.Max(aUp, 40f) * num41)));
					num34 = Mathf.Min(num34, Mathf.Max(num8 - num42, 0f - Plugin.ProtRate));
				}
				else if (num4 > num2 + 0.05f && alphaU < num4)
				{
					float num43 = num4 - alphaU;
					float num44 = Mathf.Min(Plugin.AsymRecoveryRate, Mathf.Min(3f * num43, Mathf.Sqrt(2f * Mathf.Max(aDn, 40f) * num43)));
					num34 = Mathf.Max(num34, Mathf.Min(num8 + num44, Plugin.ProtRate));
				}
			}
			// 2.1.8 AoA-sweep yaw limit. A velocity-vector roll pS at AoA a is coordinated when the stability-axis yaw rate
			// rS = -p sin a + r cos a is zero; the kinematic sideslip rate is -rS. Sweeping the AoA under a roll changes rS by
			// -pS * alphaDot, and only the yaw (r, through cos a) and roll (p, through sin a) accelerations can turn the rate
			// vector back. 12 Oct 23:00 log, 232-243 s: a 30 deg/s roll at 65 deg AoA, 57 m/s, left r = +25..29 deg/s; the full
			// push right after swept the AoA 65 -> -43 at 48 deg/s, the yaw (12-15 deg/s^2 modelled) could not reverse r, the
			// sideslip ran to -70 deg and the reversed dihedral at negative AoA drove the roll up to 43 deg/s against the
			// command. The AoA rate is held to what the lateral axes can follow; a sweep that brings rS back to zero is free.
			// Measured state only; off with no roll (|pS| under SweepPs0), beyond the AoA limits and for AI.
			pullGovTouched = false;
			pullTradeTgt = 999f;
			tradeAlphaNew = -1f;
			pullTradeHoldDbg = false;
			float qAskPre = num34;
			// 0.4.2: whether the pitch command is asking for a sweep at all. The FF roll trade's ask is the command's distance
			// from the measured flight-path rate (qAskPre - wVe); with the stick in the detent the command is *exactly* zero
			// (qRaw = sp x qB behind the num9 detent gate), so that distance becomes the measured rate itself and the trade
			// pays for a sweep nobody asked for. Rolling at 250 km/h through zero AoA with the pitch stick centred, the jet's
			// own settling sweeps the AoA 12 -> -2 deg at 2-7 deg/s, and the trade read that as a 2-7 deg/s ask: the roll-rate
			// limit swung 30 <-> 160 deg/s as it wandered - the roll "randomly speeding up and down" at low IAS. The Raptor
			// hides it: its rudders keep the capacity high enough that the same cut only moves the limit 106-127.
			// The pilot's stick (the law's own detent gate) and any bound that actually bound the command - protection, the
			// g / AoA-rate clamps, the unload - all count; a centred stick with nothing binding does not. A steady rolling
			// pull still trades (the stick is out of the detent), and the ask there is 0 anyway once the g has settled.
			// |sp| > 0.04 is the law's own detent (num9 = Clamp01((|sp| - 0.01) / 0.06) crosses 0.5 there), written on sp
			// directly so it reads the same under the rate law, where num9 stays 1
			bool qSweepAsk = Mathf.Abs(sp) > 0.04f || prot || protHold || Mathf.Abs(num34 - qRaw) > 0.05f;
			// 2.4.16: also with PitchRateCommandOnly (FcsMode.Rate). The sweep limit, the rolling-pull sideslip governor and
			// PullRollTrade only ever act on the pitch-rate command (num34) and the measured state, so they apply the same way
			// under the rate law; gated to AoAG they were simply off there (28-29 Oct logs, mode 6: sweepLim 0, slipGovPull 1,
			// pullTrade 0 in every rolling pull; full aft + full roll ran to 17-23 deg of sideslip at 150-340 kt)
			// 2.2.11 PullRollTrade, feed-forward part: with the trade on, the roll limit is also set from the AoA rate the
			// stick asks for, before the governor has to react: the roll whose coordinating yaw the yaw axis can build while
			// the AoA rises at that rate (the governor's own feed-forward, solved for pS). At high IAS the roll got to
			// 140-157 deg/s inside 0.4 s while the AoA rose at 50-60 deg/s; the governor caught it on the sideslip and
			// unloaded, and pitch and roll hitched together (19 Oct 10:29 log, 254 s at 295 kt: sideslip 14.7 deg, q 61 -> 5
			// -> 40, pS 138 -> 56 -> 129; 467 s at 345 kt the same).
			// 2.5.2 roll first, then pitch: the lateral governor's AoA-rise cap (yaw saturated with the roll already held; see
			// LateralLaw) holds the nose-up pitch-rate command to the AoA-hold rate plus that rise, with the 2.2.9 release lag
			// 2.5.3: both ways. The yaw the pair needs is |pS sin AoA|; any AoA sweep under a roll changes it at pS cos AoA x AoA
			// rate, nose-up or nose-down, at positive or negative AoA. A sweep that grows the demand is paid from the yaw left
			// on the roll's side (yawSatAlphaRate); one that shrinks it from the rudders' other side (yawSatAlphaRateDn). This is
			// the 2.1.8 sweep yaw limit (off from RollGovernorReplaces 2) from the same rule as the pull cap, so a full push +
			// full roll from positive AoA no longer sweeps the yaw demand through zero faster than the rudders can swing (bench,
			// 300 km/h: the roll outran the yaw at the zero crossing and the sideslip diverged).
			yawSatPitchDbg = 0f;
			float num34Ask = num34;
			bool planOwns = PushPlan && pushPlanOn && PushPlanOwnsPitch;
			if (yawSatAlphaRate > 0f && flag && !prot && !isAI && LatGovMode && !planOwns)
			{
				// nose-up pitch grows |AoA| at positive AoA and shrinks it at negative AoA
				float upRate = (alpha >= 0f) ? yawSatAlphaRate : yawSatAlphaRateDn;
				float dnRate = (alpha >= 0f) ? yawSatAlphaRateDn : yawSatAlphaRate;
				// 2.24.0 (SweepNoReverse), 2.31.1 written into the rates: a sweep toward zero AoA may be paced down to the flight-path
				// rotation, never below it (the nose is held, never turned back: the pitch-rate command the pace leaves is never on the
				// far side of zero). Held pushes too (SweepNoReverseHeld, 2.25.0); not under the no-fin nose-hold (2.19.0: holding the
				// AoA on a turning flight path IS a positive body rate; with the floor the no-fin held pushes departed on the bench).
				if (SweepNoReverse)
				{
					float wNr = (SweepNoReverseHeld ? 1f : stickRelW) * (1f - sweepHoldDbg);
					upRate = Mathf.Lerp(upRate, Mathf.Max(upRate, 0f - num8), wNr);
					dnRate = Mathf.Lerp(dnRate, Mathf.Max(dnRate, num8), wNr);
				}
				float capY = num8 + upRate;
				float capLo = num8 - dnRate;
				if (num34 > capY)
				{
					yawSatPitchDbg = num34 - capY;
					num34 = capY;
				}
				else if (num34 < capLo)
				{
					yawSatPitchDbg = num34 - capLo;
					num34 = capLo;
				}
				yawSatCapQ = capY;
				yawSatCapQLo = capLo;
				yawSatCapOn = true;
			}
			else if (planOwns)
			{
				// 2.36.0: the plan owns the nose-down pitch; the caps re-arm from open when it ends (no stale bound at the bottom)
				yawSatCapQ = 120f;
				yawSatCapQLo = -120f;
				yawSatCapOn = false;
			}
			else if (yawSatCapOn && !(PushPlan && pushPlanOn))
			{
				// let go over GovCapTau, not in one frame
				yawSatCapQ = Mathf.Lerp(yawSatCapQ, 120f, Mathf.Clamp01(dt / Mathf.Max(0.02f, GovCapTau)));
				yawSatCapQLo = Mathf.Lerp(yawSatCapQLo, -120f, Mathf.Clamp01(dt / Mathf.Max(0.02f, GovCapTau)));
				num34 = Mathf.Clamp(num34, yawSatCapQLo, yawSatCapQ);
				if (yawSatCapQ > 100f)
				{
					yawSatCapOn = false;
				}
			}
			// 2.35.0 (PushPlan): the planned push - the nose-down body rate is bounded at the plan's, one curve
			if (PushPlan && pushPlanOn && !prot && !isAI)
			{
				if (pushPlanSide > 0f) num34 = Mathf.Max(num34, 0f - pushPlanQNow);
				else num34 = Mathf.Min(num34, pushPlanQNow);
			}
			// 2.36.0 (PushPlanClipKey): "the sweep toward zero is held back" - the allocator's pitch priority (pitchNeedDn) and the stabs'
			// attached window (sweepHoldWin) read it next tick - is whatever bounds the push: the sweep caps, or the plan when it is on.
			// Until 2.35.3 only the caps' clip was read, so under a plan those two ran only because the 60 deg/s ceiling also clipped
			// the stick (a full push always does) - the ceiling could not stand aside without taking them with it (2.35.1: sideslip
			// median 4.8 -> 11.2 with PushPlanOwnsPitch).
			sweepClipDbg = yawSatPitchDbg;
			sweepClipOn = yawSatCapOn;
			if (PushPlanClipKey && PushPlan && pushPlanOn && !prot && !isAI)
			{
				sweepClipDbg = num34Ask - num34;
				sweepClipOn = true;
			}
			// 2.5.1 (RollGovernorReplaces >= 1): the trade is the rolling-pull protection on its own - the AoA-rise hold above is off, so
			// the trade no longer waits for it: no IAS fade (the hold's own reaction covered low IAS before), and its slow release
			// while the stick still asks (PullTradeRiseHold) is kept here
			// 2.6.0: level 3 only - the trade is the rolling-pull protection on its own, at every IAS and either AoA sign
			// 2.5.5: both ways and at either AoA sign. A nose-down sweep (push-over under a roll) or a pull from negative AoA moves
			// the yaw the roll needs just as a pull does, and the roll is what is traded first; the AoA-rate caps then only have the
			// roll that is left to contend with. 10:10 log, 268 / 315 s: full push + full roll from +9 deg at 104 / 68 m/s - no trade
			// (pullTrade 139-154 against pSmax 119), the sweep cap took 15-50 deg/s off the pitch-rate command instead, and the pace
			// window cut the roll a second later anyway.
			if (Plugin.PullRollTrade > 0f && PullTradeFFK > 0f && SweepPosPullFree && flag && !prot && !isAI && tas > 25f && LatGovMode)
			{
				float aAskF = qSweepAsk ? Mathf.Abs(qAskPre - num8) : 0f;
				pullTradeHoldDbg = aAskF > 5f;
				pullTradeAsk = aAskF;
				if (aAskF > 5f)
				{
					float caF = Mathf.Abs(Mathf.Cos(alpha * (MathF.PI / 180f)));
					// faded in with IAS (PullTradeFFIas0 -> +PullTradeFFIasSpan m/s, 180 -> 230 kt; 2.2.12, was 230 -> 280: 13 deg of sideslip at 240 kt in the 10:53 log): below that the governor's own reaction keeps up
					// with the roll build (your 0.7 / 9 setting held at 250 kt and below)
					float tgtF = 999f;
					// 2.8.0: the yaw the pair needs grows at pS cos AoA x AoA rate. Against the roll the stick asks (pSAskStick, before
					// any cap) and the measured capacity on the side the sweep needs (CapacityEstimator, last tick), TradeCapShare of it:
					// no shortfall, no trade - the pitch runs free whatever the roll limit is doing; a shortfall is split PullRollTrade to
					// the roll (its target drops until its share is paid) and the rest to the AoA rate. Nothing here reads the roll limit,
					// so a released stick (pSAskStick 0) asks nothing of the pitch.
					float kDem = caF * aAskF * (MathF.PI / 180f);
					float growM = ((qAskPre - num8) * Mathf.Sign(alpha) >= 0f) ? 1f : -1f;
					float rollRef = (Mathf.Abs(pTraj.x) > 2f) ? pTraj.x : paceTgtRaw;
					float sideM = growM * Mathf.Sign(rollRef * Mathf.Sin(alpha * (MathF.PI / 180f)));
					// 2.21.0: the low-passed read (TradeCapTau) - the trade
					// is the one accounting of the rolling pull's pitch share and must not dip with the rudders' servo transit (see CapacityEstimator)
					float capM = (sideM >= 0f) ? capEst.capTrZeP : capEst.capTrZeN;
					float capS = Mathf.Max(0f, TradeCapShare * capM);
					float demandM = pSAskStick * kDem;
					if (demandM > capS && kDem > 1E-04f)
					{
						float shortM = demandM - capS;
						float trM = (yawHealth >= 0.99f) ? Plugin.PullRollTrade : Mathf.Min(Plugin.PullRollTrade, PullRollTradeFinLoss);
						float pSNew = Mathf.Max(PullTradeFloor, pSAskStick - trM * shortM / kDem);
						float leftM = pSNew * kDem - capS;
						float aNew = (leftM > 0f && pSNew > 1f) ? (aAskF - leftM / (pSNew * caF * (MathF.PI / 180f))) : aAskF;
						tgtF = pSNew;
						tradeAlphaNew = Mathf.Max(StopSlipPitchFloor, aNew);
					}
					else
					{
						tgtF = 999f;
					}
					if (tgtF < pullTradeTgt)
					{
						float pSF = Mathf.Abs((omega.x * caF + omega.z * Mathf.Sin(alpha * (MathF.PI / 180f))) * 57.29578f);
						// it stops the roll building past the target, it does not cut a roll already established (cutting a
						// 70 deg/s roll at 35 deg AoA swung the sideslip 4 -> -10 deg on the bench); the governor's own trade does that
						// 2.2.25: below PullTradeCutA (fading out over the next 10 deg) it does cut an established roll: a pull
						// into a fast roll from low AoA otherwise ran the roll on until the governor caught the sideslip, then cut
						// the AoA rise and the roll together (20 Oct 01:41 log, 230-260 kt, rolling 175-210 deg/s: pitch rate
						// 35 -> 22-31 -> 48 deg/s, sideslip 6-8 deg). At low AoA a velocity-vector roll is nearly a body roll, so
						// slowing it does not swing the sideslip the way it did at 35 deg AoA.
						float cutW = (PullTradeCutA > 0f) ? Mathf.Clamp01((PullTradeCutA + 10f - Mathf.Abs(alpha)) / 10f) : 0f;
						tgtF = Mathf.Max(tgtF, Mathf.Lerp(Mathf.Min(pSF, pullTradePs), tgtF, cutW));
						if (pullTradePs > Mathf.Max(pSF, tgtF))
						{
							pullTradePs = Mathf.Max(pSF, tgtF);
						}
						pullTradeTgt = tgtF;
					}
				}
			}
			// 2.2.8: the gate recovers whenever it is not being updated (no roll, negative AoA, at a limit). It used to hold
			// its last value until the next rolling pull, which then started governed (19 Oct log, 30-36 s: 0.10 held through
			// 5 s of wings-level flight; the next rolling pull at 36.3 s was held to 4 deg/s of AoA rise at 1 deg of sideslip).
			if (!pullGovTouched)
			{
				pullGovF = Mathf.MoveTowards(pullGovF, 1f, PullGovIdleRise * dt);
				pullFFBoost = Mathf.MoveTowards(pullFFBoost, 1f, PullFFBoostIdle * dt);
				pullServoEx = 0f;
				slipGovPullDbg = 1f;
				// 2.2.20: when the governor stops acting (roll under SlipGovPs0, AoA through zero) its cap is let go over
				// GovCapTau instead of at once, so an unload does not snap back to the stick's 48 deg/s in one frame
			}
			pullTradePs = Mathf.MoveTowards(pullTradePs, pullTradeTgt, ((pullTradeTgt < pullTradePs) ? PullTradeFall : ((pullTradeHoldDbg && PullTradeRiseHold > 0f) ? PullTradeRiseHold : PullTradeRise)) * dt);
			if (pullTradeTgt >= 999f && pullTradePs > 400f)
			{
				pullTradePs = 999f;
			}
			qMaxDbg = num18;
			qMinDbg = num19;
			// 2.0.5: the final clamp used AoARecoveryRate (48) only, so MaxPitchRate above it did nothing
			float qAbsMax = Mathf.Max(Plugin.ProtRate, PitchRateCap);
			qCmd = Mathf.Clamp(num34, 0f - qAbsMax, qAbsMax);
			if (Mode == FcsMode.AoAG && !prot && Plugin.PitchAntiWindup > 0f && num9 > 0.5f)
			{
				float aw = Plugin.PitchAntiWindup * (qCmd - qRaw) * dt;
				if (aw * aInt < 0f)
				{
					aInt = ((aInt > 0f) ? Mathf.Max(0f, aInt + (1f - betaG) * aw) : Mathf.Min(0f, aInt + (1f - betaG) * aw));
				}
				if (aw * gInt < 0f)
				{
					gInt = ((gInt > 0f) ? Mathf.Max(0f, gInt + betaG * aw) : Mathf.Min(0f, gInt + betaG * aw));
				}
			}
			// Revoker 0.2.8: braked touchdown on the mains, nose still up: lower it at DerotationRate (see derotGate)
			if (derotGate && Mode == FcsMode.GearDownRate)
			{
				qCmd = Mathf.Min(qCmd, 0f - DerotationRate);
			}
			float num45 = qCmd - qTraj.x;
			bool pitchOnset = num45 * qTraj.x >= 0f;
			float a3 = Mathf.Max(0.7f * ((num45 > 0f) ? (pitchOnset ? aUpP : aUp) : (pitchOnset ? aDnP : aDn)), 30f);
			a3 = Mathf.Min(a3, Mathf.Max(30f, Plugin.PitchOnsetGps / Mathf.Max(0.05f, nAlpha)));
			// 1.9.0: stick released in the G regime: the pitch rate is taken off over ~ReleaseTauG instead of in one step, so
			// the load comes off at a steadier rate at high dynamic pressure (the zero-rate hold itself is unchanged)
			if (Mode == FcsMode.AoAG && Plugin.CmdShaping && !prot && betaG > 0.5f && Mathf.Abs(sp) < 0.07f && num45 * qTraj.x < 0f)
			{
				a3 = Mathf.Min(a3, Mathf.Max(3f, Mathf.Abs(qTraj.x) / ReleaseTauG));
			}
			float j = (prot ? (a3 / 0.1f) : (a3 / 0.15f));
			if (unload)
			{
				a3 = ((num10 * alphaDot > 0.5f) ? Mathf.Min(a3, Mathf.Max(60f, 2f * Plugin.UnloadDecel)) : Mathf.Min(a3, Plugin.UnloadDecel));
				j = a3 / 0.3f;
			}
			qTraj.Step(qCmd, a3, j, dt);
			// 2.0.6: nose wheel still on the runway in the rate law: the jet cannot pitch yet, so the rate trajectory is
			// kept just ahead of the measured rate (its acceleration still asks for the rotation). When the nose comes up
			// the command starts from the jet's own rate, as it would in the air, instead of a step to the full command.
			if (noseContactLaw && qTraj.x > 0f)
			{
				float qMeas = omega.y * 57.29578f;
				qTraj.x = Mathf.Min(qTraj.x, Mathf.Max(qMeas, 0f) + Mathf.Min(Mathf.Max(qTraj.v, 0f) * RotLeadT, RotLeadMax));
			}
			qT = qTraj.x;
		}

		internal static float LowIasYawBoostAt(float iasNow)
		{
			if (YawBoostFixed >= 1f)
			{
				return YawBoostFixed;
			}
			float num = Mathf.Max(1f, Plugin.LowIasYawBoost) - 1f;
			float lowIasYawBoostFullIas = Plugin.LowIasYawBoostFullIas;
			float num2 = ((iasNow <= lowIasYawBoostFullIas) ? 1f : ((float)Math.Exp((0f - (iasNow - lowIasYawBoostFullIas)) / Mathf.Max(1f, Plugin.LowIasYawBoostFalloff))));
			num2 *= Mathf.Clamp01((120f - iasNow) / 10f);
			return 1f + num * num2;
		}

		internal static float FightPedalRelease = 1f;

		internal static int FightMode = 0;

		internal static bool RelQuietHold = true;

		internal static float RelQuietErr0 = 2f;

		internal static float RelQuietErrSpan = 4f;


		internal static float PitchFightQdot = 25f;

		internal static float PitchFightQHold = 0.6f;

		private float pfQHold;

		public static float PitchFightErr0 = 3f;

		public static bool PitchFightAbsErr = false;

		public static float PitchFightErrSpan = 5f;

		internal static float FightRollRelease = 0.3f;

		private float fightRollHold;

		private float fightPed;

		// 1.8.41 pitch-reserve roll governor. Rolling at a held or restricted AoA makes pitch-up: the canted/stalled
		// down-going stab and flaperon lose lift for roll, and the inertial coupling (Izz-Ixx)/Iyy * p * r grows with the
		// roll rate squared. Above the roll-pitch-priority band (18-26 deg AoA) only the nozzles are left to hold it, and with
		// a store they sat on the -20 deg stop for whole rolls (2026-09-28 log: 28 deg cap, full aft + roll, nozzles pinned
		// 4 s). The governor watches the nozzles' remaining nose-down travel (nose-up at negative AoA) at the live thrust and,
		// while a roll is commanded, lowers the roll-rate limit and entry acceleration when less than RollPitchReserve of
		// their nose-down authority is left; it gives it back as the margin returns. Measured state only - no AoA/speed
		// schedule. 0 = off (1.8.40).
		internal static float RollGovDown = 1.5f;

		internal static float RollGovUp = 0.6f;

		internal static float RollGovFloor = 0.35f;

		internal static float RollGovMinRate = 30f;

		internal static float RollGovAccelFloor = 0.5f;

		internal static float RollGovAoA0 = 3f;

		public float rollGov = 1f;

		public float rollGovMargin;

		private Lpf rollGovMF;

		private Lpf govMarginRateF;

		private float rollGovMarginPrev = -1f;

		public float rollGovPredDbg = 1f;

		internal static bool GovPredictive = true;

		internal static float GovLookT = 0.3f;

		internal static float GovNeedHi = 0.2f;

		internal static float GovDownP = 0.8f;

		internal static float GovUpP = 1.0f;

		private void RollGovStep(float dt, float pLim, float rollSide)
		{
			float res = Plugin.RollPitchReserve;
			if (res <= 0f || tvc == null || !tvc.active || tvc.N < 2 || tvc.M == null || !Engaged || Mode == FcsMode.Ground || Mode == FcsMode.Off || revFlow)
			{
				rollGov = Mathf.MoveTowards(rollGov, 1f, 2f * dt);
				rollGovMargin = 1f;
				slatRelief = Mathf.MoveTowards(slatRelief, 0f, SlatReliefOut * dt);
				return;
			}
			float yLo = float.MaxValue;
			float yHi = float.MinValue;
			for (int k = 0; k < tvc.N; k++)
			{
				yLo = Mathf.Min(yLo, tvc.M[k].y);
				yHi = Mathf.Max(yHi, tvc.M[k].y);
			}
			float yCmd = tvc.MAt(tvc.cmd).y;
			float yMid = tvc.MAt(0f).y;
			// margin toward the side the roll's pitch moment pushes against: nose-down at positive AoA
			float half = (alpha >= 0f) ? (yMid - yLo) : (yHi - yMid);
			float marg = (alpha >= 0f) ? (yCmd - yLo) : (yHi - yCmd);
			float frac = (half > 1E-06f) ? Mathf.Clamp01(marg / half) : 1f;
			// 2.1.3: at the low-thrust AoA cap the nozzles carry little of the nose-down authority, so "nozzles pinned" no
			// longer means "no pitch reserve" (the stabs and the airframe hold it). The margin counts only by the nozzles'
			// share of the nose-down authority there; elsewhere unchanged. Without it the governor sat at its floor in every
			// idle-cap roll and jumped back whenever a pitch transient unpinned the nozzles (12 Oct log, 657 / 713 s).
			if (LowCapLatFix && Plugin.LowCapRollFix && LowThrustCapOn && alpha > 0f)
			{
				float tvShare = Mathf.Clamp01(half * IaInv[1, 1] * 57.29578f / Mathf.Max(20f, aDn));
				float nearCap = Mathf.Clamp01((alpha - (authCapEff - LowCapCoupBand)) / Mathf.Max(0.5f, LowCapCoupBand));
				frac = Mathf.Lerp(frac, Mathf.Lerp(1f, frac, tvShare), nearCap);
			}
			frac = rollGovMF.Step(frac, 0.06f, dt);
			// 2.1.5 predictive governor: the margin's rate of change projected GovLookT ahead. The old governor cut only once
			// the nozzles were already pinned (margin < 15 %) and then recovered at 0.6/s: a cut-and-restore of the roll target
			// (12 Oct 18:50 log: 120 -> 42 -> 85 deg/s at 96 s, 120 -> 74 -> 86 at 202 s, clean jet 347 s). Now it eases off
			// early and in proportion while the margin is heading for the stop, and comes back only when the margin is there and
			// no longer falling.
			float fracRate = govMarginRateF.Step(rollGovMarginPrev >= 0f ? (frac - rollGovMarginPrev) / Mathf.Max(dt, 1E-4f) : 0f, 0.1f, dt);
			rollGovMarginPrev = frac;
			float fracPred = Mathf.Clamp01(frac + Mathf.Min(0f, fracRate) * (Plugin.LoadedRollSteadyOn ? 0f : GovLookT));
			rollGovPredDbg = fracPred;
			float aoaGate = Mathf.Clamp01((Mathf.Abs(alpha) - RollGovAoA0) / 3f);
			float latNow = Mathf.Max(Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f), Mathf.Clamp01((Mathf.Abs(pTraj.x) - 10f) / 20f));
			// 2.1.9 slat roll relief. The slats (Aryx's high-lift devices) are, in this game, only wing area on WingRear_L/R
			// whose centre of lift sits 1.34 m ahead of the CG, and the flap schedule has them fully out from 20 deg AoA. Out,
			// they push the nose up: retracting them gives 8-11 deg/s^2 nose-down at 300 km/h, 14-20 at 400, 27-39 at 550
			// (20-40 deg AoA), for ~7 % of the lift. While a roll is running at positive AoA and the nozzles' nose-down margin
			// heads toward the governor's threshold, the slats come in before the governor has to cut the roll; they go back
			// out once the margin is back or the roll is over. Measured state only (same margin the governor reads).
			SlatReliefStep(dt, fracPred, frac, aoaGate * latNow);
			if (latNow * aoaGate < 0.01f)
			{
				rollGov = Mathf.MoveTowards(rollGov, 1f, 2f * dt);
				return;
			}
			float need = Mathf.Clamp01(res);
			if (GovPredictive && Plugin.RollGovPredictiveOn)
			{
				float needHi = Mathf.Max(need + 0.05f, GovNeedHi);
				if (fracPred < needHi)
				{
					float deficitP = (needHi - fracPred) / needHi;
					rollGov = Mathf.Max(RollGovFloor, rollGov - GovDownP * deficitP * aoaGate * latNow * dt);
				}
				else if (frac >= need && fracRate > -0.05f)
				{
					rollGov = Mathf.Min(1f, rollGov + GovUpP * Mathf.Clamp01((frac - need) / Mathf.Max(0.05f, need)) * dt);
				}
				return;
			}
			if (frac < need)
			{
				float deficit = (need - frac) / Mathf.Max(0.05f, need);
				rollGov = Mathf.Max(RollGovFloor, rollGov - RollGovDown * deficit * aoaGate * latNow * dt);
			}
			else
			{
				rollGov = Mathf.Min(1f, rollGov + RollGovUp * Mathf.Clamp01((frac - need) / Mathf.Max(0.05f, need)) * dt);
			}
		}

		public float slatRelief;

		internal static float SlatReliefStart = 0.5f;

		internal static float SlatReliefFull = 0.2f;

		internal static float SlatReliefIn = 4f;

		internal static float SlatReliefOut = 1f;

		internal static float SlatReliefAoA0 = 12f;

		internal static float SlatReliefAoASpan = 6f;

		internal static float SlatReliefAoAHi = 35f;

		internal static float SlatReliefAoAHiSpan = 10f;

		private void SlatReliefStep(float dt, float fracPred, float frac, float rollGate)
		{
			if (!Plugin.SlatRollReliefOn || alpha <= 0f || isAI)
			{
				slatRelief = Mathf.MoveTowards(slatRelief, 0f, SlatReliefOut * dt);
				return;
			}
			float aG = Mathf.Clamp01((alpha - SlatReliefAoA0) / Mathf.Max(1f, SlatReliefAoASpan));
			aG *= 1f - Mathf.Clamp01((alpha - SlatReliefAoAHi) / Mathf.Max(1f, SlatReliefAoAHiSpan));
			float g = aG * Mathf.Clamp01(rollGate);
			// demand from the predicted margin: 0 at SlatReliefStart, 1 at SlatReliefFull (the governor's own threshold)
			float want = g * Mathf.Clamp01((SlatReliefStart - fracPred) / Mathf.Max(0.05f, SlatReliefStart - SlatReliefFull));
			if (want > slatRelief)
			{
				slatRelief = Mathf.MoveTowards(slatRelief, want, SlatReliefIn * dt);
			}
			else if (g < 0.01f || frac > SlatReliefStart)
			{
				// roll over, or the margin is back above the start: out again (no chasing the margin it creates while rolling)
				slatRelief = Mathf.MoveTowards(slatRelief, want, SlatReliefOut * dt);
			}
		}

		internal static bool DamageLatLimit = true;
		internal static float FinLossPedalSideslip = 5f;
		internal static float FinLossPedalIas = 600f;

		internal static bool YawTargetClamp = true;

		internal static bool GroundRollP = true;

		public bool wheelsTouch;

		internal static bool AsymThrustSplit = true;

		internal static bool EngineOutFixedCap = true;

		// 2.0.12: with an engine out the asymmetric-load cap comes down this many times faster (AsymCapTightenRate)
		internal static float EngOutTightenK = 4f;

		internal static float YawTargetMargin = 1f;

		// 2.20.0: the yaw-target clamp's reach includes the stabilators' yaw range while they are off the pitch
		// (stabPitchFrac below YawClampStabFree, scaled to 0 at it)
		internal static bool YawClampStabReach = true;

		internal static float YawClampStabFree = 0.25f;

		public float yawClampStabDbg;

		public bool engOut;

		// Ifrit 0.4.1: one engine running weak (damaged, not destroyed: under EngineAsymEnter of the strongest one's thrust,
		// released above EngineAsymExit; the ratio is filtered over 1 s so spool-up transients do not trip it)
		public bool engAsym;

		// 0.4.1: weaker / stronger engine's thrust at full throttle, predicted from their damage state (1 = even)
		public float engAbRatio = 1f;

		// one engine can still light its afterburner and another cannot (predicted from the damage state)
		public bool engAbMismatch;

		// 0.4.1 thrust match: the parasitic loss the FCS is holding on the stronger engine(s), N (0 = none), for the telemetry
		public float engMatchLoss;

		private bool matchApplied;

		private struct EngModel
		{
			public float rMin;

			public float rMax;

			public float ceil;

			public float remapX;

			public float remapY;

			public float t0;

			public float ab;

			public float abStart;

			public float abEnd;
		}

		private readonly EngModel[] engModel = new EngModel[4];

		private readonly float[] engUnmatched = new float[4];

		// predicted thrust of engine i at throttle input x (the game's Turbojet model, see the engine scan)
		private float EngThrustAt(int i, float x)
		{
			ref EngModel em = ref engModel[i];
			float tr = (em.remapY > em.remapX + 0.001f) ? Mathf.Clamp01((x - em.remapX) / (em.remapY - em.remapX)) : Mathf.Clamp01(x);
			float rpm = Mathf.Lerp(em.rMin, em.ceil, tr);
			float dry = (em.rMax > em.rMin) ? Mathf.Clamp01((rpm - 0.85f * em.rMin) / (em.rMax - em.rMin)) : tr;
			float th = em.t0 * dry;
			if (em.ab > 0f && em.rMax > 0f && rpm / em.rMax > 0.85f && x > em.abStart)
			{
				th += em.ab * Mathf.Clamp01((x - em.abStart) / Mathf.Max(0.01f, em.abEnd - em.abStart));
			}
			return th;
		}

		// weaker / stronger predicted thrust at throttle input x (1 when both are at idle or there is one engine)
		private float EngRatioAt(int n, float x)
		{
			float lo = float.MaxValue;
			float hi = 0f;
			for (int i = 0; i < n; i++)
			{
				float th = EngThrustAt(i, x);
				lo = Mathf.Min(lo, th);
				hi = Mathf.Max(hi, th);
			}
			return (hi > 1000f) ? Mathf.Clamp01(lo / hi) : 1f;
		}

		// the throttle input where the afterburner starts (1 without one)
		public float engAbStart = 1f;

		// the afterburner gate is holding the throttle at the afterburner's start
		public bool abGateOn;

		private bool engFixedPrev;

		// 0.4.1 / 2.4.1 thrust match on prediction: the yaw the thrust split WOULD make without the match (deg/s^2, from the
		// engines' RPMs and the game's thrust law) against the usable yaw authority at this AoA and speed; the match comes
		// in above MatchYawOn and lets go under MatchYawOff, so a split the fins cannot hold is never allowed to form
		public float splitYawPred;

		public float splitUsage;

		internal static float MatchYawOn = 0.8f;

		internal static float MatchYawOff = 0.5f;

		private bool matchYawOn;

		// 0.4.1b / 2.4.1b: the match is proportional. On the way up to the cap it comes in ahead of the AoA (the AoA
		// predicted MatchLead s ahead from its rate, over the MatchBand deg under the cap), so the healthy engine is already
		// down to the damaged one's thrust when the cap is reached and the cap lifts without a step; stick released with
		// the AoA under the cap, it eases back out (unless the fins could not hold the split: splitUsage / the VMC latch).
		public float matchF;

		public bool matchFull;

		internal static float MatchBand = 4f;

		internal static float MatchLead = 0.3f;

		// the match is complete this long before the AoA would reach the cap at its present rate
		internal static float MatchLeadTime = 0.8f;

		internal static float MatchRiseRate = 4f;

		internal static float MatchFallRate = 1.5f;

		// the VMC protection with a weak (running) engine: the match, latched until the sideslip is calm, instead of the throttle
		private bool vmcMatch;

		private float vmcMatchCalmT;

		// after leaving the fixed-cap state the thrust term stays out of the sweep for this long (the first sweep after the
		// match engages still sees the split settling: 23 Oct 00:00 log, cap 12 for one cycle)
		private float tKHoldT;

		// IAS at the last asymmetric-load sweep: the stored yaw authority scales with q from there
		private float asymSweepIas = 1f;

		public float engRatioF = 1f;

		internal static float EngineAsymEnter = 0.85f;

		internal static float EngineAsymExit = 0.92f;

		private int engTotal = -1;

		internal static float DamageLatFloor = 0.35f;

		internal static float DamageLatFloorNoTvc = 0.15f;

		public float yawHealth = 1f;

		public float zLoDbg, zHiDbg, zTgtDbg;

		private int rudTotal = -1;

		// 2.0.12: share of the rudders still on the jet (a rudder goes with its fin)
		private void YawHealthStep()
		{
			if (rudTotal < 0)
			{
				rudTotal = 0;
				foreach (SurfaceEffector s0 in surfaces)
				{
					if (s0.kind == SurfaceKind.Rudder)
					{
						rudTotal++;
					}
				}
			}
			if (engTotal < 0)
			{
				engTotal = tvc.engines.Count;
			}
			// an engine counts as out when destroyed / inoperable or when it makes under 15 % of the strongest one's
			// thrust while that one makes over 20 kN (flame-out, damage)
			int engOk = 0;
			float tMax = 0f;
			float tMin = float.MaxValue;
			// 0.4.1: each engine's thrust-vs-throttle from its damage state (Turbojet.FixedUpdate): the RPM it can reach is
			// minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5); at a throttle input x the target RPM is
			// Lerp(minRPM, that ceiling, (x - remap.x) / (remap.y - remap.x)); dry thrust = (RPM - 0.85 minRPM) / (maxRPM - minRPM)
			// of the maximum; the afterburner runs only above 85 % of maxRPM (condition > 0.55 on a 1000-3000 RPM engine) and
			// fades in from its start throttle to its end. From that: the weaker / stronger ratio at full throttle
			// (engAbRatio) and whether one can light its afterburner while another cannot (engAbMismatch).
			int nE = 0;
			engAbStart = 1f;
			foreach (Turbojet eg in tvc.engines)
			{
				float tE = 0f;
				if (eg != null && Acc.TF_operable(eg))
				{
					engOk++;
					if (nE < engModel.Length)
					{
						ref EngModel em = ref engModel[nE];
						em.rMin = Acc.TF_minRPM(eg);
						em.rMax = Acc.TF_maxRPM(eg);
						em.ceil = em.rMin + (em.rMax - em.rMin) * (0.5f * Mathf.Clamp01(Acc.TF_condition(eg)) + 0.5f);
						Vector2 rm = Acc.TF_remap(eg);
						em.remapX = rm.x;
						em.remapY = rm.y;
						em.t0 = Mathf.Max(0f, Acc.TF_staticThrust(eg));
						em.ab = 0f;
						em.abStart = 1f;
						em.abEnd = 1f;
						nE++;
					}
					JetNozzle[] nzE = Acc.TF_nozzles(eg);
					if (nzE != null)
					{
						foreach (JetNozzle jE in nzE)
						{
							if (jE != null)
							{
								tE += Mathf.Max(0f, Acc.JN_totalThrust(jE));
								if (nE > 0 && nE <= engModel.Length)
								{
									ref EngModel em2 = ref engModel[nE - 1];
									em2.ab += Mathf.Max(0f, Acc.JN_abMax(jE));
									em2.abStart = Mathf.Min(em2.abStart, Acc.JN_abStart(jE));
									em2.abEnd = Mathf.Min(em2.abEnd, Acc.JN_abEnd(jE));
								}
								engAbStart = Mathf.Min(engAbStart, Acc.JN_abStart(jE));
							}
						}
					}
				}
				tMax = Mathf.Max(tMax, tE);
				tMin = Mathf.Min(tMin, tE);
			}
			engAbRatio = 1f;
			engAbMismatch = false;
			if (engTotal > 1 && nE >= 2)
			{
				engAbRatio = EngRatioAt(nE, 1f);
				bool anyAb = false;
				bool allAb = true;
				for (int ie = 0; ie < nE; ie++)
				{
					ref EngModel emc = ref engModel[ie];
					bool ok = emc.ab > 0f && emc.rMax > 0f && emc.ceil / emc.rMax > 0.85f;
					anyAb |= ok;
					allAb &= ok;
				}
				engAbMismatch = anyAb && !allAb;
			}
			// the yaw the thrust split would make right now without the match: each engine's dry thrust from its RPM, plus
			// the afterburner it would have at the pilot's throttle (rawThrottle, before the match holds it under the
			// afterburner's start), less the weakest one's, on its nozzles' arms.
			// 0.4.3: the afterburner was left out ("equal on both when both have it, and the throttle is held under it
			// when only one has") - but the throttle is only held while the match is in. 23 Oct 00:51 log, 62-70 s: vertical
			// at 35-42 m/s, the match released as the AoA fell under the cap, the healthy engine lit its afterburner and
			// the split was 7.1 deg/s^2 against 6-7 of yaw authority; the prediction without the 37 kN of afterburner
			// said 3.8, splitUsage peaked at 0.80 (the match latches above 0.8), and the nose yawed 0.3 -> 9.5 deg/s in 8 s
			// with the stick centred. With it the prediction is 1.5 and the match holds.
			splitYawPred = 0f;
			splitUsage = 0f;
			if (engTotal > 1 && engOk == engTotal && nE >= 2)
			{
				float tgt = float.MaxValue;
				int k = 0;
				foreach (Turbojet eg in tvc.engines)
				{
					if (eg != null && Acc.TF_operable(eg) && k < nE)
					{
						ref EngModel emk = ref engModel[k];
						float ratio = (emk.rMax > emk.rMin) ? Mathf.Clamp01((Acc.TF_rpm(eg) - 0.85f * emk.rMin) / (emk.rMax - emk.rMin)) : 1f;
						engUnmatched[k] = emk.t0 * ratio;
						if (emk.ab > 0f && emk.rMax > 0f && emk.ceil / emk.rMax > 0.85f && rawThrottle > emk.abStart)
						{
							engUnmatched[k] += emk.ab * Mathf.Clamp01((rawThrottle - emk.abStart) / Mathf.Max(0.01f, emk.abEnd - emk.abStart));
						}
						tgt = Mathf.Min(tgt, engUnmatched[k]);
						k++;
					}
				}
				Vector3 mW = Vector3.zero;
				k = 0;
				foreach (Turbojet eg in tvc.engines)
				{
					if (eg != null && Acc.TF_operable(eg) && k < nE)
					{
						float extra = engUnmatched[k] - tgt;
						JetNozzle[] nzS = Acc.TF_nozzles(eg);
						if (extra > 1f && nzS != null && nzS.Length > 0)
						{
							float per = extra / nzS.Length;
							foreach (JetNozzle jS in nzS)
							{
								Transform tS = (jS != null) ? Acc.JN_thrustT(jS) : null;
								if (tS != null)
								{
									mW += Vector3.Cross(tS.position - cg, tS.forward * per);
								}
							}
						}
						k++;
					}
				}
				Quaternion rInv = Quaternion.Inverse(((Component)(object)ac).transform.rotation);
				Vector3 mA = rInv * mW;
				splitYawPred = Mathf.Abs(AccYaw(new Vector3(0f - mA.z, 0f - mA.x, mA.y)));
				float qScale = Mathf.Clamp((ias * ias) / (asymSweepIas * asymSweepIas), 0.05f, 2f);
				float cyNow = asymValid ? (asymCyNow * qScale) : 0f;
				splitUsage = (cyNow > 1f) ? (splitYawPred / Mathf.Max(1f, AsymResMaxFrac * cyNow)) : ((splitYawPred > 1f) ? 9f : 0f);
			}
			engOut = engTotal > 1 && engOk > 0 && (engOk < engTotal || (tMax > 20000f && tMin < 0.15f * tMax));
			float ratioNow = (engTotal > 1 && engOk == engTotal && tMax > 20000f) ? Mathf.Clamp01(tMin / tMax) : 1f;
			engRatioF += (ratioNow - engRatioF) * Mathf.Clamp01(Time.fixedDeltaTime / 1f);
			// the asymmetric state is the MEASURED thrust split (filtered 1 s, with hysteresis), and only while the split is
			// allowed: with the thrust-match limiter holding the throttle (AoA above the damaged-engine cap) the thrust is
			// symmetric and there is nothing to cap for - the pilot keeps the full AoA until he brings the AoA down himself
			engAsym = !engOut && engTotal > 1 && engOk == engTotal && tMax > 20000f && !matchFull && (engAsym ? (engRatioF < EngineAsymExit) : (engRatioF < EngineAsymEnter));
			if (rudTotal <= 0)
			{
				yawHealth = 1f;
				return;
			}
			int act = 0;
			int any = 0;
			foreach (SurfaceEffector s1 in surfaces)
			{
				if (s1.active)
				{
					any++;
					if (s1.kind == SurfaceKind.Rudder)
					{
						act++;
					}
				}
			}
			yawHealth = (any == 0) ? 1f : ((float)act / (float)rudTotal);
			// Revoker 0.1.0: the fin can go without the rudder (vstab2, the upper fin, carries no surface): the share of the
			// vertical-tail area still attached counts too
			if (finAreaTotal > 0.1f)
			{
				float aF = 0f;
				foreach (AeroPart pF in finParts)
				{
					if (pF != null && !pF.IsDetached())
					{
						aF += Mathf.Max(0f, Acc.AP_wingArea(pF));
					}
				}
				yawHealth = Mathf.Min(yawHealth, Mathf.Clamp01(aF / finAreaTotal));
			}
		}

		// Revoker 0.1.0: vertical tail parts (fins and rudder) and their total area at spawn
		private readonly List<AeroPart> finParts = new List<AeroPart>();

		private float finAreaTotal;

		// ---- 2.0.12 pitch stick curve: monotone cubic (Fritsch-Carlson) through (0,0), (s24,24), (s36,36), straight on
		// past 36 with the end slope -----------------------------------------------------------------------------------
		private readonly float[] scX = new float[3];

		private readonly float[] scY = new float[3] { 0f, 24f, 36f };

		private readonly float[] scM = new float[3];

		private float scK24 = -1f;

		private float scK36 = -1f;

		private void StickCurveSetup()
		{
			float a = Plugin.StickAoA24;
			float b = Plugin.StickAoA36;
			if (a == scK24 && b == scK36)
			{
				return;
			}
			scK24 = a;
			scK36 = b;
			scX[0] = 0f;
			scX[1] = a;
			scX[2] = b;
			float d0 = (scY[1] - scY[0]) / (scX[1] - scX[0]);
			float d1 = (scY[2] - scY[1]) / (scX[2] - scX[1]);
			// interior tangent: weighted harmonic mean (monotone, no overshoot); ends: one-sided, kept inside 3x the secant
			scM[1] = (d0 * d1 > 0f) ? (3f * (scX[2] - scX[0]) / ((2f * scX[2] - scX[1] - scX[0]) / d0 + (scX[2] + scX[1] - 2f * scX[0]) / d1)) : 0f;
			scM[0] = Mathf.Clamp(((2f * (scX[1] - scX[0]) + (scX[2] - scX[1])) * d0 - (scX[1] - scX[0]) * d1) / (scX[2] - scX[0]), 0.2f * d0, 3f * d0);
			scM[2] = Mathf.Clamp(((2f * (scX[2] - scX[1]) + (scX[1] - scX[0])) * d1 - (scX[2] - scX[1]) * d0) / (scX[2] - scX[0]), 0.2f * d1, 3f * d1);
		}

		internal float StickCurve(float s)
		{
			if (s <= 0f)
			{
				return 0f;
			}
			if (s >= scX[2])
			{
				return scY[2] + scM[2] * (s - scX[2]);
			}
			int k = (s < scX[1]) ? 0 : 1;
			float h = scX[k + 1] - scX[k];
			float t = (s - scX[k]) / h;
			float t2 = t * t;
			float t3 = t2 * t;
			return (2f * t3 - 3f * t2 + 1f) * scY[k] + (t3 - 2f * t2 + t) * h * scM[k] + (-2f * t3 + 3f * t2) * scY[k + 1] + (t3 - t2) * h * scM[k + 1];
		}

		internal float StickCurveInv(float aoa)
		{
			if (aoa <= 0f)
			{
				return 0f;
			}
			if (aoa >= scY[2])
			{
				return scX[2] + (aoa - scY[2]) / Mathf.Max(1f, scM[2]);
			}
			float lo = 0f;
			float hi = scX[2];
			for (int i = 0; i < 30; i++)
			{
				float mid = 0.5f * (lo + hi);
				if (StickCurve(mid) < aoa)
				{
					lo = mid;
				}
				else
				{
					hi = mid;
				}
			}
			return 0.5f * (lo + hi);
		}

		// ---- 2.0.12 VMC protection: last resort only -------------------------------------------------------------------
		// With an engine out the live engine's thrust yaws the jet; the rudders hold it down to the speed where their
		// authority (falling with q) equals that yaw (afterburner: ~64 m/s IAS on the bench airframe). Below it nothing the
		// FCS has can hold the heading and the jet yaws off into a departure (10:37 log, 150-158 s: sideslip -1 -> -67 deg
		// at 12 deg AoA, 47-58 m/s, every surface on its stop). The throttle is NOT touched while the FCS still holds the
		// yaw: only once the yaw is saturated AND the sideslip is running away (past VmcBeta and growing, or past
		// 2 x VmcBeta) is the throttle input pulled back, at VmcCutRate per s, until the sideslip stops growing; it is given
		// back at VmcReleaseRate per s once the sideslip is under VmcBeta / 2 and the yaw has room again. The pilot's
		// throttle is never raised, only capped.
		internal static bool VmcProtect = true;

		internal static float VmcBeta = 5f;

		internal static float VmcQuiet = 1f;

		private float vmcQuietT;

		internal static float VmcCutRate = 0.8f;

		internal static float VmcReleaseRate = 0.2f;

		public float vmcLim = 1f;

		private float yawClampExcess;

		internal static bool YawSatCountClamp = true;

		public bool vmcOn;

		private float vmcBetaPrev;

		private float vmcSatF;

		// Revoker 0.2.7 (FinLossTvcHold): with the fin gone the yaw nozzle is the yaw axis, and the game stops vectoring above
		// the engine's thrustVectoringMaxAirspeed (250 m/s; Turbojet.FixedUpdate zeroes the nozzle angles past it). With the fin
		// shot off nothing held the yaw there: 28 Oct 21:22 log, fin and wings gone, full afterburner to 247 m/s, the nozzles cut
		// out at 211.0 s and the jet yawed round (r -128 deg/s, sideslip 85 deg in 2 s). While yawHealth is under 0.5 (the FCS
		// flying, TVC enabled) the cut-off is raised to FinLossTvcSpeed on this jet's engines; put back once the fin is whole
		// (repair / respawn) or the FCS lets go. Holding the throttle down below the cut-off was tried instead: the nozzle's
		// yaw authority is the thrust, and the bench departed at 230 m/s with the throttle cut (fin + wings gone).
		private readonly Dictionary<Turbojet, float> vecMaxOrig = new Dictionary<Turbojet, float>();

		public bool tvcHoldOn;

		private void TvcCutoffStep()
		{
			if (tvc == null)
			{
				return;
			}
			bool hold = Plugin.FinLossTvcHold && Engaged && !Failed && !Plugin.DisableTVC && tvcy != null && yawHealth < 0.5f;
			tvcHoldOn = hold;
			foreach (Turbojet e in tvc.engines)
			{
				if (e == null)
				{
					continue;
				}
				ref float vm = ref Acc.TF_vecMaxSpeed(e);
				if (!vecMaxOrig.TryGetValue(e, out float o))
				{
					o = vm;
					vecMaxOrig[e] = o;
				}
				vm = hold ? Mathf.Max(o, FinLossTvcSpeed) : o;
			}
		}

		public void VmcApply(ControlInputs inp, float dt)
		{
			TvcCutoffStep();
			if (!VmcProtect || !Engaged || Failed || inp == null || Mode == FcsMode.Ground || wheelsTouch)
			{
				vmcLim = 1f;
				vmcOn = false;
				vmcBetaPrev = beta;
				abGateOn = false;
				matchYawOn = false;
				vmcMatch = false;
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
				return;
			}
			float bDot = (beta - vmcBetaPrev) / Mathf.Max(dt, 0.001f);
			vmcBetaPrev = beta;
			// saturation: the yaw target out of reach (yawSat) or the asymmetric-load model's yaw usage past 1 at this AoA
			// yaw usage including the dead engine's thrust (the cap no longer counts it)
			float uY = asymValid ? (Mathf.Abs(asymAyNow) / Mathf.Max(asymCyNow, 1f)) : 0f;
			float satNow = Mathf.Max(yawSat, Mathf.Clamp01((uY - 0.9f) / 0.3f));
			vmcSatF += (satNow - vmcSatF) * Mathf.Clamp01(dt / 0.3f);
			// 2.0.13: sideslip from the pilot's own roll / pedal is not the engine's doing. The 13:07 log cut the throttle at
			// 80-87 m/s (290-315 km/h) on full-stick rolls toward the dead engine (sideslip -10..-16 from the roll, with the
			// rudders busy holding the thrust): those don't count. The lateral input (and the roll rate it makes) must have
			// been quiet for VmcQuiet s, and the sideslip must be on the side the live engine's thrust pushes it.
			bool latQuiet = Mathf.Abs(sr) < 0.15f && Mathf.Abs(sy) < 0.15f && Mathf.Abs(omega.x * 57.29578f) < 20f;
			vmcQuietT = latQuiet ? (vmcQuietT + dt) : 0f;
			bool thrustSide = !asymValid || asymAyNow * beta < 0f;
			bool growing = Mathf.Abs(beta) > VmcBeta && beta * bDot > 0f;
			bool runaway = (engOut || engAsym) && vmcSatF > 0.5f && thrustSide && (vmcQuietT > VmcQuiet || Mathf.Abs(beta) > 3f * VmcBeta) && (growing || Mathf.Abs(beta) > 2f * VmcBeta);
			// with a weak (running) engine the run-away is answered by the thrust match, not by cutting both engines
			// (23 Oct 00:06 F-22E log: both throttled to T/W 0.13 for 5 s at 40-60 m/s): the match holds until the
			// sideslip has been calm for 2 s
			bool weakEng = engAbRatio < EngineAsymEnter && !engOut;
			if (runaway && weakEng && Plugin.EngineAbAoAGate && EngineOutFixedCap)
			{
				vmcMatch = true;
				vmcMatchCalmT = 0f;
				runaway = false;
			}
			if (vmcMatch)
			{
				bool calmM = Mathf.Abs(beta) < 0.5f * VmcBeta && vmcSatF < 0.3f;
				vmcMatchCalmT = calmM ? (vmcMatchCalmT + dt) : 0f;
				if (vmcMatchCalmT > 2f || !weakEng)
				{
					vmcMatch = false;
				}
			}
			if (runaway)
			{
				if (!vmcOn)
				{
					vmcLim = Mathf.Min(vmcLim, inp.throttle);
				}
				vmcOn = true;
				vmcLim = Mathf.Max(0f, vmcLim - VmcCutRate * dt);
			}
			else
			{
				bool calm = Mathf.Abs(beta) < 0.5f * VmcBeta && vmcSatF < 0.3f;
				if (calm || !(engOut || engAsym))
				{
					vmcLim = Mathf.Min(1f, vmcLim + VmcReleaseRate * dt);
				}
				if (vmcLim >= 0.999f)
				{
					vmcOn = false;
				}
			}
			if (vmcLim < 0.999f)
			{
				inp.throttle = Mathf.Min(inp.throttle, vmcLim);
			}
			// 0.4.1 thrust match: with one engine down on thrust (predicted full-throttle ratio under EngineAsymEnter) and the
			// AoA above the damaged-engine cap, the healthy engine is held down to the damaged one's thrust at whatever
			// throttle the pilot sets: the game derates a damaged engine's whole throttle-to-RPM map (target RPM =
			// Lerp(minRPM, minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5), throttle)), so the two only ever match at
			// idle by throttle alone; the healthy engine's parasiticThrustLoss (thrust = (maxThrust - loss) x RPM ratio,
			// unused on this jet) is set each tick from the measured RPMs so its dry thrust equals the weak one's. If the weak
			// engine cannot light its afterburner the throttle input is also held at the afterburner's start. No thrust
			// split, so no AoA cap for it: the pilot keeps the full AoA, and the thrust comes back once he brings the AoA
			// down to the cap himself (on above cap + 2, off under cap + 0.5: riding the limiter counts as inside). With
			// an engine OUT there is nothing to match: above EngineOutAoACap the throttle is held at military power. The
			// VMC protection (measured sideslip run-away) stays the last resort.
			bool weak = engAbRatio < EngineAsymEnter;
			bool gateNeed = Plugin.EngineAbAoAGate && EngineOutFixedCap && (engOut || weak);
			float tgtF = 0f;
			if (gateNeed && engOut)
			{
				// an engine out: nothing to match; above EngineOutAoACap the throttle is held at the afterburner's start
				float capO = Plugin.EngineOutAoACap;
				float aAbsO = Mathf.Abs(alpha);
				abGateOn = abGateOn ? (aAbsO > capO + 0.5f) : (aAbsO > capO + 2f);
				matchYawOn = false;
				vmcMatch = false;
			}
			else if (gateNeed)
			{
				float capA = Plugin.EngineDamagedAoACap;
				float aAbs = Mathf.Abs(alpha);
				float spDir = sp * ((alpha >= 0f) ? 1f : (-1f));
				// 0.4.2 / 2.4.2: the match is timed, not placed. The AoA limiter starts braking for the cap well before it, and
				// a match that only completes as the AoA arrives at the cap leaves the pilot feeling the brake until it does
				// (23 Oct 00:39 log: full match 1.4-1.9 s after it began at 230-360 kt, where the AoA came up at 18 deg/s,
				// against 0.23 s at 95-170 kt and 35 deg/s). Now: with the stick pulling toward the cap, the match goes to 1
				// once the AoA is predicted to reach the cap within MatchLeadTime s at its present rate (or is already within
				// MatchBand of it), so it is complete, at MatchRiseRate, before the limiter would have begun to brake,
				// whatever the speed; above the cap it is 1 whatever the stick; stick relaxed under the cap it is 0 (thrust
				// comes back).
				float aRate = alphaDot * ((alpha >= 0f) ? 1f : (-1f));
				float tCap = (capA - aAbs) / Mathf.Max(aRate, 0.5f);
				float ramp = 0f;
				if (aAbs > capA + 2f)
				{
					ramp = 1f;
				}
				else if (spDir > 0.15f && (aAbs > capA - MatchBand || (aRate > 0.5f && tCap < MatchLeadTime)))
				{
					ramp = 1f;
				}
				else if (spDir >= 0.05f && aAbs > capA - MatchBand)
				{
					ramp = Mathf.Clamp01((aAbs - (capA - MatchBand)) / MatchBand);
				}
				// and on prediction: the split's yaw against what the fins have at this AoA and speed (see the engine scan)
				matchYawOn = matchYawOn ? (splitUsage > MatchYawOff) : (splitUsage > MatchYawOn);
				tgtF = Mathf.Max(ramp, (matchYawOn || vmcMatch) ? 1f : 0f);
				abGateOn = tgtF > 0.001f || matchF > 0.001f;
			}
			else
			{
				abGateOn = false;
				matchYawOn = false;
				vmcMatch = false;
			}
			if (abGateOn && engOut)
			{
				inp.throttle = Mathf.Min(inp.throttle, engAbStart);
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
			}
			else if (gateNeed && !engOut)
			{
				matchF = Mathf.MoveTowards(matchF, tgtF, ((tgtF > matchF) ? MatchRiseRate : MatchFallRate) * dt);
				matchFull = matchF >= 0.995f;
				if (matchF > 0.001f)
				{
					if (engAbMismatch)
					{
						// the afterburner the weak engine cannot light fades out with the match
						inp.throttle = Mathf.Min(inp.throttle, Mathf.Lerp(1f, engAbStart, matchF));
					}
					ThrustMatchApply(matchF);
				}
				else
				{
					ThrustMatchClear();
				}
			}
			else
			{
				matchF = 0f;
				matchFull = false;
				ThrustMatchClear();
			}
		}

		// hold every stronger engine's dry thrust down to the weakest one's, from the measured RPMs (see VmcApply)
		private void ThrustMatchApply(float frac)
		{
			float target = float.MaxValue;
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg != null && Acc.TF_operable(eg))
				{
					float rMin = Acc.TF_minRPM(eg);
					float rMax = Acc.TF_maxRPM(eg);
					float ratio = (rMax > rMin) ? Mathf.Clamp01((Acc.TF_rpm(eg) - 0.85f * rMin) / (rMax - rMin)) : 1f;
					target = Mathf.Min(target, Mathf.Max(0f, Acc.TF_staticThrust(eg)) * ratio);
				}
			}
			if (target == float.MaxValue)
			{
				ThrustMatchClear();
				return;
			}
			float lossMax = 0f;
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg == null)
				{
					continue;
				}
				float loss = 0f;
				if (Acc.TF_operable(eg))
				{
					float rMin = Acc.TF_minRPM(eg);
					float rMax = Acc.TF_maxRPM(eg);
					float ratio = (rMax > rMin) ? Mathf.Clamp01((Acc.TF_rpm(eg) - 0.85f * rMin) / (rMax - rMin)) : 1f;
					float t0 = Mathf.Max(0f, Acc.TF_staticThrust(eg));
					if (ratio > 0.001f)
					{
						loss = Mathf.Clamp(t0 - target / ratio, 0f, t0) * Mathf.Clamp01(frac);
					}
				}
				Acc.TF_parasitic(eg) = loss;
				lossMax = Mathf.Max(lossMax, loss);
			}
			engMatchLoss = lossMax;
			matchApplied = true;
		}

		public void ThrustMatchClear()
		{
			if (!matchApplied)
			{
				return;
			}
			matchApplied = false;
			engMatchLoss = 0f;
			if (tvc == null)
			{
				return;
			}
			foreach (Turbojet eg in tvc.engines)
			{
				if (eg != null)
				{
					Acc.TF_parasitic(eg) = 0f;
				}
			}
		}

		// ---- 2.0.12 fuel trim: lateral CG moved with fuel when yaw or roll runs short -----------------------------------
		// Moving the CG sideways by d (m) changes the jet's moments by -d x (thrust + lift) about the new CG: toward the
		// live engine it shortens that engine's lever arm (less engine-out yaw: at full afterburner ~3 deg/s^2 per 10 cm,
		// of ~18), and with damage it can also offset an aerodynamic yaw or roll asymmetry. It costs roll: lift through a
		// CG 10 cm off the centreline rolls the jet ~15 deg/s^2 per g. So nothing moves until the yaw or roll usage at the
		// present AoA (load / authority, the asymmetric-load model's numbers) passes FuelTrimStart; then the offset is
		// the one that best evens out the roll and yaw usage (minimax), with the roll side judged at 1.5 g or more so a
		// pull does not run the roll out, at most FuelTrimMaxOffset either side. When both usages are back under
		// FuelTrimStop for 2 s the normal balancing brings the CG back to the centreline.
		internal static float FuelTrimStart = 0.75f;

		internal static float FuelTrimStop = 0.5f;

		internal static float FuelTrimMaxOffset = 0.3f;

		public bool fuelTrimOn;

		public float fuelTrimLat;

		public float fuelTrimDelta;

		private float fuelTrimTimer;

		private float fuelTrimCalm;

		private void FuelTrimStep(float dt, Quaternion rootRot)
		{
			if (!Plugin.FuelTrim || !Plugin.FuelBalanceOn || !asymValid || wheelsTouch || Mode == FcsMode.Ground || (UnityEngine.Object)(object)ac == null)
			{
				fuelTrimOn = false;
				return;
			}
			float cr = Mathf.Max(asymCrNow, 1f);
			float cy = Mathf.Max(asymCyNow, 1f);
			float ur = Mathf.Abs(asymArNow) / cr;
			float uy = Mathf.Abs(asymAyNow) / cy;
			float uMax = Mathf.Max(ur, uy);
			fuelTrimCalm = (uMax < FuelTrimStop) ? (fuelTrimCalm + dt) : 0f;
			if (fuelTrimOn && fuelTrimCalm > 2f)
			{
				fuelTrimOn = false;
			}
			fuelTrimTimer -= dt;
			if (fuelTrimTimer > 0f || (!fuelTrimOn && uMax < FuelTrimStart))
			{
				return;
			}
			fuelTrimTimer = 0.5f;
			Transform tr = ((Component)(object)ac).transform;
			float m = Mathf.Max(1000f, mass);
			Vector3 fT = tr.forward * (tw * m * 9.81f);
			Vector3 fL = tr.up * (Mathf.Max(nz, 1.5f) * m * 9.81f);
			Quaternion inv = Quaternion.Inverse(rootRot);
			Vector3 w = inv * (-Vector3.Cross(tr.right, fT + fL));
			Vector3 dM = new Vector3(0f - w.z, 0f - w.x, w.y);
			float gR = AccRoll(dM);
			float gY = AccYaw(dM);
			float best = 0f;
			float bestU = uMax;
			for (int i = -30; i <= 30; i++)
			{
				float d = FuelTrimMaxOffset * i / 30f;
				float u = Mathf.Max(Mathf.Abs(asymArNow + gR * d) / cr, Mathf.Abs(asymAyNow + gY * d) / cy);
				if (u < bestU - 1e-4f)
				{
					bestU = u;
					best = d;
				}
			}
			if (!fuelTrimOn && uMax - bestU < 0.1f)
			{
				return;
			}
			float latNow = FuelBalance.LateralCg(ac);
			fuelTrimDelta = best;
			fuelTrimLat = latNow + best;
			fuelTrimOn = true;
		}

		public float revSenseDbg = 1f;

		// Ifrit 0.3.4: roll stick sense vs AoA for the velocity-vector roll: +1 up to RevFlowSenseStart (|AoA|), -1 from
		// RevFlowSenseEnd, smoothstep between
		private static float RevFlowSense(float a)
		{
			float a0 = Plugin.RevFlowSenseStart;
			float a1 = Mathf.Max(a0 + 1f, Plugin.RevFlowSenseEnd);
			float x = Mathf.Clamp01((Mathf.Abs(a) - a0) / (a1 - a0));
			x = x * x * (3f - 2f * x);
			return 1f - 2f * x;
		}

		// 2.35.0 (PushPlan, see the statics): plan the push once at its start; hold; let it rise only
		private void PushPlanStep(float dt, float cA, float sA, float num11Stick)
		{
			// (fins damaged: the plan's IAS-based shed does not hold - the 2.19.0 nose hold and the governor pace the push instead)
			if (!PushPlan || isAI || tas < 40f || yawHealth < 0.99f)
			{
				pushPlanOn = false;
				return;
			}
			float aAbs = Mathf.Abs(alpha);
			float sgA = (alpha >= 0f) ? 1f : -1f;
			bool pushing = sgA * sp < -0.3f;
			float rDeg = Mathf.Abs(omega.z * 57.29578f);
			float capPlan = Mathf.Clamp(PushPlanK0 + PushPlanK * Mathf.Pow(Mathf.Max(1f, ias), PushPlanExp), 6f, 80f);
			float rLeft = Mathf.Sqrt(2f * capPlan * Mathf.Max(0f, (PushPlanSlip < 0f) ? Plugin.PushoverSideslip : PushPlanSlip));
			float wv = Mathf.Max(0f, sgA * omegaV);
			if (!pushPlanOn)
			{
				if (pushing && aAbs > PushPlanA0 && rDeg > PushPlanR0)   // a push-over OF a rotation (push + roll entered together is the trade's)
				{
					pushPlanOn = true;
					pushPlanSide = sgA;
					float T0 = Mathf.Max(PushPlanTMin, Mathf.Max(0f, rDeg - rLeft) / capPlan);
					pushPlanT = T0;
					pushPlanQ = Mathf.Clamp(Mathf.Max(0f, aAbs - 10f) / T0 - PushPlanWvK * wv, PushPlanQMin, 60f);
					// 2.40.0: the 2.35.1 clamp under the sweep's AoA-rate ceiling is gone. It kept the plan 15 deg/s under the 60 deg/s ceiling
					// so the ceiling would not cut it mid-push; since 2.36.0 the ceiling stands aside under a plan (PushPlanOwnsPitch), and the
					// clamp only limited speed - the plan's own yaw model sets it now. Hybrid replays, the 20 pushes it bound: 10 deg AoA reached
					// 0.05-0.23 s sooner, sideslip +0..+2.6 deg (worst push unchanged, 8.3), the re-attach overshoot mostly smaller (23:10 70.4 s
					// 24.5 -> 3.5 deg/s: the plan now matches the jet's own pitch-down).
					pushPlanQNow = Mathf.Max(0f, -sgA * qCmd);
					pushPlanBleed = Mathf.Abs(pTraj.x) / T0;
					float pTEnd = (stickRelW > 0.5f || num11Stick * pTraj.x <= 0f) ? 0f : Mathf.Min(Mathf.Abs(num11Stick), Mathf.Abs(pTraj.x));
					pushPlanD = Mathf.Max(capPlan, (Mathf.Abs(pTraj.x) * Mathf.Abs(sA) - pTEnd * 0.17364818f) / T0);
				}
				return;
			}
			// end: the stick no longer pushes or the AoA changed side; below 10 deg the bound ramps out at PushPlanRelease (no kick at the
			// bottom) and lets go once it is above anything the stick can ask
			// 2.40.0 (2.36-dev's PushPlanEndAtBottom, now always): only the bottom's ramp-out ends it - a plan whose rate is the stick's full 60 deg/s mid-push is
			// still the plan (the yaw allows the full rate); ending there handed the push to the governor's caps mid-sweep (-60 -> -51)
			if (!pushing || sgA != pushPlanSide || (pushPlanQNow >= 60f && aAbs < 10f))
			{
				pushPlanOn = false;
				return;
			}
			if (aAbs < 10f)
			{
				pushPlanQ = 60f;
				pushPlanQNow = Mathf.MoveTowards(pushPlanQNow, 60f, PushPlanRelease * dt);
				return;
			}
			// rise only: what the yaw left now would allow, approached at PushPlanRise
			float Tn = Mathf.Max(0.2f, Mathf.Max(0f, rDeg - rLeft) / capPlan);
			float qNeed = Mathf.Clamp(Mathf.Max(0f, aAbs - 10f) / Tn - PushPlanWvK * wv, PushPlanQMin, 60f);
			if (qNeed > pushPlanQ) pushPlanQ = Mathf.Min(qNeed, pushPlanQ + PushPlanRise * dt);
			pushPlanQNow = Mathf.MoveTowards(pushPlanQNow, pushPlanQ, PushPlanJerk * dt);
		}

		private void LateralLaw(float dt)
		{
			float f = alpha * (MathF.PI / 180f);
			float num = Mathf.Cos(f);
			float num2 = Mathf.Sin(f);
			pFull = rollCtrlHalf / Mathf.Max(1f, 0f - Lp) * 57.29578f;
			pStockRef = StockRollRef();
			float trimR = ((RollTrimHeadroom && Plugin.AsymRollHeadroom) ? (Mathf.Sign(dTrimR) * Mathf.Min(Mathf.Max(0f, Mathf.Abs(dTrimR) - Mathf.Max(0f, Plugin.AsymDeadband)), 0.8f * aRoll)) : 0f);
			aRollPos = aRoll - Mathf.Max(0f, 0f - trimR);
			aRollNeg = aRoll - Mathf.Max(0f, trimR);
			bool mpoSep = Mode == FcsMode.MPO && Plugin.MpoSeparateAxes && !isAI;
			mpoSepDbg = mpoSep;
			float rollSide = mpoSep ? sr : (sr + Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f) * sy);
			if (Mathf.Abs(rollSide) < 0.02f)
			{
				rollSide = pTraj.x;
			}
			float aRollGo = ((rollSide >= 0f) ? aRollPos : aRollNeg);
			float aRollStop = ((rollSide >= 0f) ? aRollNeg : aRollPos);
			YawHealthStep();
			float num3 = Mathf.Min(Plugin.MaxRollRate, Mathf.Min(pStockRef, Plugin.RollAuthorityUse * pFull * (aRollGo / aRoll)));
			if (Plugin.RollStopFactor > 0.01f)
			{
				num3 = Mathf.Min(num3, Plugin.RollStopFactor * Mathf.Min(aRollGo, aRollStop));
			}
			float num4 = ((Mode == FcsMode.MPO) ? Plugin.MpoGPos : Plugin.GPos);
			float num5 = ((Mode == FcsMode.MPO) ? Plugin.MpoGNeg : Plugin.GNeg);
			float value = 1f;
			float t = Mathf.Clamp01((ias - 250f) / 150f);
			float num6 = Mathf.Lerp(6f, 3f, t);
			if (nz > num6)
			{
				value = Mathf.Lerp(1f, Mathf.Lerp(0.5f, 0.35f, t), (nz - num6) / Mathf.Max(0.5f, num4 - num6));
			}
			else if (nz < -1.5f)
			{
				value = Mathf.Lerp(1f, 0.6f, (-1.5f - nz) / Mathf.Max(0.5f, -1.5f - num5));
			}
			num3 *= Mathf.Clamp(value, 0.4f, 1f);
			num3 = (pMaxDbg = Mathf.Max(num3, 30f));
			float num7 = LowIasYawBoostAt(ias);
			float num8 = (rMaxDbg = Mathf.Lerp(Plugin.MaxYawRate, Mathf.Clamp(num7 * Plugin.YawStopFactor * aYaw, 20f * num7, Plugin.MaxYawRate), Mathf.Clamp01((Mathf.Abs(alpha) - 15f) / 10f)));
			// 1.8.45 braking floor: at high AoA the roll and yaw rate limits may not exceed what the stop-side authority,
			// at the IAS predicted from a rapid deceleration, brakes in the stop time of the reference manoeuvre (full
			// pedal + full lateral stick at 65 deg AoA, 280 km/h: RollStopFactor for roll, YawStopFactor x yaw boost there
			// for yaw). Replaces the fixed 30 deg/s roll / 20 x boost yaw floors there, which outran the authority below ~65 m/s.
			float gB = Mathf.Clamp01((Mathf.Abs(alpha) - LatBrakeA0) / Mathf.Max(1f, LatBrakeA1 - LatBrakeA0));
			latPredK = qPredK;
			if (LatBrakeFloor && gB > 0f)
			{
				// 1.9.3: yaw floor = the yaw braking at 270 km/h; from 210 to 150 km/h the allowed stop time grows smoothly
				// (smoothstep) by YawMarginLow (1.5 x: 50 % more yaw rate for the same authority, slower braking), flat below.
				// The roll stop time gets the same factor: at 65 deg AoA a velocity-vector turn needs body roll = cos(AoA) x its
				// rate, and a roll limit left at 1.0 s capped the turn (and so the yaw rate) at ~160 km/h.
				// The floor limits join the normal limits with a C1 soft minimum (no corner near 270 km/h).
				// Roll: the stop-side authority is the modelled asymmetric-load trim off the full authority, low-passed. The
				// measured trim (dTrimR) it used carried the turn's own dihedral / yaw-rate roll moment, which swings with
				// sideslip and yaw rate: in a max-AoA stick turn it moved 20 <-> 29 deg/s^2 at 1.4 Hz, the roll ceiling
				// 9 <-> 18 deg/s with it, the velocity-vector ceiling 20 <-> 42, and yaw rate 27 <-> 40 (a limit cycle).
				float iasKmh = ias * 3.6f;
				float sm = Mathf.Clamp01((YawMarginIasHi - iasKmh) / Mathf.Max(1f, YawMarginIasHi - YawMarginIasLo));
				sm = sm * sm * (3f - 2f * sm);
				float yawMargin = Mathf.Lerp(1f, Mathf.Max(1f, YawMarginLow), sm);
				yawMarginDbg = yawMargin;
				float tY = Plugin.YawStopFactor * LowIasYawBoostAt(BrakeRefIas) * yawMargin;
				float asymTrim = asymValid ? Mathf.Max(0f, Mathf.Abs(asymArNow) - Mathf.Max(0f, Plugin.AsymDeadband)) : 0f;
				float stopA = Mathf.Max(0.2f * aRoll, aRoll - Mathf.Min(asymTrim, 0.8f * aRoll));
				rollStopAF = (rollStopAF < 0f) ? stopA : (rollStopAF + (stopA - rollStopAF) * Mathf.Clamp01(dt / Mathf.Max(0.02f, RollStopTau)));
				float pB = Mathf.Max(LatBrakeMinRate, Mathf.Max(0.01f, Plugin.RollStopFactor) * yawMargin * rollStopAF * qPredK);
				float rB = Mathf.Max(LatBrakeMinRate, tY * aYaw * qPredK);
				num3 = Mathf.Lerp(num3, SoftMin(num3, pB, BrakeSoftMin * 0.5f * (num3 + pB)), gB);
				// 1.9.4: mid-IAS band (290-330 km/h) with less yaw stop time (x YawMidFactor), smoothstep ramps outside it
				float mLo = Mathf.Clamp01((iasKmh - (Plugin.YawMidLoKmh - Plugin.YawMidRampKmh)) / Mathf.Max(1f, Plugin.YawMidRampKmh));
				float mHi = Mathf.Clamp01(((Plugin.YawMidHiKmh + Plugin.YawMidRampKmh) - iasKmh) / Mathf.Max(1f, Plugin.YawMidRampKmh));
				mLo = mLo * mLo * (3f - 2f * mLo);
				mHi = mHi * mHi * (3f - 2f * mHi);
				float yawMid = Mathf.Lerp(1f, Plugin.YawMidFactor, Mathf.Min(mLo, mHi));
				yawMidDbg = yawMid;
				num8 = Mathf.Lerp(num8, yawMid * SoftMin(num8, rB, BrakeSoftMin * 0.5f * (num8 + rB)), gB);
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			// Ifrit 0.3.1 (HighAoALateralBrakeMargin): the roll and yaw rate limits x this factor at high AoA, ramped in
			// smoothly (smoothstep) from HighAoALateralMarginStart to HighAoALateralMarginFull. < 1 = less allowed stop
			// time: lower roll / yaw rate for the same authority, harder braking; > 1 the other way.
			{
				float kL = Plugin.HighAoALatMargin;
				if (Mathf.Abs(kL - 1f) > 0.0001f)
				{
					float a0L = Plugin.HighAoALatMarginStart;
					float wL = Mathf.Clamp01((Mathf.Abs(alpha) - a0L) / Mathf.Max(0.5f, Plugin.HighAoALatMarginFull - a0L));
					wL = wL * wL * (3f - 2f * wL);
					float fL = Mathf.Lerp(1f, kL, wL);
					num3 *= fL;
					num8 *= fL;
					pMaxDbg = num3;
					rMaxDbg = num8;
				}
			}
			// 2.0.12: rudder lost (fin shot off): the roll and yaw rate limits shrink with the yaw authority left. A body roll
			// at AoA turns AoA into sideslip at p sin(AoA) unless the rudders make the matching yaw rate; the clean limits
			// (250 deg/s) asked one rudder for twice what it has (10:29 log, 276 s: -265 deg/s, sideslip -29, departure).
			// 2.0.13: and with the yaw already partly spent holding an asymmetry (engine out: the live engine's yaw), by the
			// share of the yaw authority left over at this AoA
			float yawLeft = 1f;
			if (DamageLatLimit && asymValid && asymCyNow > 1f)
			{
				yawLeft = Mathf.Clamp01(1f - Mathf.Abs(asymAyNow) / asymCyNow);
				// 2.1.4: past the rudder fade the stabs are the yaw source as well as a roll source, and the deflection that
				// trims the load's roll is the same deflection the yaw needs: the roll trim also comes out of the yaw left
				// (in proportion to the rudder fade). Only the load beyond the deadbands counts, so a clean jet is unchanged.
				// 12 Oct 18:29 log: at the asymmetric cap (~50 deg AoA) the limits still allowed 25-33 deg/s of roll and
				// 20-30 deg/s of yaw rate; centring the stick, the stabs could not brake both against the load and the
				// sideslip ran to 23 deg (74 s, 209 s).
				if (AsymSharedLat && Plugin.AsymSharedLatOn && asymCrNow > 1f)
				{
					float dbR = Mathf.Max(0f, Plugin.AsymDeadband);
					float rollUse = Mathf.Max(0f, Mathf.Abs(asymArNow) - dbR) / asymCrNow;
					float yawUse = Mathf.Max(0f, Mathf.Abs(asymAyNow) - 0.25f * dbR) / asymCyNow;
					yawLeft = Mathf.Clamp01(1f - yawUse - (1f - rudderAuth) * rollUse);
				}
			}
			sharedLatDbg = yawLeft;
			// 2.12.0: the fin-health factor fades with the rudders (rudderAuth, 50 -> 60 deg AoA): past the fade the fins are not the
			// yaw axis, so losing them takes nothing away there - the stabs, flaperons and the airframe's own balance carry the
			// velocity-vector roll, and the measured capacity says so (18:32 log, both fins gone: capZ 38 at 50-65 deg against 39 with
			// fins; 29-33 against 125-147 at 15-40 deg). Un-faded, rMax x 0.35 capped the corkscrew at 24 deg/s of yaw = 28 deg/s of
			// velocity roll at 65 deg with the margin untouched (sideslip 2-6 deg) - Tom: "isn't that when the rudders can't really
			// do any yaw, so why is it capped". Below the fade the open-loop factor stays until the rate cap's reversal probe
			// carries the roll demand (18:08 log, 541 s: capZ 99 -> 5-23 once a no-fin reversal loaded the flaperons: 17.6 deg).
			// The asymmetric-load share (yawLeft) is already rudderAuth-aware and is not faded.
			float finH = Mathf.Lerp(1f, yawHealth, Mathf.Clamp01(rudderAuth));
			if (DamageLatLimit && Mathf.Min(finH, yawLeft) < 0.999f)
			{
				// Revoker 0.1.0: with the vertical tail damaged and the nozzle unable to yaw only drag yaw is left (see DamageLatFloorNoTvc)
				float floorK = (tvcy != null && !tvcy.active && yawHealth < 0.5f) ? Mathf.Min(DamageLatFloor, DamageLatFloorNoTvc) : DamageLatFloor;
				float hK = Mathf.Max(floorK, Mathf.Min(finH, yawLeft));
				num3 = Mathf.Max(30f * hK, num3 * hK);
				num8 *= hK;
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			// Revoker 0.2.4 (HighAoALimitTau): above 30-45 deg AoA the roll / yaw rate limits are low-passed. They come from
			// the stop-side authority estimates, which flicker with the deflections (bench corkscrew: body-roll limit 25.6 <->
			// 29.8 deg/s every few frames; in game 13 <-> 28 deg/s below 40 m/s), and the velocity-vector roll command and the
			// yaw rate followed every step. The real changes (IAS decay) take seconds.
			if (LimTau > 0f)
			{
				if (pLimF < 0f)
				{
					pLimF = num3;
					rLimF = num8;
				}
				float kLim = Mathf.Clamp01(dt / LimTau);
				pLimF += (num3 - pLimF) * kLim;
				rLimF += (num8 - rLimF) * kLim;
				float wLim = Mathf.InverseLerp(30f, 45f, Mathf.Abs(alpha));
				num3 = Mathf.Lerp(num3, pLimF, wLim);
				num8 = Mathf.Lerp(num8, rLimF, wLim);
				pMaxDbg = num3;
				rMaxDbg = num8;
			}
			float num9 = Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f);
			float num10 = Mathf.Min(num3 / Mathf.Max(Mathf.Abs(num), 0.05f), num8 / Mathf.Max(Mathf.Abs(num2), 0.05f));
			// Ifrit 0.3.4: the reversed-flow roll sense blends smoothly with the AoA (RevFlowSenseStart -> End: +1 -> -1 through
			// 0) instead of flipping at 100 deg (released at 80). Past 90 deg the velocity-vector roll is mostly body yaw
			// (sin AoA) and only a little body roll (cos AoA), so the flip there reversed the whole rotation: 20 Oct 23:01 log,
			// MPO, full right stick held from -115 to -60 deg AoA, yawing right at 10-20 deg/s; at -80 deg the target went
			// -23 -> +29 deg/s (353.7 s, 394.4 s, the same at -79 deg) and the jet had to reverse its yaw. Now the rotation
			// stays the same through +-90 deg and the sense turns over near 180 deg, where the velocity-vector roll is nearly
			// all body roll and the flip keeps the stick rolling the jet the same way from the seat. In the blend band the
			// stick's roll command passes through zero rather than jumping.
			float revS = Plugin.RevFlowRollSense ? RevFlowSense(alpha) : 1f;
			revSenseDbg = revS;
			float num11raw = Mathf.Clamp(revS * (sr + num9 * sy), -1f, 1f);
			// 2.16.0: a rate LIMIT bounds the rotation the stick may build; it may not cut a rotation the jet already has faster than
			// the yaw can shed it. With the stick on the rotation's side the limit is floored at the velocity roll the body yaw
			// implies (r / sin AoA: the coordinated roll for the yaw the jet has), which decays as the yaw brakes - the roll follows the
			// yaw down instead of braking away from it. 20:19 log (both fins gone), held push-over from 62 deg: as the AoA crossed the
			// rudder fade the fin-health factor took pSmax 61 -> 16, the trajectory braked at the tuned rate, p -28 -> -13 against a
			// yaw of -47 that needed -42 to stay coordinated, sideslip +20 deg/s, departure (and the bench twin, dmgsuite 'pushover
			// HELD', the same chain). The stick scales the target as always (easing off is the pilot's deceleration; a release is the
			// window's job); sin AoA floored at LatFloorSinMin so a low-AoA turn's yaw is not read as a rotation.
			latFloorDbg = 0f;
			pSStickRaw = Mathf.Abs(num11raw) * num10;   // 2.18.0: the stick's own target before the floor (the shed cap's credit)
			stickRelW = Mathf.Clamp01(1f - Mathf.Abs(num11raw) / Mathf.Max(0.02f, SweepNoReverseStick));   // 2.24.0: 1 = lateral stick released
			float finKL = Mathf.Clamp01(yawHealth / 0.5f);
			if (LatLimitYawFloor)
			{
				float sinF = ((num2 >= 0f) ? 1f : -1f) * Mathf.Max(Mathf.Abs(num2), LatFloorSinMin);
				float pSyaw = omega.z * 57.29578f / sinF;
				if (Mathf.Abs(num11raw) > 0.05f)
				{
					if (num11raw * pSyaw > 0f)
					{
						float fl = Mathf.Abs(pSyaw);
						if (fl > num10)
						{
							latFloorDbg = fl - num10;
							num10 = fl;
						}
					}
				}
				else if (ReleaseCoordinate && finKL < 1f && yawSatAlphaRateDn >= 0f && alphaDot * alpha < 0f && Mathf.Abs(pSyaw) > 5f)
				{
					// 2.18.0: stick released under a capped nose-down sweep with the fins gone: the roll stays coordinated with the yaw
					// the jet has (target = r / sin AoA, decaying as the yaw sheds) instead of going to zero while 20-30 deg/s of yaw is
					// still on (23:49 log, 122 s: roll target 9 -> 0 in 1.5 s, r 29 -> 22, sideslip -1 -> -53 deg). With the fins the
					// rudders shed the yaw in tenths and the stop stands as it is (finK 1: off).
					float fl = Mathf.Abs(pSyaw) * (1f - finKL);
					num10 = Mathf.Max(num10, fl);
					num11raw = ((pSyaw >= 0f) ? 1f : -1f) * Mathf.Min(1f, fl / Mathf.Max(1f, num10));
					latFloorDbg = fl;
				}
			}
			float num11 = num11raw * num10;
			pSAskStick = Mathf.Abs(num11);
			coupRollLim = 0f;
			if (Plugin.CouplingRollLimit > 0f && Ia[1, 1] > 1f && asymValid && tvc.M != null && tvc.N > 1)
			{
				float kIc = (Ia[2, 2] - Ia[0, 0]) / Ia[1, 1];
				float s2c = Mathf.Abs(Mathf.Sin(2f * f));
				{
					float gateC = Mathf.Max(pitchNeedUp, Mathf.Clamp01((Mathf.Sign(alpha) * alphaDot - 5f) / 15f)) * Mathf.Clamp01((Mathf.Abs(alpha) - CoupGateA0) / 10f);
					// 2.1.3: at the low-thrust AoA cap the gate stays on within LowCapCoupBand of the cap instead of waiting for the
					// AoA to rise: with the nozzles pinned and weak, the inertial pitch-up of the roll took the jet 2 deg past the cap,
					// pitch priority took the stabs back and the roll was cut and restored in a cycle.
					if (LowCapLatFix && Plugin.LowCapRollFix && LowThrustCapOn && alpha > 0f)
					{
						gateC = Mathf.Max(gateC, Mathf.Clamp01((alpha - (authCapEff - LowCapCoupBand)) / Mathf.Max(0.5f, LowCapCoupBand)));
					}
					gateC = Mathf.Max(gateC, (Plugin.AsymRollCoupling ? CoupHeldGate : 0f) * Mathf.Clamp01((Mathf.Abs(alpha) - 4f) / 4f) * Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / CoupHeldDySpan));
					// 2.1.10: at any AoA limit (not only the low-thrust cap) the gate stays on within CoupLimBand of the limit, and it
					// comes off over CoupGateRelease s instead of at once. It used to follow pitchNeed, which flips 0 <-> 1 as the pitch
					// error comes and goes: the roll limit switched off and on every ~0.6 s, the roll rate (and its inertial pitch-up)
					// jumped 60 <-> 110 deg/s, the nozzles went stop to stop and the pitch rate bounced (13 Oct logs, 45 deg limit,
					// 89-97 s: q 0..62 deg/s, AoA 39..52).
					if (CoupGateAtLimit && Plugin.CoupGateAtLimitOn)
					{
						float limU = Mathf.Min(Plugin.AoAPos, Mathf.Min(authCapEff, asymCapPosEff));
						float limD = Mathf.Max(Plugin.AoANeg, authCapNegEff);
						float nearLim = (alpha >= 0f) ? Mathf.Clamp01((alpha - (limU - CoupLimBand)) / Mathf.Max(0.5f, CoupLimBand)) : Mathf.Clamp01(((limD + CoupLimBand) - alpha) / Mathf.Max(0.5f, CoupLimBand));
						// only while the pilot holds the jet at the limit (stick toward it)
						nearLim *= Mathf.Clamp01(((alpha >= 0f) ? sp : (0f - sp)) / 0.5f);
						gateC = Mathf.Max(gateC, nearLim);
						// 2.2.2 (LoadedRollSteady, off by default): in a loaded roll (stick toward the AoA side, lateral held, AoA past
						// the gate start) the coupling roll limit stays on instead of following pitchNeed, and the roll governor judges
						// the nozzles' margin as it is, not projected 0.3 s ahead (their slew alone made the projection hit zero). 13 Oct
						// 11:08 log, full aft + full roll from 690 km/h: both switched every ~0.45 s, roll target 98 <-> 172 deg/s,
						// nozzles stop to stop, pitch rate 15..50, AoA 41..48 at the limit.
						float gLoaded = Plugin.LoadedRollSteadyOn ? 1f : CoupGateLoaded;
						if (gLoaded > 0f)
						{
							float loadedC = Mathf.Clamp01(((alpha >= 0f) ? sp : (0f - sp)) / 0.5f) * Mathf.Clamp01((Mathf.Abs(sr) - 0.05f) / 0.2f) * Mathf.Clamp01((Mathf.Abs(alpha) - CoupGateA0) / 10f);
							gateC = Mathf.Max(gateC, gLoaded * loadedC);
						}
						coupGateHeld = Mathf.Max(gateC, coupGateHeld - dt / Mathf.Max(0.05f, CoupGateRelease));
						gateC = coupGateHeld;
					}
					if (gateC > 0.001f && s2c > 0.05f && kIc > 0.01f)
					{
						float tvLoC = float.MaxValue;
						float tvHiC = float.MinValue;
						for (int tvK = 0; tvK < tvc.N; tvK++)
						{
							tvLoC = Mathf.Min(tvLoC, tvc.M[tvK].y);
							tvHiC = Mathf.Max(tvHiC, tvc.M[tvK].y);
						}
						float netC = ((alpha >= 0f) ? (0f - (AsymCap.Interp(asymAlphas, asymNetDn, asymAlphas.Length, alpha) + (tvc.active ? (tvLoC * IaInv[1, 1] * 57.29578f) : 0f))) : (AsymCap.Interp(asymAlphas, asymNetUp, asymAlphas.Length, alpha) + (tvc.active ? (tvHiC * IaInv[1, 1] * 57.29578f) : 0f)));
						float aAvC = Mathf.Max(CoupRollFloor, Plugin.CouplingRollLimit * netC) * (MathF.PI / 180f);
						float pLimC = Mathf.Sqrt(2f * aAvC / (kIc * s2c)) * 57.29578f;
						coupRollLim = pLimC;
						num10 = Mathf.Lerp(num10, Mathf.Min(num10, pLimC), gateC);
						num11 = Mathf.Clamp(num11, 0f - num10, num10);
					}
				}
			}
			if (AsymRollMax > 0f && (!AsymRollHardOnly || aRollGo < aRollStop))
			{
				float asymG = Mathf.Clamp01((asymUNow - AsymRollU0) / 0.3f);
				if (asymG > 0f)
				{
					num10 = Mathf.Min(num10, Mathf.Lerp(num10, AsymRollMax, asymG));
					num11 = Mathf.Clamp(num11, 0f - num10, num10);
				}
			}
			RollGovStep(dt, num10, rollSide);
			if (rollGov < 0.999f)
			{
				num10 = Mathf.Min(num10, Mathf.Max(num10 * rollGov, RollGovMinRate));
				num11 = Mathf.Clamp(num11, 0f - num10, num10);
			}
			pullTradeDbg = 0f;
			if (pullTradePs < 998f && Plugin.PullRollTrade > 0f)
			{
				// start from the roll in force so the limit acts at once, not after it has slewed down from 999
				if (pullTradePs > num10)
				{
					pullTradePs = num10;
				}
				// 2.2.26: with a fin lost the trade stays at most PullRollTradeFinLoss (bench, both fins gone, full aft + full
				// roll at 243 kt: the full trade let the AoA reach the 40 deg cap and the sideslip went 16 -> 23 deg)
				float trK = (yawHealth >= 0.99f) ? Plugin.PullRollTrade : Mathf.Min(Plugin.PullRollTrade, PullRollTradeFinLoss);
				float trLim = Mathf.Min(num10, Mathf.Max(PullTradeFloor, pullTradePs));
				pullTradeDbg = trLim;
				// 2.6.4: the pitch covers what the roll's share leaves (see pullTradeAsk)
				tradeAlphaRate = -1f;
				// 2.7.4: in measured mode the pitch's share is the measured AoA-rise cap below (capacity less the roll's build); this
				// second cap waited for the roll to slew down before letting the pull through (11:30 log, 112.7 / 167.8 s: qCmd 24 -> 12
				// -> 44 on a re-pull with the roll still at 60-190 deg/s - the hitch)
				// (2.7.4 tried it off in measured mode: 13-17 deg on pull + roll reversals on the bench, AoA overshoots; it stays)
				// 2.8.0: the pitch share is what the measured trade computed from the asked roll, not the limit's slew
				tradeAlphaRate = (pullTradeHoldDbg && tradeAlphaNew > 0f) ? tradeAlphaNew : -1f;
				num10 = Mathf.Min(num10, trLim);
				num11 = Mathf.Clamp(num11, 0f - num10, num10);
			}
			else
			{
				tradeAlphaRate = -1f;
			}
			// 2.5.0 reference governor on the commanded roll rate (replaces the 2.4.17 cap and the 2.4.18 rudder-load gate).
			// A velocity-vector roll at AoA carries a body yaw rate of pS sin(AoA); when the roll stops or reverses the wing kills
			// the body roll in a few tenths of a second while the yaw rate is braked more slowly (rudders slewing across their
			// travel, then their yaw authority), and the yaw left over turns into sideslip at r cos(AoA). Fitted on the 30 Oct
			// flight logs (200 stops and reversals, 3-40 deg AoA, 150-420 kt), the sideslip excursion of a stop or reversal is
			//   |pS| sin(AoA) cos(AoA) x (StopSlipTau0 + StopSlipTau1 x max(0, (L - StopSlipKnee) / (1 - StopSlipKnee)))
			// with L the rudders' load for the roll's yaw (their yaw moment toward the roll's yaw side over what they have on
			// that side, StopSlipHoldTau filter). It is linear in the roll rate and did not depend on IAS or on the rudders'
			// authority in the logs (the 2.4.17 bench model, quadratic in the yaw rate over the rudder authority, held low-IAS
			// rolls that stop cleanly in game to its 30 deg/s floor). Stops with the rudders at their stop before the release
			// (14:35 log 267.7 / 271.2 / 273.8 s, 10:28 log 61.8 / 139.3 / 277.1 s: 12-16 deg) are the knee term; stops from
			// rolls settled with rudder margin were benign at 124-152 deg/s (14:35 log 85.0 / 152.9 s: 1.1 deg).
			// The governor caps the roll-rate command (not the trajectory's acceleration or braking) to the rate whose
			// predicted excursion is the margin schedule: RollStopSideslip up to StopSlipAoALo deg AoA, RollStopSideslip50 at 50,
			// RollStopSideslip65 at 65, off past StopSlipAoAEnd; RollStopSlipGain scales the prediction (1 = the median fit).
			// Reference-governor dynamics: the cap comes down at StopSlipFallK x aB / sin(AoA) (and rises at StopSlipRiseK times that) - the
			// roll deceleration the rudders' braking yaw aB could coordinate - so a falling cap is a roll the yaw can follow, not
			// a stop of its own (2.4.18's gate, engaging over 0.3 s mid-roll, braked the roll with the rudders pinned and made the
			// sideslip it was meant to prevent: 23:27 log, 96-98 s). While the trajectory is still accelerating into the roll the
			// cap does not come down: the rudders pin in every fast entry and unpin as the yaw catches up.
			stopSlipABDbg = 0f;
			stopSlipMarginDbg = 0f;
			// 2.5.3: no AoA gate here (was |sin AoA| > 0.05): the AoA-sweep cap below must hold through zero AoA - with the gate
			// the cap let go for 0.15 s as a push + roll crossed zero and the pitch rate stepped from -24 to -60 deg/s there
			// (bench, 300 km/h). The margin cap itself is unbounded near zero AoA through sc's floor.
			if (!isAI && LatGovMode)
			{
				float aAbs = Mathf.Abs(alpha);
				float gS = 1f - Mathf.Clamp01((aAbs - StopSlipAoAEnd) / 5f);
				float sgnR = (Mathf.Abs(num11) > 2f) ? Mathf.Sign(num11) : ((Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : 0f);
				// yaw moment sign that brakes the roll's yaw (the roll's yaw rate is pS sin(AoA))
				float yB = 0f - sgnR * Mathf.Sign(alpha);
				float nPos = 0f;
				float nNeg = 0f;
				// Revoker 0.3.0: the rudder and the yaw nozzle (yawEffs), each within its usable travel, the rudder faded with the fin
				CollectYawEffs();
				float zRudNow = 0f;
				foreach (YawEff yS in yawEffs)
				{
					YawRange(yS, out float zHi, out float zLo);
					nPos += Mathf.Max(0f, zHi) * yS.auth;
					nNeg += Mathf.Max(0f, 0f - zLo) * yS.auth;
					zRudNow += yS.e.MAt(yS.e.now).z * yS.auth;
				}
				// Revoker 0.3.1 (from Ifrit 0.6.0): the fin's own restoring yaw is part of the yaw axis - the probe's directional
				// stiffness k (slipStiff, deg/s^2 per deg, from the airframe's tables at the present flow) over an excursion to the
				// margin m adds k m / 2 (the reversal model with the weathervane in it peaks at m for dR^2 = (2 a m + k m^2) / c).
				// Nothing entered: the probe's number and the margin. ~10 % of the rudder + nozzle here.
				stiffCredit = (PaceStiffCredit > 0f && Plugin.RollStopSlipProbe) ? (PaceStiffCredit * Mathf.Max(0f, slipStiffDbg) * 0.5f * Mathf.Max(1f, stopSlipMarginDbg)) : 0f;
				yawCapDbg = 0.5f * (nPos + nNeg) * IaInv[2, 2] * 57.29578f + stiffCredit;
				dampCredit = (PaceDampCredit > 0f && Plugin.RollStopSlipProbe) ? (PaceDampCredit * Mathf.Max(0f, 0f - slipDampF) * 0.5f * Mathf.Abs(omega.z) * 57.29578f) : 0f;
				float nB = (yB > 0f) ? nPos : ((yB < 0f) ? nNeg : Mathf.Min(nPos, nNeg));
				// the yaw effectors' load for the roll's yaw: their yaw moment now, toward the roll's yaw side, over what they have there
				float sY = 0f - yB;
				float nDrive = (sY > 0f) ? nPos : nNeg;
				float uR = (sY != 0f && nDrive > 1E-06f) ? Mathf.Clamp01(sY * zRudNow / nDrive) : 0f;
				stopSlipUDbg = uR;
				// 2.5.2: the rudders' usage whichever side they are on - after a reversal they are still pinned the old way, and
				// the yaw axis is just as saturated for the new roll until they have crossed
				float nAbs = (zRudNow >= 0f) ? nPos : nNeg;
				float uAbs = (nAbs > 1E-06f) ? Mathf.Clamp01(Mathf.Abs(zRudNow) / nAbs) : 0f;
				stopSlipAbsF += (uAbs - stopSlipAbsF) * (dt / (StopSlipHoldTau + dt));
				float aB = Mathf.Max(1f, StopSlipYawUse * nB * IaInv[2, 2] * 57.29578f + stiffCredit);
				stopSlipABDbg = aB;
				if (gS > 0f)
				{
					// margin schedule
					float bLo = Plugin.RollStopSideslip;
					// 2.5.14: one margin at every AoA (RollStopSideslip); the 50 / 65 deg values are no longer read - a schedule by AoA
					// was the last one in the lateral law, and the fins' stall bound runs the other way and is not what carries the
					// lateral authority above 35 deg (14:05 analysis). The margin is a handling number; the yaw side is from the tables.
					float bM = bLo;
					if (Plugin.RollStopSlipProbe)
					{
						bM = Mathf.Max(1f, Mathf.Min(bM, SlipTolUse * slipTolF));
					}
					stopSlipMarginDbg = bM;
					float sc = Mathf.Max(0.02f, Mathf.Abs(num2 * num));
					// 2.21.0: the 2.5.3 AoA-rise cap ('roll first, then pitch': StopSlipPitchK x (capacity - the roll's own build x sin AoA)
					// / (pS cos AoA), gated by the rudders' yaw and the sideslip lead) is deleted. It spent the same yaw capacity the trade
					// had already divided (TradeCapShare, PullRollTrade), a second time and with the roll trajectory's acceleration taken
					// off it first, so a pull with the roll stick coming in 0.3 s later went 60 -> 24 -> 44 deg/s of pitch command where
					// the trade's own share read 26-38 (3 Nov 09:53 log, 39.9-40.5 s, 214 kt: rudders to the stop, sideslip 0.1-1.5 deg)
					// - the pitch hitch. The trade is the one accounting of the sweep's yaw demand on the sweep that grows |AoA|; the
					// roll's own build is a transient the sideslip margin pays for, not a steady demand (counted as one it is this cap
					// again: 141 deg/s of roll asked, 40 deg/s^2 of build at 12 deg AoA, 45 of capacity -> 10 deg/s of pitch).
					yawSatAlphaRate = -1f;
					yawSatAlphaRateDn = -1f;
					// 2.5.4: sized from the roll the jet has, not only the reference - when the window drops the reference the body roll
					// is still 70-120 deg/s for half a second and the sweep under it still needs the yaw (09:44 log, 324 / 328 s: the cap
					// let go and the push / pull ran at 50-60 deg/s of AoA rate over a 100 deg/s roll, 9-11 deg); and whenever a roll
					// is in progress, stick held or released (280 s: released mid-roll, cap off, 13.5 deg)
					float pSNow = Mathf.Abs(omega.x * num + omega.z * num2) * 57.29578f;
					float pSCap = Mathf.Max(Mathf.Abs(pTraj.x), pSNow);
					pushGovL = float.NaN;
					PushPlanStep(dt, num, num2, num11);
					if (Plugin.RollGovernorPitch && pSCap > 5f)
					{
						// 2.34.0 (PushGov, see the static): the sweep that grows |AoA| is the trade's (either AoA sign, held stick; a released
						// stick asks nothing of it); the sweep toward zero is the governor's push side.
						float capPullG = StopSlipPitchFree;
						if (tradeAlphaRate > 0f) capPullG = Mathf.Min(capPullG, tradeAlphaRate);
						tradeAlphaDbg = (tradeAlphaRate > 0f) ? tradeAlphaRate : 0f;
						float sG = Mathf.Abs(num2), cG = Mathf.Abs(num);
						float pSSignedG = omega.x * num + omega.z * num2;
						float sideG = (Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : Mathf.Sign(pSSignedG);
						pushGovSide = sideG;
						// the yaw that coordinates the rotation is pS sin a: its sign is the side the yaw sheds FROM
						float rSideG = sideG * Mathf.Sign(num2);
						// 2.34.1 (PushGovNoStab): the yaw the push will have is the yaw WITHOUT the stabilators - the push itself takes them for pitch
						// (21:52 log, 149.0 / 189.1 s: stabPitchFrac 0 -> 0.9 by 53 deg, the full read said 30-37 deg/s^2 of shed, 16-20 was delivered
						// at 50-30 deg; the no-stab read said 14-28). With the fins gone the stabs ARE the yaw (2.22.0's blend): by the fin health.
						float capYFull = (rSideG > 0f) ? capEst.capZeN : capEst.capZeP;
						float capYNs = (rSideG > 0f) ? capEst.capZeNsN : capEst.capZeNsP;
						float capYRaw = PushGovNoStab ? Mathf.Lerp(capYFull, Mathf.Min(capYFull, capYNs), Mathf.Clamp01(yawHealth / 0.5f)) : capYFull;
						// the body roll pS cos a grows on the rotation's side
						// (the full-travel read: the discounted one follows the roll loop's own last command - braking, it read 30; growing, 94)
						float capXRaw = (sideG > 0f) ? capEst.capXPraw : capEst.capXNraw;
						float kG = dt / (Mathf.Max(0f, PushGovCapTau) + dt);
						if (!pushGovXInit) { pushGovCapXF = capXRaw; pushGovCapYF = capYRaw; pushGovXInit = true; }
						pushGovCapXF += (capXRaw - pushGovCapXF) * kG;
						pushGovCapYF += (capYRaw - pushGovCapYF) * kG;
						float capYG = Mathf.Max(0f, TradeCapShare * pushGovCapYF);
						float capXG = Mathf.Max(0f, TradeCapShare * pushGovCapXF);
						float aCmdG = Mathf.Max(0f, (0f - Mathf.Sign(alpha)) * (qCmd - omegaV));
						bool actG = aCmdG > 2f && !isAI;
						// the body roll when the sweep began (frozen while it runs)
						if (!actG) pushGovP0 = Mathf.Abs(omega.x) * 57.29578f;
						pushGovActive = actG;
						// how far the velocity roll may grow: held stick, the stick's own target (never faster than the pilot asks); released,
						// PushGovRollK x the body roll it had when the sweep began (Tom: "the roll stops, a little slower is fine" - never faster)
						float growRawG = Mathf.Max(5f, Mathf.Lerp(Mathf.Abs(num11), PushGovRollK * Mathf.Max(10f, pushGovP0) / Mathf.Max(0.2f, cG), stickRelW));
						if (!actG || pushGovGrowF <= 0f) pushGovGrowF = growRawG;
						pushGovGrowF += (growRawG - pushGovGrowF) * kG;   // (the same 0.2 s: the stick's target steps as the law's limits move)
						float growCapG = pushGovGrowF;
						float pCeilG = growCapG * cG;   // as a body roll
						pushGovCeil = growCapG; pushGovP0Dbg = pushGovP0;
						// the roll's growth is credited only while the velocity roll is under that cap
						float capXAvail = capXG * Mathf.Clamp01((growCapG - Mathf.Abs(pTraj.x)) / Mathf.Max(1f, 0.25f * growCapG));
						// (2) over the sweep: coordinated with the body roll at its ceiling the yaw is pCeil tan a, so while the yaw is r (less what
						// PushoverSideslip absorbs at the bottom) the AoA may come no lower than a_min = atan(r_ex / pCeil); a_min falls as the yaw
						// sheds (at capY: d a_min / dt = capY pCeil / (pCeil^2 + r_ex^2)). The pace closes on a_min over PushGovTau.
						float rOnG = Mathf.Max(0f, rSideG * omega.z * 57.29578f);
						float rLeftG = Mathf.Sqrt(2f * capYG * Mathf.Max(0f, PushGovSlipK * Plugin.PushoverSideslip));
						float rExG = Mathf.Max(0f, rOnG - rLeftG);
						float aMinG = Mathf.Atan(rExG / Mathf.Max(5f, pCeilG)) * 57.29578f;
						float gapG = Mathf.Max(0f, Mathf.Abs(alpha) - aMinG);
						float capTermG = ((PushGovDecel > 0f) ? Mathf.Sqrt(2f * PushGovDecel * gapG) : (gapG / Mathf.Max(0.05f, PushGovTau))) + 57.29578f * capYG * pCeilG / Mathf.Max(1f, pCeilG * pCeilG + rExG * rExG);
						pushGovAMin = aMinG;
						// (3) the shed time: the yaw over the allowance has to be gone when the AoA is at zero - |AoA-rate| <= |a| capY / r_ex. When
						// the yaw sheds as planned, a and r_ex fall together and this is ONE constant rate through the push (rising at the bottom as
						// r_ex reaches the allowance); r_ex weighted by (1 - cos a) against SweepShedCos (2.6.6: near zero a hold buys nothing)
						float wShG = Mathf.Clamp01((1f - cG) / Mathf.Max(0.05f, SweepShedCos));
						float rExW = rExG * wShG;
						float capShedG = (rExW < 0.5f) ? StopSlipPitchFree : Mathf.Abs(alpha) * capYG / rExW;
						// The roll's ACCELERATION is never what binds (100-500 deg/s^2 against the yaw's 20-50): what binds is how far it may grow, which
						// is (2). (1), the one-tick two-axis rate (capY cos a + capX sin a) / pS, was a bench switch (PushGovInst, off; deleted 2.44.1).
						float capPaceG = Mathf.Min(capShedG, capTermG);
						float capDnG = Mathf.Max(SweepFloor, capPaceG);
						// 2.19.0 nose-hold without fins (unchanged)
						sweepHoldDbg = 0f;
						float finKcG = Mathf.Clamp01(yawHealth / 0.5f);
						if (SweepHoldFade && finKcG < 1f)
						{
							float rotNowG = Mathf.Abs(pSSignedG) * 57.29578f;
							float wRG = Mathf.Clamp01((rotNowG - SweepHoldRate) / Mathf.Max(1f, SweepHoldRateBlend));
							float wSG = Mathf.Clamp01((SweepHoldStabFrac - stabPitchFrac) / Mathf.Max(0.05f, SweepHoldStabFrac));
							float wHG = (1f - finKcG) * wRG * wSG;
							capDnG = Mathf.Lerp(capDnG, 0f, wHG);
							sweepHoldDbg = wHG;
						}
						// the sweep that grows |AoA| from negative AoA (a push through zero under a held roll): the trade, and the yaw the rotation
						// needs on the far side may build no faster than its capacity there allows, SweepYawK x capY(grow) cos a / pS (the 2.1.8
						// rule for that sweep, unchanged in kind; the roll's share is the trade's)
						if (alpha < 0f)
						{
							float capYGrow = (rSideG > 0f) ? capEst.capTrZeP : capEst.capTrZeN;
							capPullG = Mathf.Min(capPullG, Mathf.Max(SweepFloor, 57.29578f * SweepYawK * Mathf.Max(0f, capYGrow) * cG / Mathf.Max(5f, pSCap)));
						}
						yawSatAlphaRate = capPullG;
						yawSatAlphaRateDn = Mathf.Min(StopSlipPitchFree, (PushPlan && pushPlanOn) ? StopSlipPitchFree : capDnG);
						// lateral: with the commanded AoA rate toward zero (this tick's pitch command less the flight path's rotation) the
						// velocity roll may come down no faster than L, and must grow when L > 0 (applied to the roll trajectory below)
						pushGovA = aCmdG;
						if (actG)
						{
							pushGovL = (Mathf.Abs(pTraj.x) * cG * aCmdG * (MathF.PI / 180f) - Mathf.Max(0f, pushGovCapYF)) / Mathf.Max(0.05f, sG);
							// the roll side: the body roll pS cos a grows by pS sin a x AoA-rate at constant pS - the velocity roll may grow by no
							// more than (capX available - pS sin a AoA-rate) / cos a (negative at the ceiling: it must come down)
							pushGovU = Mathf.Lerp(1E+06f, (capXAvail - Mathf.Abs(pTraj.x) * sG * aCmdG * (MathF.PI / 180f)) / Mathf.Max(0.2f, cG), stickRelW);
						}
						capSweepDbg = Mathf.Min(StopSlipPitchFree, capDnG);
						capShedDbg = float.IsNaN(pushGovL) ? 0f : Mathf.Clamp(pushGovL, -999f, 999f);
						aYawShedDbg = capYG;
						sweepDelF = capXAvail;   // (telemetry 'capSm' under PushGov: the roll growth credited)
					}
					else
					{
						tradeAlphaDbg = 0f;
						sweepDelF = 0f;
						capSweepDbg = 0f;
						capShedDbg = 0f;
						aYawShedDbg = 0f;
						pushGovXInit = false;
						pushGovActive = false;
					}
					num11 = Mathf.Clamp(num11, 0f - num10, num10);
				}
			}
			pSmaxDbg = num10;
			float num12 = (0f - (1f - num9)) * sy * Plugin.PedalBeta;
			// Revoker 0.2.7 (FinLossPedalSideslip): with the fin gone the pedal asks for at most this much sideslip. 0.2.6's
			// yaw clamp counted the dead rudder, which held the pedal to ~1 deg; with the yaw tiers fixed (RudderTierLiveOnly)
			// the full 15 deg went through, and the finless airframe is unstable in yaw: bench, fins gone, AB, 1 s of full
			// pedal at 1100 km/h: sideslip 13 deg with the nozzle saturated against it, then the jet swapped ends.
			if (FinLossPedalSideslip > 0f && yawHealth < 0.999f)
			{
				float pedLim = Mathf.Lerp(Plugin.PedalBeta, Mathf.Min(Plugin.PedalBeta, FinLossPedalSideslip * Mathf.Min(1f, FinLossPedalIas * FinLossPedalIas / Mathf.Max(1f, 12.96f * ias * ias))), Mathf.Clamp01((1f - yawHealth) / 0.5f));
				num12 = Mathf.Clamp(num12, 0f - pedLim, pedLim);
			}
			betaPedalCmd = num12;
			// 2.1.0 sideslip-rate lead: the guard also looks at where the sideslip is heading, from the measured body rates
			// (kinematic sideslip rate p sin(AoA) - r cos(AoA) plus gravity's g cos(theta) sin(phi) / V). A stability-axis roll
			// needs a yaw rate of p sin(AoA); when the yaw axis cannot build it (low IAS, an AoA sweep under a roll, fins near
			// stall, the roll's own adverse yaw) the sideslip rate shows it at once, while the sideslip itself builds a second
			// later (10 Oct log: -7..-10 deg/s of sideslip rate at 2..5 deg of sideslip, departing to 30..50 deg).
			float gTermB = ((tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f);
			float bdk = (omega.x * num2 - omega.z * num) * 57.29578f + gTermB;
			bdk = bdkF.Step(bdk, SlipRateTau, dt);
			slipRateDbg = bdk;
			float bErrNow = beta - num12;
			// Revoker 0.2.14/0.2.15: the lag the lateral loops answer within, for the sideslip-hold gain (SlipGainFromLoops) and
			// the governor's look-ahead below. Yaw path: 1/KYaw + the actuator lag; roll path: 1/KRoll + the actuator lag; a path
			// that would have to deliver the hold's share beyond the spare authority of its axis counts the time that share takes
			// at the spare authority instead (capped at 1 s); cos^2 / sin^2 weighted by the kinematic split of the correction.
			{
				float lagAct = Plugin.ActuatorLagT;
				latLagYawNom = 1f / Mathf.Max(0.1f, Plugin.KYaw) + lagAct;
				float lagYaw = latLagYawNom;
				float lagRoll = 1f / Mathf.Max(0.1f, Plugin.KRoll) + lagAct;
				float rSprev = rSF.init ? Mathf.Abs(rSF.y) : 0f;
				float accTraj = Mathf.Abs(pTraj.v);
				float spareYaw = Mathf.Max(5f, aYaw - accTraj * Mathf.Abs(num2));
				float spareRoll = Mathf.Max(5f, Mathf.Min(aRollGo, aRollStop) - accTraj * Mathf.Abs(num));
				lagYaw = Mathf.Min(1f, Mathf.Max(lagYaw, rSprev * Mathf.Abs(num) / spareYaw));
				lagRoll = Mathf.Min(1f, Mathf.Max(lagRoll, rSprev * Mathf.Abs(num2) / spareRoll));
				float c2 = num * num;
				float s2 = num2 * num2;
				latLagEff = (c2 * lagYaw + s2 * lagRoll) / Mathf.Max(0.01f, c2 + s2);
			}
			// Revoker 0.2.15 (SlipGovLeadFromLoops): the governor looks ahead as far as the loops take to answer (latLagEff), not a
			// fixed 0.8 s. 30 Oct 23:45 log (0.2.14), push + roll at -42 deg AoA: the sideslip crossing zero at +11 deg/s on its
			// way back from -3.3 (the hold's own recovery, which peaked at +2.4) read as +9 deg 0.8 s out and cut the roll target
			// 79 -> 46 deg/s; the roll braked at -95 deg/s^2, the yaw command fell 7 deg/s with the yaw nozzle and rudder on their
			// stops, the yaw rate ran 6 deg/s past it and the nozzle went stop to stop; 0.35 s later the release re-built the
			// roll 46 -> 85 and the yaw ran the same way again. Over the yaw loop's own 0.33-0.47 s that crossing projects to
			// 3.6 deg: no cut. The first cut (sideslip -3.3 with -10 deg/s of rate, the yaw axis behind the sweep) goes from
			// 0.38 to ~0.85 of the roll: the hold is left to take out what the F-22E takes out there (-6.8 deg in one move).
			float leadT = (Plugin.SlipGainFromLoops && Plugin.SlipGovLeadFromLoops) ? Mathf.Min(SlipLeadT, latLagEff) : SlipLeadT;
			slipLeadDbg = leadT;
			float bErrLead = bErrNow + bdk * leadT;
			float num14 = Mathf.Clamp01((Mathf.Abs(alpha) - 8f) / 10f);
			// Ifrit 0.3.8: one low-TAS / reversed-flow factor for everything that works off the sideslip or coordinates the
			// roll with yaw (0.3.7's hold fade, moved up): 1 at speed, 0 below SlipHoldMinTAS or past 120 deg |AoA|.
			float latRevK = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - 100f) / 20f);
			latRevK = latRevK * latRevK * (3f - 2f * latRevK);
			float latHoldK = Mathf.Clamp01((tas - Plugin.SlipHoldTasMin) / Mathf.Max(1f, Plugin.SlipHoldTasSpan)) * latRevK;
			// Ifrit 0.3.9: the roll limiters (sideslip guard / governor, yaw anchor, roll-acceleration coordination cap) use
			// their own, wider band (Plugin.SlipLimTasMin -> +SlipLimTasSpan, smoothstep): 20 Oct 23:59 log, 0.3.8, full
			// stick at 13-48 m/s: roll built to 29 deg/s, then as TAS passed 18-35 m/s the guard came back with 15-16 deg of
			// sideslip the 5-10 deg/s^2 yaw axis could not take out yet (target 33 -> 15) and the yaw anchor held the roll
			// on its 10 deg/s floor from 30 m/s: ~11 deg/s for 3+ s with full stick up to 48 m/s.
			float latLimK = Mathf.Clamp01((tas - Plugin.SlipLimTasMin) / Mathf.Max(1f, Plugin.SlipLimTasSpan));
			latLimK = latLimK * latLimK * (3f - 2f * latLimK) * latRevK;
			// 2.11.0: the 2.0.15 sideslip guard (SideslipGuardStart: the stick's roll scaled down past 4 deg of sideslip error) and the
			// yaw-saturation guard (YawSaturationGuard x yawSat) are deleted. 2 Nov, 2.9.x-2.10.0 logs: the sideslip term read 1.0 in
			// 93-96 % of roll-asked ticks (Tom flew it at 40 = off from 16:37 with the 22-34 deg AoA rolls unchanged: |pS| 70 -> 71,
			// sideslip p90 5.7 -> 6.4 deg), and the yaw-saturation term only ever cut (x 0.66-0.78) where the pace window was already
			// holding the target (paceHold > 2 in 67-100 % of those ticks: reversals and stops with the roll near zero). The window is
			// the saturation guard; the asymmetric-load sideslip lead (slipGov) below still scales x.
			float x = 1f;
			// the asymmetric-load sideslip lead: backs the
			// velocity-vector roll off only while the predicted sideslip error is large AND still growing, so a sideslip that is
			// already recovering never holds the roll back
			slipGovDbg = 1f;
			// 2.1.1: neither the sideslip lead nor the yaw anchor acts in a roll reversal (pilot target against the running
			// roll): the yaw rate the old roll left behind sat on the wrong side of the anchor window and the reversal's own
			// transient sideslip read as a departure, so the new roll was held near zero for ~0.7 s and then released at once
			// (10 Oct 22:50 log, 196.9 / 293.9 / 748.8 s). Reversals use the 2.0.15 braking / reversal limits again. (No AoA
			// fade: in a push + roll the AoA sweeps through any fade band exactly while the roll builds.)
			// latched from the reversal until the new roll is half-way to its target (as RevCarry does), so the sideslip the
			// reversal leaves behind does not hold the new roll back right after the zero crossing
			float tgtPre = num11;
			if (tgtPre * pTraj.x < 0f && Mathf.Abs(pTraj.x) > 2f && Mathf.Abs(tgtPre) > 5f)
			{
				latRevSign = Mathf.Sign(tgtPre);
			}
			else if (latRevSign != 0f && (tgtPre * latRevSign < 5f || pTraj.x * latRevSign >= 0.5f * Mathf.Abs(tgtPre)))
			{
				latRevSign = 0f;
			}
			bool latRev = latRevSign != 0f;
			latRevDbg = latRev;
			// 2.1.2: negative AoA only. Both limiters were for the push + lateral departure; at positive AoA the lateral law
			// is 2.0.15's again (2.1.0 / 2.1.1 changed the positive-AoA roll feel). Full at AoA <= NegGateA0 - NegGateSpan,
			// off at AoA >= NegGateA0.
			// 2.6.0: the 2.1.0-2.1.2 push + roll limiters (sideslip-rate lead, yaw anchor) act only for an asymmetric load past the
			// rudder fade (the damage case, below); the clean jet's push + roll is the governor's
			float negGate = 0f;
			// 2.1.4: at positive AoA with an asymmetric load past the rudder fade the same two limiters act: the flaperons are
			// spent trimming the load's roll, the stabs are the only yaw source and their yaw carries roll that nothing can
			// cancel, so the roll outruns the yaw it needs (12 Oct 18:29 log: sideslip 10-24 deg at ~50 deg AoA). Clean jets
			// (lateral CG offset under CoupHeldDy0) keep the 2.0.15 law.
			if (AsymSharedLat && Plugin.AsymSharedLatOn && alpha > 0f && asymValid)
			{
				float asymG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan)) * (1f - rudderAuth);
				negGate = Mathf.Max(negGate, asymG);
			}
			negGateDbg = negGate;
			// 0.5.3: the push + roll limiters (sideslip lead, yaw anchor) also act gear-down in the air - 2.1.0 left them off with
			// the gear down, and a full roll then full push at idle, gear down, 165 kt went to -34 deg AoA and -55 deg sideslip
			// (27 Oct 12:12 log, 136-138 s). On the runway (any wheel touching) the gear-down law is unchanged.
			if (SlipLeadT > 0f && !isAI && !latRev && negGate > 0f && Mode != FcsMode.Ground && (Mode != FcsMode.GearDownRate || (Plugin.GearDownSlipProtect && !gsAnyContact)))
			{
				float growing = (bdk * bErrNow > 0f || Mathf.Abs(bErrNow) < 1f) ? 1f : 0f;
				float sg = Mathf.Clamp01(1f - (Mathf.Abs(bErrLead) - SlipGovStart) / Mathf.Max(1f, SlipGovSpan));
				sg = Mathf.Lerp(1f, sg, growing * negGate);
				slipGovDbg = sg;
				x = Mathf.Min(x, sg);
			}
			// the fast cut only when the sideslip lead is what cuts (the 2.0.15 guard terms keep their 0.25 s)
			// Ifrit 0.3.8: the sideslip guard / governor fade out with latHoldK. 20 Oct 23:42 log, MPO off, 5-18 deg AoA, 11-16
			// m/s, full right stick (338.6-339.5 s): the sideslip nothing could coordinate there built to -17 deg and the guard
			// cut the roll target 19 -> 13 -> 5 -> 2 deg/s (guard 0.7 -> 0.0): roll stick dead.
			x = Mathf.Lerp(1f, x, latLimK);
			x = guardF.Step(x, (guardF.init && x < guardF.y && slipGovDbg < 0.999f) ? GuardCutTau : 0.25f, dt);
			num11 *= x;
			// 2.1.0 yaw-anchored roll: the velocity-vector roll needs a body yaw rate of pS sin(AoA). It may only ask for a yaw
			// rate the yaw axis can reach from the yaw rate it has now within YawAnchorT (at YawAnchorK of the modelled yaw
			// authority). In a coordinated roll the yaw rate already matches and this never binds; when the AoA sweeps under a
			// held roll (full push + full lateral) or the yaw axis stalls, the roll shrinks with what the yaw can follow instead
			// of turning the AoA into sideslip.
			yawAnchorDbg = 999f;
			if (YawAnchorK > 0f && !isAI && !latRev && negGate > 0f && Mode != FcsMode.Ground && (Mode != FcsMode.GearDownRate || (Plugin.GearDownSlipProtect && !gsAnyContact)) && Mathf.Abs(num2) > 0.05f)
			{
				// 2.2.10: the yaw rate the turn itself needs (gravity's g cos(theta) sin(phi) / V, the same term the
				// sideslip-rate lead uses) is taken out first. Without it, banked past ~60 deg at low IAS the turn's own yaw
				// rate (+4.5 deg/s at 55 m/s) looked like yaw going the wrong way for a right roll at -5 deg AoA, and the
				// roll was held at the 10 deg/s floor for 5 s with full stick (19 Oct 10:00 log, 205.9-210.7 s).
				// Only near zero AoA (full above -10, gone below -20 deg), where the turn's yaw is large next to pS sin(AoA), and
				// only with the AoA steady (gone above 15 deg/s of AoA rate): in pushes with roll it let the sideslip grow
				// (bench, -50 deg at 200 km/h: 3.2 -> 7.5 deg).
				float rNow = omega.z * 57.29578f - (YawAnchorGrav ? (gTermB / Mathf.Max(0.3f, num) * Mathf.Clamp01((alpha + 20f) / 10f) * Mathf.Clamp01(1f - (Mathf.Abs(alphaDot) - 5f) / 10f)) : 0f);
				float reach = YawAnchorK * aYaw * YawAnchorT;
				float lo = (rNow - reach) / num2;
				float hi = (rNow + reach) / num2;
				if (lo > hi)
				{
					float tmp = lo;
					lo = hi;
					hi = tmp;
				}
				// keep zero inside the window (stopping the roll is always allowed) and never below the floor rate
				lo = Mathf.Min(lo, 0f - YawAnchorFloor);
				hi = Mathf.Max(hi, YawAnchorFloor);
				yawAnchorDbg = (num11 >= 0f) ? hi : lo;
				num11 = Mathf.Lerp(num11, Mathf.Clamp(num11, lo, hi), negGate * latLimK);
			}
			// 2.5.3 yaw-reach (pace) governor (RollGovernorReplaces >= 2): one rule for the pace of every lateral move - the
			// pair's body-yaw demand, pS sin(AoA) plus the sideslip correction's rS cos(AoA), may not lead the measured yaw
			// rate by more than the yaw axis can close within PaceHorizon. The reach is what the jet's yaw is doing now
			// (filtered yaw acceleration x horizon, so a yaw that is already braking opens the window ahead of itself) plus what
			// the rudders can add within the horizon from where they are (their moment tables, slewed from the present
			// deflection, at PaceRudderUse), plus PaceFloor for the loop's normal tracking error. The roll-rate target is held
			// inside that window and the trajectory runs toward it at its tuned acceleration / braking, so where the yaw keeps
			// up nothing changes and where it does not the roll reference moves at the yaw's own pace. In a coordinated roll
			// the yaw already matches and the window never binds; in a stop or reversal the roll reference comes down as the
			// yaw actually brakes instead of at the wing's rate (31 Oct 01:40 log, level 3: 720 deg/s^2 brakes at 22 deg AoA
			// asked the yaw for 270 deg/s^2 against 85-100 from pinned rudders - 10.5 / 14.2 deg at 195.4 / 198.8 s); with
			// the stick released at 65 deg AoA it does CoordinatedStop's job (214 s: a 62 deg/s rotation told to stop in
			// 0.3 s, 15.5 deg for 5 s); at negative AoA it is the 2.1.0 yaw anchor with the reach from the tables and the
			// measured yaw instead of a constant. It never asks for a roll against the one in progress: when the yaw the jet
			// has only fits a roll the other way (the AoA swept through zero under a roll), the target is zero - stop, not
			// reverse. Replaces the trajectory's yaw-coordination caps (RollEntryCoord, BrakeYawCap, RevYawCap, CoordJerk,
			// CoordinatedStop) at level 3 and the push + roll limiters at level 2.
			paceLeadDbg = 0f;
			paceHoldDbg = 0f;
			rateBlendEffDbg = 0f;
			paceAccelCap = -1f;
			paceTgtRaw = num11;
			paceGateZDbg = 0f;
			paceGateXDbg = 0f;
			// 2.35.3: with the lateral stick released in a planned push-over the plan's bleed owns the velocity roll (target 0 at the bleed
			// rate, set below); the stop window's hold and pacing were computed and then overwritten every tick, so they stand down
			// Revoker 0.7.0 (PushPlanStopsRoll, Tom 5 Nov 01:10: "slow down the pitch and stop the roll during that pitchdown / AoA transient"): under a
			// planned push the velocity roll bleeds to zero over the plan's time with the lateral stick held too, not only released. The yaw that
			// coordinates a held roll is what the push has to shed before zero AoA (r = pS sin AoA); held, the roll kept making it.
			// 00:59 log, 73.1 s: a -60 deg corkscrew with the stick moved to a positive one, roll held at 70 deg/s, 63 deg/s of yaw on
			// the yaw shedding at 16 deg/s^2 - 2.2 s from -40 deg to zero and 13 deg of sideslip; 00:23 log 232 / 257 s, 17.6 / 24.6 deg.
			// The roll comes back to the stick when the plan ends (below 10 deg AoA).
			bool pushBleed = PushPlan && pushPlanOn && (stickRelW > 0.5f || PushPlanStopsRoll) && Mathf.Abs(pTraj.x) > 0.5f;
			bool pushHeld = PushPlanHeld && PushPlan && pushPlanOn && !pushBleed && Mathf.Abs(pTraj.x) > 0.5f;
			bool winDown = pushBleed && PushPlanWinOff;
			if (winDown)
			{
				paceStopNF = 0f;   // (the no-fin stop; inert here: the plan needs both fins)
			}
			if (!winDown && !isAI && LatGovMode && tas > 40f && Mathf.Max(Mathf.Abs(num2), Mathf.Abs(Mathf.Sin((alpha + alphaDot * PaceHorizon) * (MathF.PI / 180f)))) > PaceSinMin)
			{
				float rM = omega.z * 57.29578f;
				float rDotRaw = paceInit ? ((omega.z - paceOmegaZPrev) / Mathf.Max(dt, 1E-04f) * 57.29578f) : 0f;
				float rDot = paceRDotF.Step(rDotRaw, PaceAccelTau, dt);
				paceRDotLast = rDot;
				float gPos = 0f;
				float gNeg = 0f;
				float nAvPos = 0f;
				float nAvNeg = 0f;
				float iz = IaInv[2, 2] * 57.29578f;
				// Revoker 0.3.0: the rudder and the yaw nozzle (yawEffs), each from where it is at its own slew, faded by its auth
				CollectYawEffs();
				foreach (YawEff yP in yawEffs)
				{
					Effector seP = yP.e;
					float zNow = seP.MAt(seP.now).z * yP.auth;
					float zAtMax = seP.MAt(yP.hi).z * yP.auth;
					float zAtMin = seP.MAt(yP.lo).z * yP.auth;
					float rate = Mathf.Max(1f, seP.rate);
					// yaw the effector can still add toward each side from where it is, no more than one side's worth
					float nSide = 0.5f * Mathf.Abs(zAtMax - zAtMin);
					nAvPos += Mathf.Min(nSide, Mathf.Max(0f, Mathf.Max(zAtMax, zAtMin) - zNow));
					nAvNeg += Mathf.Min(nSide, Mathf.Max(0f, zNow - Mathf.Min(zAtMax, zAtMin)));
					for (int dirP = -1; dirP <= 1; dirP += 2)
					{
						float tgt = ((zAtMax >= zAtMin) == (dirP > 0)) ? yP.hi : yP.lo;
						float dd = Mathf.Abs(tgt - seP.now);
						float ts = Mathf.Min(PaceHorizon, dd / rate);
						float dh = seP.now + Mathf.Sign(tgt - seP.now) * rate * ts;
						float aS = Mathf.Max(0f, dirP * (seP.MAt(dh).z * yP.auth - zNow)) * iz;
						// the yaw acceleration ramps with the slew to ts, then holds to the horizon
						float g = 0.5f * aS * ts + aS * (PaceHorizon - ts);
						if (dirP > 0)
						{
							gPos += g;
						}
						else
						{
							gNeg += g;
						}
					}
				}
				// Revoker 0.4.1: the reach over every allocator effector (PaceReachAll)
				if (PaceReachAll)
				{
					ReachAll(out float gPA, out float gNA, out float nPA, out float nNA);
					gPos = gPA;
					gNeg = gNA;
					nAvPos = nPA / Mathf.Max(1E-06f, iz);
					nAvNeg = nNA / Mathf.Max(1E-06f, iz);
				}
				float reachPos = Mathf.Max(0f, rDot * PaceHorizon + PaceRudderUse * gPos) + PaceFloor;
				float reachNeg = Mathf.Max(0f, 0f - rDot * PaceHorizon + PaceRudderUse * gNeg) + PaceFloor;
				// 2.6.11 (PaceMarginReach): the sideslip margin still unused is reach too. A yaw-rate lead L that the yaw closes at a
				// builds L^2 cos(AoA) / (2 a) of sideslip, so the lead whose sideslip stays inside margin - |beta| is
				// sqrt(2 a (margin - |beta|) / cos AoA), a = PaceYawUse of what the rudders can add toward that side (gPos / gNeg
				// over the horizon, as a rate) - the same yaw the window already credits. Measured sideslip, so the window holds
				// only as the margin is actually being used (02 Nov 00:54 log, 340 kt corkscrew entry: roll held 70 -> 17 deg/s at
				// 45-55 deg AoA with the sideslip at 0.8-2.2 of a 4 deg margin).
				paceMarginLeadDbg = 0f;
				if (PaceMarginReach)
				{
					float mLeft = Mathf.Max(0f, Mathf.Max(1f, stopSlipMarginDbg) - Mathf.Abs(beta - betaPedalCmd));
					float cAm = Mathf.Max(0.2f, Mathf.Abs(num));
					float aP = PaceYawUse * Mathf.Max(1f, gPos / Mathf.Max(0.02f, PaceHorizon));
					float aN = PaceYawUse * Mathf.Max(1f, gNeg / Mathf.Max(0.02f, PaceHorizon));
					float lP = PaceMarginK * Mathf.Sqrt(2f * aP * mLeft / cAm);
					float lN = PaceMarginK * Mathf.Sqrt(2f * aN * mLeft / cAm);
					paceMarginLeadDbg = Mathf.Max(lP, lN);
					reachPos += lP;
					reachNeg += lN;
				}
				// 2.7.0 (PaceMeasured): the reach from what the axes are doing and what the allocator has already committed
				float pMm = omega.x * 57.29578f;
				float pDotRawM = paceInit ? ((omega.x - paceOmegaXPrev) / Mathf.Max(dt, 1E-04f) * 57.29578f) : 0f;
				float pDotM = pacePDotF.Step(pDotRawM, PaceAccelTau, dt);
				float dzH = 0f;
				float dzAll = 0f;
				float dxH = 0f;
				float dxAll = 0f;
				float gateZ = 0f;
				float gateX = 0f;
				float rHm = rM;
				float pHm = pMm;
				float ix = IaInv[0, 0] * 57.29578f;
				float hdZP = 0f;
				float hdZN = 0f;
				float hdXP = 0f;
				float hdXN = 0f;
				paceNEffDbg = 0f;
				paceDdDbg = 0f;
				foreach (Effector seM in effs)
				{
					if (seM == null || !seM.active || seM.M == null || seM.N < 2)
					{
						continue;
					}
					// the increment in flight: from where the effector is to where the allocator sent it, and how much of
					// that its servo covers within the horizon
					float dd = seM.cmd - seM.now;
					paceNEffDbg += 1f;
					paceDdDbg += Mathf.Abs(dd);
					float rateM = Mathf.Max(1f, seM.rate);
					float ts = Mathf.Min(PaceHorizon, Mathf.Abs(dd) / rateM);
					float dh = seM.now + Mathf.Sign(dd) * rateM * ts;
					Vector3 mNow = seM.MAt(seM.now);
					Vector3 mH = seM.MAt(dh);
					Vector3 mT = seM.MAt(seM.cmd);
					dzH += (mH.z - mNow.z) * iz;
					dzAll += (mT.z - mNow.z) * iz;
					dxH += (mH.x - mNow.x) * ix;
					dxAll += (mT.x - mNow.x) * ix;
					// 2.7.1: yaw and roll still available toward each side from the commanded deflections (the open capacity)
					Vector3 mMax = seM.MAt(seM.max);
					Vector3 mMin = seM.MAt(seM.min);
					hdZP += Mathf.Max(0f, Mathf.Max(mMax.z, mMin.z) - mT.z) * iz;
					hdZN += Mathf.Max(0f, mT.z - Mathf.Min(mMax.z, mMin.z)) * iz;
					hdXP += Mathf.Max(0f, Mathf.Max(mMax.x, mMin.x) - mT.x) * ix;
					hdXN += Mathf.Max(0f, mT.x - Mathf.Min(mMax.x, mMin.x)) * ix;
				}
				// the axis's demonstrated acceleration (a decaying peak), credited where nothing is committed yet
				aYawSeen = Mathf.Max(Mathf.Abs(rDot), Mathf.Max(PaceFloor / PaceHorizon, aYawSeen - aYawSeen * dt / PaceSeenTau));
				aRollSeen = Mathf.Max(Mathf.Abs(pDotM), Mathf.Max(PaceFloor / PaceHorizon, aRollSeen - aRollSeen * dt / PaceSeenTau));
				rHm = rM + (rDot + 0.5f * dzH) * PaceHorizon;
				pHm = pMm + (pDotM + 0.5f * dxH) * PaceHorizon;
				// gravity's share of the sideslip: the jet falling sideways under the sideslip loop's own correction (its time
				// constant 1 / KBeta). Tom: the one acceptable failure - excluded from the margin, never paid for with roll.
				float gBm = ((tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f);
				paceBetaG += (gBm - Mathf.Max(0.2f, Plugin.KBeta) * paceBetaG) * dt;
				paceBetaG = Mathf.Clamp(paceBetaG, -20f, 20f);
				// gravity is charged only for sideslip that exists and has its sign (the loop may be correcting it already)
				float bErrM = beta - betaPedalCmd;
				float bGEff = (bErrM * paceBetaG > 0f) ? Mathf.Sign(bErrM) * Mathf.Min(Mathf.Abs(bErrM), Mathf.Abs(paceBetaG)) : 0f;
				float betaOwn = bErrM - bGEff;
				float mLeftM = Mathf.Max(0f, Mathf.Max(1f, stopSlipMarginDbg) - Mathf.Abs(betaOwn));
				paceMLeftDbg = mLeftM;
				float cAmM = Mathf.Max(0.2f, Mathf.Abs(num));
				// the yaw's capacity toward each side: no more than the allocator is delivering of the loop's demand (demand less
				// its unmet report), no more than the axis is measurably doing plus the committed increment arriving within the
				// horizon; at least a share of what it has shown it can do (so a move can start)
				float desZ = omegaDotDes.z * 57.29578f;
				float capZP = Mathf.Min(Mathf.Max(0f, desZ - Mathf.Max(0f, allocUnmet.z)), Mathf.Max(0f, rDot + dzH));
				float capZN = Mathf.Min(Mathf.Max(0f, 0f - desZ - Mathf.Max(0f, 0f - allocUnmet.z)), Mathf.Max(0f, 0f - (rDot + dzH)));
				capZP = Mathf.Max(capZP, PaceSeenUse * aYawSeen);
				capZN = Mathf.Max(capZN, PaceSeenUse * aYawSeen);
				// 2.7.1: while the allocator reports nothing unmet toward a side, the capacity there is the authority still available
				// (so the loop is let ask for it); it falls to what is being delivered as the allocator reports it cannot. 2.7.0 used
				// only the delivered / demonstrated figure, and the loop's demand is proportional to the lead the window allows, so
				// the yaw was never asked for more than it had shown: a reversal at 413 kt sat at zero roll for 0.3 s with the rudders
				// at 14 of 35 deg and unmet 0, and one at 369 kt rolled back toward the old side (Tom, 09:59 log).
				// 2.7.2: saturation relative to the axis's authority (an unmet of 15 deg/s^2 is nothing at 400 kt and most of the yaw at
				// 200 km/h, where aYaw is 23)
				float satScZ = Mathf.Max(PaceSatDead, PaceSatFrac * aYaw);
				float satZP = Mathf.Clamp01(Mathf.Max(0f, allocUnmet.z) / satScZ);
				float satZN = Mathf.Clamp01(Mathf.Max(0f, 0f - allocUnmet.z) / satScZ);
				// the open capacity: what the axis is doing plus the committed increment arriving within the horizon, plus the
				// authority still available beyond the commanded deflections (zero with the effectors sent to their stops: then the
				// move is paced by what arrives, the slew case of the 154 s / 339 kt reversal)
				float openZP = Mathf.Max(0f, rDot + dzH) + PaceYawUse * hdZP;
				float openZN = Mathf.Max(0f, 0f - (rDot + dzH)) + PaceYawUse * hdZN;
				capZP = Mathf.Lerp(Mathf.Max(capZP, openZP), capZP, satZP);
				capZN = Mathf.Lerp(Mathf.Max(capZN, openZN), capZN, satZN);
				// 2.7.4: never above the allocator's own authority less its saturation derate (the 2.5.12 ceiling: past the rudder
				// fade the tables overstate the stabs' yaw 3-4x, and the committed increment is read from those tables). 11:30 log,
				// 116 s: a push-over from 64 deg AoA read 143-162 of capacity with 26-129 unmet and made 22 deg of sideslip.
				float capCeilZ = Mathf.Max(5f, aYaw) * (1f - Mathf.Lerp(PaceSatDerateLo, PaceSatDerateHi, 1f - rudderAuth) * Mathf.Clamp01(yawSat));
				capZP = Mathf.Min(capZP, capCeilZ);
				capZN = Mathf.Min(capZN, capCeilZ);
				// the lead that capacity closes inside the margin left (L^2 cos AoA / 2a = margin), plus the loop's ordinary
				// tracking error, which shrinks with the margin so that at the margin the roll waits for the yaw
				float flM = PaceFloor * Mathf.Clamp01(mLeftM / 2f);
				float lPm = PaceMarginK * Mathf.Sqrt(2f * capZP * mLeftM / cAmM);
				float lNm = PaceMarginK * Mathf.Sqrt(2f * capZN * mLeftM / cAmM);
				// never less than what the capacity closes within the horizon: the loop keeps driving the axis at its capacity
				// with the margin spent, instead of pinning the target to the axis (which asks it for nothing and stalls the move)
				// 2.7.1: the floor is what the yaw loop needs as error to ask for that capacity (its gain KYaw; a held target has no
				// feed-forward), so at the margin the lead x gain = the delivered capacity - the loop keeps asking exactly what the
				// axis gives. 2.7.0's capacity x 0.15 s asked for a fraction of it.
				// 2.7.1: the lead is the margin's (plus PaceLeadT of the capacity, the sideslip rate accepted while the roll waits); the
				// yaw is driven by the loop's feed-forward instead - the target moves toward the demand at the capacity (below), so the
				// loop asks exactly what the axis gives with no lead at all
				// 2.17.0: the lead floor is the error the yaw loop needs to ask for the whole capacity, capacity / KYaw (the 2.7.1 comment
				// above; the code had 0.1 s of capacity against a 2.5 /s loop, a quarter of it, and 2.7.2 switched off the feed-forward that
				// covered the rest). With the margin spent and the yaw weak (no fins, 65 deg) the target was pinned 1 deg/s ahead of the
				// rotation and the yaw was asked for ~5 deg/s^2 of its 20 (23:30 log, 110 s reversal: 2.5 s at +19 against a -30 demand).
				float leadTw = (PaceLeadT < 0f) ? (1f / Mathf.Max(0.5f, Plugin.KYaw)) : PaceLeadT;
				reachPos = Mathf.Max(lPm, capZP * leadTw) + flM;
				reachNeg = Mathf.Max(lNm, capZN * leadTw) + flM;
				paceMarginLeadDbg = Mathf.Max(lPm, lNm);
				paceCapZPDbg = capZP;
				paceCapZNDbg = capZN;
				// the roll axis the same way (its shortfall is sideslip at sin AoA)
				float desX = omegaDotDes.x * 57.29578f;
				float capXP = Mathf.Min(Mathf.Max(0f, desX - Mathf.Max(0f, allocUnmet.x)), Mathf.Max(0f, pDotM + dxH));
				float capXN = Mathf.Min(Mathf.Max(0f, 0f - desX - Mathf.Max(0f, 0f - allocUnmet.x)), Mathf.Max(0f, 0f - (pDotM + dxH)));
				float satScX = Mathf.Max(PaceSatDead, PaceSatFrac * aRoll);
				float satXP = Mathf.Clamp01(Mathf.Max(0f, allocUnmet.x) / satScX);
				float satXN = Mathf.Clamp01(Mathf.Max(0f, 0f - allocUnmet.x) / satScX);
				float openXP = Mathf.Max(0f, pDotM + dxH) + PaceRollUse * hdXP;
				float openXN = Mathf.Max(0f, 0f - (pDotM + dxH)) + PaceRollUse * hdXN;
				paceCapXPDbg = Mathf.Lerp(Mathf.Max(capXP, PaceSeenUse * aRollSeen, openXP), Mathf.Max(capXP, PaceSeenUse * aRollSeen), satXP);
				paceCapXNDbg = Mathf.Lerp(Mathf.Max(capXN, PaceSeenUse * aRollSeen, openXN), Mathf.Max(capXN, PaceSeenUse * aRollSeen), satXN);
				paceCapXPDbg = Mathf.Min(paceCapXPDbg, Mathf.Max(5f, aRollPos));
				paceCapXNDbg = Mathf.Min(paceCapXNDbg, Mathf.Max(5f, aRollNeg));
				// 2.7.2 (Tom, 10:19): the window does not hold the target in measured mode - the commanded steady rate is the only
				// lever (the cap below), and a reversal from it runs at the tuned acceleration / braking whatever the yaw does.
				// The window binds only while the allocator reports the axis cannot deliver what the loop asks (airframe-wide, any
				// effector): a latch on the unmet report, on past PaceSatDead + PaceSatSpan x PaceGateOn, off below x PaceGateOff -
				// never a blend toward the raw target (2.7.0's partial gate leaked it).
				float satZm = Mathf.Max(satZP, satZN);
				if (satZm > PaceGateOn)
				{
					paceLatchZ = true;
				}
				else if (satZm < PaceGateOff)
				{
					paceLatchZ = false;
				}
				gateZ = paceLatchZ ? 1f : 0f;
				float satXm = Mathf.Max(satXP, satXN);
				if (satXm > PaceGateOn)
				{
					paceLatchX = true;
				}
				else if (satXm < PaceGateOff)
				{
					paceLatchX = false;
				}
				gateX = paceLatchX ? 1f : 0f;
				paceHdZPDbg = hdZP;
				paceHdZNDbg = hdZN;
				paceDzHDbg = dzH;
				paceDzAllDbg = dzAll;
				paceDxHDbg = dxH;
				paceDxAllDbg = dxAll;
				paceRHDbg = rHm;
				pacePHDbg = pHm;
				paceSeenZDbg = aYawSeen;
				paceReachDbg = reachPos;
				// the sideslip correction's share of the body yaw (last tick's rS, velocity-vector yaw)
				float yawCorr = rSDbg * num;
				// the demand is judged at the AoA the jet will have at the end of the horizon (an AoA sweep under the roll moves
				// the yaw the roll needs at pS cos AoA x AoA rate; a push + roll through zero AoA reverses it)
				float aH = (alpha + alphaDot * PaceHorizon) * (MathF.PI / 180f);
				float sinH = Mathf.Sin(aH);
				// 2.5.4: inside +-PaceSinMin of zero AoA the roll's share of the yaw demand (pS sin AoA) is too small for any roll to
				// settle a yaw error - the yaw loop owns it - so the window is off there (2.5.3 divided by a floored sin and turned a
				// leftover yaw rate at the zero crossing into roll demands of 400-570 deg/s, then a flip to zero: 09:44 log, 318 / 324
				// / 328 s)
				bool winOn = Mathf.Abs(sinH) >= PaceSinMin;
				float sinD = winOn ? sinH : (PaceSinMin * ((sinH >= 0f) ? 1f : -1f));
				float lo = (rM - reachNeg - yawCorr) / sinD;
				float hi = (rM + reachPos - yawCorr) / sinD;
				if (lo > hi)
				{
					float tmpP = lo;
					lo = hi;
					hi = tmpP;
				}
				// the roll axis the same way: the pair's body-roll demand (pS cos AoA - rS sin AoA) may not lead the measured roll
				// rate by more than the roll axis can close within the horizon (its acceleration now, plus PaceRollUse of the
				// roll authority on that side). Past ~40 deg AoA the roll axis is the weak one: the yaw delivers its share and the
				// roll does not, and that too is sideslip (bench, full aft + roll at 400 km/h, 42 deg AoA: p 1 against 30 asked,
				// r 22, sideslip -9).
				float pM = pMm;
				float pDot = pDotM;
				float cosH = Mathf.Cos(aH);
				float loR = float.MinValue;
				float hiR = float.MaxValue;
				if (Mathf.Abs(cosH) > PaceSinMin)
				{
					float rollPos = Mathf.Max(0f, pDot * PaceHorizon + PaceRollUse * aRollPos * PaceHorizon) + PaceFloor;
					float rollNeg = Mathf.Max(0f, 0f - pDot * PaceHorizon + PaceRollUse * aRollNeg * PaceHorizon) + PaceFloor;
					// 2.7.0: measured; a roll shortfall is sideslip at sin AoA, so the margin buys sqrt(2 a (margin - |beta|) / sin AoA) of roll-rate lead
					float sAmR = Mathf.Max(0.05f, Mathf.Abs(num2));
					float flR = PaceFloor * Mathf.Clamp01(paceMLeftDbg / 2f);
					rollPos = Mathf.Max(PaceMarginK * Mathf.Sqrt(2f * paceCapXPDbg * paceMLeftDbg / sAmR), paceCapXPDbg * PaceLeadT) + flR;
					rollNeg = Mathf.Max(PaceMarginK * Mathf.Sqrt(2f * paceCapXNDbg * paceMLeftDbg / sAmR), paceCapXNDbg * PaceLeadT) + flR;
					float rollCorr = 0f - rSDbg * num2;
					loR = (pM - rollNeg - rollCorr) / cosH;
					hiR = (pM + rollPos - rollCorr) / cosH;
					dbgWinLoR = loR; dbgWinHiR = hiR; dbgRollPos = rollPos;
					if (loR > hiR)
					{
						float tmpR = loR;
						loR = hiR;
						hiR = tmpR;
					}
					// intersect; when the two axes disagree the yaw window stands (the sideslip is what is protected)
					if (loR <= hi && hiR >= lo)
					{
						lo = Mathf.Max(lo, loR);
						hi = Mathf.Min(hi, hiR);
					}
					else
					{
						// 2.7.0: the windows do not overlap - one axis is ahead of the other. The laggard is the one whose implied velocity
						// roll is further from the stick's demand; its window stands, so the other axis is brought back to it (a yaw that
						// cannot follow holds the roll down to it; a roll that cannot follow holds the yaw back to it)
						float pSrM = (rM - yawCorr) / sinD;
						float pSpM = (pM - rollCorr) / cosH;
						if (Mathf.Abs(num11 - pSpM) > Mathf.Abs(num11 - pSrM))
						{
							lo = loR;
							hi = hiR;
						}
					}
				}
				// never a roll against the one in progress (or against the one asked for, from rest): stop instead
				float sideP = (Mathf.Abs(pTraj.x) > 2f) ? Mathf.Sign(pTraj.x) : Mathf.Sign(num11);
				if (sideP > 0f && hi < 0f)
				{
					lo = 0f;
					hi = 0f;
				}
				else if (sideP < 0f && lo > 0f)
				{
					lo = 0f;
					hi = 0f;
				}
				// 2.7.0: the gate is the measured shortfall (allocator unmet + undelivered slew) per axis, nothing from the rudders' load
				// (2.12.1: the 2.5.x rudder-usage gate and its airframe-usage variant, overwritten here since 2.7.0, are deleted)
				float wSat = gateZ;
				paceGateF = Mathf.Max(wSat, paceGateF - dt / Mathf.Max(0.02f, PaceGateTau));
				wSat = winOn ? paceGateF : 0f;
				paceGateXF = Mathf.Max(gateX, paceGateXF - dt / Mathf.Max(0.02f, PaceGateTau));
				paceGateZDbg = wSat;
				paceGateXDbg = paceGateXF;
				float tgtRaw = num11;
				paceTgtRaw = tgtRaw;
				// 2.5.11: the two laws are blended by RollPaceRateBlend (0 = 2.5.7 pacing: window + acceleration cap; 1 = 2.5.8 rate
				// cap; between = both, in proportion). RollPaceRateMode false = blend 0.
				// 2.6.9: the rate cap below is a model of the RUDDERS - their crossing time to the far stop, their tables - so it holds
				// only where the rudders are the yaw axis. Past the rudder fade (rudderAuth -> 0, 50-60 deg AoA here) it read the
				// faded rudders' crossing against the stabs' authority and sat on PaceRateFloor (65 deg / 300 km/h: cap 10 against an
				// airframe limit of 36), and at 0.4 that held the steady falling-leaf command to ~70 % of the limit while the
				// window alone kept the reversal sideslip the same (5.7 vs 5.5 deg). So the blend counts where its model is valid:
				// your RollPaceRateBlend at full where the rudders have their authority, fading to the window alone as they fade.
				// 2.7.2: in measured mode the cap counts at every AoA (its capacity is the whole airframe's), at Tom's blend
				// 2.7.2: in measured mode the cap's capacity is the whole airframe's; its share is Tom's blend, fading with the rudders
				// (past their fade the yaw is the velocity roll and the one-axis law's own braking owns the reversal: no falling-leaf tax)
				float rateBlend = Plugin.RollPaceRateBlend * Mathf.Clamp01(rudderAuth);
				rateBlendEffDbg = rateBlend;
				float num11Rate = num11;
				float pSRateCap = 9999f;
				if (rateBlend > 0f)
				{
					// 2.5.8 rate mode: no window, no acceleration cap. The target is capped at the roll rate whose full stick
					// reversal at the tuned acceleration / braking the yaw can follow within the sideslip margin (your rule: slow
					// the commanded rate, not the move). Yaw-rate swing of the reversal: dR = 2 pS |sin AoA|. The rudders cross
					// from where they are to the other stop (tCross, slew), then pull dR down at their one-side yaw aOpp:
					// sideslip ~ cos AoA x (dR tCross / 2 + dR^2 / (2 aOpp)) x PaceRateK. Solve for dR at the margin, cap pS.
					float aOpp = 0f;
					float tCross = 0f;
					// Revoker 0.3.0: the rudder (faded with the fin) and the yaw nozzle, each crossing from where it is at its own
					// slew; the crossing time the reversal waits for is the yaw-weighted mean of theirs (the two rudders of the F-22E
					// were one time). No allocator-wide credit past the rudder fade: the nozzle is the yaw axis there (see yawEffs).
					float tCrossW = 0f;
					foreach (YawEff yP in yawEffs)
					{
						Effector seP = yP.e;
						float aE = 0.5f * Mathf.Abs(seP.MAt(yP.hi).z - seP.MAt(yP.lo).z) * iz * yP.auth;
						aOpp += aE;
						// the crossing: from where the effector is now to the stop on the far side of center (the reversal side)
						float far = (Mathf.Abs(seP.now - yP.hi) > Mathf.Abs(seP.now - yP.lo)) ? yP.hi : yP.lo;
						tCrossW += aE * Mathf.Abs(far - seP.now) / Mathf.Max(1f, seP.rate);
					}
					tCross = (aOpp > 1E-06f) ? (tCrossW / aOpp) : 0f;
					// Revoker 0.4.0: the fin's restoring yaw over the excursion (see stiffCredit)
					aOpp += stiffCredit + dampCredit;
					// 2.5.12: less what the allocator reports it cannot deliver
					aOpp *= 1f - Mathf.Lerp(PaceSatDerateLo, PaceSatDerateHi, 1f - rudderAuth) * Mathf.Clamp01(yawSat);
					float bMrg = Mathf.Max(1f, stopSlipMarginDbg);
					// 2.7.2 (Tom, 10:19): the capacity a reversal can call on is the whole airframe's, not the rudders': every effector's
					// yaw still available toward the reversal's side from where the allocator has it (rudders, stabs, the Ifrit's roll TVC
					// - whatever the allocator can turn to yaw), less what it reports unmet, plus the airframe's own directional
					// stiffness at the margin sideslip (the passive share: the airframe balancing laterally at this AoA). The dead time
					// is the yaw-weighted travel of those effectors to the deflection the reversal needs.
					float sideRev = 0f - Mathf.Sign(num11 * sinH);
					float passive = PaceStiffUse * Mathf.Max(0f, slipStiffDbg) * bMrg;
					// 2.8.1: the reversal's capacity is the estimator's figure toward the reversal side (measured yaw acceleration now, the
					// committed increment arriving within the horizon, and what the allocator itself would still give - 1.2-1.4x the realised
					// yaw in the 12:31 flight where the table headroom read 1.8x), plus the passive stiffness at the margin
					aOpp = Mathf.Max(5f, ((sideRev >= 0f) ? capEst.capZeP : capEst.capZeN) + passive);
					tCross = 0f;
					float wT = 0f;
					foreach (Effector seA in effs)
					{
						if (seA == null || !seA.active || seA.M == null || seA.N < 2)
						{
							continue;
						}
						Vector3 mA = seA.MAt(seA.max);
						Vector3 mB = seA.MAt(seA.min);
						float yA = Mathf.Abs(mA.z - mB.z) * iz;
						if (yA < 5f)
						{
							continue;
						}
						float farA = ((mA.z >= mB.z) == (sideRev > 0f)) ? seA.max : seA.min;
						tCross += yA * Mathf.Abs(farA - seA.cmd) / Mathf.Max(1f, seA.rate);
						wT += yA;
					}
					tCross = (wT > 1f) ? (tCross / wT) : 0f;
					rcAOppDbg = aOpp;
					float cH = Mathf.Max(0.2f, Mathf.Abs(cosH));
						// 2.5.10: RollPaceRateSlip (Dev key) scales the margin the cap aims at - the roll rate / sideslip compromise in flight
						float cQ = 2f * bMrg * Plugin.RollPaceRateSlip / Mathf.Max(1f, aOpp * cH * Mathf.Max(0.1f, PaceRateK));
					float dR = aOpp * (0f - 0.5f * tCross + Mathf.Sqrt(0.25f * tCross * tCross + cQ));
					// 2.6.9 (PaceRateRamp): the reversal the cap models is the one the law will fly, not a step. The demanded yaw
					// rate swings at aT |sin AoA|, aT = the trajectory's reversal acceleration as the law sets it (RevFloorK x roll
					// authority, or the brake law) bounded by the governor's own open-loop cap for the reversing side - the num17
					// the trajectory gets. The yaw answers after tCross / 2 at aOpp. Sideslip ~ the area between the two:
					//   E(dR) = dR tau + dR^2 / 2 (1/aOpp - 1/aD),  tau = tCross / 2
					// which is the 2.5.8 step model exactly as aD -> infinity. Where the yaw out-accelerates the demand it catches
					// the ramp and tracks it: E saturates at aD tm tau / 2 and, inside the margin, the cap does not bind. Measured
					// state and the law's own numbers only. (Bench, blend 0.4: full-aft reversal 320 km/h 6.6 -> 3.6 deg, worst of
					// the regression set 6.6 -> 4.3; Ifrit 0.7.2 found the same 4x pessimism in the step model.)
					if (PaceRateRamp)
					{
						float cA = Mathf.Max(Mathf.Abs(num), 0.05f);
						float aLaw = (ReversalFloor && RevConst) ? (RevFloorK * aRoll / cA) : (Plugin.RollBrakeUse * (BrakeUseAll ? aRollGo : aRollWing) / cA);
						// 2.7.2: in measured mode the trajectory runs at the law's own acceleration (no open-loop cap)
						float aT = Mathf.Clamp(aLaw, 20f, Plugin.MaxRollAccel);
						float aD = aT * Mathf.Max(Mathf.Abs(sinH), PaceSinMin);
						float tau = 0.5f * tCross * RateCapTauK;
						// 2.6.9 (RateCapNeedTravel): the rudders do not have to reach the far stop before the yaw turns - only the
						// deflection that makes the yaw acceleration the reversal asks for (aD of their one-side aOpp), from where they
						// sit, through center. At high dynamic pressure that is a few degrees (corkscrew entry at 700 km/h: rudders at
						// +-8..13 deg, the far-stop crossing put the cap at 25-30 deg/s where reversals from 208 made 3.9 deg); at
						// moderate q it is most of the stroke and nothing changes.
						if (RateCapNeedTravel)
						{
							// the effectors need only the deflection that makes aD of their aOpp, through center: about (0.5 + 0.5 need) of the stroke
							float needM = Mathf.Clamp01(aD / Mathf.Max(1f, aOpp));
							tau = 0.5f * tCross * Mathf.Lerp(0.5f, 1f, needM) * RateCapTauK;
						}
						else if (RateCapNeedTravel)
						{
							float need = Mathf.Clamp01(aD / Mathf.Max(1f, aOpp));
							float tN = 0f;
							foreach (SurfaceEffector seN in surfaces)
							{
								if (seN.kind != SurfaceKind.Rudder || !seN.active || seN.M == null || seN.N < 2)
								{
									continue;
								}
								float cN = 0.5f * (seN.max + seN.min);
								float hN = 0.5f * (seN.max - seN.min);
								float farN = (Mathf.Abs(seN.now - seN.max) > Mathf.Abs(seN.now - seN.min)) ? seN.max : seN.min;
								float dN = Mathf.Min(Mathf.Abs(farN - seN.now), Mathf.Abs(seN.now - cN) + need * hN);
								tN = Mathf.Max(tN, dN / Mathf.Max(1f, seN.rate));
							}
							tau = 0.5f * tN * RateCapTauK;
						}
						float qM = bMrg * Plugin.RollPaceRateSlip / Mathf.Max(0.01f, cH * Mathf.Max(0.1f, PaceRateK));
						float kQ = 0.5f * (1f / Mathf.Max(1f, aOpp) - 1f / Mathf.Max(1f, aD));
						float disc = tau * tau + 4f * kQ * qM;
						dR = (disc > 0f && tau + Mathf.Sqrt(disc) > 1E-04f) ? (2f * qM / (tau + Mathf.Sqrt(disc))) : 1E+06f;
					}
					pSRateCap = Mathf.Max(PaceRateFloor, dR / (2f * Mathf.Max(Mathf.Abs(sinH), PaceSinMin)));
					rcAOppDbg = aOpp;
					rcTcDbg = tCross;
					rcSinHDbg = sinH;
					if (RevSim)
					{
						// Revoker 0.6.0: the forward model, its reversal paced at the yaw the estimator says the effectors can add (as the Ifrit's)
						float aPaceRev = Mathf.Max(20f, PaceYawUse * aOpp / Mathf.Max(Mathf.Abs(sinH), PaceSinMin));
						pSRateCap = RevSimCap(bMrg * Plugin.RollPaceRateSlip, cosH, sinH, aPaceRev);
					}
					num11Rate = Mathf.Clamp(num11, 0f - pSRateCap, pSRateCap);
				}
				// the window may hold the roll in progress while the yaw comes down (that is the paced brake / reversal) but never
				// asks for more roll than the stick or the roll in progress, and never for a roll against the one in progress
				float clamped;
				// 2.19.0: where the rudders are the yaw axis (rudW -> 1) the band holds the roll at the yaw's rotation; where they are not
				// (faded past ~55 deg, or gone) the yaw effectors ARE the roll effectors (the stabs' differential, the flaperons) and a
				// roll loop holding the roll against them starves the yaw brake: 3 Nov 00:09 log, both fins gone, 82.4 s reversal at 65
				// deg - target held at 0 for 3.8 s, one stab at -21 and the other at 0, 8 deg/s^2 of yaw against a measured 24 when the
				// roll went with the yaw (23:49 log, 62 s). There the target is paced toward the demand at the yaw's capacity instead.
				float rudWwin = Mathf.Clamp01(rudderAuth) * Mathf.Clamp01(yawHealth / 0.5f);
				float clampedBand = Mathf.Clamp(num11, lo, hi);
				if (PaceHoldPacedByRudder && rudWwin < 0.999f)
				{
					// 2.17.0: while the yaw is reported saturated (the latch), the target is PACED toward the demand at the yaw's measured
					// capacity (the 2.7.1 limiter, on the estimator's fin-aware figure), not clamped to the band. The band's lead scales
					// with the sideslip margin LEFT, so with the margin spent it pinned the target to the rotation the jet had; the yaw loop
					// then had no error to brake with and nothing moved until the unmet report happened to fall under the latch's release
					// (23:30 log, both fins gone, 65 deg: reversal at 110 s held at +19 deg/s for 2.5 s against a -30 demand, zero crossing
					// 4.0 s after the stick; held push-over at 157 s cut the roll target -47 -> -10..-23 while coordination needed -30..-54,
					// sideslip +9 deg/s, departure). A move toward the axes (less lead) is free; a move away is paced at capacity / sin AoA.
					float pSrNow = (rM - yawCorr) / sinD;
					float from = paceRateInit ? paceRatePrev : pTraj.x;
					float dirT = Mathf.Sign(num11 - from);
					float capZd = Mathf.Max(PaceHoldCapFloor, (dirT * sinD > 0f) ? capEst.capZeP : capEst.capZeN);
					float rateT = capZd / Mathf.Max(Mathf.Abs(sinD), PaceSinMin);
					bool away = (num11 - pSrNow) * dirT > 0f;
					float lim = from + dirT * rateT * dt;
					float clampedPaced = (away && (num11 - lim) * dirT > 0f) ? lim : num11;
					clamped = Mathf.Lerp(clampedPaced, clampedBand, rudWwin);
				}
				else
				{
					clamped = clampedBand;
				}
				// 2.18.0: the window may HOLD the target where it is; it may never move it back toward the rotation (away from the stick's
				// demand). 23:49 log (both fins gone, 65 deg), 59.8 s reversal: the first 0.3 s of braking took the body roll 6 -> 1 deg/s
				// while the yaw (30 deg/s, slow to shed) still implied a 33 deg/s rotation; the band set the target at rotation - lead = 25
				// (p = 10.6 coordinated), the roll loop drove +X the OLD way for 1.2 s and took the stabs and flaperons from the yaw brake
				// (tgtX +47..+58 against a -30 demand); once the target came down the yaw braked at 24 deg/s^2. Tom rejected the same
				// behaviour in 2.7.0 ('keeps rolling the old way for a moment then reverses'). The target only ever moves toward the demand.
				if (PaceNoRetreat && paceRateInit && Mathf.Abs(num11 - paceRatePrev) > 0.01f)
				{
					float dirD = Mathf.Sign(num11 - paceRatePrev);
					if ((clamped - paceRatePrev) * dirD < 0f)
					{
						clamped = paceRatePrev;
					}
				}
				// 2.20.0 (PaceStopNoFin): without the rudders the yaw sheds at the stabs' pace (20-25 deg/s^2 at 65 deg) whatever the
				// roll does; a velocity-roll target of 0 against a yaw still carrying the rotation asks the body roll for -r tan AoA
				// (anti-coordinated), and the roll loop's try at it is the reversal's sideslip (bench leaf rev 370 no fins: p +11 -> -10
				// while r went 24 -> 7, sideslip 23 deg). Here the target may not go below the rotation the jet has with the body roll
				// stopped (r sin AoA, falling with the yaw): p -> 0, the nose yaws through, the sideslip grows at r cos AoA only.
				// The yaw loop is asked for its brake capacity directly (below, on rCmd), since this target alone gives it ~0.18 r.
				// Scaled by the FIN loss, not the rudder fade: with the fins on above the fade the sideslip the old stop builds is what
				// weathercocks the yaw round (bench 65 deg fullaft reversal 500: 1.6 s to zero; held to r sin AoA it took 3.1 s) - the
				// fins are the yaw brake there, and the clean jet keeps its reversal.
				paceStopNF = 0f;
				float finKw = Mathf.Clamp01(yawHealth / 0.5f);
				float stopNF = (PaceStopNoFin && finKw < 0.999f && sideP != 0f) ? Mathf.Max(0f, sideP * rM * sinD) : 0f;
				if (sideP != 0f && clamped * sideP < stopNF)
				{
					clamped = Mathf.Lerp(sideP * stopNF, 0f, finKw);
					paceStopNF = (stopNF > 0f) ? sideP : 0f;
				}
				else
				{
					float limP = Mathf.Max(Mathf.Abs(num11), Mathf.Abs(pTraj.x));
					clamped = Mathf.Clamp(clamped, 0f - limP, limP);
				}
				paceLoDbg = lo;
				paceHiDbg = hi;
				paceClampDbg = clamped;
				float num11Win = Mathf.Lerp(num11, clamped, wSat);
				dbgWinLo = lo; dbgWinHi = hi; dbgReachPos = reachPos; dbgReachNeg = reachNeg; dbgYawCorr = yawCorr; dbgWSat = wSat; dbgRateCap = pSRateCap; dbgWin = clamped; dbgRDot = rDot; dbgPDot = pDot;
				// the paced target continues from the target actually flown (the raw one while the latch is off)
				paceRatePrev = num11Win;
				paceRateInit = true;
				// 2.6.1: climbing back out of a hold at the yaw's pace (see paceWinHeld)
				paceRelDbg = 0f;
				if (PaceReleasePace > 0f)
				{
					float dirRel = Mathf.Sign(num11Win * sinH);
					float relAcc = PaceReleasePace * Mathf.Max(PaceFloor / PaceHorizon, PaceYawUse * Mathf.Max(dirRel * (rDot + dzH), PaceSeenUse * aYawSeen)) / Mathf.Max(Mathf.Abs(sinD), PaceSinMin);
					if (num11Win * paceWinPrev < 0f)
					{
						paceWinSame = false;
					}
					if (!paceWinHeld)
					{
						paceWinSame = Mathf.Abs(pTraj.x) > 2f && pTraj.x * num11Win > 0f;
					}
					if (paceWinHeld && paceWinSame && num11Win * paceWinPrev >= 0f && Mathf.Abs(num11Win) > Mathf.Abs(paceWinPrev))
					{
						// the measured yaw already supports a roll of (r - correction) / sin AoA: up to that the climb is free (no yaw
						// demand leads the yaw there); the pacing applies to the part above it
						float pSup = (rM - yawCorr) / sinD;
						float lim = Mathf.Max(Mathf.Abs(paceWinPrev) + relAcc * dt, (pSup * num11Win > 0f) ? Mathf.Abs(pSup) : 0f);
						if (Mathf.Abs(num11Win) > lim)
						{
							paceRelDbg = Mathf.Abs(num11Win) - lim;
							num11Win = Mathf.Sign(num11Win) * lim;
						}
					}
					paceWinHeld = Mathf.Abs(num11Win) < Mathf.Abs(tgtRaw) - 1f;
					paceWinPrev = num11Win;
				}
				else
				{
					paceWinHeld = false;
					paceWinSame = false;
					paceWinPrev = num11Win;
				}
				// blend: the window's hold and the rate cap's trim of the target, in proportion
				// 2.6.12 (RateCapOnReversal): the cap's share of the target applies only while a reversal is under way - the stick
				// asking for the opposite direction to the roll in progress. It predicts what a reversal from the present rate would
				// cost, and with no reversal it was trimming the steady roll for one that never came (02 Nov 00:54 log, 340 kt
				// corkscrew entry: roll 70 -> 17 deg/s at 45-55 deg AoA, sideslip 0.8-2.2 of a 4 deg margin; bench, the window
				// holds 0 there and the whole hold is the cap's 41 %)
				float revW = 1f;
				num11 = Mathf.Lerp(num11Win, num11Rate, rateBlend * PaceBlendTargetK * revW);
				paceLeadDbg = (rateBlend >= 0.5f) ? pSRateCap : ((sideP >= 0f) ? reachPos : reachNeg);
				paceHoldDbg = tgtRaw - num11;
				// 2.10.0: the 2.5.x open-loop acceleration cap (rudder tables) went with the table governor; the trajectory runs at the law's own acceleration
				paceAccelCap = -1f;
			}
			else
			{
				paceRDotF.init = false;
				pacePDotF.init = false;
				paceRateInit = false;
				paceGateF = 0f;
				paceWinHeld = false;
				paceWinSame = false;
				paceWinPrev = 0f;
				paceRelDbg = 0f;
			}
			paceOmegaZPrev = omega.z;
			paceOmegaXPrev = omega.x;
			paceInit = true;
			guardDbg = x;
			// 2.34.0 (PushGov): the lateral limit of the push side. Where the law would hold or bring the rotation down (stop, release,
			// reversal, a stick target under the rotation) and the yaw cannot shed the sweep's demand, the trajectory comes down no faster
			// than -L; where L > 0 it grows at L (the roll follows the yaw down)
			// rate band [L, U] for the velocity-roll reference; outside it the trajectory is re-aimed (target) and its acceleration set to the
			// band's edge. L > U (the pitch ran ahead of both axes this tick - the cap catches it next tick): the roll's edge, U.
			bool pushGovOn = !float.IsNaN(pushGovL) && pushGovSide != 0f && !pushBleed && !pushHeld;
			if (pushBleed) num11 = 0f;
			float pushGovRate = 0f;
			if (pushGovOn)
			{
				bool lawBrakes = pushGovSide * (num11 - pTraj.x) < 0f;
				float num11Law = num11;
				float lo = pushGovL, hi = Mathf.Max(pushGovL, pushGovU);
				if (pushGovL > pushGovU) lo = hi = pushGovU;
				// the law's own rate sign: braking (fast), or holding / growing (toward its target)
				if (lawBrakes && lo > -1E+05f)
				{
					// braking: as fast as the law wants, but no faster than -lo; if lo > 0 the reference grows at lo instead
					pushGovRate = lo;   // applied as min(num17, |lo|) below when lo < 0
					if (lo > 0f) num11 = pushGovSide * Mathf.Max(Mathf.Max(Mathf.Abs(pTraj.x), Mathf.Min(Mathf.Abs(pTraj.x) + 30f, pushGovCeil)), pushGovSide * num11);
					else if (hi < 0f) { }   // the law's brake is at least as fast as needed? (num17 >= |hi| normally)
				}
				else if (hi < 0f)
				{
					// holding or growing where the roll's ceiling says it must come down: brake at |hi|
					pushGovRate = hi;
					num11 = pushGovSide * Mathf.Max(0f, Mathf.Abs(pTraj.x) - 30f);
				}
				else if (lo > 0f)
				{
					// holding where the yaw says it must grow: grow at lo (no faster than hi)
					pushGovRate = Mathf.Min(lo, hi);
					num11 = pushGovSide * Mathf.Max(Mathf.Max(Mathf.Abs(pTraj.x), Mathf.Min(Mathf.Abs(pTraj.x) + 30f, pushGovCeil)), pushGovSide * num11);
				}
				else
				{
					pushGovOn = false;   // the law's own move is inside the band
				}
				// faded in over 2..8 deg/s of commanded AoA rate toward zero (an AoA held at a limit flickers around zero: no band there)
				float wGt = Mathf.Clamp01((pushGovA - 2f) / 6f);
				num11 = Mathf.Lerp(num11Law, num11, wGt);
			}
			pSTgtDbg = num11;
			float num15 = Mathf.Abs(pTraj.x * num);
			float max = Mathf.Lerp(Plugin.RollAccelUseLowIas, Plugin.RollAccelUse, Mathf.InverseLerp(60f, 110f, ias));
			float a = Mathf.Clamp(0.85f - num15 / Mathf.Max(30f, pFull * (aRollGo / aRoll)), 0.1f, max) * aRollGo / Mathf.Max(Mathf.Abs(num), 0.05f);
			float num16 = 0.6f * aYaw / Mathf.Max(Mathf.Abs(num2), 0.05f);
			// 2.6.0: the trajectory's yaw-coordination caps (RollEntryCoord, BrakeYawCap, RevYawCap, CoordJerk) are gone; the pace
			// governor's acceleration cap (paceAccelCap) is the only pacing of the trajectory
			float num17 = (latAccDbg = a);
			// 1.8.41: the roll entry's acceleration yields with the governor too (the entry is where the surfaces deflect most)
			num17 *= Mathf.Lerp(RollGovAccelFloor, 1f, rollGov);
			if (pTraj.x * (num11 - pTraj.x) < 0f)
			{
				float num18 = aRoll / Mathf.Max(30f, pFull);
				num17 = (Plugin.RollBrakeUse * (BrakeUseAll ? aRollGo : aRollWing) + num18 * num15) / Mathf.Max(Mathf.Abs(num), 0.05f);
				// 2.7.0 (PaceRevIntent): a reversal the stick asked for is a reversal, even while the measured window holds the target on
				// the near side - it brakes at the reversal's tuned acceleration, not the stop's
				bool revIntent = PaceRevIntent && paceTgtRaw * pTraj.x < 0f && Mathf.Abs(paceTgtRaw) > 10f;
				if (((num11 * pTraj.x < 0f && Mathf.Abs(num11) > 10f) || revIntent) && ReversalFloor)
				{
					float num19 = RevFloorK * aRoll / Mathf.Max(Mathf.Abs(num), 0.05f);
					num17 = (revA = (RevConst ? num19 : Mathf.Max(num17, num19)));
					revTarget = revIntent ? paceTgtRaw : num11;
				}
			}
			else if (revA > 0f && RevCarry)
			{
				if (num11 * revTarget <= 0f || Mathf.Abs(num11) < 10f)
				{
					revA = 0f;
				}
				else
				{
					float num20 = 1f - Mathf.Clamp01(Mathf.Abs(pTraj.x) / Mathf.Max(10f, 0.5f * Mathf.Abs(num11)));
					num17 = Mathf.Max(num17, Mathf.Lerp(num17, revA, num20));
					if (num20 <= 0f)
					{
						revA = 0f;
					}
				}
			}
			// 2.1.6: with an asymmetric load, a roll brake / reversal may only accelerate as fast as the yaw left after the load
			// can follow (the roll's yaw rate p sin AoA has to reverse with it; with the rudders already deflected for the load
			// they slew across their whole travel). 12 Oct 19:30 log, 144 s: a 110 deg/s reversal at 30 deg AoA left the yaw
			// 0.7 s behind, sideslip -11 deg, and the guard and governor cut the roll (target 116 -> 20 deg/s). Clean jets unchanged.
			loadRevDbg = 0f;
			if (AsymSharedLat && Plugin.AsymSharedLatOn && asymValid && LoadRevYawK > 0f && pTraj.x * (num11 - pTraj.x) < 0f)
			{
				float dyG = Mathf.Clamp01((Mathf.Abs(asymDy) - CoupHeldDy0) / Mathf.Max(0.005f, CoupHeldDySpan));
				if (dyG > 0f)
				{
					float revCap = LoadRevYawK * aYaw * Mathf.Max(0.2f, sharedLatDbg) / Mathf.Max(Mathf.Abs(num2), 0.05f);
					loadRevDbg = dyG;
					num17 = Mathf.Lerp(num17, Mathf.Min(num17, revCap), dyG);
				}
			}
			if (paceAccelCap > 0f)
			{
				num17 = Mathf.Min(num17, paceAccelCap);
			}
			num17 = Mathf.Clamp(num17, 20f, Plugin.MaxRollAccel);
			if (pushGovOn)
			{
				// faded in over 2..8 deg/s of commanded AoA rate (as the target above)
				float wG = Mathf.Clamp01((pushGovA - 2f) / 6f);
				num17 = Mathf.Lerp(num17, Mathf.Min(num17, Mathf.Abs(pushGovRate)), wG);
			}
			pushHeldB = 0f;
			if (pushBleed)
			{
				num17 = Mathf.Max(0.5f, pushPlanBleed);
			}
			else if (pushHeld && pTraj.x * (num11 - pTraj.x) < 0f)
			{
				// held stick, the roll moving toward a lower target (the law's own: the stick, the pace window): no faster than the plan's
				// yaw budget leaves after the AoA's share, and no faster than the law's own brake
				float aDotP = Mathf.Max(0f, (0f - Mathf.Sign(alpha)) * (qCmd - omegaV));
				pushHeldB = Mathf.Max(0f, pushPlanD - Mathf.Abs(pTraj.x) * Mathf.Abs(num) * aDotP * (MathF.PI / 180f)) / Mathf.Max(0.05f, Mathf.Abs(num2));
				num17 = Mathf.Max(0.5f, Mathf.Min(num17, pushHeldB));
			}
			float a2 = Mathf.Min(num17 / RollJerkTau, RollJerkServo * aRoll * rollSlew / Mathf.Max(Mathf.Abs(num), 0.05f));
			if (isAI && Plugin.AIDirectRoll)
			{
				float aDir = Mathf.Min(AIRollDirectAccel, 1.3f * num16);
				num17 = Mathf.Max(num17, aDir);
				a2 = Mathf.Max(a2, aDir / 0.05f);
			}
			// 2.0.6 coordinated roll jerk: the stability-axis roll needs a yaw rate of p sin(a); the rudders reach full travel
			// no faster than their servo allows (35 deg at 100 deg/s), so the yaw acceleration can only build at about
			// aYaw x (rudder rate / half travel). The roll's jerk is held to what keeps that yaw build-up within reach
			// (CoordJerkK of it), below the rudder fade only (rudderAuth), so roll entries and reversals at moderate AoA no
			// longer outrun the yaw axis and leave sideslip behind.
			float jFloor = 100f;
			coordJerkDbg = a2;
			latAccDbg = num17;
			latJerkDbg = Mathf.Max(a2, jFloor);
			float endJ = RollEndJerk * Mathf.Lerp(1f, EndJerkHi, Mathf.InverseLerp(EndJerkLp0, EndJerkLp1, aRoll / Mathf.Max(30f, pFull)));
			lpDbg = aRoll / Mathf.Max(30f, pFull);
			// Ifrit 0.2.0, Stability Assist off: lateral stick = body roll rate, pedals = body yaw rate, two independent axes.
			// No sideslip hold, no pedal-to-roll coupling, no automatic coordination: the jet rolls about its velocity vector
			// only when the pilot puts in roll and yaw together. The rate limits, the damage / asymmetric-load limits on
			// them and the allocator's VMC / asymmetry trim are unchanged.
			if (mpoSep)
			{
				// Revoker 0.2.7 (Tom, 28 Oct): Stability Assist off = the stick rolls about the velocity vector at every AoA and
				// the pedals are pure body yaw rate at every AoA. 0.2.6 (the Ifrit 0.2.0 law) made the stick body roll and only
				// the part of it shared with a same-direction pedal a velocity-vector roll; below the coupling AoA the yaw axis
				// then kept the sideslip inside +-PedalSideslip (and washed it out), which coordinated a stick-only roll there
				// anyway (velocity-vector roll below ~36 deg, body roll above), and clamped the pedal's sideslip the same way.
				// Now: stick = the coupled law's stability-axis roll (its roll limits and acceleration), with the sideslip hold
				// and gravity term while the stick is in and the pedals are centred; pedals = body yaw rate on top, no sideslip
				// bound, no washout, no hold (the hold fades out with pedal so it never fights it).
				float cRaw = Mathf.Abs(sr);
				float cSgn = Mathf.Sign(sr);
				if (!mpoSepPrev)
				{
					pTraj.Reset(pCmdPrev * num + rCmdPrev * num2);
					mpoPTraj.Reset(0f);
					mpoRTraj.Reset(0f);
					mpoCoord = 1f;
				}
				mpoSepPrev = true;
				mpoCoord = Mathf.MoveTowards(mpoCoord, cRaw * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.3f)), 4f * dt);
				// velocity-vector roll from the stick
				float pSc = sr * num10;
				pTraj.Step(pSc, num17, Mathf.Max(a2, jFloor), Mathf.Max(a2 * Mathf.Min(endJ, 1f), jFloor), dt);
				float gTermM = (tas > 40f) ? (9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - 40f) / 20f)) : 0f;
				// Ifrit 0.3.7: the sideslip hold fades out at very low TAS / in reversed flow as in the coupled law
				float holdM = Mathf.Clamp01((tas - Plugin.SlipHoldTasMin) / Mathf.Max(1f, Plugin.SlipHoldTasSpan));
				float revM = 1f - Mathf.Clamp01((Mathf.Abs(alpha) - 100f) / 20f);
				holdM *= revM * revM * (3f - 2f * revM);
				slipHoldDbg = holdM;
				float rSc = mpoCoord * rSF.Step(Mathf.Clamp(Plugin.KBeta * holdM * beta, 0f - num8, num8) + gTermM, 0.12f, dt);
				rSDbg = rSc;
				// no body roll of its own any more: the body-roll trajectory only winds down what a switch left in it
				float aB = Mathf.Clamp(Plugin.RollBrakeUse * aRollStop, 20f, Plugin.MaxRollAccel);
				float jB = Mathf.Max(aB / RollJerkTau, jFloor);
				mpoPTraj.Step(0f, aB, jB, jB, dt);
				// pure body yaw from the pedals (no sideslip limit on them). With the pedals centred and below the coupling AoA the
				// sideslip still washes out slowly (MpoSideslipWashout, like the airframe's own weathervane, which a held zero yaw
				// rate would otherwise block: bench, MPO roll and release, sideslip frozen at 57 deg).
				float rBodyT = sy * num8;
				float wB = (1f - Mathf.Clamp01((Mathf.Abs(alpha) - Plugin.CoupleAoA) / 14f)) * (1f - mpoCoord) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.05f));
				if (wB > 0f)
				{
					rBodyT = Mathf.Lerp(rBodyT, Mathf.Clamp(Plugin.MpoSlipWashout * beta + gTermM, 0f - num8, num8), wB);
				}
				float aR = Mathf.Max(10f, 0.7f * aYaw);
				mpoRTraj.Step(rBodyT, aR, Mathf.Max(aR / 0.15f, 60f), dt);
				pS = pTraj.x;
				float pM = pTraj.x * num - rSc * num2 + mpoPTraj.x;
				float rM = pTraj.x * num2 + rSc * num + mpoRTraj.x;
				float scM = Mathf.Max(1f, Mathf.Max(Mathf.Abs(pM) / num3, Mathf.Abs(rM) / num8));
				pCmd = pM / scM;
				rCmd = rM / scM;
				pDotFF = (pTraj.v * num + mpoPTraj.v) / scM;
				rDotFF = (pTraj.v * num2 + mpoRTraj.v) / scM;
				if (Mathf.Abs(omega.z * 57.29578f) > num8 && rDotFF * omega.z > 0f)
				{
					rDotFF = 0f;
				}
				pCmdPrev = pCmd;
				rCmdPrev = rCmd;
				SlipHistClear();
				return;
			}
			if (mpoSepPrev)
			{
				// back to the coupled law: the stability-axis roll rate the body rates make now
				pTraj.Reset(pCmdPrev * num + rCmdPrev * num2);
				mpoSepPrev = false;
			}
			pTraj.Step(num11, num17, Mathf.Max(a2, jFloor), Mathf.Max(a2 * Mathf.Min(endJ, 1f), jFloor), dt);
			pS = pTraj.x;
			// Revoker 0.2.4 (CorkscrewYawShaping): with roll stick held above 30-45 deg AoA (pedals centred) the velocity-
			// vector roll is mostly body yaw (sin AoA), and its yaw part, pS sin(AoA), carried every step of the limits (the
			// braking floor's IAS prediction, the authority estimates) and every AoA change straight into the yaw rate. It is
			// passed through a critically damped second-order filter (CorkYawOmega, rad/s; acceleration cap CorkYawAccel) and
			// the roll part is derived from the same filtered value, so roll and yaw stay coordinated about the velocity vector.
			float wCork = (CorkYawOmega > 0f && Mode == FcsMode.AoAG && !isAI) ? (Mathf.Clamp01((Mathf.Abs(sr) - 0.3f) / 0.4f) * Mathf.InverseLerp(CorkShapeA0, CorkShapeA0 + 15f, alpha) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f))) : 0f;
			float rTgtC = pS * num2;
			if (wCork <= 0.001f)
			{
				corkR = rTgtC;
				corkRd = 0f;
			}
			else
			{
				float accC = CorkYawOmega * CorkYawOmega * (rTgtC - corkR) - 2f * CorkYawOmega * corkRd;
				accC = Mathf.Clamp(accC, 0f - CorkYawAccel, CorkYawAccel);
				corkRd += accC * dt;
				corkR += corkRd * dt;
				float rLimC = num10 * Mathf.Abs(num2);
				if (Mathf.Abs(corkR) > rLimC)
				{
					corkR = Mathf.Sign(corkR) * rLimC;
					corkRd = 0f;
				}
				pS = Mathf.Lerp(pS, corkR / Mathf.Max(0.5f, num2), wCork);
			}
			corkWDbg = wCork;
			float num21 = 0f;
			if (tas > GravFFTas0)
			{
				num21 = 9.81f * Mathf.Sin(phi * (MathF.PI / 180f)) * Mathf.Cos(theta * (MathF.PI / 180f)) / tas * 57.29578f * Mathf.Clamp01((tas - GravFFTas0) / Mathf.Max(1f, GravFFSpan));
				num21 = Mathf.Clamp(num21, 0f - GravFFMax, GravFFMax);
			}
			float num22 = ((Mode == FcsMode.GearDownRate) ? 1f : Plugin.KBeta);
			// Ifrit 0.3.0 (SlipTvcAssist): above 25 deg AoA the sideslip gain ramps to SlipGainHighAoA by 45 deg. There the
			// correction is mostly body roll and the nozzles supply it at any IAS. Bench, 65 deg corkscrew with a 1 s full-pedal
			// kick, 140-150 km/h: back under 3 deg sideslip in 4.3-4.7 s at KBeta 1.3, 0.5-0.8 s at 4.
			// Revoker 0.2.1 (SlipYawTvcAssist): the single nozzle has no roll authority (aRollTvc 0), so the high-AoA gain never
			// ramped and the sideslip loop ran at KBeta at 70 deg AoA; the yaw nozzle carries the yaw part of the correction at any
			// IAS below the TVC cutoff, so it opens the ramp too
			bool yawTvcSlip = Plugin.SlipYawTvcAssist && tvcy != null && tvcy.active;
			if (Mode != FcsMode.GearDownRate && Plugin.SlipGainFromLoops)
			{
				// Revoker 0.2.14 (SlipGainFromLoops): the sideslip-hold gain is what the rate loops carrying the correction can
				// follow. rS is flown as body yaw rS cos(a) through the yaw-rate loop (KYaw: 2.5/s = 0.4 s) and as body roll
				// -rS sin(a) through the roll-rate loop (KRoll: 6/s = 0.17 s), each plus the actuator lag. The sideslip loop
				// closes through them as gain/s x lag (beta-dot = -rS), so its phase margin is set by gain x lag: SideslipGain
				// (1.3) is the gain the yaw loop supports (1.3 x 0.47 s), and where the correction rides the faster roll loop
				// the gain rises with the shorter lag, cos^2 / sin^2 weighted by the kinematic split. A path that would have
				// to deliver its share beyond the spare authority of its axis (the roll axis braking or saturated for the
				// velocity-vector roll itself) counts the time that share takes at the spare authority instead of 1/K, capped
				// at 1 s. 30 Oct 22:48 log, full push + full roll, -8 -> -55 deg AoA, 78 -> 55 kt: the fixed SlipGainHighAoA
				// ramp (4, opened on this jet by the yaw nozzle in 0.2.1) ran the loop through the 0.4 s yaw loop at ~15 deg of
				// phase margin: sideslip -3.4 <-> +3.6 deg at ~1 Hz, yaw rate +-12 deg/s about its command, rudder stop to
				// stop, the roll governor cutting the roll twice (the hitch). The F-22E, at 1.3 there, recovers -6.8 deg in one
				// move. Here: 0 deg AoA 1.3, -50 deg ~1.85, 70 deg ~2.3.
				float gLoops = Plugin.KBeta * latLagYawNom / Mathf.Max(0.02f, latLagEff);
				// pedals centred only: with pedal in, the pedal sideslip command is tracked at KBeta as before
				num22 = Mathf.Lerp(num22, gLoops, 1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f));
			}
			else if (Mode != FcsMode.GearDownRate && Plugin.SlipTvcAssist && (aRollTvc > 0.5f || yawTvcSlip))
			{
				// pedals centred only: with pedal in, the pedal sideslip command is tracked at KBeta as before
				num22 = Mathf.Lerp(num22, Mathf.Max(num22, Plugin.SlipGainHighAoA), Mathf.InverseLerp(Plugin.SlipGainAoAStart, Plugin.SlipGainAoAStart + 20f, Mathf.Abs(alpha)) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f)));
			}
			slipGainDbg = num22;
			// Ifrit 0.3.0 (SlipTvcAssist): the sideslip correction rS is flown as body roll -rS sin(a) and body yaw rS cos(a), so
			// each part is bounded on its own axis: the yaw part by the yaw-rate limit, the roll part by a stop-side roll budget
			// that counts the differential TVC. 0.2.2 bounded rS by the yaw-rate limit alone; at 65 deg AoA and 70-120 km/h that
			// is 11-14 deg/s from the aero yaw authority, the correction sat on it and the sideslip reached -15 deg (11:51 log,
			// 145-152 s) while the nozzles, the strongest roll effector there, had travel to spare. Same as 0.2.2 at low AoA.
			float slipLim = num8;
			pSlipMax = num3;
			if (Plugin.SlipTvcAssist && aRollTvc > 0.5f)
			{
				float aStopSide = Mathf.Min(aRollPos, aRollNeg);
				pSlipMax = Mathf.Max(num3, Mathf.Min(Plugin.MaxRollRate, Plugin.RollStopFactor * (aStopSide + aRollTvc)));
				slipLim = Mathf.Max(num8, Mathf.Min(num8 / Mathf.Max(num, 0.25f), pSlipMax / Mathf.Max(Mathf.Abs(num2), 0.25f)));
			}
			// Ifrit 0.3.5: sideslip predictor (dev SlipPredictTime, s; SlipPredictWashout, s). The sideslip correction acts
			// through the inner roll/yaw rate loops, which in game follow pCmd/rCmd ~0.3 s late (20 Oct logs: pCmd -> p
			// 0.19-0.32 s, rCmd -> r 0.29-0.37 s, gain ~1; the bench's are ~0.1 s, which is why the bench never showed it).
			// beta-dot = -rS, so the loop is gain / s with that delay: at the high-AoA gain of 4 it rings at omega ~ gain
			// (0.72 Hz) and with 0.33 s of delay it sits on the stability edge. 20 Oct 23:10 log (0.3.3), full thrust, roll
			// stick and pedals centred, 45-55 deg AoA, 54-75 m/s: sideslip peaks 2.9 -> 3.7 -> 4.4 deg every 1.4 s
			// (growing), decelerons +-30 deg; also 157-161 s (+-2.4 deg) and the 22:37 log at idle. The 0.3.3 sideslip-rate
			// lead could not cover 0.3 s (delay-loop model, gain 4, delay 0.3: decay per cycle 0.89 -> 0.78 with the lead;
			// it matched the logged 1.4 s period and growth) and is removed. The predictor is a Smith predictor: the rS this
			// law has already commanded over the last SlipPredictTime has not shown up in the sideslip yet, so the correction
			// works on beta - integral(commanded rS) over that window, the sideslip those commands will leave. The commanded
			// rS is the one actually sent (after the limit scaling), less the gravity term (which only cancels gravity's own
			// sideslip rate); it is washed out over SlipPredictWashout so a held correction (a yaw trim against a load) leaves
			// no extra steady sideslip. Model, gain 4, predictor at 0.3 s: decay per cycle <= 0.23 for a true delay of
			// 0.2-0.45 s and inner-loop gain 0.8-1.2 (no predictor: 0.21-2.99), settles in 0.8-5 s. Gear-down rate law and
			// the MPO separate-axes law unaffected; faded in from 20 to 40 m/s TAS. 0 = no predictor (0.3.2).
			float predT = (Mode == FcsMode.GearDownRate) ? 0f : Plugin.SlipPredictTime;
			// Ifrit 0.4.0: the roll-stick gate is a dev key (SlipPredictRollGate, default 0 = none). With the gate the predictor
			// was off in every held corkscrew, and that is exactly where the 22 Oct 00:30 log rang (+-12 deg, +-50 deg/s).
			float rollGate = Plugin.SlipPredictRollGate;
			float predGate = Mathf.Clamp01((tas - 20f) / 20f) * ((rollGate > 0f) ? (1f - Mathf.Clamp01(Mathf.Abs(sr) / rollGate)) : 1f);
			// faded in by the loop's gain x delay (its phase cost, rad at crossover): off up to 0.4 (KBeta 1.3), full from 1.0
			// (gain 3.3+). Below that the loop has its margin and the predictor only slowed the recovery (bench, rolling pull
			// at 600 km/h, 13 deg AoA, gain 2.4: -4.5 deg back to 0 in 2 s instead of 1 s).
			float predW = (predT > 0f) ? Mathf.Clamp01((num22 * predT - 0.4f) / 0.6f) : 0f;
			float slipPred = (predW > 0f) ? (predW * SlipHistSum(predT)) : 0f;
			slipPredDbg = slipPred;
			// Ifrit 0.3.7: the sideslip hold fades out at very low TAS (Plugin.SlipHoldTasMin -> +SlipHoldTasSpan) and in true
			// reversed flow (|AoA| 100 -> 120 deg); the pedal sideslip command stays, as a plain yaw-rate command. 20 Oct 23:42
			// log, MPO: tail slide at -172 deg AoA, 20-33 m/s, sticks centred: the hold (high-AoA gain 4, it ramps on |AoA|)
			// asked 25-36 deg/s of yaw against a 7 deg/s limit (yaw authority ~5 deg/s^2) and bang-banged on the unstable
			// backwards weathervane, sideslip +6.6 -> -9.8 -> +10 deg on a ~5 s cycle. 391-400 s, 8-17 m/s, full lateral stick:
			// sideslip 52 deg, the hold saturated at 27.9 deg/s, and the joint roll/yaw limit scaling (num26) took the roll
			// command from -5 to ~0 deg/s: roll stick dead. At those speeds the sideslip is not a usable signal.
			float holdK = latHoldK;
			slipHoldDbg = holdK;
			// Revoker 0.2.2 (CorkscrewSlipTol): with roll stick held at high AoA and the pedals centred, the sideslip hold ignores
			// sideslip inside a tolerance band, so the corkscrew keeps rotating instead of the hold taking the yaw back as soon as
			// gravity (g/V ~19 deg/s at 30 m/s) pushes the sideslip out. In-game 28 Oct 19:05 log, 70 deg AoA, 30-50 m/s, full
			// roll: sideslip 5-11 deg, rS 16-38 deg/s, body yaw command -58 -> -12 deg/s, velocity-vector roll 63 -> 13 deg/s.
			// The band opens at once and closes at CorkTolClose deg/s after the stick is released or the AoA drops; the
			// rolling-pull sideslip governor (PullSideslipMargin) still bounds the sideslip.
			float corkT = (Mode == FcsMode.AoAG && !isAI) ? (Plugin.CorkscrewSlipTol * Mathf.Clamp01((Mathf.Abs(sr) - 0.2f) / 0.5f) * Mathf.InverseLerp(CorkAoA0, CorkAoA0 + 15f, Mathf.Abs(alpha)) * (1f - Mathf.Clamp01(Mathf.Abs(sy) / 0.2f))) : 0f;
			corkTolNow = (corkT >= corkTolNow) ? corkT : Mathf.MoveTowards(corkTolNow, corkT, CorkTolClose * dt);
			float bHold = beta - slipPred;
			if (corkTolNow > 0f)
			{
				bHold = Mathf.Sign(bHold) * Mathf.Max(0f, Mathf.Abs(bHold) - corkTolNow);
			}
			float num23 = (rSDbg = rSF.Step(Mathf.Clamp(num22 * (holdK * bHold - num12), 0f - slipLim, slipLim) + num21, 0.12f, dt));
			float num24 = pS * num - num23 * num2;
			float num25 = pS * num2 + num23 * num;
			float pLimSc = Mathf.Max(num3, Mathf.Min(pSlipMax, Mathf.Abs(num23 * num2)));
			float num26 = Mathf.Max(1f, Mathf.Max(Mathf.Abs(num24) / pLimSc, Mathf.Abs(num25) / num8));
			pCmd = num24 / num26;
			rCmd = num25 / num26;
			// 2.20.0 (PaceStopNoFin): in the no-fin stop the yaw loop is asked for its brake capacity (the error that makes it, capacity /
			// KYaw, from the measured yaw), where the stopped-roll target alone gives it a fraction of that
			paceStopNFDbg = paceStopNF;
			if (paceStopNF != 0f)
			{
				float capB = Mathf.Max(PaceHoldCapFloor, (paceStopNF > 0f) ? capEst.capZeN : capEst.capZeP);
				float rNF = omega.z * 57.29578f - paceStopNF * capB / Mathf.Max(0.5f, Plugin.KYaw);
				float finKnf = Mathf.Clamp01(yawHealth / 0.5f);
				if ((rNF - rCmd) * paceStopNF < 0f)
				{
					rCmd = Mathf.Lerp(rNF, rCmd, finKnf);
				}
			}
			// 2.4.17 no-chase on a roll brake: while the trajectory brakes the roll (stop, reversal, or a limit coming down), the
			// yaw loop is not asked for more yaw rate than the jet already has on the same side. In a fast roll at AoA the yaw
			// trails its coordinated command (the rudders at their stop: 30 Oct 14:35 log, 267.7 s, r -27 against -36); left to
			// chase it, the yaw kept accelerating for 0.3 s after the stick came back (bench: -33 -> -43 deg/s), and all of that
			// had to be braked again by rudders slewing across their travel. Same AoA band as the roll-stop governor.
			float rCmdCoord = rCmd;
			noChaseDbg = 0f;
			// 2.4.18: only while the coordinated yaw command itself is shrinking. Under a rolling pull the trajectory brakes the
			// roll (the pull governor / trade) while the rising AoA raises the yaw the roll needs (pS sin AoA); holding the yaw there
			// built sideslip (bench, full roll then full aft at 400 km/h: 2.9 -> 4.9 deg in 2.4.17)
			// rate of the yaw the roll itself needs, |pS| sin|AoA|, from the trajectory's own acceleration and the AoA rate
			// (or the AoA not rising fast: at a steady AoA the braking trajectory alone sets it, from its first tick)
			// 2.5.15: retired with the pace governor at level 3 (bench at blend 0.41 / margin 4: stops 5.2 -> 4.1 worst, reversals
			// 8.8 -> 9.3, regressions mostly down 0.2-4 deg, one up 1.6; a wash - one device fewer)
			pDotFF = pTraj.v * num / num26;
			SlipHistPush((num23 / num26 - num21) * predGate, dt);
			float num27 = aRoll / Mathf.Max(30f, pFull);
			pCmd += Plugin.RollLead * Mathf.Max(0f, num27 - 2.5f) * pDotFF;
			float t2 = Mathf.Max(Mathf.InverseLerp(200f, 300f, ias), Mathf.InverseLerp(YawFFAoA0, YawFFAoA1, Mathf.Abs(alpha)));
			// 2.4.17: the yaw feed-forward follows the coordinated command, not the no-chase hold (its release would step it)
			rDotFF = Mathf.Lerp(Plugin.YawRateFF, 1f, t2) * rDotF.Step((rCmdCoord - rCmdCoordPrev) / dt, 0.1f, dt);
			rCmdCoordPrev = rCmdCoord;
			pCmdPrev = pCmd;
			// 2.4.18: the rate of the yaw-rate command the loop actually follows (after the no-chase hold), for the yaw slew lead
			rCmdRateF += ((rCmdLeadInit ? ((rCmd - rCmdPrev) / Mathf.Max(dt, 1E-04f)) : 0f) - rCmdRateF) * (dt / (0.05f + dt));
			rCmdLeadInit = true;
			rCmdPrev = rCmd;
			if (Mathf.Abs(omega.z * 57.29578f) > num8 && rDotFF * omega.z > 0f)
			{
				rDotFF = 0f;
			}
		}

		public void WriteHeader(TextWriter w)
		{
			w.Write("t,mode,ias,tas,rho,alpha,beta,nz,nAlpha,tw,p,q,r,sp,sr,sy,qB,qRaw,qCmd,qT,pCmd,rCmd,aUp,aDn,betaG,alphaCmd,nCmd,unload,prot,omegaV,phi,theta,tvcCmd,tvcNow,nMaxAero,alphaAtNMax,gAvail,gInt,aInt,flapBias,flapTarget,hlPos,relief,rudderAuth,yawSat,aRoll,aYaw,pMax,pStockRef,pFull,rMax,guard,pSmax,pSTarget,latAcc,rS,stabPitchFrac,gndBrake,revFlow,asymDy,asymAr,asymAy,asymU,asymCap,asymCapEff,asymCapNeg,dMeasR,dMeasY,asymCr,asymCy,asymFFr,asymFFy,asymMr,asymMy,asymFFGate,rollPitchFF,gUnder,pitchPrio,stabRateBound,momentLead,rollTrim,aRollPos,aRollNeg,latReserve,aUpPitch,aDnPitch,aBrakeTgt,stabBranch,tvcAnchor,tvcBudget,rudSide,gsRCmd,gsNws,gsRud,gsHold,gsLineY,asymCapPitch,pitchNeed,coupRollLim,rpGate,rpBand,rollGov,rollGovMargin,iasDot,qPredK,asymCapRaw,asymCapFilt,alphaCmdRaw,nCmdRaw,rateBlendAoA,hiRate,rateW,zoneX,authCap,authCapEff,nLiftMax,hybridDMacc,floatOn,floatS,rateBlendAoANeg,nLiftMin,hiSide,authCapNeg,vmcLim,engOut,fuelTrimCm,yawHealth,capIUp,capIDn,capPushU,slipRate,slipGov,yawAnchor,gRateLaw,rateCmdG,negGate,sharedLat,rollGovPred,loadRev,stickLead,capOn,capQ,sweepLim,slatRelief,hlPosNow,slipAlloc,coupGate,pitchLead,slipGovPull,decel,pullTrade,abShare,abTgt,abOpen,fightK,pullFFBoost,tvcdCmd,tvcdNow,splitL,splitR,aRollTvc,pSlipMax,slipGain,droopIn,droopOut,revSense,slipPred,slipHold,splitTier,tvcHold,qTrajV,fightGate,abGate,engAbRatio,engMatchLoss,splitUsage,settledWin,asymThrY,asymLoadY,massKg,nx,capStart,capFlat,capUMin,capULim,gsWheel,gsIndi,gsRdot,gsNu,gsFyK,tvcyCmd,tvcyNow,tvcyHold,canL,canR,flapHL,flapHR,latLag,slipLead,stopSlipLim,stopSlipAB,stopSlipU,stopSlipPred,stopSlipMargin,stopSlipHold,yawSatPitch,yawLead,paceLead,paceHold,slipTol,slipDep,slipAuth,slipStiff,paceRel,tradeAlpha,slipDamp,dampCredit,yawCap,rollGate,rateCap,revSimBeta,revSimT,paceReach" + CapacityEstimator.Header() + ",alphaRoot,betaRoot,airRefCg,alphaCg,betaCg,nuR,nuY,anchR,anchY,unR,unY,gUnmetX,gUnmetZ,paceGateZ,paceGateX,paceMLeft,paceCapZP,paceCapZN,paceCapXP,paceCapXN,capSwp,capShd,aYawShed,ppOn,ppQ,ppReach,rateCapLag,axWr,axWp,axWy");
			foreach (SurfaceEffector surface in surfaces)
			{
				w.Write("," + surface.name + "_cmd," + surface.name + "_now," + surface.name + "_aoaNow," + surface.name + "_aoaCmd");
			}
			w.WriteLine();
		}

		public void WriteRow(TextWriter w, float t, float spv, float srv, float syv, float rho)
		{
			CultureInfo invariantCulture = CultureInfo.InvariantCulture;
			w.Write(string.Format(invariantCulture, "{0:F3},{1},{2:F1},{3:F1},{4:F3},{5:F2},{6:F2},{7:F2},{8:F3},{9:F2},{10:F1},{11:F1},{12:F1},{13:F2},{14:F2},{15:F2},{16:F1},{17:F1},{18:F1},{19:F1},{20:F1},{21:F1},{22:F0},{23:F0},{24:F2},{25:F1},{26:F2},{27},{28},{29:F1},{30:F1},{31:F1},{32:F2},{33:F2}", t, (int)Mode, ias, tas, rho, alpha, beta, nz, nAlpha, tw, omega.x * 57.29578f, omega.y * 57.29578f, omega.z * 57.29578f, spv, srv, syv, qB, qRaw, qCmd, qT, pCmd, rCmd, aUp, aDn, betaG, alphaCmd, nCmd, unload ? 1 : 0, prot ? 1 : 0, omegaV, phi, theta, tvc.cmd, tvc.now));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F0},{2:F2},{3:F2},{4:F2},{5:F1},{6:F1},{7:F2},{8:F1},{9:F2},{10:F2},{11:F0},{12:F0}", nMaxAero, alphaAtNMax, gAvail, gInt, aInt, flaps.flapBias, flaps.flapTarget, flaps.hlPos, flaps.relief, rudderAuth, yawSat, aRoll, aYaw));
			w.Write(string.Format(invariantCulture, ",{0:F0},{1:F0},{2:F0},{3:F0},{4:F2},{5:F0},{6:F0},{7:F0},{8:F1},{9:F2}", pMaxDbg, pStockRef, pFull, rMaxDbg, guardDbg, pSmaxDbg, pSTgtDbg, latAccDbg, rSDbg, stabPitchFrac));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1}", groundBrake, revFlow ? 1 : 0));
			w.Write(string.Format(invariantCulture, ",{0:F3},{1:F1},{2:F1},{3:F2},{4:F1},{5:F1},{6:F1},{7:F1},{8:F1}", asymDy, asymArNow, asymAyNow, asymUNow, Mathf.Min(asymCapPos, 999f), Mathf.Min(asymCapPosEff, 999f), Mathf.Max(asymCapNegEff, -999f), dMeasR, dMeasY));
			w.Write(string.Format(invariantCulture, ",{0:F0},{1:F0},{2:F1},{3:F1},{4:F1},{5:F1}", asymCrNow, asymCyNow, asymFFr, asymFFy, AccRoll(asymMNow), AccYaw(asymMNow)));
			w.Write(string.Format(invariantCulture, ",{0:F2}", asymFFGate));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F1}", rollPitchFF * IaInv[1, 1] * 57.29578f, gUnderDbg));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1},{2:F2},{3:F0},{4:F0},{5:F0},{6:F2},{7:F0},{8:F0},{9:F0},{10}", pitchPrio, stabRateBound ? 1 : 0, mLeadGate * mLeadAgree, dTrimR, aRollPos, aRollNeg, latReserve, aUpPitch, aDnPitch, aBrakeTgt, stabBranch));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F1}", tvcAnchor, tvcBudgetGate, rudSideDbg));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3},{4:F2},{5:F1},{6:F2},{7:F0}", gsRCmd, gsNwsDeg, gsRud, gsHold ? 1 : 0, gsLineY, Mathf.Min(asymCapPitch, 999f), pitchNeed, coupRollLim));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F1},{2:F2},{3:F2}", rpGate, rpBand, rollGov, rollGovMargin));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F3},{2:F1},{3:F1}", iasDotF, qPredK, Mathf.Clamp(asymCapPos, -99f, 99f), Mathf.Clamp(asymCapFilt, -99f, 99f)));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F2},{2:F1},{3},{4:F2},{5:F2},{6:F1},{7:F1},{8:F2},{9:F1}", alphaCmdRaw, nCmdRaw, rateBlendAoA, hiRate ? 1 : 0, rateW, zoneX, Mathf.Min(authCap, 99f), Mathf.Min(authCapEff, 99f), nLiftMax, hybridDM * IaInv[1, 1] * 57.29578f));
			w.Write(string.Format(invariantCulture, ",{0},{1:F2}", floatOn ? 1 : 0, floatS));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F2},{2},{3:F1}", rateBlendAoANeg, nLiftMin, hiRate ? hiSide : 0, Mathf.Max(authCapNegEff, -99f)));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1},{2:F1},{3:F2}", vmcLim, engOut ? 1 : (engAsym ? 2 : 0), fuelTrimOn ? (fuelTrimDelta * 100f) : 0f, yawHealth));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2}", capIUp, capIDn, capPushU));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F2},{2:F1},{3},{4:F1}", slipRateDbg, slipGovDbg, Mathf.Clamp(yawAnchorDbg, -999f, 999f), Plugin.GRateLaw ? 1 : 0, rateCmdG));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5},{6:F1},{7:F1},{8:F2},{9:F2},{10:F2},{11:F2}", negGateDbg, sharedLatDbg, rollGovPredDbg, loadRevDbg, stickLeadDbg, capOn ? 1 : 0, capQ, sweepLimDbg, slatRelief, flaps != null ? flaps.hlPos : 0f, (YawPrioHi > 0f && Plugin.SlipYawWeightOn) ? YawHiSlipGate() * Mathf.Clamp01((Mathf.Abs(alpha) - 20f) / 20f) : 0f, coupGateHeld));
			w.Write(string.Format(invariantCulture, ",{0:F3},{1:F2},{2:F2},{3:F0},{4:F3},{5:F2},{6:F2},{7:F2},{8:F2}", pitchLeadDbg, slipGovPullDbg, decelDbg, pullTradeDbg, airbrakeShare, AirbrakeOpenTarget, airbrakeOpenNow, alloc.fightK, pullFFBoost));
			// Ifrit 0.3.0: roll TVC, the split decelerons (physical deg), the sideslip assist and the flaperon droop
			float spLog = 0f;
			float spRog = 0f;
			foreach (SplitEffector sLg in splits)
			{
				float ph = sLg.Phys(sLg.now);
				if (sLg.name.EndsWith("_L"))
				{
					spLog = ph;
				}
				else
				{
					spRog = ph;
				}
			}
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F1},{3:F1},{4:F0},{5:F0},{6:F2},{7:F1},{8:F1}", (tvcd != null) ? tvcd.cmdOut : 0f, (tvcd != null) ? tvcd.now : 0f, spLog, spRog, aRollTvc, pSlipMax, slipGainDbg, (flaps != null) ? flaps.innerDroop : 0f, (flaps != null) ? flaps.outerDroop : 0f));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F1},{6:F2},{7:F2},{8:F2},{9:F2},{10:F2},{11}", revSenseDbg, slipPredDbg, slipHoldDbg, splitTierDbg, tvcNeutralDbg, qTraj.v, fightGateDbg, abGateOn ? ((engOut || matchF <= 0f) ? 1f : matchF) : 0f, engAbRatio, engMatchLoss * 0.001f, splitUsage, settledWinDbg ? 1 : 0));
			float thrY = (tvc != null && tvc.active) ? AccYaw(((tvcd != null && tvcd.active) || (tvcy != null && tvcy.active)) ? tvc.MAt(tvc.now) : tvc.Mnow) : 0f;
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F1},{3:F3}", thrY, dMeasY + thrY, mass, nxDbg));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F1},{2:F2},{3:F2}", AsymCap.DbgStart, AsymCap.DbgFlat, AsymCap.DbgUMin, AsymCap.DbgULim));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1},{2:F1},{3:F1},{4:F2}", gsWheelDbg, gsIndiOn ? 1 : 0, gsRdotF, gsNuDbg, gsFyF * 0.001f));
			// Revoker 0.1.0: yaw nozzle command / angle / neutral-hold weight, and the two canards' deflection
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2}", (tvcy != null) ? tvcy.cmdOut : 0f, (tvcy != null) ? tvcy.now : 0f, tvcYawNeutralW, (surfaces.Count > 0) ? surfaces[0].now : 0f, (surfaces.Count > 1) ? surfaces[1].now : 0f));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2}", (hinges.Count > 0) ? hinges.Find((HingeFlapEffector h) => h.left)?.hingeNow ?? 0f : 0f, (hinges.Count > 0) ? hinges.Find((HingeFlapEffector h) => !h.left)?.hingeNow ?? 0f : 0f));
			// Revoker 0.2.15: the lateral loops' answer lag (s) and the governor's look-ahead (s)
			w.Write(string.Format(invariantCulture, ",{0:F3},{1:F3}", latLagEff, slipLeadDbg));
			w.Write(string.Format(invariantCulture, ",{0:F0},{1:F0},{2:F2},{3:F2},{4:F1},{5:F2},{6:F1},{7:F3}", Mathf.Min(stopSlipLimDbg, 999f), stopSlipABDbg, stopSlipUDbg, stopSlipPredDbg, stopSlipMarginDbg, stopSlipHoldDbg, yawSatPitchDbg, yawLeadDbg));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F1},{2:F1},{3:F1},{4:F1},{5:F2},{6:F2},{7:F1}", paceLeadDbg, paceHoldDbg, slipTolDbg, slipDepDbg, slipAuthDbg, slipStiffDbg, paceRelDbg, tradeAlphaDbg));
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F0}", slipDampDbg, dampCredit));
			w.Write(string.Format(invariantCulture, ",{0:F0},{1:F2}", yawCapDbg, rollLoadDbg));
			w.Write(string.Format(invariantCulture, ",{0:F0},{1:F2},{2:F2},{3:F1}", Mathf.Min(dbgRateCap, 9999f), revSimBetaDbg, revSimTDbg, paceReachDbg));
			capEst.Row(w, invariantCulture);
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2},{3:F2},{4:F2}", alphaRoot, betaRoot, AirDataAtCg ? 1 : 0, alphaCgDbg, betaCgDbg));
			// Revoker 0.8.2: what the roll / yaw rate loops ask for (deg/s^2; ~0 with the stick centred once the jet holds its command),
			// how far the tables' roll / yaw columns were off their live moment this tick before anchoring (deg/s^2, summed), and what
			// the allocation left unmade of its target (deg/s^2, the main solve)
			{
				Vector3 anchSum = Vector3.zero;
				foreach (SurfaceEffector sA in surfaces)
				{
					if (sA.active)
					{
						anchSum += sA.anchorDbg;
					}
				}
				foreach (HingeFlapEffector hA in hinges)
				{
					if (hA.active)
					{
						anchSum += hA.anchorDbg;
					}
				}
				w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3:F2},{4:F2},{5:F2}", omegaDotDes.x * 57.29578f, omegaDotDes.z * 57.29578f, anchSum.x * IaInv[0, 0] * 57.29578f, anchSum.z * IaInv[2, 2] * 57.29578f, -allocResMain.x, -allocResMain.z));
			}
			// 0.6.0: the governor port's columns (F-22E 3.0.0): the allocator's unmet (filtered), the pace window's gates, margin left and
			// capacities, the push governor's caps, the push plan, the reach-pass re-solves, the rate cap's servo lag, the axis weights
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F1},{2:F2},{3:F2},{4:F1},{5:F0},{6:F0},{7:F0},{8:F0}", allocUnmet.x, allocUnmet.z, paceGateZDbg, paceGateXDbg, paceMLeftDbg, paceCapZPDbg, paceCapZNDbg, paceCapXPDbg, paceCapXNDbg));
			w.Write(string.Format(invariantCulture, ",{0:F1},{1:F1},{2:F1},{3},{4:F1},{5}", capSweepDbg, Mathf.Clamp(capShedDbg, -999f, 999f), aYawShedDbg, pushPlanOn ? 1 : 0, pushPlanOn ? pushPlanQNow : 0f, alloc.reachMoves - reachLogPrev));
			reachLogPrev = alloc.reachMoves;
			w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F2},{3:F2}", rateCapLagDbg, axWDbg.x, axWDbg.y, axWDbg.z));
			foreach (SurfaceEffector surface in surfaces)
			{
				w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F1},{3:F1}", surface.cmd, surface.now, surface.localAoANow, surface.localAoACmd));
			}
			w.WriteLine();
		}
	}
}
