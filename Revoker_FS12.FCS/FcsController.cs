using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal sealed class FcsController
	{
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

		internal static float BrakeYawCap = 1.3f;

		internal static float RollEntryCoord = 0.5f;

		// 2.1.0 sideslip-rate lead for the lateral guard (see LateralLaw)
		internal static float SlipLeadT = 0.8f;

		internal static float SlipRateTau = 0.05f;

		internal static float GuardCutTau = 0.08f;

		internal static float SlipGovStart = 4f;

		internal static float SlipGovSpan = 10f;

		public float slipGovDbg = 1f;

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

		public float sweepLimDbg;

		internal static float SweepYawK = 0.5f;

		internal static float SweepPs0 = 10f;

		internal static float SweepPsSpan = 10f;

		internal static float SweepFloor = 6f;

		internal static float SweepOff = 999f;

		internal static float SweepFreeBand = 4f;

		internal static float SweepRollShare = 0.5f;

		internal static bool SweepPosPullFree = true;

		internal static float SweepPosPs0 = 80f;

		internal static float SweepPosPsSpan = 60f;

		internal static float SweepPosK = 0.25f;

		internal static float SlipGovPs0 = 10f;

		internal static float SlipGovPsSpan = 10f;

		internal static float SlipGovLead = 0.3f;

		internal static float PullGovStart = 0.7f;

		internal static float PullGovStartFinLoss = 0.7f;

		internal static float PullRollTradeFinLoss = 0.7f;

		internal static float PullGovStopK = 1f;

		internal static float PullGovAccK = 0.5f;

		internal static bool UnloadOnLeadOnly = true;

		internal static float PullGovStopA1 = 35f;

		internal static float PullGovUnloadSpan = 0.5f;

		internal static float PullGovUnloadRate = 20f;

		internal static float PullGovFallRate = 8f;

		internal static float PullGovRiseRate = 2f;

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

		internal static bool UnloadFadeFix = true;

		internal static float UnloadA0 = 5f;

		internal static float UnloadASpan = 10f;

		internal static float DecelRise = 3f;

		internal static float DecelFall = 1.5f;

		private float pullGovF = 1f;

		// 2.2.24 adaptive feed-forward in the rolling-pull governor
		private float pullFFBoost = 1f;

		public float pullServoDbg;

		private float pullServoEx;

		internal static float PullServoSlew = 30f;

		internal static bool PullServoUseXs = false;

		internal static float PullServoPs0 = 60f;

		internal static float PullServoPs1 = 100f;

		internal static float PullServoGovStart = 0.9f;

		public float pullServoWDbg;

		internal static bool PullServoAdd = false;

		internal static bool PullServoBlend = false;

		internal static bool PullServoMax = false;

		internal static bool PullServoLeadByRoll = false;

		internal static float PullServoSr0 = 5f;

		internal static bool PullServoNoUnload = true;

		internal static float RelLatRollPrio = 3f;

		public float relLatPrioDbg = 1f;

		internal static bool PullGovServo = true;

		internal static float PullServoK = 3f;

		internal static float PullServoTarget = 0.85f;

		internal static float PullServoLead = 0.2f;

		internal static bool PullServoShortLead = true;

		public float pullXsDbg;

		public float pullLeadDbg;

		internal static float PullFFBoostRise = 6f;

		internal static float PullFFBoostFall = 12f;

		internal static float PullFFBoostLo = 0.5f;

		internal static float PullFFBoostHi = 0.7f;

		internal static float PullFFBoostIdle = 2f;

		internal static float PullFFBoostSlipRate = 2f;

		internal static bool PullFFBoostTrade = false;

		internal static bool PullFFBoostTradeFF = true;

		// 2.2.8 governor state
		internal static float PullGovIdleRise = 4f;

		internal static float PullTradeFloor = 30f;

		internal static float PullTradeFFK = 0.3f;

		internal static float PullTradeFFIas0 = 93f;

		internal static float PullTradeFFIasSpan = 25f;

		internal static float PullTradeFall = 300f;

		internal static float PullTradeRise = 120f;

		internal static float PullTradeCutA = 15f;



		internal static bool PullFFBoostSlipFall = true;

		internal static float PullFFBoostSlipT = 0f;

		internal static bool PullFFBoostStatic = false;

		internal static float PullFFBoostSLo = 0.25f;

		internal static float PullFFBoostSHi = 0.65f;

		private bool pullGovTouched;


		internal static float GovCapTau = 0.3f;

		private float govCapQ;

		private bool govCapOn;

		private float pullTradePs = 999f;

		private float pullTradeTgt = 999f;

		public bool pullTradeHoldDbg;

		internal static float PullTradeRiseHold = 0f;

		public float pullTradeDbg;

		public float slipGovPullDbg = 1f;

		internal static float SweepPosFloor = 6f;

		internal static float SweepPosLag0 = 3f;

		internal static float SweepPosLag1 = 10f;

		internal static float CoordJerkK = 0.35f;

		internal static float CoordJerkFloor = 20f;

		internal static float ToeLatRelease = 0.5f;

		private float toeLatGate;

		public float toeTgtF;

		internal static float RevYawCap = 0.9f;

		public float coordJerkDbg;

		internal static float AsymRollMax = 120f;

		internal static float AsymRollU0 = 0.1f;

		internal static bool AsymRollHardOnly = true;

		internal static float EndJerkHi = 2f;

		internal static float CoupGateA0 = 12f;

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

		// Revoker 0.2.5: 20 (F-22E / Ifrit: 5). Without TVC the modelled net nose-down at the 45 deg cap is under 10 deg/s^2,
		// so the floor set the velocity-vector roll at 25 deg/s at every IAS (in-game 28 Oct 19:56, DisableTVC). Bench, TVC
		// off, loaded rolls at the cap 220-600 km/h: 23-24 -> 32-46 deg/s mean, AoA overshoot 0.3 deg, sideslip 4.5-7.3 deg,
		// push recovery unchanged (1.56 -> 1.66 s at 300 km/h). 30 gave little more (32-52) for more sideslip.
		internal static float CoupRollFloor = 20f;

		public float coupRollLim;

		private bool branchGateOn;

		internal static float BranchFailTime = 0.25f;

		internal static bool BranchFreeTvc = true;

		internal static bool DebugBranch = false;

		internal static bool BranchScaleToPeak = true;

		internal static bool BranchFullTravel = true;

		internal static float BranchDeepMargin = 5f;

		internal static float BranchCoolTime = 0.25f; // 2.0.1: 1 s -> 0.25 s (a stab could park deep in stall during the wait)

		internal static float BranchCoolTimeStop = 0.25f;

		internal static float LatTrimGain = 100f;

		public float aUpPitch;

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
					HingeFlapEffector hf = new HingeFlapEffector(apF, visF, hostF, hostLN, foils.TryGetValue(apF, out var tF) ? tF : FoilFor(apF, aircraftParameters), (hostF != null && foils.TryGetValue(hostF, out var tH)) ? tH : null, Plugin.FlapHingeRange, flaps.RestOf(visF), (hostLN != null) ? flaps.RestOf(hostLN) : null);
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
				}
			}
			if (num < 1f)
			{
				return;
			}
			mass = num;
			cg = zero / num;
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
				float toe = (AuthCapRudderToe && Plugin.RudderPitchAid) ? Plugin.RudderToeMax : 0f;
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
			vCgAir = ac.rb.velocity - wind;
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
			Vector3 vCgHat = (ac.rb.velocity - wind).normalized;
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
			Vector3 vector2 = rb.velocity - vector;
			tas = vector2.magnitude;
			float num2 = Mathf.Max(0.01f, ac.airDensity);
			ias = tas * Mathf.Sqrt(num2 / 1.225f);
			Vector3 vector3 = quaternion * vector2;
			alpha = ((tas > 1f) ? (Mathf.Atan2(0f - vector3.y, vector3.z) * 57.29578f) : 0f);
			beta = ((tas > 1f) ? (Mathf.Asin(Mathf.Clamp(vector3.x / tas, -1f, 1f)) * 57.29578f) : 0f);
			Vector3 vector4 = quaternion * rb.angularVelocity;
			omega = new Vector3(0f - vector4.z, 0f - vector4.x, vector4.y);
			Vector3 x = (havePrev ? ((rb.velocity - velPrev) / num) : Vector3.zero);
			Vector3 vector5 = accF.Step(x, 0.05f, num);
			Vector3 lhs = vector5 - Physics.gravity;
			nz = Vector3.Dot(lhs, transform.up) / 9.81f;
			nxDbg = Vector3.Dot(lhs, transform.forward) / 9.81f;   // 0.4.6 telemetry: body-axis specific force along the nose (g)
			Vector3 vector6 = ((tas > 1f) ? Vector3.Cross(vector2, transform.right).normalized : transform.up);
			eLy = vector6.y;
			float vc = Mathf.Max(tas, 30f);
			omegaV = Vector3.Dot(vector5, vector6) / Mathf.Max(tas, 5f) * 57.29578f;
			vDot = vDotF.Step((tas > 1f) ? Vector3.Dot(vector5, vector2 / tas) : 0f, 0.2f, num);
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
			teBrakeAir = (TeBrakeOn && !flag2 && Mode != FcsMode.Ground && Mode != FcsMode.Off) ? (airBrake * (Plugin.AirbrakeAoAFade ? airbrakeFade : 1f)) : 0f;
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
			havePrev = true;
			float num4 = 0f;
			float num5 = 0f;
			Vector3 zero2 = Vector3.zero;
			Vector3 zero3 = Vector3.zero;
			foreach (Effector eff2 in effs)
			{
				if (eff2.active)
				{
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
				t3 *= 1f - pitchNeed;
			}
			// Revoker 0.2.7 (HighAoARollWeight): with the nozzles not covering pitch (TVC off / low thrust) the surfaces carry
			// every axis and the 0.35 high-AoA roll weight let pitch and yaw take the roll away under saturation; with the nozzles
			// working the F-22E's 0.35 stays (bench: 1 there loosened the damaged-jet reversal, roll error x3). Faded at 1/s.
			hiRollWF = Mathf.MoveTowards(hiRollWF, noTvcOn ? 1f : 0f, num);
			alloc.axisWeight = new Vector3(Mathf.Lerp(Mathf.Lerp(1f, Mathf.Lerp(0.35f, Plugin.HighAoARollWeight, hiRollWF), t), 1.5f, t3), Mathf.Lerp(Mathf.Lerp(8f, 15f, t2), 1f, t3), Mathf.Lerp(Mathf.Lerp(Plugin.YawPriority, (YawPrioHi > 0f && Plugin.SlipYawWeightOn) ? Mathf.Lerp(Mathf.Max(2f, Plugin.YawPriority), Mathf.Min(Plugin.YawPriority, YawPrioHi), YawHiSlipGate()) : Mathf.Max(2f, Plugin.YawPriority), t), Mathf.Max(2.5f, Plugin.YawPriority), t3));
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
					surface4.symCost = Plugin.StabTrimCost * (1f + LatTrimGain * latReserve);
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
				float rlp = Mathf.Lerp(1f, RelLatRollPrio, relLat * Mathf.Clamp01((pErr - 2f) / 8f));
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
			float relRb = relStick * stabPitchFrac;
			// Revoker 0.2.8: not while the TrailingEdgeBrakeConfig set moves in or out: the elevons' common deflection needs the
			// canards to re-trim with it, and bounded to their own (near-zero) rate they held both still for 0.7 s mid-way and
			// let go in one step (bench, 400 km/h: elevons 11 -> 18 deg in 0.2 s, nz 1.07 -> 0.81)
			bool teBrakeMoving = Mathf.Abs(teBrakeAir - teBrakeAirF) > 0.001f || (teBrakeLandF > 0.001f && teBrakeLandF < 0.999f);
			teBrakeMov = teBrakeMoving;
			stabRateBound = Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off && Plugin.ReleasedStabRateBound > 0f && relRb > 0.01f && !teBrakeMoving;
			if (stabRateBound)
			{
				foreach (SurfaceEffector surface12 in surfaces)
				{
					if (surface12.kind == SurfaceKind.Stabilator && surface12.active)
					{
						float num46 = Mathf.Max(1f, surface12.rate) * Plugin.ReleasedStabRateBound / relRb;
						surface12.lo = Mathf.Max(surface12.lo, Mathf.Min(surface12.now, surface12.hi) - num46);
						surface12.hi = Mathf.Min(surface12.hi, Mathf.Max(surface12.now, surface12.lo) + num46);
					}
				}
			}
			// 1.8.40: while rolling at low/moderate AoA, the stabs are not driven past their lift peak in any pass: beyond
			// it the down-going stab's lift falls off and its nose-down turns into pitch-up exactly when it rolls hardest.
			if (RpMainStall && (StabWindowAlways || rpGate > 0.01f) && Mathf.Abs(alpha) < RpMainAoA && Engaged && Mode != FcsMode.Ground && Mode != FcsMode.Off)
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
					surfaceEffector4.cmd = Mathf.Clamp(num19 * surfaceEffector4.origPitchRange + num20 * surfaceEffector4.origRollRange + num21 * surfaceEffector4.origYawRange + brakeBias[n] + gToe + gDroop + gElev, surfaceEffector4.min, surfaceEffector4.max);
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
					if (DebugBranch)
					{
						System.Console.Error.WriteLine($"BRpre {sb.name} a {alpha:F1} cmd {sb.cmd:F1} laoa {sb.LocalAoAAt(sb.cmd):F1} stall {sb.stallAoA:F1} cool {sb.branchCool:F2}");
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
					if (DebugBranch)
					{
						System.Console.Error.WriteLine($"BR {sb.name} a {alpha:F1} on {sb.branchOn} bestD {bestD:F1} laoa {sb.LocalAoAAt(sb.cmd):F1} e0 {Mathf.Abs(y0 - vector11.y) * kP:F1} e1 {Mathf.Abs(y1 - vector11.y) * kP:F1} tvc {tvc.cmd:F1} ok {pitchOk} room {tvcRoom}");
					}
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

		internal static bool RpMainStall = false;

		internal static float RpMainAoA = 18f;

		internal static float RpMainStallMargin = 3f;

		internal static bool StabWindowAlways = false;

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

		public bool settledWinDbg;

		// 0.4.6 telemetry: the thrust moment's yaw about the CoM, which dMeasY leaves out (the nozzles are an effector, so their
		// whole moment, the CoM-offset part included, is subtracted there); asymLoadY = dMeasY + asymThrY is the measured load
		// yaw comparable with asymMy (the model's, thrust included). Plus the mass and the body-axis specific force.
		public float nxDbg;

		private float settledUseF;

		private bool settledUseHi;

		// narrow lo..hi to the deflections whose local AoA stays within +-lim (keeps the side the surface is on if none do)
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

		internal static bool AuthCapRudderToe = false;

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
					tvUp = Mathf.Max(0f, (tvMax - tvc.Mnow.y) * IaInv[1, 1] * 57.29578f);
					tvDn = Mathf.Max(0f, (tvc.Mnow.y - tvMin) * IaInv[1, 1] * 57.29578f);
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
				num34 = Mathf.Min(num34, qPilot);
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
				num34 = Mathf.Max(num34, qPilot);
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
					if (sp > 0.02f && alpha > -2f)
					{
						num34 = Mathf.Max(num34, (nz > num5) ? ((0f - Plugin.ProtRate) * (1f - sp)) : 0f);
					}
					else if (sp < -0.02f && alpha < 2f)
					{
						num34 = Mathf.Min(num34, (nz < num6) ? (Plugin.ProtRate * (1f + sp)) : 0f);
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
			sweepLimDbg = 0f;
			pullGovTouched = false;
			pullTradeTgt = 999f;
			pullTradeHoldDbg = false;
			float qAskPre = num34;
			if (Plugin.SweepYawLimitOn && SweepYawK > 0f && flag && !prot && !isAI && tas > 25f && Mode == FcsMode.AoAG)
			{
				float saS = Mathf.Sin(alpha * (MathF.PI / 180f));
				float caS = Mathf.Cos(alpha * (MathF.PI / 180f));
				float pSw = (omega.x * caS + omega.z * saS) * 57.29578f;
				float rSw = (0f - omega.x * saS + omega.z * caS) * 57.29578f;
				float wS = Mathf.Clamp01((Mathf.Abs(pSw) - SweepPs0) / Mathf.Max(1f, SweepPsSpan));
				if (wS > 0f)
				{
					// only the yaw axis counts: the lateral law brakes the body roll with the velocity-vector roll, so the roll axis
					// does not hold p up to re-coordinate the rate vector (bench: counting it let the sweep run at 50 deg/s from 60 deg)
					float cS = aYaw * Mathf.Abs(caS) + SweepRollShare * aRoll * Mathf.Abs(saS);
					float aDotMax = Mathf.Max(SweepFloor, SweepYawK * cS / Mathf.Abs(pSw) * 57.29578f);
					aDotMax = Mathf.Lerp(SweepOff, aDotMax, wS);
					// d rS / dt = -pS * alphaDot: AoA rising makes rS more negative when pS > 0. A sweep that brings rS back
					// toward zero is not limited (beyond SweepFreeBand of rS).
					float hiS = num8 + aDotMax;
					float loS = num8 - aDotMax;
					float fb = Mathf.Clamp01((Mathf.Abs(rSw) - SweepFreeBand) / Mathf.Max(0.5f, SweepFreeBand));
					// pulls at positive AoA are left alone (aero weathercock holds the sideslip there; the 2.0.15 positive-AoA
					// law is unchanged); nose-down sweeps, and pulls from negative AoA, are limited
					float posLimRate = -1f;
					if (alpha > 0f && SweepPosPullFree)
					{
						hiS = 999f;
						// 2.2.7 sideslip-margin AoA-rise governor (replaces the 2.2.4 roll-rate limit, whose fade-in
						// lerp(999, limit, w) left it at ~300 deg/s - no limit - below ~200 deg/s of roll; 17 Oct log: sideslip 19-21 deg
						// in pulls from low AoA rolling at 60-135 deg/s, 290-380 km/h, sweepLim 0).
						// A roll pS at AoA a is coordinated with a stability-axis yaw rate of pS sin a, so a rising AoA drives the
						// sideslip toward sign(pS) at pS x alphaDot until the yaw catches up. Two bounds on the AoA rise:
						//  - feed-forward: what the measured yaw authority can follow, SweepPosK x aYaw cos a / pS;
						//  - sideslip margin: the sideslip error (less the pedal command) and where it is heading (kinematic sideslip
						//    rate x SlipGovLead), on the side the AoA rise pushes it; from PullGovStart x margin the allowed rise
						//    tapers to zero (AoA held) at the margin, PullSideslipMargin (dev, default 6 deg).
						// Measured state only. Only with the jet rolling (|pS| from 10, full by 20 deg/s), only nose-up.
						float wP = Mathf.Clamp01((Mathf.Abs(pSw) - SlipGovPs0) / Mathf.Max(1f, SlipGovPsSpan));
						if (Plugin.SweepPosLimitOn && wP > 0f)
						{
							pullGovTouched = true;
							float sgP = Mathf.Sign(pSw);
							float bE = beta - betaPedalCmd;
							// 2.2.10: the look-ahead covers the time the pitch rate needs to come down to the AoA-hold rate as well
							// (at the measured nose-down authority), and adds the sideslip the AoA still rising over that time builds
							// with nothing from the yaw (pS x alphaDot, x PullGovAccK). 19 Oct 10:00 log, 270.3-272 s: full aft + full
							// roll at 112 m/s, q 55 deg/s past its 48 command when the roll reached 85 deg/s; the governor cut the
							// command at 271.1 s but the AoA rose 23 -> 33 deg while q came down and the sideslip ran to 18 deg.
							float aRise = Mathf.Max(0f, omega.y * 57.29578f - num8);
							float wLo = Mathf.Clamp01((PullGovStopA1 - alpha) / 15f);
							float tLead = SlipGovLead + PullGovStopK * wLo * aRise / Mathf.Max(40f, aDn);
							float bP = bE + slipRateDbg * tLead + sgP * 0.5f * PullGovAccK * wLo * Mathf.Abs(pSw) * (MathF.PI / 180f) * aRise * tLead * tLead;
							float xs = sgP * ((sgP * bP > sgP * bE) ? bP : bE);
							pullXsDbg = xs;
							pullLeadDbg = tLead;
							float mg = Plugin.PullSideslipMargin;
							// the feed-forward scales with the margin (6 deg = SweepPosK): a wider margin buys a faster AoA rise.
							// 2.2.8: taper from 70 % of the margin (was 35 %) and lead 0.3 s (was 0.5), so the sideslip runs up to the
							// margin (19 Oct log, 112-117 s: held at 3-4 deg of a 6 deg margin).
							float aDotP0 = SweepPosK * (mg / 6f) * aYaw * Mathf.Abs(caS) / Mathf.Abs(pSw) * 57.29578f;
							// 2.2.24: the feed-forward's share of the yaw authority (SweepPosK) is a fixed guess at how fast the yaw
							// follows; in the 20 Oct 00:58 log it held the AoA rise to ~10 deg/s at 100-125 kt with the sideslip at 1-4 deg
							// of a 9 deg margin (slipGovPull 1, sweepLim 10). It is now scaled up while the measured sideslip (with its
							// lead) stays under PullFFBoostLo of the margin and the stick asks for more, and scaled back as it passes
							// PullFFBoostHi, so the AoA rises as fast as the sideslip allows up to the margin. The sideslip taper below
							// still holds the margin itself.
							// off with a rudder lost: the sideslip there comes from the roll itself, not the AoA rise (bench, both fins
							// gone: up to +2 deg with it on)
							if (Plugin.PullFFBoostMax > 1f && yawHealth >= 0.99f)
							{
								float useB = xs / Mathf.Max(0.5f, mg);
								float askB = num34 - num8;
								float rB = 0f;
								// 2.2.25 (PullFFBoostSlipFall): also backed off as soon as the sideslip is building fast on the AoA-rise side,
								// not only once it is near the margin, so the AoA rise eases in place of the taper stopping it late
								// 2.2.26 (PullFFBoostSlipT): the sideslip rate the boost tolerates is the margin left over PullFFBoostSlipT
								// seconds (never below PullFFBoostSlipRate), so a 2-3 deg/s drift with 6 deg of margin left no longer stops it
								// (20 Oct 02:03 log, 100-160 kt: boost 1.2-2.1, sideslip 2-5 of 9 deg)
								float srTol = (PullFFBoostSlipT > 0f) ? Mathf.Max(PullFFBoostSlipRate, (mg - Mathf.Max(0f, sgP * bE)) / PullFFBoostSlipT) : PullFFBoostSlipRate;
								float srF = (PullFFBoostSlipFall && srTol > 0f) ? Mathf.Clamp01((sgP * slipRateDbg - srTol) / srTol) : 0f;
								if (useB > PullFFBoostHi || srF > 0f)
								{
									rB = -PullFFBoostFall * ((useB > PullFFBoostHi) ? 1f : srF);
								}
								else if (useB < PullFFBoostLo && askB > aDotP0 * pullFFBoost + 1f)
								{
									// and only while the sideslip is not already running toward the margin (its kinematic rate on the side
									// the AoA rise drives it under PullFFBoostSlipRate): a boost while it builds gave the pitch rate a
									// momentum the taper could not stop in time (bench, full aft + full roll together at 140 kt: 10.8 deg)
									float srB = Mathf.Clamp01(1f - sgP * slipRateDbg / Mathf.Max(0.1f, srTol));
									rB = PullFFBoostRise * (1f - useB / Mathf.Max(0.01f, PullFFBoostLo)) * srB;
								}
								if (PullFFBoostStatic)
								{
									// 2.2.26 (PullFFBoostStatic): the boost follows the margin used (sideslip with its lead) directly: full
									// below PullFFBoostSLo of the margin, 1 at PullFFBoostSHi, linear between, slewed at the rise / fall
									// rates. The 2.2.24/25 integrating boost either stalled on the sideslip-rate gates at low IAS (boost
									// 1.2-2.1 with 2-5 of 9 deg used, 02:03 log) or, with them loosened, wound up and was caught late by
									// the taper (pitch-rate dips back to 10-25 deg/s on the bench).
									float bT = 1f + (Plugin.PullFFBoostMax - 1f) * Mathf.Clamp01((PullFFBoostSHi - useB) / Mathf.Max(0.05f, PullFFBoostSHi - PullFFBoostSLo));
									if (askB <= aDotP0 + 1f)
									{
										bT = Mathf.Min(bT, pullFFBoost);
									}
									pullFFBoost = Mathf.MoveTowards(pullFFBoost, bT, ((bT > pullFFBoost) ? PullFFBoostRise : PullFFBoostFall) * dt);
								}
								else
								{
									pullFFBoost = Mathf.Clamp(pullFFBoost + rB * dt, 1f, Plugin.PullFFBoostMax);
								}
							}
							else
							{
								pullFFBoost = 1f;
							}
							float aDotP = Mathf.Max(SweepPosFloor, aDotP0 * pullFFBoost);
							// 2.2.27 (PullGovServo): the AoA rise is the feed-forward plus a term that steers the sideslip to
							// PullServoTarget of the margin. With the yaw maxed, the sideslip rate goes as roll x (AoA rate - the rate the
							// yaw follows), so AoA rate = FF + K x (target - sideslip) / roll brings the sideslip to the target with a
							// PullServoK time constant and no overshoot; above the target it slows the rise below the FF. Replaces the
							// 2.2.24-26 boost. The roll trade is judged on the FF alone, so the roll is as before.
							float aDotPff = Mathf.Max(SweepPosFloor, aDotP0);
							// servo weight: on with all fins, faded out between PullServoPs0 and PullServoPs1 of roll (at a fast roll the
							// margin buys little AoA, sideslip / roll rate, and the sideslip moves too fast to steer: bench, 230 kt at
							// 80-90 deg/s, 7 deg and a 36 deg/s dip); where it is out, the 2.2.25 boost governor flies as before
							float fSv = (PullServoPs1 > PullServoPs0) ? Mathf.Clamp01((PullServoPs1 - Mathf.Abs(pSw)) / (PullServoPs1 - PullServoPs0)) : 1f;
							float wSv = (PullGovServo && yawHealth >= 0.99f) ? ((PullServoBlend || PullServoMax) ? fSv : 1f) : 0f;
							pullServoWDbg = wSv;
							if (wSv > 0f)
							{
								if (!PullServoAdd && !PullServoMax)
								{
									pullFFBoost = Mathf.Lerp(pullFFBoost, 1f, wSv);
								}
								float bSv = PullServoUseXs ? xs : (sgP * bE + sgP * slipRateDbg * PullServoLead);
								float ex = PullServoK * (PullServoTarget * mg - bSv) * 57.29578f / Mathf.Max(5f, Mathf.Abs(pSw));
								// only while the stick asks for more AoA rise than the FF allows; otherwise nothing is built up in advance
								// (a roll before the pull had wound it to 8 deg/s, so the pull started with the full extra)
								if (num34 - num8 <= aDotP0 + 1f)
								{
									ex = Mathf.Min(ex, 0f);
								}
								// the extra rate rises at PullServoSlew deg/s per s at most (falls at once), so it builds as the margin is
								// used instead of bursting at the pull and then dipping
								if (PullServoSlew > 0f && ex > pullServoEx)
								{
									ex = Mathf.Min(ex, pullServoEx + PullServoSlew * dt);
								}
								pullServoEx = ex;
								if (!PullServoBlend && !PullServoMax && ex > 0f)
								{
									ex *= fSv;
								}
								// and backed off while the sideslip is already running fast toward the margin (PullServoSr0 -> 2x)
								if (PullServoSr0 > 0f && ex > 0f)
								{
									ex *= Mathf.Clamp01(1f - (sgP * slipRateDbg - PullServoSr0) / PullServoSr0);
								}
								aDotP = PullServoMax ? Mathf.Max(aDotP, aDotP0 + ex * wSv) : PullServoAdd ? Mathf.Max(SweepPosFloor, aDotP + ex * wSv) : Mathf.Lerp(aDotP, Mathf.Max(SweepPosFloor, aDotP0 + ex), wSv);
								pullServoDbg = ex * wSv;
							}
							else
							{
								pullServoEx = 0f;
								pullServoDbg = 0f;
							}
							float gS = Mathf.Lerp((yawHealth >= 0.99f) ? PullGovStart : Mathf.Min(PullGovStart, PullGovStartFinLoss), PullServoGovStart, wSv);
							// 2.2.27: with the servo steering the sideslip, the taper is only the backstop at the margin and judges the
							// sideslip with the short lead alone; on the stop-time projection it cut the pitch rate at 4.5 deg of real
							// sideslip (projection 8), and the servo pushed it back up: a dip each pull
							float xsG = xs;
							if (wSv > 0f && PullServoShortLead)
							{
								float bPs = bE + slipRateDbg * SlipGovLead;
								// short lead where the servo steers (slow roll); the full stop-time projection again toward a fast roll, where
								// the sideslip runs at 10-12 deg/s and the short lead caught it late (bench, 216 kt: 7.6 deg and an unload)
								xsG = Mathf.Lerp(xs, sgP * ((sgP * bPs > sgP * bE) ? bPs : bE), wSv * (PullServoLeadByRoll ? fSv : 1f));
							}
							float g = 1f - Mathf.Clamp((xsG - gS * mg) / Mathf.Max(0.5f, (1f - gS) * mg), 0f, 1f + PullGovUnloadSpan);
							// 2.2.10: the look-ahead may hold the AoA but not unload it; unloading (g < 0) goes on the sideslip the
							// jet has (and its 0.3 s trend), not on the stop-time projection. Unloading on the projection swung the
							// sideslip to the other side (bench, lateral added mid-pull at 600 km/h: -11 deg).
							if (g < 0f && UnloadOnLeadOnly)
							{
								float bP0 = bE + slipRateDbg * SlipGovLead;
								float xs0 = sgP * ((sgP * bP0 > sgP * bE) ? bP0 : bE);
								float g0 = 1f - Mathf.Clamp((xs0 - gS * mg) / Mathf.Max(0.5f, (1f - gS) * mg), 0f, 1f + PullGovUnloadSpan);
								g = Mathf.Max(g, Mathf.Min(0f, g0));
							}
							// 2.2.29 (PullServoNoUnload): with the servo steering, past the margin the AoA is held, not unloaded. In a held
							// corkscrew at 160-215 kt the unload (pitch command 46 -> 5) and the servo's push back up pumped the AoA
							// 25 -> 38 -> 31 -> 39 deg every ~1.2 s (20 Oct 10:35 log, 277-281 s)
							if (wSv > 0f && PullServoNoUnload && g < 0f)
							{
								g = Mathf.Lerp(g, 0f, wSv);
							}
							// tightens at up to PullGovFallRate per second, releases at PullGovRiseRate, so a passing sideslip
							// excursion does not pulse the pitch command (48 -> 15 -> 48 deg/s in 0.2 s on the bench at 600 km/h)
							pullGovF = Mathf.MoveTowards(pullGovF, g, ((g < pullGovF) ? PullGovFallRate : PullGovRiseRate) * dt);
							g = pullGovF;
							slipGovPullDbg = g;
							// past the margin (g < 0) the AoA may come back down at up to PullGovUnloadRate, which lowers the yaw rate
							// the roll needs (pS sin a)
							aDotP = (g >= 0f) ? (aDotP * g) : (g / Mathf.Max(0.01f, PullGovUnloadSpan) * PullGovUnloadRate);
							aDotPff = (g >= 0f) ? (aDotPff * g) : aDotP;
							// 2.2.20: no unload at low AoA. Unloading lowers the yaw rate the roll needs (pS sin a), which is near zero
							// there anyway; a sideslip made by the roll itself (fins gone) was unloaded to 0 deg AoA, the governor let go
							// there, the stick pulled it back up and it unloaded again: a 1.2 s pitch cycle, qCmd -13 <-> 48 (bench,
							// both fins detached, full aft + full roll at 460 km/h). Below UnloadA0 it holds the AoA instead.
							if (aDotP < 0f && UnloadFadeFix)
							{
								aDotP *= Mathf.Clamp01((alpha - UnloadA0) / Mathf.Max(1f, UnloadASpan));
							}
							// 2.2.20: the fade-in with roll (wP) loosens a hold (aDotP >= 0, divided by wP) but must weaken an unload
							// (aDotP < 0, times wP). Divided, an unload grew as the roll came down toward SlipGovPs0, to the full
							// -48 deg/s nose-down, and the cap vanished once the roll was under it: the pitch command went
							// 48 -> -48 -> 48 deg/s in 0.2 s each time the roll crossed 10 deg/s (19 Oct 21:37 log, 211-212 s,
							// fins gone, sideslip 10-14 deg; the nozzles stop to stop).
							hiS = num8 + ((aDotP >= 0f || !UnloadFadeFix) ? Mathf.Min(999f, aDotP / Mathf.Max(0.02f, wP)) : (aDotP * wP));
							posLimRate = Mathf.Max(0f, aDotP);
							// 2.2.8 PullRollTrade: the AoA rise the pilot asks for beyond what the governor allows is bought back by
							// slowing the roll. The sideslip a rising AoA makes goes as pS x alphaDot, so the roll that lets the whole
							// asked rise through is pS x allowed / asked (floor PullTradeFloor); the roll limit goes PullRollTrade of
							// the way from the normal limit to it. Released once the governor lets the asked rise through.
							float aAsk = num34 - num8;
							// 2.2.26: while the stick still asks for AoA rise in a governed rolling pull, the trade lets the roll back
							// only at PullTradeRiseHold (an earlier, quick release rebuilt the roll mid-pull and the governor then cut the
							// pitch rate: bench, 230 kt, roll 37 -> 82 deg/s at 23-27 deg AoA, pitch rate 51 -> 25 deg/s)
							pullTradeHoldDbg = aAsk > 5f;
							// 2.2.24: with PullFFBoostTrade off the trade is judged on the un-boosted rate, so the roll is slowed as in
							// 2.2.23 and the boost goes to the AoA
							float posLimTr = Mathf.Lerp(PullFFBoostTrade ? posLimRate : (posLimRate / pullFFBoost), Mathf.Max(0f, aDotPff), wSv);
							if (Plugin.PullRollTrade > 0f && aAsk > posLimTr + 1f)
							{
								pullTradeTgt = Mathf.Max(PullTradeFloor, Mathf.Abs(pSw) * posLimTr / Mathf.Max(0.5f, aAsk));
								// start from the roll in force (2.2.11: also when an earlier, looser limit is still above it)
								if (pullTradePs > Mathf.Max(Mathf.Abs(pSw), pullTradeTgt))
								{
									pullTradePs = Mathf.Max(Mathf.Abs(pSw), pullTradeTgt);
								}
							}
						}
					}
					// 2.2.9 (PullGovSmooth, default on): in the rolling-pull governor the AoA-sweep limit's free band (yaw ahead
					// of the roll, rS beyond 4..8 deg/s, so a rising AoA brings rS back to zero) no longer releases the cap. It
					// switched the cap between the governor's rate and 999 as rS crossed the band, and the sideslip loop then
					// cycled: 19 Oct 09:33 log, 127-131 s, qCmd 48 -> -4 -> 48 -> 19 -> 48 -> 29 -> 47 in 3 s with the
					// sideslip 1.5 -> 7.4 -> 1.2 -> 4.3 deg and the nozzles stop to stop. The sideslip governor already sees
					// the yaw lead through the kinematic sideslip rate.
					if (pSw * rSw > 0f && !(posLimRate >= 0f && Plugin.PullGovSmooth))
					{
						hiS = Mathf.Lerp(hiS, 999f, fb);
					}
					else if (pSw * rSw < 0f)
					{
						loS = Mathf.Lerp(loS, -999f, fb);
					}
					// 2.2.9 (PullGovSmooth): when the governor's cap rises it does so with a GovCapTau lag (it still falls at
					// once), so a release does not throw the command straight back to the stick's 48 deg/s.
					if (posLimRate >= 0f && GovCapTau > 0f && Plugin.PullGovSmooth)
					{
						float capT = Mathf.Min(hiS, 120f);
						govCapQ = ((govCapOn && capT > govCapQ) ? Mathf.Lerp(govCapQ, capT, Mathf.Clamp01(dt / GovCapTau)) : capT);
						govCapOn = true;
						hiS = govCapQ;
					}
					else
					{
						govCapOn = false;
					}
					float before = num34;
					num34 = Mathf.Clamp(num34, loS, hiS);
					sweepLimDbg = (num34 != before) ? ((num34 == hiS && posLimRate >= 0f) ? Mathf.Max(0.01f, posLimRate) : aDotMax) : 0f;
				}
			}
			// 2.2.11 PullRollTrade, feed-forward part: with the trade on, the roll limit is also set from the AoA rate the
			// stick asks for, before the governor has to react: the roll whose coordinating yaw the yaw axis can build while
			// the AoA rises at that rate (the governor's own feed-forward, solved for pS). At high IAS the roll got to
			// 140-157 deg/s inside 0.4 s while the AoA rose at 50-60 deg/s; the governor caught it on the sideslip and
			// unloaded, and pitch and roll hitched together (19 Oct 10:29 log, 254 s at 295 kt: sideslip 14.7 deg, q 61 -> 5
			// -> 40, pS 138 -> 56 -> 129; 467 s at 345 kt the same).
			if (Plugin.PullRollTrade > 0f && PullTradeFFK > 0f && Plugin.SweepYawLimitOn && Plugin.SweepPosLimitOn && SweepPosPullFree && flag && !prot && !isAI && tas > 25f && Mode == FcsMode.AoAG && alpha > 0f)
			{
				float aAskF = qAskPre - num8;
				if (aAskF > 5f)
				{
					float caF = Mathf.Abs(Mathf.Cos(alpha * (MathF.PI / 180f)));
					// faded in with IAS (PullTradeFFIas0 -> +PullTradeFFIasSpan m/s, 180 -> 230 kt; 2.2.12, was 230 -> 280: 13 deg of sideslip at 240 kt in the 10:53 log): below that the governor's own reaction keeps up
					// with the roll build (your 0.7 / 9 setting held at 250 kt and below)
					float wIasF = Mathf.Clamp01((ias - PullTradeFFIas0) / Mathf.Max(1f, PullTradeFFIasSpan));
					float tgtF = (wIasF <= 0f) ? 999f : Mathf.Max(PullTradeFloor, PullTradeFFK * SweepPosK * (PullFFBoostTradeFF ? pullFFBoost : 1f) * (Plugin.PullSideslipMargin / 6f) * aYaw * caF * 57.29578f / aAskF);
					tgtF = Mathf.Lerp(999f, tgtF, wIasF);
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
						float cutW = (PullTradeCutA > 0f) ? Mathf.Clamp01((PullTradeCutA + 10f - alpha) / 10f) : 0f;
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
				if (UnloadFadeFix && govCapOn && Plugin.PullGovSmooth && GovCapTau > 0f)
				{
					govCapQ = Mathf.Lerp(govCapQ, 120f, Mathf.Clamp01(dt / GovCapTau));
					num34 = Mathf.Min(num34, govCapQ);
					if (govCapQ > 100f)
					{
						govCapOn = false;
					}
				}
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
			if (DamageLatLimit && Mathf.Min(yawHealth, yawLeft) < 0.999f)
			{
				// Revoker 0.1.0: with the vertical tail damaged and the nozzle unable to yaw (above the vectoring speed, low
				// thrust or engine out), only drag yaw is left (crossed flaperon + canards, ~7 deg/s^2 at 500 km/h): the finless
				// airframe diverges once the sideslip passes ~1 deg. The damaged roll-rate floor drops to DamageLatFloorNoTvc
				// there (bench, fins gone, full roll at 900 km/h: floor 0.35 -> 87 deg/s, sideslip ran to 72 deg; 0.15 -> 37
				// deg/s, 0.2 deg).
				float floorK = (tvcy != null && !tvcy.active && yawHealth < 0.5f) ? Mathf.Min(DamageLatFloor, DamageLatFloorNoTvc) : DamageLatFloor;
				float hK = Mathf.Max(floorK, Mathf.Min(yawHealth, yawLeft));
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
			float num11 = Mathf.Clamp(revS * (sr + num9 * sy), -1f, 1f) * num10;
			coupRollLim = 0f;
			if (Plugin.CouplingRollLimit > 0f && Ia[1, 1] > 1f && asymValid && tvc.M != null && tvc.N > 1)
			{
				float kIc = (Ia[2, 2] - Ia[0, 0]) / Ia[1, 1];
				float s2c = Mathf.Abs(Mathf.Sin(2f * f));
				float gateC = Mathf.Max(pitchNeed, Mathf.Clamp01((Mathf.Sign(alpha) * alphaDot - 5f) / 15f)) * Mathf.Clamp01((Mathf.Abs(alpha) - CoupGateA0) / 10f);
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
				float trLim = Mathf.Lerp(num10, Mathf.Max(PullTradeFloor, pullTradePs), trK);
				pullTradeDbg = trLim;
				num10 = Mathf.Min(num10, trLim);
				num11 = Mathf.Clamp(num11, 0f - num10, num10);
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
			float num13 = Mathf.Abs(beta - num12);
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
			float bErrLead = bErrNow + bdk * SlipLeadT;
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
			float x = Mathf.Clamp01(1f - (num13 - Plugin.SideslipGuardStart) / 10f) * (1f - Plugin.YawSatGuard * yawSat * num14);
			// independent of SideslipGuardStart (which a pilot may raise out of the way for deliberate sideslip): backs the
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
			float negGate = Mathf.Clamp01((NegGateA0 - alpha) / Mathf.Max(0.5f, NegGateSpan));
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
			guardDbg = x;
			pSTgtDbg = num11;
			float num15 = Mathf.Abs(pTraj.x * num);
			float max = Mathf.Lerp(Plugin.RollAccelUseLowIas, Plugin.RollAccelUse, Mathf.InverseLerp(60f, 110f, ias));
			float a = Mathf.Clamp(0.85f - num15 / Mathf.Max(30f, pFull * (aRollGo / aRoll)), 0.1f, max) * aRollGo / Mathf.Max(Mathf.Abs(num), 0.05f);
			float num16 = 0.6f * aYaw / Mathf.Max(Mathf.Abs(num2), 0.05f);
			// Ifrit 0.3.8: the yaw-coordination cap on the roll acceleration fades with latHoldK too (11 m/s, 10 deg AoA: ~8
			// deg/s^2 from a 5 deg/s^2 yaw axis held the roll entry to a crawl where no coordination is possible anyway)
			float num17 = (latAccDbg = Mathf.Min(a, Mathf.Lerp(a, RollEntryCoord * num16, latLimK)));
			// 1.8.41: the roll entry's acceleration yields with the governor too (the entry is where the surfaces deflect most)
			num17 *= Mathf.Lerp(RollGovAccelFloor, 1f, rollGov);
			if (pTraj.x * (num11 - pTraj.x) < 0f)
			{
				float num18 = aRoll / Mathf.Max(30f, pFull);
				num17 = Mathf.Min((Plugin.RollBrakeUse * (BrakeUseAll ? aRollGo : aRollWing) + num18 * num15) / Mathf.Max(Mathf.Abs(num), 0.05f), BrakeYawCap * num16);
				if (num11 * pTraj.x < 0f && Mathf.Abs(num11) > 10f && ReversalFloor)
				{
					float num19 = Mathf.Min(RevFloorK * aRoll / Mathf.Max(Mathf.Abs(num), 0.05f), num16);
					num17 = (revA = (RevConst ? Mathf.Min(num19, RevYawCap * num16) : Mathf.Max(num17, num19)));
					revTarget = num11;
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
			num17 = Mathf.Clamp(num17, 20f, Plugin.MaxRollAccel);
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
			if (CoordJerkK > 0f && !isAI && rudderAuth > 0.01f)
			{
				float yawSlew = 0f;
				foreach (SurfaceEffector seR in surfaces)
				{
					if (seR.kind == SurfaceKind.Rudder && seR.active && seR.rate > 0f)
					{
						yawSlew = Mathf.Max(yawSlew, seR.rate / Mathf.Max(1f, 0.5f * (seR.max - seR.min)));
					}
				}
				if (yawSlew > 0f)
				{
					float jY = CoordJerkK * aYaw * yawSlew / Mathf.Max(Mathf.Abs(num2), 0.05f);
					jY /= Mathf.Max(rudderAuth, 0.01f);
					a2 = Mathf.Min(a2, jY);
					jFloor = Mathf.Min(jFloor, Mathf.Max(CoordJerkFloor, jY));
				}
			}
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
			if (Mode != FcsMode.GearDownRate && Plugin.SlipTvcAssist && (aRollTvc > 0.5f || yawTvcSlip))
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
			pDotFF = pTraj.v * num / num26;
			SlipHistPush((num23 / num26 - num21) * predGate, dt);
			float num27 = aRoll / Mathf.Max(30f, pFull);
			pCmd += Plugin.RollLead * Mathf.Max(0f, num27 - 2.5f) * pDotFF;
			float t2 = Mathf.Max(Mathf.InverseLerp(200f, 300f, ias), Mathf.InverseLerp(YawFFAoA0, YawFFAoA1, Mathf.Abs(alpha)));
			rDotFF = Mathf.Lerp(Plugin.YawRateFF, 1f, t2) * rDotF.Step((rCmd - rCmdPrev) / dt, 0.1f, dt);
			pCmdPrev = pCmd;
			rCmdPrev = rCmd;
			if (Mathf.Abs(omega.z * 57.29578f) > num8 && rDotFF * omega.z > 0f)
			{
				rDotFF = 0f;
			}
		}

		public void WriteHeader(TextWriter w)
		{
			w.Write("t,mode,ias,tas,rho,alpha,beta,nz,nAlpha,tw,p,q,r,sp,sr,sy,qB,qRaw,qCmd,qT,pCmd,rCmd,aUp,aDn,betaG,alphaCmd,nCmd,unload,prot,omegaV,phi,theta,tvcCmd,tvcNow,nMaxAero,alphaAtNMax,gAvail,gInt,aInt,flapBias,flapTarget,hlPos,relief,rudderAuth,yawSat,aRoll,aYaw,pMax,pStockRef,pFull,rMax,guard,pSmax,pSTarget,latAcc,rS,stabPitchFrac,gndBrake,revFlow,asymDy,asymAr,asymAy,asymU,asymCap,asymCapEff,asymCapNeg,dMeasR,dMeasY,asymCr,asymCy,asymFFr,asymFFy,asymMr,asymMy,asymFFGate,rollPitchFF,gUnder,pitchPrio,stabRateBound,momentLead,rollTrim,aRollPos,aRollNeg,latReserve,aUpPitch,aDnPitch,aBrakeTgt,stabBranch,tvcAnchor,tvcBudget,rudSide,gsRCmd,gsNws,gsRud,gsHold,gsLineY,asymCapPitch,pitchNeed,coupRollLim,rpGate,rpBand,rollGov,rollGovMargin,iasDot,qPredK,asymCapRaw,asymCapFilt,alphaCmdRaw,nCmdRaw,rateBlendAoA,hiRate,rateW,zoneX,authCap,authCapEff,nLiftMax,hybridDMacc,floatOn,floatS,rateBlendAoANeg,nLiftMin,hiSide,authCapNeg,vmcLim,engOut,fuelTrimCm,yawHealth,capIUp,capIDn,capPushU,slipRate,slipGov,yawAnchor,gRateLaw,rateCmdG,negGate,sharedLat,rollGovPred,loadRev,stickLead,capOn,capQ,sweepLim,slatRelief,hlPosNow,slipAlloc,coupGate,pitchLead,slipGovPull,decel,pullTrade,abShare,abTgt,abOpen,fightK,pullFFBoost,tvcdCmd,tvcdNow,splitL,splitR,aRollTvc,pSlipMax,slipGain,droopIn,droopOut,revSense,slipPred,slipHold,splitTier,tvcHold,qTrajV,fightGate,abGate,engAbRatio,engMatchLoss,splitUsage,settledWin,asymThrY,asymLoadY,massKg,nx,capStart,capFlat,capUMin,capULim,gsWheel,gsIndi,gsRdot,gsNu,gsFyK,tvcyCmd,tvcyNow,tvcyHold,canL,canR,flapHL,flapHR");
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
			foreach (SurfaceEffector surface in surfaces)
			{
				w.Write(string.Format(invariantCulture, ",{0:F2},{1:F2},{2:F1},{3:F1}", surface.cmd, surface.now, surface.localAoANow, surface.localAoACmd));
			}
			w.WriteLine();
		}
	}
}
