using System;
using System.Collections.Generic;
using BepInEx.Configuration;
using BepInEx.Logging;
using HarmonyLib;
using NuclearOption.Jobs;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 1.8.43: one DLL for both builds. PerformanceProfile picks stock or buffed performance at runtime.
	public enum PerformanceProfile
	{
		Buffed,
		Stock
	}

	// Ifrit 0.3.1: lateral law with Stability Assist off (MPO)
	public enum MpoRollMode
	{
		VelocityVectorRoll,
		SeparateRollYaw
	}

	// 2.1.0: gear-up pitch command law
	public enum PitchCommandLaw
	{
		GAndPitchRate,
		GAoAAndPitchRate,
		AoAThenRateHold
	}

	// 2.0.8: debug AoA-limit test switch
	public enum AoALimitTest
	{
		Normal,
		Full65,
		Cap36
	}

	internal static class Plugin
	{
		internal static ManualLogSource Log;

		private static ConfigEntry<bool> cEnabled;

		private static ConfigEntry<PerformanceProfile> cProfile;

		private static ConfigEntry<bool> cHostAllowFcs;

		private static ConfigEntry<bool> cHostLock;


		private static ConfigEntry<bool> cTelemetry;

		private static ConfigEntry<bool> cAIFcs;

		private static ConfigEntry<bool> cAILog;

		private static ConfigEntry<bool> cAIDirectRoll;

		private static ConfigEntry<float> cStabUp;

		private static ConfigEntry<float> cStabDn;


		private static ConfigEntry<float> cFlapRange;

		private static ConfigEntry<float> cTvcRange;

		private static ConfigEntry<float> cTvcSpeed;

		private static ConfigEntry<bool> cAIGunRollFade;

		private static ConfigEntry<bool> cAsymOn;

		private static ConfigEntry<bool> cAsymRollHead;

		private static ConfigEntry<bool> cRevRoll;



		private static ConfigEntry<float> cAoAPos;

		private static ConfigEntry<float> cAoANeg;

		private static ConfigEntry<int> cAoANegRev;

		private static ConfigEntry<bool> cIfritSplit;

		private static ConfigEntry<float> cFlapCamber;

		private static ConfigEntry<float> cAilCamber;

		private static ConfigEntry<MpoRollMode> cMpoSep;

		private static ConfigEntry<int> cIfritRev;

		// Revoker 0.1.0
		private static ConfigEntry<bool> cYawTvc;

		private static ConfigEntry<float> cTvcYawMu;

		private static ConfigEntry<bool> cElevonPitch;

		private static ConfigEntry<bool> cCanardYawTier;

		private static ConfigEntry<float> cGroundElevonPitch;

		private static ConfigEntry<bool> cSlipYawTvc;

		private static ConfigEntry<float> cCorkTol;

		// Revoker 0.2.7
		private static ConfigEntry<float> cHiAoARollW;

		private static ConfigEntry<bool> cCanWin;

		private static ConfigEntry<float> cCanWinMargin;

		private static ConfigEntry<bool> cFinTvcHold;

		private static ConfigEntry<float> cFinNoTvcCap;

		private static ConfigEntry<bool> cVisHinge;

		private static ConfigEntry<float> cYawTvcNeutral;

		private static ConfigEntry<bool> cFlapRelief;

		private static ConfigEntry<bool> cCanardPitch;

		private static ConfigEntry<int> cRevokerRev;

		private static ConfigEntry<bool> cFlapFlaperons;

		private static ConfigEntry<float> cFlapHinge;

		private static ConfigEntry<float> cFlapHiLift;

		private static ConfigEntry<float> cElevonSymCost;

		public static float FlapCamberK => (cFlapCamber != null) ? Mathf.Clamp(cFlapCamber.Value, 0f, 1f) : 0.46f;

		public static float AileronCamberK => (cAilCamber != null) ? Mathf.Clamp(cAilCamber.Value, 0f, 1f) : 0.45f;

		public static bool MpoSeparateAxes => cMpoSep != null && cMpoSep.Value == MpoRollMode.SeparateRollYaw;

		private static ConfigEntry<float> cMpoWash;

		public static float MpoSlipWashout => (cMpoWash != null) ? Mathf.Max(0f, cMpoWash.Value) : 0.5f;

		private static ConfigEntry<bool> cIfritTvcRoll;

		private static ConfigEntry<float> cSplitMu;

		private static ConfigEntry<bool> cSlipTvc;

		private static ConfigEntry<bool> cIfritFlaperons;

		private static ConfigEntry<float> cServoSpeed;

		private static ConfigEntry<float> cSplitRange;

		private static ConfigEntry<bool> cTvcRollNeutral;

		private static ConfigEntry<float> cTvcRollNeutralW;

		private static ConfigEntry<float> cSlipGainHi;

		private static ConfigEntry<bool> cDecHldFold;

		private static ConfigEntry<float> cSlipHoldTas0;

		private static ConfigEntry<float> cSlipLimTas0;

		private static ConfigEntry<float> cSlipLimTasSpan;

		private static ConfigEntry<float> cSlipHoldTasSpan;

		private static ConfigEntry<float> cSlipPredT;

		private static ConfigEntry<float> cSlipPredRollGate;

		private static ConfigEntry<bool> cSplitYawTier;

		private static ConfigEntry<float> cStabYawRelax;


		private static ConfigEntry<float> cSlipPredW;

		private static ConfigEntry<float> cRevS0;

		private static ConfigEntry<float> cRevS1;

		private static ConfigEntry<float> cSlipGainA0;

		private static ConfigEntry<float> cFullStickQ;

		private static ConfigEntry<float> cSplitBrakeHold;

		private static ConfigEntry<float> cTvcDiffMu;

		public static bool IfritSplitYaw => cIfritSplit == null || cIfritSplit.Value;

		public static bool IfritTvcRoll => cIfritTvcRoll == null || cIfritTvcRoll.Value;

		public static bool SlipTvcAssist => cSlipTvc == null || cSlipTvc.Value;

		public static bool IfritFlaperons => cIfritFlaperons == null || cIfritFlaperons.Value;

		public static float SurfaceServoSpeed => (cServoSpeed != null) ? Mathf.Clamp(cServoSpeed.Value, 0f, 300f) : 100f;

		// split limit: SplitRange, or 0 = the deceleron's own travel (FlaperonRange; the stock 25 when that is 0 too)
		public static float SplitRange => (cSplitRange != null && cSplitRange.Value > 0f) ? Mathf.Clamp(cSplitRange.Value, 1f, 45f) : ((FlaperonRange > 0f) ? FlaperonRange : 0f);

		public static bool TvcRollNeutralHold => cTvcRollNeutral != null && cTvcRollNeutral.Value;

		public static float TvcRollNeutralWeight => (cTvcRollNeutralW != null) ? Mathf.Max(0f, cTvcRollNeutralW.Value) : 30f;

		public static float FullStickPitchRate => (cFullStickQ != null) ? Mathf.Clamp(cFullStickQ.Value, 0f, 90f) : 60f;

		public static float SlipGainAoAStart => (cSlipGainA0 != null) ? Mathf.Clamp(cSlipGainA0.Value, 0f, 60f) : 5f;

		public static float RevFlowSenseStart => (cRevS0 != null) ? Mathf.Clamp(cRevS0.Value, 90f, 178f) : 120f;

		public static float RevFlowSenseEnd => (cRevS1 != null) ? Mathf.Clamp(cRevS1.Value, 91f, 180f) : 150f;

		public static float SlipLimTasMin => (cSlipLimTas0 != null) ? Mathf.Clamp(cSlipLimTas0.Value, 0f, 100f) : 25f;

		public static float SlipLimTasSpan => (cSlipLimTasSpan != null) ? Mathf.Clamp(cSlipLimTasSpan.Value, 1f, 100f) : 50f;

		public static float SlipHoldTasMin => (cSlipHoldTas0 != null) ? Mathf.Clamp(cSlipHoldTas0.Value, 0f, 60f) : 15f;

		public static float SlipHoldTasSpan => (cSlipHoldTasSpan != null) ? Mathf.Clamp(cSlipHoldTasSpan.Value, 1f, 60f) : 20f;

		public static bool DeceleronHldFold => cDecHldFold == null || cDecHldFold.Value;

		public static float SlipPredictTime => (cSlipPredT != null) ? Mathf.Clamp(cSlipPredT.Value, 0f, 0.8f) : 0.3f;

		public static bool SplitYawTier => cSplitYawTier == null || cSplitYawTier.Value;


		public static float StabYawRelax => (cStabYawRelax != null) ? Mathf.Clamp01(cStabYawRelax.Value) : 1f;

		public static float SlipPredictRollGate => (cSlipPredRollGate != null) ? Mathf.Clamp(cSlipPredRollGate.Value, 0f, 1f) : 0f;

		public static float SlipPredictWashout => (cSlipPredW != null) ? Mathf.Clamp(cSlipPredW.Value, 0f, 10f) : 1.5f;

		public static float SlipGainHighAoA => (cSlipGainHi != null) ? Mathf.Max(0f, cSlipGainHi.Value) : 4f;

		public static float SplitMu => (cSplitMu != null) ? Mathf.Max(1E-06f, cSplitMu.Value) : 1E-03f;

		public static float SplitBrakeHoldCost => (cSplitBrakeHold != null) ? Mathf.Max(0f, cSplitBrakeHold.Value) : 0.5f;

		public static bool YawTvc => cYawTvc == null || cYawTvc.Value;

		public static float TvcYawMu => (cTvcYawMu != null) ? Mathf.Max(1E-06f, cTvcYawMu.Value) : 0.0005f;

		public static bool ElevonPitch => cElevonPitch == null || cElevonPitch.Value;

		public static bool FlapFlaperons => cFlapFlaperons == null || cFlapFlaperons.Value;

		public static float FlapHingeRange => (cFlapHinge != null) ? Mathf.Clamp(cFlapHinge.Value, 5f, 35f) : 25f;

		public static float FlapHighLiftDroop => (cFlapHiLift != null) ? Mathf.Clamp(cFlapHiLift.Value, 0f, 35f) : 0f;

		public static bool CanardPitchPriority => cCanardPitch == null || cCanardPitch.Value;

		public static bool FlaperonLoadRelief => cFlapRelief != null && cFlapRelief.Value;

		public static float YawTvcNeutralHold => (cYawTvcNeutral != null) ? Mathf.Max(0f, cYawTvcNeutral.Value) : 300f;

		public static bool VisualHingeFix => cVisHinge == null || cVisHinge.Value;

		public static float CorkscrewSlipTol => (cCorkTol != null) ? Mathf.Clamp(cCorkTol.Value, 0f, 15f) : 0f;

		public static float HighAoARollWeight => (cHiAoARollW != null) ? Mathf.Clamp(cHiAoARollW.Value, 0.1f, 5f) : 1f;

		public static bool CanardAttachedWindow => cCanWin == null || cCanWin.Value;

		public static float CanardWindowMargin => (cCanWinMargin != null) ? Mathf.Clamp(cCanWinMargin.Value, 0f, 30f) : 12f;

		public static bool FinLossTvcHold => cFinTvcHold == null || cFinTvcHold.Value;

		public static float FinLossNoTvcAoACap => (cFinNoTvcCap != null) ? Mathf.Clamp(cFinNoTvcCap.Value, 0f, 70f) : 30f;

		public static bool SlipYawTvcAssist => cSlipYawTvc == null || cSlipYawTvc.Value;

		public static float GroundElevonPitch => (cGroundElevonPitch != null) ? Mathf.Clamp01(cGroundElevonPitch.Value) : 1f;

		public static bool CanardYawTier => cCanardYawTier != null && cCanardYawTier.Value;

		public static float ElevonSymCost => (cElevonSymCost != null) ? Mathf.Max(0f, cElevonSymCost.Value) : 2f;

		public static float TvcDiffMu => (cTvcDiffMu != null) ? Mathf.Max(1E-06f, cTvcDiffMu.Value) : 0.003f;

		private static ConfigEntry<float> cGPos;

		private static ConfigEntry<float> cGNeg;

		private static ConfigEntry<float> cMpoGPos;

		private static ConfigEntry<float> cMpoGNeg;

		private static ConfigEntry<float> cMaxPitch;

		private static ConfigEntry<float> cProt;

		private static ConfigEntry<float> cMaxRoll;

		private static ConfigEntry<float> cMaxYaw;

		private static ConfigEntry<float> cCouple;

		private static ConfigEntry<float> cPedalBeta;

		private static ConfigEntry<float> cKBeta;

		private static ConfigEntry<float> cKRoll;

		private static ConfigEntry<float> cKPitch;

		private static ConfigEntry<float> cKYaw;

		private static ConfigEntry<float> cKpAoA;

		private static ConfigEntry<bool> cPairTrim;

		private static ConfigEntry<float> cLatRes;

		private static ConfigEntry<bool> cBranch;

		private static ConfigEntry<bool> cOvPrio;

		private static ConfigEntry<float> cCoupRoll;

		private static ConfigEntry<float> cBranchMaxAoA;

		private static ConfigEntry<float> cBranchTarget;

		private static ConfigEntry<float> cBranchTol;

		private static ConfigEntry<float> cAntiWind;

		private static ConfigEntry<bool> cThrBrake;

		private static ConfigEntry<float> cBrakeBlend;

		private static ConfigEntry<float> cKiAoA;

		private static ConfigEntry<float> cKpG;

		private static ConfigEntry<float> cKiG;



		private static ConfigEntry<float> cActuator;



		private static ConfigEntry<float> cUnloadDecel;

		private static ConfigEntry<float> cUnloadRise;

		private static ConfigEntry<float> cRollLo;

		private static ConfigEntry<float> cRollHi;

		private static ConfigEntry<float> cRollAuth;

		private static ConfigEntry<float> cRollBrake;

		private static ConfigEntry<float> cStabTrim;





		private static ConfigEntry<float> cStabPeakMargin;




		private static ConfigEntry<bool> cTvcOff;

		private static ConfigEntry<AoALimitTest> cAoATest;

		private static ConfigEntry<float> cNoTvcCapPos;

		private static ConfigEntry<float> cNoTvcCapNeg;

		private static ConfigEntry<float> cNoTvcThrCapPos;

		private static ConfigEntry<float> cNoTvcThrCapNeg;

		private static ConfigEntry<float> cLowThrustTW;

		private static ConfigEntry<bool> cRateOnly;

		private static ConfigEntry<PitchCommandLaw> cPitchLaw;

		private static ConfigEntry<bool> cLowCapRoll;

		private static ConfigEntry<bool> cAsymShared;

		private static ConfigEntry<bool> cGovPred;

		private static ConfigEntry<bool> cSweepYaw;

		private static ConfigEntry<bool> cSweepPos;

		private static ConfigEntry<float> cPullSlipMargin;

		private static ConfigEntry<float> cPullRollTrade;

		private static ConfigEntry<float> cPullFFBoostMax;

		private static ConfigEntry<bool> cPullGovSmooth;

		private static ConfigEntry<float> cSplitFlapRollYield;

		private static ConfigEntry<bool> cSplitFlapYaw;

		private static ConfigEntry<float> cSplitFlapYawUse;

		private static ConfigEntry<float> cRollPitchLead;

		private static ConfigEntry<bool> cStabsPitchOnly;

		private static ConfigEntry<bool> cSlatRelief;

		private static ConfigEntry<bool> cCoupGateLim;

		private static ConfigEntry<bool> cLoadedRollSteady;

		private static ConfigEntry<bool> cSlipYawW;


		private static ConfigEntry<bool> cCapLag;


		private static ConfigEntry<float> cLeadK;

		private static ConfigEntry<float> cLeadTau;

		private static ConfigEntry<bool> cFuelBal;

		private static ConfigEntry<bool> cFuelAttached;

		private static ConfigEntry<bool> cFuelBalLog;

		private static ConfigEntry<float> cFuelRate;

		private static ConfigEntry<float> cFuelTolLat;

		private static ConfigEntry<float> cFuelTolLong;





		private static ConfigEntry<bool> cGroundRate;

		private static ConfigEntry<float> cGroundRotRamp;






		private static ConfigEntry<float> cTvcNeutral;

		private static ConfigEntry<bool> cStabPark;

		private static ConfigEntry<float> cSettledMargin;

		private static ConfigEntry<float> cSettledOn;

		private static ConfigEntry<float> cSettledOff;

		private static ConfigEntry<float> cSettledWalk;

		private static ConfigEntry<float> cSettledSlope;

		private static ConfigEntry<float> cSettledYawUse;


		private static ConfigEntry<bool> cRollFightRel;


		private static ConfigEntry<int> cAirframeRev;


		private static ConfigEntry<float> cCoupFF;

		private static ConfigEntry<float> cRelPitchPrio;

		private static ConfigEntry<float> cRelStabRate;

		private static ConfigEntry<float> cWashSlew;

		private static ConfigEntry<float> cMomLead;

		private static ConfigEntry<float> cOnset;

		private static ConfigEntry<float> cStabRoll;

		private static ConfigEntry<float> cStabRollShare;

		private static ConfigEntry<float> cStickSmooth;

		private static ConfigEntry<float> cActDamp;

		private static ConfigEntry<float> cYawStop;

		private static ConfigEntry<float> cOuterFlap;

		private static ConfigEntry<float> cRollAcc;

		private static ConfigEntry<float> cThrust;

		private static ConfigEntry<float> cDrag;

		private static ConfigEntry<float> cDrAoA;

		private static ConfigEntry<float> cStockDrAoA;

		private static ConfigEntry<float> cStockDrFade0;

		private static ConfigEntry<float> cStockDrFade1;



		private static ConfigEntry<float> cDrTrans;


		private static ConfigEntry<float> cYawPri;

		private static ConfigEntry<float> cYawFF;

		private static ConfigEntry<float> cRollLead;

		private static ConfigEntry<float> cFlareScaleStock;

		private static ConfigEntry<float> cFlareScaleBuffed;

		private static ConfigEntry<float> cEwCapStock;

		private static ConfigEntry<float> cEwCapBuffed;

		private static ConfigEntry<float> cEwRateStock;

		private static ConfigEntry<float> cEwRateBuffed;

		private static ConfigEntry<float> cLowYaw;

		private static ConfigEntry<float> cLowYawIas;

		private static ConfigEntry<float> cLowYawFall;

		private static ConfigEntry<float> cTvcMove;

		private static ConfigEntry<float> cRollTvc;

		private static ConfigEntry<float> cRollWash;

		private static ConfigEntry<float> cRollTvcBudget;

		private static ConfigEntry<float> cRollTvcBudgetCost;


		private static ConfigEntry<bool> cNwsFbw;

		private static ConfigEntry<bool> cGndStock;

		private static ConfigEntry<bool> cGearDownSlip;

		private static ConfigEntry<bool> cGndIndi;

		private static ConfigEntry<float> cGndIndiK;

		private static ConfigEntry<float> cGndIndiTau;

		private static ConfigEntry<float> cGndDriftAllow;

		private static ConfigEntry<float> cGndDriftGain;

		private static ConfigEntry<float> cGndDriftLead;

		private static ConfigEntry<float> cGndDriftFade;

		private static ConfigEntry<float> cGndYawRate;

		private static ConfigEntry<float> cGndLatAcc;

		private static ConfigEntry<float> cGndLine;



		private static ConfigEntry<float> cAsymPitchMargin;

		private static ConfigEntry<bool> cAsymRollCoupling;

		private static ConfigEntry<bool> cRollPitchPriority;

		private static ConfigEntry<float> cRollFightCost;


		private static ConfigEntry<float> cPitchFight;

		private static ConfigEntry<float> cRollPitchReserve;



		private static ConfigEntry<float> cGunAmmoStock;

		private static ConfigEntry<float> cGunAmmoBuffed;

		private static ConfigEntry<float> cFuelUse;

		private static ConfigEntry<float> cRotRate;

		private static ConfigEntry<float> cAsymReserve;

		private static ConfigEntry<float> cAsymFloor;

		private static ConfigEntry<float> cAsymFloorNeg;

		private static ConfigEntry<float> cAsymTighten;

		private static ConfigEntry<float> cAsymRelax;

		private static ConfigEntry<float> cAsymRec;

		private static ConfigEntry<float> cAsymMinIas;

		private static ConfigEntry<float> cAsymRollAcc;

		private static ConfigEntry<float> cAsymYawAcc;

		private static ConfigEntry<float> cAsymBoost;

		private static ConfigEntry<float> cAsymDead;

		private static ConfigEntry<float> cAsymFF;

		private static ConfigEntry<float> cAsymFFTau;

		private static ConfigEntry<float> cAsymRollTau;

		private static ConfigEntry<float> cAsymRelaxBoost;

		private static ConfigEntry<float> cRollPitchFF;

		private static ConfigEntry<float> cRollPitchFFAoA;

		private static ConfigEntry<float> cAsymFFBeta;

		private static ConfigEntry<float> cAsymFFAoA;

		private static ConfigEntry<float> cBetaGuard;

		private static ConfigEntry<float> cYawSatGuard;

		private static ConfigEntry<float> cLowQReg;

		private static ConfigEntry<float> cRollStop;

		private static ConfigEntry<float> cRollAccUse;

		private static ConfigEntry<float> cRollAccUseLow;






		private static ConfigEntry<float> cHlMaxAoA;

		private static ConfigEntry<float> cHlHighAoA;


		private static ConfigEntry<float> cRotMinSpeed;

		private static ConfigEntry<float> cSlatStart;

		private static ConfigEntry<float> cSlatFull;


		private static ConfigEntry<float> cGUnder;

		private static ConfigEntry<float> cFFMax;

		private static ConfigEntry<float> cFFStabCost;

		private static ConfigEntry<bool> cGndBrakeSplit;

		private static ConfigEntry<bool> cThrIndepBrake;

		public static bool ThrottleIndependentAirbrake => cThrIndepBrake == null || cThrIndepBrake.Value;




		public static bool Enabled => cEnabled.Value && MpPolicy.FcsAllowed;

		public static bool TelemetryEnabled => cTelemetry.Value;

		public static bool AIFlightControl => cAIFcs.Value;

		public static bool AIFlightLog => cAILog.Value;

		public static bool AIDirectRoll => cAIDirectRoll.Value;

		public static float StabRangeTEUp => Mathf.Clamp(cStabUp.Value, 0f, 45f);

		public static float StabRangeTEDown => Mathf.Clamp(cStabDn.Value, 0f, 45f);

		public static float RudderRange => Mathf.Clamp((0f), 0f, 45f);

		public static float FlaperonRange => Mathf.Clamp(cFlapRange.Value, 0f, 40f);

		public static float TvcRange => Mathf.Clamp(cTvcRange.Value, 0f, 20f);

		public static float TvcSlew => (cTvcSpeed.Value > 0f) ? Mathf.Clamp(cTvcSpeed.Value, 10f, 300f) : (70f * ActuatorScale);

		public static bool AIGunRollFade => cAIGunRollFade.Value;

		// 2.0.8: the debug AoALimit test switch overrides the positive limit (Full65 = 65, Cap36 = 36 at most)
		public static float AoAPos => AoAFull65 ? 65f : (AoACap36 ? Mathf.Min(cAoAPos.Value, NoTvcAoACapPos) : cAoAPos.Value);

		// 2.0.9: AoA caps while the nozzles do not cover pitch (DisableTVC or T/W under LowThrustFixedAoATW), debug-adjustable
		public static float NoTvcAoACapPos => (cNoTvcCapPos != null) ? Mathf.Clamp(cNoTvcCapPos.Value, 10f, 65f) : 45f;

		public static float NoTvcAoACapNeg => (cNoTvcCapNeg != null) ? Mathf.Clamp(Mathf.Abs(cNoTvcCapNeg.Value), 10f, 45f) : 45f;

		// 2.2.2: with DisableTVC, the caps when the thrust is up (positive: from T/W NoTvcThrustCapTW0, full at NoTvcThrustCapTW1)
		public static float NoTvcThrCapPos => (cNoTvcThrCapPos != null) ? Mathf.Clamp(cNoTvcThrCapPos.Value, 10f, 65f) : 45f;

		public static float NoTvcThrCapNeg => (cNoTvcThrCapNeg != null) ? Mathf.Clamp(Mathf.Abs(cNoTvcThrCapNeg.Value), 10f, 45f) : 45f;

		public static bool DisableTVC => cTvcOff != null && cTvcOff.Value;

		public static bool AoAFull65 => cAoATest != null && cAoATest.Value == AoALimitTest.Full65;

		public static bool AoACap36 => cAoATest != null && cAoATest.Value == AoALimitTest.Cap36;

		public static float StabDownwash => Mathf.Clamp((0f), 0f, 0.7f);

		public static float StabPeakMargin => (cStabPeakMargin != null) ? Mathf.Clamp(cStabPeakMargin.Value, 0f, 90f) : 90f;

		public static bool BodyLiftShaping => (false);

		public static float BodyLiftFore => Mathf.Clamp((0.9f), 0.3f, 2f);

		public static float BodyLiftAft => Mathf.Clamp((1.8f), 0.3f, 3f);

		public static float AoANeg => AoACap36 ? Mathf.Max(0f - Mathf.Abs(cAoANeg.Value), 0f - NoTvcAoACapNeg) : (0f - Mathf.Abs(cAoANeg.Value));

		public static float GPos => cGPos.Value;

		public static float GNeg => 0f - Mathf.Abs(cGNeg.Value);

		public static float MpoGPos => cMpoGPos.Value;

		public static float MpoGNeg => 0f - Mathf.Abs(cMpoGNeg.Value);

		public static float MaxPitchRate => cMaxPitch.Value;

		public static float RotationRate => cRotRate.Value;

		public static float ProtRate => cProt.Value;

		public static float MaxRollRate => cMaxRoll.Value;

		public static float MaxYawRate => cMaxYaw.Value;

		public static float RollGainLowIAS => cRollLo.Value;

		public static float RollGainHighIAS => cRollHi.Value;

		public static float RollAuthorityUse => cRollAuth.Value;

		public static float RollBrakeUse => cRollBrake.Value;

		public static float StabTrimCost => cStabTrim.Value;

		public static float LerxEffectiveness => (1f);

		public static bool StallStagger => (false);

		public static float LowThrustTW => cLowThrustTW.Value;

		public static bool RateCommandOnly => cRateOnly.Value;

		public static float StickLeadGain => cLeadK == null ? 0f : Mathf.Clamp(cLeadK.Value, 0f, 3f);

		public static float StickLeadTau => cLeadTau == null ? 0.15f : Mathf.Clamp(cLeadTau.Value, 0.03f, 0.5f);

		public static bool RollGovPredictiveOn => cGovPred == null || cGovPred.Value;

		public static bool SweepYawLimitOn => cSweepYaw == null || cSweepYaw.Value;

		public static bool SplitFlaperonYaw => cSplitFlapYaw == null || cSplitFlapYaw.Value;

		public static float SplitFlaperonYawUse => cSplitFlapYawUse == null ? 1f : Mathf.Clamp(cSplitFlapYawUse.Value, 0f, 1f);

		public static float PullSideslipMargin => cPullSlipMargin == null ? 15f : Mathf.Clamp(cPullSlipMargin.Value, 2f, 15f);

		public static float SplitFlaperonRollYield => cSplitFlapRollYield == null ? 0f : Mathf.Clamp(cSplitFlapRollYield.Value, 0f, 1f);

		public static bool PullGovSmooth => cPullGovSmooth == null || cPullGovSmooth.Value;

		public static float PullFFBoostMax => cPullFFBoostMax == null ? 4f : Mathf.Clamp(cPullFFBoostMax.Value, 1f, 8f);

		public static float PullRollTrade => cPullRollTrade == null ? 0.7f : Mathf.Clamp(cPullRollTrade.Value, 0f, 1f);

		public static bool SweepPosOn => cSweepPos == null || cSweepPos.Value;

		public static bool StabsPitchOnly => cStabsPitchOnly != null && cStabsPitchOnly.Value;

		public static float RollPitchLead => cRollPitchLead == null ? 0.25f : Mathf.Clamp(cRollPitchLead.Value, 0f, 0.6f);

		public static bool SweepPosLimitOn => SweepYawLimitOn && SweepPosOn;

		public static bool SlatRollReliefOn => cSlatRelief != null && cSlatRelief.Value;

		public static bool CoupGateAtLimitOn => cCoupGateLim == null || cCoupGateLim.Value;

		public static bool LoadedRollSteadyOn => cLoadedRollSteady == null || cLoadedRollSteady.Value;

		public static bool SlipYawWeightOn => cSlipYawW == null || cSlipYawW.Value;

		public static bool CapLagOn => cCapLag == null || cCapLag.Value;

		public static float FinCamberK => false ? 0.5f : Mathf.Clamp((0f), 0f, 1f);

		public static float WingCamberK => false ? 0f : Mathf.Clamp((0f), 0f, 1f);

		public static bool AsymSharedLatOn => cAsymShared == null || cAsymShared.Value;

		public static bool LowCapRollFix => cLowCapRoll == null || cLowCapRoll.Value;

		public static bool GRateLaw => cPitchLaw == null || cPitchLaw.Value == PitchCommandLaw.GAndPitchRate;

		public static bool AoAThenRateLaw => cPitchLaw != null && cPitchLaw.Value == PitchCommandLaw.AoAThenRateHold;

		public static bool FuelBalanceOn => cFuelBal.Value;

		public static bool FuelAttachedOnly => cFuelAttached.Value;

		public static bool FuelBalanceLog => cFuelBalLog.Value;

		public static float FuelTransferRate => Mathf.Max(0f, cFuelRate.Value);

		public static bool FuelTrim => cFuelTrim.Value;

		public static float FuelTrimRate => Mathf.Max(0f, cFuelTrimRate.Value);

		public static float FuelBalanceTolLat => Mathf.Max(0.001f, cFuelTolLat.Value);

		public static float FuelBalanceTolLong => Mathf.Max(0.001f, cFuelTolLong.Value);

		public static bool RudderPitchAid => (false);

		public static float RudderToeCost => Mathf.Max(0.001f, (0.05f));

		public static float RudderToeMax => Mathf.Clamp((20f), 0f, 35f);

		public static float RudderToeGain => Mathf.Max(0f, (2f));

		public static bool GroundRateLaw => cGroundRate.Value;

		public static float GroundRotationRamp => Mathf.Max(1f, cGroundRotRamp.Value);

		public static float StabStallAoA => (25f);

		public static float InnerFlaperonStallAoA => (35f);

		public static float OuterFlaperonStallAoA => (45f);

		public static float StallStaggerLiftSlope => (0f);

		public static bool RollFightReleased => cRollFightRel == null || cRollFightRel.Value;

		// Ifrit: the F-22E's speedbrake fade (2.2.16) works on Aryx's Airbrake components; the Ifrit has none
		public static bool AirbrakeAoAFade => false;

		public static float AirbrakeFadeShareFull => 0.4f;

		public static float AirbrakeFadeShareZero => 0.15f;

		public static float FinLossAoACap => false ? 40f : Mathf.Clamp((0f), 0f, 65f);

		public static bool StabParkEscape => cStabPark == null || cStabPark.Value;

		public static float SettledStallMargin => (cSettledMargin == null) ? 6f : Mathf.Clamp(cSettledMargin.Value, 0f, 20f);

		public static float SettledRateOn => (cSettledOn == null) ? 10f : Mathf.Clamp(cSettledOn.Value, 1f, 60f);

		public static float SettledRateOff => (cSettledOff == null) ? 20f : Mathf.Max(SettledRateOn + 1f, cSettledOff.Value);

		public static float SettledWalkRate => (cSettledWalk == null) ? 30f : Mathf.Clamp(cSettledWalk.Value, 1f, 100f);

		public static float SettledSlopeFrac => (cSettledSlope == null) ? 0.15f : Mathf.Clamp(cSettledSlope.Value, 0f, 1f);

		public static float SettledYawUse => (cSettledYawUse == null) ? 0.4f : Mathf.Clamp(cSettledYawUse.Value, 0f, 5f);

		public static float TvcNeutralHold => cTvcNeutral == null ? 1000f : Mathf.Max(0f, cTvcNeutral.Value);

		public static float StabCLScale => false ? 1f : Mathf.Clamp((1f), 0.5f, 2f);

		public static float StabLiftSlope => false ? -1f : Mathf.Clamp((-1f), -1f, 1f);

		public static float CgShiftAft => (0f);

		public static float CouplingFF => cCoupFF.Value;

		public static float ReleasedPitchPriority => cRelPitchPrio.Value;

		public static float ReleasedStabRateBound => Mathf.Max(0f, cRelStabRate.Value);

		public static float RollNozzleWashoutSlew => Mathf.Max(0f, cWashSlew.Value);

		public static float ReleasedMomentLead => Mathf.Clamp(cMomLead.Value, 0f, 0.15f);

		public static float PitchOnsetGps => cOnset.Value;

		public static float StabRollCost => cStabRoll.Value;

		public static float StickSmoothing => cStickSmooth.Value;

		public static float ActuatorDamping => cActDamp.Value;

		public static float YawStopFactor => cYawStop.Value;

		public static float LowIasYawBoost => cLowYaw.Value;

		public static float LowIasYawBoostFullIas => cLowYawIas.Value;

		public static float LowIasYawBoostFalloff => cLowYawFall.Value;

		public static float TvcMoveCost => cTvcMove.Value;

		public static float RollNozzleHold => cRollTvc.Value;

		public static float RollNozzleWashout => cRollWash.Value;

		public static float RollTvcBudget => cRollTvcBudget.Value;

		public static float RollTvcBudgetCost => cRollTvcBudgetCost.Value;

		public static float RudderSideslipYawCap => (2f);

		public static bool NoseWheelFbw => cNwsFbw.Value;

		public static bool GroundStockMatch => cGndStock == null || cGndStock.Value;

		public static bool GearDownSlipProtect => cGearDownSlip == null || cGearDownSlip.Value;

		public static bool GroundIndi => cGndIndi == null || cGndIndi.Value;

		public static float GroundIndiGain => (cGndIndiK == null) ? 15f : Mathf.Max(0f, cGndIndiK.Value);

		public static float GroundIndiTau => (cGndIndiTau == null) ? 0.04f : cGndIndiTau.Value;

		public static float GroundDriftAllow => (cGndDriftAllow == null) ? 8f : cGndDriftAllow.Value;

		public static float GroundDriftGain => (cGndDriftGain == null) ? 0.2f : cGndDriftGain.Value;

		public static float GroundDriftLead => (cGndDriftLead == null) ? 0.25f : cGndDriftLead.Value;

		public static float GroundDriftFadeSpeed => (cGndDriftFade == null) ? 15f : cGndDriftFade.Value;

		public static float GroundYawRateMax => cGndYawRate.Value;

		public static float GroundLateralAccel => cGndLatAcc.Value;

		public static float GroundLineHold => cGndLine.Value;

		public static bool NozzleAnimation => (false);

		public static bool NozzleAnimationAI => (false);

		public static float AsymPitchMargin => cAsymPitchMargin.Value;

		public static bool AsymRollCoupling => cAsymRollCoupling.Value;

		public static bool RollPitchPriority => cRollPitchPriority.Value;

		public static float RollFightCost => cRollFightCost.Value;

		public static float NozzleSeamShade => (0.07f);

		public static float RollPitchFightCost => cPitchFight.Value;

		public static float RollPitchReserve => cRollPitchReserve.Value;

		public static bool NozzleIdleOpen => (true);

		public static float NozzleAreaRate => (1.2f);

		public static float YawPriority => cYawPri.Value;

		public static float RollLead => cRollLead.Value;

		public static float YawRateFF => cYawFF.Value;

		public static float OuterFlapShare => cOuterFlap.Value;

		public static float StabRollShare => cStabRollShare.Value;

		public static float CoupleAoA => cCouple.Value;

		public static float PedalBeta => cPedalBeta.Value;

		public static float KBeta => cKBeta.Value;

		public static float KRoll => cKRoll.Value;

		public static float KPitch => cKPitch.Value;

		public static float KYaw => cKYaw.Value;

		public static float KpAoA => cKpAoA.Value;

		public static bool StabTvcPairedSearch => cPairTrim.Value;

		public static float LateralStabReserve => cLatRes.Value;

		public static bool StabBranchSwitch => cBranch.Value;

		public static bool OvershootPitchPriority => cOvPrio.Value;

		public static float CouplingRollLimit => cCoupRoll.Value;

		public static float StabBranchMaxAoA => cBranchMaxAoA.Value;

		public static float StabBranchTarget => cBranchTarget.Value;

		public static float StabBranchPitchTol => cBranchTol.Value;

		public static float PitchAntiWindup => Mathf.Max(0f, cAntiWind.Value);

		public static bool AoACaptureThrustBrake => cThrBrake.Value;

		public static float AoACaptureBrakeBlend => cBrakeBlend.Value;

		public static float KiAoA => cKiAoA.Value;

		// Revoker 0.1.0: the single centreline rudder keeps its yaw authority to ~75 deg AoA (bench: rudder 25 deg = -40
		// deg/s^2 of stability-axis yaw at 500 km/h from 0 to 75 deg; reversed by 80), unlike the F-22E's canted fins (50-60)
		public static float RudderFadeStart => (75f);

		public static float RudderFadeEnd => (85f);

		public static float UnloadDecel => cUnloadDecel.Value;

		public static float UnloadRiseGain => cUnloadRise.Value;

		public static float MaxRollAccel => cRollAcc.Value;

		private static ConfigEntry<float> cDecelLook;

		private static ConfigEntry<float> cDecelStart;

		private static ConfigEntry<float> cDecelFull;

		private static ConfigEntry<float> cAsymCapDemand;

		private static ConfigEntry<float> cAsymCapSmooth;

		private static ConfigEntry<bool> cHiAoABrake;
		private static ConfigEntry<bool> cCmdShape;

		private static ConfigEntry<float> cShapeASpan;

		private static ConfigEntry<float> cShapeAKSmall;

		private static ConfigEntry<float> cShapeAKLarge;

		private static ConfigEntry<float> cShapeOnset;

		private static ConfigEntry<float> cShapeGSpan;

		private static ConfigEntry<float> cShapeGKSmall;

		private static ConfigEntry<float> cShapeGKLarge;

		private static ConfigEntry<bool> cRateBlend;

		private static ConfigEntry<float> cRateBlendAoA;

		private static ConfigEntry<float> cRateBlendStick;

		private static ConfigEntry<bool> cFuelTrim;

		private static ConfigEntry<float> cFuelTrimRate;

		private static ConfigEntry<bool> cTeBrake;

		private static ConfigEntry<float> cGbElev, cGbInb, cGbCan, cAbElevUp, cAbInbDn;

		private static ConfigEntry<float> cEngOutCap;

		private static ConfigEntry<float> cEngDmgCap;

		private static ConfigEntry<float> cThrResp;

		private static ConfigEntry<bool> cEngAbGate;

		private static ConfigEntry<int> cStickRev;

		private static ConfigEntry<float> cStick24;

		private static ConfigEntry<float> cStick36;

		private static ConfigEntry<float> cRateBlendAoAMin;

		private static ConfigEntry<float> cRateBlendLiftG;

		private static ConfigEntry<float> cRateBlendBand;

		private static ConfigEntry<bool> cRateBlendNeg;

		private static ConfigEntry<float> cRateBlendAoANeg;

		private static ConfigEntry<float> cRateBlendAoAMinNeg;

		private static ConfigEntry<bool> cAuthCap;

		private static ConfigEntry<float> cAuthCapMargin;

		private static ConfigEntry<float> cAuthCapFloor;

		private static ConfigEntry<float> cHybridIndi;

		private static ConfigEntry<float> cHybridHiLead;

		private static ConfigEntry<bool> cAllocNozzleNewton;


		public static float DecelLookahead => Mathf.Max(0f, cDecelLook.Value);

		public static float DecelStart => cDecelStart.Value;

		public static float DecelFull => Mathf.Max(cDecelStart.Value + 0.5f, cDecelFull.Value);

		public static float AsymCapDemand => Mathf.Clamp(cAsymCapDemand.Value, 0.5f, 2f);

		public static float AsymCapSmoothing => Mathf.Max(0.05f, cAsymCapSmooth.Value);

		public static bool HighAoABrakeFloor => cHiAoABrake.Value;

		public static bool CmdShaping => cCmdShape.Value;

		public static float ShapeAoASpan => Mathf.Max(1f, cShapeASpan.Value);

		public static float ShapeAoAGainSmall => Mathf.Max(0.3f, cShapeAKSmall.Value);

		public static float ShapeAoAGainLarge => Mathf.Max(0.3f, cShapeAKLarge.Value);

		public static float ShapeOnsetSmall => Mathf.Clamp(cShapeOnset.Value, 0f, 1f);

		public static float ShapeGSpan => Mathf.Max(0.2f, cShapeGSpan.Value);

		public static float ShapeGGainSmall => Mathf.Max(0.3f, cShapeGKSmall.Value);

		public static float ShapeGGainLarge => Mathf.Max(0.3f, cShapeGKLarge.Value);

		public static bool HighAoARateBlend => cRateBlend.Value;

		public static float RateBlendAoA => Mathf.Clamp(cRateBlendAoA.Value, 10f, 64f);

		public static float RateBlendStick => Mathf.Clamp(cRateBlendStick.Value, 0.2f, 0.9f);

		// 2.0.12 stick curve: stick position of 24 and 36 deg AoA (defaults on the 72 deg per stick line). 24 within
		// 0.333 +-0.25, 36 within 0.5 +-0.2, and 36 at least 0.1 of stick past 24 (36 is pushed out if they come closer).
		public static float EngineOutAoACap => Mathf.Clamp(cEngOutCap.Value, 10f, 65f);

		public static float EngineDamagedAoACap => (cEngDmgCap != null) ? Mathf.Clamp(cEngDmgCap.Value, 10f, 65f) : 40f;

		public static float ThrottleResponseScale => (cThrResp != null) ? Mathf.Clamp(cThrResp.Value, 0.25f, 4f) : 1f;

		public static bool EngineAbAoAGate => cEngAbGate == null || cEngAbGate.Value;

		public static float StickAoA24 => Mathf.Clamp(cStick24.Value, 0.0833f, 0.7f);

		public static float StickAoA36 => Mathf.Clamp(Mathf.Max(Mathf.Clamp(cStick36.Value, 0.3f, 0.8f), StickAoA24 + 0.1f), 0.3f, 0.95f);

		public static float RateBlendAoAMin => Mathf.Clamp(cRateBlendAoAMin.Value, 10f, 64f);

		public static float RateBlendLiftG => Mathf.Max(1.1f, cRateBlendLiftG.Value);

		public static float RateBlendBand => Mathf.Clamp(cRateBlendBand.Value, 0.5f, 15f);

		public static bool HighAoARateBlendNeg => cRateBlendNeg.Value;

		public static float RateBlendAoANeg => Mathf.Clamp(Mathf.Abs(cRateBlendAoANeg.Value), 10f, 64f);

		public static float RateBlendAoAMinNeg => Mathf.Clamp(Mathf.Abs(cRateBlendAoAMinNeg.Value), 10f, 64f);

		public static bool LowThrustAoACap => cAuthCap.Value;

		public static float AuthCapMargin => cAuthCapMargin.Value;

		public static float AuthCapFloor => Mathf.Clamp(cAuthCapFloor.Value, 5f, 65f);

		public static float HybridIndiGain => Mathf.Clamp(cHybridIndi.Value, 0f, 1.5f);

		public static float HybridIndiHighAoALead => (cHybridHiLead != null) ? Mathf.Clamp(cHybridHiLead.Value, 0f, 0.3f) : 0.15f;

		public static bool AllocNozzleNewton => cAllocNozzleNewton == null || cAllocNozzleNewton.Value;


		private static ConfigEntry<float> cYawRefKmh;

		private static ConfigEntry<float> cYawMargStart;

		private static ConfigEntry<float> cYawMargFull;

		private static ConfigEntry<float> cYawMargLow;

		public static float YawBrakeRefKmh => Mathf.Max(50f, cYawRefKmh.Value);

		public static float YawMarginStartKmh => cYawMargStart.Value;

		public static float YawMarginFullKmh => Mathf.Min(cYawMargFull.Value, cYawMargStart.Value - 1f);

		public static float YawMarginLowIas => Mathf.Max(1f, cYawMargLow.Value);

		private static ConfigEntry<float> cYawMidF;

		private static ConfigEntry<float> cYawMidLo;

		private static ConfigEntry<float> cYawMidHi;

		private static ConfigEntry<float> cYawMidRamp;

		public static float YawMidFactor => Mathf.Clamp(cYawMidF.Value, 0.2f, 2f);

		public static float YawMidLoKmh => cYawMidLo.Value;

		public static float YawMidHiKmh => Mathf.Max(cYawMidHi.Value, cYawMidLo.Value);

		public static float YawMidRampKmh => Mathf.Max(1f, cYawMidRamp.Value);

		// Ifrit 0.4.0 multiplayer: the Local* values are this machine's settings; the plain ones are what applies here, which
		// as a client is the host's policy (MpPolicy): the host's variant when it locks it, stock otherwise.
		public static bool HostAllowFcs => cHostAllowFcs == null || cHostAllowFcs.Value;

		public static bool HostLockVariant => cHostLock != null && cHostLock.Value;

		public static bool LocalStockPerformance => cProfile != null && cProfile.Value == PerformanceProfile.Stock;

		public static float LocalThrustScale => LocalStockPerformance ? 1f : cThrust.Value;

		public static float LocalFlareCountScale => LocalStockPerformance ? cFlareScaleStock.Value : cFlareScaleBuffed.Value;

		public static float LocalEWCapacityScale => LocalStockPerformance ? cEwCapStock.Value : cEwCapBuffed.Value;

		public static float LocalEWRechargeScale => LocalStockPerformance ? cEwRateStock.Value : cEwRateBuffed.Value;

		public static float LocalGunAmmoScale => LocalStockPerformance ? cGunAmmoStock.Value : cGunAmmoBuffed.Value;

		public static float LocalFuelUseScale => LocalStockPerformance ? 1f : cFuelUse.Value;

		public static float LocalDragScale => LocalStockPerformance ? 1f : cDrag.Value;

		public static float LocalDragReliefAoAHigh => LocalStockPerformance ? 0f : Mathf.Clamp(cDrAoA.Value, 0f, 0.5f);

		public static float LocalDragReliefTransonic => LocalStockPerformance ? 0f : Mathf.Clamp(cDrTrans.Value, 0f, 0.5f);

		public static bool StockPerformance => MpPolicy.StockForced || (MpPolicy.VariantLocked ? MpPolicy.Host.stock : LocalStockPerformance);

		public static float ThrustScale => MpPolicy.VariantLocked ? MpPolicy.Host.thrust : (MpPolicy.StockForced ? 1f : LocalThrustScale);

		public static float FlareCountScale => MpPolicy.VariantLocked ? MpPolicy.Host.flare : (MpPolicy.StockForced ? 1f : LocalFlareCountScale);

		public static float EWCapacityScale => MpPolicy.VariantLocked ? MpPolicy.Host.ewCap : (MpPolicy.StockForced ? 1f : LocalEWCapacityScale);

		public static float EWRechargeScale => MpPolicy.VariantLocked ? MpPolicy.Host.ewRate : (MpPolicy.StockForced ? 1f : LocalEWRechargeScale);

		public static float GunAmmoScale => MpPolicy.VariantLocked ? MpPolicy.Host.gun : (MpPolicy.StockForced ? 1f : LocalGunAmmoScale);

		public static float FuelUseScale => MpPolicy.VariantLocked ? MpPolicy.Host.fuel : (MpPolicy.StockForced ? 1f : LocalFuelUseScale);

		public static float DragScale => MpPolicy.VariantLocked ? MpPolicy.Host.drag : (MpPolicy.StockForced ? 1f : LocalDragScale);

		// 2.2.3 (rev 2): AoA drag relief as a curve over each part's local AoA. Up to DragReliefFadeStart it is DragReliefAoA
		// (LowAoADragRelief, both profiles); across the fade band it goes over to DragReliefAoAHigh: none in Stock (post-stall
		// drag is Aryx's), the old flat AoADragRelief in Buffed.
		public static float DragReliefAoA => MpPolicy.FcsAllowed ? Mathf.Clamp(cStockDrAoA.Value, 0f, 0.5f) : 0f;

		public static float DragReliefAoAHigh => MpPolicy.VariantLocked ? MpPolicy.Host.drAoAHigh : (MpPolicy.StockForced ? 0f : LocalDragReliefAoAHigh);

		public static float DragReliefFadeStart => Mathf.Clamp(cStockDrFade0.Value, 5f, 60f);

		public static float DragReliefFadeEnd => Mathf.Clamp(Mathf.Max(cStockDrFade1.Value, cStockDrFade0.Value + 1f), 6f, 70f);

		public static float HighAoALatMargin => (cLatMargin != null) ? Mathf.Clamp(cLatMargin.Value, 0.5f, 1.5f) : 0.86f;

		public static float HighAoALatMarginStart => (cLatMargin0 != null) ? Mathf.Clamp(cLatMargin0.Value, 0f, 89f) : 25f;

		public static float HighAoALatMarginFull => (cLatMargin1 != null) ? Mathf.Clamp(cLatMargin1.Value, HighAoALatMarginStart + 0.5f, 90f) : 30f;

		private static ConfigEntry<float> cLatMargin, cLatMargin0, cLatMargin1;

		public static bool DeceleronSplitBudget => cSplitBudget == null || cSplitBudget.Value;

		private static ConfigEntry<bool> cSplitBudget;

		public static float AoADragBump => (cDragBump != null) ? Mathf.Clamp(cDragBump.Value, 0f, 1f) : 0.3f;

		public static float AoADragBumpStart => (cDragBump0 != null) ? Mathf.Clamp(cDragBump0.Value, 0f, 89f) : 18f;

		public static float AoADragBumpPeak => (cDragBump1 != null) ? Mathf.Clamp(cDragBump1.Value, AoADragBumpStart, 89f) : 24f;

		public static float AoADragBumpEnd => (cDragBump2 != null) ? Mathf.Clamp(cDragBump2.Value, AoADragBumpPeak + 1f, 90f) : 35f;

		private static ConfigEntry<float> cDragBump, cDragBump0, cDragBump1, cDragBump2;

		public static float DragReliefTransonic => MpPolicy.VariantLocked ? MpPolicy.Host.drTrans : (MpPolicy.StockForced ? 0f : LocalDragReliefTransonic);


		public static float KpG => cKpG.Value;

		public static float KiG => cKiG.Value;

		public static float FlaperonEffectiveness => (1f);

		public static float StabilatorEffectiveness => (1f);

		public static float ActuatorScale => cActuator.Value;

		public static bool AsymLimiter => cAsymOn.Value;

		public static bool AsymRollHeadroom => cAsymRollHead.Value;

		public static float AsymReserve => cAsymReserve.Value;

		public static float AsymCapFloor => Mathf.Abs(cAsymFloor.Value);

		public static float AsymCapFloorNeg => Mathf.Abs(cAsymFloorNeg.Value);

		public static float AsymTighten => Mathf.Max(0.1f, cAsymTighten.Value);

		public static float AsymRelax => Mathf.Max(0.1f, cAsymRelax.Value);

		public static float AsymRecoveryRate => Mathf.Max(1f, cAsymRec.Value);

		public static float AsymMinIas => cAsymMinIas.Value;

		public static float AsymRollReserveAccel => Mathf.Max(0f, cAsymRollAcc.Value);

		public static float AsymYawReserveAccel => Mathf.Max(0f, cAsymYawAcc.Value);

		public static float AsymAxisBoost => cAsymBoost.Value;

		public static float AsymDeadband => cAsymDead.Value;

		public static float AsymFeedForward => cAsymFF.Value;

		public static float AsymFFWashout => cAsymFFTau.Value;

		public static float AsymFFMaxBeta => cAsymFFBeta.Value;

		public static float AsymFFMaxAoA => cAsymFFAoA.Value;

		public static float SideslipGuardStart => cBetaGuard.Value;

		public static float YawSatGuard => Mathf.Clamp01(cYawSatGuard.Value);

		public static float LowQRegRelief => cLowQReg.Value;

		public static float RollStopFactor => cRollStop.Value;

		public static float RollAccelUse => Mathf.Clamp(cRollAccUse.Value, 0.1f, 0.95f);

		public static float RollAccelUseLowIas => Mathf.Clamp(cRollAccUseLow.Value, 0.1f, 0.95f);

		// Revoker 0.2.8 (TrailingEdgeBrakeConfig): the F-22E's brake surface configurations, in the air and on the runway.
		// Off with ThrottleIndependentAirbrake off (the airbrake is then fully stock).
		public static bool TeBrakeConfig => cTeBrake == null || cTeBrake.Value;

		public static float GroundAeroBrake => Mathf.Clamp01((1f));

		public static float GroundBrakeMinSpeed => (0f);

		public static float GroundBrakeFlap => (cGbElev != null) ? Mathf.Clamp(cGbElev.Value, 0f, 30f) : 30f;

		public static float GroundBrakeInboard => (cGbInb != null) ? Mathf.Clamp(cGbInb.Value, 0f, 25f) : 25f;

		public static float GroundBrakeStab => (cGbCan != null) ? Mathf.Clamp(cGbCan.Value, 0f, 45f) : 45f;

		public static float GroundBrakeRudder => (25f);

		public static float HighLiftMaxAoA => cHlMaxAoA.Value;

		public static float SlatAoAStart => cSlatStart.Value;

		public static float SlatAoAFull => cSlatFull.Value;

		public static float HighLiftHighAoA => Mathf.Clamp01(cHlHighAoA.Value);

		public static float GroundBrakeTvc => (0f);

		public static float AirBrakeElevonUp => (cAbElevUp != null) ? Mathf.Clamp(cAbElevUp.Value, 0f, 30f) : 20f;

		public static float AirBrakeInboardDown => (cAbInbDn != null) ? Mathf.Clamp(cAbInbDn.Value, 0f, 25f) : 25f;

		public static float GroundBrakeThrottle => (-1f);

		public static bool GroundBrakeLog => (false);

		public static bool AirbrakeSplit => cGndBrakeSplit.Value;

		public static string VentDoors => ("Bypass");

		public static float RotationMinSpeed => Mathf.Max(20f, cRotMinSpeed.Value);

		public static bool RevFlowRollSense => cRevRoll.Value;

		public static float AsymRollTau => cAsymRollTau.Value;

		public static float AsymRelaxBoost => cAsymRelaxBoost.Value;

		public static float RollPitchFF => cRollPitchFF.Value;

		public static float RollPitchFFAoA => cRollPitchFFAoA.Value;

		public static float GUndershootCapture => cGUnder.Value;

		public static float RollPitchFFMax => Mathf.Max(1f, cFFMax.Value);

		public static float RollPitchFFStabCost => cFFStabCost.Value;

		public static void Init(ConfigFile cfg, ConfigFile dev, bool devUnlocked, bool devFresh, ManualLogSource log)
		{
			Log = log;
			cProfile = cfg.Bind("1. General", "PerformanceProfile", PerformanceProfile.Stock, "Stock (default) = the game's FS-12 Revoker thrust, parasitic drag and fuel burn. Buffed = thrust x1.08 (dry and afterburner), parasitic drag x0.90, fuel burn x0.65 at every throttle setting (35 % less). Countermeasures and gun ammo come from the matching Countermeasures section below. The flight control is the same in both. Switch before spawning: thrust, drag, countermeasures and ammo are set when a jet spawns and stay until it respawns; fuel burn follows the setting live.");
			cThrIndepBrake = cfg.Bind("1. General", "ThrottleIndependentAirbrake", defaultValue: true, "What opens the Revoker's airbrake. true = the brake key / axis only, in the air and on the ground; idle or zero throttle never opens it. false = 0.2.6: the stock rule only: throttle at zero opens it; the brake key is the wheel brake only (it no longer opens it, so Brake and Countermeasures can share a controller button). Only while the FCS flies the jet.");
			cTeBrake = cfg.Bind("1. General", "TrailingEdgeBrakeConfig", defaultValue: true, "Revoker 0.2.8: the brake also sets the control surfaces as the F-22E does. In the air, with the airbrake out: inboard elevons trailing edge down, outboard elevons trailing edge up, moved as fast as the actuators go (0.2.9), with the outboard share solved for zero added lift and the canards trimming the pitch; roll stays on the differential. On the runway (main wheels down, brake held, stick not pulled back, from touchdown on): every trailing-edge surface fully up to dump the wing lift and the canards leading edge fully down for drag and nose-wheel load. Needs ThrottleIndependentAirbrake = true (with it false the airbrake is fully stock). false = the dedicated airbrake only (0.2.7). Takes effect at once.");
			cEnabled = cfg.Bind("1. General", "Enabled", defaultValue: true, "Replace the stock fly-by-wire with the Revoker FCS (player aircraft only).");
			cHostAllowFcs = cfg.Bind("8. Multiplayer (host)", "HostAllowFcs", defaultValue: true, "Read on the HOST (or dedicated server). true = players who run this plugin may fly their Revokers with its control laws on this server. false = every client flies the stock fly-by-wire. A client whose host does not run this plugin always flies stock (fly-by-wire, thrust, drag, fuel burn, countermeasures and ammo), whatever its own settings. The host's own jet is bound by this switch too. While this or HostLockVariant is on, the server is flagged as modded (server browser / NOMM 'Modded' tag).");
			// Ifrit 0.4.7: HostLockVariant on by default. A player file written by an older build still says "# Default value:
			// false" above the key; if it also still holds false (the old default), it is moved to true once. The file is then
			// saved, which rewrites the default line, so a later false set by the player stays.
			bool hostLockOldFile = HostLockOldDefault(cfg.ConfigFilePath);
			cHostLock = cfg.Bind("8. Multiplayer (host)", "HostLockVariant", defaultValue: true, "Read on the HOST. On by default. true = every client gets the host's variant: the host's PerformanceProfile with its thrust, drag and fuel-burn scaling and the host's countermeasure and gun-ammo capacities from the matching Countermeasures section. false = every client flies the Stock variant with stock countermeasures and ammo (their own profile and capacity settings are ignored while connected); the host flies Stock on its own server as well. Changes are sent to the clients at once and apply at the next spawn.");
			if (hostLockOldFile)
			{
				if (!cHostLock.Value)
				{
					cHostLock.Value = true;
				}
				cfg.Save();
				Log.LogInfo("Ifrit 0.4.7 defaults: HostLockVariant on.");
			}
			cRateOnly = cfg.Bind("1. General", "PitchRateCommandOnly", defaultValue: false, "2.0.6: true = the pitch stick is a pitch-rate command in every mode (gear up or down, on the runway, Stability Assist on or off), like the gear-down law. With Stability Assist on, the AoA and G limits and AoA protection still apply; with it off (MPO) the MPO limits apply as before. false = AoA/G command with the gear up (default). Takes effect at once.");
			cPitchLaw = cfg.Bind("1. General", "PitchCommandLaw", PitchCommandLaw.GAndPitchRate, "2.1.0: the gear-up pitch law (Stability Assist on). GAndPitchRate (default) = the stick commands pitch rate where full G is out of reach (low IAS) and G where it is in reach, blended as before; no AoA command - the AoA limits, the low-thrust / asymmetric-load caps and AoA protection still apply as limits. GAoAAndPitchRate = the 1.9-2.0 law: AoA command at low IAS (stick curve in section 5), G command at high IAS, pitch-rate command past the hand-over AoA. AoAThenRateHold (2.1.8) = as GAoAAndPitchRate, but once the stick is steady and the AoA has settled on its command, the pitch rate the jet has then is held instead of the AoA (the AoA drifts with speed); moving the stick goes back to AoA command. PitchRateCommandOnly = true overrides all of them. Takes effect at once.");
			cFuelBal = dev.Bind("6. Fuel", "FuelBalancing", defaultValue: true, "2.0.6: move fuel between the healthy tanks to hold the centre of mass where the clean jet has it (centreline; stock fore/aft position without stores), against store asymmetry, a part shot off or a leaking tank. Only between attached, non-leaking, non-burning tanks; mass is conserved. All Ifrits simulated on this machine.");
			cEngOutCap = dev.Bind("Asymmetric load", "EngineOutAoACap", 22f, "Ifrit 0.4.0: 22 (was 30; the Ifrit's engines sit 1.19 m off the centreline, one at AB is 24 deg/s^2 of yaw against 10-24 from the tails at 200-300 km/h and 20 deg AoA; bench one-engine pulls from 300 km/h: 18-25 hold without the throttle cut, 30 bleeds to 50-85 m/s and departs). 2.0.13: with an engine out the AoA limit is this, both signs, steady (plus whatever an aerodynamic asymmetry from damage needs on top). The dead engine's thrust yaw is left to the VMC protection (throttle, last resort) instead of the AoA cap, which made the limit follow speed and throttle (12 <-> 65 deg within seconds in 2.0.12).");
			cEngDmgCap = dev.Bind("Asymmetric load", "EngineDamagedAoACap", 40f, "Ifrit 0.4.2: 40 (first 0.4.1 / 2.4.1 cut: 30, the engine-out cap; a running engine's split is a third to a half of a dead one's, and below the speed where the fins could not hold it the thrust match now takes over on prediction, so the cap only says where the full split is traded for matched thrust). Ifrit 0.4.1: with one engine running weak (damaged, not destroyed: under 85 % of the other's thrust, released above 92 %, 1 s filter) the AoA limit is this fixed value, both signs, instead of the dynamic thrust cap (which fell to 12 deg within 5 s in the 22 Oct 22:08 log and then followed IAS). The dead-engine case keeps EngineOutAoACap. The game caps a damaged engine at 50-100 % of its RPM range by its condition (no afterburner below condition ~0.6), so the weak engine makes 35-75 % of the good one's thrust at AB. Bench, full pulls from 300-400 km/h with the left engine at condition 0 / 0.5: 26-30 hold with 5-8 deg peak sideslip, 36 saturates the yaw once the speed is gone (throttle cut a third of the time, one departure), 45 departs.");
			cThrResp = dev.Bind("5. Airframe", "ThrottleResponseScale", 1f, "Ifrit 0.4.1: throttle response (Ifrit 0.4.5: back to the game's own, 1) for every variant (Stock included): the engines' spool rate (RPM per second toward the throttle's target) and the afterburner's light-up / fade speed are multiplied by this at the airframe setup. 1 = the mod's own values.");
			cEngAbGate = dev.Bind("Asymmetric load", "EngineAbAoAGate", true, "Ifrit 0.4.1: thrust-match limiter. With one engine down on thrust (predicted from the engines' damage state: an engine's reachable RPM is minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5), the dry thrust follows the RPM and the afterburner needs 85 % of maxRPM, i.e. condition above 0.55) and the AoA above EngineDamagedAoACap, the throttle input is held where the two engines still make the same thrust, so there is no thrust split and no AoA cap for it: you keep the full AoA, and the thrust comes back once you bring the AoA down to the cap yourself (on above cap + 2, off under cap + 0.5). Below the cap the throttle is free; the thrust split then brings the fixed EngineDamagedAoACap. With an engine OUT there is nothing to match: above EngineOutAoACap the throttle is held at military power (the afterburner's start). The VMC throttle protection (measured sideslip run-away) stays as the last resort. false = 0.4.0 (dynamic cap from the thrust split, nose pushed down).");
			cFuelTrim = dev.Bind("6. Fuel", "FuelTrim", defaultValue: true, "2.0.12: when the yaw or roll authority runs short (an engine out, a fin or wing section shot off), the balancing moves the CG sideways with fuel instead of holding it on the centreline: toward the live engine to shorten its thrust lever arm, or against an aerodynamic asymmetry. Only once the yaw or roll usage passes 0.75, only as far as evens out the roll and yaw usage (a sideways CG costs roll: ~15 deg/s^2 per g per 10 cm), at most 30 cm; back to the centreline once both are under 0.5 for 2 s. false = centreline always (2.0.6-2.0.11).");
			cFuelTrimRate = dev.Bind("6. Fuel", "FuelTrimRate", 240f, "2.0.12: fuel moved per second (kg/s, in total) while the fuel trim works (normal balancing: FuelTransferRate). Ifrit 0.4.1: 240 (was 120).");
			cFuelRate = dev.Bind("6. Fuel", "FuelTransferRate", 80f, "2.0.6: total fuel the balancing moves per second (kg/s). Ifrit 0.4.1: 80 (was 40).");
			cFuelTolLat = dev.Bind("6. Fuel", "FuelBalanceLateralTolerance", 0.01f, "2.0.6: lateral CG error (m) the balancing weighs as one unit; it leaves the fuel alone inside half of it.");
			cFuelTolLong = dev.Bind("6. Fuel", "FuelBalanceLongitudinalTolerance", 0.03f, "2.0.6: fore/aft CG error (m) the balancing weighs as one unit; it leaves the fuel alone inside half of it.");
			cFuelAttached = dev.Bind("6. Fuel", "FuelFromAttachedTanksOnly", defaultValue: true, "2.0.6: engines draw fuel only from tanks still attached, and the fuel gauge / fuel checks count only those (against the full original capacity); a tank on a part that has been shot off is not refuelled. false = stock (a shot-off tank's fuel keeps feeding the engines and showing on the gauge).");
			cFuelBalLog = dev.Bind("0. Debug", "FuelBalanceLog", defaultValue: false, "2.0.6: log the fuel balancing's plan (CG error and the fuel it moves) when it starts and every 10 s while it works.");
			cFlareScaleStock = cfg.Bind("3. Countermeasures - Stock profile", "FlareCountScale", 1f, "Flare capacity multiplier (the load at spawn and what rearming refills to). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cGunAmmoStock = cfg.Bind("3. Countermeasures - Stock profile", "GunAmmoScale", 1f, "Gun ammunition multiplier: magazine capacity, so the load at spawn and what rearming refills to. Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cEwCapStock = cfg.Bind("3. Countermeasures - Stock profile", "EWCapacityScale", 1f, "Electrical storage multiplier for the EW/jammer power supply (more jamming time per charge). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cEwRateStock = cfg.Bind("3. Countermeasures - Stock profile", "EWRechargeScale", 1f, "Recharge-rate multiplier for the EW/jammer power supply (engine-driven charging). Used when PerformanceProfile = Stock. 1 = stock. All Ifrits, applied at spawn.");
			cFlareScaleBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "FlareCountScale", 10f, "Flare capacity multiplier (the load at spawn and what rearming refills to). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cGunAmmoBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "GunAmmoScale", 2f, "Gun ammunition multiplier: magazine capacity, so the load at spawn and what rearming refills to. Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cEwCapBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "EWCapacityScale", 1.5f, "Electrical storage multiplier for the EW/jammer power supply (more jamming time per charge). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cEwRateBuffed = cfg.Bind("4. Countermeasures - Buffed profile", "EWRechargeScale", 1.2f, "Recharge-rate multiplier for the EW/jammer power supply (engine-driven charging). Used when PerformanceProfile = Buffed. 1 = stock. All Ifrits, applied at spawn.");
			cTelemetry = dev.Bind("0. Debug", "Telemetry", defaultValue: false, "Write a 50 Hz CSV of the FCS state to BepInEx/Ifrit_FCS_<time>.csv.");
			cAIFcs = cfg.Bind("2. AI", "AIFlightControl", defaultValue: false, "Off by default (every AI Revoker in a mission would otherwise fly this FCS). true = AI-flown Revokers simulated on this machine (no human pilot) get the full FCS with a pitch-rate command law (the AI's stick is a pitch-rate command; AoA/G protection, coordination and the allocator as for a player). Takes effect at the next spawn. Experimental.");
			cAILog = dev.Bind("0. Debug", "AIFlightLog", defaultValue: false, "Every 20 s, log one summary line per AI F-22E the FCS is flying (mode, IAS, AoA and g range, time in AoA protection), so AI jets can be checked from BepInEx/LogOutput.log after any mission. Each F-22E also logs once which control path it takes when it first reaches the flight-control hook.");
			cAIDirectRoll = cfg.Bind("2. AI", "AIDirectRoll", defaultValue: true, "AI Revokers only: the AI's roll stick drives the velocity-vector roll rate almost directly (up to the jet's roll authority, still capped by the roll limits).");
			cAIGunRollFade = cfg.Bind("2. AI", "AIGunRollFade", defaultValue: true, "AI Revokers only: while the AI points its nose (guns) at a moving target, its roll stick is faded out when the aim error is small, so it does not wobble the gun solution.");
			cAoAPos = dev.Bind("2. Envelope", "AoALimitPositive", 70f, "AoA limit (deg) in AoA/G command and gear-down rate command. Revoker 0.2.0: 70 (F-22E/Ifrit 65). Bench, full aft + full roll at the limit, 300/450 km/h: 65 -> 5.3/5.6 deg max sideslip, 70 -> 12.7/9.9, 75 -> 23/22, 80 -> 37/35, 90 -> 44/36. The pitch axis holds to 90 (yaw TVC); past 70-75 the body-roll authority a velocity-vector roll needs runs out (all four elevons and the canards saturated), so 70 is the highest that keeps a loaded roll inside 15 deg of sideslip.");
			cAoANeg = dev.Bind("2. Envelope", "AoALimitNegative", 55f, "Negative AoA limit magnitude (deg), with the nozzle covering pitch. Revoker 0.2.7: 55 (canards at +-45 kept in their attached flow, CanardAttachedWindow; bench, full push + roll / pedal at 150-300 km/h, light and full fuel: -55 holds with sideslip <= 4.4 deg and 69-78 deg/s velocity-vector roll, -60 held too with up to 7.6 deg of sideslip on pedal; -50 before 0.2.7 gave 63-73 deg/s). Revoker 0.1.0: 50 (the F-22E's hard limit). Bench, 500 km/h full thrust: canards + TVC still hold ~+100 deg/s^2 of nose-up margin at -50..-60.");
			cFlapCamber = dev.Bind("5. Airframe", "FlapCamberCoupling", 0.46f, "Revoker 0.2.0: the inboard elevon (flap_L/R) also cambers the wing2 section ahead of it: wing2's lift normal turns by K x the flap deflection (the stock HLD turns it 4 deg for 25, K 0.16). Thin-airfoil effectiveness of the 0.24-chord flap (1.2 of 4.9 m) is 0.60; host share (0.60 x 5.2 - 1.4) / 3.8 = 0.46. 0 = off.");
			cAilCamber = dev.Bind("5. Airframe", "AileronCamberCoupling", 0.45f, "Revoker 0.1.0: the outboard elevon (aileron_L/R) also cambers the wingtip section ahead of it: the wingtip's lift normal turns by K x the deflection. Thin-airfoil effectiveness of the 0.30-chord surface (1.02 of 3.41 m) is 0.66; host share (0.66 x 5.25 - 2.0) / 3.25 = 0.45. 0 = off.");
			cLatMargin = cfg.Bind("1. General", "HighAoALateralBrakeMargin", 0.86f, new ConfigDescription("High-AoA lateral braking margin. Above HighAoALateralMarginFull (30 deg AoA, dev cfg) the roll- and yaw-rate limits are multiplied by this, blended in smoothly from HighAoALateralMarginStart (25). 0.86 = 14 % less allowed stop time: 14 % less roll and yaw rate for the same authority, the rotation stops sooner. 1 = 0.3.0. Takes effect at once.", new AcceptableValueRange<float>(0.5f, 1.5f)));
			cMpoSep = cfg.Bind("1. General", "MpoRollMode", MpoRollMode.SeparateRollYaw, "The lateral law with Stability Assist off (MPO). VelocityVectorRoll = as with it on: the stick rolls about the velocity vector, the sideslip is held and the pedals work as they do with it on. SeparateRollYaw (default) = Revoker 0.2.7: the stick rolls about the velocity vector at every AoA (sideslip held while the stick is in and the pedals centred) and the pedals command pure body yaw rate at every AoA (no sideslip limit on them; with both centred, below the coupling AoA, sideslip washes out slowly as before). Before 0.2.7 the stick was body roll, coordinated below the coupling AoA by a sideslip limit on the yaw axis. Takes effect at once.");
			cMpoWash = dev.Bind("3. Lateral-directional", "MpoSideslipWashout", 0.5f, "Revoker 0.2.7: pedals centred only (the pedals themselves are pure body yaw, with no sideslip limit). Ifrit 0.2.0, separate axes: with the pedals centred and below the coupling AoA, the yaw axis lets sideslip wash out at this rate (deg/s of yaw rate per deg of sideslip), like the airframe's own weathervane. Sideslip is kept inside +-PedalSideslip there. 0 = the yaw rate is held exactly (sideslip stays where the roll left it).");
			cIfritRev = dev.Bind("2. Envelope", "IfritConfigRevision", 8, "Internal: config revision (inherited from the Ifrit FCS). Leave it alone.");
			cIfritSplit = dev.Bind("3. Lateral-directional", "SplitFlapYaw", defaultValue: true, "Ifrit: the split-flap decelerons are allocator effectors (drag yaw; the game splits a flap from its job's yaw input). The stock zero-throttle airbrake split is suppressed while the FCS flies. false = the flaps never split while the FCS is engaged. Applied when the jet spawns.");
			cIfritTvcRoll = dev.Bind("3. Lateral-directional", "RollTvc", defaultValue: true, "Ifrit: differential (roll) nozzle deflection about the pitch angle, the Ifrit's stock roll vectoring (+-15 deg), as an allocator effector. false = pitch-only TVC like the F-22E law. Applied when the jet spawns.");
			cSplitMu = dev.Bind("4. Gains", "SplitFlapCost", 1E-03f, "Ifrit: allocator cost of split-flap use (drag). Bench: roll/yaw results unchanged from 1e-4 to 5e-3.");
			cSlipTvc = dev.Bind("3. Lateral-directional", "SlipTvcAssist", true, "Ifrit 0.3.0: the sideslip correction may use body roll up to what the wing surfaces plus the differential TVC can brake, instead of being held to the aero yaw-rate limit. At high AoA the correction is mostly body roll (x sin AoA), and the nozzles roll the jet at any IAS. false = 0.2.2.");
			cIfritFlaperons = dev.Bind("5. Airframe", "IfritFlaperons", true, "Ifrit 0.3.0: the decelerons (inner) and ailerons (outer) work as flaperons on the F-22E flap schedule: up to 23 deg (inner) and OuterFlaperonDroop x that (outer) TE-down with AoA, load and speed (the high-lift schedule), held off the stall on each surface's own flow, out of the way of full roll stick, faded out from 20 to 30 deg AoA. false = 0.2.2 (no droop).");
			cTvcRollNeutral = dev.Bind("4. Gains", "TvcRollNeutralHold", false, "Ifrit 0.3.0, default OFF: on the bench any weight here makes the released-stick pitch hold sluggish (q -0.3 deg/s bleeding off over ~7 s in level flight at 400 km/h, the stabs held still), an interaction with the roll-fight cost. TvcNeutralHold for the differential (roll) nozzles. With the lateral stick and pedals centred, the roll on its command and the sideslip under 2 deg, the nozzles go back to symmetric and the wing surfaces hold the roll trim; any roll input, roll error or sideslip frees them at once. false = 0.2.2.");
			cTvcRollNeutralW = dev.Bind("4. Gains", "TvcRollNeutralWeight", 30f, "Ifrit 0.3.0: cost weight of TvcRollNeutralHold.");
			cSlipGainHi = dev.Bind("3. Lateral-directional", "SlipGainHighAoA", 4f, "Ifrit 0.3.0 (SlipTvcAssist): the sideslip gain (KBeta) ramps to this from 25 to 45 deg AoA, where the correction is mostly body roll from the nozzles. Bench, 65 deg corkscrew, 1 s full-pedal kick: back under 3 deg in 4.3-4.7 s at 1.3, 0.5-0.8 s at 4. Below 25 deg KBeta applies as before.");
			cSlipLimTas0 = dev.Bind("3. Lateral-directional", "RollSlackMinTAS", 25f, "Ifrit 0.3.9: below this true airspeed (m/s) the roll is not held back for sideslip or yaw coordination (sideslip guard / governor, yaw-anchored roll limit, roll-acceleration coordination cap); they come back in over RollSlackTASSpan (smoothstep) and are off past 120 deg |AoA|. Higher = more roll slack at low speed, more sideslip.");
			cSlipLimTasSpan = dev.Bind("3. Lateral-directional", "RollSlackTASSpan", 50f, "Ifrit 0.3.9: m/s over which the roll limiters come back in above RollSlackMinTAS (default: full from 75 m/s TAS, 270 km/h). Bench, full stick from near zero airspeed accelerating to 250 km/h: 25/50 holds 21-44 deg/s where 20/30 sat at 11 deg/s; corkscrew from 150 km/h and slow rolls unchanged.");
			cSlipHoldTas0 = dev.Bind("3. Lateral-directional", "SlipHoldMinTAS", 15f, "Ifrit 0.3.7: the sideslip hold is off below this true airspeed (m/s) and fades in over SlipHoldTASSpan; it also fades out from 100 to 120 deg |AoA| (flying backwards). Pedals keep commanding yaw there. In tail slides and near zero airspeed the hold asked for yaw it had no authority for: the jet swayed left/right and the roll stick was scaled to nothing. 0 = 0.3.6 at speed (reversed-flow fade still applies).");
			cSlipHoldTasSpan = dev.Bind("3. Lateral-directional", "SlipHoldTASSpan", 20f, "Ifrit 0.3.7: m/s over which the sideslip hold fades in above SlipHoldMinTAS (default: full from 35 m/s TAS).");
			cDecHldFold = dev.Bind("5. Airframe", "DeceleronHldFold", true, "Ifrit 0.3.6: the stock high-lift device also rotates each deceleron's inner hinge 20 deg TE-down (the hinge carrying the deceleron's lift and both split halves), on top of the FCS deflection and the split: a split half reached 50 deg on the runway with the airbrake out (80 worst case). With this on, that hinge stays put and the decelerons fly the same 20 deg x deploy as droop instead, so everything stays inside the +-30 deg deceleron travel and the split budget. The airbrake has priority over that droop (it retracts while the splits are out). false = 0.3.5.");
			cSlipPredT = dev.Bind("3. Lateral-directional", "SlipPredictTime", 0.3f, "Ifrit 0.3.5: sideslip predictor window (s), the inner roll/yaw rate loops' lag in game (~0.3 s in the 20 Oct logs). The sideslip correction works on the sideslip minus the yaw (stability-axis) rate it has already commanded over this window, which has not reached the sideslip yet. Fixes the 0.7 Hz sideslip / roll oscillation at the high-AoA sideslip gain (20 Oct 22:37 and 23:10 logs). 0 = off (0.3.2). Replaces 0.3.3's SlipRateLead.");
			cSlipPredRollGate = dev.Bind("3. Lateral-directional", "SlipPredictRollGate", 0f, "Ifrit 0.4.0: roll-stick fraction over which the sideslip predictor fades OUT (0.3.5-0.3.9: 0.2, i.e. the predictor was off whenever roll stick was in). 0 = the predictor runs with roll stick in too. 22 Oct 00:30 log, 65 deg AoA corkscrew at 55 m/s with full roll stick held: without the predictor the high-AoA sideslip loop (gain 4) rang against the ~0.34 s inner-loop lag, sideslip +-12 deg and body roll +-50 deg/s on a 1.6 s cycle, every roll surface and the nozzles bang-bang on their stops. A delay-loop model with that lag and 60-100 deg/s^2 of roll authority reproduces the +-10-12 deg cycle without the predictor and settles 12 -> -1.5 -> 0.5 deg in 1.6 s with it.");
			cSplitYawTier = dev.Bind("3. Lateral-directional", "SplitYawTier", true, "Ifrit 0.4.0: least drag first with an engine out or a damaged yaw axis. The canted tails carry the yaw as long as they can; the inner/outer flaperon crossing comes next (the allocator finds it), the split decelerons open only for the yaw still out of reach (a gate on measured yaw saturation that leaks shut once the yaw is met, the split then having priority over its deceleron's deflection for their shared travel), the VMC throttle limit last. Bench, yaw per kN of added drag at 250 km/h and 20 deg AoA: tails +-10 deg 4.8 deg/s^2, crossing 1.1-1.5, one split at 30 deg 0.4 (17.7 kN of drag), the throttle cut 0.17, false = 0.3.9 (tails and splits one tier, the split free: with the right engine alone at 140 m/s the left split sat at 15-24 deg while the tails held +-2 deg; at the 30 deg cap the flaperons crossed with the splits shut and the engine cut to 30-40 %). Both engines running: no change.");
			cStabYawRelax = dev.Bind("3. Lateral-directional", "StabYawRelax", 1f, "Ifrit 0.4.0 (SplitYawTier): with an engine out or a damaged yaw axis, how far (0-1) the roll-fight cost is relaxed for the canted tails, so their differential may buy yaw with the wing taking the roll back. 0 = the 1.8.41 fight cost in full (the allocator then prefers sideslip or the splits to a tail difference).");
			cSlipPredW = dev.Bind("3. Lateral-directional", "SlipPredictWashout", 1.5f, "Ifrit 0.3.5: washout (s) on the predictor's command history, so a held correction (e.g. a yaw trim against a load) leaves no extra steady sideslip. 0 = no washout.");
			cRevS0 = dev.Bind("3. Lateral-directional", "ReversedFlowSenseStart", 120f, "Ifrit 0.3.4 (ReversedFlowRollSense): |AoA| (deg) where the roll stick sense starts to turn over for reversed flow. It blends smoothly to fully flipped at ReversedFlowSenseEnd, passing through zero. 0.3.2 flipped at once at 100 deg (released at 80), next to 90 deg, where the velocity-vector roll is nearly all body yaw, so the flip reversed the rotation mid-manoeuvre.");
			cRevS1 = dev.Bind("3. Lateral-directional", "ReversedFlowSenseEnd", 150f, "Ifrit 0.3.4: |AoA| (deg) where the roll stick sense is fully flipped for reversed flow (see ReversedFlowSenseStart).");
			cSlipGainA0 = dev.Bind("3. Lateral-directional", "SlipGainAoAStart", 5f, "Ifrit 0.3.1: AoA (deg) where the SlipGainHighAoA ramp starts (0.3.0: 25); it reaches SlipGainHighAoA 20 deg higher. Pedals centred only. Bench, corkscrew entry at 475 km/h: from 25 the sideslip built at 15-20 deg AoA and the rolling-pull governor held the AoA there (the 20 deg hitch).");
			cFullStickQ = dev.Bind("2. Envelope", "FullStickPitchRate", 60f, "Ifrit 0.3.1: pitch-rate limit (deg/s) at full aft stick; the limit rises from MaxPitchRate (48) over the last 20 % of aft stick, so up to 0.8 stick and the push side map as before. MPO unchanged. 0 or <= MaxPitchRate = off.");
			cSplitBrakeHold = dev.Bind("7. Airbrake", "SplitBrakeHoldCost", 0.5f, "Ifrit 0.3.0: 0.5 (0.2.2: 0.05). With the airbrake out, the allocator cost of bringing both splits in together (the brake symmetric part). With the 30 deg split and 100 deg/s servos, 0.05 let the allocator trade split for pitch and stall there (bench, 700 km/h at 3 km: splits held at 27, pitch rate drifting to -0.6 deg/s, no retraction on release). Higher keeps the full brake through pulls; one-sided retraction for yaw is free, as on the stock Ifrit.");
			cTvcDiffMu = dev.Bind("4. Gains", "RollTvcCost", 0.003f, "Ifrit 0.3.1: 0.003 (was 5e-5). Allocator cost of differential (roll) nozzle deflection. At 5e-5 the roll TVC was nearly free, so on a pitch release the allocator could run the decelerons/ailerons unevenly and cancel their roll with the nozzles (one nozzle back to neutral before the other, 20 Oct 14:03 log; bench 330-420 km/h: up to 12 deg nozzle split, pitch-rate overshoot -3 deg/s). At 0.003: under 1 deg, overshoot -0.2; corkscrew and rolling-pull sideslip within 0.1-0.3 deg of before.");
			cYawTvc = dev.Bind("3. Lateral-directional", "YawTvc", true, "Revoker 0.1.0: the nozzle's stock yaw axis (+-15 deg, controlInputs.yaw x thrustVectoring.y) is an allocator effector, in the first yaw tier with the rudder. The game zeroes it under 3 m radar altitude and above 250 m/s (with pitch). false = pitch-only TVC (F-22E principle).");
			cTvcYawMu = dev.Bind("4. Gains", "YawTvcCost", 0.0005f, "Revoker 0.1.0: allocator cost of yaw nozzle deflection.");
			cFlapFlaperons = dev.Bind("5. Airframe", "FlapFlaperons", true, "Revoker 0.2.0: the inboard trailing-edge surfaces (flap_L/R; in the game moved only by the high-lift device, 25 deg TE-down below 90-150 m/s) are inboard elevons: allocator effectors for pitch and lateral (the inboard pair, roll group 3, so they can also cross the ailerons for drag yaw as split elevons), driven at SurfaceServoSpeed. The FCS drives the hinge and turns the wing2 section's lift normal by FlapCamberCoupling x the deflection. No flap schedule (FlapHighLiftDroop 0). false = HLD flaps as stock.");
			cFlapHinge = dev.Bind("5. Airframe", "FlapHingeRange", 25f, "Revoker 0.2.0: travel each way of the inboard elevons (deg). The stock HLD goes 25 TE-down.");
			cFlapHiLift = dev.Bind("5. Airframe", "FlapHighLiftDroop", 0f, "Revoker 0.2.0: default 0. TE-down droop (deg) the inboard elevons are drawn toward at full HLD deployment (the game's low-speed flap schedule: out below 90 m/s, in above 150). A canard-delta's trailing edges are elevons with no flap schedule (unlike the F-22E / Ifrit flaperons), so 0 = no droop: the allocator alone sets them.");
			cCanardPitch = dev.Bind("4. Gains", "CanardPitchPriority", true, "Revoker 0.1.0: during rolls the roll-time nozzle budget (RollTvcBudget) keeps the nozzle within its band of NEUTRAL (F-22E: of the pitch-only trim) at every thrust (F-22E: open from T/W 0.7 to 1.1), so the canards cancel the roll's nose-up / nose-down and carry the pitch trim; the nozzle still goes past the band where the canards cannot (their stall band, AoA protection, AoA/G capture braking).");
			cFlapRelief = dev.Bind("5. Airframe", "FlaperonLoadRelief", false, "Revoker 0.1.0: off (F-22E/Ifrit: on). The flap schedule's high-g term that turns the trailing-edge flaperons up to 9.6 deg TE-up from 3 to 9 g below 16-24 deg AoA. Off, the trailing edges stay at neutral in a held turn and the canards carry the pitch.");
			cYawTvcNeutral = dev.Bind("4. Gains", "YawTvcNeutralHold", 300f, "Revoker 0.1.0: while the lateral stick and pedals are steady, the roll on its command and the sideslip under 2 deg (a held turn), the nozzle's yaw axis is drawn back to neutral over ~1 s and the rudder and canards carry the steady yaw; the nozzle still takes every change. Cost weight; 0 = off. The pitch axis has the same (TvcNeutralHold, canards carrying the steady pitch).");
			cVisHinge = dev.Bind("5. Airframe", "VisualHingeFix", true, "Revoker 0.2.3: draw the trailing-edge surfaces turning about their modelled hinge lines (render only; aero unchanged). Off: the game's own visuals.");
			cCorkTol = dev.Bind("3. Lateral-directional", "CorkscrewSlipTol", 0f, "Revoker 0.2.2: with roll stick held above ~35-50 deg AoA (pedals centred), sideslip (deg) the sideslip hold lets build before it takes the yaw back; the corkscrew keeps rotating. 0 = 0.2.1.");
			cHiAoARollW = dev.Bind("3. Lateral-directional", "HighAoARollWeight", 1f, "Revoker 0.2.7: 1 (F-22E/Ifrit 0.35). The allocator's roll-axis weight from 40 deg AoA (blended in from 20) while the nozzles do not cover pitch (TVC off or the low-thrust T/W); with them covering it stays 0.35. With the roll target out of reach the pitch (weight 8) and yaw (YawPriority) axes took everything at 0.35 and the roll went the wrong way for moments, which with the game's surface lag set up a ~0.6 Hz cycle of every surface stop to stop (28 Oct 20:31 log, TVC off at the 50 deg cap). Bench, TVC off, 50 deg loaded rolls with a game-like 4-frame command delay: surface slew peaks 35-70 -> 10-45 deg/s, sideslip 6.2-6.9 -> 2.9-4.6 deg, roll 24-27 -> 29-33 deg/s. Takes effect at once.");
			cCanWin = dev.Bind("3. Lateral-directional", "CanardAttachedWindow", true, "Revoker 0.2.7: above 30 deg AoA either sign, with lateral input in, the canards are walked into the deflections where their own flow still has lift slope (|local AoA| <= the foil's knee + CanardWindowMargin), when their travel reaches them. Past the knee the allocator sees no roll / yaw gradient on a canard and parked them stalled (28 Oct 21:18 log, full push + roll at -50 deg: both canards at -1 deg, local AoA -48..-50, for 20 s). false = 0.2.6. Takes effect at once.");
			cCanWinMargin = dev.Bind("3. Lateral-directional", "CanardWindowMargin", 12f, "Revoker 0.2.7: local AoA (deg) past the canard foil's lift-slope knee (15.8 deg) that CanardAttachedWindow allows. Bench: 6 cost up to 5 deg of AoA in 50 deg roll reversals, 12 held it within 2-4.");
			cFinTvcHold = dev.Bind("3. Lateral-directional", "FinLossTvcHold", true, "Revoker 0.2.7: with the fin gone (yawHealth under 0.5) the engines keep vectoring above their 250 m/s cut-off (raised to 1000 m/s on this jet, put back when the fin is whole or the FCS lets go). The yaw nozzle is the only yaw axis then; at the cut-off the game zeroes the nozzles and the jet yawed round (28 Oct 21:22 log, fin + wings gone, 247 m/s). false = the game's cut-off always. Takes effect at once.");
			cFinNoTvcCap = dev.Bind("2. Envelope", "FinLossNoTvcAoACap", 30f, "Revoker 0.2.7: positive AoA limit (deg) with the fin gone and no nozzle yaw (TVC off, or the thrust under the low-thrust T/W), phased in from no fin damage to the fin gone. Bench, fin gone, TVC off, held pull + roll: 45-50 departs, 40 departs with a game-like command delay, 35 and below hold. 0 = off. Takes effect at once.");
			cSlipYawTvc = dev.Bind("3. Lateral-directional", "SlipYawTvcAssist", true, "Revoker 0.2.1: with the yaw nozzle active, the sideslip gain ramps to SlipGainHighAoA at high AoA (Ifrit: only with roll TVC, which the Revoker has none of).");
			cGroundElevonPitch = dev.Bind("5. Airframe", "GroundElevonPitch", 1f, "Revoker 0.2.1: on the ground, the share of each trailing-edge elevon's travel the pitch stick moves (0-1; full aft = TE up). In the air the allocator uses them for pitch (ElevonPitch).");
			cCanardYawTier = dev.Bind("3. Lateral-directional", "CanardYawTier", false, "Revoker 0.1.0, default off: with the fin or rudder damaged, count the canards' yaw (differential drag) in the first yaw tier with the rudder and the yaw TVC. Off: the canards are still full pitch / roll / yaw effectors in the allocator, but their yaw is not counted as free reach (one canard at 40 deg makes ~45 deg/s^2 of yaw at a pitch and roll cost the allocator will not pay, so counting it kept the split-flaperon tier shut: bench, fins gone at 700 km/h, a pedal kick departed with nothing moving for 1.3 s).");
			cElevonPitch = dev.Bind("5. Airframe", "ElevonPitch", true, "Revoker 0.2.0: default on. The ailerons (outboard elevons) are pitch effectors as well as roll, with their true pitch moment in the allocator; the inboard elevons (FlapFlaperons) always are. false = ailerons for roll/yaw only, pitch from canards + TVC + inboard elevons.");
			cElevonSymCost = dev.Bind("4. Gains", "ElevonSymCost", 2f, "Revoker 0.1.0: allocator cost of the symmetric (pitch) part of the flaperon pair (F-22E/Ifrit: 2).");
			cAoANegRev = dev.Bind("2. Envelope", "AoALimitNegativeRevision", 1, "Internal: the AoALimitNegative default revision this file was last updated to. Leave it alone.");
			if (cAoANegRev.Value < 1)
			{
				// 2.2.4: default 45 -> 50. Runs once: a file still holding the old default 45 is moved; any other value is the
				// player's own and is kept. Afterwards the file is at revision 1 and 45 set later stays.
				if (Mathf.Abs(Mathf.Abs(cAoANeg.Value) - 45f) < 0.001f)
				{
					cAoANeg.Value = 50f;
					Log.LogInfo("Revoker FCS 2.2.4: AoALimitNegative moved from the old default 45 to the new 50 (once).");
				}
				cAoANegRev.Value = 1;
				dev.Save();
			}
			cGPos = dev.Bind("2. Envelope", "GLimitPositive", 9.5f, "Positive load limit (g), normal law.");
			cGNeg = dev.Bind("2. Envelope", "GLimitNegative", 3.5f, "Negative load limit magnitude (g), normal law.");
			cMpoGPos = dev.Bind("2. Envelope", "MPOGLimitPositive", 12.5f, "Positive load limit (g) with Stability Assist OFF (MPO).");
			cMpoGNeg = dev.Bind("2. Envelope", "MPOGLimitNegative", 3.5f, "Negative load limit magnitude (g) in MPO.");
			cMaxPitch = dev.Bind("2. Envelope", "MaxPitchRate", 48f, "Pitch-rate budget ceiling (deg/s).");
			cProt = dev.Bind("2. Envelope", "AoARecoveryRate", 48f, "AoA-protection recovery rate (deg/s), independent of thrust and weight.");
			cMaxRoll = dev.Bind("3. Lateral-directional", "MaxRollRate", 250f, "Absolute body roll-rate ceiling (deg/s). Below ~250 m/s IAS the original-aircraft schedule and RollAuthorityUse set the rate; above it this ceiling holds the rate flat.");
			cMaxYaw = dev.Bind("3. Lateral-directional", "MaxYawRate", 90f, "Body yaw-rate limit (deg/s). Revoker 0.2.2: 90 (was 75); bench, 70 deg loaded rolls 68-70 -> 75-80 deg/s velocity-vector roll at the same sideslip, corkscrews body yaw 64 -> 70 deg/s.");
			cRollLo = dev.Bind("3. Lateral-directional", "RollRateGainLowIAS", 1.2f, "Roll-rate ceiling vs the ORIGINAL aircraft's roll rate below ~120 m/s IAS (blends to the high-IAS gain by 170 m/s).");
			cRollHi = dev.Bind("3. Lateral-directional", "RollRateGainHighIAS", 1.1f, "Roll-rate ceiling vs the original aircraft's roll rate above ~170 m/s IAS.");
			cRollBrake = dev.Bind("3. Lateral-directional", "RollBrakeUse", 0.8f, "Fraction of the full roll authority (wings and stabilators) used to stop a roll, on top of roll damping. Before 1.8.26 this was a fraction of the wing-only authority.");
			cRollAuth = dev.Bind("3. Lateral-directional", "RollAuthorityUse", 1f, "Steady roll never needs more than this fraction of the available roll control moment (roll rate is given up instead of saturating the surfaces).");
			cCouple = dev.Bind("3. Lateral-directional", "CoupledAxisAoA", 36f, "Above this |AoA| the pedals blend into the stick's velocity-vector roll (full coupling 14 deg higher).");
			cPedalBeta = dev.Bind("3. Lateral-directional", "PedalSideslip", 15f, "Sideslip commanded by full pedal below the coupling AoA (deg).");
			cRollAcc = dev.Bind("3. Lateral-directional", "MaxRollAccel", 720f, "Ceiling on the shaped velocity-vector roll acceleration (deg/s^2); below it the measured authority sets the rate.");
			cBetaGuard = dev.Bind("3. Lateral-directional", "SideslipGuardStart", 4f, "Sideslip error (deg) above which the velocity-vector roll/yaw command is backed off, fading to zero command 10 deg above it. Raise it for manoeuvres that live at large sideslip on purpose (falling leaf); lower it for tighter coordination.");
			cYawSatGuard = dev.Bind("3. Lateral-directional", "YawSaturationGuard", 0.7f, "How hard a yaw-saturated allocation backs the roll/yaw command off (0..1, fraction removed at full saturation). Lower = the command stays up when the surfaces cannot fully deliver it (more sideslip, more yaw rate); 0 = off.");
			cKBeta = dev.Bind("4. Gains", "SideslipGain", 1.3f, "Stability-axis yaw rate per degree of sideslip error (1/s).");
			cKRoll = dev.Bind("4. Gains", "RollRateGain", 6f, "Roll-rate loop bandwidth (1/s).");
			cKPitch = dev.Bind("4. Gains", "PitchRateGain", 5f, "Pitch-rate loop bandwidth (1/s).");
			cKYaw = dev.Bind("4. Gains", "YawRateGain", 2.5f, "Yaw-rate loop bandwidth (1/s).");
			cBrakeBlend = dev.Bind("4. Gains", "AoACaptureBrakeBlend", 0.3f, "How far the capture braking plan leans from the authority at the target AoA (0) toward what is available right now (1). Lower = earlier braking, less overshoot, slower AoA build; higher = faster AoA build, more overshoot below afterburner. 0.3 is the bench balance.");
			cThrBrake = dev.Bind("4. Gains", "AoACaptureThrustBrake", defaultValue: true, "AoA capture plans its braking with the nose-down authority it can count on at the TARGET AoA: the nozzles' full authority at the live thrust (so less below afterburner and much less at part throttle) plus the stabs' authority at the target AoA from the surface model (almost none past ~45 deg, where they stall). The approach follows sqrt(2 x 0.7 x that x distance), the same deceleration the pitch trajectory can deliver. false = pre-1.8.20.");
			cAntiWind = dev.Bind("4. Gains", "AoAGAntiWindup", 4f, "Back-calculation anti-windup for the AoA and G integrators (1/s): whenever a limiter (AoA/G capture braking, AoA limit, pitch-rate budget) clips the pitch-rate command below what the AoA/G loop asks for, the integrator is pulled back toward the command at this rate. Without it the integrator winds up during every capture and then holds AoA about 0.2 deg above the command until it unwinds (several seconds). 0 = pre-1.8.20.");
			cLatRes = dev.Bind("4. Gains", "LateralStabReserve", 0.7f, "While you hold pitch stick AND lateral stick (roll or pedal) together, the pitch law builds pitch rate with the nozzles' authority plus only (1 - this x |lateral stick|) of the stabs' remaining authority, and the allocator's cost for symmetric (pitch) stab deflection rises with it, so the nozzles take the pitch and the stabs stay for roll/yaw. Braking toward the commanded AoA/G, the AoA/G limits and protection always use full authority. No lateral stick, or pitch stick released = unchanged. 0 = pre-1.8.19.");
			cPairTrim = dev.Bind("4. Gains", "StabTvcPairedSearch", defaultValue: true, "The allocator also tries moving each stabilator with the nozzles taking up its pitch change, and keeps the pair move when it lowers the total error. Without it, a stab parked on a stalled, flat part of its curve for pitch cannot be moved to where it makes roll, because moving it alone breaks pitch and moving the nozzles alone breaks it the other way. Lets the nozzles carry the pitch trim whenever they have room, freeing the stabs for roll/yaw. false = pre-1.8.18.");
			cBranch = dev.Bind("4. Gains", "StabBranchSwitch", defaultValue: true, "When a stabilator has been driven deep past its stall (local AoA more than 10 deg past the peak) during roll/yaw at moderate AoA, move it to the attached-flow side of its lift peak instead (trailing edge up, where the lift curve is steep), hold it there, and re-allocate everything else around it, the nozzles taking up the pitch. Undone when the modelled pitch error would grow by more than StabBranchPitchTol. false = pre-1.8.21.");
			cOvPrio = dev.Bind("4. Gains", "OvershootPitchPriority", defaultValue: true, "While the nose is pitching up faster than the pitch law commands above ~15-25 deg AoA (an AoA run-away), or AoA protection is active: (1) the allocator keeps pitch priority instead of handing stalled stabs to roll/yaw, and (2) it models the nose-down a stab really loses when it is driven deep past its stall for roll/yaw (the monotone stab pitch model hides that loss). false = pre-1.8.23.");
			cLeadK = dev.Bind("4. Gains", "StickLeadGain", 0f, "2.1.7: G + pitch-rate law only. Extra pitch-rate command on stick movement, washed out over StickLeadTau: a quick small input moves the nose a little at once, like an AoA step, then the plain rate response takes over; a held stick gets none. 0 = off (default). Around 1 = noticeable crispness; much above 1.5 small inputs can feel twitchy. Takes effect at once.");
			cLeadTau = dev.Bind("4. Gains", "StickLeadTau", 0.15f, "2.1.7: wash-out time (s) of StickLeadGain. Longer = the extra nose movement lasts longer and is larger for the same gain.");
			cGovPred = dev.Bind("3. Lateral-directional", "PredictiveRollGovernor", true, "2.1.5: the roll governor (1.8.41, keeps nozzle margin for pitch during rolls) looks at where the nozzle margin is heading 0.4 s ahead and eases the roll off early and in proportion, instead of cutting it once the nozzles are already pinned and restoring it a second later (the roll hitch). It comes back only when the margin is there and no longer falling. false = 2.1.4. Takes effect at once.");
			cSweepYaw = dev.Bind("3. Lateral-directional", "AoASweepYawLimit", true, "2.1.8: while the jet carries a velocity-vector roll, the AoA may only sweep as fast as the yaw and roll axes can re-point the rate vector to follow it (a roll at AoA a needs a yaw rate of p sin a; sweeping a under that roll turns the yaw rate already there into sideslip). Mainly a full push right after a high-AoA roll at low IAS. No effect with no roll, and none beyond the AoA limits (protection keeps its own rate). false = off (no AoA-sweep limit). Takes effect at once.");
			cSweepPos = dev.Bind("3. Lateral-directional", "AoASweepYawLimitPositive", true, "2.2.7: positive-AoA pulls while rolling: the AoA rise is held to what the yaw can follow, and tapered to a hold as the sideslip (and where it is heading) approaches PullSideslipMargin, on the side the AoA rise drives it. Acts from ~10 deg/s of roll (full by 20). Pulls without roll and steady-AoA rolls are unaffected. Needs AoASweepYawLimit on. false = no positive-AoA limit (2.2.3). Takes effect at once.");
			cPullSlipMargin = dev.Bind("3. Lateral-directional", "PullSideslipMargin", 15f, "Ifrit 0.3.1: default 15 (0.3.0: 9). Bench, corkscrew entry at 475 km/h: at 9 the governor held the AoA near 20 deg while the roll settled (the hitch). 2.2.7 (2.2.8: uses the full margin; 2.2.23: default 6 -> 9): sideslip (deg) the AoA-rise governor in a rolling pull runs up to. The AoA rises as fast as the yaw can follow, tapers from 70 % of the margin (2.2.7: 35 %) and holds at it; past it the AoA comes back down. Judged on the sideslip and where it is heading 0.3 s ahead. Allowed 2-15: lower = tighter sideslip, slower AoA build-up while rolling. Takes effect at once.");
			cPullRollTrade = dev.Bind("3. Lateral-directional", "PullRollTrade", 0.7f, "2.2.8 (2.2.23: default 0 -> 0.7; 2.2.26: 1; 2.2.27: back to 0.7): in a rolling pull held back by the sideslip governor, how much of the AoA rise you ask for is bought back by slowing the roll (the sideslip a rising AoA makes goes as roll rate x AoA rate). 0 = roll kept, AoA rise limited (2.2.7). 1 = roll slowed (to no less than 30 deg/s) until the asked AoA rise fits the sideslip margin; in between, the roll limit goes that share of the way. The roll comes back once the AoA rise is no longer limited. 2.2.11: above ~180-230 kt (2.2.12; 2.2.11: 230-280) the roll limit is also set ahead of time from the AoA rate the stick asks for, so the roll does not build past what the yaw can coordinate before the governor reacts. Takes effect at once.");
			cPullFFBoostMax = dev.Bind("3. Lateral-directional", "PullFFBoostMax", 4f, "2.2.24: in a rolling pull, the governor's yaw-authority feed-forward (how fast the AoA may rise for the roll in force) is scaled up to this many times while the sideslip stays under half of PullSideslipMargin and the stick asks for more, and scaled back as the sideslip passes 70 % of it; the sideslip taper still holds the margin. Off with a fin lost. 1 = off (2.2.23: the feed-forward alone held the AoA rise to ~10 deg/s at 100-125 kt with 1-4 deg of a 9 deg margin). Allowed 1-8. Takes effect at once.");
			cPullGovSmooth = dev.Bind("3. Lateral-directional", "PullGovSmooth", true, "2.2.9: rolling-pull governor without the yaw-lead release, and its pitch-rate cap comes back over ~0.3 s instead of at once. Removes most of the pitch bounce when pulling with lateral stick, but full aft + full roll together from wings level builds AoA more slowly (bench 450 km/h: 39 -> 21 deg at 4 s); PullRollTrade buys that back. false = 2.2.8 (faster AoA in pull + roll together, bouncier). Takes effect at once.");
			cSplitFlapYaw = dev.Bind("3. Lateral-directional", "SplitFlaperonYaw", true, "2.2.7: decelerons. When the yaw needed is past what the rudders can give (both fins gone, one fin gone, or one engine out with the other's thrust more than the rudders hold), the inner and outer flaperons on one wing open against each other (their rolls cancel, their drag yaws the jet), up to SplitFlaperonYawUse of the drag yaw they have. Only the part the rudders cannot cover; the engine-out VMC throttle limit acts after it. false = 2.2.6 (yaw target limited to the rudders). Takes effect at once.");
			cSplitFlapYawUse = dev.Bind("3. Lateral-directional", "SplitFlaperonYawUse", 1f, "2.2.19: default 1 (was 0.7). Bench, both fins gone, full roll held at 25-40 deg AoA: worst sideslip mean 12.4 -> 10.8 deg, roll tracking unchanged. 2.2.7: share (0-1) of the flaperons' remaining drag yaw the deceleron split may ask for. Higher = more yaw, more roll and pitch disturbance from the split. Takes effect at once.");
			cSplitFlapRollYield = dev.Bind("3. Lateral-directional", "SplitFlaperonRollYield", 0f, "2.2.8: how much of the deceleron split gives way to lateral stick (the split's reach shrinks by this x |lateral stick|, held 1 s after the stick returns). 0 = split kept through rolls (as 2.2.7; least sideslip). 1 = no split while rolling (roll tracks more evenly, but with both fins gone the sideslip in rolls runs much higher, e.g. 15 -> 27 deg replaying your 19 Oct 338-362 s inputs on the bench). Takes effect at once.");
			cRollPitchLead = dev.Bind("4. Gains", "RollPitchLead", 0.25f, "2.2.5: pull + lateral stick (the stabs handed to the roll, the nozzles carrying the pitch). The pitch-rate loop reads the measured pitch rate this many seconds ahead along the measured pitch acceleration, scaled by the nozzles' share of the pitch authority and by how much of the stab reserve the lateral stick has taken, so the slow nozzles stop driving stop to stop and the pitch rate stops swinging about its command (the hitch with an AoA overshoot in a full pull with roll added mid-pull, worse at high IAS). Pulls only. 0 = off (2.2.3). Bench: 0.15 / 0.25 / 0.35 / 0.5 all helped; 0.25 had the least AoA overshoot. Takes effect at once.");
			cSlatRelief = dev.Bind("3. Lateral-directional", "SlatRollRelief", false, "2.1.9 (off by default, experimental): during a roll at 12-45 deg AoA, when the nozzles' nose-down margin is heading for the roll governor's threshold, the slats retract (they are wing area ahead of the CG, so out they push the nose up) to give the pitch axis nose-down moment before the governor has to slow the roll. Costs ~7 % of the lift while they are in (bench: 0.05-0.4 g less in the roll); they come back out when the margin returns or the roll ends. Bench gain is modest: peak roll rate +4..+28 deg/s in 20-30 deg AoA rolls with thrust, no change to the first second of the roll. false = 2.1.8. Takes effect at once.");
			cCoupGateLim = dev.Bind("3. Lateral-directional", "CouplingLimitAtAoALimit", true, "2.1.10: rolls at an AoA limit. (1) While the stick holds the jet within 4 deg of an AoA limit, the roll-rate limit that keeps the roll's inertial pitch-up within the nose-down authority stays on and eases off over 0.6 s; it used to follow the pitch error on and off every ~0.6 s, so the roll rate and the pitch rate bounced. (2) While a velocity-vector roll runs, the AoA-limit capture plans its braking with what is left for pitch less the roll's inertial pitch-up, so it no longer overshoots the limit, hits protection and drops back several degrees in a cycle. In fast rolls the AoA can sit 1-3 deg under the limit. false = 2.1.9. Takes effect at once.");
			cLoadedRollSteady = dev.Bind("3. Lateral-directional", "LoadedRollSteady", true, "2.2.3: on by default (2.2.2: off). full aft + roll. The roll-rate limit that keeps the roll's inertial pitch-up within the nose-down authority stays on for the whole loaded roll (not only within 4 deg of the AoA limit), and the roll governor reads the nozzles' margin as it is instead of projecting it 0.3 s ahead. Both switched on and off every ~0.45 s in the 13 Oct 11:08 log (690 km/h pull + roll: roll target 98 <-> 172 deg/s, nozzles stop to stop, AoA 41..48). Bench: the pitch-rate and nozzle oscillation in the roll about halve; bank after 4 s 0-11 % lower. Only with CouplingLimitAtAoALimit on. Takes effect at once.");
			cSlipYawW = dev.Bind("3. Lateral-directional", "HighAoASideslipAllocation", true, "2.1.10: above ~20 deg AoA, while sideslip is building (from 2 deg, full by 8, counting where it is heading 0.3 s ahead), the control allocator stops trading roll accuracy for yaw (yaw weight 8 -> 2 at 40+ deg AoA): at high AoA a body roll that runs past its command makes sideslip as fast as a yaw-rate error does. No change while the sideslip is small. false = 2.1.9. Takes effect at once.");
			cCapLag = dev.Bind("2. Envelope", "AoACaptureEffectorLag", true, "2.2.1: the AoA-limit capture plans its stop with the pitch effectors' lag (0.3 s x the nozzles' share of the nose-down authority - they slew ~70 deg/s and carry most of it at low IAS), not as if the jet could start braking at once. A re-pull from ~38 deg at ~200 km/h overshot a 45 deg limit by 1.5 deg (3.5 with a roll entry). Where the nozzles carry the authority the AoA can stop 1-2 deg short of the limit and creep the rest. false = 2.2.0. Takes effect at once.");
			cAsymShared = dev.Bind("Asymmetric load", "AsymSharedStabLateral", true, "2.1.4: with an asymmetric load, past the rudder fade the roll and yaw rate limits also give up the stab deflection that trims the load's roll (the stabs are both the roll and the yaw source there), not only the yaw part of the load; past the rudder fade the yaw anchor and sideslip lead also act at positive AoA; and (2.1.6) a roll brake or reversal accelerates no faster than the yaw left after the load can follow. Only the load beyond the deadbands / a lateral CG offset over 4 cm counts; a clean jet is unchanged. false = 2.1.3. Takes effect at once.");
			cLowCapRoll = dev.Bind("3. Lateral-directional", "LowThrustCapRollFix", true, "2.1.3: rolls at the low-thrust AoA cap (36 deg at idle / low T-W). Within 4 deg of the cap the roll-coupling limit stays on instead of waiting for the AoA to rise, and the roll governor judges the nozzle margin only by the nozzles' share of the nose-down authority (pinned weak nozzles no longer read as no pitch reserve). Aimed at the hitches where the roll was cut and restored as the AoA overshot the cap. Only acts at the low-thrust cap. false = 2.1.2. Takes effect at once.");
			cCoupRoll = dev.Bind("4. Gains", "CouplingRollLimit", 0.5f, "While AoA is building past 20-30 deg (or running away), cap the velocity-vector roll rate so the inertial pitch-up it causes, (Izz-Ixx)/Iyy * p * r, stays within this fraction of the nose-down authority the jet has at that AoA (aero + stabs + nozzles at live thrust). This is what let full-aft + full-roll pulls overshoot to 70-77 deg below afterburner. Held-AoA rolls are not affected. 0 = off (pre-1.8.23).");
			cBranchMaxAoA = dev.Bind("4. Gains", "StabBranchMaxAoA", 42f, "Body AoA (deg) above which StabBranchSwitch is not used. Above ~44 deg the stalled side of the stab is the better one for roll/yaw.");
			cBranchTarget = dev.Bind("4. Gains", "StabBranchTarget", 20f, "Local AoA (deg) StabBranchSwitch moves the stab to, on the attached side of its lift peak.");
			cBranchTol = dev.Bind("4. Gains", "StabBranchPitchTol", 5f, "Largest growth in modelled pitch-acceleration error (deg/s^2) StabBranchSwitch accepts. Protects AoA holding when the nozzles cannot take up the stab's pitch.");
			cKpAoA = dev.Bind("4. Gains", "AoAGain", 5f, "AoA command: deg/s of pitch rate per deg of AoA error.");
			cKiAoA = dev.Bind("4. Gains", "AoAIntegral", 3f, "AoA command integral gain (1/s) near the target.");
			cStick24 = cfg.Bind("5. Pitch stick curve", "StickFor24degAoA", 0.6f, "2.0.12 (default 0.6 since 2.0.15; 0.45 in 2.0.13-2.0.14): pull-stick position (0..1) that commands 24 deg AoA. 0.333 = the standard 72 deg per full stick line. Allowed 0.083-0.7: lower = more AoA early in the pull (sharper), higher = less (softer, finer control at low AoA). The curve through 0, this point and the 36 deg point is smooth (monotone cubic), so the gain changes gradually instead of at a corner. Takes effect at once.");
			cStick36 = cfg.Bind("5. Pitch stick curve", "StickFor36degAoA", 0.8f, "2.0.12 (default 0.8 since 2.0.15; 0.65 in 2.0.13-2.0.14): pull-stick position (0..1) that commands 36 deg AoA (on the standard line: 0.5). Allowed 0.3-0.8, and always at least 0.1 of stick past StickFor24degAoA (if set closer it is moved out to 24's point + 0.1, so 36 can never come at or before 24). Past 36 the curve continues to the AoA limit at full stick as before (the high-AoA rate hand-over is unchanged). Applies to the pull side; the push side keeps the 72 deg per stick line. Takes effect at once.");
			cStickRev = cfg.Bind("5. Pitch stick curve", "StickCurveRevision", 2, "Internal: the stick-curve default revision this file was last updated to. Leave it alone.");
			if (cStickRev.Value < 1)
			{
				// 2.0.13: new defaults 0.45 / 0.65; a file still holding 2.0.12's 0.333 / 0.5 is moved once
				if (Mathf.Abs(cStick24.Value - 0.3333f) < 0.001f)
				{
					cStick24.Value = 0.45f;
				}
				if (Mathf.Abs(cStick36.Value - 0.5f) < 0.001f)
				{
					cStick36.Value = 0.65f;
				}
				cStickRev.Value = 1;
				cfg.Save();
			}
			if (cStickRev.Value < 2)
			{
				// 2.0.15: new defaults 0.6 / 0.8. Runs once, on the first start of 2.0.15 or later: a file still holding
				// the 2.0.13-2.0.14 defaults (both 0.45 and 0.65) is moved; any other pair is the player's own and is kept.
				// From then on the file is at revision 2 and is never touched again, so 0.45 / 0.65 set later stays.
				if (Mathf.Abs(cStick24.Value - 0.45f) < 0.001f && Mathf.Abs(cStick36.Value - 0.65f) < 0.001f)
				{
					cStick24.Value = 0.6f;
					cStick36.Value = 0.8f;
					Log.LogInfo("Revoker FCS 2.0.15: pitch stick curve moved from the old defaults 0.45 / 0.65 to the new 0.6 / 0.8 (once).");
				}
				cStickRev.Value = 2;
				cfg.Save();
			}
			cStickSmooth = cfg.Bind("1. General", "StickSmoothing", 0.06f, "Time constant (s) of the first-order smoothing on pitch/roll/pedal inputs. 0 = off.");
			cActDamp = dev.Bind("5. Airframe", "ActuatorDamping", 0.06f, "Actuator smoothing (s): time for a servo command to accelerate to full servo speed (and brake into its target on a matching curve). Servo speed is unchanged and a steady slew runs at full speed. 0 = off.");
			cYawStop = dev.Bind("3. Lateral-directional", "YawStopFactor", 0.7f, "Above 15-25 deg AoA the body yaw rate is capped at this x measured yaw authority (so the yaw can be stopped again). Higher = more responsive moderate-AoA rolls, more sideslip on stops.");
			cRollLead = dev.Bind("3. Lateral-directional", "RollLagCompensation", 0.008f, "Roll command lead per unit of roll damping (s per 1/s above 2.5/s): cancels the roll loop's tracking lag at high dynamic pressure so roll stops end when the command does, without a slow tail. 0 = off.");
			cYawPri = dev.Bind("4. Gains", "YawAxisPriority", 8f, "Allocator weight of the yaw axis relative to roll below ~20 deg AoA. Higher = rudders keep the roll coordinated even when their roll side-effect costs some roll acceleration.");
			cYawFF = dev.Bind("4. Gains", "YawRateFeedForward", 0.5f, "Fraction of the yaw-rate command's rate of change fed forward to the yaw loop at low AoA below 200 m/s IAS (blends to 1.0 by 300 m/s IAS, and by 28 deg AoA, where the body yaw is the velocity-vector roll). Lower = calmer rudders at low speed, slightly more sideslip.");
			cLowYaw = dev.Bind("3. Lateral-directional", "LowIasYawBoost", 1.7f, "Above 15-25 deg AoA, widens the yaw-rate cap (YawStopFactor x yaw authority, and its 20 deg/s floor) by this factor at very low IAS (falling leaf). It fades out along a curve above LowIasYawBoostFullIAS (see LowIasYawBoostFalloff). 1 = off.");
			cLowYawIas = dev.Bind("3. Lateral-directional", "LowIasYawBoostFullIAS", 70f, "IAS (m/s) up to which the full LowIasYawBoost applies.");
			cLowYawFall = dev.Bind("3. Lateral-directional", "LowIasYawBoostFalloff", 20f, "Above LowIasYawBoostFullIAS the extra margin decays exponentially with this IAS scale (m/s): 10 = ~50 % of it left at +7 m/s, ~10 % at +23 m/s, gone by 120 m/s.");
			cTvcMove = dev.Bind("4. Gains", "TvcMoveCost", 2E-05f, "Allocator cost of moving the TVC nozzles. 2e-5 = nozzles take pitch changes first; ~0.02 = stabs do the quick pitch changes and the nozzles slowly take over the sustained (trim) share. Post-stall the nozzles are always fast.");
			cTvcNeutral = dev.Bind("4. Gains", "TvcNeutralHold", 1000f, "2.2.14: while the pitch rate is held steady (no change of pitch-rate command, no pitch error, no roll in progress) the nozzles are drawn back to neutral over ~1 s and the stabs carry the steady pitch moment; the nozzles still take every change. Cost weight: 1000 (default); 0 = off (2.2.13). Takes effect at once.");
			cStabPark = dev.Bind("4. Gains", "StabParkEscape", true, "0.4.3 (the F-22E's 2.3.0/2.3.1 settled stall window): with the lateral stick and pedals centred and the lateral rates settled (under SettledRateOn deg/s, off again above SettledRateOff or with lateral input), the stabilators and rudders may not rest past the smaller of (lift peak - SettledStallMargin) and the foil's lift-slope knee (SettledSlopeFrac) of local AoA, at any AoA where such a range exists. This jet's stabilator foil has no stall before 45 deg (its lift keeps rising from 20 to 45 deg at a fifth of the linear slope, the drag four times), so a stab parked out there had next to no gradient for the allocator and made its maximum lift: 23 Oct 00:51 log, left stab on its stop for 10 s after a push through zero AoA at 58 m/s, the nozzle and the crossed wing against it. 0.4.2 and earlier only acted below 30 deg AoA and allowed the peak itself. false = off. Takes effect at once.");
			cSettledMargin = dev.Bind("4. Gains", "SettledStallMargin", 6f, "0.4.3: how far below the lift peak (deg of local AoA) the settled stall window keeps the stabilators and rudders (see StabParkEscape); the foil's slope knee applies where it is lower. Larger = further onto the attached slope, less trim deflection available from them. Takes effect at once.");
			cSettledSlope = dev.Bind("4. Gains", "SettledSlopeFrac", 0.15f, "0.4.3: the settled stall window's limit is also where the foil's lift slope has fallen to this fraction of its steepest (the knee of a plateau foil: 26 deg on the stabilator foil at 0.15, where the slope is 0.66 /rad against 4.4 at zero; 0.25 = 17 deg, 0.1 = 31 deg). 0 = the peak - SettledStallMargin only. Takes effect at once.");
			cSettledOn = dev.Bind("4. Gains", "SettledRateOn", 10f, "0.4.3: the settled stall window (StabParkEscape) comes on when the body roll and yaw rates are both under this (deg/s). Takes effect at once.");
			cSettledYawUse = dev.Bind("4. Gains", "SettledYawUse", 0.4f, "0.4.3: the settled stall window is held off while the thrust yaw (an engine out or down on thrust) uses more than this fraction of the fins' yaw authority at the present AoA (|asymAy| / asymCy, filtered 0.5 s, back on under this - 0.1): this jet has no rudders, and with an engine out a stabilator run out past its lift knee is its drag rudder (bench, engine-out pull at 250 km/h: windowed, the yaw saturated and the VMC cut the throttle). The 00:51 log's parked set had 0.29; the engine-out trims run 0.5-1.3. Takes effect at once.");
			cSettledOff = dev.Bind("4. Gains", "SettledRateOff", 20f, "0.4.3: the settled stall window goes off again above this body roll / yaw rate (deg/s), or with lateral input. Takes effect at once.");
			cSettledWalk = dev.Bind("4. Gains", "SettledWalkRate", 30f, "0.4.3: deg/s at which the settled window walks a surface resting past it back in (the window's edge moves, the allocator re-solves around each step). Takes effect at once.");
			cRollFightRel = dev.Bind("4. Gains", "RollFightReleased", true, "2.2.16: the roll-fight cost (RollFightCost) also acts while the lateral stick is centred, not only with the pedals in. With the stick released, a crossed set of surfaces - one stab split against the flaperons, ailerons and rudders, all near their stops, their roll cancelling - could hold for 10-15 s (19 Oct 18:05 / 18:32 logs, TVC off). false = 2.2.15. Takes effect at once.");
			cGndBrakeSplit = dev.Bind("7. Airbrake", "AirbrakeSplitInFlight", defaultValue: true, "In the AIR, the brake control deploys the airframe's split surfaces (the stock speedbrake mechanism). On the ground they stay stowed and the airbrake is the rollout surface configuration alone. The stock rule deployed them whenever the throttle was at idle, airborne as well as on the ground; the FCS drives them now, so idle throttle deploys nothing. false = split surfaces never deploy.");
			cAbElevUp = dev.Bind("7. Airbrake", "AirbrakeOutboardElevonUp", 20f, "Revoker 0.2.8 (TrailingEdgeBrakeConfig): outboard elevons trailing edge up with the airbrake out in the air, as the F-22E's ailerons. 0.2.9: the deflection is solved every frame so that the set adds no lift (with the canards' re-trim of its pitch counted): 14-19 deg on the bench at 400-1200 km/h. This key only switches the outboard share on (> 0) or off (0); 0.2.8 used it as a fixed deflection. The differential stays free for roll.");
			cAbInbDn = dev.Bind("7. Airbrake", "AirbrakeInboardElevonDown", 25f, "Revoker 0.2.8 (TrailingEdgeBrakeConfig): inboard elevon trailing-edge-down deflection (deg, of 25) with the airbrake out in the air, as the F-22E's flaperons.");
			cGbElev = dev.Bind("7. Airbrake", "GroundBrakeOutboardElevon", 30f, "Revoker 0.2.8 (TrailingEdgeBrakeConfig): outboard elevon trailing-edge-up deflection (deg, of 30) in the rollout aero-brake.");
			cGbInb = dev.Bind("7. Airbrake", "GroundBrakeInboardElevon", 25f, "Revoker 0.2.8 (TrailingEdgeBrakeConfig): inboard elevon trailing-edge-up deflection (deg, of 25) in the rollout aero-brake.");
			cGbCan = dev.Bind("7. Airbrake", "GroundBrakeCanard", 45f, "Revoker 0.2.8 (TrailingEdgeBrakeConfig): canard leading-edge-down deflection (deg, of 45) in the rollout aero-brake: drag, and a nose-down moment that keeps the nose wheel loaded.");
			cRotMinSpeed = dev.Bind("Ground", "RotationMinSpeed", 45f, "The take-off/landing rotation law (closed-loop pitch with the mains on the ground) only arms above this TAS (m/s). It used to arm at 15 m/s, an ordinary taxi speed, where it flew closed-loop pitch against the ground constraint and threw the surfaces about. It disarms 8 m/s below this.");
			cGroundRate = dev.Bind("Ground", "GroundRateLaw", defaultValue: true, "2.0.6: on the runway above RotationMinSpeed, pulling the stick hands pitch to the same rate law as gear-down flight, held through the rotation and lift-off (the rate command, its limits and stick scaling are the airborne ones). Stick neutral or forward with all three wheels down for 0.3 s goes back to the open-loop ground mode (steering, aero-brake). false = 2.0.5 behaviour (open-loop stick until the nose is up, then RotationRate).");
			cGroundRotRamp = dev.Bind("Ground", "GroundRotationRamp", 600f, "Revoker 0.2.9: 600 (F-22E 60). At 60 the nose stayed on the wheel for 0.7-0.85 s after a full pull at 200-320 km/h on the bench (in game, 29 Oct 00:25 log at 283 km/h: two full pulls of 0.45 s each did not lift it); at 600 it lifts in 0.16-0.20 s, with the same or smaller pitch-rate overshoot after lift-off. 2.0.6: while the nose wheel still touches in the ground rate law, the nose-up pitch moment the FCS asks for may grow by at most this much per second (in deg/s^2 of pitch acceleration). Stops the surfaces winding to full deflection against the nose leg and over-rotating when it lifts.");
			cNwsFbw = dev.Bind("Ground", "NoseWheelFbw", true, "1.8.36: fly-by-wire ground steering. Pedal commands a yaw rate (GroundYawRateMax at taxi speed, limited to GroundLateralAccel of turn at speed); pedal released = yaw braking to zero rate, then the jet holds a straight line along the track it had (a crab/drift at touchdown is steered out, the heading lined up with the track). The nose wheel is steered by the FCS (kinematic feed-forward + yaw-rate feedback) and, in the ground roll, the rudders join the yaw-rate loop. Below 1.5 m/s the pedal steers the wheel directly (pivoting). false = stock: pedal sets the nose-wheel angle, rudders follow the pedal.");
			cGndYawRate = dev.Bind("Ground", "GroundYawRateMax", 15f, "Full-pedal yaw rate on the ground (deg/s) at taxi speed.");
			cGndLatAcc = dev.Bind("Ground", "GroundLateralAccel", 2.5f, "Full pedal never asks for more than this lateral acceleration (m/s^2) in a ground turn: the yaw-rate command is min(GroundYawRateMax, this / ground speed). 2.5 = full pedal gives 2 deg/s at 260 km/h.");
			cGndLine = dev.Bind("Ground", "GroundLineHold", 1f, "Pedal released: how strongly the jet steers back onto the straight line it was on (1 = full, 0 = hold the track direction only and accept the sideways offset a gust or touchdown drift left).");
			cGndStock = dev.Bind("Ground", "GroundStockMatch", true, "0.5.2: with NoseWheelFbw on, stock steering with a drift limit and the straight-line correction. While the pedal is in (and after release until the yaw has stopped) the nose wheel, the rudders and the differential brake get the stock fly-by-wire's yaw, computed with the jet's own stock parameters (yaw-rate command with yaw damping; raw pedal below the stock minimum speed), so the turn is stock; GroundDrift* limit the sideslip on top. Once the yaw has stopped the FCS holds the straight line along the track until the pedal moves; at touchdown it holds the approach track. GroundYawRateMax and GroundLateralAccel are not used. false = the capped yaw-rate law of 1.8.36-0.4.9.");
			cGearDownSlip = dev.Bind("Ground", "GearDownSlipProtect", true, "0.5.3: the negative-AoA push + roll limiters (sideslip lead and yaw-anchored roll, 2.1.0) also act with the gear down in the air (not with a wheel on the ground). false = off with the gear down, as before.");
			cGndIndi = dev.Bind("Ground", "GroundIndi", true, "0.5.4: INDI on the nose wheel after the pedal is released - the release braking (stop the yaw) and the straight-line / touchdown hold. The wheel effectiveness comes from the game each tick (nose-leg load, tyre friction and slope, CG and yaw inertia); the measured yaw acceleration covers the rest. While the pedal is in the steering stays stock. false = 2.4.13 (stock law on release, PI line hold).");
			cGndIndiK = dev.Bind("Ground", "GroundIndiGain", 15f, "0.5.4: INDI nose wheel: wanted yaw acceleration per deg/s of yaw-rate error (1/s).");
			cGndIndiTau = dev.Bind("Ground", "GroundIndiTau", 0.04f, "0.5.4: INDI nose wheel: low-pass time constant (s) on the measured yaw acceleration and the tyre-force estimate.");
			cGndDriftAllow = dev.Bind("Ground", "GroundDriftAllow", 8f, "0.5.2: sideslip (deg) the jet may drift to on the ground before the steering counters it (stock steering does not limit it). 8 = about half of the Ifrit stock-pedal drift above 50 kt (27 Oct logs: 17-25 deg peaks, 42 in a slalom). 0 = hold the nose on the track; large = no limit.");
			cGndDriftGain = dev.Bind("Ground", "GroundDriftGain", 0.2f, "0.5.2: counter-yaw per degree of sideslip past GroundDriftAllow (1 = full pedal; 0.2 = full counter-yaw 5 deg past the allowance). 0 = drift limit off.");
			cGndDriftLead = dev.Bind("Ground", "GroundDriftLead", 0.25f, "0.5.2: the drift limit acts on the sideslip predicted this far ahead (s) from its rate, so a fast skid is caught before it overshoots.");
			cGndDriftFade = dev.Bind("Ground", "GroundDriftFadeSpeed", 15f, "0.5.2: the drift limit fades in from this ground speed (m/s) to 10 m/s above it (default 15-25 m/s = 29-49 kt), so tight taxi turns keep their natural sideslip.");
			cPitchFight = dev.Bind("4. Gains", "RollPitchFightCost", 1f, "1.8.42: while you roll, a stabilator pitching the nose up (trailing edge up) while the nozzles pitch it down to cancel costs the allocator this x its pitch acceleration squared (rad/s^2), so it rolls with less stab trailing-edge up and the nozzles keep their nose-down travel instead of the roll rate. Near the stabs' roll reversal (the trailing-edge-down stab stalled) one stab went fully trailing-edge up with the other near neutral and the nozzles pinned nose-down. Only that pattern counts (stabs nose-down with nozzles nose-up, the roll-pitch-priority trim, is left alone); off while the pitch-rate command is moving, while the nose comes up faster than commanded, in AoA protection and on AoA run-away. Bench vs 1.8.41: reversals at 19 deg AoA / 300 km/h nozzle min -20 -> -13, AoA over 1.2 -> 0.5 deg; roll stop at 200 km/h -14 -> -10; 2.5 t store at 600 km/h -20 -> -12, over 2.9 -> 0.9; at 26 deg a stab past 35 deg local AoA 32 % -> 0 % of the roll. Costs: reversal at 13 deg / 600 km/h over 3.9 -> 8.3 deg, 10 deg / 300 km/h 1.1 -> 2.7; bank at 1 s at 19-22 deg AoA 42 -> 36 and 28 -> 20 deg. 0 = off (1.8.41).");
			cRotRate = dev.Bind("2. Envelope", "RotationRate", 8f, "Only with GroundRateLaw = false: takeoff / landing rotation (main wheels on the ground, nose wheel up): full stick commands this pitch rate (deg/s) and neutral stick holds the nose attitude.");
			cRollTvc = dev.Bind("4. Gains", "RollNozzleHold", 0f, "1.8.31: default 0 (the handover kept both stabs trailing-edge down for whole roll cycles and bounced the nozzles in roll spam; with ±20 deg the nozzles now take the roll trim). Low-AoA rolls with no pitch command: the stabs take over the roll surfaces' pitch-moment change so the TVC stays near its pre-roll trim. Below ~150 m/s IAS the nozzles still catch the fast part and hand the sustained part to the stabs as far as the stabs have travel to spare; above it the nozzles simply hold still. 1 = full, 0 = off (nozzles trim the roll alone, 1.7.0 behaviour).");
			cAsymPitchMargin = dev.Bind("4. Gains", "AsymPitchMargin", 20f, "1.8.37: with an asymmetric load (lateral CG offset above ~8 cm) the asymmetric-load AoA cap is also held below the AoA where the jet runs out of nose-down authority - full nose-down stabs plus the nozzles at the thrust you have - minus this margin (deg/s^2), less 2 deg. On Aryx's airframe that point is only ~18-19 deg AoA at idle (the stabs stall trailing-edge down), so at idle the old cap (up to ~21 deg) sat right on the edge and any overshoot pitched up into a departure. Throttle up and the nozzles raise it again. Never below 10 deg. 0 = off (1.8.36).");
			cAsymRollCoupling = dev.Bind("4. Gains", "AsymRollCoupling", true, "1.8.37: with an asymmetric load, the roll-coupling limit (CouplingRollLimit) also works at a held AoA, not just while AoA is building past 12-22 deg, and pitch gets priority over roll as soon as AoA overshoots the asymmetric-load cap. Rolling toward the heavy wing at 12-20 deg AoA and idle used to reach 100-160 deg/s and then pitch up past the cap it could not stop in. Clean jets are unaffected. false = 1.8.36.");
			cRollPitchPriority = dev.Bind("4. Gains", "RollPitchPriority", true, "1.8.40: while you roll (below ~18-26 deg AoA), the nozzles stay within RollTvcBudget of their pre-roll trim and the pitch moment the roll makes is taken by the stabilators, with roll rate given up where they cannot. The allocation is re-solved with the flaperons' true pitch moment and the stabs kept on the attached side of their lift peak (past it the down-going stab turns nose-down into pitch-up), pitch and yaw weighted 10x over roll. Accepted only if the modelled pitch error stays within 4 deg/s^2 of what free nozzles would give; otherwise the band widens (x3, x8) or the normal allocation is kept. The nozzles are let go in AoA protection / overshoot past the AoA or asymmetric-load limit and briefly on roll stops and reversals. Bench cost: up to ~20 % roll rate at 15-20 deg AoA, ~35 % in a 1 g roll at 250 km/h (19 deg AoA); none above ~350 km/h at 1 g. RollTvcBudget sets the band: raise it to trade nozzle travel back for roll rate. false = 1.8.39.");
			cRollFightCost = dev.Bind("4. Gains", "RollFightCost", 5f, "1.8.41: while the pedals are in (and 1 s after), the allocator may not split the horizontal surfaces against each other: a group (stabilators, outboard flaperons, inboard flaperons) whose roll opposes the net roll of the others costs this x its roll acceleration squared. With full pedal the rudders reach their stops and the allocator used to buy a few deg/s^2 of yaw from the flaperons' drag by rolling them one way and the stabs the other (roll cancelled) - the large, reversing stab/flaperon deflections on pedal inputs, and all of the yaw once the vertical tails are shot off. Rolls without pedal are allocated exactly as in 1.8.40. Bench, full pedal at 340 km/h: stab-vs-flaperon roll fight mean 24 -> 4 deg/s^2; tails shot off: 79 -> 9. 0 = off (1.8.40).");
			cRollPitchReserve = dev.Bind("4. Gains", "RollPitchReserve", 0.15f, "1.8.41: while you roll, the roll-rate limit and roll-entry acceleration yield when the TVC nozzles have less than this fraction of their nose-down travel (nose-up at negative AoA) left at the live thrust, and come back as the margin returns. Above the RollPitchPriority band (18-26 deg AoA) the stabs lose nose-down authority when they roll (the down-going stab stalls) and the nozzles carry the roll's pitch-up alone: they sat on the -20 deg stop for whole rolls at 22-32 deg AoA, with or without a store. Bench, full roll at 26 deg AoA, AB, 450 km/h: nozzles on the stop 85 % -> 26 % of the roll, bank at 2 s 110 -> 87 deg; with an 800 kg store at the 22 deg cap: 35 % -> 18 %, 87 -> 77 deg. No change below ~18 deg AoA. Higher = more nozzle margin, less roll rate. 0 = off (1.8.40).");
			cRollTvcBudget = dev.Bind("4. Gains", "RollTvcBudget", 1.5f, "1.8.34: 1.5 (1.8.33: 2, 1.8.32: 4). While you roll or yaw, the TVC nozzles stay within this many degrees of where they were before the lateral input (the pitch trim for your own pitch command), and the stabilators take the pitch moment the roll itself makes (mostly the flaperons' lift change). It is an allocator cost, not a clamp: if the stabs cannot supply it (on a stop), the nozzles still go further. The limit opens up to full nozzle travel between 14 and 20 deg AoA (the down-going stab is then near its stall and nose-down from it costs roll rate), between T/W 0.7 and 1.1 (with afterburner the roll trim is only a few degrees and the nozzles stay the fast pitch effector), in AoA protection / run-away and while the pitch-rate command is moving. Cost on the bench: 3-10 % roll rate in 300-350 km/h part-throttle roll spam, about 1 % bank at 1 s in single rolls. 0 = off (1.8.31: the nozzles take the whole roll trim, down to -20 deg at part throttle).");
			cRollTvcBudgetCost = dev.Bind("4. Gains", "RollTvcBudgetCost", 8f, "1.8.34: 8 (1.8.33: 6, 1.8.32: 2). Allocator cost per (degree / 10)^2 of nozzle travel beyond RollTvcBudget. Higher = a harder budget.");
			cRollWash = dev.Bind("4. Gains", "RollNozzleWashout", 0.4f, "Time constant (s) separating 'fast' (nozzles) from 'sustained' (stabs) in RollNozzleHold. Lower = the stabs take over sooner.");
			cSlatStart = dev.Bind("5. Airframe", "SlatAoAStart", 2f, "AoA (deg) at which the slats start coming down. The high-lift device has no aero model beyond the wing area it adds, and the stock AoA term is a step rather than a schedule, so the FCS drives it off AoA directly.");
			cSlatFull = dev.Bind("5. Airframe", "SlatAoAFull", 20f, "AoA (deg) at which the slats are fully down. They then stay down to HighLiftMaxAoA.");
			cHlMaxAoA = dev.Bind("5. Airframe", "HighLiftMaxAoA", 70f, "The slats (high-lift device) stay deployed up to this |AoA|, fading over the 10 deg below it. They used to be tied to the flaperon's trailing-edge-down schedule and so retracted by 30 deg AoA. The device's mechanical TE-down swing is subtracted by the flap drive, so the net flaperon bias is unchanged: only the slats and the wing area they add stay out.");
			cHlHighAoA = dev.Bind("5. Airframe", "HighLiftHighAoAShare", 1f, "How much of the slat deployment is kept at high AoA (0..1). 0 = pre-1.8.0 behaviour (slats retract with the flap schedule).");
			cOuterFlap = dev.Bind("5. Airframe", "OuterFlaperonDroop", 0f, "Revoker 0.1.0: 0 (F-22E/Ifrit 0.6). How far the trailing-edge flaperons droop TE-down with the flap schedule (fraction of the inboard flap angle, stall-capped, roll-biased). 0 = the trailing edges stay at neutral except for roll, and the canards carry a held turn's pitch (with the nozzle drawn to neutral by TvcNeutralHold).");
			cStabRoll = dev.Bind("4. Gains", "StabilatorRollCost", 0.0005f, "Allocator cost of differential (roll) stabilator use at attached flow. Lower = stabs do more of the roll.");
			cStabRollShare = dev.Bind("4. Gains", "StabilatorRollShare", 0.55f, "Share of the differential-stab roll moment counted as usable roll authority (sets roll rate ceiling / acceleration).");
			cOnset = dev.Bind("4. Gains", "PitchOnsetLimit", 150f, "High-speed pitch pacing: pitch-rate command acceleration <= this / lift slope (g per deg). Lower = gentler at high dynamic pressure.");
			cRollPitchFF = dev.Bind("4. Gains", "RollPitchFeedForward", 1f, "Corrects the pitch target by the pitch moment the roll surfaces are about to make (their pitch side-effect is not in the allocator's model), so the stabilators and nozzles move with a roll instead of chasing it afterwards. Mostly visible at low IAS, where the nozzles are the fast pitch effector and the chase shows as nozzle twitch in rolls. 0 = off (pre-1.7.12 behaviour).");
			cLowQReg = dev.Bind("4. Gains", "LowQAllocatorRelief", 1f, "Scales the allocator's regularisation (neutral preference, move cost, pair costs) with dynamic pressure squared, capped at the high-speed value. The cost's error term grows with q^2 and the regularisation did not, so below ~150 m/s the surfaces were held near neutral and near their last position: weak roll and sluggish reversals at low IAS. 0 = pre-1.8.0 behaviour.");
			cRollStop = dev.Bind("3. Lateral-directional", "RollStopFactor", 1f, "Roll-rate ceiling is also limited to this many seconds of the measured roll acceleration (the roll equivalent of YawStopFactor). Since roll acceleration is moment / inertia, this is what makes the roll-rate limit follow a heavy load, especially one far from the CG. 0 = off.");
			cRollAccUse = dev.Bind("3. Lateral-directional", "RollAccelUse", 0.5f, "Share of the roll authority planned for acceleration at and above ~110 m/s IAS (the rest is reserve for regulation against roll damping).");
			cRollAccUseLow = dev.Bind("3. Lateral-directional", "RollAccelUseLowIAS", 0.8f, "Same at and below 60 m/s IAS, blending to RollAccelUse by 110. Roll damping is small down there and the surfaces are far from saturation, so more of the authority can go into the manoeuvre - this is what makes a low-speed reversal crisp.");
			cFFMax = dev.Bind("4. Gains", "RollPitchFeedForwardMax", 30f, "Ceiling on the roll->pitch feed-forward, in deg/s^2 of pitch acceleration. It is an anticipation rather than a measurement, and at high dynamic pressure the roll surfaces' pitch moment is large enough that an error in it moves real g.");
			cFFStabCost = dev.Bind("4. Gains", "RollPitchFeedForwardStabCost", 1f, "During the feed-forward pass, symmetric (pitch) stabilator deflection costs this much more, so the correction lands on the nozzles instead of showing up as a large mean stabilator deflection under the roll's differential. 1 = no preference.");
			cRollPitchFFAoA = dev.Bind("4. Gains", "RollPitchFeedForwardMaxAoA", 35f, "RollPitchFeedForward fades out over the 15 deg below this |AoA| and is off above it: post-stall the stabilators have no pitch authority to redistribute and the pass would only compete for the surfaces holding the jet in yaw.");
			cCoupFF = dev.Bind("4. Gains", "InertialCouplingFF", 0f, "Feed-forward of the gyroscopic roll/pitch/yaw coupling (w x Iw) between commanded and present rates. 0 = off (default since 1.8.14: evaluated at the commanded rates it arrives as a nose-down step the instant you roll, before the jet has the rates; the INDI loop already measures the real coupling).");
			cRelPitchPrio = dev.Bind("4. Gains", "ReleasedStickPitchPriority", 30f, "Stick released (zero-rate hold): when the pitch rate starts to leave the hold, the allocator weights pitch this many times harder against roll/yaw for the stabilators, so a roll cannot pull them into a nose-down shove. Scales in with the pitch-rate error (1 to 9 deg/s) and out with stick (fully off from 18% stick) and with stabilator stall. 1 = off (pre-1.8.14).");
			cRelStabRate = dev.Bind("4. Gains", "ReleasedStickStabRateBound", 0.1f, "Stick released: each allocation may only ask the stabilators for what their servos reach in this many seconds, so the nozzles and wing surfaces are never placed against a stabilator position that is still a third of a second away. Relaxes with stick and stabilator stall. 0 = off (pre-1.8.14).");
			cWashSlew = dev.Bind("4. Gains", "RollNozzleWashoutSlew", 5f, "RollNozzleHold hands the nozzles' pitch trim to the stabs during rolls. While a stab is on its stop that handover is blocked and builds up; this caps how fast it is then applied (deg of nozzle per second), so it cannot all land in one frame the moment the stabs come off the stop - which is when they are braking the roll. 0 = uncapped (pre-1.8.15).");
			cMomLead = dev.Bind("4. Gains", "ReleasedStickMomentLead", 0.035f, "1.8.31: 0.035 (was 0.07; with any control delay 0.07 rang for 1-2 s after a pitch stop). Stick released, zero-rate hold established: the INDI pitch loop leads its measured effector moment by this many seconds of its own rate of change, covering the servo/filter lag when the trim moment is ramping (AoA bleeding off after a hard pull). Fades in over 0.2 s; off with stick, while the pitch-rate command is moving, in AoA protection and with stab stall. 0 = off (pre-1.8.16). Above ~0.1 s it starts to wobble.");
			cStabTrim = dev.Bind("4. Gains", "StabilatorTrimCost", 0.0002f, "Revoker 0.1.0: 0.0002 (F-22E/Ifrit 0.004): the canards carry the steady pitch (held turns, rolls) and the nozzle the transients. Allocator cost of symmetric (pitch) stabilator deflection relative to the nozzles. Higher = steady trim / sustained pitch rate carried by TVC, stabs kept for transients and whatever TVC cannot hold.");
			cUnloadDecel = dev.Bind("4. Gains", "UnloadDecel", 20f, "Easing the stick: pitch-rate deceleration (deg/s^2) toward the rate the aircraft will sustain at the new AoA/G. Lower = smoother.");
			cUnloadRise = dev.Bind("4. Gains", "UnloadRiseGain", 1.5f, "Easing the stick: if AoA still rises during the decel, the pitch-rate target drops by this x AoA rate (to zero at most).");
			cGUnder = dev.Bind("4. Gains", "GUndershootCapture", 1f, "Easing out of a hard pull at high dynamic pressure, the pitch rate is bounded so the load is captured at the commanded G instead of continuing past it into negative g - the same kinematic capture the load limiter uses, applied from above the target. Lower = a softer bound (0.5 allows twice the nose-down rate); 0 = off (pre-1.8.10 behaviour). Only in the G-command regime.");
			cKpG = dev.Bind("4. Gains", "GGain", 2.5f, "G command bandwidth (1/s): load error is converted to AoA error through the live lift slope.");
			cKiG = dev.Bind("4. Gains", "GIntegral", 1f, "G command integral gain (1/s^2), removes steady load error.");
			cActuator = dev.Bind("5. Airframe", "ActuatorSpeedScale", 1f, "Servo speed multiplier for every control surface and the TVC nozzles (Aryx's servos run at 70 deg/s; 1.428571 = 100 deg/s).");
			cStabUp = dev.Bind("5. Airframe", "StabilatorRangeTEUp", 45f, "Revoker 0.2.7: 45 (the canards; stock 40). Canard travel, trailing edge up (deg). 0 = the stock travel.");
			cStabDn = dev.Bind("5. Airframe", "StabilatorRangeTEDown", 45f, "Revoker 0.2.7: 45 (the canards; stock 40). Canard travel, trailing edge down (deg). 0 = the stock travel.");
			cFlapRange = dev.Bind("5. Airframe", "FlaperonRange", 30f, "Ifrit 0.3.0: travel each way for the ailerons (outer) and decelerons (inner) (deg). Stock: ailerons 30, decelerons 20. 0 = stock.");
			cServoSpeed = dev.Bind("5. Airframe", "SurfaceServoSpeed", 100f, "Ifrit 0.3.0: servo speed of every control surface (deg/s), the F-22E FCS's 100. Stock: stabilators 60, ailerons 45, decelerons 70. 0 = the stock speeds x ActuatorSpeedScale. TVC: TvcSlewRate.");
			cSplitRange = dev.Bind("5. Airframe", "SplitRange", 0f, "Ifrit 0.3.0: deceleron split limit (deg), for the airbrake and for split yaw. 0 = the decelerons' own travel (FlaperonRange, 30); the stock limit is 25. Split drag is split x splitDrag x rho x v^2, so 30 is 20 % more than 25.");
			cTvcRange = dev.Bind("5. Airframe", "TvcPitchRange", 0f, "Pitch thrust-vectoring travel each way (deg, game maximum 20). 0 = Aryx's original (10).");
			cTvcSpeed = dev.Bind("5. Airframe", "TvcSlewRate", 70f, "TVC nozzle slew rate (deg/s). 0 = follow ActuatorSpeedScale like the control surfaces (70 x scale).");
			cStabPeakMargin = dev.Bind("5. Airframe", "StabPeakMargin", 90f, "2.0.11: default 90 (off, 2.0.8 behaviour). 2.0.10: below 40 deg body AoA a stab is not commanded more than this many degrees past its lift peak (local AoA, either sign); the limit opens up from 40 to 50 deg AoA (deep-stall use above, e.g. falling leaf, unchanged). Stops the stabs swinging through deep stall in rolls at 36 deg and parking past the peak after a roll (10:00 flight). Bench, full roll at 36 deg with TVC off: stab rate 40 -> 16 deg/s rms, roll rate 27-33 -> 19-25 deg/s; with TVC on the roll rate is within 5 %. 90 = off.");
			cTvcOff = dev.Bind("0.1 Debug - Airframe test", "DisableTVC", defaultValue: false, "2.0.8 test switch: nozzles locked at 0 deg pitch at every throttle, and the FCS flies without them (pitch from the stabs, flaperons and rudders only). With AoALimit Normal the low-thrust caps (NoTvcAoACapPositive / Negative, +-36) then apply (the nozzles never cover pitch). All Ifrits on this machine. Takes effect at once.");
			cAoATest = dev.Bind("0.1 Debug - Airframe test", "AoALimit", AoALimitTest.Normal, "2.0.8 test switch for the AoA limits. Normal = AoALimitPositive / Negative (+65 / -45), capped at NoTvcAoACapPositive / Negative when the nozzles do not cover pitch (low thrust or DisableTVC). Full65 = +65 / -45 and those caps off (the swept authority cap, which stops the FCS pulling past where the stabs can still push the nose down, stays on; LowThrustAoACap = false removes that too). Cap36 = the NoTvcAoACap values in every condition. Takes effect at once.");
			cStabsPitchOnly = dev.Bind("0.1 Debug - Airframe test", "StabsPitchOnly", false, "2.2.5 test switch: the stabilators move symmetrically only (pitch), with no roll or yaw job, so roll and yaw come from the flaperons and rudders alone. For checking what the flaperons can do. The lateral rate limits still count the stabs' authority, so at high AoA the FCS will ask for more roll/yaw than the jet then has (expect slower rolls and more sideslip there). Off for AI. Takes effect at once.");
			cNoTvcCapPos = dev.Bind("0.1 Debug - Airframe test", "NoTvcAoACapPositive", 45f, "2.2.3: default 45 (2.0.9-2.2.2: 36). Positive AoA cap (deg) while the nozzles do not cover pitch (T/W under LowThrustFixedAoATW, or DisableTVC). Also the positive limit with AoALimit = Cap36. Takes effect at once.");
			cNoTvcCapNeg = dev.Bind("0.1 Debug - Airframe test", "NoTvcAoACapNegative", 45f, "2.2.3: default 45 (2.0.9-2.2.2: 36). Negative AoA cap magnitude (deg) under the same conditions (2.0.8 had none: -45 always). Also the negative limit with AoALimit = Cap36. Takes effect at once.");
			cNoTvcThrCapPos = dev.Bind("0.1 Debug - Airframe test", "NoTvcThrustAoACapPositive", 50f, "Revoker 0.2.5: 50 (canards + elevons; bench, TVC off, full push from the cap on afterburner recovers below 20 deg in 2.1-2.3 s at 180-300 km/h; a fixed 50 at idle did not recover, so NoTvcAoACapPositive stays 45; 55+ did not recover at 220 km/h). F-22E: 2.2.2: with DisableTVC, the positive AoA cap (deg) once the thrust is up: NoTvcAoACapPositive (36) up to T/W 0.8, this value from T/W 1.0 (afterburner), linear between. Bench: with afterburner the stabs alone hold 45 deg at 160-750 km/h, rolls and reversals included (sideslip 6 deg or less); at part or idle throttle, rolling as the speed bleeds through ~200-250 km/h, 45 departed to 48-62 deg in 5 of 20 cases where 36 held. Takes effect at once.");
			cNoTvcThrCapNeg = dev.Bind("0.1 Debug - Airframe test", "NoTvcThrustAoACapNegative", 45f, "2.2.2: with DisableTVC, the negative AoA cap magnitude (deg) at every thrust (NoTvcAoACapNegative still applies at low thrust with the nozzles working). Bench: -45 held at 200-450 km/h, idle to afterburner, with a roll, sideslip 5.4 deg or less. Takes effect at once.");
			cAirframeRev = dev.Bind("5. Airframe", "AirframeRevision", 13, "Internal: airframe-default revision (inherited from the Ifrit FCS). Leave it alone.");
			cFuelUse = dev.Bind("5. Airframe", "FuelConsumptionScale", 0.65f, "Fuel burn multiplier at every throttle setting, afterburner included (0.65 = 35 % less fuel used per throttle setting). Buffed PerformanceProfile only.");
			cThrust = dev.Bind("5. Airframe", "ThrustScale", 1.08f, "Engine thrust multiplier (dry staticThrust and afterburner thrust alike). Revoker 0.1.0: 1.08. Buffed PerformanceProfile only. Fuel flow per power setting is unchanged by this; see FuelConsumptionScale.");
			cDrag = dev.Bind("5. Airframe", "ParasiticDragScale", 0.90f, "Multiplier on every part's parasitic drag area (1 = stock). Revoker 0.1.0: 0.90 (10 % less parasitic drag). Buffed PerformanceProfile only.");
			cDrAoA = dev.Bind("5. Airframe", "AoADragRelief", 0f, "Revoker 0.1.0: 0 (off). Buffed profile only: post-stall AoA drag relief (F-22E 2.2.3 feature for Aryx's airfoils). The Revoker keeps its stock drag polar.");
			cDrTrans = dev.Bind("5. Airframe", "TransonicDragRelief", 0f, "Revoker 0.1.0: 0 (off). Buffed profile only: Mach drag relief (F-22E 1.8.38 feature). The Revoker keeps its stock transonic drag.");
			cStockDrAoA = dev.Bind("5. Airframe", "LowAoADragRelief", 0f, "Revoker 0.1.0: 0 (off). Both profiles: fraction of each part's AoA-dependent drag removed at low AoA (F-22E 2.2.3, a fix for Aryx's airfoil drag). The Revoker keeps its stock drag polar.");
			cStockDrFade0 = dev.Bind("5. Airframe", "LowAoADragReliefFadeStart", 25f, "2.2.3: local AoA (deg) up to which LowAoADragRelief applies in full (both profiles).");
			cLatMargin0 = dev.Bind("3. Lateral-directional", "HighAoALateralMarginStart", 25f, "Ifrit 0.3.1: AoA (deg) where HighAoALateralBrakeMargin (main cfg) starts to blend in.");
			cLatMargin1 = dev.Bind("3. Lateral-directional", "HighAoALateralMarginFull", 30f, "Ifrit 0.3.1: AoA (deg) from which HighAoALateralBrakeMargin applies in full (smoothstep in between).");
			cSplitBudget = dev.Bind("5. Airframe", "DeceleronSplitBudget", true, "Ifrit 0.3.1: the deceleron's deflection and its split share one travel: the game turns the two halves to deflection +- split, so a 30 deg deflection with a 30 deg split put one half at 60. Now |deflection| + split <= the deceleron travel (FlaperonRange); the deflection comes first and the split (airbrake and split yaw) gets what is left. false = 0.3.0 (they add).");
			cDragBump = dev.Bind("5. Airframe", "AoADragBump", 0f, "Revoker 0.1.0: 0 (off). Ifrit 0.3.1 feature: extra drag over a band of body AoA.");
			cDragBump0 = dev.Bind("5. Airframe", "AoADragBumpStart", 18f, "Ifrit 0.3.1: body |AoA| (deg) where the AoADragBump starts rising.");
			cDragBump1 = dev.Bind("5. Airframe", "AoADragBumpPeak", 24f, "Ifrit 0.3.1: body |AoA| (deg) of the full AoADragBump.");
			cDragBump2 = dev.Bind("5. Airframe", "AoADragBumpEnd", 35f, "Ifrit 0.3.1: body |AoA| (deg) by which the AoADragBump is gone.");
			cStockDrFade1 = dev.Bind("5. Airframe", "LowAoADragReliefFadeEnd", 30f, "2.2.3: local AoA (deg) by which the relief has gone over to its post-stall value: none in Stock, AoADragRelief in Buffed.");
			cAsymOn = dev.Bind("Asymmetric load", "AsymLoadAoALimit", defaultValue: true, "Cap the AoA when an asymmetric load (e.g. one heavy store on one wing) would use more of the roll/yaw control authority than AsymLateralReserve leaves free. The cap is the stick's full-travel AoA (the stick is remapped onto it). A load within the margin is simply trimmed out by the control loop (no cap). Not applied in MPO.");
			cAsymRollHead = dev.Bind("Asymmetric load", "AsymRollHeadroom", defaultValue: true, "The roll-rate and roll-acceleration limits count only the roll authority left in each direction after the measured asymmetric trim (asymmetric store, wing damage). The trim is learned from the measured non-control roll moment minus roll damping while the jet is not rolling (under 20 deg/s) and held during rolls; it only ever lowers the limit on the loaded side. With the left wing shot off it stops a full right roll at the AoA cap from saturating the stabs and rudders, building sideslip and departing. No effect on a symmetric jet. false = pre-1.8.17.");
			cAsymReserve = dev.Bind("Asymmetric load", "AsymLateralReserve", 0.35f, "Share of the roll/yaw control authority kept free for the pilot's roll and pedal commands (0..1). Used together with AsymRollReserveAccel / AsymYawReserveAccel: whichever asks for more sets the cap. Higher = lower cap, more roll/yaw left.");
			cAsymDead = dev.Bind("Asymmetric load", "AsymDeadbandAccel", 5f, "Asymmetric roll acceleration (deg/s^2) ignored before any cap is considered - the airframe is never perfectly symmetric even clean (off-centre gun and ammunition, engines never exactly matched). The yaw deadband is a quarter of this. A balanced loadout stays well inside it; one 300 kg store is far outside it.");
			cAsymRollAcc = dev.Bind("Asymmetric load", "AsymRollReserveAccel", 80f, "Roll acceleration (deg/s^2) that must stay available to the pilot after the asymmetric load is trimmed out, whatever AsymLateralReserve works out to. Post-stall a share of the remaining authority is almost nothing, so this absolute floor is what sets the cap up high and at low speed. Higher = lower cap.");
			cAsymYawAcc = dev.Bind("Asymmetric load", "AsymYawReserveAccel", 20f, "Same for yaw acceleration (deg/s^2).");
			cAsymFF = dev.Bind("Asymmetric load", "AsymFeedForward", 0f, "DEFAULT 0 (off) since 1.8.17. Hands the CHANGE in the modelled asymmetric moment straight to the allocator, washed out over AsymFeedForwardWashout seconds. Inside the INDI loop an added moment is not a one-off kick: the loop measures the acceleration it causes and settles with a steady roll-rate error of about FF/(I x KRoll) for as long as the term lasts, and while the load keeps changing (pulling, levelling off) it lasts the whole time. With a wing shot off it gave a 20-35 deg/s roll drift in pulls and level-offs and helped depart the jet. The loop already measures the asymmetric moment. 1 = pre-1.8.17.");
			cAsymFFTau = dev.Bind("Asymmetric load", "AsymFeedForwardWashout", 1f, "Time constant (s) over which the feed-forward above hands the load back to the control loop.");
			cAsymFFBeta = dev.Bind("Asymmetric load", "AsymFeedForwardMaxSideslip", 6f, "AsymFeedForward fades out over the 12 deg of sideslip above this and is off beyond it: the asymmetric moment it is built from is modelled at zero sideslip, so at large sideslip (falling leaf) it is not a measurement of anything.");
			cRevRoll = dev.Bind("3. Lateral-directional", "ReversedFlowRollSense", defaultValue: true, "Flying backwards: the velocity-vector roll maps to body roll through cos(AoA), which flips sign past 90 deg, so the stick would roll the jet the other way from the pilot's seat. This flips the roll command with it, blended smoothly from ReversedFlowSenseStart to ReversedFlowSenseEnd (0.3.4; 0.3.2 flipped at once at 100 deg AoA, released at 80). Pitch and pedal are unaffected - the control loop already handles reversed flow on the surfaces themselves.");
			cAsymFFAoA = dev.Bind("Asymmetric load", "AsymFeedForwardMaxAoA", 40f, "AsymFeedForward fades out over the 15 deg of AoA above this and is off beyond it. Post-stall the lateral authority it would spend is what the manoeuvre needs.");
			cAsymRollTau = dev.Bind("Asymmetric load", "AsymRollTau", 0.5f, "The roll reserve also has to cover the roll rate the law allows at this speed, reached in this many seconds. At high IAS that is what sets the reserve (250 deg/s in 0.5 s = 500 deg/s^2), not AsymRollReserveAccel. 0 = off (fixed reserve only).");
			cAsymRelaxBoost = dev.Bind("Asymmetric load", "AsymCapRelaxBoost", 4f, "The cap relaxes faster the further the demand is below the limit: rate = AsymCapRelaxRate x (1 + this x margin). With the load gone the cap comes back at (1 + this) x the base rate. 0 = fixed rate.");
			cAsymBoost = dev.Bind("Asymmetric load", "AsymAxisPriorityBoost", 0f, "With more than half the lateral authority taken by an asymmetric load, the allocator's roll and yaw weight is raised by up to this factor (1 = up to double) so the surfaces hold the jet straight before they fine-tune pitch. DEFAULT 0 (off) since 1.7.11: it starves the pitch axis, which showed up as the nozzles running nose-down in rolling pulls.");
			cAsymFloor = dev.Bind("Asymmetric load", "AsymMinAoACap", 12f, "The asymmetric-load cap never goes below this positive AoA (deg).");
			cAsymFloorNeg = dev.Bind("Asymmetric load", "AsymMinAoACapNegative", 8f, "The asymmetric-load cap never goes closer to 0 than this on the negative side (deg, magnitude).");
			cAsymTighten = dev.Bind("Asymmetric load", "AsymCapTightenRate", 10f, "How fast the cap comes down when the asymmetry grows (deg/s).");
			cAsymRelax = dev.Bind("Asymmetric load", "AsymCapRelaxRate", 8f, "How fast the cap goes back up when the asymmetry shrinks (store released, fuel, speed) (deg/s).");
			cAsymRec = dev.Bind("Asymmetric load", "AsymCapRecoveryRate", 15f, "Above the cap (it just came down), AoA is brought back to it at up to this rate (deg/s). Not cancelled by full stick.");
			cAsymMinIas = dev.Bind("Asymmetric load", "AsymMinIAS", 35f, "Below this IAS (m/s) the cap is held at its last value instead of being recomputed (no aero authority to judge by).");
			cAsymCapDemand = dev.Bind("Asymmetric load", "AsymCapDemand", 1.08f, "1.8.45: lateral demand (load share of the authority left after the reserve) at which the asymmetric-load AoA cap sits (1.8.44 and before: 1.0). With a light mixed load (6.5 cm lateral CG offset) the demand rises ~0.025 per degree above the cap, so 1.08 lets ~3 deg more AoA; heavier loads, whose demand rises faster with AoA, gain less.");
			cAsymCapSmooth = dev.Bind("Asymmetric load", "AsymCapSmoothing", 1f, "1.8.45: time constant (s) with which the asymmetric-load cap may rise. Each 0.2 s sweep's cap goes through a median of the last three and then this low-pass; a lower cap is followed with 0.3 s. The raw cap moved 1-8 deg every sweep (4.4 Hz) with a 6.5 cm lateral offset at 35-40 deg AoA, which is what made full-aft stick twitch.");
			cDecelLook = dev.Bind("Asymmetric load", "DecelLookahead", 1f, "1.8.45: seconds of IAS decay projected ahead (filtered IAS rate, rapid decelerations only, see DecelStart/DecelFull). The asymmetric-load cap is judged, and the high-AoA roll/yaw rate limits are set, at the dynamic pressure of that predicted IAS, so they come down before the surfaces lose the authority. 0 = off.");
			cDecelStart = dev.Bind("Asymmetric load", "DecelStart", 6f, "1.8.45: IAS deceleration (m/s^2) below which nothing is anticipated. Between this and DecelFull the deceleration is weighted by the square of where it sits, so ordinary turning bleed (and most of a J-turn's tail) predicts almost nothing and a collapsing corkscrew predicts nearly all of it.");
			cDecelFull = dev.Bind("Asymmetric load", "DecelFull", 20f, "1.8.45: IAS deceleration (m/s^2) from which it is anticipated in full.");
			cHiAoABrake = dev.Bind("3. Lateral-directional", "HighAoABrakeFloor", true, "1.8.45: from 25 to 35 deg AoA up, the roll and yaw rate limits may not exceed what the stop-side authority (at the IAS predicted by DecelLookahead) brakes in the stop time of the reference manoeuvre, full pedal + full lateral stick at 65 deg AoA and 280 km/h (RollStopFactor for roll, YawStopFactor x the yaw boost at 280 km/h for yaw). Replaces there the fixed 30 deg/s roll and 20 x boost yaw floors, which outran the authority below ~65 m/s IAS. Costs yaw rate in slow post-stall yaw: bench, 65 deg AoA at 200 km/h 34 -> 23 deg/s, 300 km/h 37 -> 32. false = 1.8.44.");
			cCmdShape = dev.Bind("4. Gains", "CommandShaping", true, "1.9.0: AoA/G command pre-filter. The AoA and G commands are shaped before the loop: a small request (a few degrees or a fraction of a g from where the jet is) builds up gently, a large one runs as before. The shaping speed (1/s) grows with the step from ShapingAoAGainSmall to ShapingAoAGainLarge over ShapingAoASpan (G: ShapingG*), and small steps also ramp their onset over ShapingOnset. The same idea as the self-adjusting pre-filter gain of the T-50 control law (60 % gain below 1 g of error, 100 % above 3 g). false = 1.8.45.");
			cShapeASpan = dev.Bind("4. Gains", "ShapingAoASpan", 15f, "1.9.0: AoA step (deg) from which the AoA command is no longer slowed.");
			cShapeAKSmall = dev.Bind("4. Gains", "ShapingAoAGainSmall", 1.5f, "1.9.0: shaping speed (1/s) of the AoA command for a very small step. Lower = gentler fine tracking.");
			cShapeAKLarge = dev.Bind("4. Gains", "ShapingAoAGainLarge", 10f, "1.9.0: shaping speed (1/s) of the AoA command at and above ShapingAoASpan (fast enough to be transparent).");
			cShapeOnset = dev.Bind("4. Gains", "ShapingOnset", 0.25f, "1.9.0: onset time (s) of a very small AoA/G step (the command's rate builds up over this); fades to 0.03 s at the span.");
			cShapeGSpan = dev.Bind("4. Gains", "ShapingGSpan", 3f, "1.9.0: load step (g) from which the G command is no longer slowed.");
			cShapeGKSmall = dev.Bind("4. Gains", "ShapingGGainSmall", 1.5f, "1.9.0: shaping speed (1/s) of the G command for a very small step (onset and unload).");
			cShapeGKLarge = dev.Bind("4. Gains", "ShapingGGainLarge", 10f, "1.9.0: shaping speed (1/s) of the G command at and above ShapingGSpan.");
			cRateBlend = dev.Bind("2. Envelope", "HighAoARateBlend", true, "1.9.1: AoA command hands over to pitch-rate command past RateBlendAoA. The stick scale is fixed: RateBlendAoA at RateBlendStick (36 deg at 0.5 = 72 deg per full stick); the threshold's stick point moves with the threshold (24 deg -> 0.333). Up to that point the stick is AoA command; past it, still AoA command, on to the AoA limit at full stick (full stick goes through the threshold at the rate budget). Holding the threshold's point holds the threshold AoA. When the jet crosses the threshold with the stick past its point, the stick becomes a pitch-rate command starting from the pitch rate at the crossing, up to the budget at full stick; a full pull or a release switches it to 0..full stick = 0..budget, and a release holds zero rate. Easing below the point stays rate command until the jet is back at the threshold. If the jet comes back to the threshold with the stick still past the point, that stick position is the threshold: easing from it holds the live threshold (soft knee) until the stick meets the threshold's own point on the line. false = AoA command to the limit (1.8.45).");
			cRateBlendStick = dev.Bind("2. Envelope", "RateBlendStick", 0.5f, "1.9.1: stick position of RateBlendAoA; sets the AoA per stick below the threshold (36 / 0.5 = 72 deg per full stick). It stays fixed when the threshold moves.");
			cRateBlendAoA = dev.Bind("2. Envelope", "RateBlendAoA", 36f, "1.9.0: AoA (deg) where the AoA command hands over to pitch-rate command when the wings still lift at least RateBlendLiftG times the weight.");
			cRateBlendAoAMin = dev.Bind("2. Envelope", "RateBlendAoAMin", 25f, "1.9.0: the handover AoA when the wings can lift only the weight (1 g, thrust not counted); in between it is interpolated by the lift-to-weight ratio (dynamic pressure x wing lift / mass). Slower = lower threshold.");
			cRateBlendLiftG = dev.Bind("2. Envelope", "RateBlendLiftG", 2f, "1.9.0: wing lift-to-weight ratio (g, thrust not counted) from which the full RateBlendAoA applies. ~330 km/h clean at combat weight.");
			cRateBlendBand = dev.Bind("2. Envelope", "RateBlendBand", 4f, "1.9.0: AoA band (deg) below the threshold over which rate command and AoA command are blended once the rate command is engaged (pitch rate stays continuous across it).");
			cRateBlendNeg = dev.Bind("2. Envelope", "HighAoARateBlendNegative", true, "1.9.7: the same AoA -> pitch-rate hand-over on the negative side, mirrored: RateBlendAoANegative at -RateBlendStick (-36 deg at -0.5 = 72 deg per full push, the same scale as the positive side), full push = the negative AoA limit, rate command (nose-down) past the threshold, floating hold point. RateBlendStick, RateBlendLiftG and RateBlendBand are shared. false = linear AoA command to the negative limit (1.9.6).");
			cRateBlendAoANeg = dev.Bind("2. Envelope", "RateBlendAoANegative", 36f, "1.9.7: negative hand-over AoA magnitude (deg) while the wings can still push at least RateBlendLiftG times the weight nose-down (negative lift).");
			cRateBlendAoAMinNeg = dev.Bind("2. Envelope", "RateBlendAoAMinNegative", 25f, "1.9.7: negative hand-over AoA magnitude when the wings' negative lift is only the weight; interpolated by the negative lift-to-weight ratio in between.");
			cAuthCap = dev.Bind("2. Envelope", "LowThrustAoACap", true, "1.9.0: the AoA limit is held below the AoA where the nose-down authority you have now runs out: stabilators full nose-down, flaperons at their trim, plus the nozzles at the present thrust (none above the thrust-vectoring cut-off, 450 m/s true airspeed - the high-altitude supersonic case where the nozzles freeze). Bench, 350 km/h: zero thrust 23 deg, ~0.1 T/W 26 deg, 30 % throttle 34 deg, full afterburner no cap; a full pull then full push recovers in every case (1.8.45 departed to 73-90 deg at the first two). The stick is remapped onto the capped envelope like the asymmetric-load cap. false = 1.8.45.");
			cAuthCapMargin = dev.Bind("2. Envelope", "LowThrustCapMargin", 8f, "1.9.0: nose-down pitch acceleration (deg/s^2) that must be left at the cap. Higher = lower cap.");
			cAuthCapFloor = dev.Bind("2. Envelope", "LowThrustCapFloor", 20f, "1.9.0: the low-thrust AoA cap never goes below this (deg).");
			cLowThrustTW = dev.Bind("2. Envelope", "LowThrustFixedAoATW", 0.3f, "2.0.5: below this thrust-to-weight the nozzles are treated as not covering pitch and the AoA limit is fixed at NoTvcAoACapPositive / Negative (+-36 by default; 2.0.8 and before: +36 only, released 0.03 above). The swept LowThrustAoACap still applies under it.");
			cHybridIndi = dev.Bind("4. Gains", "HybridIndi", 1f, "1.9.0: pitch-loop lag compensation. The incremental (INDI) pitch loop estimates the airframe's own moment from filtered sensors; at high dynamic pressure that moment moves fast with AoA and pitch rate and the filter lag made the pitch rate overshoot (bench, 2 g command at 1300 km/h: 2.8 g peak then 1.8 g, a slow G oscillation). The lag is now predicted from the airframe model (moment slope with AoA and pitch rate x the filter lag). 0 = off (1.8.45).");
			cHybridHiLead = dev.Bind("4. Gains", "HybridIndiHighAoALead", 0.15f, "0.4.9 (from F-22E 2.4.7/2.4.8): disturbance-rate lead, seconds. Above 25-40 deg AoA with the pitch stick released, the pitch loop takes the airframe's own pitch acceleration (what the effectors are not doing) forward by its rate x this, so a steady fall in it as the AoA unwinds after a corkscrew release is met when it arrives instead of ~0.1 s late. Pitch damping is left out. Fades out with T/W 0.4 -> 0.2 (the horizon is the nozzles' response). 0 = off (0.4.8).");
			cAllocNozzleNewton = dev.Bind("4. Gains", "AllocNozzleNewton", true, "0.4.9 (from F-22E 2.4.7/2.4.8): with the pitch stick released and T/W at least 0.25, the allocator's last pass also tries a local Newton step for the nozzles, not only their grid points (every 4 deg). Without it the nozzles could sit between two grid points with a pitch miss while the joint steps were rejected (stabs stalled on their stops, large yaw miss): the post-corkscrew nose dip. false = 0.4.8.");
			cYawRefKmh = dev.Bind("3. Lateral-directional", "YawBrakeRefIAS", 270f, "1.9.2: IAS (km/h) of the reference manoeuvre for the high-AoA yaw braking floor (HighAoABrakeFloor): full pedal + full lateral stick at max AoA. The yaw-rate limit may not exceed what the yaw authority brakes in YawStopFactor x the low-IAS yaw boost at this IAS (1.8.45: 280).");
			cYawMargStart = dev.Bind("3. Lateral-directional", "YawStopTimeStartIAS", 210f, "1.9.3: below this IAS (km/h) the allowed yaw stop time grows smoothly (smoothstep) to YawStopTimeLowIAS x at YawStopTimeFullIAS; flat above and below.");
			cYawMargFull = dev.Bind("3. Lateral-directional", "YawStopTimeFullIAS", 150f, "1.9.3: IAS (km/h) at and below which the full YawStopTimeLowIAS applies.");
			cYawMargLow = dev.Bind("3. Lateral-directional", "YawStopTimeLowIAS", 1.5f, "1.9.3: allowed yaw stop time at low IAS relative to the reference: 1.5 = 50 % more yaw rate for the same yaw authority (slower braking). 1 = flat. (1.9.2's YawBrakeMargin* keys worked the other way and are no longer read.)");
			cYawMidF = dev.Bind("3. Lateral-directional", "YawStopTimeMidFactor", 1f, "Revoker 0.2.4: 1 (no mid-IAS band; the F-22E rudder rule cut the corkscrew entry yaw limit 77 -> 54 deg/s from 330 to 290 km/h). F-22E: 2.0.12: 0.75 (1.9.4-2.0.11: 0.8). high-AoA yaw stop time (so yaw-rate limit) factor between YawStopTimeMidLoIAS and YawStopTimeMidHiIAS: 0.8 = 20 % less yaw rate for the same authority, faster braking. Smoothstep ramps of YawStopTimeMidRamp km/h outside the band. 1 = off.");
			cYawMidLo = dev.Bind("3. Lateral-directional", "YawStopTimeMidLoIAS", 280f, "2.0.12: 280 (1.9.4: 290). lower edge (km/h) of the mid-IAS yaw band (full factor from here up).");
			cYawMidHi = dev.Bind("3. Lateral-directional", "YawStopTimeMidHiIAS", 380f, "2.0.12: 380 (1.9.4: 330). upper edge (km/h) of the mid-IAS yaw band (full factor up to here).");
			cYawMidRamp = dev.Bind("3. Lateral-directional", "YawStopTimeMidRamp", 30f, "1.9.4: width (km/h) of the smooth ramps outside the mid-IAS yaw band (250-280 and 380-410 by default).");
			DevUnlocked = devUnlocked;
			// Revoker 0.2.0: one-time move of the 0.1.0 defaults that changed (AoALimitPositive 65 -> 70, ElevonPitch false ->
			// true). A dev file written by 0.1.0 has no RevokerConfigRevision line; values set by hand to anything else stay.
			bool revOld = devUnlocked && !devFresh && !FileHasKey(dev.ConfigFilePath, "RevokerConfigRevision");
			cRevokerRev = dev.Bind("2. Envelope", "RevokerConfigRevision", 6, "Internal: the Revoker config revision this file was last updated to. Leave it alone.");
			if (revOld)
			{
				cRevokerRev.Value = 0;
			}
			if (cRevokerRev.Value < 1)
			{
				if (Mathf.Abs(cAoAPos.Value - 65f) < 0.001f)
				{
					cAoAPos.Value = 70f;
				}
				if (!cElevonPitch.Value)
				{
					cElevonPitch.Value = true;
				}
				cRevokerRev.Value = 1;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.0 defaults: AoALimitPositive 70, ElevonPitch on (moved from the 0.1.0 defaults once).");
			}
			// Revoker 0.2.2: MaxYawRate 75 -> 90 once (a 0.2.0 / 0.2.1 dev file holds the old default)
			if (cRevokerRev.Value < 2)
			{
				if (Mathf.Abs(cMaxYaw.Value - 75f) < 0.001f)
				{
					cMaxYaw.Value = 90f;
				}
				cRevokerRev.Value = 2;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.2 defaults: MaxYawRate 90 (moved from 75 once).");
			}
			// Revoker 0.2.4: YawStopTimeMidFactor 0.75 -> 1 once
			if (cRevokerRev.Value < 3)
			{
				if (Mathf.Abs(cYawMidF.Value - 0.75f) < 0.001f)
				{
					cYawMidF.Value = 1f;
				}
				cRevokerRev.Value = 3;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.4 defaults: YawStopTimeMidFactor 1 (moved from 0.75 once).");
			}
			// Revoker 0.2.5: NoTvcThrustAoACapPositive 45 -> 50 once
			if (cRevokerRev.Value < 4)
			{
				if (Mathf.Abs(cNoTvcThrCapPos.Value - 45f) < 0.001f)
				{
					cNoTvcThrCapPos.Value = 50f;
				}
				cRevokerRev.Value = 4;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.5 defaults: NoTvcThrustAoACapPositive 50 (moved from 45 once).");
			}
			// Revoker 0.2.7: canard travel 40 -> 45 (StabilatorRangeTEUp/Down 0 -> 45) and AoALimitNegative 50 -> 55, once
			if (cRevokerRev.Value < 5)
			{
				if (cStabUp.Value < 0.001f && cStabDn.Value < 0.001f)
				{
					cStabUp.Value = 45f;
					cStabDn.Value = 45f;
				}
				if (Mathf.Abs(cAoANeg.Value - 50f) < 0.001f)
				{
					cAoANeg.Value = 55f;
				}
				cRevokerRev.Value = 5;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.7 defaults: canard travel 45 (StabilatorRangeTEUp/Down, from 0 = stock 40) and AoALimitNegative 55 (from 50), once.");
			}
			// Revoker 0.2.9: GroundRotationRamp 60 -> 600, once (only if it was exactly 60)
			if (cRevokerRev.Value < 6)
			{
				if (Mathf.Abs(cGroundRotRamp.Value - 60f) < 0.001f)
				{
					cGroundRotRamp.Value = 600f;
				}
				cRevokerRev.Value = 6;
				if (devUnlocked)
				{
					dev.Save();
				}
				Log.LogInfo("Revoker FCS 0.2.9 defaults: GroundRotationRamp 600 (from 60), once.");
			}
			MigrateOldConfig(cfg, dev, devUnlocked && devFresh);
			// Ifrit 0.2.0: 0.1.0 ran the F-22E 2.0.3 airframe migration and wrote 1.2 into the (unused) effectiveness keys; reset
			// them once. The Ifrit airframe is never effectiveness-scaled.
			// Ifrit 0.2.1: the rollout aero-brake is off; the split decelerons are the airbrake on the ground too
			if (cIfritRev.Value < 2)
			{
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 2;
					dev.Save();
				}
			}
			// Ifrit 0.2.2: the Buffed profile's parasitic drag 0.885 -> 0.95 and transonic relief 0.2 -> 0.1; a file still on the
			// old defaults is moved once (a value set by hand is left alone)
			if (cIfritRev.Value < 3)
			{
				if (Mathf.Abs(cDrag.Value - 0.885f) < 0.0005f)
				{
					cDrag.Value = 0.95f;
				}
				if (Mathf.Abs(cDrTrans.Value - 0.2f) < 0.0005f)
				{
					cDrTrans.Value = 0.1f;
				}
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 3;
					dev.Save();
				}
			}
			// Ifrit 0.3.0: FlaperonRange stock (0) -> 30 (ailerons and decelerons +-30); the servo speed and split limit are new keys
			if (cIfritRev.Value < 4)
			{
				if (cFlapRange.Value <= 0f)
				{
					cFlapRange.Value = 30f;
				}
				if (Mathf.Abs(cSplitBrakeHold.Value - 0.05f) < 0.0001f)
				{
					cSplitBrakeHold.Value = 0.5f;
				}
				if (cIfritRev.Value >= 1)
				{
					cIfritRev.Value = 4;
					dev.Save();
				}
			}
			// Ifrit 0.3.1: roll TVC cost 5e-5 -> 0.003 (uneven nozzles on pitch release), rolling-pull sideslip margin 9 -> 15 and
			// the high-AoA sideslip gain ramp from 5 deg (was 25): the corkscrew-entry AoA hitch near 20 deg
			if (cIfritRev.Value < 5)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cTvcDiffMu.Value - 5E-05f) < 1E-07f)
					{
						cTvcDiffMu.Value = 0.003f;
					}
					if (Mathf.Abs(cPullSlipMargin.Value - 9f) < 0.001f)
					{
						cPullSlipMargin.Value = 15f;
					}
					if (Mathf.Abs(cSlipGainA0.Value - 25f) < 0.001f)
					{
						cSlipGainA0.Value = 5f;
					}
					cIfritRev.Value = 5;
					dev.Save();
				}
			}
			// Ifrit 0.3.7: MPO lateral law default VelocityVectorRoll -> SeparateRollYaw; a file still holding the old default
			// is moved once (a SeparateRollYaw / later choice is never touched)
			if (cIfritRev.Value < 6)
			{
				if (cIfritRev.Value >= 1)
				{
					if (cMpoSep.Value == MpoRollMode.VelocityVectorRoll)
					{
						cMpoSep.Value = MpoRollMode.SeparateRollYaw;
						cfg.Save();
					}
					cIfritRev.Value = 6;
					dev.Save();
				}
			}
			// Ifrit 0.4.0: the engine-out AoA cap 30 -> 22 (bench, one engine at AB: at 30 the pull bleeds to 50-85 m/s where the
			// tails, splits and crossing are all spent and the live engine gets cut; 22 holds without the throttle cut from
			// 300 km/h). A file still holding 30 is moved once.
			if (cIfritRev.Value < 7)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cEngOutCap.Value - 30f) < 0.01f)
					{
						cEngOutCap.Value = 22f;
					}
					cIfritRev.Value = 7;
					dev.Save();
				}
			}
			// Ifrit 0.4.1: the buffed variant's fuel burn 0.75 -> 0.65 (35 % less). A file still holding 0.75 is moved once.
			if (cIfritRev.Value < 8)
			{
				if (cIfritRev.Value >= 1)
				{
					if (Mathf.Abs(cFuelUse.Value - 0.75f) < 0.001f)
					{
						cFuelUse.Value = 0.65f;
					}
					cIfritRev.Value = 8;
					dev.Save();
				}
			}
			if (cIfritRev.Value < 1)
			{
				if (cAirframeRev.Value < 3)
				{
					cAirframeRev.Value = 3;
				}
				cIfritRev.Value = 8;
				dev.Save();
			}
			// 2.0.12: the high-AoA mid-IAS yaw band is 280-380 km/h at 0.75 (25 % less yaw braking margin); a file still on
			// 1.9.4's 290-330 / 0.8 is moved once
			if (cAirframeRev.Value < 6)
			{
				if (Mathf.Abs(cYawMidF.Value - 0.8f) < 0.001f)
				{
					cYawMidF.Value = 0.75f;
				}
				if (Mathf.Abs(cYawMidLo.Value - 290f) < 0.01f)
				{
					cYawMidLo.Value = 280f;
				}
				if (Mathf.Abs(cYawMidHi.Value - 330f) < 0.01f)
				{
					cYawMidHi.Value = 380f;
				}
			}
			if (cAirframeRev.Value < 6)
			{
				cAirframeRev.Value = 6;
			}
			// 2.2.23: rolling-pull governor defaults 6 / 0 -> 9 / 0.7; a file still on the old pair is moved once, anything
			// set by hand is kept
			if (cAirframeRev.Value < 7)
			{
				if (Mathf.Abs(cPullSlipMargin.Value - 6f) < 0.001f)
				{
					cPullSlipMargin.Value = 9f;
				}
				if (Mathf.Abs(cPullRollTrade.Value) < 0.0001f)
				{
					cPullRollTrade.Value = 0.7f;
				}
				Log.LogInfo($"Revoker FCS 0.3.0 (F-22E 2.2.23) defaults: PullSideslipMargin {cPullSlipMargin.Value}, PullRollTrade {cPullRollTrade.Value}.");
				cAirframeRev.Value = 7;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// 2.2.26 (revision 8) moved PullRollTrade 0.7 -> 1; 2.2.27 undoes it
			// 2.2.27: PullRollTrade back to 0.7. A file that 2.2.26 moved to 1 (revision 8 with the value 1) goes back once;
			// a file never touched by 2.2.26 keeps its value
			if (cAirframeRev.Value < 9)
			{
				if (cAirframeRev.Value == 8 && Mathf.Abs(cPullRollTrade.Value - 1f) < 0.001f)
				{
					cPullRollTrade.Value = 0.7f;
				}
				Log.LogInfo($"Revoker FCS 0.3.0 (F-22E 2.2.27) defaults: PullRollTrade {cPullRollTrade.Value}.");
				cAirframeRev.Value = 9;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// 2.2.29: ReversedFlowRollSense on (a file holding false is set to true once)
			if (cAirframeRev.Value < 10)
			{
				cRevRoll.Value = true;
				Log.LogInfo("Revoker FCS 0.3.0 (F-22E 2.2.29) defaults: ReversedFlowRollSense true.");
				cAirframeRev.Value = 10;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.1: fuel moved twice as fast, balancing 40 -> 80 kg/s and trim 120 -> 240; files still on the old values are moved once
			if (cAirframeRev.Value < 11)
			{
				if (Mathf.Abs(cFuelRate.Value - 40f) < 0.01f)
				{
					cFuelRate.Value = 80f;
				}
				if (Mathf.Abs(cFuelTrimRate.Value - 120f) < 0.01f)
				{
					cFuelTrimRate.Value = 240f;
				}
				Log.LogInfo("Ifrit 0.4.1 defaults: FuelTransferRate 80, FuelTrimRate 240.");
				cAirframeRev.Value = 11;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.2: the damaged-engine cap 30 -> 40; files still on 30 are moved once
			if (cAirframeRev.Value < 12)
			{
				if (Mathf.Abs(cEngDmgCap.Value - 30f) < 0.01f)
				{
					cEngDmgCap.Value = 40f;
				}
				Log.LogInfo("Ifrit 0.4.2 defaults: EngineDamagedAoACap 40.");
				cAirframeRev.Value = 12;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.4.5: throttle response back to the game's own (1.3 -> 1); files still on 1.3 are moved once
			if (cAirframeRev.Value < 13)
			{
				if (Mathf.Abs(cThrResp.Value - 1.3f) < 0.001f)
				{
					cThrResp.Value = 1f;
				}
				Log.LogInfo("Ifrit 0.4.5 defaults: ThrottleResponseScale 1.");
				cAirframeRev.Value = 13;
				if (devUnlocked)
				{
					dev.Save();
				}
			}
			// Ifrit 0.3.0: keys this build no longer reads (the F-22E-only settings, keys that moved section) are dropped from
			// both files instead of being carried as orphans
			DropOrphans(cfg, true);
			DropOrphans(dev, devUnlocked);
		}

		// true when the player file exists and was written with HostLockVariant's old default (false)
		// Revoker 0.2.0: whether a config file already has a key line
		private static bool FileHasKey(string path, string key)
		{
			try
			{
				if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
				{
					return false;
				}
				foreach (string l in System.IO.File.ReadAllLines(path))
				{
					if (l.TrimStart().StartsWith(key + " ") || l.TrimStart().StartsWith(key + "="))
					{
						return true;
					}
				}
			}
			catch (System.Exception)
			{
			}
			return false;
		}

		private static bool HostLockOldDefault(string path)
		{
			try
			{
				if (string.IsNullOrEmpty(path) || !System.IO.File.Exists(path))
				{
					return false;
				}
				string[] lines = System.IO.File.ReadAllLines(path);
				for (int i = 0; i < lines.Length; i++)
				{
					if (lines[i].TrimStart().StartsWith("HostLockVariant"))
					{
						for (int k = i - 1; k >= 0 && k >= i - 4; k--)
						{
							string l = lines[k].Trim();
							if (l.StartsWith("# Default value:"))
							{
								return l.EndsWith("false", System.StringComparison.OrdinalIgnoreCase);
							}
						}
						return false;
					}
				}
			}
			catch (System.Exception)
			{
			}
			return false;
		}

		public static bool DevUnlocked { get; private set; }

		private static void DropOrphans(ConfigFile f, bool save)
		{
			try
			{
				var prop = typeof(ConfigFile).GetProperty("OrphanedEntries", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
				if (f == null || !(prop?.GetValue(f) is Dictionary<ConfigDefinition, string> orph) || orph.Count == 0)
				{
					return;
				}
				int n = orph.Count;
				orph.Clear();
				if (save)
				{
					f.Save();
				}
				Log.LogInfo($"Revoker FCS: dropped {n} setting(s) this build no longer uses from {System.IO.Path.GetFileName(f.ConfigFilePath)}.");
			}
			catch (Exception ex)
			{
				Log.LogWarning("Revoker FCS: could not tidy " + ((f != null) ? System.IO.Path.GetFileName(f.ConfigFilePath) : "config") + ": " + ex.Message);
			}
		}

		// 1.8.44: the config was split into the player file (this plugin's) and the dev/debug file. Keys left in the player
		// file from older builds are orphans: the ones that moved section inside the player file (AI toggles) and, when the
		// dev file is being created now, the dev settings are carried over by key name once; then the orphans are dropped.
		private static void MigrateOldConfig(ConfigFile cfg, ConfigFile dev, bool toDev)
		{
			try
			{
				var prop = typeof(ConfigFile).GetProperty("OrphanedEntries", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
				if (!(prop?.GetValue(cfg) is Dictionary<ConfigDefinition, string> orph) || orph.Count == 0)
				{
					return;
				}
				var byKey = new Dictionary<string, string>();
				foreach (KeyValuePair<ConfigDefinition, string> o in orph)
				{
					if (o.Key.Section.StartsWith("Countermeasures (all") || !byKey.ContainsKey(o.Key.Key))
					{
						byKey[o.Key.Key] = o.Value;
					}
				}
				int n = 0;
				void Carry(ConfigFile f, bool skipStockCm)
				{
					foreach (ConfigDefinition def in new List<ConfigDefinition>(f.Keys))
					{
						if (skipStockCm && def.Section.StartsWith("3. Countermeasures"))
						{
							continue;
						}
						if (!byKey.TryGetValue(def.Key, out string v))
						{
							continue;
						}
						ConfigEntryBase e = f[def];
						try
						{
							e.BoxedValue = TomlTypeConverter.ConvertToValue(v, e.SettingType);
							n++;
						}
						catch
						{
						}
					}
				}
				Carry(cfg, skipStockCm: true);
				if (toDev)
				{
					Carry(dev, skipStockCm: false);
					dev.Save();
				}
				orph.Clear();
				cfg.Save();
				Log.LogInfo($"Revoker FCS: old config keys migrated ({n} values carried over{(toDev ? ", dev file created" : "")}, leftovers removed).");
			}
			catch (Exception ex)
			{
				Log.LogWarning("Revoker FCS config migration: " + ex.Message);
			}
		}

		public static void ApplyPatches(Harmony h)
		{
			MpPolicy.Install(h, (string what, Action a) => TryPatch(h, what, a));
			TryPatch(h, "ControlsFilter.Filter", delegate
			{
				h.Patch(AccessTools.Method(typeof(ControlsFilter), "Filter"), new HarmonyMethod(typeof(Hooks), "FilterPrefix"));
			});
			TryPatch(h, "AutopilotPlane.AutoAim", delegate
			{
				h.Patch(AccessTools.Method(typeof(AutopilotPlane), "AutoAim", new Type[9]
				{
					typeof(GlobalPosition),
					typeof(bool),
					typeof(bool),
					typeof(bool),
					typeof(float),
					typeof(float),
					typeof(bool),
					typeof(float),
					typeof(Vector3)
				}), new HarmonyMethod(typeof(Hooks), "AutoAimPrefix"), new HarmonyMethod(typeof(Hooks), "AutoAimPostfix"));
			});
			TryPatch(h, "ControlSurface.UpdateJobFields", delegate
			{
				h.Patch(AccessTools.Method(typeof(ControlSurface), "UpdateJobFields"), null, new HarmonyMethod(typeof(Hooks), "SurfacePostfix"));
			});
			TryPatch(h, "PowerSupply.ModifyCapacitance", delegate
			{
				h.Patch(AccessTools.Method(typeof(PowerSupply), "ModifyCapacitance"), new HarmonyMethod(typeof(Countermeasures), "ModifyCapacitancePrefix"));
			});
			TryPatch(h, "Aircraft.UseFuel", delegate
			{
				h.Patch(AccessTools.Method(typeof(Aircraft), "UseFuel", new Type[1] { typeof(float) }), new HarmonyMethod(typeof(FuelBalance), "UseFuelPrefix"));
			});
			TryPatch(h, "Aircraft.GetFuelLevel", delegate
			{
				h.Patch(AccessTools.Method(typeof(Aircraft), "GetFuelLevel"), null, new HarmonyMethod(typeof(FuelBalance), "GetFuelLevelPostfix"));
			});
			TryPatch(h, "Aircraft.GetFuelQuantity", delegate
			{
				h.Patch(AccessTools.Method(typeof(Aircraft), "GetFuelQuantity"), null, new HarmonyMethod(typeof(FuelBalance), "GetFuelQuantityPostfix"));
			});
			TryPatch(h, "FuelTank.Refuel", delegate
			{
				h.Patch(AccessTools.Method(typeof(FuelTank), "Refuel"), new HarmonyMethod(typeof(FuelBalance), "RefuelPrefix"));
			});
			TryPatch(h, "Hardpoint.ModifyMass", delegate
			{
				h.Patch(AccessTools.Method(typeof(Hardpoint), "ModifyMass"), null, new HarmonyMethod(typeof(FuelBalance), "HardpointMassPostfix"));
			});
			TryPatch(h, "HighLiftDevice.FixedUpdate", delegate
			{
				h.Patch(AccessTools.Method(typeof(HighLiftDevice), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "FlapDevicePrefix"));
			});
			TryPatch(h, "LandingGear.FixedUpdate", delegate
			{
				h.Patch(AccessTools.Method(typeof(LandingGear), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "GearPrefix"), new HarmonyMethod(typeof(Hooks), "GearPostfix"));
			});
			TryPatch(h, "Turbojet.FixedUpdate", delegate
			{
				// Revoker 0.1.0: after the KR-67 Ifrit FCS's transpiler on the same method (it chains onto its calls; see Hooks)
				HarmonyMethod tr = new HarmonyMethod(typeof(Hooks), "TurbojetTranspiler");
				tr.priority = Priority.Last;
				tr.after = new string[1] { Hooks.IfritHarmonyId };
				h.Patch(AccessTools.Method(typeof(Turbojet), "FixedUpdate"), new HarmonyMethod(typeof(Hooks), "TurbojetPrefix"), null, tr);
			});
			// Revoker 0.1.0: the Revoker's dedicated airbrake (game Airbrake component, opened by the game at zero throttle)
			// is commanded by the FCS's brake logic instead (F-22E 2.2.6 / 2.2.16 hooks)
			TryPatch(h, "Airbrake.Update", delegate
			{
				h.Patch(AccessTools.Method(typeof(Airbrake), "Update"), new HarmonyMethod(typeof(Hooks), "AirbrakeUpdatePrefix"), new HarmonyMethod(typeof(Hooks), "AirbrakeUpdatePostfix"));
			});
			Log.LogInfo($"FS-12 Revoker FCS {FcsStandalonePlugin.Version} (yaw TVC {(YawTvc ? "on" : "off")}, elevon pitch {(ElevonPitch ? "on" : "off")}; {(StockPerformance ? "stock" : "buffed")} performance{(DevUnlocked ? ", dev/debug settings unlocked" : "")}) ready (AoA +{AoAPos}/{AoANeg}, G +{GPos}/{GNeg}, MPO +{MpoGPos}/{MpoGNeg}, " + string.Format("q<= {0}, p<= {1}, r<= {2}; asym-load AoA cap {3}; flaperon camber K {4:F2}, MPO separate axes {8}, actuators x{5}, thrust x{6}, parasitic drag x{7}, fuel burn x{9:F2}).", MaxPitchRate, MaxRollRate, MaxYawRate, AsymLimiter ? ("on, reserve " + AsymReserve.ToString("0.00")) : "off", AileronCamberK, ActuatorScale, ThrustScale, DragScale, MpoSeparateAxes, FuelUseScale));
		}

		private static void TryPatch(Harmony h, string what, Action a)
		{
			try
			{
				a();
				Log.LogInfo("Installed hook: " + what);
			}
			catch (Exception ex)
			{
				Log.LogError("Failed to install hook " + what + ": " + ex);
			}
		}
	}
}
