using System.Collections.Generic;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// Stock-jet port of the F-22E 3.0.0 capacity estimator (F-22E 2.7.5 - 2.22.0) - TELEMETRY ONLY, drives nothing.
	//
	// What each axis can still do toward a side, in deg/s^2, as three parts that are each either measured or the allocator's
	// own decision:
	//   1. what the axis is doing now: the filtered measured angular acceleration (passive airframe + effectors at `now`);
	//   2. what is already committed and still arriving within CapHorizon: the table increment from `now` to this tick's command,
	//      per effector group, counted only as far as the servo gets there within the horizon;
	//   3. what the allocator would still give: a probe - the allocator solved once more from this tick's commands for a large
	//      extra demand toward the side, with the rest of its target unchanged - read per group, then the commands restored.
	//
	// What differs from the F-22E file:
	//  - EVERY effector is in a group. The F-22E's Group() returned -1 for anything that is not a SurfaceEffector (only its
	//    pitch-only TVC there); on the stock jets that dropped real lateral effectors from the committed and probe terms: the
	//    Ifrit's roll TVC and splits, the Revoker's yaw TVC, inboard elevons and splits. Six groups: rudders, stabilators (the
	//    Ifrit's canted tails, the Revoker's canards), the other surfaces (ailerons / flaperons / elevons), TVC of any kind,
	//    split decelerons, hinged surfaces.
	//  - every group gain is 1 until the per-jet gains are fitted from a flight (the F-22E's rudder 1.0 / stab 0.7 / flaperon 1.0
	//    were identified on the F-22E's logs and are not this airframe's);
	//  - a sideslip-axis read (capBP/BN): the acceleration along b = (sin a, -cos a) in (roll, yaw), i.e. the effector part of
	//    d2(beta)/dt2 ~ p' sin a - r' cos a. Above ~40 deg a body-roll moment is mostly a sideslip moment, so the roll surfaces,
	//    roll TVC and the tails'/canards' differential are sideslip effectors whatever they are called. Measured + committed +
	//    a probe along b, less the allocator's unmet along b;
	//  - rudW (the weight of the yaw-only read against the yaw component of the coordinated rotation) is derived from the
	//    yaw-only probes: the share of the yaw the allocator can add without spending roll, 1 - |roll collateral| / |yaw added|.
	//    The F-22E's rudder fade (rudderAuth x fin health) is logged beside it (ceRudWFade) for comparison only. Neither drives
	//    anything in this build.
	internal sealed class CapacityEstimator
	{
		internal static bool CapEstOn = true;

		// it drives nothing yet, so in the game it runs only while the FCS flight log is being written (8 extra allocator solves a
		// tick, ~0.3-0.65 ms per tick on the bench); the bench sync sets this false so the bench always runs it
		internal static bool OnlyWhileLogging = false;   // 0.6.0: the governor reads it - always runs

		internal static float ProbeZ = 400f;      // deg/s^2 of extra yaw asked of the allocator in the probe

		internal static float ProbeX = 1500f;     // deg/s^2 of extra roll

		internal static float ProbeS = 800f;      // deg/s^2 of extra velocity-roll acceleration (two-axis probe along (cos a, sin a))

		internal static float ProbeB = 800f;      // deg/s^2 of extra sideslip-axis acceleration (probe along (sin a, -cos a))

		internal static int ProbeIters = 1;       // allocator solves per probe (warm-started from this tick's command)

		internal static float CapHorizon = 0.15f; // s, how far ahead the committed increment is counted

		internal static float TradeCapTau = 0.05f; // s, the trade's low-pass of capZeP/N (F-22E 2.21.0)

		internal static float RudWFloor = 5f;     // deg/s^2: below this much yaw added a side's rudW reads 0

		internal const int NG = 6;                // r, s, f, t, p, h

		internal static readonly string[] GroupTag = { "r", "s", "f", "t", "p", "h" };

		// per-group gains, yaw and roll (r, s, f, t, p, h): 1 until fitted per jet
		internal static readonly float[] GainZ = { 1f, 1f, 1f, 1f, 1f, 1f };

		internal static readonly float[] GainX = { 1f, 1f, 1f, 1f, 1f, 1f };

		internal static float GainTau = 10f;      // s, RLS forgetting (lumped online gain, telemetry)

		internal static float GainRidge = 0.002f;

		internal static float GainLo = 0.2f;

		internal static float GainHi = 1.6f;

		internal static float GainExcite = 3f;

		internal static int DiffTicks = 6;

		// telemetry (deg/s^2 unless noted); Z = yaw, X = roll, B = sideslip axis; P/N = toward the positive / negative side
		public float wdZ, wdX, wdB;
		public readonly float[] cmZ = new float[NG], cmX = new float[NG], disc = new float[NG];
		public readonly float[] prZP = new float[NG], prZN = new float[NG], prXP = new float[NG], prXN = new float[NG];
		public readonly float[] prSP = new float[NG], prSN = new float[NG], prBP = new float[NG], prBN = new float[NG];
		public float prZPx, prZNx, prXPz, prXNz;
		public float gZ = 1f, gX = 1f;
		public float capZP, capZN, capXP, capXN, capSP, capSN, capBP, capBN;
		public float capZPraw, capZNraw, capXPraw, capXNraw;     // every gain at 1, committed term undiscounted (the push governor's roll read)
		public float capZeP, capZeN, rudWDbg, rudWFadeDbg, yawPureP, yawPureN;
		public float capTrZeP, capTrZeN, capZeNsP, capZeNsN, wdZNs;
		public float unX, unZ;
		public int gUpdZ, gUpdX;
		private bool trInit;

		private float thZ = 1f, thX = 1f, PZ = 1f, PX = 1f;
		private readonly float[] ringMZ, ringMX, ringWdZ, ringWdX;
		private int ringN, ringHead;
		private float[] savedCmd = new float[32];
		private readonly float[] cmZd = new float[NG], cmXd = new float[NG], trav = new float[NG], travIn = new float[NG];
		private readonly float[] tA = new float[NG], tB = new float[NG], tC = new float[NG];

		public CapacityEstimator()
		{
			int n = Mathf.Max(2, DiffTicks);
			ringMZ = new float[n]; ringMX = new float[n]; ringWdZ = new float[n]; ringWdX = new float[n];
		}

		internal static int Group(Effector e)
		{
			if (e is SurfaceEffector s)
			{
				if (s.kind == SurfaceKind.Rudder) return 0;
				if (s.kind == SurfaceKind.Stabilator) return 1;
				return 2;
			}
			string n = e.GetType().Name;
			if (n.StartsWith("Tvc")) return 3;
			if (n == "SplitEffector") return 4;
			return 5;   // hinged surfaces (HingeFlapEffector) and anything else: counted, never dropped
		}

		// Called once per tick right after the allocation, with the effectors' cmd = this tick's decision and Mnow at `now`.
		// wdot: filtered measured angular acceleration (rad/s^2, aero frame x roll / y pitch / z yaw). unmet: the allocator's
		// unmet report per axis (deg/s^2, signed toward the side the demand went). rudWFade: the F-22E's rudder-fade weight
		// (telemetry only).
		public void Step(List<Effector> effs, Allocator alloc, Vector3 wdot, float[,] IaInv, Vector3 target, float dt, Vector3 unmet, float alphaDeg, float rudWFade)
		{
			if (!CapEstOn) return;
			float ix = IaInv[0, 0] * 57.29578f, iz = IaInv[2, 2] * 57.29578f;
			float ca = Mathf.Cos(alphaDeg * (3.14159274f / 180f)), sa = Mathf.Sin(alphaDeg * (3.14159274f / 180f));
			wdZ = wdot.z * 57.29578f;
			wdX = wdot.x * 57.29578f;
			wdB = wdX * sa - wdZ * ca;
			unX = unmet.x; unZ = unmet.z;
			for (int g = 0; g < NG; g++) { cmZ[g] = cmX[g] = cmZd[g] = cmXd[g] = trav[g] = travIn[g] = 0f; }
			float mNowZ = 0f, mNowX = 0f, mNowZs = 0f;
			for (int i = 0; i < effs.Count; i++)
			{
				Effector e = effs[i];
				if (!e.active) continue;
				int g = Group(e);
				Vector3 mn = e.Mnow, mc = e.MAt(e.cmd);
				mNowZ += mn.z * iz; mNowX += mn.x * ix;
				if (g == 1) mNowZs += mn.z * iz;
				float dz = (mc.z - mn.z) * iz, dx = (mc.x - mn.x) * ix;
				cmZ[g] += dz; cmX[g] += dx;
				float tr = Mathf.Abs(e.cmd - e.now);
				float share = (tr > 1E-03f) ? Mathf.Clamp01(Mathf.Max(1f, e.rate) * CapHorizon / tr) : 1f;
				cmZd[g] += dz * share; cmXd[g] += dx * share;
				trav[g] += tr; travIn[g] += tr * share;
			}
			for (int g = 0; g < NG; g++) disc[g] = (trav[g] > 1E-03f) ? travIn[g] / trav[g] : 1f;
			// ---- lumped online gains (telemetry)
			gUpdZ = Rls(ref thZ, ref PZ, ringMZ, ringWdZ, mNowZ, wdZ, dt) ? 1 : 0;
			gUpdX = Rls(ref thX, ref PX, ringMX, ringWdX, mNowX, wdX, dt) ? 1 : 0;
			int nR = ringMZ.Length;
			ringHead = (ringHead + 1) % nR;
			ringMZ[ringHead] = mNowZ; ringMX[ringHead] = mNowX; ringWdZ[ringHead] = wdZ; ringWdX[ringHead] = wdX;
			if (ringN < nR) ringN++;
			gZ = thZ; gX = thX;
			// ---- allocator probes from this tick's commands (restored afterwards)
			if (savedCmd.Length < effs.Count) savedCmd = new float[effs.Count + 8];
			int cnt = effs.Count;
			for (int i = 0; i < cnt; i++) savedCmd[i] = effs[i].cmd;
			float colX, colZ;
			// yaw only
			Probe(effs, alloc, target, new Vector3(0f, 0f, ProbeZ / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prZP[g] = tB[g];
			prZPx = colX;
			float addZP = Sum(tB);
			float colZP = colX;
			Probe(effs, alloc, target, new Vector3(0f, 0f, -ProbeZ / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prZN[g] = -tB[g];
			prZNx = colX;
			float addZN = -Sum(tB);
			float colZN = colX;
			// roll only
			Probe(effs, alloc, target, new Vector3(ProbeX / ix, 0f, 0f), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prXP[g] = tA[g];
			prXPz = colZ;
			Probe(effs, alloc, target, new Vector3(-ProbeX / ix, 0f, 0f), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prXN[g] = -tA[g];
			prXNz = colZ;
			// the coordinated rotation, along u = (cos a, sin a): read on the yaw axis (the F-22E 2.16.0 read)
			float sgZ = (sa >= 0f) ? 1f : -1f;
			Probe(effs, alloc, target, new Vector3(ProbeS * ca / ix, 0f, ProbeS * sa / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) { if (sa >= 0f) prSP[g] = sgZ * tB[g]; else prSN[g] = -sgZ * tB[g]; }
			Probe(effs, alloc, target, new Vector3(-ProbeS * ca / ix, 0f, -ProbeS * sa / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) { if (sa >= 0f) prSN[g] = -sgZ * tB[g]; else prSP[g] = sgZ * tB[g]; }
			// the sideslip axis, along b = (sin a, -cos a): read along b
			Probe(effs, alloc, target, new Vector3(ProbeB * sa / ix, 0f, -ProbeB * ca / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prBP[g] = tC[g];
			Probe(effs, alloc, target, new Vector3(-ProbeB * sa / ix, 0f, ProbeB * ca / iz), ix, iz, ca, sa, tA, tB, tC, out colX, out colZ, cnt);
			for (int g = 0; g < NG; g++) prBN[g] = -tC[g];
			for (int i = 0; i < cnt; i++) effs[i].cmd = savedCmd[i];
			// ---- composites (positive magnitudes; negative = the axis cannot even hold this side)
			float uZP = Mathf.Max(0f, unmet.z), uZN = Mathf.Max(0f, -unmet.z), uXP = Mathf.Max(0f, unmet.x), uXN = Mathf.Max(0f, -unmet.x);
			float uB = unmet.x * sa - unmet.z * ca;
			float uBP = Mathf.Max(0f, uB), uBN = Mathf.Max(0f, -uB);
			float comZ = 0f, comX = 0f, comZNs = 0f;
			float pZP = 0f, pZN = 0f, pXP = 0f, pXN = 0f, pSP = 0f, pSN = 0f, pBP = 0f, pBN = 0f;
			float pZPNs = 0f, pZNNs = 0f, pSPNs = 0f, pSNNs = 0f;
			for (int g = 0; g < NG; g++)
			{
				comZ += GainZ[g] * cmZd[g]; comX += GainX[g] * cmXd[g];
				pZP += GainZ[g] * prZP[g]; pZN += GainZ[g] * prZN[g];
				pXP += GainX[g] * prXP[g]; pXN += GainX[g] * prXN[g];
				pSP += GainZ[g] * prSP[g]; pSN += GainZ[g] * prSN[g];
				pBP += prBP[g]; pBN += prBN[g];   // prB* already carry the per-axis gains (see Probe)
				if (g != 1)
				{
					comZNs += GainZ[g] * cmZd[g];
					pZPNs += GainZ[g] * prZP[g]; pZNNs += GainZ[g] * prZN[g];
					pSPNs += GainZ[g] * prSP[g]; pSNNs += GainZ[g] * prSN[g];
				}
			}
			float comB = comX * sa - comZ * ca;
			capZP = wdZ + comZ + Mathf.Max(0f, pZP - uZP);
			capZN = -wdZ - comZ + Mathf.Max(0f, pZN - uZN);
			capXP = wdX + comX + Mathf.Max(0f, pXP - uXP);
			capXN = -wdX - comX + Mathf.Max(0f, pXN - uXN);
			capSP = wdZ + comZ + Mathf.Max(0f, pSP - uZP);
			capSN = -wdZ - comZ + Mathf.Max(0f, pSN - uZN);
			capBP = wdB + comB + Mathf.Max(0f, pBP - uBP);
			capBN = -wdB - comB + Mathf.Max(0f, pBN - uBN);
			float comZ1 = 0f, comX1 = 0f, pZPr = 0f, pZNr = 0f, pXPr = 0f, pXNr = 0f;
			for (int g = 0; g < NG; g++) { comZ1 += cmZ[g]; comX1 += cmX[g]; pZPr += prZP[g]; pZNr += prZN[g]; pXPr += prXP[g]; pXNr += prXN[g]; }
			capZPraw = wdZ + comZ1 + pZPr;
			capZNraw = -wdZ - comZ1 + pZNr;
			capXPraw = wdX + comX1 + pXPr;
			capXNraw = -wdX - comX1 + pXNr;
			// rudW from the probes: the yaw the allocator can add without spending roll. The yaw-only probe asks for far more yaw
			// than the jet has and the allocator trades roll for it (Ifrit bench, 300 km/h: +39 yaw for -333 deg/s^2 of roll at 10 deg
			// AoA), so its own collateral is not the answer. The roll probe on the opposite side is added in the share that cancels
			// the yaw probe's roll collateral (at most all of it), and the yaw that is left is the roll-neutral yaw; rudW per side is
			// that over the yaw-only probe's yaw (scaled down when even the whole roll probe cannot cancel the collateral).
			float addXP = Sum(prXP), addXN = Sum(prXN);
			yawPureP = RollNeutralYaw(addZP, colZP, addXP, prXPz, addXN, prXNz);
			yawPureN = RollNeutralYaw(addZN, colZN, addXP, -prXPz, addXN, -prXNz);
			float wP = (addZP > RudWFloor) ? Mathf.Clamp01(yawPureP / addZP) : 0f;
			float wN = (addZN > RudWFloor) ? Mathf.Clamp01(yawPureN / addZN) : 0f;
			rudWDbg = 0.5f * (wP + wN);
			rudWFadeDbg = Mathf.Clamp01(rudWFade);
			capZeP = Mathf.Lerp(capSP, capZP, rudWDbg);
			capZeN = Mathf.Lerp(capSN, capZN, rudWDbg);
			// without the stabilators (F-22E 2.22.0)
			float sMeas = GainZ[1] * mNowZs;
			wdZNs = wdZ - sMeas;
			float zPNs = wdZNs + comZNs + Mathf.Max(0f, pZPNs - uZP);
			float zNNs = -wdZNs - comZNs + Mathf.Max(0f, pZNNs - uZN);
			float sPNs = wdZNs + comZNs + Mathf.Max(0f, pSPNs - uZP);
			float sNNs = -wdZNs - comZNs + Mathf.Max(0f, pSNNs - uZN);
			capZeNsP = Mathf.Lerp(sPNs, zPNs, rudWDbg);
			capZeNsN = Mathf.Lerp(sNNs, zNNs, rudWDbg);
			// the trade's read (F-22E 2.21.0)
			if (!trInit) { capTrZeP = capZeP; capTrZeN = capZeN; trInit = true; }
			float kTr = dt / (Mathf.Max(0f, TradeCapTau) + dt);
			capTrZeP += (capZeP - capTrZeP) * kTr;
			capTrZeN += (capZeN - capTrZeN) * kTr;
		}

		// addZ: the yaw the yaw probe added toward its side (>0); colRoll: its roll collateral; the roll probes' roll added (>0
		// toward + / -) and their yaw collateral, all signed so that + yaw is the side being read
		private static float RollNeutralYaw(float addZ, float colRoll, float addXP, float colZXP, float addXN, float colZXN)
		{
			if (addZ <= 0f)
			{
				return 0f;
			}
			// cancel a negative roll collateral with the + roll probe, a positive one with the - roll probe
			float rollAdd = (colRoll < 0f) ? addXP : addXN;
			float yawCol = (colRoll < 0f) ? colZXP : colZXN;
			if (rollAdd <= 1f)
			{
				return 0f;
			}
			float need = Mathf.Abs(colRoll) / rollAdd;
			float b = Mathf.Min(1f, need);
			float y = addZ + b * yawCol;
			if (need > 1f)
			{
				y *= 1f / need;   // the roll that is left uncancelled: only that share of the yaw comes roll-neutral
			}
			return Mathf.Max(0f, y);
		}

		private static float Sum(float[] a)
		{
			float s = 0f;
			for (int i = 0; i < a.Length; i++) s += a[i];
			return s;
		}

		// one probe: re-solve from this tick's commands for target + dT; per group the roll (outX), yaw (outZ) and sideslip-axis
		// (outB, with the per-axis gains) increments, plus the total roll / yaw collateral
		private void Probe(List<Effector> effs, Allocator alloc, Vector3 target, Vector3 dT, float ix, float iz, float ca, float sa, float[] outX, float[] outZ, float[] outB, out float colX, out float colZ, int cnt)
		{
			for (int i = 0; i < cnt; i++) effs[i].cmd = savedCmd[i];
			for (int it = 0; it < ProbeIters; it++) alloc.Solve(effs, target + dT);
			for (int g = 0; g < NG; g++) outX[g] = outZ[g] = outB[g] = 0f;
			colX = colZ = 0f;
			for (int i = 0; i < cnt; i++)
			{
				Effector e = effs[i];
				if (!e.active) continue;
				Vector3 d = e.MAt(e.cmd) - e.MAt(savedCmd[i]);
				float dx = d.x * ix, dz = d.z * iz;
				colX += dx; colZ += dz;
				int g = Group(e);
				outX[g] += dx; outZ[g] += dz;
				outB[g] += GainX[g] * dx * sa - GainZ[g] * dz * ca;
			}
		}

		private bool Rls(ref float th, ref float P, float[] ringM, float[] ringWd, float mNow, float wd, float dt)
		{
			int n = ringM.Length;
			if (ringN < n) return false;
			int old = (ringHead + 1) % n;
			float x = mNow - ringM[old];
			float y = wd - ringWd[old];
			if (Mathf.Abs(x) < GainExcite) return false;
			float lam = Mathf.Exp(-dt / Mathf.Max(0.1f, GainTau));
			float px = P * x;
			float den = lam + x * px;
			if (den < 1E-06f) return false;
			float k = px / den;
			th += k * (y - x * th);
			P = Mathf.Min(5f, (P - k * px) / lam);
			th += GainRidge * (1f - th);
			th = Mathf.Clamp(th, GainLo, GainHi);
			return true;
		}

		// telemetry header and row (appended to the FCS CSV)
		internal static string Header()
		{
			var sb = new System.Text.StringBuilder(",ceWdZ,ceWdX,ceWdB,ceUnX,ceUnZ");
			string[] blocks = { "ceCmZ", "ceCmX", "ceD", "cePrZP", "cePrZN", "cePrXP", "cePrXN", "cePrSP", "cePrSN", "cePrBP", "cePrBN" };
			foreach (string b in blocks) foreach (string t in GroupTag) sb.Append(',').Append(b).Append(t);
			sb.Append(",cePrZPx,cePrZNx,cePrXPz,cePrXNz,ceGZ,ceGX,ceCapZP,ceCapZN,ceCapXP,ceCapXN,ceCapSP,ceCapSN,ceCapBP,ceCapBN,ceCapZeP,ceCapZeN,ceRudW,ceRudWFade,ceYawPureP,ceYawPureN,ceCapTrP,ceCapTrN,ceCapNsP,ceCapNsN,ceWdNs,ceGUpdZ,ceGUpdX");
			return sb.ToString();
		}

		internal void Row(System.IO.TextWriter w, System.IFormatProvider ci)
		{
			w.Write(string.Format(ci, ",{0:F1},{1:F1},{2:F1},{3:F1},{4:F1}", wdZ, wdX, wdB, unX, unZ));
			float[][] blocks = { cmZ, cmX, disc, prZP, prZN, prXP, prXN, prSP, prSN, prBP, prBN };
			for (int b = 0; b < blocks.Length; b++)
			{
				float[] a = blocks[b];
				for (int g = 0; g < NG; g++) w.Write(string.Format(ci, (b == 2) ? ",{0:F2}" : ",{0:F1}", a[g]));
			}
			w.Write(string.Format(ci, ",{0:F1},{1:F1},{2:F1},{3:F1},{4:F3},{5:F3}", prZPx, prZNx, prXPz, prXNz, gZ, gX));
			w.Write(string.Format(ci, ",{0:F1},{1:F1},{2:F1},{3:F1},{4:F1},{5:F1},{6:F1},{7:F1}", capZP, capZN, capXP, capXN, capSP, capSN, capBP, capBN));
			w.Write(string.Format(ci, ",{0:F1},{1:F1},{2:F2},{3:F2},{4:F1},{5:F1},{6:F1},{7:F1},{8:F1},{9:F1},{10:F1},{11},{12}", capZeP, capZeN, rudWDbg, rudWFadeDbg, yawPureP, yawPureN, capTrZeP, capTrZeN, capZeNsP, capZeNsN, wdZNs, gUpdZ, gUpdX));
		}
	}
}
