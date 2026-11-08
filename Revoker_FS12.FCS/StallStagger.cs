using System;
using System.Collections.Generic;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.0.0: staggered stall of the horizontal control surfaces.
	// Aryx gives every horizontal surface the same airfoil (F22E_Wing, lift peak at +/-35 deg local AoA). Each
	// family (stabilators, inner flaperons Flap_*, outer flaperons Aileron_*) gets its own copy of that curve with the
	// peak moved to its own stall AoA. Shape-preserving remap, symmetric on the negative side:
	//   |x| <= P:       CL'(x) = A * CL(x * P0 / P)                  (A = (P/P0)^LiftSlope)
	//   P < |x| <= 90:  CL'(x) = A(x) * CL(P0 + (|x|-P)(90-P0)/(90-P)) (A fades back to 1 at 90)
	//   |x| > 90:       unchanged (reversed flow)
	// LiftSlope 1 keeps the small-angle lift slope (the peak CL scales with the stall AoA, like a real wing whose
	// stall moves); 0 keeps the peak CL (the slope scales instead). Drag follows the same angle map (drag rise moves
	// with the stall), without the amplitude factor.
	// Shared by the plugin and the bench: the game's aero job reads the remapped charts (StallStaggerJobs), the FCS's
	// effector and body models read the same 128-point tables through TableFor.
	internal static class StallStagger
	{
		public enum Fam
		{
			None = -1,
			Stab = 0,
			Inner = 1,
			Outer = 2,
			BodyFore = 3,
			BodyAft = 4
		}

		public const int ChartN = 128;

		public const float ChartStep = 0.04908734f;

		// family of a part whose chart is replaced: a staggered control surface (with StallStagger) or a 2.0.8 body
		// section (with BodyLiftShaping), else None
		public static Fam FamilyOf(string name)
		{
			Fam f = SurfaceFamilyOf(name);
			if (f != Fam.None)
			{
				return Plugin.StallStagger ? f : Fam.None;
			}
			return Plugin.BodyLiftShaping ? BodyLift.FamilyOf(name) : Fam.None;
		}

		public static Fam SurfaceFamilyOf(string name)
		{
			if (string.IsNullOrEmpty(name))
			{
				return Fam.None;
			}
			// Revoker 0.2.20: the FS-12's trailing-edge elevon parts (lower case): aileron_L/R = outboard, flap_L/R = inboard.
			// The canards keep their own foil (no family).
			if (name.StartsWith("aileron_") && !name.Contains("visible"))
			{
				return Fam.Outer;
			}
			if (name.StartsWith("flap_") && !name.Contains("visible"))
			{
				return Fam.Inner;
			}
			if (name.Contains("Elevator"))
			{
				return Fam.Stab;
			}
			if (name.Contains("Aileron"))
			{
				return Fam.Outer;
			}
			if (name.Contains("Flap") && !name.Contains("Servo") && !name.Contains("Drive"))
			{
				return Fam.Inner;
			}
			return Fam.None;
		}

		public static bool Enabled => Plugin.StallStagger || Plugin.BodyLiftShaping;

		public static float PeakFor(Fam f)
		{
			switch (f)
			{
			case Fam.Stab:
				return Plugin.StabStallAoA;
			case Fam.Inner:
				return Plugin.InnerFlaperonStallAoA;
			case Fam.Outer:
				return Plugin.OuterFlaperonStallAoA;
			default:
				return 0f;
			}
		}

		// cache key: one remapped chart per (source airfoil, peak, slope mode)
		public static string Key(object af, Fam f)
		{
			if (f == Fam.BodyFore || f == Fam.BodyAft)
			{
				return (af == null ? 0 : af.GetHashCode()) + "|" + (int)f + "|" + BodyLift.KeyPart(f);
			}
			return (af == null ? 0 : af.GetHashCode()) + "|" + (int)f + "|" + PeakFor(f).ToString("F2") + "|knee";
		}

		// peak AoA (deg, magnitude) of a lift curve on one side (sign +1 / -1), searched 2..89 deg
		public static float FindPeakDeg(Func<float, float> clOfRad, int sign)
		{
			float best = 0f, bestA = 35f;
			for (float a = 2f; a <= 89f; a += 0.25f)
			{
				float c = sign * clOfRad(sign * a * Mathf.Deg2Rad);
				if (c > best)
				{
					best = c;
					bestA = a;
				}
			}
			return bestA;
		}

		// Revoker 0.2.22: the knee of a lift curve on one side (deg, magnitude): the first local minimum of the lift slope after its
		// steepest point, below the peak, if that minimum is under 25 % of the steepest slope (Fighter1_wing: the 17.19 deg keyframe,
		// slope 0.15 against 3.8 /rad, recovering to ~0.4 above it); else where the slope first falls under 15 % of it; 0 if neither
		// (no knee: the whole curve below the peak is compressed).
		public static float FindKneeDeg(Func<float, float> clOfRad, int sign, float peakDeg)
		{
			const float h = 0.25f;
			int n = Mathf.FloorToInt((peakDeg - 0.5f) / h);
			if (n < 4)
			{
				return 0f;
			}
			float[] sl = new float[n];
			for (int i = 0; i < n; i++)
			{
				float a = 0.5f + i * h;
				sl[i] = sign * (clOfRad(sign * (a + h) * Mathf.Deg2Rad) - clOfRad(sign * (a - h) * Mathf.Deg2Rad)) / (2f * h);
			}
			int iMax = 0;
			for (int i = 1; i < n; i++)
			{
				if (sl[i] > sl[iMax])
				{
					iMax = i;
				}
			}
			float sMax = sl[iMax];
			if (sMax <= 0f)
			{
				return 0f;
			}
			for (int i = iMax + 1; i < n - 1; i++)
			{
				if (sl[i] <= sl[i - 1] && sl[i] < sl[i + 1] && sl[i] < 0.25f * sMax)
				{
					return 0.5f + i * h;
				}
			}
			for (int i = iMax + 1; i < n; i++)
			{
				if (sl[i] < 0.15f * sMax)
				{
					return 0.5f + i * h;
				}
			}
			return 0f;
		}

		// Revoker 0.2.22: knee-preserving angle map for one side. Below the source curve's knee the curve is untouched; the plateau
		// from the knee to the source peak P0 is compressed (or stretched) linearly onto knee..P; past P the post-peak side runs
		// from P0 to 90 as before. Lift amplitude 1 everywhere: the peak CL is kept and nothing below the knee changes. 0.2.20-0.2.21
		// compressed the whole curve (and with it the 17 deg knee, to 12-14 deg): roll authority -6..-30 % at 10 deg and ~-23 %
		// at 15 deg for the inboard pair, and a lift loss above the knee.
		public static void MapKnee(float x, float P, float P0, float knee, out float m)
		{
			P = Mathf.Clamp(P, 5f, 80f);
			float k = Mathf.Min(knee, Mathf.Min(P, P0) - 2f);
			if (k < 0f)
			{
				k = 0f;
			}
			if (x <= k)
			{
				m = x;
			}
			else if (x <= P)
			{
				m = k + (x - k) * (P0 - k) / Mathf.Max(0.01f, P - k);
			}
			else if (x <= 90f)
			{
				float t = (x - P) / (90f - P);
				m = P0 + t * (90f - P0);
			}
			else
			{
				m = x;
			}
		}

		// angle map for one side: x = |AoA| (deg), returns the source |AoA| (deg) and the lift amplitude
		public static void Map(float x, float P, float P0, float slope, out float m, out float amp)
		{
			P = Mathf.Clamp(P, 5f, 80f);
			float aP = Mathf.Pow(P / P0, Mathf.Clamp01(slope));
			if (x <= P)
			{
				m = x * P0 / P;
				amp = aP;
			}
			else if (x <= 90f)
			{
				float t = (x - P) / (90f - P);
				m = P0 + t * (90f - P0);
				// Revoker 0.2.20: the amplitude holds aP past the peak and blends back to 1 only over 80-90 deg. A linear fade from
				// the peak (the F-22E's form, harmless at LiftSlope 0 where aP = 1) outran the flat-topped wing foil's post-peak
				// lift drop at LiftSlope 1 and moved the peak: inboard 32 -> 39.4 deg on Fighter1_wing.
				float tf = Mathf.Clamp01((x - 80f) / 10f);
				tf = tf * tf * (3f - 2f * tf);
				amp = aP + (1f - aP) * tf;
			}
			else
			{
				m = x;
				amp = 1f;
			}
		}

		// fills 128-point lift and drag charts (the game's sampling: index i <-> (i - 64) * ChartStep rad)
		public static void BuildCharts(Func<float, float> clOfRad, Func<float, float> cdOfRad, float P, float slope, float[] cl, float[] cd)
		{
			float p0Pos = FindPeakDeg(clOfRad, 1);
			float p0Neg = FindPeakDeg(clOfRad, -1);
			float kPos = FindKneeDeg(clOfRad, 1, p0Pos);
			float kNeg = FindKneeDeg(clOfRad, -1, p0Neg);
			for (int i = 0; i < ChartN; i++)
			{
				float a = (i - 64) * ChartStep;
				float xd = Mathf.Abs(a) * Mathf.Rad2Deg;
				int s = (a < 0f) ? -1 : 1;
				MapKnee(xd, P, (s > 0) ? p0Pos : p0Neg, (s > 0) ? kPos : kNeg, out float m);
				float amp = 1f;
				float src = s * m * Mathf.Deg2Rad;
				if (xd > 90f)
				{
					src = a;
				}
				cl[i] = amp * clOfRad(src);
				cd[i] = cdOfRad(src);
			}
		}

		// the chart for a family: the stall remap for a control surface, the lift multiplier for a body section
		public static void BuildFor(Fam f, Func<float, float> clOfRad, Func<float, float> cdOfRad, float[] cl, float[] cd)
		{
			if (f == Fam.BodyFore || f == Fam.BodyAft)
			{
				BodyLift.BuildCharts(f, clOfRad, cdOfRad, cl, cd);
				return;
			}
			BuildCharts(clOfRad, cdOfRad, PeakFor(f), Plugin.StallStaggerLiftSlope, cl, cd);
		}

		private static readonly Dictionary<string, AirfoilTable> tables = new Dictionary<string, AirfoilTable>();

		// the FCS's table for an AeroPart: the remapped one for a staggered control surface, else the airfoil's own
		public static AirfoilTable TableFor(AeroPart part, Airfoil af)
		{
			Fam f = (part != null && Enabled && af != null && af.liftCoef != null && af.dragCoef != null) ? FamilyOf(part.gameObject.name) : Fam.None;
			return TableForFamily(af, f);
		}

		// Revoker 0.2.20: a family's remapped table for any airfoil. Used for the camber increment a trailing-edge elevon puts on
		// the wing section ahead of it: the increment is that elevon's circulation, so it stalls (and reverses) with the elevon,
		// while the section's own clean lift keeps its own curve.
		public static AirfoilTable TableForFamily(Airfoil af, Fam f)
		{
			if (f == Fam.None || af == null || af.liftCoef == null || af.dragCoef == null)
			{
				return new AirfoilTable(af);
			}
			string k = Key(af, f);
			if (!tables.TryGetValue(k, out AirfoilTable t))
			{
				float[] cl = new float[ChartN];
				float[] cd = new float[ChartN];
				BuildFor(f, r => af.liftCoef.Evaluate(r), r => af.dragCoef.Evaluate(r), cl, cd);
				t = new AirfoilTable(cl, cd);
				tables[k] = t;
			}
			return t;
		}
	}
}
