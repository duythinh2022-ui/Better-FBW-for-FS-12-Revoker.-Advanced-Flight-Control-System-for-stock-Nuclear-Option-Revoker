using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal sealed class AirfoilTable
	{
		private readonly float[] cl = new float[128];

		private readonly float[] cd = new float[128];

		private readonly bool generic;

		public AirfoilTable(Airfoil af)
		{
			if (af == null || af.liftCoef == null || af.dragCoef == null)
			{
				generic = true;
				return;
			}
			for (int i = 0; i < 128; i++)
			{
				float time = (float)(i - 64) * 0.04908734f;
				cl[i] = af.liftCoef.Evaluate(time);
				cd[i] = af.dragCoef.Evaluate(time);
			}
		}

		// 2.0.0: from ready-made 128-point charts (StallStagger's remapped curves)
		public AirfoilTable(float[] clChart, float[] cdChart)
		{
			for (int i = 0; i < 128; i++)
			{
				cl[i] = clChart[i];
				cd[i] = cdChart[i];
			}
		}

		private static float Read(float[] t, float idx)
		{
			if (idx <= 0f)
			{
				return t[0];
			}
			if (idx >= 127f)
			{
				return t[127];
			}
			int num = (int)idx;
			float num2 = idx - (float)num;
			return t[num] + (t[num + 1] - t[num]) * num2;
		}

		public void Get(float a, out float CL, out float CD)
		{
			if (generic)
			{
				CL = 1.8f * Mathf.Sin(5f * a);
				CD = 1.5f * (1f - Mathf.Cos(2f * a)) + 0.02f;
			}
			else
			{
				float idx = a * 20.37185f + 64f;
				CL = Read(cl, idx);
				CD = Read(cd, idx);
			}
		}

		public float StallAoADeg()
		{
			if (generic)
			{
				return 18f;
			}
			int num = 64;
			for (int i = 64; i < 96; i++)
			{
				if (cl[i] > cl[num])
				{
					num = i;
				}
			}
			return (float)(num - 64) * 0.04908734f * 57.29578f;
		}

		// 0.4.3: the AoA (deg, positive side) where the lift slope has fallen to frac of its steepest value below the peak:
		// the knee of a plateau foil. The Ifrit's stabilator foil (Multirole1_elevators) has no stall before 45 deg: its lift
		// keeps rising from 20 to 45 deg at a fifth of the linear slope (4.4 -> 0.84 /rad at 20 deg, 0.17 at 40) while the
		// drag goes 0.09 -> 0.48, so its "peak" is 45 deg and a window set a few degrees under it leaves a stab on the flat.
		// Returns the peak itself for a foil with no such knee (or the generic one).
		public float KneeAoADeg(float frac)
		{
			float peak = StallAoADeg();
			if (generic || frac <= 0f)
			{
				return peak;
			}
			int iPk = 64 + Mathf.Clamp(Mathf.RoundToInt(peak / (0.04908734f * 57.29578f)), 0, 62);
			float sMax = 0f;
			int iMax = 64;
			for (int i = 64; i < iPk; i++)
			{
				float sl = cl[i + 1] - cl[i];
				if (sl > sMax)
				{
					sMax = sl;
					iMax = i;
				}
			}
			if (sMax <= 0f)
			{
				return peak;
			}
			for (int i = iMax; i < iPk; i++)
			{
				float sl = cl[i + 1] - cl[i];
				if (sl < frac * sMax)
				{
					float sPrev = (i > iMax) ? (cl[i] - cl[i - 1]) : sMax;
					float f = (sPrev > sl) ? Mathf.Clamp01((sPrev - frac * sMax) / (sPrev - sl)) : 1f;
					return ((float)(i - 64) + f) * 0.04908734f * 57.29578f;
				}
			}
			return peak;
		}
	}
}
