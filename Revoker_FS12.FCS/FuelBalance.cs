using System;
using System.Collections.Generic;
using HarmonyLib;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.0.6 fuel balancing (CG keeping). Every F-22E simulated on this machine moves fuel between its healthy tanks so the
	// centre of mass stays as close as the fuel allows to the clean jet's: on the centreline laterally, and longitudinally
	// where the stock fuel schedule puts it without stores. Stores, a part shot off or a leaking tank move the CG; the
	// transfer moves it back, at FuelTransferRate kg/s in total. Fuel is only ever moved between tanks that are attached
	// and not leaking or burning, mass is conserved exactly (each kilogram taken from one tank's part is added to another
	// tank's part through UnitPart.ModifyMass, the same call the game uses for burn and leaks).
	//
	// Phantom fuel: stock Aircraft.UseFuel / GetFuelLevel / GetFuelQuantity keep counting a tank whose part has been shot
	// off (the tank stays registered; it only leaks from the debris), so the engines kept burning fuel out of a wing lying
	// on the ground and the gauge kept showing it, and once the debris tank had leaked dry a refuel filled it again. For
	// F-22Es the engines now draw only from attached tanks, the gauge and fuel checks count only attached tanks (against
	// the full original capacity), and a detached tank is skipped by refuelling.
	internal static class FuelBalance
	{
		private static readonly AccessTools.FieldRef<FuelTank, UnitPart> FT_part = AccessTools.FieldRefAccess<FuelTank, UnitPart>("part");

		private static readonly AccessTools.FieldRef<FuelTank, Aircraft> FT_aircraft = AccessTools.FieldRefAccess<FuelTank, Aircraft>("aircraft");

		private static readonly AccessTools.FieldRef<FuelTank, float> FT_leak = AccessTools.FieldRefAccess<FuelTank, float>("leakRate");

		private static readonly AccessTools.FieldRef<FuelTank, bool> FT_fire = AccessTools.FieldRefAccess<FuelTank, bool>("onFire");

		private static readonly AccessTools.FieldRef<Aircraft, List<FuelTank>> AC_tanks = AccessTools.FieldRefAccess<Aircraft, List<FuelTank>>("fuelTanks");

		private static readonly AccessTools.FieldRef<Aircraft, float> AC_cap = AccessTools.FieldRefAccess<Aircraft, float>("fuelCapacity");

		// store / ammunition mass hung on each part (Hardpoint.ModifyMass), by part instance id
		private static readonly Dictionary<int, float> storeMass = new Dictionary<int, float>();

		private sealed class State
		{
			public float timer;

			public float xferDt;

			public float[] target;

			public FuelTank[] tanks;

			public int n;

			public float logTimer;

			public bool active;
		}

		private static readonly Dictionary<int, State> states = new Dictionary<int, State>();

		private static readonly HashSet<Rigidbody> rbSeen = new HashSet<Rigidbody>();

		public static bool Connected(FuelTank t, Aircraft a)
		{
			if (t == null)
			{
				return false;
			}
			UnitPart unitPart = FT_part(t);
			return unitPart != null && !unitPart.IsDetached() && (UnityEngine.Object)(object)unitPart.parentUnit == (UnityEngine.Object)(object)a;
		}

		private static bool Healthy(FuelTank t)
		{
			return FT_leak(t) <= 0f && !FT_fire(t);
		}

		// ---- phantom-fuel fixes ------------------------------------------------------------------------------------

		public static bool UseFuelPrefix(Aircraft __instance, ref float __0, ref bool __result)
		{
			try
			{
				if ((UnityEngine.Object)(object)__instance == null || !Registry.IsF22(__instance))
				{
					return true;
				}
				if (Plugin.FuelUseScale != 1f)
				{
					__0 *= Mathf.Max(0f, Plugin.FuelUseScale);
				}
				if (!Plugin.FuelAttachedOnly)
				{
					return true;
				}
				List<FuelTank> list = AC_tanks(__instance);
				float total = 0f;
				foreach (FuelTank item in list)
				{
					if (Connected(item, __instance))
					{
						total += item.fuelMass;
					}
				}
				float cap = AC_cap(__instance);
				float level = (cap > 0f) ? (total / cap) : 0f;
				if (total <= 0f)
				{
					__instance.CheckNeedsFuel(0f);
					__result = false;
					return false;
				}
				foreach (FuelTank item2 in list)
				{
					if (Connected(item2, __instance) && item2.fuelMass > 0f)
					{
						item2.UseFuel(__0 * item2.fuelMass / total);
					}
				}
				__instance.CheckNeedsFuel(level);
				__result = true;
				return false;
			}
			catch
			{
				return true;
			}
		}

		public static void GetFuelLevelPostfix(Aircraft __instance, ref float __result)
		{
			try
			{
				if (!Plugin.FuelAttachedOnly || (UnityEngine.Object)(object)__instance == null || !Registry.IsF22(__instance))
				{
					return;
				}
				float num = 0f;
				float num2 = 0f;
				foreach (FuelTank item in AC_tanks(__instance))
				{
					if (!(item == null))
					{
						num2 += item.GetCapacity();
						if (Connected(item, __instance))
						{
							num += item.fuelMass;
						}
					}
				}
				__result = ((num2 > 0f) ? (num / num2) : 0f);
			}
			catch
			{
			}
		}

		public static void GetFuelQuantityPostfix(Aircraft __instance, ref float __result)
		{
			try
			{
				if (!Plugin.FuelAttachedOnly || (UnityEngine.Object)(object)__instance == null || !Registry.IsF22(__instance))
				{
					return;
				}
				float num = 0f;
				foreach (FuelTank item in AC_tanks(__instance))
				{
					if (Connected(item, __instance))
					{
						num += item.fuelMass;
					}
				}
				__result = num;
			}
			catch
			{
			}
		}

		// a tank whose part is gone is not refuelled (and does not block the refuel of the others)
		public static bool RefuelPrefix(FuelTank __instance, ref bool __result)
		{
			try
			{
				if (!Plugin.FuelAttachedOnly || __instance == null)
				{
					return true;
				}
				Aircraft aircraft = FT_aircraft(__instance);
				if ((UnityEngine.Object)(object)aircraft == null || !Registry.IsF22(aircraft))
				{
					return true;
				}
				if (!Connected(__instance, aircraft))
				{
					__result = true;
					return false;
				}
			}
			catch
			{
			}
			return true;
		}

		public static void HardpointMassPostfix(Hardpoint __instance, float change)
		{
			try
			{
				if (change == 0f || __instance == null || __instance.part == null || __instance.part.rb == null)
				{
					return;
				}
				int instanceID = __instance.part.GetInstanceID();
				storeMass.TryGetValue(instanceID, out var value);
				storeMass[instanceID] = value + change;
			}
			catch
			{
			}
		}

		public static void Forget(Aircraft a)
		{
			if ((UnityEngine.Object)(object)a != null)
			{
				states.Remove(((UnityEngine.Object)(object)a).GetInstanceID());
			}
		}

		// ---- balancing ---------------------------------------------------------------------------------------------

		public static void Step(Aircraft a, float dt)
		{
			if (!Plugin.FuelBalanceOn || (UnityEngine.Object)(object)a == null || !a.LocalSim)
			{
				return;
			}
			int instanceID = ((UnityEngine.Object)(object)a).GetInstanceID();
			if (!states.TryGetValue(instanceID, out var value))
			{
				if (states.Count > 64)
				{
					states.Clear();
				}
				value = new State();
				states[instanceID] = value;
			}
			value.timer -= dt;
			value.logTimer -= dt;
			value.xferDt += dt;
			if (value.timer <= 0f)
			{
				// plan and move every 0.1 s: fewer float additions onto the part masses (each one rounds)
				value.timer = 0.1f;
				// 2.0.12: the FCS may ask for a lateral CG offset (fuel trim, engine out / damage), moved at FuelTrimRate
				FcsController fc = Registry.Find(a);
				bool trim = fc != null && fc.fuelTrimOn;
				Solve(a, value, trim ? fc.fuelTrimLat : 0f);
				if (value.n > 0)
				{
					Transfer(value, (trim ? Mathf.Max(Plugin.FuelTransferRate, Plugin.FuelTrimRate) : Plugin.FuelTransferRate) * value.xferDt, a);
				}
				value.xferDt = 0f;
			}
		}

		// 2.0.12: whole-jet lateral CG (m along the jet's right, from its root), the frame Solve works in
		public static float LateralCg(Aircraft a)
		{
			Transform t = ((Component)(object)a).transform;
			double M = 0.0;
			double Sx = 0.0;
			rbSeen.Clear();
			foreach (UnitPart item in a.partLookup)
			{
				if (item == null || item.IsDetached() || item.rb == null || !rbSeen.Add(item.rb))
				{
					continue;
				}
				M += item.rb.mass;
				Sx += (double)(item.rb.mass * Vector3.Dot(Pos(item) - t.position, t.right));
			}
			return (M > 1.0) ? ((float)(Sx / M)) : 0f;
		}

		private static Vector3 Pos(UnitPart p)
		{
			return (p.rb != null) ? p.rb.worldCenterOfMass : p.transform.position;
		}

		private static void Solve(Aircraft a, State st, float txLat)
		{
			st.n = 0;
			List<FuelTank> list = AC_tanks(a);
			if (list == null || list.Count < 2)
			{
				return;
			}
			Transform transform = ((Component)(object)a).transform;
			Vector3 position = transform.position;
			Vector3 right = transform.right;
			Vector3 forward = transform.forward;
			// whole-jet centre of mass (attached rigidbodies), and the clean jet's: no stores, healthy tanks' fuel spread
			// the stock way (in proportion to capacity)
			double M = 0.0;
			double Sx = 0.0;
			double Sz = 0.0;
			double Mc = 0.0;
			double Scz = 0.0;
			rbSeen.Clear();
			foreach (UnitPart item in a.partLookup)
			{
				if (item == null || item.IsDetached())
				{
					continue;
				}
				Vector3 lhs = Pos(item) - position;
				float x = Vector3.Dot(lhs, right);
				float z = Vector3.Dot(lhs, forward);
				if (item.rb != null && rbSeen.Add(item.rb))
				{
					M += item.rb.mass;
					Sx += (double)(item.rb.mass * x);
					Sz += (double)(item.rb.mass * z);
				}
				storeMass.TryGetValue(item.GetInstanceID(), out var value);
				float num = item.mass - Mathf.Max(0f, value);
				Mc += num;
				Scz += (double)(num * z);
			}
			if (M < 1000.0)
			{
				return;
			}
			int count = list.Count;
			if (st.tanks == null || st.tanks.Length < count)
			{
				st.tanks = new FuelTank[count];
				st.target = new float[count];
			}
			if (xs.Length < count)
			{
				xs = new float[count];
				zs = new float[count];
				cs = new float[count];
				f0 = new float[count];
				fp = new float[count];
				fv = new float[count];
			}
			int n = 0;
			float F = 0f;
			float capH = 0f;
			foreach (FuelTank item2 in list)
			{
				if (!Connected(item2, a))
				{
					continue;
				}
				UnitPart unitPart = FT_part(item2);
				Vector3 lhs2 = Pos(unitPart) - position;
				float x2 = Vector3.Dot(lhs2, right);
				float z2 = Vector3.Dot(lhs2, forward);
				// the clean reference holds every tank's fuel out (added back below the stock way)
				Mc -= item2.fuelMass;
				Scz -= (double)(item2.fuelMass * z2);
				if (!Healthy(item2))
				{
					// a leaking / burning tank keeps its own fuel: fixed mass for both the jet and the reference
					Mc += item2.fuelMass;
					Scz += (double)(item2.fuelMass * z2);
					continue;
				}
				st.tanks[n] = item2;
				xs[n] = x2;
				zs[n] = z2;
				cs[n] = Mathf.Max(1f, item2.GetCapacity());
				f0[n] = Mathf.Clamp(item2.fuelMass, 0f, cs[n]);
				F += f0[n];
				capH += cs[n];
				n++;
			}
			if (n < 2 || F < 1f)
			{
				return;
			}
			double refZ = Scz;
			for (int i = 0; i < n; i++)
			{
				fp[i] = F * cs[i] / capH;
				refZ += (double)(fp[i] * zs[i]);
			}
			refZ /= Math.Max(1.0, Mc + (double)F);
			// fuel-free parts of the present jet's moments
			double ax = Sx;
			double az = Sz;
			for (int j = 0; j < n; j++)
			{
				ax -= (double)(f0[j] * xs[j]);
				az -= (double)(f0[j] * zs[j]);
			}
			float tx = txLat;
			float tz = (float)refZ;
			float exNow = (float)(Sx / M) - tx;
			float ezNow = (float)(Sz / M) - tz;
			// least squares in the CG error (lateral weighted over longitudinal) plus a small pull toward the stock
			// spread; projected gradient on {sum f = F, 0 <= f <= capacity}
			float sx = Plugin.FuelBalanceTolLat;
			float sz = Plugin.FuelBalanceTolLong;
			float lam = 1f;
			float m = (float)M;
			double L = 0.0;
			for (int k = 0; k < n; k++)
			{
				L += (double)(xs[k] * xs[k] / (m * m * sx * sx) + zs[k] * zs[k] / (m * m * sz * sz));
			}
			float minCap = float.MaxValue;
			for (int l = 0; l < n; l++)
			{
				minCap = Mathf.Min(minCap, cs[l]);
			}
			L = 2.0 * L + 2.0 * (double)lam / (double)(minCap * minCap);
			float step = (float)(1.0 / Math.Max(L, 1E-09));
			for (int num2 = 0; num2 < n; num2++)
			{
				fv[num2] = f0[num2];
			}
			for (int it = 0; it < 80; it++)
			{
				double X = ax;
				double Z = az;
				for (int num3 = 0; num3 < n; num3++)
				{
					X += (double)(fv[num3] * xs[num3]);
					Z += (double)(fv[num3] * zs[num3]);
				}
				float ex = (float)(X / M) - tx;
				float ez = (float)(Z / M) - tz;
				for (int num4 = 0; num4 < n; num4++)
				{
					float g = 2f * ex / (sx * sx) * xs[num4] / m + 2f * ez / (sz * sz) * zs[num4] / m + 2f * lam * (fv[num4] - fp[num4]) / (cs[num4] * cs[num4]);
					fv[num4] -= step * g;
				}
				Project(fv, cs, n, F);
			}
			double X2 = ax;
			double Z2 = az;
			float moved = 0f;
			for (int num5 = 0; num5 < n; num5++)
			{
				X2 += (double)(fv[num5] * xs[num5]);
				Z2 += (double)(fv[num5] * zs[num5]);
				moved += Mathf.Abs(fv[num5] - f0[num5]);
			}
			float exT = (float)(X2 / M) - tx;
			float ezT = (float)(Z2 / M) - tz;
			// deadband: leave the fuel alone when the CG is already within half the tolerances, or the plan gains little
			bool inBand = Mathf.Abs(exNow) < 0.5f * sx && Mathf.Abs(ezNow) < 0.5f * sz;
			float gain = exNow * exNow / (sx * sx) + ezNow * ezNow / (sz * sz) - (exT * exT / (sx * sx) + ezT * ezT / (sz * sz));
			if (moved < 2f || (inBand && !st.active) || gain < 0.05f)
			{
				st.active = false;
				return;
			}
			for (int num6 = 0; num6 < n; num6++)
			{
				st.target[num6] = fv[num6];
			}
			st.n = n;
			if (!st.active || st.logTimer <= 0f)
			{
				st.logTimer = 10f;
				if (Plugin.FuelBalanceLog)
				{
					Plugin.Log.LogInfo($"Revoker FCS fuel balance on {((UnityEngine.Object)(object)a).name}: CG {exNow * 100f:+0.0;-0.0} cm lateral, {ezNow * 100f:+0.0;-0.0} cm fore/aft of the clean jet's; plan moves {moved * 0.5f:F0} kg among {n} tanks for {exT * 100f:+0.0;-0.0} / {ezT * 100f:+0.0;-0.0} cm ({F:F0} kg balanceable fuel).");
				}
			}
			st.active = true;
		}

		private static float[] xs = new float[0];

		private static float[] zs = new float[0];

		private static float[] cs = new float[0];

		private static float[] f0 = new float[0];

		private static float[] fp = new float[0];

		private static float[] fv = new float[0];

		// projection onto {sum f = F, 0 <= f_i <= c_i}: f_i = clamp(y_i - mu, 0, c_i), mu by bisection
		internal static void Project(float[] f, float[] c, int n, float F)
		{
			float lo = float.MaxValue;
			float hi = float.MinValue;
			for (int i = 0; i < n; i++)
			{
				lo = Mathf.Min(lo, f[i] - c[i]);
				hi = Mathf.Max(hi, f[i]);
			}
			for (int j = 0; j < 50; j++)
			{
				float mu = 0.5f * (lo + hi);
				float s = 0f;
				for (int k = 0; k < n; k++)
				{
					s += Mathf.Clamp(f[k] - mu, 0f, c[k]);
				}
				if (s > F)
				{
					lo = mu;
				}
				else
				{
					hi = mu;
				}
			}
			float mu2 = 0.5f * (lo + hi);
			for (int l = 0; l < n; l++)
			{
				f[l] = Mathf.Clamp(f[l] - mu2, 0f, c[l]);
			}
		}

		// moves fuel toward the plan, at most 'budget' kg in total this step (as much out as in), only between tanks that
		// are still attached and healthy now
		private static void Transfer(State st, float budget, Aircraft a)
		{
			if (budget <= 0f)
			{
				return;
			}
			float outSum = 0f;
			float inSum = 0f;
			for (int i = 0; i < st.n; i++)
			{
				FuelTank fuelTank = st.tanks[i];
				if (!Connected(fuelTank, a) || !Healthy(fuelTank))
				{
					st.n = 0;
					st.timer = 0f;
					return;
				}
				float d = Mathf.Clamp(st.target[i], 0f, fuelTank.GetCapacity()) - fuelTank.fuelMass;
				if (d > 0f)
				{
					inSum += d;
				}
				else
				{
					outSum -= d;
				}
			}
			float avail = Mathf.Min(outSum, inSum);
			if (avail < 0.01f)
			{
				st.n = 0;
				return;
			}
			float move = Mathf.Min(budget, avail);
			float kOut = move / outSum;
			float kIn = move / inSum;
			float taken = 0f;
			for (int j = 0; j < st.n; j++)
			{
				FuelTank fuelTank2 = st.tanks[j];
				float d2 = Mathf.Clamp(st.target[j], 0f, fuelTank2.GetCapacity()) - fuelTank2.fuelMass;
				if (d2 < 0f)
				{
					float num = Mathf.Min(fuelTank2.fuelMass, (0f - d2) * kOut);
					fuelTank2.fuelMass -= num;
					FT_part(fuelTank2).ModifyMass(0f - num);
					taken += num;
				}
			}
			float given = 0f;
			for (int k = 0; k < st.n; k++)
			{
				FuelTank fuelTank3 = st.tanks[k];
				float d3 = Mathf.Clamp(st.target[k], 0f, fuelTank3.GetCapacity()) - fuelTank3.fuelMass;
				if (d3 > 0f)
				{
					float num2 = Mathf.Min(d3 * kIn, taken - given);
					num2 = Mathf.Min(num2, fuelTank3.GetCapacity() - fuelTank3.fuelMass);
					if (num2 > 0f)
					{
						fuelTank3.fuelMass += num2;
						FT_part(fuelTank3).ModifyMass(num2);
						given += num2;
					}
				}
			}
			// whatever could not be placed (rounding) goes back where it came from: never created, never lost
			float rest = taken - given;
			if (rest > 0f)
			{
				for (int l = 0; l < st.n && rest > 0f; l++)
				{
					FuelTank fuelTank4 = st.tanks[l];
					float room = fuelTank4.GetCapacity() - fuelTank4.fuelMass;
					float put = Mathf.Min(room, rest);
					if (put > 0f)
					{
						fuelTank4.fuelMass += put;
						FT_part(fuelTank4).ModifyMass(put);
						rest -= put;
					}
				}
			}
		}
	}
}
