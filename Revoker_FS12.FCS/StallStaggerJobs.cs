using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using HarmonyLib;
using NuclearOption.Jobs;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.0.0: puts StallStagger's remapped lift/drag curves into the game's aero job.
	// The aero job (AeroJob_Math) reads each part's lift and drag from one global chart array (128 points per airfoil,
	// built once by JobManager.GenerateCharts) at the part's job-field airfoilID. The remapped charts are appended to
	// that array and the staggered surfaces' airfoilID pointed at them. Both happen in a prefix of
	// AeroJobSettings.Schedule: the previous aero job was completed in FixedUpdateLate, so nothing is reading the array
	// while it is swapped. Registration (Registry.EnsureSetup) only queues the part.
	internal static class StallStaggerJobs
	{
		private static readonly AccessTools.FieldRef<AeroPart, int> AP_airfoilID = AccessTools.FieldRefAccess<AeroPart, int>("airfoilID");

		private static readonly AccessTools.FieldRef<AeroPart, PtrAllocation<AeroPartFields>> AP_job = AccessTools.FieldRefAccess<AeroPart, PtrAllocation<AeroPartFields>>("JobFields");

		private static readonly AccessTools.FieldRef<AeroJobSettings, JobHandle> AJ_handleMath = AccessTools.FieldRefAccess<AeroJobSettings, JobHandle>("handleMath");

		private sealed class Entry
		{
			public AeroPart part;

			public Airfoil af;

			public StallStagger.Fam fam;

			public string key;

			public float peak;

			public float slope;
		}

		private sealed class State
		{
			public int length;

			public readonly Dictionary<string, int> ids = new Dictionary<string, int>();

			// 0.4.3: what this plugin wrote at each id, to recognise its own charts after another plugin (the other FCS
			// mods use the same scheme) appends to the shared table. Before, any length change counted as "the table was
			// rebuilt": the F-22E and Revoker re-appended after each other on every Schedule call, without end, so the
			// table and the per-tick copy kept growing (the gradual FPS loss with both mods in one mission).
			public readonly Dictionary<string, float[]> liftOwn = new Dictionary<string, float[]>();

			public readonly Dictionary<string, float[]> dragOwn = new Dictionary<string, float[]>();
		}

		private static readonly List<Entry> entries = new List<Entry>();

		private static readonly HashSet<int> registered = new HashSet<int>();

		private static readonly ConditionalWeakTable<AeroJobSettings, State> states = new ConditionalWeakTable<AeroJobSettings, State>();

		private static bool loggedError;

		private static int loggedCharts;

		private static int loggedKept;

		public static int Register(Aircraft a)
		{
			if (!StallStagger.Enabled || a == null)
			{
				return 0;
			}
			AircraftParameters prm = a.GetAircraftParameters();
			int n = 0;
			foreach (UnitPart p in a.partLookup)
			{
				if (!(p is AeroPart ap) || !registered.Add(ap.GetInstanceID()))
				{
					continue;
				}
				StallStagger.Fam f = StallStagger.FamilyOf(ap.gameObject.name);
				int idx = Acc.AP_airfoil(ap);
				if (f == StallStagger.Fam.None || prm == null || prm.airfoils == null || idx < 0 || idx >= prm.airfoils.Length)
				{
					continue;
				}
				Airfoil af = prm.airfoils[idx];
				if (af == null || af.liftCoef == null || af.dragCoef == null)
				{
					continue;
				}
				entries.Add(new Entry
				{
					part = ap,
					af = af,
					fam = f,
					key = StallStagger.Key(af, f),
					peak = StallStagger.PeakFor(f),
					slope = Plugin.StallStaggerLiftSlope
				});
				n++;
			}
			return n;
		}

		public static void SchedulePrefix(AeroJobSettings __instance)
		{
			if (entries.Count == 0 || __instance == null)
			{
				return;
			}
			try
			{
				if (!__instance.liftCharts.IsCreated || !__instance.dragCharts.IsCreated)
				{
					return;
				}
				State st = states.GetOrCreateValue(__instance);
				if (st.length != __instance.liftCharts.Length)
				{
					// 0.4.3: the table changed length. Only a rebuild (new JobManager / charts regenerated) loses our charts;
					// another plugin appending its own leaves them in place, so check ours before forgetting them.
					int len0 = st.length;
					if (!OwnChartsIntact(__instance, st))
					{
						st.ids.Clear();
						st.liftOwn.Clear();
						st.dragOwn.Clear();
					}
					else if (st.ids.Count > 0 && loggedKept < 4)
					{
						loggedKept++;
						Plugin.Log.LogInfo($"Revoker FCS aero charts: table grew {len0 / StallStagger.ChartN} -> {__instance.liftCharts.Length / StallStagger.ChartN} (another plugin's charts); ours are intact, not re-added.");
					}
					st.length = __instance.liftCharts.Length;
				}
				List<Entry> need = null;
				for (int i = entries.Count - 1; i >= 0; i--)
				{
					Entry e = entries[i];
					if ((UnityEngine.Object)(object)e.part == null)
					{
						entries.RemoveAt(i);
						continue;
					}
					if (!st.ids.ContainsKey(e.key))
					{
						if (need == null)
						{
							need = new List<Entry>();
						}
						if (!need.Exists(x => x.key == e.key))
						{
							need.Add(e);
						}
					}
				}
				if (need != null)
				{
					Extend(__instance, st, need);
				}
				for (int j = 0; j < entries.Count; j++)
				{
					Entry e2 = entries[j];
					if (!st.ids.TryGetValue(e2.key, out int id))
					{
						continue;
					}
					AP_airfoilID(e2.part) = id;
					ref PtrAllocation<AeroPartFields> pa = ref AP_job(e2.part);
					if (pa.IsCreated && pa.Ref().airfoilID != id)
					{
						pa.Ref().airfoilID = id;
					}
				}
			}
			catch (Exception ex)
			{
				if (!loggedError)
				{
					loggedError = true;
					Plugin.Log.LogError("Revoker FCS staggered stall (aero job charts): " + ex);
				}
			}
		}

		// 0.4.3: true when every chart this plugin registered is still at its id with exactly the values written there.
		// Runs only when the table length changes (spawns), not per tick.
		private static bool OwnChartsIntact(AeroJobSettings js, State st)
		{
			if (st.ids.Count == 0)
			{
				return true;
			}
			AJ_handleMath(js).Complete();
			int len = js.liftCharts.Length;
			if (js.dragCharts.Length != len)
			{
				return false;
			}
			foreach (KeyValuePair<string, int> kv in st.ids)
			{
				if (!st.liftOwn.TryGetValue(kv.Key, out float[] l) || !st.dragOwn.TryGetValue(kv.Key, out float[] d))
				{
					return false;
				}
				int off = kv.Value * StallStagger.ChartN;
				if (off < 0 || off + StallStagger.ChartN > len)
				{
					return false;
				}
				for (int i = 0; i < StallStagger.ChartN; i++)
				{
					if (!js.liftCharts[off + i].Equals(l[i]) || !js.dragCharts[off + i].Equals(d[i]))
					{
						return false;
					}
				}
			}
			return true;
		}

		private static void Extend(AeroJobSettings js, State st, List<Entry> need)
		{
			AJ_handleMath(js).Complete();
			int n0 = js.liftCharts.Length;
			int n1 = n0 + need.Count * StallStagger.ChartN;
			NativeArray<float> lift = new NativeArray<float>(n1, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			NativeArray<float> drag = new NativeArray<float>(n1, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
			NativeArray<float>.Copy(js.liftCharts, 0, lift, 0, n0);
			NativeArray<float>.Copy(js.dragCharts, 0, drag, 0, n0);
			float[] cl = new float[StallStagger.ChartN];
			float[] cd = new float[StallStagger.ChartN];
			for (int k = 0; k < need.Count; k++)
			{
				Entry e = need[k];
				StallStagger.BuildFor(e.fam, r => e.af.liftCoef.Evaluate(r), r => e.af.dragCoef.Evaluate(r), cl, cd);
				int off = n0 + k * StallStagger.ChartN;
				for (int i = 0; i < StallStagger.ChartN; i++)
				{
					lift[off + i] = cl[i];
					drag[off + i] = cd[i];
				}
				st.ids[e.key] = off / StallStagger.ChartN;
				st.liftOwn[e.key] = (float[])cl.Clone();
				st.dragOwn[e.key] = (float[])cd.Clone();
			}
			NativeArray<float> oldL = js.liftCharts;
			NativeArray<float> oldD = js.dragCharts;
			js.liftCharts = lift;
			js.dragCharts = drag;
			oldL.Dispose();
			oldD.Dispose();
			st.length = n1;
			if (loggedCharts < 8)
			{
				loggedCharts++;
				string s = string.Join(", ", need.ConvertAll(e => (e.fam == StallStagger.Fam.BodyFore || e.fam == StallStagger.Fam.BodyAft) ? $"{e.fam} lift x{BodyLift.Core(e.fam):F2} -> chart {st.ids[e.key]}" : $"{e.fam} stall {e.peak:F0} deg -> chart {st.ids[e.key]}"));
				Plugin.Log.LogInfo($"Revoker FCS staggered stall / body lift: aero charts {n0 / StallStagger.ChartN} -> {n1 / StallStagger.ChartN} ({s}; lift slope mode {need[0].slope:F2}).");
			}
		}
	}
}
