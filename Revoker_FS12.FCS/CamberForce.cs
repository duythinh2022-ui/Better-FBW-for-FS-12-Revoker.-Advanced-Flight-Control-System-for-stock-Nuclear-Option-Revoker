using System.Collections.Generic;
using NuclearOption.Jobs;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.2.0: game-side flap-camber coupling (see Camber). Adds, every physics step, the host part's force increment for
	// its control surface's current deflection, at the host's centre of lift.
	internal class CamberForce : MonoBehaviour
	{
		private Aircraft ac;

		private readonly List<ControlSurface> surfs = new List<ControlSurface>();

		private readonly List<AeroPart> hosts = new List<AeroPart>();

		private readonly List<AirfoilTable> tables = new List<AirfoilTable>();

		// Revoker 0.2.20: the inboard elevons (hinge flaps). With ElevonStallStagger their camber share on wing2 is applied here
		// as a force on the inboard curve instead of by turning wing2's lift normal (which the game evaluates on wing2's own,
		// unstaggered curve: that share, 0.46 x 3.8 m2, is larger than the elevon's own 1.4 m2 and would have kept the pair's
		// reversal near 40-45 deg).
		private readonly List<HingeFlapEffector> hinges = new List<HingeFlapEffector>();

		private readonly List<AirfoilTable> hingeTables = new List<AirfoilTable>();

		public static void AddHinge(Aircraft a, HingeFlapEffector hf, AirfoilTable table)
		{
			if (a == null || hf == null || table == null)
			{
				return;
			}
			GameObject go = ((Component)(object)a).gameObject;
			CamberForce c = go.GetComponent<CamberForce>();
			if (c == null)
			{
				c = go.AddComponent<CamberForce>();
				c.ac = a;
			}
			if (!c.hinges.Contains(hf))
			{
				c.hinges.Add(hf);
				c.hingeTables.Add(table);
			}
		}

		public static int Attach(Aircraft a)
		{
			if (a == null)
			{
				return 0;
			}
			GameObject go = ((Component)(object)a).gameObject;
			if (go.GetComponent<CamberForce>() != null)
			{
				return 0;
			}
			AircraftParameters prm = a.GetAircraftParameters();
			Dictionary<string, AeroPart> byName = new Dictionary<string, AeroPart>();
			foreach (UnitPart p in a.partLookup)
			{
				if (p is AeroPart ap)
				{
					byName[ap.gameObject.name.Replace("Aryx_KingRaptor_", "")] = ap;
				}
			}
			CamberForce c = null;
			foreach (ControlSurface cs in Acc.Collect<ControlSurface>(a))
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
				if (c == null)
				{
					c = go.AddComponent<CamberForce>();
					c.ac = a;
				}
				c.surfs.Add(cs);
				c.hosts.Add(host);
				// Revoker 0.2.20: the camber increment uses the elevon's own (staggered) curve, so it reverses with the elevon
				StallStagger.Fam fS = StallStagger.Enabled ? StallStagger.FamilyOf(cs.gameObject.name) : StallStagger.Fam.None;
				c.tables.Add((fS != StallStagger.Fam.None) ? StallStagger.TableForFamily(af, fS) : StallStagger.TableFor(host, af));
			}
			return (c != null) ? c.surfs.Count : 0;
		}

		private void FixedUpdate()
		{
			if ((Object)(object)ac == null)
			{
				Destroy(this);
				return;
			}
			if (!ac.LocalSim || (Plugin.FlapCamberK <= 0f && Plugin.AileronCamberK <= 0f && Plugin.FinCamberK <= 0f))
			{
				return;
			}
			float rho = ac.airDensity;
			if (rho <= 0f)
			{
				return;
			}
			Vector3 wind = Vector3.zero;
			try
			{
				wind = ac.GetWindVelocity();
			}
			catch
			{
			}
			if (Plugin.StallStagger && Plugin.FlapCamberK > 0f)
			{
				for (int j = 0; j < hinges.Count; j++)
				{
					HingeFlapEffector hf = hinges[j];
					if (hf == null || !hf.driving || !hf.Valid || hf.host == null || hf.host.rb == null || hf.host.IsDetached() || hf.hostLN == null)
					{
						continue;
					}
					Vector3 vH = hf.host.rb.velocity - wind;
					float v2H = vH.sqrMagnitude;
					if (v2H < 25f)
					{
						continue;
					}
					float qSH = 0.5f * rho * v2H * Acc.AP_wingArea(hf.host) * Acc.AP_wingEff(hf.host);
					Quaternion h0 = hf.HostRot0;
					Vector3 dFH = Camber.Delta(vH, h0, hf.HingeAxisWorld, Plugin.FlapCamberK, hf.hingeNow, hingeTables[j], qSH);
					if (dFH.sqrMagnitude > 0f)
					{
						hf.host.rb.AddForceAtPosition(dFH, hf.host.rb.worldCenterOfMass + h0 * Acc.AP_centerOfLift(hf.host));
					}
				}
			}
			for (int i = 0; i < surfs.Count; i++)
			{
				ControlSurface cs = surfs[i];
				AeroPart host = hosts[i];
				if (cs == null || host == null || host.rb == null || host.IsDetached() || !Acc.JobCreated(cs))
				{
					continue;
				}
				ref ControlSurfaceFields job = ref Acc.Job(cs);
				if (job.IsDetached)
				{
					continue;
				}
				float k = Camber.KForName(cs.gameObject.name);
				if (k <= 0f)
				{
					continue;
				}
				Transform vis = Acc.CS_visible(cs).transform;
				float delta = Camber.DeflectionOf(vis, job.restingRotation);
				Transform lt = Acc.AP_liftNormal(host);
				if (lt == null)
				{
					continue;
				}
				Vector3 v = host.rb.velocity - wind;
				float v2 = v.sqrMagnitude;
				if (v2 < 25f)
				{
					continue;
				}
				float qS = 0.5f * rho * v2 * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				Vector3 dF = Camber.Delta(v, lt.rotation, vis.rotation * Vector3.right, k, delta, tables[i], qS);
				if (dF.sqrMagnitude > 0f)
				{
					host.rb.AddForceAtPosition(dF, host.rb.worldCenterOfMass + lt.rotation * Acc.AP_centerOfLift(host));
				}
			}
		}
	}
}
