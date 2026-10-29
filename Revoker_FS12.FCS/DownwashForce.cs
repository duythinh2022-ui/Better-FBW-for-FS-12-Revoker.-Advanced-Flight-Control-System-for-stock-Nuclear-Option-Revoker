using System.Collections.Generic;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.0.8: puts the stab downwash into the game's aero. The aero job (AeroJob_Math) gives each stab its airfoil lift
	// and drag in the free-stream flow; this adds, every physics step, the difference between that airfoil force and the
	// one in the downwash-turned flow (same chart, same area and effectiveness, same point of application: rigidbody
	// COM + rotated centre of lift). Parasitic drag depends only on the speed, which the turn keeps, so it cancels.
	// The game's transonic drag factor is applied to the drag difference too.
	internal class DownwashForce : MonoBehaviour
	{
		private Aircraft ac;

		private readonly List<AeroPart> parts = new List<AeroPart>();

		private readonly List<AirfoilTable> tables = new List<AirfoilTable>();

		public static int Attach(Aircraft a)
		{
			if (a == null || Plugin.StabDownwash <= 0f)
			{
				return 0;
			}
			GameObject go = ((Component)(object)a).gameObject;
			if (go.GetComponent<DownwashForce>() != null)
			{
				return 0;
			}
			DownwashForce d = go.AddComponent<DownwashForce>();
			d.ac = a;
			AircraftParameters prm = a.GetAircraftParameters();
			foreach (UnitPart p in a.partLookup)
			{
				if (p is AeroPart ap && Downwash.IsStab(ap))
				{
					int idx = Acc.AP_airfoil(ap);
					Airfoil af = (idx >= 0 && prm != null && prm.airfoils != null && idx < prm.airfoils.Length) ? prm.airfoils[idx] : null;
					if (af == null || af.liftCoef == null || af.dragCoef == null)
					{
						continue;
					}
					d.parts.Add(ap);
					d.tables.Add(StallStagger.TableFor(ap, af));
				}
			}
			return d.parts.Count;
		}

		private static Vector3 Foil(Vector3 v, Quaternion rot, AirfoilTable t, float qS, out Vector3 dragPart)
		{
			Vector3 vl = Quaternion.Inverse(rot) * v;
			t.Get(Mathf.Atan2(vl.y, vl.z), out var cl, out var cd);
			Vector3 n = Vector3.Cross(v, rot * Vector3.right).normalized;
			dragPart = -v.normalized * (cd * qS);
			return -n * (cl * qS);
		}

		private void FixedUpdate()
		{
			if ((UnityEngine.Object)(object)ac == null)
			{
				Destroy(this);
				return;
			}
			if (!ac.LocalSim || Plugin.StabDownwash <= 0f)
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
			Quaternion root = ((Component)(object)ac).transform.rotation;
			// eps from the aircraft's own flow (the wing's AoA), not each stab's: the stab keeps its pitch-rate AoA
			float eps = (ac.rb != null) ? Downwash.EpsFor(ac.rb.velocity - wind, root) : 0f;
			for (int i = 0; i < parts.Count; i++)
			{
				AeroPart ap = parts[i];
				if (ap == null || ap.rb == null || ap.IsDetached())
				{
					continue;
				}
				Transform lt = Acc.AP_liftNormal(ap);
				if (lt == null)
				{
					continue;
				}
				Vector3 v = ap.rb.velocity - wind;
				float v2 = v.sqrMagnitude;
				if (v2 < 25f)
				{
					continue;
				}
				Vector3 v1 = Downwash.Turn(v, root, eps);
				Quaternion rot = lt.rotation;
				float qS = 0.5f * rho * v2 * Acc.AP_wingArea(ap) * Acc.AP_wingEff(ap);
				Vector3 l0 = Foil(v, rot, tables[i], qS, out var d0);
				Vector3 l1 = Foil(v1, rot, tables[i], qS, out var d1);
				Vector3 com = ap.rb.worldCenterOfMass;
				float mach = Mathf.Sqrt(v2) / DragReliefMath.SpeedOfSound(com.y);
				Vector3 dF = (l1 - l0) + (d1 - d0) * DragReliefMath.Transonic(mach);
				ap.rb.AddForceAtPosition(dF, com + rot * Acc.AP_centerOfLift(ap));
			}
		}
	}
}
