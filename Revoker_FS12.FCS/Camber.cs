using UnityEngine;

namespace Revoker_FS12.FCS
{
	// 2.2.0: flap-camber coupling. Nuclear Option flies each control surface as its own plate: a deflected rudder changes
	// only its own lift, and the fixed fin ahead of it does not see the camber. On a real flapped section the deflection
	// cambers the whole fin + rudder (or wing + flaperon) section, so the fixed part carries a share of the extra lift
	// (thin-airfoil flap effectiveness ~0.75 for a half-chord rudder, ~0.55 for the ~0.35-chord flaperons). Modelled as
	// the host part's airfoil force at its local AoA plus K x deflection (about the control surface's hinge axis), minus
	// its force at its local AoA - zero with the surface centred, so the trimmed centre of lift does not move.
	// Shared by the game-side CamberForce, the FCS surface models and the bench.
	internal static class Camber
	{
		public static string HostOf(string surfaceName)
		{
			string n = surfaceName.Replace("Aryx_KingRaptor_", "");
			// Revoker 0.1.0: aileron_L/R (the delta's trailing-edge elevon/flaperon) is a child of wingtip_L/R and sits right
			// behind it (chord 1.02 of the section's 3.41 m). The canards and the rudder are not coupled.
			if (n.StartsWith("aileron_"))
			{
				return "wingtip_" + n.Substring(8);
			}
			if (n.StartsWith("Rudder_"))
			{
				return "Vstab_" + n.Substring(7);
			}
			if (n.StartsWith("Flap_"))
			{
				return "WingRear_" + n.Substring(5);
			}
			if (n.StartsWith("Aileron_"))
			{
				return "Wingmid_" + n.Substring(8);
			}
			return null;
		}

		public static float K(SurfaceKind kind)
		{
			switch (kind)
			{
			case SurfaceKind.Rudder:
				return Plugin.FinCamberK;
			case SurfaceKind.Flaperon:
				return Plugin.FlapCamberK;
			case SurfaceKind.Aileron:
				return Plugin.AileronCamberK;
			default:
				return 0f;
			}
		}

		public static float KForName(string surfaceName)
		{
			string n = surfaceName.Replace("Aryx_KingRaptor_", "");
			if (n.StartsWith("aileron_"))
			{
				return Plugin.AileronCamberK;
			}
			if (n.StartsWith("Rudder_"))
			{
				return Plugin.FinCamberK;
			}
			if (n.StartsWith("Flap_") || n.StartsWith("Aileron_"))
			{
				return Plugin.WingCamberK;
			}
			return 0f;
		}

		// lift + airfoil drag of a part with lift-normal rotation rot in flow v (qS = 0.5 rho v^2 S eff)
		public static Vector3 Foil(Vector3 v, Quaternion rot, AirfoilTable t, float qS)
		{
			Vector3 vl = Quaternion.Inverse(rot) * v;
			t.Get(Mathf.Atan2(vl.y, vl.z), out var cl, out var cd);
			Vector3 n = Vector3.Cross(v, rot * Vector3.right);
			float m = n.magnitude;
			if (m < 1E-06f || v.sqrMagnitude < 1E-06f)
			{
				return Vector3.zero;
			}
			return -n / m * (cl * qS) - v.normalized * (cd * qS);
		}

		// host force increment for a control-surface deflection delta (deg) about the world hinge axis
		public static Vector3 Delta(Vector3 v, Quaternion hostRot, Vector3 hingeAxis, float k, float delta, AirfoilTable t, float qS)
		{
			if (k <= 0f || Mathf.Abs(delta) < 0.01f)
			{
				return Vector3.zero;
			}
			Quaternion r1 = Quaternion.AngleAxis(k * delta, hingeAxis) * hostRot;
			return Foil(v, r1, t, qS) - Foil(v, hostRot, t, qS);
		}

		// a control surface's current deflection about its hinge (deg), from the visible mesh vs its resting rotation
		public static float DeflectionOf(Transform visible, Quaternion resting)
		{
			Quaternion d = Quaternion.Inverse(resting) * visible.localRotation;
			float a = 2f * Mathf.Atan2(d.x, d.w) * 57.29578f;
			if (a > 180f)
			{
				a -= 360f;
			}
			else if (a < -180f)
			{
				a += 360f;
			}
			return a;
		}
	}
}
