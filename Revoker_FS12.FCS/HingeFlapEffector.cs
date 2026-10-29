using UnityEngine;

namespace Revoker_FS12.FCS
{
	// Revoker 0.2.0: the inboard trailing-edge flaps (flap_L/R) as flaperons. In the game they are not ControlSurfaces: the
	// high-lift device rotates flap_X_visible (the flap AeroPart's lift normal, so the game's aero follows it) 25 deg TE-down
	// below 90-150 m/s, and wing2_X_liftNormal 4 deg with it (stock camber coupling, K 0.16). With the FCS engaged its
	// FlapSystem owns the HLD (HighLiftDevice.FixedUpdate is skipped); this effector drives the flap's hinge directly at the
	// servo speed, about the visible mesh's own right axis, and turns the wing2 section's lift normal by FlapCamberK x the
	// deflection (thin-airfoil host share for the 0.24-chord flap: (0.60 x 5.2 - 1.4) / 3.8 = 0.46). The HLD's high-lift
	// function becomes a symmetric droop target (25 deg x the HLD position) the allocator is drawn toward.
	internal sealed class HingeFlapEffector : Effector
	{
		public readonly AeroPart part;

		public readonly Transform vis;

		private readonly Quaternion visRest;

		public readonly AeroPart host;

		public readonly Transform hostLN;

		private readonly Quaternion hostRest;

		public readonly AirfoilTable foil;

		public readonly AirfoilTable hostFoil;

		public readonly bool left;

		public float hingeNow;

		public float teUp;

		public float servo = 100f;

		public float stallAoA;

		public float localAoANow;

		private Quaternion visRot0;

		private Vector3 v;

		private Vector3 vhat;

		private float v2;

		private float qS;

		private Vector3 com;

		private Vector3 col;

		private float hostK;

		private Quaternion hostRot0;

		private Vector3 hostCom;

		private Vector3 hostCol;

		private Vector3 hostV;

		private float hostQS;

		private Vector3 hingeAxis;

		public bool Valid => part != null && vis != null && part.rb != null && !part.IsDetached();

		public HingeFlapEffector(AeroPart part, Transform vis, AeroPart host, Transform hostLN, AirfoilTable foil, AirfoilTable hostFoil, float range, Quaternion? visRest0, Quaternion? hostRest0)
		{
			this.part = part;
			this.vis = vis;
			this.host = host;
			this.hostLN = hostLN;
			this.foil = foil;
			this.hostFoil = hostFoil;
			// the HLD's retracted pose (the jet may spawn with the HLD out)
			visRest = visRest0 ?? vis.localRotation;
			hostRest = hostRest0 ?? ((hostLN != null) ? hostLN.localRotation : Quaternion.identity);
			left = part.gameObject.name.EndsWith("_L");
			name = part.gameObject.name;
			min = 0f - range;
			max = range;
			lo = min;
			hi = max;
			regNorm = 0f;
			mu = 0.0001f;
			lam = 2E-05f;
			stallAoA = (foil != null) ? foil.StallAoADeg() : 45f;
			AllocGrid(Mathf.Clamp(Mathf.RoundToInt(2f * range) + 1, 9, 31));
			for (int i = 0; i < N; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(N - 1);
			}
		}

		public override void Evaluate(ref EvalContext ctx)
		{
			active = false;
			if (!Valid)
			{
				for (int i = 0; i < N; i++)
				{
					M[i] = Vector3.zero;
				}
				Mnow = Vector3.zero;
				return;
			}
			// the hinge's zero-deflection pose in world (the visible mesh is turned by hingeNow about its local right)
			Transform par = vis.parent;
			visRot0 = ((par != null) ? par.rotation : Quaternion.identity) * visRest;
			com = part.rb.worldCenterOfMass;
			col = Acc.AP_centerOfLift(part);
			v = part.rb.velocity - ctx.wind;
			v2 = v.sqrMagnitude;
			if (v2 < 1f)
			{
				v2 = 0f;
				vhat = Vector3.forward;
			}
			else
			{
				vhat = v / Mathf.Sqrt(v2);
			}
			qS = 0.5f * ctx.rho * v2 * Acc.AP_wingArea(part) * Acc.AP_wingEff(part);
			hingeAxis = visRot0 * Vector3.right;
			hostK = 0f;
			if (host != null && hostLN != null && hostFoil != null && host.rb != null && !host.IsDetached() && Plugin.FlapCamberK > 0f)
			{
				hostK = Plugin.FlapCamberK;
				Transform hp = hostLN.parent;
				hostRot0 = ((hp != null) ? hp.rotation : Quaternion.identity) * hostRest;
				hostCom = host.rb.worldCenterOfMass;
				hostCol = Acc.AP_centerOfLift(host);
				hostV = host.rb.velocity - ctx.wind;
				hostQS = 0.5f * ctx.rho * hostV.sqrMagnitude * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
			}
			if (teUp == 0f && v2 > 100f)
			{
				teUp = (LocalAoAAt(5f) < LocalAoAAt(-5f)) ? 1f : (-1f);
			}
			for (int k = 0; k < N; k++)
			{
				M[k] = Eval(grid[k], ref ctx, out _);
				FN[k] = lastFN;
			}
			Mnow = Eval(hingeNow, ref ctx, out localAoANow);
			now = hingeNow;
			rate = servo;
			active = true;
		}

		private Vector3 Eval(float delta, ref EvalContext ctx, out float localAoADeg)
		{
			lastFN = 0f;
			Quaternion q = visRot0 * Quaternion.AngleAxis(delta, Vector3.right);
			Vector3 vl = Quaternion.Inverse(q) * v;
			float a = Mathf.Atan2(vl.y, vl.z);
			localAoADeg = (0f - a) * 57.29578f;
			if (v2 <= 0f || foil == null)
			{
				return Vector3.zero;
			}
			foil.Get(a, out var CL, out var CD);
			Vector3 n = Vector3.Cross(v, q * Vector3.right);
			float m = n.magnitude;
			if (m < 0.0001f)
			{
				return Vector3.zero;
			}
			Vector3 f = -n / m * (CL * qS) - vhat * (CD * qS);
			Vector3 mT = Vector3.Cross(com + q * col - ctx.cg, f);
			Vector3 fT = f;
			if (hostK > 0f)
			{
				Vector3 dH = Camber.Delta(hostV, hostRot0, hingeAxis, hostK, delta, hostFoil, hostQS);
				mT += Vector3.Cross(hostCom + hostRot0 * hostCol - ctx.cg, dH);
				fT += dH;
			}
			lastFN = (ctx.rootInv * fT).y;
			return ToAero(ctx.rootInv, mT);
		}

		// host lift normal with no camber deflection (the airframe models leave the camber to this effector)
		public Quaternion HostRot0 => (hostLN != null) ? (((hostLN.parent != null) ? hostLN.parent.rotation : Quaternion.identity) * hostRest) : Quaternion.identity;

		// this flap's moment (and its host camber increment) at deflection delta in a given flow, as SurfaceEffector.EvalFlow
		public Vector3 EvalFlow(float delta, Vector3 vFlow, float rho, ref EvalContext ctx)
		{
			float sq = vFlow.sqrMagnitude;
			if (sq < 1f || foil == null || part == null || part.rb == null)
			{
				return Vector3.zero;
			}
			Transform par = vis.parent;
			Quaternion r0 = ((par != null) ? par.rotation : Quaternion.identity) * visRest;
			Quaternion q = r0 * Quaternion.AngleAxis(delta, Vector3.right);
			Vector3 vl = Quaternion.Inverse(q) * vFlow;
			foil.Get(Mathf.Atan2(vl.y, vl.z), out var CL, out var CD);
			Vector3 n = Vector3.Cross(vFlow, q * Vector3.right);
			float m = n.magnitude;
			if (m < 0.0001f)
			{
				return Vector3.zero;
			}
			float qs = 0.5f * rho * sq * Acc.AP_wingArea(part) * Acc.AP_wingEff(part);
			Vector3 f = -n / m * (CL * qs) - vFlow / Mathf.Sqrt(sq) * (CD * qs);
			Vector3 mF = Vector3.Cross(part.rb.worldCenterOfMass + q * Acc.AP_centerOfLift(part) - ctx.cg, f);
			if (host != null && hostLN != null && hostFoil != null && host.rb != null && !host.IsDetached() && Plugin.FlapCamberK > 0f)
			{
				Quaternion h0 = HostRot0;
				float hq = 0.5f * rho * sq * Acc.AP_wingArea(host) * Acc.AP_wingEff(host);
				Vector3 dH = Camber.Delta(vFlow, h0, r0 * Vector3.right, Plugin.FlapCamberK, delta, hostFoil, hq);
				mF += Vector3.Cross(host.rb.worldCenterOfMass + h0 * Acc.AP_centerOfLift(host) - ctx.cg, dH);
			}
			return ToAero(ctx.rootInv, mF);
		}

		// symmetric (pitch) part of the pair's deflection, physical deg (both hinges turn the same way for the same sign)
		public HingeFlapEffector mate;

		public float SymNow => (mate != null && mate.Valid) ? (0.5f * (hingeNow + mate.hingeNow)) : hingeNow;

		public float GeoTeUp(Transform root)
		{
			if (teUp != 0f)
			{
				return teUp;
			}
			if (vis == null || root == null)
			{
				return 0f;
			}
			Quaternion r0 = ((vis.parent != null) ? vis.parent.rotation : Quaternion.identity) * visRest;
			return SurfaceEffector.GeoSign(r0 * Vector3.right, root);
		}

		public float LocalAoAAt(float delta)
		{
			Vector3 vl = Quaternion.Inverse(visRot0 * Quaternion.AngleAxis(delta, Vector3.right)) * v;
			return (0f - Mathf.Atan2(vl.y, vl.z)) * 57.29578f;
		}

		// drive the hinge toward target (deg) at the servo speed; the wing2 section's lift normal turns with it
		public void Drive(float target, float dt)
		{
			if (vis == null)
			{
				return;
			}
			hingeNow = Mathf.MoveTowards(hingeNow, Mathf.Clamp(target, min, max), servo * dt);
			vis.localRotation = visRest * Quaternion.AngleAxis(hingeNow, Vector3.right);
			if (hostLN != null)
			{
				hostLN.localRotation = hostRest * Quaternion.AngleAxis(((host != null && !host.IsDetached()) ? Plugin.FlapCamberK : 0f) * hingeNow, Vector3.right);
			}
		}

		// back to the HLD's rest pose (the FCS let go); the game's HLD takes over again
		public void Release()
		{
			hingeNow = 0f;
			if (vis != null)
			{
				vis.localRotation = visRest;
			}
			if (hostLN != null)
			{
				hostLN.localRotation = hostRest;
			}
		}
	}
}
