// Ifrit port: extra FCS effectors for the stock KR-67's own control devices.
//  SplitEffector  - one per split-flap deceleron (0..maxSplit deg): pure drag at the flap, F = -v^ * split * splitDrag * rho * v^2
//                   (ControlSurface.Aero). The game drives the split from the surface job's yaw input (yawSplitFactor), so the
//                   FCS output is a per-surface yaw value; the throttle==0 airbrake term is suppressed in FCS mode.
//  TvcDiffEffector - differential (roll) nozzle deflection d about the symmetric pitch angle: engine angle = sym + s_e*d, clamped
//                   to the +-20 deg transform limit (Turbojet: thrustVectoring.z, read through controlInputs.roll x gain.z).
using System.Collections.Generic;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal sealed class SplitEffector : Effector
	{
		public readonly ControlSurface cs;
		public readonly AeroPart part;
		public readonly float splitDrag;
		public readonly float yawSign;

		// Ifrit 0.2.2: airbrake out. The job then drives the split to full travel plus the yaw input (stock idle-brake rule),
		// so the effector's variable becomes the retraction from full split (x = max - split): the allocator's pull toward
		// 0 holds the brake, and a retraction on one side is the stock Ifrit's yaw with the brake out.
		public bool brakeMode;

		// physical split (deg) for the effector variable x
		public float Phys(float x) => brakeMode ? (max - x) : x;

		public void SetBrakeMode(bool on)
		{
			if (on == brakeMode)
			{
				return;
			}
			brakeMode = on;
			cmd = max - cmd;
			cmdOut = max - cmdOut;
			cmdLag = max - cmdLag;
			float v0 = outTraj.v;
			outTraj.Reset(max - outTraj.x);
			outTraj.v = 0f - v0;
		}

		public SplitEffector(ControlSurface cs, AeroPart part)
		{
			this.cs = cs;
			this.part = part;
			name = "Split_" + (cs.gameObject.name.EndsWith("_L") ? "L" : "R");
			splitDrag = Acc.CS_splitDrag(cs);
			float ysf = Acc.CS_yawSplitFactor(cs);
			yawSign = Mathf.Sign(ysf == 0f ? 1f : ysf);
			min = 0f;
			max = Mathf.Max(1f, Acc.CS_maxSplit(cs));
			lo = min;
			hi = max;
			regNorm = max;
			mu = Plugin.SplitMu;
			lam = 2E-05f;
			AllocGrid(11);
			for (int i = 0; i < N; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(N - 1);
			}
		}

		public override void Evaluate(ref EvalContext ctx)
		{
			active = false;
			for (int i = 0; i < N; i++)
			{
				M[i] = Vector3.zero;
			}
			Mnow = Vector3.zero;
			if (part == null || part.rb == null || !Acc.JobCreated(cs) || Acc.Job(cs).IsDetached)
			{
				return;
			}
			Vector3 v = part.rb.velocity - ctx.wind;
			float v2 = v.sqrMagnitude;
			if (v2 < 1f)
			{
				return;
			}
			// Ifrit 0.3.0: follow the live split limit (SplitRange is applied at the airframe setup, which can come after this
			// effector was made)
			float maxNow = Mathf.Max(1f, Acc.CS_maxSplit(cs));
			if (Mathf.Abs(maxNow - max) > 0.01f)
			{
				float f0 = cmd / max;
				float f1 = cmdOut / max;
				float f2 = cmdLag / max;
				max = maxNow;
				regNorm = max;
				for (int i = 0; i < N; i++)
				{
					grid[i] = min + (float)i * (max - min) / (float)(N - 1);
				}
				cmd = f0 * max;
				cmdOut = f1 * max;
				cmdLag = f2 * max;
				outTraj.Reset(cmdOut);
			}
			Vector3 vhat = v / Mathf.Sqrt(v2);
			Vector3 arm = part.rb.worldCenterOfMass - ctx.cg;
			for (int k = 0; k < N; k++)
			{
				Vector3 f = -vhat * (Phys(grid[k]) * splitDrag * ctx.rho * v2);
				M[k] = ToAero(ctx.rootInv, Vector3.Cross(arm, f));
			}
			now = Phys(Acc.CS_splitAmount(cs));
			Mnow = MAt(now);
			rate = Mathf.Max(1f, Acc.CS_servo(cs));
			lo = min;
			hi = max;
			active = true;
		}

		// value for the surface job's yaw input that makes the game's split target equal cmd (job: target = (brake ? max : 0)
		// + yawSplitFactor x yaw x max, clamped to [0, max])
		public float JobYaw(float cmd)
		{
			return JobYawPhys(Phys(cmd));
		}

		// job yaw input for a physical split t (deg)
		public float JobYawPhys(float t)
		{
			return Mathf.Clamp((brakeMode ? (t - max) : t) / max * yawSign, -1f, 1f);
		}

		// Ifrit 0.3.1 (DeceleronSplitBudget): the deceleron's halves sit at deflection +- split (the game turns splitUpper /
		// splitLower by +-split on top of the surface's own deflection), so deflection and split share the deceleron's
		// travel: |deflection| + split <= travel. The deflection has priority; this is the split that is left (deg).
		public SurfaceEffector host;

		public float BudgetSplit()
		{
			if (host == null || !Plugin.DeceleronSplitBudget)
			{
				return max;
			}
			float travel = Mathf.Max(host.max, 0f - host.min);
			float d = Mathf.Max(Mathf.Abs(host.cmdOut), Mathf.Abs(host.now));
			return Mathf.Clamp(travel - d, 0f, max);
		}
	}

	internal sealed class TvcDiffEffector : Effector
	{
		public readonly TvcEffector sym;
		private readonly float[] side;   // -1 for the engine whose nozzle goes trailing-edge-down-equivalent for +d

		public TvcDiffEffector(TvcEffector sym)
		{
			this.sym = sym;
			name = "TVCd";
			side = new float[sym.engines.Count];
			float z = 0f;
			for (int i = 0; i < sym.engines.Count; i++)
			{
				float vz = Acc.TF_vec(sym.engines[i]).z;
				z = Mathf.Max(z, Mathf.Abs(vz));
				side[i] = (vz == 0f) ? 0f : -Mathf.Sign(vz);
			}
			min = 0f - Mathf.Max(0.1f, z);
			max = Mathf.Max(0.1f, z);
			lo = min;
			hi = max;
			regNorm = 10f;
			mu = Plugin.TvcDiffMu;
			lam = 2E-05f;
			AllocGrid(13);
			for (int i = 0; i < N; i++)
			{
				grid[i] = min + (float)i * (max - min) / (float)(N - 1);
			}
		}

		public override void Evaluate(ref EvalContext ctx)
		{
			active = false;
			for (int i = 0; i < N; i++)
			{
				M[i] = Vector3.zero;
			}
			Mnow = Vector3.zero;
			float symNow = sym.now;
			Quaternion rootInv = ctx.rootInv;
			Vector3 cgW = ctx.cg;
			float dNowSum = 0f;
			int dn = 0;
			bool flag = false;
			for (int ei = 0; ei < sym.engines.Count; ei++)
			{
				Turbojet engine = sym.engines[ei];
				if (engine == null || !Acc.TF_operable(engine) || side[ei] == 0f)
				{
					continue;
				}
				Aircraft aircraft = Acc.TF_aircraft(engine);
				if ((Object)(object)aircraft != null && aircraft.speed > Acc.TF_vecMaxSpeed(engine))
				{
					continue;
				}
				Transform[] vt = Acc.TF_vecT(engine);
				if (vt == null || vt.Length == 0 || vt[0] == null)
				{
					continue;
				}
				Transform t0 = vt[0];
				Quaternion parentRot = (t0.parent != null) ? t0.parent.rotation : Quaternion.identity;
				Vector3 na = Acc.TF_nozzleAngles(engine);
				dNowSum += (na.x - symNow) * side[ei];
				dn++;
				JetNozzle[] nz = Acc.TF_nozzles(engine);
				if (nz == null)
				{
					continue;
				}
				foreach (JetNozzle jn in nz)
				{
					Transform tt = (jn != null) ? Acc.JN_thrustT(jn) : null;
					if (tt == null)
					{
						continue;
					}
					float th = Acc.JN_totalThrust(jn);
					if (th <= 1f)
					{
						continue;
					}
					Quaternion rot = t0.rotation;
					Quaternion rel = Quaternion.Inverse(rot) * tt.rotation;
					Vector3 off = Quaternion.Inverse(rot) * (tt.position - t0.position);
					Vector3 MAtAngle(float ang)
					{
						Quaternion q3 = parentRot * Quaternion.Euler(Mathf.Clamp(ang, -20f, 20f), na.y, 0f);
						Vector3 dir = q3 * rel * Vector3.forward;
						Vector3 pos = t0.position + q3 * off;
						return ToAero(rootInv, Vector3.Cross(pos - cgW, dir * th));
					}
					Vector3 m0 = MAtAngle(symNow);
					for (int k = 0; k < N; k++)
					{
						M[k] += MAtAngle(symNow + side[ei] * grid[k]) - m0;
					}
					flag = true;
				}
			}
			now = (dn > 0) ? (dNowSum / (float)dn) : 0f;
			// the symmetric TvcEffector's Mnow is computed at the engines' actual (differential) angles; report only the
			// difference to its symmetric model here so sum(Mnow) = sym model + differential model
			Mnow = MAt(now) - (sym.active ? (sym.Mnow - sym.MAt(sym.now)) : Vector3.zero);
			rate = Plugin.TvcSlew;
			active = flag;
			lo = min;
			hi = max;
			if (!active)
			{
				cmd = 0f;
			}
		}

		// controlInputs.roll value that makes Turbojet's roll term deliver cmd (engine 0 sign convention)
		public float RollInput(float d)
		{
			if (sym.engines.Count == 0 || sym.engines[0] == null)
			{
				return 0f;
			}
			Vector3 vz = Acc.TF_vec(sym.engines[0]);
			float g = Acc.TF_vecGain(sym.engines[0]).z;
			if (Mathf.Abs(vz.z) < 0.01f || Mathf.Abs(g) < 0.001f)
			{
				return 0f;
			}
			// Turbojet: n5 -= clamp(roll*g)*vec.z ; engine 0 gets (sym + side0*d) => roll*g*vec.z = -side0*d
			return Mathf.Clamp((0f - side[0]) * d / (g * vz.z), -1f, 1f);
		}
	}
}

