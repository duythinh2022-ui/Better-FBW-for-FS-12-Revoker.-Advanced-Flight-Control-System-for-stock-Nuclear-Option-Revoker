using UnityEngine;

namespace Revoker_FS12.FCS
{
	// Revoker 0.1.0: the FS-12's single nozzle vectors in pitch AND yaw (Turbojet.thrustVectoring = (15, 15, 0)): the
	// yaw angle is nozzleAngles.y = clamp(-controlInputs.yaw, -1, 1) x vec.y, slewed at 70 deg/s, zeroed below 3 m radar
	// altitude and (with pitch) above thrustVectoringMaxAirspeed. This effector is the yaw angle about the current pitch
	// angle. The pitch TvcEffector's grid is taken at yaw 0 (TvcEffector.yawSeparate), so the model is
	// tvc.M(pitch) + tvcy.M(yaw), with tvcy.M(0) = 0.
	internal sealed class TvcYawEffector : Effector
	{
		public readonly TvcEffector sym;

		public TvcYawEffector(TvcEffector sym)
		{
			this.sym = sym;
			name = "TVCy";
			float y = 0f;
			foreach (Turbojet e in sym.engines)
			{
				y = Mathf.Max(y, Mathf.Abs(Acc.TF_vec(e).y));
			}
			min = 0f - Mathf.Max(0.1f, y);
			max = Mathf.Max(0.1f, y);
			lo = min;
			hi = max;
			regNorm = 10f;
			mu = Plugin.TvcYawMu;
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
			if (Plugin.DisableTVC)
			{
				now = 0f;
				lo = 0f;
				hi = 0f;
				cmd = 0f;
				return;
			}
			float yNowSum = 0f;
			int yn = 0;
			bool flag = false;
			foreach (Turbojet engine in sym.engines)
			{
				if (engine == null || !Acc.TF_operable(engine) || Mathf.Abs(Acc.TF_vec(engine).y) < 0.01f)
				{
					continue;
				}
				Aircraft aircraft = Acc.TF_aircraft(engine);
				// the game zeroes the yaw angle under 3 m radar altitude and both angles above the vectoring speed
				if ((Object)(object)aircraft != null && (aircraft.speed > Acc.TF_vecMaxSpeed(engine) || aircraft.radarAlt < 3f))
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
				yNowSum += na.y;
				yn++;
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
					float px = Mathf.Clamp(na.x + na.z, -20f, 20f);
					Quaternion rootInv = ctx.rootInv;
					Vector3 cgW = ctx.cg;
					Vector3 MAtAngle(float yaw)
					{
						Quaternion q3 = parentRot * Quaternion.Euler(px, yaw, 0f);
						Vector3 dir = q3 * rel * Vector3.forward;
						Vector3 pos = t0.position + q3 * off;
						return ToAero(rootInv, Vector3.Cross(pos - cgW, dir * th));
					}
					Vector3 m0 = MAtAngle(0f);
					for (int k = 0; k < N; k++)
					{
						M[k] += MAtAngle(grid[k]) - m0;
					}
					flag = true;
				}
			}
			now = (yn > 0) ? (yNowSum / (float)yn) : 0f;
			// TvcEffector.Mnow is at the nozzle's actual angles (pitch and yaw): report only the part its pitch model at yaw 0
			// leaves out, so sum(Mnow) = tvc.M(pitch) + tvcy.M(yaw)
			Mnow = MAt(now) - (sym.active ? (sym.Mnow - sym.MAt(sym.now)) : Vector3.zero);
			rate = Plugin.TvcSlew;
			active = flag;
			lo = min * sym.speedFade;
			hi = max * sym.speedFade;
			if (!active)
			{
				cmd = 0f;
			}
		}

		// controlInputs.yaw that makes Turbojet's yaw term (nozzleAngles.y = clamp(-yaw) x vec.y) deliver y deg
		public float YawInput(float y)
		{
			if (sym.engines.Count == 0 || sym.engines[0] == null)
			{
				return 0f;
			}
			float vy = Acc.TF_vec(sym.engines[0]).y;
			if (Mathf.Abs(vy) < 0.01f)
			{
				return 0f;
			}
			return Mathf.Clamp((0f - y) / vy, -1f, 1f);
		}
	}
}
