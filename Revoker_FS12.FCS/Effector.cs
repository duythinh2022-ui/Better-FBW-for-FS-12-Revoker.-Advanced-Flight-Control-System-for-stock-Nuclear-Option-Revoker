using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal abstract class Effector
	{
		public string name;

		public bool active;

		public float min;

		public float max;

		public float now;

		public float cmd;

		public float cmdOut;

		public float cmdLag;

		public RateTraj outTraj;

		public float rate;

		// 2.41.0: the allocator balances the other effectors against where this one will be within reachTau s (its servo's
		// reach toward its command), not against the command itself. 0 = not used.
		public float reachTau;

		// 2.42 (Allocator.TwoHorizon): this effector's reach plan is scored against its destination plan
		public bool twoHorizon;

		public float mu = 0.0001f;

		public float lam = 2E-05f;

		public float lo;

		public float hi;

		public Effector partner;

		public float pairSign = 1f;

		public float symCost;

		// 2.0.6: symmetric-part target (deg, the common deflection both pair members are pulled toward; rudder toe)
		public float symTarget;

		public float regNorm;

		public float RegN => (regNorm > 0f && Allocator.RegOrigRange) ? regNorm : Mathf.Max(1f, max);

		public float antiCost;

		// 1.8.32 nozzle budget: beyond +-band of anchor, each degree costs anchorK x (deg/RegN)^2 on top of mu/lam.
		public float anchor;

		public float band;

		public float anchorK;

		public int N;

		public float[] grid;

		public Vector3[] M;

		// Revoker 0.2.9: normal force (body up, N) at each grid point, and of the last Eval (TrailingEdgeBrakeConfig lift balance)
		public float[] FN;

		public float lastFN;

		public Vector3 Mnow;

		// 1.8.41: roll-fight group for the Allocator: 1 = stabilators, 2 = outboard flaperons, 3 = inboard flaperons, 0 = none
		public int rollGroup;

		// 1.8.42: pitch-fight group: 1 = stabilators, 2 = TVC, 0 = none
		public int pitchGroup;

		public float Step => (max - min) / (float)(N - 1);

		protected void AllocGrid(int n)
		{
			N = n;
			grid = new float[n];
			M = new Vector3[n];
			FN = new float[n];
		}

		public Vector3 MAt(float c)
		{
			if (N < 2)
			{
				return Vector3.zero;
			}
			float num = (c - min) / Step;
			if (num <= 0f)
			{
				return M[0];
			}
			if (num >= (float)(N - 1))
			{
				return M[N - 1];
			}
			int num2 = (int)num;
			float num3 = num - (float)num2;
			Vector3 m = M[num2] + (M[num2 + 1] - M[num2]) * num3;
			// Revoker 0.8.2 (AnchorLateral): the live point as an extra node of the table, between its two grid neighbours
			if (nodeOn && num2 == nodeK)
			{
				float g0 = min + (float)num2 * Step;
				float w = (c <= nodeC) ? ((nodeC > g0) ? ((c - g0) / (nodeC - g0)) : 1f) : ((g0 + Step > nodeC) ? ((g0 + Step - c) / (g0 + Step - nodeC)) : 1f);
				m += nodeD * w;
			}
			return m;
		}

		// Revoker 0.8.2 (AnchorLateral): INDI builds its target on the live moment (sum Mnow); the allocator answers from this table,
		// linear between grid points. Where the two disagree at the surface's position, the difference was added to every tick's
		// target as if the pilot had asked for it, and the rate loop settled with a standing error of (mismatch) / (I K): a slow
		// roll / yaw creep with the stick centred whenever a trim puts the left and right surfaces at different deflections (an
		// engine out, a load, damage), where their errors stop cancelling. The surface sets its live point as a node of the roll
		// and yaw columns: the table passes through the live moment there and is unchanged at every grid point, so it stays the
		// table everywhere else (a whole-table shift, Ifrit 0.8.3, carried the error at 'now' to deflections far from it).
		public bool nodeOn;

		public int nodeK;

		public float nodeC;

		public Vector3 nodeD;

		public void SetLiveNode(float c, Vector3 live)
		{
			nodeOn = false;
			if (N < 2)
			{
				return;
			}
			float u = (c - min) / Step;
			if (u <= 0f || u >= (float)(N - 1))
			{
				return;
			}
			Vector3 at = MAt(c);
			nodeK = (int)u;
			nodeC = c;
			nodeD = new Vector3(live.x - at.x, 0f, live.z - at.z);
			nodeOn = true;
		}

		public float FNAt(float c)
		{
			if (N < 2 || FN == null)
			{
				return 0f;
			}
			float num = (c - min) / Step;
			if (num <= 0f)
			{
				return FN[0];
			}
			if (num >= (float)(N - 1))
			{
				return FN[N - 1];
			}
			int num2 = (int)num;
			return FN[num2] + (FN[num2 + 1] - FN[num2]) * (num - (float)num2);
		}

		public abstract void Evaluate(ref EvalContext ctx);

		protected static Vector3 ToAero(Quaternion rootInv, Vector3 worldMoment)
		{
			Vector3 vector = rootInv * worldMoment;
			return new Vector3(0f - vector.z, 0f - vector.x, vector.y);
		}
	}
}
