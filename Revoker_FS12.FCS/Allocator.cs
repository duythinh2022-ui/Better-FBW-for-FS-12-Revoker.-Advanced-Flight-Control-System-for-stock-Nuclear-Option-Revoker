using System;
using System.Collections.Generic;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal sealed class Allocator
	{
		public Vector3 axisWeight = new Vector3(1f, 1.5f, 1f);

		public int gnIterations = 4;

		public float regScale = 1f;

		public static double LmFactor = 0.001;

		// 1.8.41 roll-fight cost: sqrt-weight on the roll acceleration each surface group (stabs, flaperons) makes by its
		// own deflection. Splitting the groups against each other (roll cancelled) to buy yaw or anything else costs both
		// groups' roll squared; an ordinary roll costs little because the groups add. 0 = off.
		public float fightK;

		// 2.2.8: per-group weight on the roll-fight cost (index = roll group; 1 stabs, 2 ailerons, 3 flaperons). The deceleron
		// split relaxes groups 2 and 3 only, so the stabs are still kept from rolling against the wing.
		public readonly float[] fightW = new float[] { 1f, 1f, 1f, 1f };

		// 1.8.42 pitch-fight cost: the stabs' and nozzles' own pitch accelerations; whichever group pitches against their
		// net costs pitchFightK x its pitch acceleration squared (stabs rolled trailing-edge up past their roll reversal,
		// nozzles pinned nose-down to cancel it).
		public float pitchFightK;

		// +1 at positive AoA: only 'stabs nose-up, nozzles nose-down' counts as a fight (mirrored at negative AoA)
		public float pitchFightSign = 1f;

		private int[] pgrp = new int[0];

		private double[] y0 = new double[0];

		private double[] Jp = new double[0];

		private readonly double[] pgR = new double[3];

		private readonly bool[] pgOn = new bool[3];

		private double[] r0 = new double[0];

		private int[] grp = new int[0];

		private double[] Jg = new double[0];

		public const int FightGroups = 3;

		// 2.2.16b: reference sign for the roll-fight cost. When the roll the surfaces must add (target less their roll at
		// zero deflection) is beyond FightTargetMin deg/s^2, the groups opposing THAT are the ones penalised; the groups' own
		// net only decides when the target is near zero (steady, or the crossed set with the stick centred). With the net as
		// the reference, a roll stop was fought: the net still had the old roll's sign, so the group reversing first (the
		// stabs) paid the cost and the wing alone stopped the roll (bench, release at 450 km/h: overshoot 1 -> 9 deg/s).
		public static float FightTargetMin = 15f;

		private double fightSignHint;

		private readonly double[] grR = new double[FightGroups + 1];

		private readonly bool[] grOn = new bool[FightGroups + 1];

		private float i00;

		private float i01;

		private float i02;

		private float i10;

		private float i11;

		private float i12;

		private float i20;

		private float i21;

		private float i22;

		private readonly List<Effector> act = new List<Effector>();

		private double[] x = new double[0];

		private double[] prev = new double[0];

		private double[] dx = new double[0];

		private double[,] H = new double[0, 0];

		private double[] rhs = new double[0];

		private Vector3[] Jc = new Vector3[0];

		private int[] pidx = new int[0];

		public void SetInverseInertia(float[,] inv)
		{
			i00 = inv[0, 0];
			i01 = inv[0, 1];
			i02 = inv[0, 2];
			i10 = inv[1, 0];
			i11 = inv[1, 1];
			i12 = inv[1, 2];
			i20 = inv[2, 0];
			i21 = inv[2, 1];
			i22 = inv[2, 2];
		}

		private Vector3 E(Vector3 m)
		{
			float num = i00 * m.x + i01 * m.y + i02 * m.z;
			float num2 = i10 * m.x + i11 * m.y + i12 * m.z;
			return new Vector3(z: (i20 * m.x + i21 * m.y + i22 * m.z) * Mathf.Sqrt(axisWeight.z), x: num * Mathf.Sqrt(axisWeight.x), y: num2 * Mathf.Sqrt(axisWeight.y));
		}

		private float Cost(Vector3 err)
		{
			return E(err).sqrMagnitude;
		}

		private float Reg(Effector e, float c, float p)
		{
			float num = e.RegN;
			float num2 = c / num;
			float num3 = (c - p) / num;
			return regScale * (e.mu * num2 * num2 + e.lam * num3 * num3) + BudgetCost(e, c);
		}

		// Budget term is NOT scaled by regScale (the low-q relief): it has to stay a firm budget at low dynamic
		// pressure, where the stabs still have the authority to take the pitch. The controller fades it instead.
		private static float BudgetCost(Effector e, float c)
		{
			if (e.anchorK <= 0f)
			{
				return 0f;
			}
			float ex = BudgetExcess(e, c) / e.RegN;
			return e.anchorK * ex * ex;
		}

		private static float BudgetExcess(Effector e, float c)
		{
			float d = c - e.anchor;
			if (d > e.band)
			{
				return d - e.band;
			}
			if (d < 0f - e.band)
			{
				return d + e.band;
			}
			return 0f;
		}

		private float PairReg(int i)
		{
			int num = pidx[i];
			if (num < 0)
			{
				return 0f;
			}
			Effector effector = act[i];
			Effector effector2 = act[num];
			float num2 = effector.RegN;
			float num3 = effector.pairSign * (float)x[i] / num2;
			float num4 = effector2.pairSign * (float)x[num] / num2;
			float num5 = 0.5f * (num3 - num4) - 0.5f * (effector.pairSign - effector2.pairSign) * effector.symTarget / num2;
			float num6 = 0.5f * (num3 + num4);
			return regScale * (effector.symCost * num5 * num5 + effector.antiCost * num6 * num6);
		}

		private float PairRegWith(int j, float c)
		{
			double num = x[j];
			x[j] = c;
			float num2 = 0f;
			for (int i = 0; i < act.Count; i++)
			{
				if (pidx[i] >= 0 && (i == j || pidx[i] == j))
				{
					num2 += PairReg(i);
				}
			}
			x[j] = num;
			return num2;
		}

		// 2.2.21: for the stabilators the fight cost sees only the roll of their split, not of their common (pitch)
		// deflection. In sideslip the two stabs see different flow, so a common deflection also rolls the jet; counted as
		// group roll it was priced as a "fight" and the stabs stopped taking pitch with the stick released (19 Oct 22:05 log,
		// fins gone, 2-6 deg of sideslip at 200-330 m/s: stabs frozen for 3-6 s, the nozzles walking to their stop and the
		// zero-rate hold drifting 1-2 deg/s, then a jump to +8 deg/s when the pitch priority overrode it).
		public static bool FightStabSplitOnly = true;

		private bool StabSplitMode(int i)
		{
			return FightStabSplitOnly && grp[i] == 1 && pidx[i] >= 0 && grp[pidx[i]] == 1;
		}

		private double GroupRoll(int g)
		{
			double s = 0.0;
			for (int i = 0; i < act.Count; i++)
			{
				if (grp[i] == g)
				{
					if (StabSplitMode(i))
					{
						int j = pidx[i];
						double sym = 0.5 * ((double)act[i].pairSign * x[i] - (double)act[j].pairSign * x[j]);
						float xs = (float)((double)act[i].pairSign * sym);
						s += (double)i00 * ((double)act[i].MAt((float)x[i]).x - (double)act[i].MAt(xs).x);
						continue;
					}
					s += (double)i00 * ((double)act[i].MAt((float)x[i]).x - r0[i]);
				}
			}
			return s;
		}

		// sign of the horizontal surfaces' net roll: groups rolling against it are the ones penalised
		private double NetSign()
		{
			if (fightSignHint != 0.0)
			{
				return fightSignHint;
			}
			double t = 0.0;
			for (int g = 1; g <= FightGroups; g++)
			{
				t += GroupRoll(g);
			}
			return (t >= 0.0) ? 1.0 : (-1.0);
		}

		private float FightCost()
		{
			if (fightK <= 0f)
			{
				return 0f;
			}
			double sg = NetSign();
			double t = 0.0;
			for (int g = 1; g <= FightGroups; g++)
			{
				double a = GroupRoll(g);
				if (a * sg < 0.0)
				{
					t += (double)fightW[g] * a * a;
				}
			}
			return (float)((double)fightK * t);
		}

		// fight cost with effector j at deflection c, others at x
		private float FightCostWith(int j, float c)
		{
			if (fightK <= 0f || grp[j] == 0)
			{
				return 0f;
			}
			double d = (double)i00 * ((double)act[j].MAt(c).x - (double)act[j].MAt((float)x[j]).x);
			double tot = 0.0;
			bool splitJ = StabSplitMode(j);
			double xj = x[j];
			if (splitJ)
			{
				x[j] = c;
			}
			for (int g = 1; g <= FightGroups; g++)
			{
				grR[g] = GroupRoll(g) + ((g == grp[j] && !splitJ) ? d : 0.0);
				tot += grR[g];
			}
			x[j] = xj;
			double sg = (fightSignHint != 0.0) ? fightSignHint : ((tot >= 0.0) ? 1.0 : (-1.0));
			double t = 0.0;
			for (int g = 1; g <= FightGroups; g++)
			{
				if (grR[g] * sg < 0.0)
				{
					t += (double)fightW[g] * grR[g] * grR[g];
				}
			}
			return (float)((double)fightK * t);
		}

		private double GroupPitch(int g)
		{
			double s = 0.0;
			for (int i = 0; i < act.Count; i++)
			{
				if (pgrp[i] == g)
				{
					s += (double)i11 * ((double)act[i].MAt((float)x[i]).y - y0[i]);
				}
			}
			return s;
		}

		// a: stabs' pitch, b: nozzles' pitch (rad/s^2). Only the stabs-nose-up / nozzles-nose-down pattern counts; in it,
		// the group opposing the pair's net pitch is penalised.
		private double PFTerm(double a, double b)
		{
			double k = pitchFightSign;
			if (a * k <= 0.0 || b * k >= 0.0)
			{
				return 0.0;
			}
			double sg = (a + b >= 0.0) ? 1.0 : (-1.0);
			return ((a * sg < 0.0) ? (a * a) : 0.0) + ((b * sg < 0.0) ? (b * b) : 0.0);
		}

		private float PitchFightCost()
		{
			if (pitchFightK <= 0f)
			{
				return 0f;
			}
			return (float)((double)pitchFightK * PFTerm(GroupPitch(1), GroupPitch(2)));
		}

		private float PFPair(int s, float cs, int t, float ct)
		{
			if (pitchFightK <= 0f)
			{
				return 0f;
			}
			double xs = x[s];
			double xt = x[t];
			x[s] = cs;
			x[t] = ct;
			float r = PitchFightCost();
			x[s] = xs;
			x[t] = xt;
			return r;
		}

		private float PitchFightCostWith(int j, float c)
		{
			if (pitchFightK <= 0f || pgrp[j] == 0)
			{
				return 0f;
			}
			double d = (double)i11 * ((double)act[j].MAt(c).y - (double)act[j].MAt((float)x[j]).y);
			double a = GroupPitch(1) + ((pgrp[j] == 1) ? d : 0.0);
			double b = GroupPitch(2) + ((pgrp[j] == 2) ? d : 0.0);
			return (float)((double)pitchFightK * PFTerm(a, b));
		}

		private float Total(Vector3 target)
		{
			Vector3 zero = Vector3.zero;
			float num = 0f;
			for (int i = 0; i < act.Count; i++)
			{
				zero += act[i].MAt((float)x[i]);
				num += Reg(act[i], (float)x[i], (float)prev[i]) + PairReg(i);
			}
			return Cost(zero - target) + num + FightCost() + PitchFightCost();
		}

		private void Ensure(int n)
		{
			if (x.Length != n)
			{
				x = new double[n];
				prev = new double[n];
				dx = new double[n];
				H = new double[n, n];
				rhs = new double[n];
				Jc = new Vector3[n];
				pidx = new int[n];
				r0 = new double[n];
				grp = new int[n];
				Jg = new double[n];
				pgrp = new int[n];
				y0 = new double[n];
				Jp = new double[n];
			}
		}

		public void Solve(List<Effector> effs, Vector3 target)
		{
			act.Clear();
			foreach (Effector eff in effs)
			{
				if (eff.active)
				{
					act.Add(eff);
				}
			}
			int count = act.Count;
			if (count == 0)
			{
				return;
			}
			Ensure(count);
			for (int i = 0; i < count; i++)
			{
				x[i] = Mathf.Clamp(act[i].cmd, act[i].lo, act[i].hi);
				prev[i] = act[i].cmd;
			}
			for (int j = 0; j < count; j++)
			{
				pidx[j] = ((act[j].partner != null) ? act.IndexOf(act[j].partner) : (-1));
				grp[j] = ((fightK > 0f) ? act[j].rollGroup : 0);
				r0[j] = act[j].MAt(0f).x;
				pgrp[j] = ((pitchFightK > 0f) ? act[j].pitchGroup : 0);
				y0[j] = act[j].MAt(0f).y;
			}
			fightSignHint = 0.0;
			if (fightK > 0f && FightTargetMin > 0f)
			{
				double tr = (double)i00 * (double)target.x;
				for (int jh = 0; jh < count; jh++)
				{
					tr -= (double)i00 * r0[jh];
				}
				if (Math.Abs(tr * 57.29578) > FightTargetMin)
				{
					fightSignHint = (tr >= 0.0) ? 1.0 : (-1.0);
				}
			}
			float num = Total(target);
			for (int k = 0; k < gnIterations; k++)
			{
				Vector3 zero = Vector3.zero;
				for (int l = 0; l < count; l++)
				{
					zero += act[l].MAt((float)x[l]);
				}
				Vector3 vector = E(zero - target);
				for (int m = 0; m < count; m++)
				{
					Effector effector = act[m];
					float num2 = 0.5f * effector.Step;
					float num3 = Mathf.Max(effector.min, (float)x[m] - num2);
					float num4 = Mathf.Min(effector.max, (float)x[m] + num2);
					Jc[m] = ((num4 > num3) ? (E(effector.MAt(num4) - effector.MAt(num3)) / (num4 - num3)) : Vector3.zero);
					Jp[m] = ((pgrp[m] != 0 && num4 > num3) ? (Math.Sqrt(pitchFightK) * (double)i11 * (double)(effector.MAt(num4).y - effector.MAt(num3).y) / (double)(num4 - num3)) : 0.0);
					Jg[m] = ((grp[m] != 0 && num4 > num3) ? (Math.Sqrt(fightK * fightW[grp[m]]) * (double)i00 * (double)(effector.MAt(num4).x - effector.MAt(num3).x) / (double)(num4 - num3)) : 0.0);
					if (grp[m] != 0 && num4 > num3 && StabSplitMode(m))
					{
						double xm = x[m];
						x[m] = num4;
						double g4 = GroupRoll(grp[m]);
						x[m] = num3;
						double g3 = GroupRoll(grp[m]);
						x[m] = xm;
						Jg[m] = Math.Sqrt(fightK * fightW[grp[m]]) * (g4 - g3) / (double)(num4 - num3);
					}
				}
				if (pitchFightK > 0f)
				{
					double pa = GroupPitch(1);
					double pb = GroupPitch(2);
					double sgP = (pa + pb >= 0.0) ? 1.0 : (-1.0);
					bool pat = pa * (double)pitchFightSign > 0.0 && pb * (double)pitchFightSign < 0.0;
					pgOn[1] = pat && pa * sgP < 0.0;
					pgOn[2] = pat && pb * sgP < 0.0;
					pgR[1] = pgOn[1] ? (Math.Sqrt(pitchFightK) * pa) : 0.0;
					pgR[2] = pgOn[2] ? (Math.Sqrt(pitchFightK) * pb) : 0.0;
				}
				else
				{
					pgOn[1] = (pgOn[2] = false);
				}
				if (fightK > 0f)
				{
					double sgF = NetSign();
					for (int g = 1; g <= FightGroups; g++)
					{
						double gr = GroupRoll(g);
						grOn[g] = gr * sgF < 0.0;
						grR[g] = grOn[g] ? (Math.Sqrt(fightK * fightW[g]) * gr) : 0.0;
					}
				}
				for (int n = 0; n < count; n++)
				{
					Effector effector2 = act[n];
					float num5 = effector2.RegN;
					double num6 = 1.0 / (double)(num5 * num5);
					for (int num7 = 0; num7 < count; num7++)
					{
						H[n, num7] = Vector3.Dot(Jc[n], Jc[num7]);
						if (pgrp[n] != 0 && pgrp[n] == pgrp[num7] && pgOn[pgrp[n]])
						{
							H[n, num7] += Jp[n] * Jp[num7];
						}
						if (grp[n] != 0 && grp[n] == grp[num7] && grOn[grp[n]])
						{
							H[n, num7] += Jg[n] * Jg[num7];
						}
					}
					double num8 = LmFactor * H[n, n] + 1E-09;
					double num9 = regScale;
					H[n, n] += num9 * (double)(effector2.mu + effector2.lam) * num6 + num8;
					rhs[n] = 0.0 - ((double)Vector3.Dot(Jc[n], vector) + num9 * ((double)effector2.mu * num6 * x[n] + (double)effector2.lam * num6 * (x[n] - prev[n])));
					if (grp[n] != 0 && grOn[grp[n]])
					{
						rhs[n] -= Jg[n] * grR[grp[n]];
					}
					if (pgrp[n] != 0 && pgOn[pgrp[n]])
					{
						rhs[n] -= Jp[n] * pgR[pgrp[n]];
					}
					if (effector2.anchorK > 0f)
					{
						float bx = BudgetExcess(effector2, (float)x[n]);
						if (bx != 0f)
						{
							H[n, n] += (double)effector2.anchorK * num6;
							rhs[n] -= (double)effector2.anchorK * num6 * (double)bx;
						}
					}
				}
				for (int num10 = 0; num10 < count; num10++)
				{
					int num11 = pidx[num10];
					if (num11 >= 0)
					{
						Effector effector3 = act[num10];
						Effector effector4 = act[num11];
						float num12 = effector3.RegN;
						double num13 = 1.0 / (double)(num12 * num12);
						double num14 = effector3.pairSign;
						double num15 = effector4.pairSign;
						double num16 = 0.25 * (double)regScale * (double)effector3.symCost * num13;
						double num17 = 0.25 * (double)regScale * (double)effector3.antiCost * num13;
						double num18 = num14 * x[num10] - num15 * x[num11] - (num14 - num15) * (double)effector3.symTarget;
						double num19 = num14 * x[num10] + num15 * x[num11];
						H[num10, num10] += num16 + num17;
						H[num11, num11] += num16 + num17;
						H[num10, num11] += (0.0 - num16 + num17) * num14 * num15;
						H[num11, num10] += (0.0 - num16 + num17) * num14 * num15;
						rhs[num10] -= num16 * num14 * num18 + num17 * num14 * num19;
						rhs[num11] -= (0.0 - num16) * num15 * num18 + num17 * num15 * num19;
					}
				}
				if (!SolveLinear(H, rhs, dx, count))
				{
					break;
				}
				double num20 = 1.0;
				bool flag = false;
				double[] array = new double[count];
				int num21 = 0;
				while (num21 < 4 && !flag)
				{
					for (int num22 = 0; num22 < count; num22++)
					{
						array[num22] = x[num22];
					}
					for (int num23 = 0; num23 < count; num23++)
					{
						x[num23] = Mathf.Clamp((float)(array[num23] + num20 * dx[num23]), act[num23].lo, act[num23].hi);
					}
					float num24 = Total(target);
					if (num24 < num)
					{
						num = num24;
						flag = true;
					}
					else
					{
						for (int num25 = 0; num25 < count; num25++)
						{
							x[num25] = array[num25];
						}
					}
					num21++;
					num20 *= 0.5;
				}
				if (!flag)
				{
					break;
				}
			}
			Vector3 vector2 = Vector3.zero;
			for (int num26 = 0; num26 < count; num26++)
			{
				vector2 += act[num26].MAt((float)x[num26]);
			}
			for (int num27 = 0; num27 < count; num27++)
			{
				Effector effector5 = act[num27];
				float num28 = (float)x[num27];
				float p = (float)prev[num27];
				Vector3 vector3 = vector2 - effector5.MAt(num28);
				Vector3 vector4 = target - vector3;
				float num29 = Cost(effector5.MAt(num28) - vector4) + Reg(effector5, num28, p) + PairRegWith(num27, num28) + FightCostWith(num27, num28) + PitchFightCostWith(num27, num28);
				float num30 = num28;
				int num31 = -1;
				for (int num32 = 0; num32 < effector5.N; num32++)
				{
					float num33 = effector5.grid[num32];
					if (!(num33 < effector5.lo - 0.0001f) && !(num33 > effector5.hi + 0.0001f))
					{
						float num34 = Cost(effector5.M[num32] - vector4) + Reg(effector5, num33, p) + PairRegWith(num27, num33) + FightCostWith(num27, num33) + PitchFightCostWith(num27, num33);
						if (num34 < num29 - 1E-09f)
						{
							num29 = num34;
							num30 = num33;
							num31 = num32;
						}
					}
				}
				if (num31 > 0 && num31 < effector5.N - 1)
				{
					float c = effector5.grid[num31 - 1];
					float c2 = effector5.grid[num31 + 1];
					float num35 = Cost(effector5.M[num31 - 1] - vector4) + Reg(effector5, c, p) + PairRegWith(num27, c) + FightCostWith(num27, c) + PitchFightCostWith(num27, c);
					float num36 = Cost(effector5.M[num31 + 1] - vector4) + Reg(effector5, c2, p) + PairRegWith(num27, c2) + FightCostWith(num27, c2) + PitchFightCostWith(num27, c2);
					float num37 = num35 - 2f * num29 + num36;
					if (num37 > 1E-12f)
					{
						float num38 = Mathf.Clamp(effector5.grid[num31] + Mathf.Clamp(0.5f * (num35 - num36) / num37, -0.5f, 0.5f) * effector5.Step, effector5.lo, effector5.hi);
						float num39 = Cost(effector5.MAt(num38) - vector4) + Reg(effector5, num38, p) + PairRegWith(num27, num38) + FightCostWith(num27, num38) + PitchFightCostWith(num27, num38);
						if (num39 < num29)
						{
							num29 = num39;
							num30 = num38;
						}
					}
				}
				// Ifrit 0.4.9 (F-22E 2.4.7): a local Newton candidate. The pass above only tries the effector's grid points (the nozzles': every
				// 4 deg) and a parabola through the best of them, so an effector sitting between two points stays put when
				// neither point beats where it is, however large the miss. The Gauss-Newton steps before it are joint: with
				// the stabs on their stall plateau and a large yaw miss the line search rejects them all, and the nozzles,
				// the one pitch effector with a gradient, were never moved (bench, corkscrew release at 64 deg: nozzles
				// frozen 0.15 s at -3.8 deg with a 2-10 deg/s^2 nose-down pitch miss; the hold sagged to q -2.5).
				if (CoordNewton == 2 || (CoordNewton == 1 && (effector5 is TvcEffector || effector5 is TvcYawEffector)))
				{
					float xc = (float)x[num27];
					float hN = 0.25f * effector5.Step;
					float aN = Mathf.Max(effector5.lo, xc - hN);
					float bN = Mathf.Min(effector5.hi, xc + hN);
					if (bN > aN + 1E-05f)
					{
						Vector3 gN = E(effector5.MAt(bN) - effector5.MAt(aN)) / (bN - aN);
						Vector3 rN = E(effector5.MAt(xc) - vector4);
						float rn2 = effector5.RegN * effector5.RegN;
						double denN = (double)Vector3.Dot(gN, gN) + (double)(regScale * (effector5.mu + effector5.lam) / rn2);
						if (denN > 1E-12)
						{
							double grN = (double)Vector3.Dot(gN, rN) + (double)(regScale * (effector5.mu * xc + effector5.lam * (xc - p)) / rn2);
							float cN = Mathf.Clamp((float)((double)xc - grN / denN), effector5.lo, effector5.hi);
							float costN = Cost(effector5.MAt(cN) - vector4) + Reg(effector5, cN, p) + PairRegWith(num27, cN) + FightCostWith(num27, cN) + PitchFightCostWith(num27, cN);
							if (costN < num29)
							{
								num29 = costN;
								num30 = cN;
								coordNewtonMoves++;
							}
						}
					}
				}
				x[num27] = num30;
				vector2 = vector3 + effector5.MAt(num30);
			}
			if (PairedTrimSearch)
			{
				PairedSearch(target, ref vector2);
			}
			for (int num40 = 0; num40 < count; num40++)
			{
				act[num40].cmd = Mathf.Clamp((float)x[num40], act[num40].lo, act[num40].hi);
			}
		}

		public static int CoordNewton = 1;

		public int coordNewtonMoves;

		public static bool RegOrigRange = true;

		public bool PairedTrimSearch;

		public static float PairHorizon = 0.1f;

		public int pairedMoves;

		private void PairedSearch(Vector3 target, ref Vector3 total)
		{
			int it = -1;
			for (int i = 0; i < act.Count; i++)
			{
				if (act[i] is TvcEffector)
				{
					it = i;
					break;
				}
			}
			if (it < 0)
			{
				return;
			}
			Effector tv = act[it];
			if (tv.hi - tv.lo < 0.05f)
			{
				return;
			}
			Vector3 tLo = tv.MAt(tv.lo);
			Vector3 tHi = tv.MAt(tv.hi);
			float kY = (tHi.y - tLo.y) / (tv.hi - tv.lo);
			if (Mathf.Abs(kY) < 1E-06f)
			{
				return;
			}
			for (int s = 0; s < act.Count; s++)
			{
				if (!(act[s] is SurfaceEffector surfaceEffector) || surfaceEffector.kind != SurfaceKind.Stabilator)
				{
					continue;
				}
				float xs = (float)x[s];
				float xt = (float)x[it];
				Vector3 mS = surfaceEffector.MAt(xs);
				Vector3 mT = tv.MAt(xt);
				Vector3 others = total - mS - mT;
				float best = Cost(total - target) + Reg(surfaceEffector, xs, (float)prev[s]) + PairRegWith(s, xs) + Reg(tv, xt, (float)prev[it]) + FightCostWith(s, xs) + PFPair(s, xs, it, xt);
				float bs = xs;
				float bt = xt;
				Vector3 bMS = mS;
				Vector3 bMT = mT;
				for (int g = 0; g < surfaceEffector.N; g++)
				{
					float gs = surfaceEffector.grid[g];
					if (gs < surfaceEffector.lo - 0.0001f || gs > surfaceEffector.hi + 0.0001f || Mathf.Abs(gs - surfaceEffector.now) > Mathf.Max(surfaceEffector.Step, PairHorizon * Mathf.Max(1f, surfaceEffector.rate)))
					{
						continue;
					}
					Vector3 ms = surfaceEffector.M[g];
					float gt = Mathf.Clamp(xt + (mS.y - ms.y) / kY, tv.lo, tv.hi);
					Vector3 mt = tv.MAt(gt);
					float c = Cost(others + ms + mt - target) + Reg(surfaceEffector, gs, (float)prev[s]) + PairRegWith(s, gs) + Reg(tv, gt, (float)prev[it]) + FightCostWith(s, gs) + PFPair(s, gs, it, gt);
					if (c < best - 1E-09f)
					{
						best = c;
						bs = gs;
						bt = gt;
						bMS = ms;
						bMT = mt;
					}
				}
				if (bs != xs)
				{
					x[s] = bs;
					x[it] = bt;
					total = others + bMS + bMT;
					pairedMoves++;
				}
			}
		}

		private static bool SolveLinear(double[,] A0, double[] b0, double[] outX, int n)
		{
			double[,] array = (double[,])A0.Clone();
			double[] array2 = (double[])b0.Clone();
			for (int i = 0; i < n; i++)
			{
				int num = i;
				double num2 = Math.Abs(array[i, i]);
				for (int j = i + 1; j < n; j++)
				{
					if (Math.Abs(array[j, i]) > num2)
					{
						num2 = Math.Abs(array[j, i]);
						num = j;
					}
				}
				if (num2 < 1E-18)
				{
					return false;
				}
				if (num != i)
				{
					for (int k = 0; k < n; k++)
					{
						double num3 = array[i, k];
						array[i, k] = array[num, k];
						array[num, k] = num3;
					}
					double num4 = array2[i];
					array2[i] = array2[num];
					array2[num] = num4;
				}
				for (int l = i + 1; l < n; l++)
				{
					double num5 = array[l, i] / array[i, i];
					if (num5 != 0.0)
					{
						for (int m = i; m < n; m++)
						{
							array[l, m] -= num5 * array[i, m];
						}
						array2[l] -= num5 * array2[i];
					}
				}
			}
			for (int num6 = n - 1; num6 >= 0; num6--)
			{
				double num7 = array2[num6];
				for (int num8 = num6 + 1; num8 < n; num8++)
				{
					num7 -= array[num6, num8] * outX[num8];
				}
				outX[num6] = num7 / array[num6, num6];
			}
			return true;
		}
	}
}
