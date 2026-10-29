using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal struct Lpf3
	{
		public Vector3 y;

		public bool init;

		public Vector3 Step(Vector3 x, float tau, float dt)
		{
			if (!init)
			{
				y = x;
				init = true;
				return y;
			}
			y += (x - y) * (dt / (tau + dt));
			return y;
		}
	}
}
