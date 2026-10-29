using UnityEngine;

namespace Revoker_FS12.FCS
{
	public static class FcsApi
	{
		// 4 (Revoker 0.2.5): discovery contract for tools (flight-data HUD, FCS visualizer), additions only. A tool finds every
		// public static class named FcsApi in any loaded plugin and asks each about the local aircraft; nothing in the tool
		// names a particular FCS mod. Members:
		//   ApiId        "duythinh2022.fcs" - marks a class that follows this contract (v4+)
		//   Version      contract version (int)
		//   FcsName      display name of this FCS ("FS-12 Revoker FCS")
		//   FcsVersion   plugin version ("0.2.5")
		//   Owns(Aircraft)             true when this FCS is attached to that aircraft (engaged or not)
		//   GetController(Aircraft)    that aircraft's controller object (its fields are what the visualizer reads), or null
		//   GetPitchCommand(Aircraft, float[10])   unchanged since v3
		public const string ApiId = "duythinh2022.fcs";

		public const string FcsName = "FS-12 Revoker FCS";

		public static readonly string FcsVersion = FcsStandalonePlugin.Version;

		public static bool Owns(Aircraft aircraft)
		{
			return Registry.Find(aircraft) != null;
		}

		public static object GetController(Aircraft aircraft)
		{
			return Registry.Find(aircraft);
		}

		public const int Version = 4; // 2 (1.9.1): code 7 = high-AoA pitch-rate command, v[3] = commanded pitch rate
		// 3 (2.1.0): code 8 = pitch-rate command of the G + pitch-rate law (PitchCommandLaw = GAndPitchRate, full G out of
		// reach); code 4 = its G command. v[1] (AoA command) is not meaningful under that law (it tracks the AoA).

		public static bool GetPitchCommand(Aircraft aircraft, float[] v)
		{
			if (v == null || v.Length < 10)
			{
				return false;
			}
			FcsController fcsController = Registry.Find(aircraft);
			if (fcsController == null || fcsController.Failed || !fcsController.Engaged)
			{
				return false;
			}
			v[0] = fcsController.Mode switch
			{
				FcsMode.Ground => 1f, 
				FcsMode.GearDownRate => 2f, 
				FcsMode.AoAG => (Mathf.Abs(fcsController.sp) < 0.04f && !fcsController.prot) ? 6f : (Plugin.GRateLaw ? ((fcsController.betaG < 0.5f) ? 8f : 4f) : ((fcsController.hiRate && fcsController.rateW > 0.5f) ? 7f : (fcsController.capOn ? 8f : ((fcsController.betaG < 0.5f) ? 3f : 4f)))), 
				FcsMode.MPO => 5f, 
				FcsMode.AIRate => 2f, 
				FcsMode.Rate => 2f, 
				_ => 0f, 
			};
			v[1] = fcsController.alphaCmd;
			v[2] = fcsController.nCmd;
			v[3] = fcsController.qT;
			v[4] = fcsController.alpha;
			v[5] = fcsController.nz;
			v[6] = (fcsController.prot ? 1f : 0f);
			v[7] = (fcsController.unload ? 1f : 0f);
			v[8] = Plugin.AoAPos;
			v[9] = ((fcsController.Mode == FcsMode.MPO) ? Plugin.MpoGPos : Plugin.GPos);
			return true;
		}
	}
}
