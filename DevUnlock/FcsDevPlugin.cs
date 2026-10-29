using System;
using System.IO;
using BepInEx;
using UnityEngine;

namespace Revoker_FS12.FCS.Dev
{
	// Dev/debug unlock for the FS-12 Revoker FCS. When it is loaded, the FCS binds every setting other than the player ones to
	// BepInEx/config/Revoker_FS12.FCS.Dev.cfg and shows them in the ConfigurationManager (F1).
	// 0.2.3: once, it also copies the Revoker's wing / trailing-edge mesh data (8 meshes, 211,968 bytes) out of
	// resources.assets.resS into BepInEx/Revoker_mesh.bin, so the hinge lines can be measured offline. Read-only on the game.
	[BepInPlugin("Revoker_FS12.FCS.Dev", "FS-12 Revoker FCS - Dev/Debug settings", "0.2.9")]
	public class FcsDevPlugin : BaseUnityPlugin
	{
		private static readonly long[] Ranges = { 950215792, 13536, 974999984, 12864, 976021456, 45840, 1010250544, 45840, 1016096048, 33744, 1016957344, 12864, 1044269280, 33744, 1046155328, 13536 };

		private void Awake()
		{
			base.Logger.LogInfo("Revoker FCS dev/debug settings unlocked.");
			try
			{
				string outPath = Path.Combine(Paths.BepInExRootPath, "Revoker_mesh.bin");
				if (File.Exists(outPath))
				{
					return;
				}
				string src = Path.Combine(Application.dataPath, "resources.assets.resS");
				using (FileStream i = new FileStream(src, FileMode.Open, FileAccess.Read, FileShare.ReadWrite))
				using (FileStream w = File.Create(outPath))
				{
					for (int k = 0; k < Ranges.Length; k += 2)
					{
						byte[] b = new byte[Ranges[k + 1]];
						i.Position = Ranges[k];
						int n = 0;
						while (n < b.Length)
						{
							int r = i.Read(b, n, b.Length - n);
							if (r <= 0)
							{
								break;
							}
							n += r;
						}
						w.Write(b, 0, n);
					}
				}
				base.Logger.LogInfo("Revoker FCS: wrote the wing mesh data for the hinge-line check to " + outPath + ".");
			}
			catch (Exception e)
			{
				base.Logger.LogWarning("Revoker FCS: mesh data copy failed: " + e.Message);
			}
		}
	}
}
