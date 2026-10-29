using System.IO;
using BepInEx;
using BepInEx.Bootstrap;
using BepInEx.Configuration;
using HarmonyLib;

namespace Revoker_FS12.FCS
{
	[BepInPlugin("Revoker_FS12.FCS", "FS-12 Revoker FCS", Version)]
	[BepInDependency(DevGuid, BepInDependency.DependencyFlags.SoftDependency)]
	public class FcsStandalonePlugin : BaseUnityPlugin
	{
		public const string Version = "0.2.9";

		// the dev/debug unlock: a separate, empty plugin DLL. When it is loaded (soft dependency, so before this one),
		// every setting other than the player ones is bound to its config file and shows in the ConfigurationManager
		// under it; without it those settings are held at their defaults in memory and never written anywhere.
		public const string DevGuid = "Revoker_FS12.FCS.Dev";

		private void Awake()
		{
			ConfigFile dev = null;
			if (Chainloader.PluginInfos.TryGetValue(DevGuid, out PluginInfo devInfo) && devInfo != null && devInfo.Instance != null)
			{
				dev = devInfo.Instance.Config;
			}
			bool devUnlocked = dev != null;
			bool devFresh = devUnlocked && !File.Exists(dev.ConfigFilePath);
			if (!devUnlocked)
			{
				dev = new ConfigFile(Path.Combine(Paths.CachePath, "Revoker_FS12.FCS.builtin-defaults.cfg"), saveOnInit: false);
				dev.SaveOnConfigSet = false;
			}
			Plugin.Init(base.Config, dev, devUnlocked, devFresh, base.Logger);
			Plugin.ApplyPatches(new Harmony("tom.revoker_fs12.fcs"));
			cfgFile = base.Config;
			devFile = devUnlocked ? dev : null;
			// 0.4.0 multiplayer: the host's policy follows its settings (a change in F1, on disk or by migration)
			base.Config.SettingChanged += delegate
			{
				MpPolicy.SettingsChanged();
			};
			base.Config.ConfigReloaded += delegate
			{
				MpPolicy.SettingsChanged();
			};
			if (devUnlocked)
			{
				dev.SettingChanged += delegate
				{
					MpPolicy.SettingsChanged();
				};
				dev.ConfigReloaded += delegate
				{
					MpPolicy.SettingsChanged();
				};
			}
			cfgStamp = Stamp(cfgFile);
			devStamp = Stamp(devFile);
		}

		// 2.0.5: BepInEx reads a config file only at start-up. Edits made in a text editor while the game runs were not
		// applied, and the next save from inside the game (a setting changed in F1, a new key, a migration) wrote the old
		// values back over them. Both files are now re-read whenever they change on disk; flight settings apply at once,
		// spawn-time ones (airframe, countermeasures, ammo) on the next spawn.
		private ConfigFile cfgFile;

		private ConfigFile devFile;

		private System.DateTime cfgStamp;

		private System.DateTime devStamp;

		private float nextCheck;

		private static System.DateTime Stamp(ConfigFile f)
		{
			try
			{
				return (f != null && File.Exists(f.ConfigFilePath)) ? File.GetLastWriteTimeUtc(f.ConfigFilePath) : default(System.DateTime);
			}
			catch
			{
				return default(System.DateTime);
			}
		}

		private void Update()
		{
			MpPolicy.Tick();
			if (UnityEngine.Time.unscaledTime < nextCheck)
			{
				return;
			}
			nextCheck = UnityEngine.Time.unscaledTime + 1f;
			CheckReload(cfgFile, ref cfgStamp);
			CheckReload(devFile, ref devStamp);
		}

		private void CheckReload(ConfigFile f, ref System.DateTime stamp)
		{
			if (f == null)
			{
				return;
			}
			System.DateTime now = Stamp(f);
			if (now == stamp || now == default(System.DateTime))
			{
				return;
			}
			try
			{
				f.Reload();
				Logger.LogInfo("Revoker FCS: " + Path.GetFileName(f.ConfigFilePath) + " changed on disk, settings re-read.");
			}
			catch (System.Exception ex)
			{
				Logger.LogWarning("Revoker FCS: re-reading " + Path.GetFileName(f.ConfigFilePath) + " failed: " + ex.Message);
			}
			stamp = Stamp(f);
		}
	}
}
