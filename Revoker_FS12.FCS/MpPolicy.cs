// Ifrit 0.4.0 / F-22E 2.4.0: multiplayer host checks.
//
// Anything here that changes the flight model (the control laws, the airframe and engine changes, the performance
// profile, the countermeasure and ammo capacities) is a host decision in multiplayer. The host (or dedicated server)
// running this plugin sends every client a policy message when the client authenticates and again whenever the host's
// settings change. A client that never gets one (the host does not run the plugin) flies the stock aircraft with the
// stock fly-by-wire, stock thrust, drag, fuel burn, countermeasures and ammo, whatever its own config says. With a
// policy: the FCS is allowed or not (HostAllowFcs), and the variant (performance profile + countermeasures + ammo) is
// either locked to the host's values for everyone (HostLockVariant) or stock for every client. The host itself and
// single player use the local config as before.
//
// Transport: a Mirage message (NetworkPlayer.Send / NetworkServer.SendToAll) with hand-written Writer/Reader
// delegates (no weaver). A vanilla client that receives it logs "Unexpected message ID" and carries on (its
// client-side player has no error-rate bucket, so SetError returns at once); nothing is ever sent client -> server.
using System;
using BepInEx.Configuration;
using HarmonyLib;
using Mirage;
using Mirage.Serialization;
using NuclearOption.Networking;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	public struct FcsHostPolicyMsg
	{
		public byte ver;

		public bool allowFcs;

		public bool lockVariant;

		public bool stock;

		public float thrust;

		public float drag;

		public float fuel;

		public float flare;

		public float gun;

		public float ewCap;

		public float ewRate;

		public float drAoAHigh;

		public float drTrans;

		public string Describe()
		{
			return string.Format(System.Globalization.CultureInfo.InvariantCulture, "FCS {0}, variant {1}{2}", allowFcs ? "allowed" : "not allowed", lockVariant ? ("locked to the host's " + (stock ? "Stock" : "Buffed")) : "stock for everyone", lockVariant ? string.Format(System.Globalization.CultureInfo.InvariantCulture, " (thrust x{0:F3}, drag x{1:F3}, fuel burn x{2:F2}, flares x{3:F1}, gun ammo x{4:F1}, EW x{5:F2}/x{6:F2})", thrust, drag, fuel, flare, gun, ewCap, ewRate) : "");
		}
	}

	internal static class MpPolicy
	{
		private const byte Ver = 1;

		private static NetworkManagerNuclearOption nm;

		private static bool hostKnown;

		private static FcsHostPolicyMsg host;

		private static float lastSend;

		private static bool sendPending;

		private static int lastState = -1;

		// --- network state -------------------------------------------------------------------------------------------
		public static bool IsServer
		{
			get
			{
				try
				{
					return nm != null && nm.Server != null && nm.Server.Active;
				}
				catch
				{
					return false;
				}
			}
		}

		public static bool IsClientOnly
		{
			get
			{
				try
				{
					return !IsServer && nm != null && nm.Client != null && nm.Client.Active;
				}
				catch
				{
					return false;
				}
			}
		}

		public static bool HostKnown => hostKnown;

		// 0.4.1b / 2.4.1b: single player runs on a local server too (GameState.SinglePlayer with Server.Active), and the
		// first 0.4.1 / 2.4.1 cut bound the host by its own switches there as well: with HostLockVariant off every single-player
		// jet flew the Stock variant (23 Oct 00:30). The host rules apply to multiplayer only.
		public static bool IsMultiplayer
		{
			get
			{
				try
				{
					return GameManager.gameState != GameState.SinglePlayer && (IsServer || IsClientOnly);
				}
				catch
				{
					return IsServer || IsClientOnly;
				}
			}
		}

		// the FCS (control laws, airframe and engine changes) may fly this machine's jets. 0.4.1 / 2.4.1: the host is bound
		// by its own switch too (NOMNOM: a feature that gives an advantage is passable only when the host has enabled it,
		// and a host that disables it for the clients while keeping it is the advantage the rule is about)
		public static bool FcsAllowed => !IsMultiplayer || (IsServer ? Plugin.HostAllowFcs : (hostKnown && host.allowFcs));

		// the variant (profile, countermeasures, ammo) comes from the host's policy
		public static bool VariantLocked => IsMultiplayer && IsClientOnly && hostKnown && host.lockVariant;

		// a client the host does not lock flies the stock variant; so does the host itself with the lock off
		public static bool StockForced => IsMultiplayer && ((IsClientOnly && !VariantLocked) || (IsServer && !Plugin.HostLockVariant));

		public static FcsHostPolicyMsg Host => host;

		public static string Describe()
		{
			if (!IsMultiplayer)
			{
				return "single player: local settings apply";
			}
			if (IsServer)
			{
				return "host: the host's own switches apply to its jet too, policy sent to clients (" + LocalPolicy().Describe() + ")";
			}
			if (IsClientOnly)
			{
				return hostKnown ? ("client: " + host.Describe()) : "client: no policy from the host (it does not run this plugin): stock fly-by-wire, stock performance";
			}
			return "offline: local settings apply";
		}

		// the policy this machine's settings would impose as host
		public static FcsHostPolicyMsg LocalPolicy()
		{
			bool stock = Plugin.LocalStockPerformance;
			return new FcsHostPolicyMsg
			{
				ver = Ver,
				allowFcs = Plugin.HostAllowFcs,
				lockVariant = Plugin.HostLockVariant,
				stock = stock,
				thrust = Plugin.LocalThrustScale,
				drag = Plugin.LocalDragScale,
				fuel = Plugin.LocalFuelUseScale,
				flare = Plugin.LocalFlareCountScale,
				gun = Plugin.LocalGunAmmoScale,
				ewCap = Plugin.LocalEWCapacityScale,
				ewRate = Plugin.LocalEWRechargeScale,
				drAoAHigh = Plugin.LocalDragReliefAoAHigh,
				drTrans = Plugin.LocalDragReliefTransonic
			};
		}

		// --- serialization (no weaver: the delegates are set by hand) ----------------------------------------------------
		private static void WriteF(NetworkWriter w, float f)
		{
			w.WriteInt32(BitConverter.ToInt32(BitConverter.GetBytes(f), 0));
		}

		private static float ReadF(NetworkReader r)
		{
			return BitConverter.ToSingle(BitConverter.GetBytes(r.ReadInt32()), 0);
		}

		private static void Write(NetworkWriter w, FcsHostPolicyMsg m)
		{
			w.WriteByte(m.ver);
			w.WriteByte((byte)((m.allowFcs ? 1 : 0) | (m.lockVariant ? 2 : 0) | (m.stock ? 4 : 0)));
			WriteF(w, m.thrust);
			WriteF(w, m.drag);
			WriteF(w, m.fuel);
			WriteF(w, m.flare);
			WriteF(w, m.gun);
			WriteF(w, m.ewCap);
			WriteF(w, m.ewRate);
			WriteF(w, m.drAoAHigh);
			WriteF(w, m.drTrans);
		}

		private static FcsHostPolicyMsg Read(NetworkReader r)
		{
			FcsHostPolicyMsg m = default(FcsHostPolicyMsg);
			m.ver = r.ReadByte();
			byte flags = r.ReadByte();
			m.allowFcs = (flags & 1) != 0;
			m.lockVariant = (flags & 2) != 0;
			m.stock = (flags & 4) != 0;
			m.thrust = ReadF(r);
			m.drag = ReadF(r);
			m.fuel = ReadF(r);
			m.flare = ReadF(r);
			m.gun = ReadF(r);
			m.ewCap = ReadF(r);
			m.ewRate = ReadF(r);
			m.drAoAHigh = ReadF(r);
			m.drTrans = ReadF(r);
			return m;
		}

		// --- hooks ---------------------------------------------------------------------------------------------------
		public static void Install(Harmony h, Action<string, Action> tryPatch)
		{
			try
			{
				Writer<FcsHostPolicyMsg>.Write = Write;
				Reader<FcsHostPolicyMsg>.Read = Read;
			}
			catch (Exception ex)
			{
				Plugin.Log.LogError("FCS multiplayer: message serializers: " + ex);
			}
			tryPatch("NetworkManagerNuclearOption.Awake", delegate
			{
				h.Patch(AccessTools.Method(typeof(NetworkManagerNuclearOption), "Awake"), null, new HarmonyMethod(typeof(MpPolicy), "AwakePostfix"));
			});
			tryPatch("NetworkManagerNuclearOption.ClientStarted", delegate
			{
				h.Patch(AccessTools.Method(typeof(NetworkManagerNuclearOption), "ClientStarted"), null, new HarmonyMethod(typeof(MpPolicy), "ClientStartedPostfix"));
			});
			tryPatch("NetworkManagerNuclearOption.OnServerAuthenticated", delegate
			{
				h.Patch(AccessTools.Method(typeof(NetworkManagerNuclearOption), "OnServerAuthenticated"), null, new HarmonyMethod(typeof(MpPolicy), "ServerAuthenticatedPostfix"));
			});
			tryPatch("NetworkManagerNuclearOption.ClientDisconnected", delegate
			{
				h.Patch(AccessTools.Method(typeof(NetworkManagerNuclearOption), "ClientDisconnected"), null, new HarmonyMethod(typeof(MpPolicy), "ClientDisconnectedPostfix"));
			});
		}

		public static void AwakePostfix(NetworkManagerNuclearOption __instance)
		{
			nm = __instance;
			MarkModded();
		}

		// 0.4.1 / 2.4.1: the game's own modded-server flag (NetworkManagerNuclearOption.ModdedServer, read when a lobby is
		// created and by the dedicated server's lobby update; the server browser and NOMM show it as "Modded"). Set while the
		// host's settings change anything for the players on this server (the FCS allowed or the variant locked); only ever
		// set, never cleared, so another plugin's flag is left alone.
		private static void MarkModded()
		{
			try
			{
				if (nm != null && (Plugin.HostAllowFcs || Plugin.HostLockVariant))
				{
					nm.SetModdedServer(true);
				}
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("FCS multiplayer: could not set the modded-server flag: " + ex.Message);
			}
		}

		public static void ClientStartedPostfix(NetworkManagerNuclearOption __instance)
		{
			nm = __instance;
			hostKnown = false;
			try
			{
				if (__instance.Client != null && __instance.Client.MessageHandler != null)
				{
					__instance.Client.MessageHandler.RegisterHandler<FcsHostPolicyMsg>(OnPolicy, allowUnauthenticated: true);
				}
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("FCS multiplayer: could not register the host policy handler: " + ex.Message);
			}
		}

		public static void ServerAuthenticatedPostfix(NetworkManagerNuclearOption __instance, INetworkPlayer player)
		{
			nm = __instance;
			try
			{
				if (player != null)
				{
					player.Send(LocalPolicy(), Channel.Reliable);
				}
			}
			catch (Exception ex)
			{
				Plugin.Log.LogWarning("FCS multiplayer: could not send the host policy: " + ex.Message);
			}
		}

		public static void ClientDisconnectedPostfix()
		{
			hostKnown = false;
		}

		private static void OnPolicy(INetworkPlayer player, FcsHostPolicyMsg msg)
		{
			if (msg.ver != Ver)
			{
				Plugin.Log.LogWarning("FCS multiplayer: host policy version " + msg.ver + " (this build speaks " + Ver + "); treated as no policy.");
				hostKnown = false;
				return;
			}
			bool changed = !hostKnown || !Same(host, msg);
			host = msg;
			hostKnown = true;
			if (changed)
			{
				Plugin.Log.LogInfo("FCS multiplayer: host policy: " + msg.Describe() + ". " + (IsServer ? "(this machine is the host, its own settings apply)" : "Takes effect now for the FCS, at the next spawn for thrust, drag, countermeasures and ammo."));
			}
		}

		private static bool Same(FcsHostPolicyMsg a, FcsHostPolicyMsg b)
		{
			return a.allowFcs == b.allowFcs && a.lockVariant == b.lockVariant && a.stock == b.stock && a.thrust == b.thrust && a.drag == b.drag && a.fuel == b.fuel && a.flare == b.flare && a.gun == b.gun && a.ewCap == b.ewCap && a.ewRate == b.ewRate && a.drAoAHigh == b.drAoAHigh && a.drTrans == b.drTrans;
		}

		// the host's settings changed: tell every client (throttled; called from the config change events)
		public static void SettingsChanged()
		{
			sendPending = true;
			MarkModded();
		}

		// once a frame from the plugin
		public static void Tick()
		{
			if (sendPending && IsServer && Time.unscaledTime - lastSend > 0.5f)
			{
				sendPending = false;
				lastSend = Time.unscaledTime;
				try
				{
					nm.Server.SendToAll(LocalPolicy(), excludeLocalPlayer: false, Channel.Reliable);
				}
				catch (Exception ex)
				{
					Plugin.Log.LogWarning("FCS multiplayer: could not send the host policy: " + ex.Message);
				}
			}
			int state = !IsMultiplayer ? 0 : (IsServer ? 1 : (IsClientOnly ? (hostKnown ? 3 : 2) : 0));
			if (state != lastState)
			{
				lastState = state;
				Plugin.Log.LogInfo("FCS multiplayer: " + Describe());
			}
		}
	}
}
