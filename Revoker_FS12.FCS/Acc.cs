using System;
using System.Collections.Generic;
using HarmonyLib;
using NuclearOption.Jobs;
using UnityEngine;

namespace Revoker_FS12.FCS
{
	internal static class Acc
	{
		public static readonly AccessTools.FieldRef<LandingGear, Aircraft> LG_aircraft = AccessTools.FieldRefAccess<LandingGear, Aircraft>("aircraft");

		public static readonly AccessTools.FieldRef<LandingGear, ControlInputs> LG_inputs = AccessTools.FieldRefAccess<LandingGear, ControlInputs>("controlInputs");

		public static readonly AccessTools.FieldRef<LandingGear, bool> LG_steering = AccessTools.FieldRefAccess<LandingGear, bool>("steering");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_steerLock = AccessTools.FieldRefAccess<LandingGear, float>("steeringLock");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_steerSpeed = AccessTools.FieldRefAccess<LandingGear, float>("steeringSpeed");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_friction = AccessTools.FieldRefAccess<LandingGear, float>("frictionCoef");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_steerAngle = AccessTools.FieldRefAccess<LandingGear, float>("steeringAngle");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_compForce = AccessTools.FieldRefAccess<LandingGear, float>("compressionForce");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_dampForce = AccessTools.FieldRefAccess<LandingGear, float>("dampingForce");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_wheelSpeed = AccessTools.FieldRefAccess<LandingGear, float>("wheelSpeed");

		public static readonly AccessTools.FieldRef<LandingGear, float> LG_response = AccessTools.FieldRefAccess<LandingGear, float>("response");

		public static readonly AccessTools.FieldRef<LandingGear, RaycastHit> LG_hit = AccessTools.FieldRefAccess<LandingGear, RaycastHit>("hit");

		public static readonly AccessTools.FieldRef<AeroPart, int> AP_airfoil = AccessTools.FieldRefAccess<AeroPart, int>("airfoil");

		public static readonly AccessTools.FieldRef<AeroPart, Vector3> AP_centerOfLift = AccessTools.FieldRefAccess<AeroPart, Vector3>("centerOfLift");

		public static readonly AccessTools.FieldRef<AeroPart, Transform> AP_liftNormal = AccessTools.FieldRefAccess<AeroPart, Transform>("liftNormal");

		public static readonly AccessTools.FieldRef<AeroPart, float> AP_wingArea = AccessTools.FieldRefAccess<AeroPart, float>("wingArea");

		public static readonly AccessTools.FieldRef<AeroPart, float> AP_wingEff = AccessTools.FieldRefAccess<AeroPart, float>("wingEffectiveness");

		public static readonly AccessTools.FieldRef<Aircraft, List<ControlSurface>> AC_surfaces = AccessTools.FieldRefAccess<Aircraft, List<ControlSurface>>("controlSurfaces");

		public static readonly AccessTools.FieldRef<Aircraft, Vector3> AC_wind = AccessTools.FieldRefAccess<Aircraft, Vector3>("windVelocity");

		public static readonly AccessTools.FieldRef<ControlSurface, PtrAllocation<ControlSurfaceFields>> CS_job = AccessTools.FieldRefAccess<ControlSurface, PtrAllocation<ControlSurfaceFields>>("JobFields");

		public static readonly AccessTools.FieldRef<ControlSurface, Aircraft> CS_aircraft = AccessTools.FieldRefAccess<ControlSurface, Aircraft>("aircraft");

		public static readonly AccessTools.FieldRef<ControlSurface, UnitPart> CS_attached = AccessTools.FieldRefAccess<ControlSurface, UnitPart>("attachedSurface");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_pitchRange = AccessTools.FieldRefAccess<ControlSurface, float>("pitchRange");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_rollRange = AccessTools.FieldRefAccess<ControlSurface, float>("rollRange");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_yawRange = AccessTools.FieldRefAccess<ControlSurface, float>("yawRange");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_brakeRange = AccessTools.FieldRefAccess<ControlSurface, float>("brakeRange");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_maxSplit = AccessTools.FieldRefAccess<ControlSurface, float>("maxSplit");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_splitAmount = AccessTools.FieldRefAccess<ControlSurface, float>("splitAmount");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_splitDrag = AccessTools.FieldRefAccess<ControlSurface, float>("splitDrag");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_yawSplitFactor = AccessTools.FieldRefAccess<ControlSurface, float>("yawSplitFactor");

		public static readonly AccessTools.FieldRef<ControlSurface, float> CS_servo = AccessTools.FieldRefAccess<ControlSurface, float>("servoSpeed");

		public static readonly AccessTools.FieldRef<ControlSurface, bool> CS_locked = AccessTools.FieldRefAccess<ControlSurface, bool>("locked");

		public static readonly AccessTools.FieldRef<ControlSurface, GameObject> CS_visible = AccessTools.FieldRefAccess<ControlSurface, GameObject>("visibleMesh");

		public static readonly AccessTools.FieldRef<ControlsFilter, Aircraft> CF_aircraft = AccessTools.FieldRefAccess<ControlsFilter, Aircraft>("aircraft");

		public static readonly AccessTools.FieldRef<JetNozzle, Transform> JN_thrustT = AccessTools.FieldRefAccess<JetNozzle, Transform>("thrustTransform");

		public static readonly AccessTools.FieldRef<JetNozzle, float> JN_totalThrust = AccessTools.FieldRefAccess<JetNozzle, float>("totalThrust");

		public static readonly AccessTools.FieldRef<Turbojet, Aircraft> TF_aircraft = AccessTools.FieldRefAccess<Turbojet, Aircraft>("aircraft");

		public static readonly AccessTools.FieldRef<Turbojet, Vector3> TF_nozzleAngles = AccessTools.FieldRefAccess<Turbojet, Vector3>("nozzleAngles");

		public static readonly AccessTools.FieldRef<Turbojet, JetNozzle[]> TF_nozzles = AccessTools.FieldRefAccess<Turbojet, JetNozzle[]>("nozzles");

		public static readonly AccessTools.FieldRef<Turbojet, bool> TF_operable = AccessTools.FieldRefAccess<Turbojet, bool>("operable");

		public static readonly AccessTools.FieldRef<Turbojet, Vector3> TF_vec = AccessTools.FieldRefAccess<Turbojet, Vector3>("thrustVectoring");

		public static readonly AccessTools.FieldRef<Turbojet, Vector3> TF_vecGain = AccessTools.FieldRefAccess<Turbojet, Vector3>("thrustVectoringGain");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_vecMaxSpeed = AccessTools.FieldRefAccess<Turbojet, float>("thrustVectoringMaxAirspeed");

		public static readonly AccessTools.FieldRef<Turbojet, Transform[]> TF_vecT = AccessTools.FieldRefAccess<Turbojet, Transform[]>("vectoringTransforms");

		public static readonly AccessTools.FieldRef<AeroPart, float> AP_dragArea = AccessTools.FieldRefAccess<AeroPart, float>("dragArea");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_staticThrust = AccessTools.FieldRefAccess<Turbojet, float>("maxThrust");

		// Ifrit 0.4.1: the engine's damage state and RPM range (Turbojet.FixedUpdate: target RPM = Lerp(minRPM, minRPM + (maxRPM - minRPM) x (0.5 x condition + 0.5), throttle);
		// thrustRatio = (rpm - 0.85 minRPM) / (maxRPM - minRPM); the afterburner runs only with rpm / maxRPM > 0.85)
		public static readonly AccessTools.FieldRef<Turbojet, float> TF_condition = AccessTools.FieldRefAccess<Turbojet, float>("condition");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_minRPM = AccessTools.FieldRefAccess<Turbojet, float>("minRPM");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_maxRPM = AccessTools.FieldRefAccess<Turbojet, float>("maxRPM");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_spoolRate = AccessTools.FieldRefAccess<Turbojet, float>("spoolRate");

		public static readonly AccessTools.FieldRef<Turbojet, Vector2> TF_remap = AccessTools.FieldRefAccess<Turbojet, Vector2>("throttleRemap");

		public static readonly AccessTools.FieldRef<Turbojet, float> TF_rpm = AccessTools.FieldRefAccess<Turbojet, float>("rpm");

		// thrust = max(0, (maxThrust - parasiticThrustLoss) x thrustRatio); in the game only the LiftFan writes it, so on a jet
		// without one the FCS may use it to hold a healthy engine down to a damaged one's thrust (0.4.1)
		public static readonly AccessTools.FieldRef<Turbojet, float> TF_parasitic = AccessTools.FieldRefAccess<Turbojet, float>("parasiticThrustLoss");

		private static readonly System.Reflection.FieldInfo fJnAfterburners = AccessTools.Field(typeof(JetNozzle), "afterburners");

		// JetNozzle.Afterburner is a private nested class: its fields are resolved from the array's element type
		private static readonly System.Type tAfterburner = (fJnAfterburners != null && fJnAfterburners.FieldType.IsArray) ? fJnAfterburners.FieldType.GetElementType() : null;

		private static readonly System.Reflection.FieldInfo fAbThrust = (tAfterburner != null) ? AccessTools.Field(tAfterburner, "thrust") : null;

		private static readonly System.Reflection.FieldInfo fAbStart = (tAfterburner != null) ? AccessTools.Field(tAfterburner, "throttleStart") : null;

		private static readonly System.Reflection.FieldInfo fAbEnd = (tAfterburner != null) ? AccessTools.Field(tAfterburner, "throttleEnd") : null;

		// the nozzle's afterburner thrust at full throttle (sum of its afterburners), 0 without one
		public static float JN_abMax(JetNozzle n)
		{
			float sum = 0f;
			if (n == null || fJnAfterburners == null || fAbThrust == null || !(fJnAfterburners.GetValue(n) is System.Array arr))
			{
				return 0f;
			}
			for (int i = 0; i < arr.Length; i++)
			{
				object ab = arr.GetValue(i);
				if (ab != null)
				{
					sum += (float)fAbThrust.GetValue(ab);
				}
			}
			return sum;
		}

		// the throttle input at which the nozzle's afterburner starts (the lowest of its afterburners); 1 without one
		public static float JN_abStart(JetNozzle n)
		{
			float st = 1f;
			if (n == null || fJnAfterburners == null || fAbStart == null || !(fJnAfterburners.GetValue(n) is System.Array arr))
			{
				return 1f;
			}
			for (int i = 0; i < arr.Length; i++)
			{
				object ab = arr.GetValue(i);
				if (ab != null)
				{
					st = Mathf.Min(st, (float)fAbStart.GetValue(ab));
				}
			}
			return st;
		}
		// the throttle input at which the nozzle's afterburner is fully in (the lowest of its afterburners); 1 without one
		public static float JN_abEnd(JetNozzle n)
		{
			float en = 1f;
			if (n == null || fJnAfterburners == null || fAbEnd == null || !(fJnAfterburners.GetValue(n) is System.Array arr))
			{
				return 1f;
			}
			for (int i = 0; i < arr.Length; i++)
			{
				object ab = arr.GetValue(i);
				if (ab != null)
				{
					en = Mathf.Min(en, (float)fAbEnd.GetValue(ab));
				}
			}
			return en;
		}
		private static readonly System.Reflection.FieldInfo fAbSmooth = (tAfterburner != null) ? AccessTools.Field(tAfterburner, "smoothing") : null;

		// scale the nozzle's afterburner light-up / fade speed (Afterburner.smoothing: amount = Lerp(amount, target, smoothing x dt))
		public static void JN_scaleAbSmoothing(JetNozzle n, float k)
		{
			if (n == null || fJnAfterburners == null || fAbSmooth == null || !(fJnAfterburners.GetValue(n) is System.Array arr))
			{
				return;
			}
			for (int i = 0; i < arr.Length; i++)
			{
				object ab = arr.GetValue(i);
				if (ab != null)
				{
					fAbSmooth.SetValue(ab, (float)fAbSmooth.GetValue(ab) * k);
				}
			}
		}



		public static readonly AccessTools.FieldRef<HighLiftDevice, AeroPart> HL_part = AccessTools.FieldRefAccess<HighLiftDevice, AeroPart>("aeroPart");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_speedDep = AccessTools.FieldRefAccess<HighLiftDevice, float>("speedDeployed");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_speedRet = AccessTools.FieldRefAccess<HighLiftDevice, float>("speedRetracted");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_areaDep = AccessTools.FieldRefAccess<HighLiftDevice, float>("partAreaDeployed");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_areaRet = AccessTools.FieldRefAccess<HighLiftDevice, float>("partAreaRetracted");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_position = AccessTools.FieldRefAccess<HighLiftDevice, float>("position");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_alphaMin = AccessTools.FieldRefAccess<HighLiftDevice, float>("alphaMin");

		public static readonly AccessTools.FieldRef<HighLiftDevice, float> HL_alphaFactor = AccessTools.FieldRefAccess<HighLiftDevice, float>("alphaFactor");

		public static List<Component> Collect(Aircraft a, Type t)
		{
			HashSet<int> seen = new HashSet<int>();
			List<Component> list = new List<Component>();
			if ((UnityEngine.Object)(object)a == null || t == null)
			{
				return list;
			}
			Component[] componentsInChildren = ((Component)(object)a).GetComponentsInChildren(t, includeInactive: true);
			for (int i = 0; i < componentsInChildren.Length; i++)
			{
				Add(componentsInChildren[i]);
			}
			if (a.partLookup != null)
			{
				foreach (UnitPart item in a.partLookup)
				{
					if (item != null)
					{
						componentsInChildren = item.GetComponentsInChildren(t, includeInactive: true);
						for (int i = 0; i < componentsInChildren.Length; i++)
						{
							Add(componentsInChildren[i]);
						}
					}
				}
			}
			return list;
			void Add(Component c)
			{
				if (c != null && seen.Add(c.GetInstanceID()))
				{
					list.Add(c);
				}
			}
		}

		public static List<T> Collect<T>(Aircraft a) where T : Component
		{
			HashSet<int> seen = new HashSet<int>();
			List<T> list = new List<T>();
			if ((UnityEngine.Object)(object)a == null)
			{
				return list;
			}
			T[] componentsInChildren = ((Component)(object)a).GetComponentsInChildren<T>(includeInactive: true);
			for (int i = 0; i < componentsInChildren.Length; i++)
			{
				Add(componentsInChildren[i]);
			}
			if (a.partLookup != null)
			{
				foreach (UnitPart item in a.partLookup)
				{
					if (item != null)
					{
						componentsInChildren = item.GetComponentsInChildren<T>(includeInactive: true);
						for (int i = 0; i < componentsInChildren.Length; i++)
						{
							Add(componentsInChildren[i]);
						}
					}
				}
			}
			if (typeof(T) == typeof(ControlSurface))
			{
				List<ControlSurface> list2 = AC_surfaces(a);
				if (list2 != null)
				{
					foreach (ControlSurface item2 in list2)
					{
						Add(item2 as T);
					}
				}
			}
			return list;
			void Add(T c)
			{
				if (c != null && seen.Add(c.GetInstanceID()))
				{
					list.Add(c);
				}
			}
		}

		public static bool JobCreated(ControlSurface cs)
		{
			return CS_job(cs).IsCreated;
		}

		public static ref ControlSurfaceFields Job(ControlSurface cs)
		{
			return ref CS_job(cs).Ref();
		}
	}
}
