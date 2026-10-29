using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace Revoker_FS12.FCS
{
	// Revoker 0.2.3: render-only hinge correction for the trailing-edge surfaces. The game (ControlSurface job, and the HLD
	// for the inboard flaps) turns each surface's visible mesh about its own local x axis through the mesh origin; on the
	// Revoker the modelled hinge lines are not on that axis, so a deflected surface opens a gap at one end and clips into the
	// wing at the other. This draws a copy of the mesh turned about the measured hinge line instead and hides the original
	// renderer. Physics is untouched: the visible transform (also the part's lift normal) keeps the game's rotation, so the
	// aero, the FCS's effector models and the damage model are exactly as before. The deflection is read back every frame
	// from the visible transform, so whatever moves the surface (game, HLD, FCS) is followed.
	internal sealed class VisualHinge : MonoBehaviour
	{
		// per surface: hinge axis and a point on it, both in the visible mesh's local frame at rest (measured from the meshes)
		internal struct Hinge
		{
			public string vis;

			public Vector3 axis;

			public Vector3 pivot;
		}

		// Measured from the game meshes (fighter1_aileron_L/R, resources.assets.resS): the aileron's leading edge, and the
		// wingtip's cove lip 2.4-5.2 cm behind it, run along z = 0.033 -/+ 0.161 x (9.15 deg sweep to the mesh x axis, 0.5 deg
		// dihedral); the nose centre (hinge) is 4.5 cm behind the leading edge, 1.4 cm from the mesh origin. Rotating about the
		// mesh x axis at 30 deg moved the leading-edge ends +8.2 / -5.0 cm (into the wing at one end, out of the cove at the
		// other); about this line both move 2.2 cm, inside the 3-7 cm cove. The inboard flaps' leading edge runs along the
		// mesh x axis (z 0.058, nose on y 0), so the game's hinge is already right there and they are not listed.
		internal static readonly List<Hinge> Table = new List<Hinge>
		{
			new Hinge
			{
				vis = "aileron_L_visible",
				axis = new Vector3(1f, 0.0086f, -0.1611f),
				pivot = new Vector3(-0.0072f, 0f, -0.0114f)
			},
			new Hinge
			{
				vis = "aileron_R_visible",
				axis = new Vector3(1f, -0.0086f, 0.1611f),
				pivot = new Vector3(0.0072f, 0f, -0.0114f)
			}
		};

		private sealed class Entry
		{
			public Transform vis;

			public Quaternion rest;

			public Vector3 restPos;

			public Vector3 axis;

			public Vector3 pivot;

			public Transform proxy;

			public MeshRenderer orig;

			public MeshRenderer prox;

			public MeshFilter origF;

			public MeshFilter proxF;
		}

		private float syncT;

		private readonly List<Entry> entries = new List<Entry>();

		public static int Attach(Aircraft a)
		{
			if (a == null || Table.Count == 0 || !Plugin.VisualHingeFix)
			{
				Plugin.Log.LogInfo($"Revoker FCS hinge visuals: off (table {Table.Count}, VisualHingeFix {Plugin.VisualHingeFix})");
				return 0;
			}
			GameObject go = ((Component)(object)a).gameObject;
			if (go.GetComponent<VisualHinge>() != null)
			{
				return -1;
			}
			Dictionary<string, Transform> byName = new Dictionary<string, Transform>();
			foreach (Transform t in go.GetComponentsInChildren<Transform>(true))
			{
				byName[t.name] = t;
			}
			Dictionary<Transform, Quaternion> rests = new Dictionary<Transform, Quaternion>();
			foreach (ControlSurface cs in Acc.Collect<ControlSurface>(a))
			{
				GameObject v = Acc.CS_visible(cs);
				if (v == null)
				{
					continue;
				}
				// 0.2.3a: the surface's own visible mesh, wherever the part sits in the scene (0.2.3 only searched under the
				// aircraft root and found nothing in game)
				byName[v.name] = v.transform;
				if (Acc.JobCreated(cs))
				{
					rests[v.transform] = Acc.Job(cs).restingRotation;
				}
			}
			VisualHinge vh = null;
			foreach (Hinge h in Table)
			{
				if (!byName.TryGetValue(h.vis, out Transform vis) || vis.parent == null)
				{
					Plugin.Log.LogInfo("Revoker FCS hinge visuals: " + h.vis + " not found");
					continue;
				}
				MeshFilter mf = vis.GetComponent<MeshFilter>();
				MeshRenderer mr = vis.GetComponent<MeshRenderer>();
				if (mf == null || mr == null || mf.sharedMesh == null)
				{
					Plugin.Log.LogInfo($"Revoker FCS hinge visuals: {h.vis} has no mesh renderer (filter {mf != null}, renderer {mr != null}, mesh {(mf != null && mf.sharedMesh != null)})");
					continue;
				}
				// the HLD-driven inboard flaps rest at identity (anglesRetracted 0,0,0); the ailerons at the job's resting pose
				Quaternion rest = rests.TryGetValue(vis, out Quaternion r) ? r : Quaternion.identity;
				GameObject p = new GameObject(vis.name + "_hingeVis");
				p.layer = vis.gameObject.layer;
				p.transform.SetParent(vis.parent, false);
				p.transform.localPosition = vis.localPosition;
				p.transform.localRotation = rest;
				p.transform.localScale = vis.localScale;
				MeshFilter pf = p.AddComponent<MeshFilter>();
				pf.sharedMesh = mf.sharedMesh;
				MeshRenderer pr = p.AddComponent<MeshRenderer>();
				pr.sharedMaterials = mr.sharedMaterials;
				pr.shadowCastingMode = mr.shadowCastingMode;
				pr.receiveShadows = mr.receiveShadows;
				pr.lightProbeUsage = mr.lightProbeUsage;
				pr.reflectionProbeUsage = mr.reflectionProbeUsage;
				mr.forceRenderingOff = true;
				if (vh == null)
				{
					vh = go.AddComponent<VisualHinge>();
					vh.lod = go.GetComponent<LODGroup>();
				}
				// the airframe's LODGroup switches the LOD0 renderers off at range: the copy takes the original's slot
				vh.SwapLod(mr, pr);
				vh.entries.Add(new Entry
				{
					vis = vis,
					rest = rest,
					restPos = vis.localPosition,
					axis = h.axis.normalized,
					pivot = h.pivot,
					proxy = p.transform,
					orig = mr,
					prox = pr,
					origF = mf,
					proxF = pf
				});
			}
			return (vh != null) ? vh.entries.Count : 0;
		}

		private void LateUpdate()
		{
			// a damage state may swap the mesh or the materials: the copy follows (checked twice a second)
			syncT += Time.deltaTime;
			bool sync = syncT > 0.5f;
			if (sync)
			{
				syncT = 0f;
			}
			for (int i = 0; i < entries.Count; i++)
			{
				Entry e = entries[i];
				if (e.vis == null || e.proxy == null)
				{
					continue;
				}
				// the game's deflection: its rotation about local x from the rest pose
				Quaternion d = Quaternion.Inverse(e.rest) * e.vis.localRotation;
				float delta = 2f * Mathf.Atan2(d.x, d.w) * 57.29578f;
				if (delta > 180f)
				{
					delta -= 360f;
				}
				else if (delta < -180f)
				{
					delta += 360f;
				}
				Quaternion q = Quaternion.AngleAxis(delta, e.axis);
				e.proxy.localRotation = e.rest * q;
				// about the hinge point, not the mesh origin: x' = p + q (x - p)
				e.proxy.localPosition = e.vis.localPosition + e.rest * (e.pivot - q * e.pivot);
				e.prox.enabled = e.orig.enabled && e.vis.gameObject.activeInHierarchy;
				if (sync)
				{
					if (e.origF != null && e.proxF != null && e.proxF.sharedMesh != e.origF.sharedMesh)
					{
						e.proxF.sharedMesh = e.origF.sharedMesh;
					}
					if (e.orig.sharedMaterial != e.prox.sharedMaterial)
					{
						e.prox.sharedMaterials = e.orig.sharedMaterials;
					}
				}
			}
		}

		private LODGroup lod;

		private void SwapLod(Renderer from, Renderer to)
		{
			if (lod == null || from == null || to == null)
			{
				return;
			}
			LOD[] lods = lod.GetLODs();
			bool changed = false;
			for (int i = 0; i < lods.Length; i++)
			{
				Renderer[] rs = lods[i].renderers;
				if (rs == null)
				{
					continue;
				}
				for (int j = 0; j < rs.Length; j++)
				{
					if (rs[j] == from)
					{
						rs[j] = to;
						changed = true;
					}
				}
			}
			if (changed)
			{
				lod.SetLODs(lods);
			}
		}

		private void OnDestroy()
		{
			foreach (Entry e in entries)
			{
				if (e.orig != null)
				{
					e.orig.forceRenderingOff = false;
					SwapLod(e.prox, e.orig);
				}
				if (e.proxy != null)
				{
					Destroy(e.proxy.gameObject);
				}
			}
		}
	}
}
