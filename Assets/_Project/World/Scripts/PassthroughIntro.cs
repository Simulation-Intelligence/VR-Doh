using System.Collections.Generic;
using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// "Take me to the Moon": the intro starts in the player's real room (passthrough) with only the usu, the
    /// pedestal, the dough, the kine, the hands and the story text, then a round portal opens around the usu and the
    /// Moon garden spreads out (sky, island, bamboo, Moon, Earth) while the room fades away. On a reset the portal closes
    /// back to the room. Runtime only: the camera clears to transparent and the garden's renderers are hidden for the
    /// room; during the transition a veil sphere around the head (MoonRabbit/PortalVeil, drawn after the opaque scene
    /// and the sky, before transparent effects) lets passthrough through outside the portal, except on the usu, pedestal,
    /// dough and kine (stencil bubbles, MoonRabbit/PortalKeep). Extra draws only during the transition. Passthrough
    /// itself (and the hand cut-out) is the team's setup and is not touched. Turn off with <see cref="passthroughIntro"/>.
    /// </summary>
    [RequireComponent(typeof(MoonProgress))]
    public class PassthroughIntro : MonoBehaviour
    {
        public bool passthroughIntro = true;
        [Tooltip("Seconds for the portal to open (also after a skip it opens in at most skipRevealTime).")]
        public float revealTime = 3.5f;
        public float closeTime = 2.2f;
        public float skipRevealTime = 1.4f;
        [Tooltip("When the portal opens: 1-4 = as that intro line appears; 0 = after the last line has gone (the intro's end).")]
        public int revealAtLine = 0;
        public Material veilMaterial;
        [Tooltip("MoonRabbit/PortalKeep: marks the usu, pedestal, dough and kine so the veil leaves them alone.")]
        public Material keepMaterial;
        public Mesh sphereMesh;
        [Tooltip("Radius of the kept bubble around the usu bowl, dough and rice (m); small, so it does not reach the ground.")]
        public float usuKeepRadius = 0.32f;
        [Tooltip("Portal edge softness (deg) and rim glow.")]
        public float edgeSoftness = 10f;
        public float rimIntensity = 0.25f;

        enum State { Garden, Room, Opening, Closing }
        State state = State.Garden;

        MoonProgress progress;
        StoryIntro intro;
        MochiLink link;
        Transform envRoot;
        readonly List<Renderer> hiddenOpaque = new List<Renderer>(), hiddenTransparent = new List<Renderer>();
        readonly List<Behaviour> hiddenBehaviours = new List<Behaviour>();
        Camera cam;
        CameraClearFlags camFlags;
        Color camColor;
        GameObject portal, veil; // portal = unscaled holder of the veil sphere and the keep bubbles
        Material veilInstance;
        Transform usuKeep, kineKeep;
        Renderer kineRenderer, pedestal;
        Transform pedestalCopy;
        Material pedestalKeep;
        readonly List<Material> keepInstances = new List<Material>();
        float t0;
        bool wasPlaying, launched;

        public bool InRoom => state == State.Room || state == State.Opening || state == State.Closing;

        void Awake()
        {
            MoonStory.GardenHidden = false;
            progress = GetComponent<MoonProgress>();
            intro = GetComponent<StoryIntro>();
            link = GetComponent<MochiLink>();
            envRoot = transform.parent != null ? transform.parent : transform;
        }

        void Start()
        {
            progress.onIntroLine.AddListener(n => { if (revealAtLine > 0 && n == revealAtLine && Active && state == State.Room) Open(revealTime); });
            if (intro != null) intro.Started += OnIntroStarted;
        }

        bool Active => passthroughIntro && intro != null && intro.playIntro && isActiveAndEnabled;

        // every intro (launch, reset A / R, headset, after the ending) starts in the room. The intro begins at full black,
        // so the garden is swapped for the room behind the black (no visible jump); without a black fade the portal closes.
        void OnIntroStarted()
        {
            if (!Active) return;
            if (state == State.Garden && intro.blackFade < 0.3f && launched) Close();
            else EnterRoom();
        }

        void OnDisable()
        {
            MoonStory.GardenHidden = false;
            if (state != State.Garden) ShowGarden();
            SetPortal(false);
        }

        void OnDestroy()
        {
            if (intro != null) intro.Started -= OnIntroStarted;
            if (veilInstance != null) Destroy(veilInstance);
            foreach (var m in keepInstances) if (m != null) Destroy(m);
        }

        void Update()
        {
            if (!Active) { if (state != State.Garden) ShowGarden(); launched = true; return; }
            if (!launched) { launched = true; if (state == State.Garden) EnterRoom(); } // never show the garden first

            // the intro ended (after-last-line setting, or the line was never reached) or was skipped: open it now
            bool playing = intro.Playing;
            if (!playing && (state == State.Room || state == State.Closing) && wasPlaying)
                Open(intro.WasSkipped ? Mathf.Min(revealTime, skipRevealTime) : revealTime);
            wasPlaying = playing;

            if (state == State.Opening || state == State.Closing) Animate();
            MoonStory.GardenHidden = state != State.Garden;
        }

        // ------------------------------------------------------------------ states

        void EnterRoom()
        {
            if (state != State.Garden) Restore(); // re-collect from the garden as it is, never from an already hidden one
            CollectGarden();
            foreach (var r in hiddenOpaque) if (r != null) r.enabled = false;
            foreach (var r in hiddenTransparent) if (r != null) r.enabled = false;
            foreach (var b in hiddenBehaviours) if (b != null) b.enabled = false;
            SetCameraTransparent(true);
            SetPortal(false);
            if (state != State.Room) Debug.Log("[Moon Rabbit] Passthrough intro: room");
            state = State.Room;
            MoonStory.GardenHidden = true;
        }

        void Open(float time)
        {
            if (state == State.Garden) return;
            // the garden's opaque world and sky come back now, behind the veil; its glows once the portal is open
            foreach (var r in hiddenOpaque) if (r != null) r.enabled = true;
            foreach (var b in hiddenBehaviours) if (b != null && !(b is SkyMist)) b.enabled = true;
            SetCameraTransparent(false);
            EnsureVeil();
            if (veil == null) { ShowGarden(); return; } // no veil material: straight to the garden
            SetPortal(true);
            t0 = Time.time;
            openTime = Mathf.Max(0.2f, time);
            state = State.Opening;
            Debug.Log($"[Moon Rabbit] Passthrough intro: portal opening ({openTime:F1} s, intro playing: {intro.Playing})");
            SetVeil(0f);
        }

        void Close()
        {
            EnsureVeil();
            if (veil == null) { EnterRoom(); return; }
            CollectGarden();
            foreach (var r in hiddenTransparent) if (r != null) r.enabled = false;
            foreach (var b in hiddenBehaviours) if (b is SkyMist && b != null) b.enabled = false;
            SetPortal(true);
            t0 = Time.time;
            state = State.Closing;
            SetVeil(1f);
        }

        void Restore()
        {
            foreach (var r in hiddenOpaque) if (r != null) r.enabled = true;
            foreach (var r in hiddenTransparent) if (r != null) r.enabled = true;
            foreach (var b in hiddenBehaviours) if (b != null) b.enabled = true;
            hiddenOpaque.Clear(); hiddenTransparent.Clear(); hiddenBehaviours.Clear();
        }

        void ShowGarden()
        {
            Restore();
            SetCameraTransparent(false);
            SetPortal(false);
            state = State.Garden;
            MoonStory.GardenHidden = false;
        }

        float openTime = 3.5f;

        void Animate()
        {
            float dur = state == State.Opening ? openTime : closeTime;
            float u = Mathf.Clamp01((Time.time - t0) / dur);
            float e = u * u * (3f - 2f * u);
            SetVeil(state == State.Opening ? e : 1f - e);
            if (u < 1f) return;
            if (state == State.Opening) ShowGarden();
            else EnterRoom();
        }

        /// <summary>0 = only the room (and the near things), 1 = the whole garden.</summary>
        void SetVeil(float open)
        {
            if (veil == null || cam == null) return;
            // the sphere sits around the head, just beyond the usu, so the usu, dough, kine and hands are in front of it
            var centre = link != null && link.Usu != null ? link.Usu.position + Vector3.up * 0.1f : cam.transform.position + cam.transform.forward;
            float dist = Vector3.Distance(cam.transform.position, centre);
            veil.transform.position = cam.transform.position;
            veil.transform.localScale = Vector3.one * Mathf.Clamp(dist + 0.6f, 1.2f, 4f) * 2f;
            if (usuKeep != null)
            {
                // the usu bowl with the dough and rice, and the kine wherever it is held; the pedestal marks itself
                usuKeep.position = link != null && link.Usu != null ? link.Usu.position + Vector3.up * 0.06f : centre;
                usuKeep.localScale = Vector3.one * usuKeepRadius * 2f;
                if (kineRenderer == null && link != null && link.Kine != null) kineRenderer = link.Kine.GetComponentInChildren<Renderer>();
                if (pedestalCopy != null && pedestal != null)
                {
                    var pt = pedestal.transform;
                    pedestalCopy.SetPositionAndRotation(pt.position, pt.rotation);
                    pedestalCopy.localScale = pt.lossyScale; // the portal holder is unscaled
                }
                bool kine = kineRenderer != null;
                kineKeep.gameObject.SetActive(kine);
                if (kine)
                {
                    var b = kineRenderer.bounds;
                    kineKeep.position = b.center;
                    kineKeep.localScale = Vector3.one * (b.extents.magnitude + 0.05f) * 2f;
                }
            }
            float soft = edgeSoftness * Mathf.Deg2Rad;
            veilInstance.SetVector("_Center", centre);
            // slow at first: the part in front of the player (around the usu) opens over most of the time
            veilInstance.SetFloat("_Radius", Mathf.Pow(Mathf.Clamp01(open), 1.7f) * (Mathf.PI + soft * 2f));
            veilInstance.SetFloat("_Soft", soft);
            veilInstance.SetFloat("_RimIntensity", rimIntensity * Mathf.Sin(Mathf.Clamp01(open) * Mathf.PI));
        }

        void EnsureVeil()
        {
            if (veil != null || veilMaterial == null || sphereMesh == null) return;
            portal = new GameObject("MR_Portal");
            portal.transform.SetParent(transform, false);
            veil = new GameObject("MR_PortalVeil");
            veil.transform.SetParent(portal.transform, false);
            veil.AddComponent<MeshFilter>().sharedMesh = sphereMesh;
            var mr = veil.AddComponent<MeshRenderer>();
            veilInstance = new Material(veilMaterial) { name = "MR_PortalVeil (runtime)" };
            veilInstance.renderQueue = 2999; // after the opaque scene and the sky, before text and glows
            mr.sharedMaterial = veilInstance;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
            if (keepMaterial != null)
            {
                usuKeep = MakeKeep("MR_PortalKeep_Usu", 64);
                kineKeep = MakeKeep("MR_PortalKeep_Kine", 128);
                pedestalKeep = new Material(keepMaterial) { name = "MR_PortalKeep_Pedestal (runtime)" };
                pedestalKeep.SetInt("_KeepBit", 32);
                pedestalKeep.renderQueue = 2998;
                keepInstances.Add(pedestalKeep);
                EnsurePedestalKeep();
            }
            portal.SetActive(false);
        }

        /// <summary>Shows or hides the portal (veil, keep bubbles, pedestal marker).</summary>
        void SetPortal(bool on)
        {
            if (portal != null) portal.SetActive(on);
        }

        // the pedestal marks itself exactly: a stencil-only copy of its mesh (its own materials are not touched)
        void EnsurePedestalKeep()
        {
            if (pedestalCopy != null || pedestalKeep == null || portal == null) return;
            foreach (var r in envRoot.GetComponentsInChildren<Renderer>(true)) if (r.name == "MR_UsuPedestal") { pedestal = r; break; }
            var mf = pedestal != null ? pedestal.GetComponent<MeshFilter>() : null;
            if (mf == null || mf.sharedMesh == null) return;
            var go = new GameObject("MR_PortalKeep_Pedestal");
            go.transform.SetParent(portal.transform, false);
            go.AddComponent<MeshFilter>().sharedMesh = mf.sharedMesh;
            var mr = go.AddComponent<MeshRenderer>();
            var mats = new Material[mf.sharedMesh.subMeshCount];
            for (int i = 0; i < mats.Length; i++) mats[i] = pedestalKeep;
            mr.sharedMaterials = mats;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
            pedestalCopy = go.transform;
        }

        Transform MakeKeep(string name, int bit)
        {
            var go = new GameObject(name);
            go.transform.SetParent(portal.transform, false);
            go.AddComponent<MeshFilter>().sharedMesh = sphereMesh;
            var mr = go.AddComponent<MeshRenderer>();
            var m = new Material(keepMaterial) { name = name + " (runtime)" };
            m.SetInt("_KeepBit", bit);
            m.renderQueue = 2998; // just before the veil
            keepInstances.Add(m);
            mr.sharedMaterial = m;
            mr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            mr.receiveShadows = false;
            return go.transform;
        }

        void SetCameraTransparent(bool transparent)
        {
            if (cam == null) cam = MoonStory.MainCamera();
            if (cam == null) return;
            if (transparent)
            {
                if (cam.clearFlags != CameraClearFlags.SolidColor || cam.backgroundColor.a > 0f)
                {
                    camFlags = cam.clearFlags;
                    camColor = cam.backgroundColor;
                }
                cam.clearFlags = CameraClearFlags.SolidColor;
                cam.backgroundColor = new Color(0f, 0f, 0f, 0f); // alpha 0: passthrough everywhere nothing is drawn
            }
            else if (cam.clearFlags == CameraClearFlags.SolidColor && cam.backgroundColor.a == 0f)
            {
                cam.clearFlags = camFlags == 0 ? CameraClearFlags.Skybox : camFlags;
                cam.backgroundColor = camColor;
            }
        }

        /// <summary>
        /// The garden = every renderer under MR_Environment except the story (text, orb) and the pedestal (the usu stands
        /// on it), plus the instanced bamboo and the sky mist.
        /// </summary>
        void CollectGarden()
        {
            if (cam == null) cam = MoonStory.MainCamera();
            hiddenOpaque.Clear(); hiddenTransparent.Clear(); hiddenBehaviours.Clear();
            foreach (var r in envRoot.GetComponentsInChildren<Renderer>(false))
            {
                if (!r.enabled || r.transform.IsChildOf(transform) || r.name == "MR_UsuPedestal") continue;
                
                bool transparent = false;
                foreach (var m in r.sharedMaterials) if (m != null && m.renderQueue >= 2500) transparent = true;
                (transparent ? hiddenTransparent : hiddenOpaque).Add(r);
            }
            foreach (var b in envRoot.GetComponentsInChildren<BambooGrove>(false)) if (b.enabled) hiddenBehaviours.Add(b);
            foreach (var b in envRoot.GetComponentsInChildren<SkyMist>(false)) if (b.enabled) hiddenBehaviours.Add(b);
        }
    }
}
