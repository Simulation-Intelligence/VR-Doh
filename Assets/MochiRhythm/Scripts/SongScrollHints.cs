using System.Collections.Generic;
using Oculus.Interaction.HandGrab;
using Oculus.Interaction.HandGrab.Visuals;
using Oculus.Interaction.Input;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Wordless hints for the song scrolls (<see cref="SongScrolls"/>):
    ///  - gold eighth notes (♪, ♫) drift up out of each scroll's top corners: these are songs;
    ///  - a flat dark silhouette with a gold rim of the scroll's mallet floats above it: what it turns into;
    ///  - a ghost hand (the knead guide's ISDK ghost) reaches to one scroll after another and pinches it, with the
    ///    finger shapes of Meta's recorded pinch (Resources/MochiRhythm/PinchPose.json): how to take one.
    /// The ghost hand only shows while nobody reaches for the scrolls.
    /// All drawing uses Sprites/Default (unlit, transparent), which the rhythm rings already use on the headset.
    /// </summary>
    public class SongScrollHints
    {
        public static readonly Color Gold = new Color(1f, 0.8f, 0.42f);
        static readonly Color SilhouetteFill = new Color(0.05f, 0.05f, 0.09f);

        public class Decor
        {
            public Transform silhouette;
            public Material fill, rim, notes;
            public readonly List<Transform> live = new List<Transform>();
            public readonly List<float> born = new List<float>();
            public readonly List<int> side = new List<int>();
            public float nextNote, baseY;
            public int count;
        }

        readonly Shader shader;
        readonly Mesh noteEighth, noteBeamed;
        readonly Vector2 paper;

        public SongScrollHints(Vector2 paperOnScreen)
        {
            paper = paperOnScreen;
            shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("UI/Default");
            noteEighth = EighthNote();
            noteBeamed = BeamedNotes();
        }

        Material Mat(string name, Color c, int queueOffset)
        {
            var m = new Material(shader) { name = name, color = c };
            m.renderQueue = 3000 + queueOffset;
            return m;
        }

        // ---- per scroll: silhouette and notes ----

        /// <summary>Adds the silhouette of <paramref name="malletMesh"/> above the scroll, and the note emitter.</summary>
        public Decor Decorate(Transform scroll, Mesh malletMesh, float maxSize, float above)
        {
            var d = new Decor
            {
                fill = Mat("Silhouette", SilhouetteFill, 2),
                rim = Mat("Silhouette rim", Gold, 1),
                notes = Mat("Melody", Gold, 3),
            };
            if (malletMesh != null)
            {
                var root = new GameObject("Mallet silhouette").transform;
                root.SetParent(scroll, false);
                // side view: the handle (kine -Z) points sideways (to -X), the head hangs down
                var turn = Quaternion.Euler(0f, 90f, 0f);
                Bounds b = RotatedBounds(malletMesh.bounds, turn);
                float k = maxSize / Mathf.Max(1e-3f, Mathf.Max(b.size.x, b.size.y));
                d.baseY = paper.y * 0.5f + above + b.size.y * k * 0.5f;
                root.localPosition = new Vector3(0f, d.baseY, -0.004f);
                root.localScale = new Vector3(k, k, k * 0.02f); // flat: a cut-out shape facing the player
                Part(root, "Fill", malletMesh, turn, -b.center, 1f, 0f, d.fill);
                Part(root, "Rim", malletMesh, turn, -b.center, 1.07f, 0.5f, d.rim); // a little larger, just behind
                d.silhouette = root;
            }
            return d;
        }

        static void Part(Transform root, string name, Mesh mesh, Quaternion turn, Vector3 centre, float grow, float behind, Material m)
        {
            var go = new GameObject(name);
            go.transform.SetParent(root, false);
            go.transform.localRotation = turn;
            go.transform.localScale = Vector3.one * grow;
            go.transform.localPosition = centre * grow + new Vector3(0f, 0f, behind);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var r = go.AddComponent<MeshRenderer>();
            var mats = new Material[Mathf.Max(1, mesh.subMeshCount)];
            for (int i = 0; i < mats.Length; i++) mats[i] = m;
            r.sharedMaterials = mats;
            r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            r.receiveShadows = false;
            r.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off;
        }

        static Bounds RotatedBounds(Bounds b, Quaternion q)
        {
            var r = new Bounds(q * b.center, Vector3.zero);
            for (int i = 0; i < 8; i++)
            {
                var c = new Vector3((i & 1) == 0 ? b.min.x : b.max.x, (i & 2) == 0 ? b.min.y : b.max.y, (i & 4) == 0 ? b.min.z : b.max.z);
                r.Encapsulate(q * c);
            }
            return r;
        }

        /// <summary>Fades the scroll's hints with <paramref name="alpha"/>; <paramref name="glow"/> (0..1) brightens them.</summary>
        public void Tick(Decor d, Transform scroll, float alpha, float glow)
        {
            if (d == null) return;
            var f = SilhouetteFill; f.a = 0.9f * alpha;
            d.fill.color = f;
            var g = Color.Lerp(Gold * 0.75f, Gold * 1.25f, glow); g.a = (0.45f + 0.55f * glow) * alpha;
            d.rim.color = g;
            if (d.silhouette != null)
            {
                float s = 1f + 0.08f * glow;
                d.silhouette.localPosition = new Vector3(0f, d.baseY + 0.004f * Mathf.Sin(Time.time * 1.7f), -0.004f); // a slow bob
                d.silhouette.GetChild(0).localScale = Vector3.one * s;
                d.silhouette.GetChild(1).localScale = Vector3.one * 1.07f * s;
            }

            // notes: one every ~0.75 s from alternating top corners, rising, swaying, fading
            if (alpha > 0.5f && Time.time >= d.nextNote)
            {
                d.nextNote = Time.time + Random.Range(0.6f, 0.9f);
                int side = d.count++ % 2 == 0 ? -1 : 1;
                var n = new GameObject("Note");
                n.transform.SetParent(scroll, false);
                n.AddComponent<MeshFilter>().sharedMesh = Random.value < 0.5f ? noteEighth : noteBeamed;
                var r = n.AddComponent<MeshRenderer>();
                r.sharedMaterial = d.notes;
                r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                r.receiveShadows = false;
                d.live.Add(n.transform); d.born.Add(Time.time); d.side.Add(side);
            }
            const float life = 1.9f;
            for (int i = d.live.Count - 1; i >= 0; i--)
            {
                float age = Time.time - d.born[i];
                var t = d.live[i];
                if (age > life || t == null)
                {
                    if (t != null) Object.Destroy(t.gameObject);
                    d.live.RemoveAt(i); d.born.RemoveAt(i); d.side.RemoveAt(i);
                    continue;
                }
                float u = age / life;
                int sd = d.side[i];
                float x = sd * (paper.x * 0.5f - 0.035f + 0.03f * u) + 0.006f * Mathf.Sin(age * 5f + sd);
                float y = paper.y * 0.5f - 0.01f + 0.085f * u;
                t.localPosition = new Vector3(x, y, -0.006f);
                t.localRotation = Quaternion.Euler(0f, 0f, sd * -12f * u + 6f * Mathf.Sin(age * 3f));
                t.localScale = Vector3.one * (0.6f + 0.4f * Mathf.Min(1f, age / 0.25f)) * Mathf.Clamp01((life - age) / 0.45f);
            }
            var nc = Gold; nc.a = 0.9f * alpha;
            d.notes.color = nc; // the notes fade with the scroll; each one shrinks away at the end of its rise
        }

        /// <summary>Removes the live notes (scroll closed).</summary>
        public void ClearNotes(Decor d)
        {
            if (d == null) return;
            foreach (var t in d.live) if (t != null) Object.Destroy(t.gameObject);
            d.live.Clear(); d.born.Clear(); d.side.Clear();
        }

        // ---- note shapes (metres, in the scroll plane, readable from -Z) ----

        static Mesh EighthNote()
        {
            var v = new List<Vector3>(); var tri = new List<int>();
            Ellipse(v, tri, Vector2.zero, 0.0075f, 0.0055f, -25f);
            Rect(v, tri, new Vector2(0.0058f, 0.001f), new Vector2(0.0074f, 0.033f));
            // flag: a curved band down and to the right from the stem's top
            Vector2 a0 = new Vector2(0.0074f, 0.033f), a1 = new Vector2(0.016f, 0.024f), a2 = new Vector2(0.0135f, 0.012f);
            Band(v, tri, a0, a1, a2, 0.0045f, 0.0012f);
            return Build("Eighth note", v, tri);
        }

        static Mesh BeamedNotes()
        {
            var v = new List<Vector3>(); var tri = new List<int>();
            Ellipse(v, tri, Vector2.zero, 0.0072f, 0.0052f, -25f);
            Ellipse(v, tri, new Vector2(0.019f, 0.004f), 0.0072f, 0.0052f, -25f);
            Rect(v, tri, new Vector2(0.0056f, 0.001f), new Vector2(0.0071f, 0.032f));
            Rect(v, tri, new Vector2(0.0246f, 0.005f), new Vector2(0.0261f, 0.036f));
            Quad(v, tri, new Vector2(0.0056f, 0.0275f), new Vector2(0.0261f, 0.0315f), new Vector2(0.0261f, 0.036f), new Vector2(0.0056f, 0.032f));
            return Build("Beamed notes", v, tri);
        }

        static void Ellipse(List<Vector3> v, List<int> tri, Vector2 c, float rx, float ry, float deg)
        {
            const int n = 16;
            int centre = v.Count;
            v.Add(c);
            float a = deg * Mathf.Deg2Rad, ca = Mathf.Cos(a), sa = Mathf.Sin(a);
            for (int i = 0; i < n; i++)
            {
                float t = i * Mathf.PI * 2f / n;
                float x = Mathf.Cos(t) * rx, y = Mathf.Sin(t) * ry;
                v.Add(new Vector3(c.x + x * ca - y * sa, c.y + x * sa + y * ca, 0f));
            }
            for (int i = 0; i < n; i++) { tri.Add(centre); tri.Add(centre + 1 + i); tri.Add(centre + 1 + (i + 1) % n); }
        }

        static void Rect(List<Vector3> v, List<int> tri, Vector2 lo, Vector2 hi) =>
            Quad(v, tri, lo, new Vector2(hi.x, lo.y), hi, new Vector2(lo.x, hi.y));

        static void Quad(List<Vector3> v, List<int> tri, Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            int i = v.Count;
            v.Add(a); v.Add(b); v.Add(c); v.Add(d);
            tri.Add(i); tri.Add(i + 1); tri.Add(i + 2); tri.Add(i); tri.Add(i + 2); tri.Add(i + 3);
        }

        // a band along a quadratic Bézier, its width tapering from w0 to w1
        static void Band(List<Vector3> v, List<int> tri, Vector2 p0, Vector2 p1, Vector2 p2, float w0, float w1)
        {
            const int n = 8;
            int start = v.Count;
            for (int i = 0; i <= n; i++)
            {
                float t = i / (float)n, u = 1f - t;
                Vector2 p = u * u * p0 + 2f * u * t * p1 + t * t * p2;
                Vector2 d = (2f * u * (p1 - p0) + 2f * t * (p2 - p1)).normalized;
                Vector2 nrm = new Vector2(-d.y, d.x) * Mathf.Lerp(w0, w1, t) * 0.5f;
                v.Add(p + nrm); v.Add(p - nrm);
            }
            for (int i = 0; i < n; i++)
            {
                int a = start + i * 2;
                tri.Add(a); tri.Add(a + 1); tri.Add(a + 2); tri.Add(a + 1); tri.Add(a + 3); tri.Add(a + 2);
            }
        }

        static Mesh Build(string name, List<Vector3> v, List<int> tri)
        {
            var m = new Mesh { name = name };
            m.SetVertices(v);
            m.SetTriangles(tri, 0);
            m.RecalculateBounds();
            return m;
        }

        // ---- the pinch demonstration ----

        [System.Serializable] class PinchFile { public float[] open, pinch; }

        public class PinchHint
        {
            public HandGhost ghost;
            public HandPose open, pinch;
            public Vector3 tipOffset; // pinch point from the wrist, in the frame LookRotation(fingers, palm)
            public Transform thumbTip, indexTip;
        }

        /// <summary>Makes the ghost hand and loads the pinch shapes; null if the knead guide or the pose file is missing.</summary>
        public static PinchHint MakePinch(KneadGhostGuide guide, Transform parent)
        {
            var file = Resources.Load<TextAsset>("MochiRhythm/PinchPose");
            if (guide == null || file == null) return null;
            var data = JsonUtility.FromJson<PinchFile>(file.text);
            int joints = FingersMetadata.HAND_JOINT_IDS.Length;
            if (data?.open == null || data.pinch == null || data.open.Length < joints * 4 || data.pinch.Length < joints * 4) return null;
            var ghost = guide.MakeGhost(Handedness.Right, parent);
            if (ghost == null) return null;
            ghost.name = "PinchHint_Right";
            var h = new PinchHint { ghost = ghost, open = Pose(data.open), pinch = Pose(data.pinch) };
            foreach (var t in ghost.GetComponentsInChildren<Transform>(true))
            {
                if (t.name == "XRHand_ThumbTip") h.thumbTip = t;
                if (t.name == "XRHand_IndexTip") h.indexTip = t;
            }
            // where the fingertips meet, relative to the wrist, with fingers along +Z and the palm facing -Y
            guide.PoseGhost(ghost, h.pinch, Vector3.zero, Vector3.forward, Vector3.down);
            Vector3 tips = h.thumbTip != null && h.indexTip != null ? (h.thumbTip.position + h.indexTip.position) * 0.5f : new Vector3(0f, -0.03f, 0.12f);
            h.tipOffset = Quaternion.Inverse(Quaternion.LookRotation(Vector3.forward, Vector3.down)) * tips;
            ghost.gameObject.SetActive(false);
            return h;

            HandPose Pose(float[] q)
            {
                var p = new HandPose(Handedness.Right);
                for (int i = 0; i < joints; i++) p.JointRotations[i] = new Quaternion(q[i * 4], q[i * 4 + 1], q[i * 4 + 2], q[i * 4 + 3]);
                return p;
            }
        }
    }
}
