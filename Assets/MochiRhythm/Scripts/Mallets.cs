using System.Collections.Generic;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// The mallets a song scroll can turn into, after real rice-cake pounding tools. All share the kine's frame:
    /// origin = centre of the flat striking face, -Y = strike direction, head along +Y. Only the sizes change, so
    /// the dough solver, strike detection and scoring treat them all the same way.
    ///  - Yokogine (横杵): the Japanese T-shaped festival mallet, a round head with a long handle out to the side.
    ///    The team's kine as it already is.
    ///  - Tteokme (떡메): the Korean tteok mallet, a heavy rough log head on a thick handle.
    ///  - Jeolgut-gongi (절굿공이): the Korean mortar pestle the Moon Rabbit holds, a long rod with a thick end at
    ///    each end, held in the middle and driven straight down.
    /// </summary>
    public class Mallet
    {
        public string name, native, songMatch;
        public float headRadius, headLength, faceRounding;
        /// <summary>The handle as the GrabZone segment (kine local), and the contact radius of the handle.</summary>
        public Vector3 gripA, gripB;
        public float gripRadius, handleRadius;
        /// <summary>Where the hand is put on the handle when the scroll turns into the mallet (kine local).</summary>
        public Vector3 handPoint;
        /// <summary>Handle along the head axis (the pestle): the partner's tilt over the rim does not apply.</summary>
        public bool vertical;
        /// <summary>Builds the visible mesh; null = keep the kine's own skin. Sub-meshes: see <see cref="MalletMeshes"/>.</summary>
        public System.Func<Mallet, Mesh> build;

        /// <summary>Shown under the scroll. The story's fonts have no Hangul or kanji, so the name only.</summary>
        public string Label => name;

        public static List<Mallet> All(KineTool kine, GrabZone zone, float handleRadius) => new List<Mallet>
        {
            new Mallet
            {
                name = "Tteokme", native = "떡메", songMatch = "bangarang",
                headRadius = 0.075f, headLength = 0.26f, faceRounding = 0.02f,
                gripA = new Vector3(0f, 0.13f, 0f), gripB = new Vector3(0f, 0.13f, -0.62f), gripRadius = 0.055f, handleRadius = 0.021f,
                handPoint = new Vector3(0f, 0.13f, -0.54f), build = MalletMeshes.Tteokme,
            },
            new Mallet
            {
                name = "Yokogine", native = "横杵", songMatch = "believer",
                headRadius = kine.headRadius, headLength = kine.headLength, faceRounding = kine.faceRounding,
                gripA = zone.segmentA, gripB = zone.segmentB, gripRadius = zone.segmentRadius, handleRadius = handleRadius,
                handPoint = Vector3.Lerp(zone.segmentA, zone.segmentB, 0.88f), build = null,
            },
            new Mallet
            {
                name = "Jeolgut-gongi", native = "절굿공이", songMatch = "gangnam",
                headRadius = 0.045f, headLength = 0.17f, faceRounding = 0.014f,
                gripA = new Vector3(0f, 0.22f, 0f), gripB = new Vector3(0f, 0.48f, 0f), gripRadius = 0.05f, handleRadius = 0.024f,
                handPoint = new Vector3(0f, 0.34f, 0f), vertical = true, build = MalletMeshes.Pestle,
            },
        };
    }

    /// <summary>
    /// Lathe-turned mallet meshes made at runtime. Sub-meshes, in the order of the materials passed with them:
    /// 0 handle wood, 1 head wood, 2 pink wrap, 3 dark log wood, 4 end grain.
    /// </summary>
    public static class MalletMeshes
    {
        public const int SubMeshes = 5;
        const int N = 20;

        public static Mesh Tteokme(Mallet m)
        {
            var b = new Builder();
            float R = m.headRadius, L = m.headLength, f = m.faceRounding;
            // a log, slightly barrel-shaped, end grain on both faces, two lighter hoops near the ends
            b.Lathe(new[] { V(0f, 0f), V(R - f, 0f) }, 4);
            b.Lathe(new[] { V(R - f, 0f), V(R - f * 0.25f, f * 0.4f), V(R, f), V(R * 1.04f, L * 0.5f), V(R, L - f), V(R - f * 0.25f, L - f * 0.4f), V(R - f, L) }, 3);
            b.Lathe(new[] { V(R - f, L), V(0f, L) }, 4);
            foreach (float y in new[] { f + 0.02f, L - f - 0.02f })
            {
                float r = Mathf.Lerp(R, R * 1.04f, 1f - Mathf.Abs(y - L * 0.5f) / (L * 0.5f)) + 0.003f;
                b.Lathe(new[] { V(r, y - 0.009f), V(r, y + 0.009f) }, 1);
            }
            // thick straight handle out the side, a pink grip wrap near the end
            Vector3 a = new Vector3(0f, m.gripA.y, -R * 0.6f), e = m.gripB;
            b.Frustum(a, e, m.handleRadius * 1.1f, m.handleRadius, 10, 0, false, true);
            Vector3 dir = (e - a).normalized;
            b.Frustum(e - dir * 0.15f, e - dir * 0.02f, m.handleRadius * 1.18f, m.handleRadius * 1.15f, 10, 2, true, true);
            return b.ToMesh("Tteokme");
        }

        public static Mesh Pestle(Mallet m)
        {
            var b = new Builder();
            float R = m.headRadius, L = m.headLength, f = m.faceRounding, w = m.handleRadius;
            float top = m.gripB.y + 0.17f;
            b.Lathe(new[] { V(0f, 0f), V(R - f, 0f) }, 4);
            // thick striking end, a long waist to hold, a thick end on top (both ends are used in turn)
            b.Lathe(new[]
            {
                V(R - f, 0f), V(R - f * 0.25f, f * 0.4f), V(R, f), V(R, L - 0.02f), V(w * 1.15f, m.gripA.y),
                V(w, m.gripA.y + 0.04f), V(w, m.gripB.y - 0.04f), V(w * 1.15f, m.gripB.y), V(R * 0.95f, m.gripB.y + 0.05f),
                V(R * 0.95f, top - f), V(R * 0.95f - f, top),
            }, 1);
            b.Lathe(new[] { V(R * 0.95f - f, top), V(0f, top) }, 4);
            // pink wrap where the hands go
            b.Lathe(new[] { V(w + 0.003f, m.handPoint.y - 0.06f), V(w + 0.003f, m.handPoint.y + 0.06f) }, 2);
            return b.ToMesh("Jeolgut-gongi");
        }

        static Vector2 V(float r, float y) => new Vector2(r, y);

        class Builder
        {
            readonly List<Vector3> verts = new List<Vector3>(), normals = new List<Vector3>();
            readonly List<int>[] tris = new List<int>[SubMeshes];

            public Builder() { for (int i = 0; i < SubMeshes; i++) tris[i] = new List<int>(); }

            /// <summary>Turns a (radius, height) profile around +Y. Each segment gets its own ring, so edges stay crisp.</summary>
            public void Lathe(IList<Vector2> profile, int sub)
            {
                for (int i = 0; i + 1 < profile.Count; i++)
                {
                    Vector2 p0 = profile[i], p1 = profile[i + 1];
                    Ring(new Vector3(0f, p0.y, 0f), new Vector3(0f, p1.y, 0f), Vector3.up, p0.x, p1.x, sub);
                }
            }

            /// <summary>A truncated cone between two points, optionally capped.</summary>
            public void Frustum(Vector3 p0, Vector3 p1, float r0, float r1, int sides, int sub, bool cap0, bool cap1)
            {
                Vector3 axis = (p1 - p0).normalized;
                Ring(p0, p1, axis, r0, r1, sub, sides);
                if (cap0) Disc(p0, -axis, r0, sub, sides);
                if (cap1) Disc(p1, axis, r1, sub, sides);
            }

            void Ring(Vector3 c0, Vector3 c1, Vector3 axis, float r0, float r1, int sub, int sides = N)
            {
                Basis(axis, out Vector3 u, out Vector3 v);
                float len = Vector3.Distance(c0, c1);
                // flat ring (a cap in the profile): normal along the axis
                bool flat = len < 1e-5f;
                float slope = flat ? 0f : (r0 - r1) / len;
                Vector3 capNormal = r1 < r0 ? axis : -axis;
                int start = verts.Count;
                for (int k = 0; k <= sides; k++)
                {
                    float a = k * Mathf.PI * 2f / sides;
                    Vector3 d = Mathf.Cos(a) * u + Mathf.Sin(a) * v;
                    Vector3 n = flat ? capNormal : (d + axis * slope).normalized;
                    verts.Add(c0 + d * r0); normals.Add(n);
                    verts.Add(c1 + d * r1); normals.Add(n);
                }
                for (int k = 0; k < sides; k++)
                {
                    int i0 = start + k * 2, i1 = i0 + 1, i2 = i0 + 2, i3 = i0 + 3;
                    Face(i0, i1, i2, sub, flat ? capNormal : Vector3.zero);
                    Face(i2, i1, i3, sub, flat ? capNormal : Vector3.zero);
                }
            }

            void Disc(Vector3 c, Vector3 n, float r, int sub, int sides)
            {
                Basis(n, out Vector3 u, out Vector3 v);
                int centre = verts.Count;
                verts.Add(c); normals.Add(n);
                for (int k = 0; k <= sides; k++)
                {
                    float a = k * Mathf.PI * 2f / sides;
                    verts.Add(c + (Mathf.Cos(a) * u + Mathf.Sin(a) * v) * r); normals.Add(n);
                }
                for (int k = 0; k < sides; k++) Face(centre, centre + 1 + k, centre + 2 + k, sub, n);
            }

            // winding chosen so the triangle faces along its vertex normals (or the given facing)
            void Face(int a, int b, int c, int sub, Vector3 facing)
            {
                Vector3 n = Vector3.Cross(verts[b] - verts[a], verts[c] - verts[a]);
                Vector3 want = facing != Vector3.zero ? facing : normals[a] + normals[b] + normals[c];
                if (Vector3.Dot(n, want) < 0f) { tris[sub].Add(a); tris[sub].Add(c); tris[sub].Add(b); }
                else { tris[sub].Add(a); tris[sub].Add(b); tris[sub].Add(c); }
            }

            static void Basis(Vector3 axis, out Vector3 u, out Vector3 v)
            {
                u = Vector3.Cross(axis, Mathf.Abs(axis.y) < 0.9f ? Vector3.up : Vector3.right).normalized;
                v = Vector3.Cross(axis, u);
            }

            public Mesh ToMesh(string name)
            {
                var mesh = new Mesh { name = name };
                mesh.SetVertices(verts);
                mesh.SetNormals(normals);
                mesh.subMeshCount = SubMeshes;
                for (int i = 0; i < SubMeshes; i++) mesh.SetTriangles(tris[i], i);
                mesh.RecalculateBounds();
                return mesh;
            }
        }
    }
}
