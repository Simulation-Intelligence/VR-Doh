using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.WorldEditor
{
    /// <summary>
    /// Builds flat-shaded low-poly meshes with several sub-meshes (one per material).
    /// Every triangle takes a "facing" hint, so winding is always correct.
    /// </summary>
    public class MeshBuilder
    {
        readonly List<Vector3> verts = new List<Vector3>();
        readonly List<Vector3> normals = new List<Vector3>();
        readonly List<Color> colors = new List<Color>();
        readonly List<Vector2> uvs = new List<Vector2>();
        readonly List<List<int>> subs = new List<List<int>>();

        public Matrix4x4 matrix = Matrix4x4.identity;
        public Color color = Color.white;

        public MeshBuilder(int subMeshCount = 1)
        {
            for (int i = 0; i < subMeshCount; i++) subs.Add(new List<int>());
        }

        public int VertexCount => verts.Count;

        /// <summary>Adds a triangle. <paramref name="facing"/> is a direction the front face should point toward (local space).</summary>
        public void Tri(Vector3 a, Vector3 b, Vector3 c, Vector3 facing, int sub = 0)
        {
            var n = Vector3.Cross(b - a, c - a);
            if (n.sqrMagnitude < 1e-14f) return;
            if (Vector3.Dot(n, facing) < 0) { var t = b; b = c; c = t; }

            a = matrix.MultiplyPoint3x4(a);
            b = matrix.MultiplyPoint3x4(b);
            c = matrix.MultiplyPoint3x4(c);
            var wn = Vector3.Cross(b - a, c - a).normalized;

            int i = verts.Count;
            verts.Add(a); verts.Add(b); verts.Add(c);
            normals.Add(wn); normals.Add(wn); normals.Add(wn);
            colors.Add(color); colors.Add(color); colors.Add(color);
            uvs.Add(new Vector2(0, 0)); uvs.Add(new Vector2(1, 0)); uvs.Add(new Vector2(0, 1));
            var list = subs[sub];
            list.Add(i); list.Add(i + 1); list.Add(i + 2);
        }

        public void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 facing, int sub = 0)
        {
            Tri(a, b, c, facing, sub);
            Tri(a, c, d, facing, sub);
        }

        /// <summary>Two-sided triangle (leaves).</summary>
        public void TwoSided(Vector3 a, Vector3 b, Vector3 c, int sub = 0)
        {
            var n = Vector3.Cross(b - a, c - a);
            Tri(a, b, c, n, sub);
            Tri(a, b, c, -n, sub);
        }

        /// <summary>Tapered cylinder from p0 to p1 (flat facets).</summary>
        public void Frustum(Vector3 p0, Vector3 p1, float r0, float r1, int sides, int sub = 0,
                            bool capBottom = true, bool capTop = true, float angleOffset = 0f)
        {
            var axis = p1 - p0;
            if (axis.sqrMagnitude < 1e-10f) return;
            var up = axis.normalized;
            var side = Vector3.Cross(up, Mathf.Abs(up.y) < 0.95f ? Vector3.up : Vector3.right).normalized;
            var fwd = Vector3.Cross(side, up);

            var bottom = new Vector3[sides];
            var top = new Vector3[sides];
            var radial = new Vector3[sides];
            for (int i = 0; i < sides; i++)
            {
                float a = angleOffset + i * Mathf.PI * 2f / sides;
                radial[i] = side * Mathf.Cos(a) + fwd * Mathf.Sin(a);
                bottom[i] = p0 + radial[i] * r0;
                top[i] = p1 + radial[i] * r1;
            }
            for (int i = 0; i < sides; i++)
            {
                int j = (i + 1) % sides;
                var outward = (radial[i] + radial[j]).normalized;
                Quad(bottom[i], top[i], top[j], bottom[j], outward, sub);
                if (capBottom && r0 > 0) Tri(p0, bottom[i], bottom[j], -up, sub);
                if (capTop && r1 > 0) Tri(p1, top[i], top[j], up, sub);
            }
        }

        /// <summary>
        /// Surface of revolution around local Y. Profile is (radius, height) points walked so that
        /// the visible side is on the right: outer wall upward, rim inward, inner wall downward.
        /// </summary>
        public void Lathe(IList<Vector2> profile, int sides, int sub = 0, float angleOffset = 0f)
        {
            for (int k = 0; k < profile.Count - 1; k++)
            {
                var p = profile[k];
                var q = profile[k + 1];
                var n2 = new Vector2(q.y - p.y, -(q.x - p.x)); // right-hand normal in (r, y)
                for (int i = 0; i < sides; i++)
                {
                    float a0 = angleOffset + i * Mathf.PI * 2f / sides;
                    float a1 = angleOffset + (i + 1) * Mathf.PI * 2f / sides;
                    var d0 = new Vector3(Mathf.Cos(a0), 0, Mathf.Sin(a0));
                    var d1 = new Vector3(Mathf.Cos(a1), 0, Mathf.Sin(a1));
                    var dm = (d0 + d1).normalized;
                    var facing = dm * n2.x + Vector3.up * n2.y;
                    var a = d0 * p.x + Vector3.up * p.y;
                    var b = d0 * q.x + Vector3.up * q.y;
                    var c = d1 * q.x + Vector3.up * q.y;
                    var d = d1 * p.x + Vector3.up * p.y;
                    Quad(a, b, c, d, facing, sub);
                }
            }
        }

        /// <summary>Flat-shaded, noisy low-poly blob (rocks, canopy).</summary>
        public void Blob(Vector3 center, Vector3 scale, int subdivisions, float noise, System.Random rng, int sub = 0)
        {
            Icosphere.Build(subdivisions, out var pts, out var tris);
            var jitter = new float[pts.Count];
            for (int i = 0; i < pts.Count; i++) jitter[i] = 1f + ((float)rng.NextDouble() * 2f - 1f) * noise;
            var p = new Vector3[pts.Count];
            for (int i = 0; i < pts.Count; i++)
                p[i] = center + Vector3.Scale(pts[i] * jitter[i], scale);
            for (int t = 0; t < tris.Count; t += 3)
            {
                var a = p[tris[t]]; var b = p[tris[t + 1]]; var c = p[tris[t + 2]];
                Tri(a, b, c, (a + b + c) / 3f - center, sub);
            }
        }

        public Mesh ToMesh(string name)
        {
            var m = new Mesh { name = name };
            if (verts.Count > 65000) m.indexFormat = IndexFormat.UInt32;
            m.SetVertices(verts);
            m.SetNormals(normals);
            m.SetColors(colors);
            m.SetUVs(0, uvs);
            m.subMeshCount = subs.Count;
            for (int i = 0; i < subs.Count; i++) m.SetTriangles(subs[i], i, false);
            m.RecalculateBounds();
            return m;
        }
    }

    /// <summary>Unit icosphere (shared vertices).</summary>
    public static class Icosphere
    {
        public static void Build(int subdivisions, out List<Vector3> points, out List<int> tris)
        {
            float t = (1f + Mathf.Sqrt(5f)) / 2f;
            points = new List<Vector3>
            {
                new Vector3(-1, t, 0), new Vector3(1, t, 0), new Vector3(-1, -t, 0), new Vector3(1, -t, 0),
                new Vector3(0, -1, t), new Vector3(0, 1, t), new Vector3(0, -1, -t), new Vector3(0, 1, -t),
                new Vector3(t, 0, -1), new Vector3(t, 0, 1), new Vector3(-t, 0, -1), new Vector3(-t, 0, 1),
            };
            for (int i = 0; i < points.Count; i++) points[i] = points[i].normalized;
            tris = new List<int>
            {
                0,11,5, 0,5,1, 0,1,7, 0,7,10, 0,10,11,
                1,5,9, 5,11,4, 11,10,2, 10,7,6, 7,1,8,
                3,9,4, 3,4,2, 3,2,6, 3,6,8, 3,8,9,
                4,9,5, 2,4,11, 6,2,10, 8,6,7, 9,8,1,
            };
            for (int s = 0; s < subdivisions; s++)
            {
                var cache = new Dictionary<long, int>();
                var next = new List<int>(tris.Count * 4);
                var pts = points;
                int Mid(int a, int b)
                {
                    long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                    if (cache.TryGetValue(key, out int idx)) return idx;
                    pts.Add(((pts[a] + pts[b]) * 0.5f).normalized);
                    cache[key] = pts.Count - 1;
                    return pts.Count - 1;
                }
                for (int i = 0; i < tris.Count; i += 3)
                {
                    int a = tris[i], b = tris[i + 1], c = tris[i + 2];
                    int ab = Mid(a, b), bc = Mid(b, c), ca = Mid(c, a);
                    next.AddRange(new[] { a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca });
                }
                tris = next;
            }
        }

        /// <summary>Smooth sphere mesh with per-vertex colors (Moon, Earth).</summary>
        public static Mesh SmoothSphere(string name, int subdivisions, Func<Vector3, Color> colorAt)
        {
            Build(subdivisions, out var pts, out var tris);
            // Make sure triangles face outward (Unity: clockwise = front).
            for (int i = 0; i < tris.Count; i += 3)
            {
                var a = pts[tris[i]]; var b = pts[tris[i + 1]]; var c = pts[tris[i + 2]];
                if (Vector3.Dot(Vector3.Cross(b - a, c - a), a + b + c) < 0)
                {
                    int tmp = tris[i + 1]; tris[i + 1] = tris[i + 2]; tris[i + 2] = tmp;
                }
            }
            var cols = new List<Color>(pts.Count);
            foreach (var p in pts) cols.Add(colorAt(p));
            var m = new Mesh { name = name };
            m.SetVertices(pts);
            m.SetNormals(pts);
            m.SetColors(cols);
            m.SetTriangles(tris, 0);
            m.RecalculateBounds();
            return m;
        }
    }
}
