using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Collision of the kine with the usu, the table and free hands, as plain geometry (no scene objects), so
/// KineContact can drive it and a test can run it off-Editor. A port of the three.js scene's src/kineContact.js
/// (immerse/mochi-pounding), which collides exactly the shapes that are drawn:
///  - the usu and both parts of the kine (head, handle) are solids of revolution, each given by the cross-section
///    its visible mesh is lathed from (UsuKineSkinBuilder's MR_UsuSkin / MR_KineSkin, or UsuTool's lathe and the
///    KineTool cylinder without the skins). The distance to a solid of revolution is the 2D distance in its
///    cross-section, so every distance here is exact for the drawn shape, up to its facets;
///  - contacts: the deepest point of the kine's surface inside the usu or the table (each surface patch scanned,
///    then refined), the usu's convex corners (circles: the rim's edges, the band) inside the kine, and hand balls;
///  - all current contacts are cleared together by one move: a pivot about the grip (or a push, when the push runs
///    along the handle and pivoting can't help), bisected to the smallest move that clears every one of them, so
///    the kine ends touching, not hovering. The move may be at most about the overlap's size (4x, or the handle's
///    radius): a larger one would swing the head over or around what it hit, a jump to another surface. If neither
///    kind of move clears the contacts that locally, the kine is wedged and keeps its last pose;
///  - motion is marched from last frame's resolved pose toward this frame's pose in steps no longer than the free
///    space around the kine allows (and at least the handle's radius near something), settling after each. Once a
///    step is blocked and keeps less than half its advance, the kine stays there, on the near side: a hand that
///    carries on through the usu leaves the kine resting on it instead of popping it out on the far side.
/// Station-local metres throughout; kine parts are in kine-local space (origin = centre of the flat striking face,
/// +Y = head axis, the handle along -Z).
/// </summary>
public sealed class KineCollision
{
    public const float Eps = 1e-6f; // m: overlap below this counts as touching
    public enum Kind { Usu, Table, Hand }

    /// <summary>A solid of revolution: cross-section poly (x = distance from the axis, y = height along it from
    /// origin), counter-clockwise; surface[i] is false for an edge poly[i] -> poly[i + 1] that is not a surface
    /// (along the axis, buried in what the usu stands on, hidden inside another part).</summary>
    public sealed class Revolved
    {
        public readonly Vector2[] poly;
        public readonly bool[] surface;
        public Vector3 origin, axis, e1, e2;

        public Revolved(Vector2[] poly, bool[] surface, Vector3 origin, Vector3 axis)
        {
            this.poly = poly;
            this.surface = surface;
            this.origin = origin;
            this.axis = axis.normalized;
            e1 = Vector3.Cross(this.axis, Mathf.Abs(this.axis.x) < 0.9f ? Vector3.right : Vector3.forward).normalized;
            e2 = Vector3.Cross(this.axis, e1);
        }

        public Vector3 Point(float rho, float h, float theta) =>
            origin + axis * h + (e1 * Mathf.Cos(theta) + e2 * Mathf.Sin(theta)) * rho;

        /// <summary>Signed distance (negative inside) from p, in the owner's frame, and the outward unit normal.</summary>
        public float Sdf(Vector3 p, out Vector3 normal)
        {
            Vector3 d = p - origin;
            float h = Vector3.Dot(d, axis);
            Vector3 radial = d - axis * h;
            float rho = radial.magnitude;
            Vector3 ru = rho > 1e-9f ? radial / rho : e1;
            float sd = PolySdf(poly, surface, rho, h, out float gr, out float gh);
            normal = ru * gr + axis * gh;
            return sd;
        }

        /// <summary>The largest distance from the axis of any surface vertex (for bounds and step sizes).</summary>
        public float MaxRadius()
        {
            float m = 0f;
            foreach (var v in poly) m = Mathf.Max(m, v.x);
            return m;
        }
    }

    /// <summary>2D signed distance to a closed counter-clockwise polygon, counting only its surface edges, and the
    /// outward unit gradient (gx, gy). Inside by crossing number over every edge.</summary>
    public static float PolySdf(Vector2[] poly, bool[] surface, float px, float py, out float gx, out float gy)
    {
        bool inside = false;
        float best = float.MaxValue, qx = 0f, qy = 0f, ex = 0f, ey = 1f;
        int n = poly.Length;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = poly[i], b = poly[i + 1 == n ? 0 : i + 1];
            if ((a.y > py) != (b.y > py) && px < a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y)) inside = !inside;
            if (!surface[i]) continue;
            float abx = b.x - a.x, aby = b.y - a.y, l2 = abx * abx + aby * aby;
            float t = l2 > 0f ? Mathf.Clamp01(((px - a.x) * abx + (py - a.y) * aby) / l2) : 0f;
            float cx = a.x + abx * t, cy = a.y + aby * t, dd = (px - cx) * (px - cx) + (py - cy) * (py - cy);
            if (dd < best)
            {
                best = dd; qx = cx; qy = cy;
                float l = Mathf.Sqrt(l2);
                if (l > 0f) { ex = aby / l; ey = -abx / l; }
            }
        }
        float dist = Mathf.Sqrt(best), sd = inside ? -dist : dist;
        if (dist > 1e-9f) { gx = (px - qx) / sd; gy = (py - qy) / sd; } // (p - q) / sd points outward
        else { gx = ex; gy = ey; }
        return sd;
    }

    /// <summary>An axis-aligned box (the table under the usu), station frame.</summary>
    public sealed class Box
    {
        public Vector3 lo, hi;

        public float Sdf(Vector3 c, out Vector3 normal)
        {
            Vector3 centre = (lo + hi) * 0.5f, half = (hi - lo) * 0.5f, d = c - centre;
            Vector3 q = new Vector3(Mathf.Abs(d.x) - half.x, Mathf.Abs(d.y) - half.y, Mathf.Abs(d.z) - half.z);
            Vector3 outside = new Vector3(Mathf.Max(q.x, 0f), Mathf.Max(q.y, 0f), Mathf.Max(q.z, 0f));
            float outLen = outside.magnitude;
            if (outLen > 0f)
            {
                normal = new Vector3(outside.x * Mathf.Sign(d.x), outside.y * Mathf.Sign(d.y), outside.z * Mathf.Sign(d.z)) / outLen;
                return outLen;
            }
            if (q.y >= q.x && q.y >= q.z) normal = new Vector3(0f, Mathf.Sign(d.y), 0f); // inside: out through the nearest face
            else if (q.x >= q.z) normal = new Vector3(Mathf.Sign(d.x), 0f, 0f);
            else normal = new Vector3(0f, 0f, Mathf.Sign(d.z));
            return Mathf.Max(q.x, Mathf.Max(q.y, q.z));
        }
    }

    /// <summary>A part of the kine and what it collides with.</summary>
    public sealed class Part
    {
        public Revolved shape;
        public bool usu, table, hands;
    }

    // ---------------- configuration (set, then Build) ----------------
    public Revolved usu;    // station frame, may be null
    public Box table;       // station frame, may be null
    public readonly List<Part> parts = new List<Part>();
    /// <summary>Kine-local points whose motion bounds how far a step moves the kine (head ends and rim, handle end).</summary>
    public readonly List<Vector3> ends = new List<Vector3>();
    /// <summary>Shortest step near something (m): the thinnest part's radius, so nothing can pass through it.</summary>
    public float minStep = 0.016f;
    /// <summary>A target that moves farther than this in one frame (m) was put somewhere (picked up, put down,
    /// KineDriver.Rest), not swung: it is placed there and pushed out, not marched.</summary>
    public float teleport = 0.3f;
    public int maxSettle = 12, maxMarch = 32;
    /// <summary>Distance evaluations since the counters were zeroed (the test reports them).</summary>
    public int sceneEvals, kineEvals;

    /// <summary>The deepest contact of the last frame that touched: what was hit, where, and its outward normal.</summary>
    public Kind lastKind;
    public Vector3 lastPoint, lastNormal;

    sealed class Patch { public int part; public Vector2 a, b; public Vector3 center; public float radius; public bool warm; public float ws, wt; }
    readonly List<Patch> patches = new List<Patch>();
    readonly List<Vector2> corners = new List<Vector2>();
    readonly List<float> cornerWarm = new List<float>();
    const float PatchLength = 0.04f; // m: long edges (the handle, the head's side) are split into patches this long
    bool partsHitHands;

    /// <summary>Precomputes the surface patches of the kine and the usu's convex corners. Call after setting the shapes.</summary>
    public void Build()
    {
        patches.Clear();
        partsHitHands = false;
        for (int pi = 0; pi < parts.Count; pi++)
        {
            var s = parts[pi].shape;
            partsHitHands |= parts[pi].hands;
            int n = s.poly.Length;
            for (int i = 0; i < n; i++)
            {
                if (!s.surface[i]) continue;
                Vector2 a = s.poly[i], b = s.poly[(i + 1) % n];
                int pieces = Mathf.Max(1, Mathf.CeilToInt((b - a).magnitude / PatchLength));
                for (int k = 0; k < pieces; k++)
                {
                    var p = new Patch { part = pi, a = Vector2.Lerp(a, b, k / (float)pieces), b = Vector2.Lerp(a, b, (k + 1) / (float)pieces) };
                    // bounding sphere on the axis at mid-height: the farthest points of a revolved segment are its end circles
                    float hm = 0.5f * (p.a.y + p.b.y);
                    p.center = s.origin + s.axis * hm;
                    p.radius = Mathf.Max(Mathf.Sqrt(p.a.x * p.a.x + (p.a.y - hm) * (p.a.y - hm)),
                                         Mathf.Sqrt(p.b.x * p.b.x + (p.b.y - hm) * (p.b.y - hm))) + 1e-4f;
                    patches.Add(p);
                }
            }
        }
        corners.Clear();
        cornerWarm.Clear();
        if (usu != null)
        {
            var poly = usu.poly;
            int n = poly.Length;
            for (int i = 0; i < n; i++)
            {
                int ip = (i + n - 1) % n;
                Vector2 a = poly[ip], b = poly[i], c = poly[(i + 1) % n];
                if (b.x <= 1e-4f || !usu.surface[ip] || !usu.surface[i]) continue;
                float cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
                if (cross > 1e-9f) { corners.Add(b); cornerWarm.Add(float.NaN); }
            }
        }
        Reset();
    }

    // ---------------- distances ----------------

    /// <summary>Signed distance from a station point to the usu and/or the table, with the outward normal and what is nearest.</summary>
    public float SceneSdf(Vector3 p, bool useUsu, bool useTable, out Vector3 n, out Kind kind)
    {
        sceneEvals++;
        float sd = float.MaxValue;
        n = Vector3.up;
        kind = Kind.Usu;
        if (useUsu && usu != null) sd = usu.Sdf(p, out n);
        if (useTable && table != null)
        {
            float t = table.Sdf(p, out Vector3 tn);
            if (t < sd) { sd = t; n = tn; kind = Kind.Table; }
        }
        return sd;
    }

    /// <summary>Signed distance from a station point to the kine posed at (pos, rot), counting the parts that hit
    /// the usu (mask 0), the table (1) or hands (2); n = the kine's outward normal there.</summary>
    public float KineSdf(Vector3 p, Vector3 pos, Quaternion rot, int mask, out Vector3 n)
    {
        kineEvals++;
        Vector3 local = Quaternion.Inverse(rot) * (p - pos);
        float best = float.MaxValue;
        Vector3 bn = Vector3.up;
        for (int i = 0; i < parts.Count; i++)
        {
            var pt = parts[i];
            if (!(mask == 0 ? pt.usu : mask == 1 ? pt.table : pt.hands)) continue;
            float d = pt.shape.Sdf(local, out Vector3 nl);
            if (d < best) { best = d; bn = nl; }
        }
        n = rot * bn;
        return best;
    }

    // ---------------- the deepest overlaps of the kine posed at (pos, rot) ----------------

    struct Contact { public float pen, r; public int type, part; public Kind kind; public Vector3 body, point, n; } // type 0: kine point in the scene; 1: usu corner in the kine; 2: hand ball in the kine
    readonly Contact[] pool = new Contact[96];
    int nContacts;
    Contact deepest;
    bool hasDeepest;

    void Record(Contact c)
    {
        if (nContacts < pool.Length) pool[nContacts++] = c;
        if (!hasDeepest || c.pen > deepest.pen) { deepest = c; hasDeepest = true; }
    }

    float PatchPen(Patch pa, float s, float th, Vector3 pos, Quaternion rot)
    {
        var part = parts[pa.part];
        Vector2 q = Vector2.LerpUnclamped(pa.a, pa.b, s);
        return -SceneSdf(pos + rot * part.shape.Point(q.x, q.y, th), part.usu, part.table, out _, out _);
    }

    // local ascent of the overlap over a patch (s along the edge, th around the axis): pattern search, halving steps
    void Refine(Patch pa, ref float s, ref float th, ref float f, Vector3 pos, Quaternion rot, float ds, float dt)
    {
        for (int it = 0; it < 30 && (ds > 1e-4f || dt > 1e-4f); it++)
        {
            bool moved = false;
            for (int k = 0; k < 4 && !moved; k++)
            {
                float ns = s, nt = th;
                if (k == 0) ns = Mathf.Min(1f, s + ds);
                else if (k == 1) ns = Mathf.Max(0f, s - ds);
                else if (k == 2) nt = th + dt;
                else nt = th - dt;
                if (ns == s && nt == th) continue;
                float nf = PatchPen(pa, ns, nt, pos, rot);
                if (nf > f) { s = ns; th = nt; f = nf; moved = true; }
            }
            if (!moved) { ds *= 0.5f; dt *= 0.5f; }
        }
    }

    float CornerPen(Vector2 cr, float phi, Vector3 pos, Quaternion rot) => -KineSdf(usu.Point(cr.x, cr.y, phi), pos, rot, 0, out _);

    bool FindContacts(Vector3 pos, Quaternion rot, List<Vector4> hands, bool scan)
    {
        nContacts = 0;
        hasDeepest = false;
        // (A) the kine's surface inside the usu or the table
        for (int i = 0; i < patches.Count; i++)
        {
            var pa = patches[i];
            var part = parts[pa.part];
            if (!part.usu && !part.table) continue;
            if (SceneSdf(pos + rot * pa.center, part.usu, part.table, out _, out _) > pa.radius) { pa.warm = false; continue; } // all of it clear
            float bs = 0f, bt = 0f, bf = float.NegativeInfinity;
            if (pa.warm)
            {
                bs = pa.ws; bt = pa.wt; bf = PatchPen(pa, bs, bt, pos, rot); // where it was deepest last time
                Refine(pa, ref bs, ref bt, ref bf, pos, rot, 0.05f, Mathf.PI / 32f);
            }
            if (scan || !pa.warm)
            {
                // a coarse scan, so a new contact region is not missed
                const int NA = 16, NB = 2;
                float ss = 0f, st = 0f, sf = float.NegativeInfinity;
                for (int a = 0; a < NA; a++)
                    for (int b = 0; b <= NB; b++)
                    {
                        float s = b / (float)NB, th = a * 2f * Mathf.PI / NA, f = PatchPen(pa, s, th, pos, rot);
                        if (f > sf) { sf = f; ss = s; st = th; }
                    }
                if (sf > bf)
                {
                    Refine(pa, ref ss, ref st, ref sf, pos, rot, 0.25f, Mathf.PI / NA);
                    if (sf > bf) { bf = sf; bs = ss; bt = st; }
                }
            }
            pa.warm = bf > -pa.radius;
            pa.ws = bs;
            pa.wt = bt;
            if (bf > Eps)
            {
                Vector2 q = Vector2.LerpUnclamped(pa.a, pa.b, bs);
                Vector3 body = part.shape.Point(q.x, q.y, bt), w = pos + rot * body;
                SceneSdf(w, part.usu, part.table, out Vector3 n, out Kind kind);
                Record(new Contact { pen = bf, type = 0, part = pa.part, kind = kind, body = body, point = w, n = n });
            }
        }
        // (B) the usu's convex corners (circles) inside the kine
        if (usu != null)
            for (int ci = 0; ci < corners.Count; ci++)
            {
                Vector2 cr = corners[ci];
                bool near = false;
                for (int i = 0; i < patches.Count && !near; i++)
                {
                    var pa = patches[i];
                    if (!parts[pa.part].usu) continue;
                    Vector3 pc = pos + rot * pa.center - usu.origin;
                    float h = Vector3.Dot(pc, usu.axis), rho = (pc - usu.axis * h).magnitude, dr = rho - cr.x, dh = h - cr.y;
                    near = dr * dr + dh * dh < pa.radius * pa.radius;
                }
                if (!near) { cornerWarm[ci] = float.NaN; continue; }
                bool warm = !float.IsNaN(cornerWarm[ci]);
                float bp = warm ? cornerWarm[ci] : 0f, bg = warm ? CornerPen(cr, bp, pos, rot) : float.NegativeInfinity;
                if (scan || !warm)
                    for (int k = 0; k < 48; k++)
                    {
                        float phi = k * 2f * Mathf.PI / 48f, g = CornerPen(cr, phi, pos, rot);
                        if (g > bg) { bg = g; bp = phi; }
                    }
                float step = Mathf.PI / 48f;
                for (int k = 0; k < 60 && step > 1e-5f; k++)
                {
                    float g1 = CornerPen(cr, bp + step, pos, rot), g0 = CornerPen(cr, bp - step, pos, rot);
                    if (g1 > bg) { bg = g1; bp += step; }
                    else if (g0 > bg) { bg = g0; bp -= step; }
                    else step *= 0.5f;
                }
                cornerWarm[ci] = bp;
                if (bg > Eps)
                {
                    Vector3 w = usu.Point(cr.x, cr.y, bp);
                    KineSdf(w, pos, rot, 0, out Vector3 kn);
                    Record(new Contact { pen = bg, type = 1, kind = Kind.Usu, point = w, n = -kn }); // move the kine off the corner
                }
            }
        // (C) hand balls inside the kine
        if (hands != null && partsHitHands)
            for (int i = 0; i < hands.Count; i++)
            {
                Vector4 h = hands[i];
                Vector3 c = new Vector3(h.x, h.y, h.z);
                float pen = h.w - KineSdf(c, pos, rot, 2, out Vector3 kn);
                if (pen > Eps) Record(new Contact { pen = pen, type = 2, kind = Kind.Hand, point = c, r = h.w, n = -kn });
            }
        return hasDeepest;
    }

    // ---------------- resolving ----------------

    float ContactDist(int i, Vector3 mp, Quaternion mq)
    {
        var c = pool[i];
        if (c.type == 0)
        {
            var part = parts[c.part];
            return SceneSdf(mp + mq * c.body, part.usu, part.table, out _, out _);
        }
        if (c.type == 1) return KineSdf(c.point, mp, mq, 0, out _);
        return KineSdf(c.point, mp, mq, 2, out _) - c.r;
    }

    static void Moved(bool rotate, float s, Vector3 pos, Quaternion rot, Vector3 pivot, Vector3 ax, Vector3 dir, out Vector3 mp, out Quaternion mq)
    {
        if (rotate)
        {
            Quaternion q = Quaternion.AngleAxis(s * Mathf.Rad2Deg, ax);
            mq = q * rot;
            mp = pivot + q * (pos - pivot);
        }
        else
        {
            mq = rot;
            mp = pos + dir * s;
        }
    }

    bool AllClear(bool rotate, float s, Vector3 pos, Quaternion rot, Vector3 pivot, Vector3 ax, Vector3 dir)
    {
        Moved(rotate, s, pos, rot, pivot, ax, dir, out Vector3 mp, out Quaternion mq);
        for (int i = 0; i < nContacts; i++) if (ContactDist(i, mp, mq) < 0f) return false;
        return true;
    }

    // Until nothing overlaps: every current contact is cleared together by one move, their pushes combined (each
    // normal weighted by its depth) and applied at their depth-weighted centre, bisected to the smallest move that
    // clears them all, so the kine ends touching. stuck: no move within reach clears them (wedged).
    bool Settle(ref Vector3 pos, ref Quaternion rot, Vector3 pivotLocal, List<Vector4> hands, out bool stuck)
    {
        bool touched = false;
        stuck = false;
        for (int it = 0; it < maxSettle; it++)
        {
            if (!FindContacts(pos, rot, hands, it == 0)) break;
            touched = true;
            lastKind = deepest.kind;
            lastPoint = deepest.point;
            lastNormal = deepest.n;
            Vector3 sumN = Vector3.zero, sumP = Vector3.zero;
            float w = 0f;
            for (int i = 0; i < nContacts; i++) { sumN += pool[i].n * pool[i].pen; sumP += pool[i].point * pool[i].pen; w += pool[i].pen; }
            if (sumN.sqrMagnitude < 1e-20f) sumN = deepest.n; // opposed pushes cancel: fall back on the deepest one
            Vector3 dir = sumN.normalized;
            sumP /= w;
            Vector3 pivot = pos + rot * pivotLocal;
            Vector3 rv = sumP - pivot, ax = Vector3.Cross(rv, dir);
            float lever = rv.magnitude, sin = lever > 1e-9f ? ax.magnitude / lever : 0f;
            bool rotate = sin >= 0.2f; // pushing along the handle itself: pivoting can't help, slide instead
            if (sin > 1e-6f) ax /= ax.magnitude;
            // A contact is cleared by a move about as large as its depth. A much larger one is not a push out but a
            // jump (the head swung over or around what it hit), which is what made the kine teleport between
            // surfaces: then the other kind of move is tried, and if neither clears it locally the kine is wedged.
            float reach = Mathf.Max(4f * deepest.pen, minStep);
            if (!Clearing(rotate, lever, sin, reach, pos, rot, pivot, ax, dir, out float s))
            {
                rotate = !rotate;
                if ((rotate && sin < 0.05f) || !Clearing(rotate, lever, sin, reach, pos, rot, pivot, ax, dir, out s)) { stuck = true; return touched; }
            }
            Moved(rotate, s, pos, rot, pivot, ax, dir, out Vector3 np, out Quaternion nq);
            pos = np;
            rot = nq.normalized;
        }
        return touched;
    }

    // The smallest move of one kind (a pivot about `pivot`, or a push along `dir`) that clears every current contact,
    // with the contact point moving no more than `reach`: bracketed, then bisected.
    bool Clearing(bool rotate, float lever, float sin, float reach, Vector3 pos, Quaternion rot, Vector3 pivot, Vector3 ax, Vector3 dir, out float s)
    {
        float hi = rotate ? deepest.pen / (lever * sin) : deepest.pen;
        float cap = rotate ? reach / Mathf.Max(lever, 1e-6f) : reach; // a pivot moves the contact point by angle x lever
        s = 0f;
        if (hi > cap) hi = cap;
        bool ok = false;
        for (int k = 0; k < 30; k++)
        {
            if (AllClear(rotate, hi, pos, rot, pivot, ax, dir)) { ok = true; break; }
            if (hi >= cap) break;
            hi = Mathf.Min(hi * 2f, cap);
        }
        if (!ok) return false;
        float lo = 0f;
        for (int k = 0; k < 40 && hi - lo > 1e-7f; k++)
        {
            float mid = 0.5f * (lo + hi);
            if (AllClear(rotate, mid, pos, rot, pivot, ax, dir)) hi = mid;
            else lo = mid;
        }
        s = hi;
        return true;
    }

    float FarthestMove(Vector3 pos, Quaternion rot, Vector3 tPos, Quaternion tRot)
    {
        float d = (pos - tPos).magnitude;
        for (int i = 0; i < ends.Count; i++) d = Mathf.Max(d, (pos + rot * ends[i] - (tPos + tRot * ends[i])).magnitude);
        return d;
    }

    // the free space the kine surely has (0 if it might be touching): how far it can move in one step
    float Clearance(Vector3 pos, Quaternion rot, List<Vector4> hands)
    {
        float c = float.MaxValue;
        for (int i = 0; i < patches.Count; i++)
        {
            var pa = patches[i];
            var part = parts[pa.part];
            if (!part.usu && !part.table) continue;
            c = Mathf.Min(c, SceneSdf(pos + rot * pa.center, part.usu, part.table, out _, out _) - pa.radius);
        }
        if (hands != null && partsHitHands)
            for (int i = 0; i < hands.Count; i++)
            {
                Vector4 h = hands[i];
                c = Mathf.Min(c, KineSdf(new Vector3(h.x, h.y, h.z), pos, rot, 2, out _) - h.w);
            }
        return Mathf.Max(0f, c);
    }

    Vector3 prevPos, prevTarget;
    Quaternion prevRot = Quaternion.identity;
    bool hasPrev;

    /// <summary>Forget the last pose (the next frame is placed, not marched) and the remembered deepest points.</summary>
    public void Reset()
    {
        hasPrev = false;
        foreach (var p in patches) p.warm = false;
        for (int i = 0; i < cornerWarm.Count; i++) cornerWarm[i] = float.NaN;
    }

    /// <summary>
    /// Resolves this frame's target pose (station-local) against the usu, the table and the hand balls
    /// (station-local centre, radius). pivotLocal: the kine-local point it pivots about (where it is held).
    /// Returns true when the kine touches something or is held back; (pos, rot) is where it ends up.
    /// </summary>
    public bool Resolve(Vector3 tPos, Quaternion tRot, Vector3 pivotLocal, List<Vector4> hands, out Vector3 pos, out Quaternion rot)
    {
        bool touched = false;
        float moved = hasPrev ? (tPos - prevTarget).magnitude : 0f;
        if (!hasPrev || moved > teleport)
        {
            // placed (picked up, put down, a mode change): no sweep
            pos = tPos;
            rot = tRot;
            touched = Settle(ref pos, ref rot, pivotLocal, hands, out _);
        }
        else
        {
            // march toward this frame's pose by the free space around the kine, settling after each step; if something
            // blocks the way, the kine stays on the near side
            pos = prevPos;
            rot = prevRot;
            for (int k = 0; k < maxMarch; k++)
            {
                float left = FarthestMove(pos, rot, tPos, tRot);
                if (left < 1e-6f) break;
                float t = Mathf.Min(1f, Mathf.Max(minStep, Clearance(pos, rot, hands)) / left);
                Vector3 keepPos = pos;
                Quaternion keepRot = rot;
                pos = Vector3.Lerp(pos, tPos, t);
                rot = Quaternion.Slerp(rot, tRot, t);
                bool blocked = Settle(ref pos, ref rot, pivotLocal, hands, out bool stuck);
                if (stuck) { pos = keepPos; rot = keepRot; touched = true; break; } // wedged: stay where it was
                if (blocked) touched = true;
                // held back: a step that had to be corrected and kept less than half its advance can't be followed by
                // better ones (the kine only creeps along what blocks it)
                if (blocked && left - FarthestMove(pos, rot, tPos, tRot) < 0.5f * t * left) break;
            }
            if ((pos - tPos).sqrMagnitude > 1e-12f || Quaternion.Angle(rot, tRot) > 1e-3f) touched = true;
        }
        prevPos = pos;
        prevRot = rot;
        prevTarget = tPos;
        hasPrev = true;
        return touched;
    }

    // ---------------- the drawn shapes ----------------

    static float Facet(float r, int sides) => r / Mathf.Cos(Mathf.PI / sides); // the skins put vertices here, so facet middles lie on r

    static Revolved Make(List<Vector2> pts, Vector3 origin, Vector3 axis, params int[] notSurface)
    {
        var surface = new bool[pts.Count];
        for (int i = 0; i < surface.Length; i++) surface[i] = true;
        foreach (int i in notSurface) surface[(i + pts.Count) % pts.Count] = false;
        return new Revolved(pts.ToArray(), surface, origin, axis);
    }

    /// <summary>The usu as UsuKineSkinBuilder.BuildUsu draws it (MR_UsuSkin: foot, waist, pink band, rounded rim,
    /// end-grain rim top, inner wall 8 mm outside the solver's cone, floor 6 mm under it), from UsuTool's sizes
    /// (they match the skin's settings), closed downward into what it stands on.</summary>
    public static Revolved UsuSkin(float outerRadius, float baseY, float depth, float innerTop, float innerBottom, Vector3 origin, Vector3 axis)
    {
        const int N = 32;
        const float WallGap = 0.008f, FloorGap = 0.006f;
        float top = baseY + depth, Ro = Facet(outerRadius, N);
        float rt = Facet(innerTop + WallGap, N), rb = Facet(innerBottom + WallGap, N), rimIn = rt + 0.006f;
        float bandY = top * 0.48f, bandR = Ro * 0.87f + 0.006f, b0 = bandY - 0.011f, b1 = bandY + 0.011f;
        Vector2 w0 = new Vector2(Ro * 0.93f, top * 0.22f), waist = new Vector2(Ro * 0.87f, top * 0.48f), w1 = new Vector2(Ro * 0.93f, top * 0.78f);
        Vector2 OnWall(Vector2 a, Vector2 b, float y) => Vector2.Lerp(a, b, (y - a.y) / (b.y - a.y));
        var pts = new List<Vector2>
        {
            new Vector2(0f, -1f), new Vector2(Ro * 0.90f, -1f),                            // buried: what the usu stands on
            new Vector2(Ro * 0.90f, 0f), new Vector2(Ro * 0.95f, 0.008f), w0,                // foot, lower wall
            OnWall(w0, waist, b0), new Vector2(bandR, b0), new Vector2(bandR, b1), OnWall(waist, w1, b1), // the band round the waist
            w1, new Vector2(Ro, top - 0.012f), new Vector2(Ro * 0.985f, top),               // upper wall, rounded rim
            new Vector2(rimIn, top), new Vector2(rt, top - 0.004f),                          // rim top, inner chamfer
            new Vector2(Mathf.Lerp(rb, rt, 0.5f), baseY + depth * 0.5f), new Vector2(rb, baseY - FloorGap), new Vector2(0f, baseY - FloorGap), // inner wall, floor
        };
        return Make(pts, origin, axis, 0, 1, -1); // buried bottom and side; the axis
    }

    /// <summary>The usu as UsuTool.BuildLatheMesh draws it (no skin), closed downward into what it stands on.</summary>
    public static Revolved UsuLathe(float ro, float baseY, float depth, float rTop, float rBot, Vector3 origin, Vector3 axis)
    {
        float rimY = baseY + depth + 0.02f;
        var pts = new List<Vector2>
        {
            new Vector2(0f, -1f), new Vector2(ro * 0.96f, -1f), new Vector2(ro * 0.96f, 0f), new Vector2(ro, 0.015f),
            new Vector2(ro, rimY - 0.01f), new Vector2(ro - 0.01f, rimY), new Vector2(rTop + 0.01f, rimY), new Vector2(rTop, rimY - 0.01f),
            new Vector2(Mathf.Lerp(rBot, rTop, 0.5f), baseY + depth * 0.5f), new Vector2(rBot, baseY + 0.015f),
            new Vector2(rBot - 0.02f, baseY), new Vector2(0f, baseY),
        };
        return Make(pts, origin, axis, 0, 1, -1);
    }

    /// <summary>The kine head as UsuKineSkinBuilder.BuildKine draws it (MR_KineSkin): flat face, chamfered edge,
    /// straight body with two pink bands, chamfered top. Kine-local, around +Y from the face.</summary>
    public static Revolved KineSkinHead(float radius, float length, float rounding)
    {
        float r = Facet(radius, 16), L = length, f = rounding, y1 = f + 0.012f, y2 = L - f - 0.012f, rb = r + 0.003f;
        var pts = new List<Vector2>
        {
            new Vector2(0f, 0f), new Vector2(r - f, 0f), new Vector2(r - 0.3f * f, 0.35f * f), new Vector2(r, f),
            new Vector2(r, y1 - 0.007f), new Vector2(rb, y1 - 0.007f), new Vector2(rb, y1 + 0.007f), new Vector2(r, y1 + 0.007f),
            new Vector2(r, y2 - 0.007f), new Vector2(rb, y2 - 0.007f), new Vector2(rb, y2 + 0.007f), new Vector2(r, y2 + 0.007f),
            new Vector2(r, L - f), new Vector2(r - f, L), new Vector2(0f, L),
        };
        return Make(pts, Vector3.zero, Vector3.up, -1);
    }

    /// <summary>The kine handle as UsuKineSkinBuilder.BuildKine draws it: tapered from inside the head's middle back
    /// along -Z to handleLength, with the pink grip wrap over its last 12 cm and the knob at the end.</summary>
    public static Revolved KineSkinHandle(float radius, float length, float handleLength)
    {
        const float hw = 0.016f;
        float r = Facet(radius, 16), t0 = r * 0.5f, H = handleLength, g0 = H - 0.13f, g1 = H - 0.01f;
        float Taper(float t) => Mathf.Lerp(hw * 1.15f, hw, (t - t0) / (H - t0));
        var pts = new List<Vector2>
        {
            new Vector2(0f, t0), new Vector2(hw * 1.15f, t0),
            new Vector2(Taper(g0), g0), new Vector2(hw * 1.25f, g0), new Vector2(hw * 1.2f, g1), new Vector2(Taper(g1), g1), // grip wrap
            new Vector2(hw, H), new Vector2(hw * 1.35f, H), new Vector2(hw * 0.7f, H + 0.025f), new Vector2(0f, H + 0.025f), // knob
        };
        return Make(pts, new Vector3(0f, length * 0.5f, 0f), Vector3.back, 0, -1); // its start is inside the head; the axis
    }

    /// <summary>The handle's shaft radius in the skin (the thinnest part of the kine).</summary>
    public const float KineSkinHandleRadius = 0.016f;

    /// <summary>The KineTool head without a skin: a cylinder with a rounded face and top edge.</summary>
    public static Revolved CylinderHead(float radius, float length, float rounding)
    {
        float R = radius, L = length, f = rounding;
        var pts = new List<Vector2> { new Vector2(0f, 0f), new Vector2(R - f, 0f), new Vector2(R, f), new Vector2(R, L - f), new Vector2(R - f, L), new Vector2(0f, L) };
        return Make(pts, Vector3.zero, Vector3.up, -1);
    }

    /// <summary>A plain cylindrical handle from a to b (kine-local), starting `start` along it (inside the head).</summary>
    public static Revolved CylinderHandle(Vector3 a, Vector3 b, float radius, float start)
    {
        float len = (b - a).magnitude;
        var pts = new List<Vector2> { new Vector2(0f, start), new Vector2(radius, start), new Vector2(radius, len), new Vector2(0f, len) };
        return Make(pts, a, b - a, 0, -1);
    }
}
