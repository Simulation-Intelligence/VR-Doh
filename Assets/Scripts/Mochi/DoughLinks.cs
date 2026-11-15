using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// One body: keeps the native solver's dough from coming apart into blobs. Ported 2026-11-15 from the
/// three.js scene's relink (mochi-pounding/src/dough/mpm-core.js), after the user asked that the dough
/// never separate from itself, only stretch and fold over itself.
///
/// MPM particles hold together only through the grid nodes they share, so dough stretched thinner than
/// about a cell comes apart for good. Particles closer than the link reach (MochiSolver.linkReach times
/// the particle lattice spacing) are linked. Every few frames the positions are read back and, on a
/// worker thread:
///  - pairs that came within the reach are linked, so a fold links up with the dough it lies on;
///  - a stretched link (longer than the reach) is dropped when its two particles are still joined through
///    short links (the same connected component of the short-link graph, by union-find), so the dough can
///    still flow and rearrange; otherwise it is kept, because it is what holds two parts together;
///  - short links that are bridges of the short-link graph (removing one would split the dough, as along
///    a thin strand; Tarjan's algorithm) are marked too, before they stretch, so a tear that happens
///    between two rebuilds can't slip through.
/// The kept stretched links and the bridges are the tethers. In every substep the Particles kernel holds
/// each tether at the reach, and its ends can't move apart along it: a taut strand passes the pull on to
/// the dough behind it, which flows into the strand. So the dough stretches, and never tears.
/// </summary>
public sealed class DoughLinks : IDisposable
{
    public int TetherCount { get; private set; }
    public int LinkCount { get; private set; }
    public float Reach => reach;

    readonly int count;
    readonly float reach;
    readonly int everyFrames;

    // worker state: only Rebuild touches these, and one rebuild runs at a time
    readonly List<int> la = new List<int>(), lb = new List<int>();
    readonly HashSet<long> linkSet = new HashSet<long>();
    float[] pos;                       // positions from the last readback, 3 per particle (NaN = skip)
    int[] resultStart, resultOther;    // tethers per particle, both directions (CSR)
    int resultOtherCount, resultTethers, resultLinks;
    volatile bool resultReady, busy;
    volatile string workerError;
    bool requested, disposed;
    int frame;

    ComputeBuffer posBuf;      // float4 per particle, copied from the particles for the readback
    ComputeBuffer rangeBuf;    // uint2 per particle: first tether in otherBuf, tether count
    ComputeBuffer otherBuf;    // uint: the particle at the far end of each tether
    readonly uint[] rangeData;

    public DoughLinks(int particleCount, float reach, int everyFrames)
    {
        count = particleCount;
        this.reach = reach;
        this.everyFrames = Mathf.Max(1, everyFrames);
        int n = Mathf.Max(count, 1);
        rangeData = new uint[n * 2];
        posBuf = new ComputeBuffer(n, 16);
        rangeBuf = new ComputeBuffer(n, 8);
        rangeBuf.SetData(rangeData);
        otherBuf = new ComputeBuffer(1024, 4);
    }

    /// <summary>
    /// Once per frame, after the substeps: uploads a finished rebuild, and every few frames copies the
    /// positions out and reads them back (asynchronously); the rebuild then runs on a worker thread.
    /// </summary>
    public void Tick(ComputeShader cs, int copyKernel, ComputeBuffer parts)
    {
        if (disposed) return;
        if (workerError != null) { Debug.LogWarning("DoughLinks: rebuild failed: " + workerError); workerError = null; }
        if (resultReady) Upload();
        if (busy || requested || parts == null || count == 0) return;
        if (++frame % everyFrames != 0) return;

        cs.SetBuffer(copyKernel, "Parts", parts);
        cs.SetBuffer(copyKernel, "PosOut", posBuf);
        cs.Dispatch(copyKernel, (count + 63) / 64, 1, 1);
        requested = true;
        AsyncGPUReadback.Request(posBuf, req =>
        {
            requested = false;
            if (disposed || req.hasError) return;
            NativeArray<float> d = req.GetData<float>();
            if (d.Length < count * 4) return;
            pos ??= new float[count * 3];
            for (int i = 0; i < count; i++)
            {
                pos[3 * i] = d[4 * i]; pos[3 * i + 1] = d[4 * i + 1]; pos[3 * i + 2] = d[4 * i + 2];
            }
            busy = true;
            Task.Run(() =>
            {
                try { Rebuild(); resultReady = true; }
                catch (Exception e) { workerError = e.Message; busy = false; }
            });
        });
    }

    /// <summary>Binds the tethers to a kernel (the Particles kernel enforces them).</summary>
    public void Bind(ComputeShader cs, int kernel, bool on)
    {
        cs.SetBuffer(kernel, "LinkRange", rangeBuf);
        cs.SetBuffer(kernel, "LinkOther", otherBuf);
        cs.SetFloat("LinkReach", reach);
        cs.SetInt("LinksOn", on && TetherCount > 0 && !disposed ? 1 : 0);
    }

    void Upload()
    {
        resultReady = false;
        if (otherBuf.count < Mathf.Max(resultOtherCount, 1))
        {
            otherBuf.Release();
            otherBuf = new ComputeBuffer(Mathf.NextPowerOfTwo(Mathf.Max(resultOtherCount, 1)), 4);
        }
        for (int i = 0; i < count; i++)
        {
            rangeData[2 * i] = (uint)resultStart[i];
            rangeData[2 * i + 1] = (uint)(resultStart[i + 1] - resultStart[i]);
        }
        rangeBuf.SetData(rangeData);
        if (resultOtherCount > 0) otherBuf.SetData(resultOther, 0, 0, resultOtherCount);
        TetherCount = resultTethers;
        LinkCount = resultLinks;
        busy = false;
    }

    // ---------------------------------------------------------------------------------------------
    // Worker thread
    // ---------------------------------------------------------------------------------------------

    int[] cellStart, cellFill, cellIdx, cellOf;
    int[] deg, adjNode, adjEdge, uf, disc, low, stNode, stEdge, stIt, tCount;
    bool[] isLong, isBridge, keep;

    static void Grow(ref int[] a, int n) { if (a == null || a.Length < n) a = new int[Mathf.NextPowerOfTwo(Mathf.Max(n, 16))]; }
    static void Grow(ref bool[] a, int n) { if (a == null || a.Length < n) a = new bool[Mathf.NextPowerOfTwo(Mathf.Max(n, 16))]; }

    int Find(int i)
    {
        while (uf[i] != i) { uf[i] = uf[uf[i]]; i = uf[i]; }
        return i;
    }

    void Rebuild()
    {
        int N = count;
        float R = reach, R2 = R * R;
        float[] x = pos;

        // ---- particles into cells of size R (counting sort over the particles' bounding box) ----
        float mnx = float.MaxValue, mny = float.MaxValue, mnz = float.MaxValue, mxx = float.MinValue, mxy = float.MinValue, mxz = float.MinValue;
        for (int p = 0; p < N; p++)
        {
            float px = x[3 * p], py = x[3 * p + 1], pz = x[3 * p + 2];
            if (float.IsNaN(px) || float.IsNaN(py) || float.IsNaN(pz)) continue;
            if (px < mnx) mnx = px; if (py < mny) mny = py; if (pz < mnz) mnz = pz;
            if (px > mxx) mxx = px; if (py > mxy) mxy = py; if (pz > mxz) mxz = pz;
        }
        if (mnx > mxx) return;   // nothing valid
        int kx = Mathf.Clamp((int)((mxx - mnx) / R) + 1, 1, 256), ky = Mathf.Clamp((int)((mxy - mny) / R) + 1, 1, 256), kz = Mathf.Clamp((int)((mxz - mnz) / R) + 1, 1, 256);
        int cells = kx * ky * kz;
        Grow(ref cellStart, cells + 1); Grow(ref cellFill, cells); Grow(ref cellIdx, N); Grow(ref cellOf, N);
        Array.Clear(cellStart, 0, cells + 1);
        for (int p = 0; p < N; p++)
        {
            float px = x[3 * p], py = x[3 * p + 1], pz = x[3 * p + 2];
            if (float.IsNaN(px) || float.IsNaN(py) || float.IsNaN(pz)) { cellOf[p] = -1; continue; }
            int cx = Mathf.Min((int)((px - mnx) / R), kx - 1), cy = Mathf.Min((int)((py - mny) / R), ky - 1), cz = Mathf.Min((int)((pz - mnz) / R), kz - 1);
            int c = cx + kx * (cy + ky * cz);
            cellOf[p] = c;
            cellStart[c + 1]++;
        }
        for (int c = 0; c < cells; c++) cellStart[c + 1] += cellStart[c];
        Array.Copy(cellStart, cellFill, cells);
        for (int p = 0; p < N; p++) if (cellOf[p] >= 0) cellIdx[cellFill[cellOf[p]]++] = p;

        // ---- new links: pairs within the reach that are not linked yet ----
        for (int p = 0; p < N; p++)
        {
            int c = cellOf[p];
            if (c < 0) continue;
            int cx = c % kx, cy = (c / kx) % ky, cz = c / (kx * ky);
            for (int dz = -1; dz <= 1; dz++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int qx = cx + dx, qy = cy + dy, qz = cz + dz;
                        if (qx < 0 || qy < 0 || qz < 0 || qx >= kx || qy >= ky || qz >= kz) continue;
                        int qc = qx + kx * (qy + ky * qz);
                        for (int i = cellStart[qc]; i < cellStart[qc + 1]; i++)
                        {
                            int q = cellIdx[i];
                            if (q <= p) continue;
                            float ex = x[3 * q] - x[3 * p], ey = x[3 * q + 1] - x[3 * p + 1], ez = x[3 * q + 2] - x[3 * p + 2];
                            if (ex * ex + ey * ey + ez * ez >= R2) continue;
                            if (linkSet.Add((long)p * N + q)) { la.Add(p); lb.Add(q); }
                        }
                    }
        }

        // ---- the short-link graph: components (union-find) and adjacency with link ids (CSR) ----
        int L = la.Count;
        Grow(ref isLong, L); Grow(ref isBridge, L); Grow(ref keep, L);
        Grow(ref uf, N); Grow(ref deg, N + 1);
        for (int p = 0; p < N; p++) uf[p] = p;
        Array.Clear(deg, 0, N + 1);
        for (int k = 0; k < L; k++)
        {
            int a = la[k], b = lb[k];
            float ex = x[3 * b] - x[3 * a], ey = x[3 * b + 1] - x[3 * a + 1], ez = x[3 * b + 2] - x[3 * a + 2];
            float d2 = ex * ex + ey * ey + ez * ez;
            isLong[k] = !(d2 <= R2);   // NaN counts as long
            isBridge[k] = false;
            if (isLong[k]) continue;
            deg[a + 1]++; deg[b + 1]++;
            int ra = Find(a), rb = Find(b);
            if (ra != rb) uf[ra] = rb;
        }
        for (int p = 0; p < N; p++) deg[p + 1] += deg[p];
        int adjLen = deg[N];
        Grow(ref adjNode, adjLen); Grow(ref adjEdge, adjLen); Grow(ref stIt, N);
        for (int p = 0; p < N; p++) stIt[p] = deg[p];   // fill cursors
        for (int k = 0; k < L; k++)
        {
            if (isLong[k]) continue;
            int a = la[k], b = lb[k];
            adjNode[stIt[a]] = b; adjEdge[stIt[a]++] = k;
            adjNode[stIt[b]] = a; adjEdge[stIt[b]++] = k;
        }

        // ---- bridges of the short-link graph (iterative Tarjan) ----
        Grow(ref disc, N); Grow(ref low, N); Grow(ref stNode, N); Grow(ref stEdge, N);
        for (int p = 0; p < N; p++) disc[p] = -1;
        int timer = 0;
        for (int r = 0; r < N; r++)
        {
            if (disc[r] >= 0 || deg[r + 1] == deg[r]) continue;
            int sp = 0;
            stNode[0] = r; stEdge[0] = -1; stIt[0] = deg[r];
            disc[r] = low[r] = timer++;
            while (sp >= 0)
            {
                int u = stNode[sp];
                if (stIt[sp] < deg[u + 1])
                {
                    int it = stIt[sp]++;
                    int e = adjEdge[it], v = adjNode[it];
                    if (e == stEdge[sp]) continue;   // the link we came in by
                    if (disc[v] < 0)
                    {
                        disc[v] = low[v] = timer++;
                        sp++;
                        stNode[sp] = v; stEdge[sp] = e; stIt[sp] = deg[v];
                    }
                    else if (disc[v] < low[u]) low[u] = disc[v];
                }
                else
                {
                    int e = stEdge[sp];
                    sp--;
                    if (sp >= 0)
                    {
                        int par = stNode[sp];
                        if (low[u] < low[par]) low[par] = low[u];
                        if (low[u] > disc[par]) isBridge[e] = true;
                    }
                }
            }
        }

        // ---- keep, drop, and the tethers ----
        Grow(ref tCount, N + 1);
        Array.Clear(tCount, 0, N + 1);
        int tethers = 0, w = 0;
        for (int k = 0; k < L; k++)
        {
            int a = la[k], b = lb[k];
            bool tether;
            if (isLong[k])
            {
                if (cellOf[a] >= 0 && cellOf[b] >= 0 && Find(a) == Find(b)) { linkSet.Remove((long)a * N + b); continue; }   // joined some other way
                tether = true;
            }
            else tether = isBridge[k];
            la[w] = a; lb[w] = b; keep[w] = tether; w++;
            if (tether) { tCount[a + 1]++; tCount[b + 1]++; tethers++; }
        }
        la.RemoveRange(w, la.Count - w);
        lb.RemoveRange(w, lb.Count - w);
        for (int p = 0; p < N; p++) tCount[p + 1] += tCount[p];
        int total = tCount[N];
        if (resultStart == null || resultStart.Length < N + 1) resultStart = new int[N + 1];
        Grow(ref resultOther, total);
        Array.Copy(tCount, resultStart, N + 1);
        Grow(ref stIt, N);
        for (int p = 0; p < N; p++) stIt[p] = tCount[p];
        for (int k = 0; k < w; k++)
        {
            if (!keep[k]) continue;
            int a = la[k], b = lb[k];
            resultOther[stIt[a]++] = b;
            resultOther[stIt[b]++] = a;
        }
        resultOtherCount = total;
        resultTethers = tethers;
        resultLinks = w;
    }

    public void Dispose()
    {
        disposed = true;
        posBuf?.Release(); rangeBuf?.Release(); otherBuf?.Release();
        posBuf = rangeBuf = otherBuf = null;
    }
}
