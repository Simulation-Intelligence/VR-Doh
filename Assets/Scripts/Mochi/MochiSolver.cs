using System.Collections.Generic;
using MarchingCubes;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// The mochi dough as an MLS-MPM simulation in plain compute shaders (Resources/MochiSolver.compute).
/// It replaces Mpm3DMarching and the Taichi runtime for the standalone Quest build: no native
/// plugins, a few thousand particles, and a bond ("tack") between dough and tools that is tuned
/// for how a strike looks rather than for physical accuracy.
///
/// Local space is metres. The domain is the box [0, gridCells * cellSize] and the dough floor is
/// floorCells * cellSize above its bottom; MochiStation places the transform so that floor is the
/// bottom of the usu.
/// </summary>
[DefaultExecutionOrder(100)]
public class MochiSolver : MonoBehaviour, IMochiSim
{
    [Header("Grid")]
    [Tooltip("Edge of a simulation cell, m. Cost grows with the inverse fourth power.")]
    public float cellSize = 0.011f;
    public Vector3Int gridCells = new Vector3Int(32, 26, 32);
    public int floorCells = 3;
    [Range(2, 8)] public int particlesPerCell = 4;

    [Header("Dough (SI units)")]
    public float massKg = 2.2f;
    public float density = 1150f;
    [Tooltip("Shear modulus, Pa. Sets the springback of a quick press and how hard a stretched tent pulls back.")]
    public float shearModulus = 2500f;
    [Tooltip("Pressure wave speed, m/s. Higher keeps volume better under a strike but needs more substeps.")]
    public float waveSpeed = 4f;
    [Tooltip("Below this shear stress (Pa) the dough holds its shape; gravity stress in the lump is about 900 Pa.")]
    public float yieldStress = 80f;
    [Tooltip("Seconds for stress above the yield to relax. Short = runny, long = rubbery.")]
    public float relaxTime = 0.05f;
    [Tooltip("Pull toward the usu axis, 1/s^2. With a low yield stress this is what makes the dough settle back into a dome (height at the centre is about centering * radius^2 / 20 above the edge). 0 = off.")]
    public float centering = 80f;
    [Tooltip("Flow exponent: 1 = no thinning. Below 1 the dough relaxes faster once the stress above yield passes thinningStress, so it stays firm under hands and runs under the kine.")]
    [Range(0.3f, 1f)] public float flowExponent = 1f;
    [Tooltip("Stress above yield where thinning sets in, Pa. Hands stay below it; a strike is far above it.")]
    public float thinningStress = 3000f;
    [Tooltip("Hard cap on shear stress, Pa. Keeps extreme strikes from storing explosive elastic energy.")]
    public float maxShearStress = 20000f;
    [Tooltip("Pressure (Pa per unit of excess density) that keeps particles from packing denser than rest. Too low and the lump shrinks as it is kneaded.")]
    public float crowdStiffness = 20000f;
    [Tooltip("Dough with less than this fraction of rest density around it is a stray bit and is braked by strayDrag (1/s).")]
    [Range(0f, 0.6f)] public float strayDensity = 0.3f;
    public float strayDrag = 40f;
    [Tooltip("Velocity decay, 1/s.")]
    public float damping = 3f;
    public float toolFriction = 0.5f;

    [Header("Tack (seconds a tool keeps hold of dough while pulling away)")]
    [Tooltip("The kine is dry: the dough follows it up into a tent, then lets go.")]
    public float kineTack = 0.15f;
    [Tooltip("Open hands are wet: 0 slides off.")]
    public float handTack = 0f;
    [Tooltip("A closed fist grips the dough for as long as this. 0 = fists only push. Gripping with real hands rips the lump into pieces, so it is off until that is solved.")]
    public float fistTack = 0f;
    [Tooltip("Other tools (test-bench palms).")]
    public float otherTack = 0f;
    [Tooltip("Reach of the bond from a tool surface, in cells.")]
    public float tackBandCells = 1.5f;
    [Tooltip("A bond only ages while its tool moves faster than this, m/s.")]
    public float pullSpeed = 0.2f;
    [Tooltip("How far into the dough a pulling tool reaches, m. About the lump height gives a tent of the whole lump.")]
    public float pullReach = 0.08f;
    [Tooltip("Falloff of the pull across the reach. 1 = even stretch, higher = a thinner neck under the tool.")]
    public float pullShape = 1f;
    [Range(0f, 1f)] public float pullGain = 0.8f;
    [Tooltip("A fist carries the dough within this distance of the hand, m (a handful).")]
    public float gripReach = 0.045f;

    [Tooltip("Tensile strength of the bond between dough and the usu floor, Pa.")]
    public float floorHold = 20000f;

    [Header("Stepping")]
    [Tooltip("Cap on substeps per frame. Past it the dough runs in slow motion instead of dropping frames.")]
    public int maxSubsteps = 12;
    [Range(0.2f, 0.9f)] public float cfl = 0.5f;
    public float timeScale = 1f;
    [Tooltip("Dough never moves faster than this, m/s. Stops bits being shot out by fast or jittery fingers.")]
    public float maxDoughSpeed = 2.5f;
    [Tooltip("Log one line of solver statistics per second (readable with `metavr log`).")]
    public bool logStats = true;
    [Tooltip("Tool speeds above this are treated as tracking glitches, m/s.")]
    public float maxToolSpeed = 8f;

    [Header("One body (DoughLinks)")]
    [Tooltip("Keep the dough in one piece: neighbouring particles are linked, and a link that holds a part of the dough to the rest never stretches past the reach, so the dough stretches and folds but never comes apart into blobs.")]
    public bool keepInOnePiece = true;
    [Tooltip("Link reach, in particle lattice spacings (1.5: the face and edge neighbours of the starting lattice).")]
    public float linkReach = 1.5f;
    [Tooltip("Frames between link rebuilds (each copies the positions out, reads them back and rebuilds on a worker thread).")]
    [Range(1, 12)] public int linkEveryFrames = 3;
    [Tooltip("Thin dough (a stretched strand or sheet) below this fraction of rest density is drawn heavier, so it stays visible.")]
    public float thinDensity = 0.42f;
    [Tooltip("At most this many times heavier.")]
    public float thinBoost = 6f;

    [Header("Tools")]
    public List<MatTool> tools = new();
    [Tooltip("The usu wall is evaluated analytically from this tool's dimensions instead of its capsules.")]
    public UsuTool usu;

    [Header("Rendering")]
    public ComputeShader marchingCubesCompute;
    public Material surfaceMaterial;
    [Tooltip("Render voxels per simulation cell edge.")]
    [Range(1, 3)] public int renderScale = 2;
    public float isoValue = 0.45f;
    [Tooltip("Weight of the new frame in the density field. Lower hides particle shimmer, adds lag.")]
    [Range(0.1f, 1f)] public float temporalBlend = 0.6f;
    public int triangleBudget = 40000;

    /// <summary>The tools that took part in the latest frame (tracked hands, the kine, test tools).</summary>
    public readonly List<MatTool> activeTools = new();

    // ---- IMochiSim ----
    public int ParticleCount { get; private set; }
    public float GridDx => cellSize;
    public double SimTime { get; private set; }
    public float LastFrameSimTime { get; private set; }
    public int LastSubstepCount { get; private set; }

    /// <summary>Size of the simulated box, m.</summary>
    public Vector3 DomainSize => new Vector3(gridCells.x, gridCells.y, gridCells.z) * cellSize;
    /// <summary>Height of the dough floor above the bottom of the box, m.</summary>
    public float FloorOffset => floorCells * cellSize;

    const int Slots = 24;           // must match SLOTS in the compute shader
    const int ParticleStride = 112; // 7 float4
    const int MaxPrims = 512, MaxTools = 16;

    ComputeShader cs;
    int kToolSdf, kNodes, kParticles, kBin, kClear, kDensity, kCopy;
    ComputeBuffer parts, ids, gridV, gridM, gridR, toolA, toolB, prims, toolBuf, voxels;
    DoughLinks links;
    readonly ComputeBuffer[] counts = new ComputeBuffer[2];
    int cur;
    MeshBuilder builder;
    MeshFilter meshFilter;
    Vector3Int voxDims;

    readonly float[] primData = new float[MaxPrims * 24];
    readonly float[] toolData = new float[MaxTools * 8];
    class ToolState { public Vector3[] prev; public int lastFrame = -10; public OVRHand hand; public OVRSkeleton skeleton; public bool lookedUp; public bool fist; }
    readonly Dictionary<MatTool, ToolState> toolStates = new();

    float[] readback;
    bool readbackReady;

    void Start()
    {
        cs = Resources.Load<ComputeShader>("MochiSolver");
        if (cs == null) { Debug.LogError("MochiSolver: Resources/MochiSolver.compute not found."); enabled = false; return; }
        if (!SystemInfo.supportsComputeShaders) { Debug.LogError("MochiSolver: compute shaders are not supported on this device."); enabled = false; return; }
        kToolSdf = cs.FindKernel("ToolSdf");
        kNodes = cs.FindKernel("Nodes");
        kParticles = cs.FindKernel("Particles");
        kBin = cs.FindKernel("Bin");
        kClear = cs.FindKernel("ClearCounts");
        kDensity = cs.FindKernel("Density");
        kCopy = cs.FindKernel("CopyPositions");
        Allocate();
        ResetDough();
    }

    void Allocate()
    {
        int nodes = gridCells.x * gridCells.y * gridCells.z;
        counts[0] = new ComputeBuffer(nodes, 4);
        counts[1] = new ComputeBuffer(nodes, 4);
        ids = new ComputeBuffer(nodes * Slots, 4);
        gridV = new ComputeBuffer(nodes, 16);
        gridM = new ComputeBuffer(nodes, 8);
        gridR = new ComputeBuffer(nodes, 4);
        toolA = new ComputeBuffer(nodes, 16);
        toolB = new ComputeBuffer(nodes, 16);
        prims = new ComputeBuffer(MaxPrims, 96);
        toolBuf = new ComputeBuffer(MaxTools, 32);

        voxDims = gridCells * renderScale;
        voxels = new ComputeBuffer(voxDims.x * voxDims.y * voxDims.z, 4);
        voxels.SetData(new float[voxels.count]);
        if (marchingCubesCompute != null)
        {
            builder = new MeshBuilder(voxDims, triangleBudget, marchingCubesCompute);
            var go = new GameObject("MochiSurface");
            go.transform.SetParent(transform, false);
            go.transform.localPosition = DomainSize * 0.5f;   // the builder centres its mesh on the voxel box
            meshFilter = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.sharedMaterial = surfaceMaterial;
            mr.shadowCastingMode = ShadowCastingMode.On;
        }
        else Debug.LogWarning("MochiSolver: no marching cubes compute shader assigned; the dough will be invisible.");
    }

    /// <summary>Puts the dough back as a settled mound in the middle of the usu.</summary>
    [ContextMenu("Reset Dough")]
    public void ResetDough()
    {
        if (cs == null) return;
        var data = BuildMound();
        ParticleCount = data.Length / 28;
        parts?.Dispose();
        parts = new ComputeBuffer(Mathf.Max(ParticleCount, 1), ParticleStride);
        parts.SetData(data);
        readback = null; readbackReady = false;
        links?.Dispose();
        links = new DoughLinks(ParticleCount, linkReach * cellSize / Mathf.Pow(particlesPerCell, 1f / 3f), linkEveryFrames);

        SetConstants(1e-3f);
        cs.SetBuffer(kClear, "CountA", counts[0]);
        cs.SetBuffer(kClear, "CountB", counts[1]);
        cs.SetBuffer(kClear, "GridV", gridV);
        cs.SetBuffer(kClear, "GridM", gridM);
        cs.SetBuffer(kClear, "GridR", gridR);
        cs.DispatchThreads(kClear, gridCells);
        cs.SetBuffer(kBin, "Parts", parts);
        cs.SetBuffer(kBin, "CountB", counts[0]);
        cs.SetBuffer(kBin, "Ids", ids);
        cs.DispatchThreads(kBin, ParticleCount, 1, 1);
        cur = 0;
        Debug.Log($"MochiSolver: {ParticleCount} particles, grid {gridCells.x}x{gridCells.y}x{gridCells.z} at {cellSize * 1000f:F1} mm, " +
                  $"render {voxDims.x}x{voxDims.y}x{voxDims.z}");
    }

    /// <summary>Centre of the usu floor in local space.</summary>
    Vector3 FloorCentre => new Vector3(gridCells.x * cellSize * 0.5f, FloorOffset, gridCells.z * cellSize * 0.5f);

    float[] BuildMound()
    {
        // A spherical cap of the lump's volume: base radius a, height h, V = pi h (3a^2 + h^2) / 6.
        float volume = massKg / density;
        float a = Mathf.Min(0.115f, (usu != null ? usu.innerRadiusBottom : 0.125f) - 0.012f);
        float lo = 0.005f, hi = 0.3f;
        for (int i = 0; i < 40; i++)
        {
            float mid = 0.5f * (lo + hi);
            if (Mathf.PI * mid * (3f * a * a + mid * mid) / 6f < volume) lo = mid; else hi = mid;
        }
        float h = 0.5f * (lo + hi);
        float R = (a * a + h * h) / (2f * h);
        Vector3 fc = FloorCentre;
        Vector3 centre = new Vector3(fc.x, fc.y + h - R, fc.z);

        float s = cellSize / Mathf.Pow(particlesPerCell, 1f / 3f);
        var rng = new System.Random(1);
        var list = new List<float>(8192 * 28);
        int nxz = Mathf.CeilToInt(a / s) + 1, ny = Mathf.CeilToInt(h / s) + 1;
        for (int iy = 0; iy < ny; iy++)
            for (int iz = -nxz; iz <= nxz; iz++)
                for (int ix = -nxz; ix <= nxz; ix++)
                {
                    var p = new Vector3(fc.x + (ix + 0.5f) * s, fc.y + (iy + 0.5f) * s, fc.z + (iz + 0.5f) * s);
                    p += new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f) * (0.5f * s);
                    if ((p - centre).sqrMagnitude > R * R || p.y < fc.y + 0.1f * s) continue;
                    int o = list.Count;
                    for (int k = 0; k < 28; k++) list.Add(0f);
                    list[o] = p.x; list[o + 1] = p.y; list[o + 2] = p.z;
                    list[o + 20] = 1f; list[o + 21] = 1f; list[o + 22] = 1f;   // b = identity
                    list[o + 7] = 1f;   // v.w: measured density, rest until the first substep measures it
                }
        return list.ToArray();
    }

    void SetConstants(float dt)
    {
        float dx = cellSize;
        float pVol = dx * dx * dx / particlesPerCell;
        cs.SetInts("Dims", gridCells.x, gridCells.y, gridCells.z);
        cs.SetInt("ParticleCount", ParticleCount);
        cs.SetFloat("Dx", dx);
        cs.SetFloat("Dt", dt);
        cs.SetFloat("PVol", pVol);
        cs.SetFloat("PMass", pVol * density);
        cs.SetFloat("Mu", shearModulus);
        cs.SetFloat("Kappa", Mathf.Max(density * waveSpeed * waveSpeed - 4f / 3f * shearModulus, shearModulus));
        cs.SetFloat("YieldStress", yieldStress);
        cs.SetFloat("RelaxRate", 1f / Mathf.Max(relaxTime, 1e-3f));
        cs.SetFloat("MaxShear", maxShearStress);
        cs.SetFloat("Centering", centering);
        cs.SetFloat("FlowExp", flowExponent);
        cs.SetFloat("ThinStress", Mathf.Max(thinningStress, 1f));
        cs.SetFloat("Gravity", -9.81f);
        cs.SetFloat("Damping", damping);
        cs.SetFloat("Friction", toolFriction);
        cs.SetFloat("Band", tackBandCells * dx);
        cs.SetFloat("Cutoff", Mathf.Max(3f * dx, pullReach + dx));
        cs.SetFloat("PullSpeed", pullSpeed);
        cs.SetFloat("PullReach", pullReach);
        cs.SetFloat("PullShape", pullShape);
        cs.SetFloat("PullGain", pullGain);
        cs.SetFloat("GripReach", gripReach);
        cs.SetFloat("CrowdStiffness", crowdStiffness);
        cs.SetFloat("StrayDensity", strayDensity);
        cs.SetFloat("StrayDrag", strayDrag);
        cs.SetFloat("RestNodeMass", pVol * density * particlesPerCell);
        cs.SetFloat("FloorHold", floorHold);
        cs.SetInt("FloorCells", floorCells);
        cs.SetFloat("VMax", Mathf.Min(0.8f * dx / dt, maxDoughSpeed));
        cs.SetFloat("PushOut", 1f);
        cs.SetFloat("ThinDensity", thinDensity);
        cs.SetFloat("ThinBoost", Mathf.Max(thinBoost, 1f));
        Vector3 fc = FloorCentre;
        cs.SetVector("LumpCentre", fc + new Vector3(0f, 0.04f, 0f));
        if (usu != null)
        {
            cs.SetVector("Usu0", new Vector4(fc.x, fc.y, fc.z, usu.innerRadiusBottom));
            cs.SetVector("Usu1", new Vector4(usu.innerRadiusTop, usu.depth, 1f, 0f));
        }
        else cs.SetVector("Usu1", Vector4.zero);
    }

    void Update()
    {
        if (cs == null || parts == null || ParticleCount == 0) return;

        // Substeps: stable for the elastic wave and for dough pushed along at tool speed.
        // Tools cover this frame's displacement in the time that is actually simulated, so when the
        // substep cap makes the dough run slow, a tool still sweeps through it without leaving it behind.
        int primCount = GatherTools(out float maxMove);
        float frame = Mathf.Min(Time.deltaTime, 1f / 30f) * Mathf.Max(timeScale, 0f);
        float dtMax = cellSize / (waveSpeed / cfl + Mathf.Min(maxMove / Mathf.Max(frame, 1e-4f), maxToolSpeed) / 0.8f);
        int n = Mathf.Clamp(Mathf.CeilToInt(frame / dtMax), 1, Mathf.Max(maxSubsteps, 1));
        float dt = Mathf.Min(frame / n, dtMax);
        if (frame <= 0f) { n = 0; dt = dtMax; }
        float simFrame = Mathf.Max(n * dt, 1e-4f);
        UploadTools(primCount, simFrame);

        SetConstants(dt);

        cs.SetBuffer(kToolSdf, "Prims", prims);
        cs.SetBuffer(kToolSdf, "Tools", toolBuf);
        cs.SetBuffer(kToolSdf, "ToolA", toolA);
        cs.SetBuffer(kToolSdf, "ToolB", toolB);
        cs.DispatchThreads(kToolSdf, gridCells);

        cs.SetBuffer(kNodes, "Parts", parts);
        cs.SetBuffer(kNodes, "Ids", ids);
        cs.SetBuffer(kNodes, "GridV", gridV);
        cs.SetBuffer(kNodes, "GridM", gridM);
        cs.SetBuffer(kNodes, "GridR", gridR);
        cs.SetBuffer(kNodes, "ToolA", toolA);
        cs.SetBuffer(kNodes, "ToolB", toolB);
        cs.SetBuffer(kParticles, "Parts", parts);
        cs.SetBuffer(kParticles, "Ids", ids);
        cs.SetBuffer(kParticles, "GridV", gridV);
        cs.SetBuffer(kParticles, "GridM", gridM);
        cs.SetBuffer(kParticles, "ToolA", toolA);
        cs.SetBuffer(kParticles, "ToolB", toolB);
        links.Bind(cs, kParticles, keepInOnePiece);
        for (int s = 0; s < n; s++)
        {
            cs.SetFloat("TimeLeft", (n - 1 - s) * dt);
            cs.SetBuffer(kNodes, "CountA", counts[cur]);
            cs.SetBuffer(kNodes, "CountB", counts[1 - cur]);
            cs.DispatchThreads(kNodes, gridCells);
            cs.SetBuffer(kParticles, "CountB", counts[1 - cur]);
            cs.DispatchThreads(kParticles, ParticleCount, 1, 1);
            cur = 1 - cur;
        }
        if (logStats && Time.unscaledTime >= nextStats) { nextStats = Time.unscaledTime + 1f; LogStats(primCount, maxMove / Mathf.Max(frame, 1e-4f)); }
        LastSubstepCount = n;
        LastFrameSimTime = n * dt;
        SimTime += LastFrameSimTime;
        if (keepInOnePiece && n > 0) links.Tick(cs, kCopy, parts);

        if (builder != null && n > 0)
        {
            cs.SetInts("VoxDims", voxDims.x, voxDims.y, voxDims.z);
            cs.SetFloat("VoxDx", cellSize / renderScale);
            cs.SetFloat("VoxBlend", temporalBlend);
            cs.SetBuffer(kDensity, "GridR", gridR);
            cs.SetBuffer(kDensity, "Voxels", voxels);
            cs.DispatchThreads(kDensity, voxDims);
            builder.BuildIsosurface(voxels, isoValue, cellSize / renderScale);
            meshFilter.sharedMesh = builder.Mesh;
        }
    }

    /// <summary>Turns the gathered displacements into velocities over the simulated time and uploads the primitives.</summary>
    void UploadTools(int np, float simFrame)
    {
        float maxMove = maxToolSpeed * simFrame;
        for (int i = 0; i < np; i++)
            for (int k = 0; k < 3; k++)
            {
                int o = i * 24 + 12 + k * 4;
                var d = new Vector3(primData[o], primData[o + 1], primData[o + 2]);
                float m = d.magnitude;
                d *= (m > maxMove ? maxMove / m : 1f) / simFrame;
                primData[o] = d.x; primData[o + 1] = d.y; primData[o + 2] = d.z;
            }
        if (np > 0) prims.SetData(primData, 0, 0, np * 24);
    }

    /// <summary>Collects this frame's tool primitives in local space with their displacement since last frame.
    /// Returns the primitive count; maxMove is the largest displacement of a primitive near the domain.</summary>
    int GatherTools(out float maxMove)
    {
        activeTools.Clear();
        int np = 0, nt = 0;
        float vMax = 0f;
        Vector3 size = DomainSize;
        foreach (var tool in tools)
        {
            if (tool == null || !tool.isActiveAndEnabled || tool.primitives == null) continue;
            if (tool == usu && usu != null) continue;
            if (!toolStates.TryGetValue(tool, out var st)) toolStates[tool] = st = new ToolState();
            float tack = TackFor(tool, st);
            if (tack < 0f) continue;   // an untracked hand
            int count = Mathf.Min(tool.numPrimitives, tool.primitives.Length);
            if (count == 0 || np + count > MaxPrims || nt >= MaxTools) continue;
            activeTools.Add(tool);
            if (st.prev == null || st.prev.Length != count * 3) { st.prev = new Vector3[count * 3]; st.lastFrame = -10; }
            bool hasPrev = st.lastFrame == Time.frameCount - 1;
            st.lastFrame = Time.frameCount;

            Vector3 mn = Vector3.positiveInfinity, mx = Vector3.negativeInfinity;
            int first = np;
            for (int j = 0; j < count; j++)
            {
                var pr = tool.primitives[j];
                int o = np * 24;
                for (int k = 0; k < 3; k++)
                {
                    Vector3 world = k == 0 ? pr.sphere1 : (k == 1 ? pr.sphere2 : pr.sphere3);
                    float r = k == 0 ? pr.radii1 : (k == 1 ? pr.radii2 : pr.radii3);
                    Vector3 local = transform.InverseTransformPoint(world);
                    Vector3 vel = hasPrev ? local - st.prev[j * 3 + k] : Vector3.zero;
                    float speed = vel.magnitude;
                    st.prev[j * 3 + k] = local;
                    primData[o + k * 4] = local.x; primData[o + k * 4 + 1] = local.y; primData[o + k * 4 + 2] = local.z; primData[o + k * 4 + 3] = r;
                    primData[o + 12 + k * 4] = vel.x; primData[o + 12 + k * 4 + 1] = vel.y; primData[o + 12 + k * 4 + 2] = vel.z; primData[o + 12 + k * 4 + 3] = 0f;
                    if (r > 0f || k < 2)
                    {
                        mn = Vector3.Min(mn, local - Vector3.one * r);
                        mx = Vector3.Max(mx, local + Vector3.one * r);
                        // Only tools that can reach the dough limit the time step.
                        if (local.x > -0.1f && local.y > -0.1f && local.z > -0.1f && local.x < size.x + 0.1f && local.y < size.y + 0.1f && local.z < size.z + 0.1f)
                            vMax = Mathf.Max(vMax, speed);
                    }
                }
                primData[o + 15] = tack;
                np++;
            }
            int t = nt * 8;
            toolData[t] = mn.x; toolData[t + 1] = mn.y; toolData[t + 2] = mn.z; toolData[t + 3] = first;
            toolData[t + 4] = mx.x; toolData[t + 5] = mx.y; toolData[t + 6] = mx.z; toolData[t + 7] = count;
            nt++;
        }
        if (nt > 0) toolBuf.SetData(toolData, 0, 0, nt * 8);
        cs.SetInt("ToolCount", nt);
        maxMove = vMax;
        return np;
    }

    /// <summary>Tack seconds for a tool this frame, or -1 to leave it out (an untracked hand).</summary>
    float TackFor(MatTool tool, ToolState st)
    {
        if (tool is KineTool) return kineTack;
        if (tool is MatHand)
        {
            if (!st.lookedUp)
            {
                st.lookedUp = true;
                var go = GameObject.Find(tool.handType == MatTool.HandType.LeftHand
                    ? "OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand"
                    : "OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand");
                if (go != null) { st.hand = go.GetComponent<OVRHand>(); st.skeleton = go.GetComponent<OVRSkeleton>(); }
            }
            if (st.hand == null) return handTack;
            if (!st.hand.IsTracked) { st.fist = false; return -1f; }
            float curl = FistAmount(st.skeleton);
            st.fist = st.fist ? curl > 0.6f : curl > 0.85f;
            return st.fist ? fistTack : handTack;
        }
        return otherTack;
    }

    static readonly OVRSkeleton.BoneId[] FingerTips =
    {
        OVRSkeleton.BoneId.Hand_IndexTip, OVRSkeleton.BoneId.Hand_MiddleTip,
        OVRSkeleton.BoneId.Hand_RingTip, OVRSkeleton.BoneId.Hand_PinkyTip,
    };

    /// <summary>0 for an open hand, 1 for a closed fist, from how close the fingertips are to the wrist.</summary>
    public static float FistAmount(OVRSkeleton skeleton)
    {
        if (skeleton == null) return 0f;
        var wrist = HandBoneCompat.Find(skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
        if (wrist == null) return 0f;
        float sum = 0f; int n = 0;
        foreach (var id in FingerTips)
        {
            var tip = HandBoneCompat.Find(skeleton, id);
            if (tip == null) continue;
            sum += (tip.position - wrist.position).magnitude; n++;
        }
        if (n == 0) return 0f;
        return Mathf.InverseLerp(0.14f, 0.09f, sum / n);
    }

    float nextStats;

    /// <summary>One line per second for tuning on the headset: frame rate, substeps, tool load, and how wild the dough is.</summary>
    void LogStats(int primCount, float toolSpeed)
    {
        int count = ParticleCount; int sub = LastSubstepCount; float fps = 1f / Mathf.Max(Time.unscaledDeltaTime, 1e-4f);
        string tetherInfo = keepInOnePiece && links != null ? $"{links.LinkCount}/tethers {links.TetherCount}" : "off";
        float floor = FloorOffset, rim = FloorOffset + (usu != null ? usu.depth : 0.13f);
        string fists = "";
        foreach (var kv in toolStates) if (kv.Key is MatHand) fists += (kv.Key.handType == MatTool.HandType.LeftHand ? "L" : "R") + (kv.Value.fist ? "fist " : "open ");
        AsyncGPUReadback.Request(parts, req =>
        {
            if (req.hasError || this == null || count != ParticleCount) return;
            NativeArray<float> d = req.GetData<float>();
            int fast = 0, air = 0, bonded = 0, bad = 0; float vmax = 0f, top = 0f;
            for (int i = 0; i < count; i++)
            {
                int o = i * 28;
                float y = d[o + 1];
                if (float.IsNaN(y)) { bad++; continue; }
                float v = new Vector3(d[o + 4], d[o + 5], d[o + 6]).magnitude;
                if (v > 1.5f) fast++;
                if (y > rim) air++;
                if (d[o + 3] > 0f) bonded++;
                vmax = Mathf.Max(vmax, v); top = Mathf.Max(top, y - floor);
            }
            Debug.Log($"MochiStats fps={fps:F0} sub={sub} prims={primCount} toolSpeed={toolSpeed:F2} {fists}vmax={vmax:F2} fast={fast} aboveRim={air} bonded={bonded} top={top * 100f:F0}cm nan={bad} links={tetherInfo}");
        });
    }

    // ---- particle readback (gameplay, tests) ----

    public bool RequestParticleReadback()
    {
        if (parts == null || ParticleCount == 0) return false;
        int count = ParticleCount;
        AsyncGPUReadback.Request(parts, req =>
        {
            if (req.hasError || this == null || count != ParticleCount) return;
            NativeArray<float> data = req.GetData<float>();
            readback ??= new float[count * 3];
            for (int i = 0; i < count; i++)
            {
                readback[3 * i] = data[28 * i];
                readback[3 * i + 1] = data[28 * i + 1];
                readback[3 * i + 2] = data[28 * i + 2];
            }
            readbackReady = true;
        });
        return true;
    }

    public bool ReadParticles(float[] positions)
    {
        if (!readbackReady || readback == null || positions == null || positions.Length != readback.Length) return false;
        System.Array.Copy(readback, positions, readback.Length);
        return true;
    }

    /// <summary>Blocking readback for tuning in the Editor: bonded particles, highest point above the floor (m), mean volume ratio.</summary>
    public string DebugStats()
    {
        if (parts == null) return "no particles";
        var data = new float[ParticleCount * 28];
        parts.GetData(data);
        int bonded = 0, bad = 0, dropped = 0, atEdge = 0; float top = 0f, vmax = 0f; double j = 0;
        var perCell = new Dictionary<int, int>();
        Vector3 size = DomainSize;
        for (int i = 0; i < ParticleCount; i++)
        {
            int o = i * 28;
            if (float.IsNaN(data[o]) || float.IsNaN(data[o + 1]) || float.IsNaN(data[o + 2]) || float.IsInfinity(data[o] + data[o + 1] + data[o + 2])) { bad++; continue; }
            // Same binning as the shader: particles beyond the slots of their cell are left out of the grid (and of the picture).
            int cx = (int)(data[o] / cellSize - 0.5f), cy = (int)(data[o + 1] / cellSize - 0.5f), cz = (int)(data[o + 2] / cellSize - 0.5f);
            int key = cx + gridCells.x * (cy + gridCells.y * cz);
            perCell.TryGetValue(key, out int c); perCell[key] = c + 1;
            if (c >= Slots) dropped++;
            if (data[o] < 2.5f * cellSize || data[o + 2] < 2.5f * cellSize || data[o] > size.x - 2.5f * cellSize || data[o + 2] > size.z - 2.5f * cellSize || data[o + 1] > size.y - 2.5f * cellSize) atEdge++;
            if (data[o + 3] > 0f) bonded++;
            top = Mathf.Max(top, data[o + 1] - FloorOffset);
            vmax = Mathf.Max(vmax, new Vector3(data[o + 4], data[o + 5], data[o + 6]).magnitude);
            float bxx = data[o + 20], byy = data[o + 21], bzz = data[o + 22], bxy = data[o + 24], bxz = data[o + 25], byz = data[o + 26];
            float det = bxx * (byy * bzz - byz * byz) - bxy * (bxy * bzz - byz * bxz) + bxz * (bxy * byz - byy * bxz);
            j += Mathf.Sqrt(Mathf.Max(det, 0f));
        }
        return $"bonded={bonded} top={top * 100f:F1}cm vmax={vmax:F2} J={j / ParticleCount:F3} sub={LastSubstepCount} nan={bad} dropped={dropped} cells={perCell.Count} edge={atEdge}";
    }

    void OnDestroy()
    {
        parts?.Dispose(); ids?.Dispose(); gridV?.Dispose(); gridM?.Dispose(); gridR?.Dispose(); links?.Dispose();
        toolA?.Dispose(); toolB?.Dispose(); prims?.Dispose(); toolBuf?.Dispose(); voxels?.Dispose();
        counts[0]?.Dispose(); counts[1]?.Dispose();
        builder?.Dispose();
        parts = null;
    }
}
