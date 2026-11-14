using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Scripted mochitsuki cycle for judging how the simulated dough behaves:
///   settle -> palm press (kneader) -> 3 x [kine strike (pounder) + fold (kneader)] -> final press -> rest.
/// Tools move on simulated time so their speeds are physical even if the simulation runs slower
/// than real time. Measurements and side/top images are written to Temp/mochi_bench/.
/// </summary>
[DefaultExecutionOrder(-500)]
public class MochiTestBench : MonoBehaviour
{
    public MochiStation station;
    public Transform kine;
    public Transform palm;
    public Transform foldHand;
    public bool runOnStart = true;
    public int strikes = 3;
    public float kineImpactSpeed = 4f;
    public float kineLiftSpeed = 1.5f;
    public int imageSize = 512;

    public bool Finished { get; private set; }
    public string Phase { get; private set; } = "idle";

    Mpm3DMarching sim;
    float[] xs;
    readonly List<Dictionary<string, object>> checkpoints = new();
    readonly List<string> notes = new();
    string outDir;
    Camera sideCam, topCam;
    RenderTexture rt;
    float realStart; double simStart; int frames; long substeps;
    Vector3 parkKine, parkPalm, parkFold;

    void Start()
    {
        sim = station.mochi;
        outDir = Path.Combine(Application.dataPath, "..", "Temp", "mochi_bench");
        Directory.CreateDirectory(outDir);
        parkKine = new Vector3(0.0f, 0.45f, 0.0f);
        parkPalm = new Vector3(0.30f, 0.45f, 0.0f);
        parkFold = new Vector3(-0.30f, 0.45f, 0.0f);
        Place(kine, parkKine, Quaternion.identity);
        Place(palm, parkPalm, Quaternion.identity);
        Place(foldHand, parkFold, Quaternion.identity);
        SetupCameras();
        if (runOnStart) StartCoroutine(Run());
    }

    void Update()
    {
        if (Phase != "idle" && !Finished) { frames++; substeps += sim.LastSubstepCount; }
    }

    // ---------- scripted sequence ----------

    IEnumerator Run()
    {
        // Wait for the solver to initialise and allocate its buffers.
        while (sim.ParticleCount == 0 || sim.SimTime <= 0) yield return null;
        xs = new float[sim.ParticleCount * 3];
        realStart = Time.realtimeSinceStartup; simStart = sim.SimTime;

        Phase = "settle";
        yield return WaitSim(3.0f);
        yield return Snap();
        var settled = Measure("settled");
        float h0 = (float)settled["height_m"];
        yield return WaitSim(2.0f);
        yield return Snap();
        Measure("settled_plus_2s");

        // Kneader: press the heel of the palm into the middle, hold, lift.
        Phase = "press";
        yield return Snap();
        var pressTop = Measure("before_press");
        float hTop = (float)pressTop["height_m"];
        float pressTo = Mathf.Max(0.35f * hTop, 0.02f);
        yield return MoveSim(palm, P(0f, hTop + 0.06f, 0f), P(0f, hTop + 0.06f, 0f), Quaternion.identity, Quaternion.identity, 0.01f);
        yield return MoveSim(palm, P(0f, hTop + 0.06f, 0f), P(0f, pressTo, 0f), Quaternion.identity, Quaternion.identity, (hTop + 0.06f - pressTo) / 0.25f);
        yield return WaitSim(0.4f);
        yield return MoveSim(palm, P(0f, pressTo, 0f), P(0f, hTop + 0.12f, 0f), Quaternion.identity, Quaternion.identity, (hTop + 0.12f - pressTo) / 0.3f);
        yield return Snap();
        var afterPress = Measure("press_released", hTop - pressTo);
        Capture("press_released");
        yield return WaitSim(1.0f);
        yield return Snap();
        Measure("press_recovered_1s", hTop - pressTo, afterPress);
        Place(palm, parkPalm, Quaternion.identity);

        for (int s = 1; s <= strikes; s++)
        {
            // Pounder: kine falls from 40 cm at impact speed, drives to near the bottom, lifts.
            Phase = $"strike{s}";
            yield return Snap();
            var before = Measure($"strike{s}_before");
            float hb = (float)before["height_m"];
            float bottom = Mathf.Max(0.25f * hb, 0.02f);
            float top = 0.40f;
            yield return MoveSim(kine, P(0, top, 0), P(0, bottom, 0), Quaternion.identity, Quaternion.identity, (top - bottom) / kineImpactSpeed);
            Capture($"strike{s}_impact");
            yield return WaitSim(0.05f);
            // Lift, sampling how far the sticky mochi follows the kine up.
            float strand = 0f; float liftTime = (top - bottom) / kineLiftSpeed; float t = 0f; int sample = 0;
            while (t < liftTime)
            {
                float dt = StepDt();
                t += dt;
                Place(kine, P(0, Mathf.Lerp(bottom, top, Mathf.Clamp01(t / liftTime)), 0), Quaternion.identity);
                if (sample > 0 && sim.ReadParticles(xs)) strand = Mathf.Max(strand, MaxHeightNearAxis(0.07f) - hb);
                sim.RequestParticleReadback(); sample++;
                yield return null;
            }
            yield return Snap();
            var crater = Measure($"strike{s}_released", hb - bottom);
            crater["strand_above_lump_m"] = strand;
            Capture($"strike{s}_released");
            yield return WaitSim(1.0f);
            yield return Snap();
            Measure($"strike{s}_recovered_1s", hb - bottom, crater);
            Place(kine, parkKine, Quaternion.identity);

            // Kneader: kaeshi - scoop the far edge from underneath and fold it onto the centre.
            Phase = $"fold{s}";
            yield return Snap();
            var pre = Measure($"fold{s}_before");
            float r = (float)pre["diameter_m"] * 0.5f; float h = (float)pre["height_m"];
            // PalmTool's normal is local -Y. Rotating about Z by theta points it at (sin theta, -cos theta) in XY:
            // -110 deg faces in toward the lump and slightly up (scooping), -60 pushes the flap over, 0 presses down.
            Quaternion scoop = Quaternion.Euler(0, 0, -110f);
            Quaternion roll = Quaternion.Euler(0, 0, -60f);
            Quaternion flat = Quaternion.identity;
            float palmY = 0.035f;
            yield return MoveSim(foldHand, P(r + 0.06f, 0.12f, 0), P(r + 0.05f, palmY, 0), scoop, scoop, 0.3f);
            yield return MoveSim(foldHand, P(r + 0.05f, palmY, 0), P(r - 0.07f, palmY, 0), scoop, scoop, 0.12f / 0.3f);
            yield return MoveSim(foldHand, P(r - 0.07f, palmY, 0), P(0.03f, h + 0.06f, 0), scoop, roll, 0.5f);
            yield return MoveSim(foldHand, P(0.03f, h + 0.06f, 0), P(-0.01f, 0.6f * h, 0), roll, flat, 0.3f);
            yield return WaitSim(0.15f);
            yield return MoveSim(foldHand, P(-0.01f, 0.6f * h, 0), P(-0.01f, h + 0.15f, 0), flat, flat, 0.4f);
            yield return Snap();
            Measure($"fold{s}_done");
            Capture($"fold{s}_done");
            Place(foldHand, parkFold, Quaternion.identity);
        }

        Phase = "final_press";
        yield return Snap();
        var pf = Measure("final_before_press");
        float hf = (float)pf["height_m"];
        yield return MoveSim(palm, P(0f, hf + 0.06f, 0f), P(0f, 0.45f * hf, 0f), Quaternion.identity, Quaternion.identity, (0.55f * hf + 0.06f) / 0.25f);
        yield return WaitSim(0.3f);
        yield return MoveSim(palm, P(0f, 0.45f * hf, 0f), P(0f, hf + 0.15f, 0f), Quaternion.identity, Quaternion.identity, 0.5f);
        Place(palm, parkPalm, Quaternion.identity);
        yield return WaitSim(2.0f);
        yield return Snap();
        Measure("final_rest");
        Capture("final_rest");

        Phase = "done";
        WriteReport(h0);
        Finished = true;
    }

    IEnumerator Snap()
    {
        sim.RequestParticleReadback();
        yield return null;
        yield return null;
    }

    Vector3 P(float x, float y, float z) => new Vector3(x, station.FloorY + y, z);

    void Place(Transform t, Vector3 stationPos, Quaternion stationRot)
    {
        if (t == null) return;
        t.SetPositionAndRotation(station.transform.TransformPoint(stationPos), station.transform.rotation * stationRot);
    }

    float StepDt() => sim.LastFrameSimTime > 0f ? sim.LastFrameSimTime : Time.deltaTime;

    IEnumerator WaitSim(float seconds)
    {
        double end = sim.SimTime + seconds;
        while (sim.SimTime < end) yield return null;
    }

    IEnumerator MoveSim(Transform t, Vector3 a, Vector3 b, Quaternion ra, Quaternion rb, float duration)
    {
        float e = 0f;
        Place(t, a, ra);
        while (e < duration)
        {
            e += StepDt();
            float f = Mathf.Clamp01(e / Mathf.Max(duration, 1e-4f));
            Place(t, Vector3.Lerp(a, b, f), Quaternion.Slerp(ra, rb, f));
            yield return null;
        }
        Place(t, b, rb);
    }

    // ---------- measurements ----------

    Vector3 Pos(int i) => station.SimToStation(new Vector3(xs[3 * i], xs[3 * i + 1], xs[3 * i + 2]));

    float MaxHeightNearAxis(float radius)
    {
        float m = 0f; int n = sim.ParticleCount;
        for (int i = 0; i < n; i++)
        {
            var p = Pos(i);
            if (p.x * p.x + p.z * p.z < radius * radius) m = Mathf.Max(m, p.y - station.FloorY);
        }
        return m;
    }

    Dictionary<string, object> Measure(string label, float toolPenetration = -1f, Dictionary<string, object> reference = null)
    {
        var d = new Dictionary<string, object> { ["label"] = label, ["sim_time_s"] = (float)(sim.SimTime - simStart) };
        if (!sim.ReadParticles(xs)) { d["error"] = "readback failed"; checkpoints.Add(d); return d; }
        int n = sim.ParticleCount;
        float floor = station.FloorY;
        var heights = new float[n]; var radii = new float[n];
        Vector3 com = Vector3.zero;
        int nearWall = 0, outsideBowl = 0, nonFinite = 0;
        float half = station.domainSize * 0.5f, cell = station.domainSize / station.gridResolution;
        var usu = station.sharedTools.Find(t => t is UsuTool) as UsuTool;
        float bowl = usu != null ? usu.innerRadiusTop + 0.01f : half;
        for (int i = 0; i < n; i++)
        {
            var p = Pos(i);
            if (!float.IsFinite(p.x) || !float.IsFinite(p.y) || !float.IsFinite(p.z)) { nonFinite++; continue; }
            com += p;
            if (Mathf.Abs(p.x) > half - 4 * cell || Mathf.Abs(p.z) > half - 4 * cell) nearWall++;
            if (Mathf.Sqrt(p.x * p.x + p.z * p.z) > bowl) outsideBowl++;
        }
        int valid = Mathf.Max(n - nonFinite, 1);
        com /= valid;
        for (int i = 0; i < n; i++)
        {
            var p = Pos(i);
            heights[i] = p.y - floor;
            radii[i] = Mathf.Sqrt((p.x - com.x) * (p.x - com.x) + (p.z - com.z) * (p.z - com.z));
        }
        float height = Percentile(heights, 0.99f);
        float diameter = 2f * Percentile(radii, 0.97f);
        d["height_m"] = height;
        d["diameter_m"] = diameter;
        d["aspect_h_over_d"] = diameter > 0 ? height / diameter : 0;
        d["com_station_m"] = new float[] { com.x, com.y - floor, com.z };
        float volRatio = VolumeRatio();
        d["volume_ratio"] = volRatio;
        d["frac_near_domain_wall"] = nearWall / (float)n;
        d["frac_outside_bowl"] = outsideBowl / (float)n;
        d["nonfinite_particles"] = nonFinite;
        var (pieces, largest) = Components(cell);
        d["pieces"] = pieces; d["largest_piece_frac"] = largest;
        var (rim, centre) = CraterProfile();
        d["surface_rim_m"] = rim; d["surface_centre_m"] = centre; d["crater_depth_m"] = rim - centre;
        if (toolPenetration > 0f)
        {
            d["tool_penetration_m"] = toolPenetration;
            d["dent_retained_frac"] = toolPenetration > 0 ? Mathf.Clamp((rim - centre) / toolPenetration, -1f, 2f) : 0f;
        }
        if (reference != null && reference.TryGetValue("crater_depth_m", out var refDepth))
        {
            float r0 = (float)refDepth, r1 = rim - centre;
            d["springback_frac_1s"] = r0 > 1e-4f ? (r0 - r1) / r0 : 0f;
        }
        checkpoints.Add(d);
        Debug.Log($"[MochiBench] {label}: h={height * 100:F1}cm d={diameter * 100:F1}cm h/d={(diameter > 0 ? height / diameter : 0):F2} " +
                  $"vol={volRatio:F3} pieces={pieces} ({largest:P0}) crater={(rim - centre) * 100:F1}cm " +
                  $"outsideBowl={outsideBowl / (float)n:P1} nonfinite={nonFinite}");
        return d;
    }

    // Volume relative to rest, from particle density: particles start at 8 per solver cell, so the mean count
    // in fully interior cells (all six neighbours occupied) over 8 is the compression; volume ~ its inverse.
    float VolumeRatio()
    {
        int g = station.gridResolution;
        var count = new int[g * g * g];
        int n = sim.ParticleCount;
        for (int i = 0; i < n; i++)
        {
            int ix = Mathf.FloorToInt(xs[3 * i] * g), iy = Mathf.FloorToInt(xs[3 * i + 1] * g), iz = Mathf.FloorToInt(xs[3 * i + 2] * g);
            if (ix < 1 || iy < 1 || iz < 1 || ix >= g - 1 || iy >= g - 1 || iz >= g - 1) continue;
            count[(ix * g + iy) * g + iz]++;
        }
        long sum = 0; int cells = 0;
        for (int ix = 1; ix < g - 1; ix++)
            for (int iy = 1; iy < g - 1; iy++)
                for (int iz = 1; iz < g - 1; iz++)
                {
                    int c = (ix * g + iy) * g + iz;
                    if (count[c] == 0) continue;
                    if (count[c + g * g] == 0 || count[c - g * g] == 0 || count[c + g] == 0 || count[c - g] == 0 || count[c + 1] == 0 || count[c - 1] == 0) continue;
                    sum += count[c]; cells++;
                }
        return cells > 0 ? 8f * cells / sum : 1f;
    }

    // Surface height (max particle height) in an annulus 7-10 cm from the usu axis vs within 3 cm of it.
    (float rim, float centre) CraterProfile()
    {
        const float cs = 0.01f; const int g = 40;
        var top = new float[g, g];
        int n = sim.ParticleCount;
        for (int i = 0; i < n; i++)
        {
            var p = Pos(i);
            int ix = Mathf.FloorToInt(p.x / cs) + g / 2, iz = Mathf.FloorToInt(p.z / cs) + g / 2;
            if (ix < 0 || iz < 0 || ix >= g || iz >= g) continue;
            top[ix, iz] = Mathf.Max(top[ix, iz], p.y - station.FloorY);
        }
        var ring = new List<float>(); var mid = new List<float>();
        for (int ix = 0; ix < g; ix++)
            for (int iz = 0; iz < g; iz++)
            {
                float cx = (ix - g / 2 + 0.5f) * cs, cz = (iz - g / 2 + 0.5f) * cs, r = Mathf.Sqrt(cx * cx + cz * cz);
                if (r < 0.03f) mid.Add(top[ix, iz]);
                else if (r > 0.07f && r < 0.10f && top[ix, iz] > 0f) ring.Add(top[ix, iz]);
            }
        ring.Sort(); mid.Sort();
        float rimH = ring.Count > 0 ? ring[ring.Count / 2] : 0f;
        float midH = mid.Count > 0 ? mid[mid.Count / 2] : 0f;
        return (rimH, midH);
    }

    // Connected pieces on an occupancy grid at the solver's cell size (6-connected).
    (int pieces, float largest) Components(float cell)
    {
        int g = station.gridResolution;
        var count = new int[g * g * g];
        int n = sim.ParticleCount;
        for (int i = 0; i < n; i++)
        {
            int ix = Mathf.FloorToInt(xs[3 * i] * g), iy = Mathf.FloorToInt(xs[3 * i + 1] * g), iz = Mathf.FloorToInt(xs[3 * i + 2] * g);
            if (ix < 0 || iy < 0 || iz < 0 || ix >= g || iy >= g || iz >= g) continue;
            count[(ix * g + iy) * g + iz]++;
        }
        var label = new int[count.Length];
        var stack = new Stack<int>();
        int pieces = 0; int best = 0; int total = 0;
        for (int c = 0; c < count.Length; c++)
        {
            if (count[c] == 0 || label[c] != 0) continue;
            pieces++; int mass = 0;
            stack.Push(c); label[c] = pieces;
            while (stack.Count > 0)
            {
                int q = stack.Pop(); mass += count[q];
                int qx = q / (g * g), qy = (q / g) % g, qz = q % g;
                void Try(int x, int y, int z)
                {
                    if (x < 0 || y < 0 || z < 0 || x >= g || y >= g || z >= g) return;
                    int id = (x * g + y) * g + z;
                    if (count[id] > 0 && label[id] == 0) { label[id] = pieces; stack.Push(id); }
                }
                Try(qx - 1, qy, qz); Try(qx + 1, qy, qz); Try(qx, qy - 1, qz); Try(qx, qy + 1, qz); Try(qx, qy, qz - 1); Try(qx, qy, qz + 1);
            }
            total += mass; best = Mathf.Max(best, mass);
        }
        // Ignore specks of a few particles: count pieces holding at least 0.5% of the mass.
        int significant = 0;
        var massPer = new Dictionary<int, int>();
        for (int c = 0; c < count.Length; c++) if (label[c] > 0) { massPer.TryGetValue(label[c], out int m); massPer[label[c]] = m + count[c]; }
        foreach (var kv in massPer) if (kv.Value >= 0.005f * total) significant++;
        return (significant, total > 0 ? best / (float)total : 0f);
    }

    static float Percentile(float[] v, float q)
    {
        var a = (float[])v.Clone();
        Array.Sort(a);
        return a.Length == 0 ? 0f : a[Mathf.Clamp((int)(q * (a.Length - 1)), 0, a.Length - 1)];
    }

    // ---------- images ----------

    void SetupCameras()
    {
        rt = new RenderTexture(imageSize, imageSize, 24, RenderTextureFormat.ARGB32) { antiAliasing = 4 };
        // Side view at table height through the (hidden) usu wall, and a straight-down view of the bowl.
        sideCam = MakeCam("BenchSideCam", new Vector3(0.0f, station.FloorY + 0.10f, -0.48f), Quaternion.Euler(8f, 0f, 0f), 30f);
        topCam = MakeCam("BenchTopCam", new Vector3(0.0f, station.FloorY + 0.60f, 0.0f), Quaternion.Euler(90f, 0f, 0f), 32f);
    }

    Camera MakeCam(string name, Vector3 stationPos, Quaternion stationRot, float fov)
    {
        var go = new GameObject(name);
        go.transform.SetParent(station.transform, false);
        go.transform.localPosition = stationPos; go.transform.localRotation = stationRot;
        var cam = go.AddComponent<Camera>();
        cam.enabled = false;
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.16f, 0.17f, 0.19f, 1f);
        cam.fieldOfView = fov; cam.nearClipPlane = 0.02f; cam.farClipPlane = 5f;
        cam.targetTexture = rt;
        return cam;
    }

    void Capture(string label)
    {
        var tex = new Texture2D(imageSize * 2, imageSize, TextureFormat.RGB24, false);
        int x = 0;
        // Hide the usu body so the dough is visible from the side; the top view sees into the bowl anyway.
        var usuVisual = station.GetComponentInChildren<UsuTool>()?.transform.Find("UsuVisual");
        bool usuWasActive = usuVisual != null && usuVisual.gameObject.activeSelf;
        if (usuVisual != null) usuVisual.gameObject.SetActive(false);
        foreach (var cam in new[] { sideCam, topCam })
        {
            var req = new RenderPipeline.StandardRequest { destination = rt };
            if (RenderPipeline.SupportsRenderRequest(cam, req)) RenderPipeline.SubmitRenderRequest(cam, req);
            else cam.Render();
            var prev = RenderTexture.active; RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, imageSize, imageSize), x, 0);
            RenderTexture.active = prev;
            x += imageSize;
        }
        if (usuVisual != null) usuVisual.gameObject.SetActive(usuWasActive);
        tex.Apply();
        File.WriteAllBytes(Path.Combine(outDir, $"{checkpoints.Count:D2}_{label}.png"), tex.EncodeToPNG());
        Destroy(tex);
    }

    // ---------- report ----------

    void WriteReport(float h0)
    {
        float real = Time.realtimeSinceStartup - realStart;
        float simT = (float)(sim.SimTime - simStart);
        var sb = new StringBuilder();
        sb.Append("{\n  \"station\": {");
        sb.Append($"\"mass_kg\": {station.massKg}, \"density\": {station.density}, \"E_pa\": {station.youngsModulus}, \"nu\": {station.poissonRatio}, ");
        sb.Append($"\"yield_pa\": {station.yieldStress}, \"damping\": {station.viscousDamping}, \"stickiness\": {station.stickiness}, ");
        sb.Append($"\"domain_m\": {station.domainSize}, \"grid\": {station.gridResolution}, \"particles\": {sim.ParticleCount}}},\n");
        sb.Append($"  \"performance\": {{\"frames\": {frames}, \"real_s\": {real:F2}, \"sim_s\": {simT:F2}, \"sim_over_real\": {(real > 0 ? simT / real : 0):F3}, ");
        sb.Append($"\"mean_fps\": {(real > 0 ? frames / real : 0):F1}, \"mean_substeps\": {(frames > 0 ? substeps / (double)frames : 0.0).ToString("F1", System.Globalization.CultureInfo.InvariantCulture)}}},\n");
        sb.Append("  \"checkpoints\": [\n");
        for (int i = 0; i < checkpoints.Count; i++)
        {
            sb.Append("    {");
            int j = 0;
            foreach (var kv in checkpoints[i])
            {
                if (j++ > 0) sb.Append(", ");
                sb.Append('"').Append(kv.Key).Append("\": ");
                switch (kv.Value)
                {
                    case string s: sb.Append('"').Append(s).Append('"'); break;
                    case float f: sb.Append(float.IsFinite(f) ? f.ToString("G6", System.Globalization.CultureInfo.InvariantCulture) : "null"); break;
                    case int k: sb.Append(k); break;
                    case float[] a: sb.Append('[').Append(string.Join(", ", Array.ConvertAll(a, z => z.ToString("G5", System.Globalization.CultureInfo.InvariantCulture)))).Append(']'); break;
                    default: sb.Append('"').Append(kv.Value).Append('"'); break;
                }
            }
            sb.Append(i < checkpoints.Count - 1 ? "},\n" : "}\n");
        }
        sb.Append("  ]\n}\n");
        File.WriteAllText(Path.Combine(outDir, "report.json"), sb.ToString());
        Debug.Log($"[MochiBench] done: {checkpoints.Count} checkpoints, sim {simT:F1}s in {real:F1}s real ({simT / Mathf.Max(real, 1e-3f):P0} of real time), " +
                  $"{(frames > 0 ? substeps / (float)frames : 0):F1} substeps/frame. Report: {outDir}");
    }
}
