using System.Collections;
using System.IO;
using UnityEngine;

/// <summary>
/// Development aid: makes the kine pound on its own and saves a side view of every few frames
/// (usu wall hidden) to Temp/mochi_native/, so a strike can be compared with the reference video.
/// Add it at runtime; it is not part of the game.
/// </summary>
public class MochiStrikeRecorder : MonoBehaviour
{
    public float settleSeconds = 1.5f;
    public float recordSeconds = 1.3f;
    public int everyNthFrame = 1;
    public float period = 2.2f, impactSpeed = 4f, liftSpeed = 0.8f, bottomHeight = 0.04f, dwell = 0.06f, raisedHeight = 0.30f;
    public bool hideUsu = true;
    [Tooltip("strike: auto-pounding kine. knead: a scripted palm presses, pushes sideways, then grips and lifts like a fist.")]
    public string mode = "strike";
    public string tag = "run";
    public bool Finished { get; private set; }
    public string Summary { get; private set; } = "";

    IEnumerator Start()
    {
        var station = FindFirstObjectByType<MochiStation>();
        var drv = station.GetComponentInChildren<KineDriver>(true);
        var sim = station.Sim;
        drv.period = period; drv.impactSpeed = impactSpeed; drv.liftSpeed = liftSpeed;
        drv.bottomHeight = bottomHeight; drv.dwell = dwell; drv.raisedHeight = raisedHeight;
        drv.mode = KineDriver.Mode.AutoPound;
        PalmTool palm = null;
        if (mode == "knead")
        {
            drv.mode = KineDriver.Mode.External; drv.Rest();
            foreach (var r in drv.GetComponentsInChildren<Renderer>()) r.enabled = false;
            palm = station.GetComponentsInChildren<PalmTool>(true)[0];
            palm.transform.SetParent(station.transform, false);
            palm.transform.localPosition = new Vector3(0f, station.FloorY + 0.25f, 0f);
            palm.gameObject.SetActive(true);
            if (station.solver != null && !station.solver.tools.Contains(palm)) station.solver.tools.Add(palm);
            recordSeconds = 6.2f;
        }

        Renderer usuRenderer = null;
        var usuVisual = station.GetComponentInChildren<UsuTool>()?.transform.Find("UsuVisual");
        if (usuVisual != null) usuRenderer = usuVisual.GetComponent<Renderer>();

        var go = new GameObject("StrikeRecorderCam");
        go.transform.SetParent(station.transform, false);
        // Side-on, at right angles to the handle, looking at the lump.
        go.transform.localPosition = new Vector3(-0.50f, station.FloorY + 0.14f, 0f);
        go.transform.localRotation = Quaternion.Euler(6f, 90f, 0f);
        var cam = go.AddComponent<Camera>();
        cam.enabled = false; cam.fieldOfView = 30f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 5f;
        cam.clearFlags = CameraClearFlags.SolidColor; cam.backgroundColor = new Color(0.08f, 0.08f, 0.09f);
        cam.stereoTargetEye = StereoTargetEyeMask.None;
        var rt = new RenderTexture(480, 360, 24);
        cam.targetTexture = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);

        string dir = Path.Combine(Application.dataPath, "..", "Temp", "mochi_native", tag);
        if (Directory.Exists(dir)) Directory.Delete(dir, true);
        Directory.CreateDirectory(dir);

        // Let the lump settle, then wait for the kine to start falling.
        double settleEnd = sim.SimTime + settleSeconds;
        while (sim.SimTime < settleEnd) yield return null;
        var kine = palm != null ? palm.transform : drv.transform;
        if (palm == null)
            while (station.transform.InverseTransformPoint(kine.position).y > station.FloorY + raisedHeight - 0.01f) yield return null;
        var log = new System.Text.StringBuilder();
        float real0 = Time.realtimeSinceStartup; double sim0 = sim.SimTime;
        int frame = 0, shots = 0; long substeps = 0;
        while (sim.SimTime - sim0 < recordSeconds)
        {
            if (palm != null) Knead(station, palm, (float)(sim.SimTime - sim0));
            yield return new WaitForEndOfFrame();
            substeps += sim.LastSubstepCount;
            if (station.solver != null)
                log.AppendLine($"{(sim.SimTime - sim0) * 1000.0:0000}ms kine={(station.transform.InverseTransformPoint(kine.position).y - station.FloorY) * 100f:F1}cm {station.solver.DebugStats()}");
            if (frame++ % Mathf.Max(everyNthFrame, 1) != 0) continue;
            if (hideUsu && usuRenderer != null) usuRenderer.enabled = false;
            cam.Render();
            if (hideUsu && usuRenderer != null) usuRenderer.enabled = true;
            var prev = RenderTexture.active; RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0); tex.Apply();
            RenderTexture.active = prev;
            File.WriteAllBytes(Path.Combine(dir, $"f_{shots++:000}_{(sim.SimTime - sim0) * 1000.0:0000}ms.png"), tex.EncodeToPNG());
        }
        float real = Time.realtimeSinceStartup - real0;
        Summary = $"frames={frame} shots={shots} fps={frame / real:F1} substeps/frame={substeps / (float)frame:F1} " +
                  $"sim/real={(sim.SimTime - sim0) / real:F2} particles={sim.ParticleCount}";
        File.WriteAllText(Path.Combine(dir, "series.txt"), log.ToString());
        File.WriteAllText(Path.Combine(dir, "summary.txt"), Summary);
        Debug.Log("MochiStrikeRecorder: " + Summary);
        Finished = true;
        Destroy(go); rt.Release();
    }

    struct Key { public float t; public Vector3 p; public float zAngle; public bool grip;
                 public Key(float t, float x, float y, float z, float a = 0f, bool grip = false) { this.t = t; p = new Vector3(x, y, z); zAngle = a; this.grip = grip; } }

    // Heights are above the dough floor. The palm normal is local -Y; a Z rotation of 90 turns it to face +X.
    static readonly Key[] KneadKeys =
    {
        // open-hand press and lift: the dough should dent, spring back a little, and not follow the hand up
        new Key(0.0f, 0f, 0.25f, 0f), new Key(0.5f, 0f, 0.04f, 0f), new Key(0.8f, 0f, 0.04f, 0f), new Key(1.2f, 0f, 0.25f, 0f),
        // sideways push across the usu: the dough should pile up and fold, the hand slides off
        new Key(1.5f, -0.14f, 0.25f, 0f, 90f), new Key(1.8f, -0.14f, 0.05f, 0f, 90f), new Key(2.3f, 0.00f, 0.05f, 0f, 90f),
        new Key(2.5f, 0.00f, 0.05f, 0f, 90f), new Key(2.9f, -0.14f, 0.25f, 0f, 90f),
        // grip (fist) and lift: the dough should come up with the hand and hang, then drop when released
        new Key(3.2f, 0f, 0.25f, 0f), new Key(3.7f, 0f, 0.05f, 0f, 0f, true), new Key(3.9f, 0f, 0.05f, 0f, 0f, true),
        new Key(4.5f, 0f, 0.20f, 0f, 0f, true), new Key(5.2f, 0f, 0.20f, 0f, 0f, true), new Key(5.25f, 0f, 0.20f, 0f), new Key(9f, 0f, 0.20f, 0f),
    };

    void Knead(MochiStation station, PalmTool palm, float t)
    {
        int i = 0;
        while (i < KneadKeys.Length - 2 && KneadKeys[i + 1].t <= t) i++;
        Key a = KneadKeys[i], b = KneadKeys[i + 1];
        float u = Mathf.Clamp01((t - a.t) / Mathf.Max(b.t - a.t, 1e-4f));
        Vector3 p = Vector3.Lerp(a.p, b.p, u);
        p.y += station.FloorY;
        palm.transform.localPosition = p;
        palm.transform.localRotation = Quaternion.Euler(0f, 0f, Mathf.Lerp(a.zAngle, b.zAngle, u));
        if (station.solver != null) station.solver.otherTack = a.grip ? station.solver.fistTack : 0f;
    }
}
