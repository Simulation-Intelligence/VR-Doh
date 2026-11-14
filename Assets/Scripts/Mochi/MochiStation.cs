using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Sets up one VR-Doh MPM volume as a lump of freshly pounded mochi sitting in an usu (mortar)
/// on a real table, from physical quantities in SI units.
///
/// The MPM solver works in a unit cube that the Mpm3DMarching transform scales into the world.
/// It does not rescale gravity or material constants with that cube, so this component converts:
///   L = domain edge (m), rho_sim = 1000 (solver's particle density per unit volume)
///   stress_sim = stress_phys * (rho_sim / rho_phys) / L^2   (keeps wave speed and yield strain physical)
///   gravity_sim = -9.81 / L
/// so that the lump falls, sags and moves at real speed in metres and seconds.
///
/// The station's own transform is the contact point between the usu bottom and the table, +Y up.
/// </summary>
[DefaultExecutionOrder(-1000)]
public class MochiStation : MonoBehaviour
{
    public enum ToolSet { TrackedHands, TestBench }

    [Header("Simulation object")]
    public Mpm3DMarching mochi;
    [Tooltip("Native compute-shader solver (standalone Quest). When set, it is used instead of the Taichi solver above.")]
    public MochiSolver solver;
    /// <summary>The active simulation, whichever solver drives it.</summary>
    public IMochiSim Sim => solver != null ? solver : mochi;
    public ToolSet toolSet = ToolSet.TrackedHands;
    [Tooltip("Hand tools used when toolSet is TrackedHands (VR-Doh's MatHand per hand).")]
    public List<MatTool> handTools = new();
    [Tooltip("Scripted tools used when toolSet is TestBench.")]
    public List<MatTool> benchTools = new();
    [Tooltip("Tools that are always present: the kine (mallet) and the usu rim.")]
    public List<MatTool> sharedTools = new();

    [Header("Space")]
    [Tooltip("Edge of the simulated cube in metres. Must contain the usu; larger costs resolution or speed.")]
    public float domainSize = 0.42f;
    [Tooltip("Height of the dough floor above the station origin (the usu's base thickness; 0 for a bare table).")]
    public float floorHeight = 0.03f;
    [Tooltip("MPM grid cells per edge. 64 gives 6.6 mm cells at 0.42 m.")]
    public int gridResolution = 64;
    public int renderResolution = 96;

    [Header("Mochi (physical units)")]
    [Tooltip("Mass of the lump. A home mochitsuki batch is roughly 1-2 kg of cooked rice.")]
    public float massKg = 1.4f;
    [Tooltip("Density of pounded glutinous rice, kg/m^3.")]
    public float density = 1150f;
    [Tooltip("Young's modulus, Pa. Warm fresh mochi is very soft (shear modulus around 10^3-10^4 Pa).")]
    public float youngsModulus = 15000f;
    [Tooltip("Poisson ratio. Mochi is nearly incompressible; higher values cost more substeps.")]
    [Range(0.2f, 0.48f)] public float poissonRatio = 0.4f;
    [Tooltip("Von Mises yield stress, Pa. Below the gravity stress of the lump it slumps flat; well above it, dents spring back.")]
    public float yieldStress = 700f;
    [Tooltip("Velocity decay rate (1/s). Stands in for mochi's high viscosity: quiets jiggling without stopping free fall.")]
    public float viscousDamping = 6f;
    [Tooltip("Coulomb friction against tools.")]
    public float toolFriction = 0.5f;
    [Tooltip("Stickiness to hands and kine (0-1). Kneaders wet their hands, so keep this moderate.")]
    [Range(0f, 1f)] public float stickiness = 0.25f;
    [Tooltip("Mochi sticks to a dry usu; true makes the domain floor and walls no-slip.")]
    public bool stickyUsu = true;
    public bool gravity = true;

    [Header("Performance")]
    [Tooltip("Cap on physics substeps per frame. Above what one frame needs, a slow frame asks for more substeps, which makes the next frame slower (a spiral). At the cap the dough runs in slight slow motion instead.")]
    public int maxSubstepsPerFrame = 16;

    [Header("Placement")]
    [Tooltip("At start, move the station in front of the player's head (the tracking origin may be eye level or floor level, so a fixed world position is unreliable).")]
    public bool placeInFrontOfHead = true;
    [Tooltip("Horizontal distance from the eyes to the usu centre, m.")]
    public float forwardDistance = 0.45f;
    [Tooltip("Height of the eyes above the table, m (about 0.45-0.55 seated, 0.6-0.75 standing).")]
    public float eyeAboveTable = 0.35f;
    public Transform head;
    [Tooltip("Remember where the player last put the usu (by grabbing its rim) and start there next time.")]
    public bool rememberPlacement = true;
    public GrabZone rimZone;
    const string PlacementKey = "MochiStation.Placement";

    [Header("Look")]
    public Material surfaceMaterial;

    public float SimToMetres => domainSize;

    void Awake()
    {
        if (solver == null) solver = GetComponentInChildren<MochiSolver>(true);
        if (solver != null) { ConfigureSolver(); return; }
        if (mochi == null) mochi = GetComponentInChildren<Mpm3DMarching>(true);
        if (mochi == null) { Debug.LogError("MochiStation: no Mpm3DMarching or MochiSolver assigned."); return; }
        Configure();
    }

    System.Collections.IEnumerator Start()
    {
        if (rimZone != null) rimZone.Released += _ => SavePlacement();
        if (rememberPlacement && LoadPlacement()) yield break;
        if (!placeInFrontOfHead) yield break;
        if (head == null) { var cam = Camera.main; if (cam != null) head = cam.transform; }
        if (head == null) yield break;
        // Give tracking a few frames to deliver a real head pose.
        for (int i = 0; i < 10; i++) yield return null;
        PlaceInFrontOf(head);
    }

    public void SavePlacement()
    {
        if (!rememberPlacement) return;
        var p = transform.position; var e = transform.eulerAngles.y;
        PlayerPrefs.SetString(PlacementKey, string.Join(",", new[] { p.x, p.y, p.z, e }.Select(v => v.ToString("R", System.Globalization.CultureInfo.InvariantCulture))));
        PlayerPrefs.Save();
    }

    bool LoadPlacement()
    {
        var v = PlayerPrefs.GetString(PlacementKey, "");
        var f = v.Split(',');
        if (f.Length != 4) return false;
        var n = new float[4];
        for (int i = 0; i < 4; i++) if (!float.TryParse(f[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out n[i])) return false;
        transform.SetPositionAndRotation(new Vector3(n[0], n[1], n[2]), Quaternion.Euler(0f, n[3], 0f));
        return true;
    }

    [ContextMenu("Forget Saved Placement")]
    public void ForgetPlacement() { PlayerPrefs.DeleteKey(PlacementKey); PlayerPrefs.Save(); }

    /// <summary>Puts the usu on a virtual table forwardDistance in front of and eyeAboveTable below the given head pose.</summary>
    public void PlaceInFrontOf(Transform viewer)
    {
        Vector3 fwd = Vector3.ProjectOnPlane(viewer.forward, Vector3.up);
        if (fwd.sqrMagnitude < 1e-4f) fwd = Vector3.forward;
        fwd.Normalize();
        transform.SetPositionAndRotation(viewer.position + fwd * forwardDistance + Vector3.down * eyeAboveTable,
                                         Quaternion.LookRotation(fwd, Vector3.up));
    }

    /// <summary>Places the native solver so its floor is the usu floor and hands it the tools, before its Start().</summary>
    public void ConfigureSolver()
    {
        var t = solver.transform;
        t.SetParent(transform, false);
        t.localRotation = Quaternion.identity;
        t.localScale = Vector3.one;
        Vector3 size = solver.DomainSize;
        t.localPosition = new Vector3(-0.5f * size.x, floorHeight - solver.FloorOffset, -0.5f * size.z);

        solver.tools.Clear();
        foreach (var tool in toolSet == ToolSet.TrackedHands ? handTools : benchTools)
            if (tool != null) solver.tools.Add(tool);
        foreach (var tool in sharedTools)
            if (tool != null) solver.tools.Add(tool);
        if (solver.usu == null) solver.usu = sharedTools.OfType<UsuTool>().FirstOrDefault();
        if (solver.surfaceMaterial == null) solver.surfaceMaterial = surfaceMaterial;
        solver.maxSubsteps = maxSubstepsPerFrame;
        Debug.Log($"MochiStation: native solver, {solver.massKg:F2} kg lump in a {size.x * 100f:F0}x{size.y * 100f:F0}x{size.z * 100f:F0} cm box, dx={solver.cellSize * 1000f:F1} mm");
    }

    /// <summary>Applies everything to the Mpm3DMarching before its Start() allocates buffers.</summary>
    public void Configure()
    {
        float L = domainSize;
        int n = gridResolution;
        float dxSim = 1f / n;

        // Place the cube so the solver's floor (bound cells above the cube bottom) is the station origin.
        var t = mochi.transform;
        t.SetParent(transform, false);
        t.localRotation = Quaternion.identity;
        t.localScale = Vector3.one * L;
        t.localPosition = new Vector3(-0.5f * L, floorHeight - mochi.bound * dxSim * L, -0.5f * L);

        // Geometry: start as a ball dropped into the usu, like steamed rice tipped in.
        float volume = massKg / density;
        float diameter = Mathf.Pow(6f * volume / Mathf.PI, 1f / 3f);
        mochi.initShape = Mpm3DMarching.InitShape.Sphere;
        mochi.cube_size = diameter / L;
        mochi.n_grid = n;
        mochi.render_n_grid = renderResolution;
        mochi.particle_per_grid = 8;
        mochi.renderType = Mpm3DMarching.RenderType.MarchingCubes;
        mochi.use_gaussian_acceleration = false;

        // Material, converted to solver units.
        float rhoSim = 1000f;
        float stressScale = rhoSim / density / (L * L);
        mochi.p_rho = rhoSim;
        mochi.use_unified_material = true;
        mochi.materialType = Mpm3DMarching.MaterialType.Default_Clay;
        mochi.plasticityType = Mpm3DMarching.PlasticityType.Von_Mises;
        mochi.stressType = Mpm3DMarching.StressType.Kirchhoff;
        mochi._E = youngsModulus * stressScale;
        mochi._nu = poissonRatio;
        mochi._SigY = yieldStress * stressScale;
        mochi.damping = viscousDamping;
        mochi.friction_k = toolFriction;
        mochi.adhesion = stickiness;
        mochi.adhesion_band_cells = 1.5f;
        mochi.SetStickyBoundary(stickyUsu);
        mochi.SetGravity(gravity ? -9.81f / L : 0f);

        // Real-time stepping: dt from the elastic wave speed, simulated time follows the wall clock.
        mochi.use_optimized_kernels = true;
        mochi.realtime_stepping = true;
        mochi.time_scale = 1f;
        mochi.max_dt = 2e-3f;
        mochi.allowed_cfl = 0.5f;
        mochi.max_substeps = maxSubstepsPerFrame;
        mochi.use_correct_cfl = false;
        mochi.lerp_tool = false;
        mochi.adjust_particle = true;
        mochi.use_grid_force = false;
        mochi.use_standard_mpm_boundary = true;
        mochi.rotate_speed = 0f;
        mochi.RunSimulation = true;
        mochi.UsePinchGestureLeft = false;
        mochi.UsePinchGestureRight = false;
        mochi.squeeze_particles = false;
        // Simulate the whole cube regardless of where the tools are.
        mochi.SetHandsimulationRadius(2f * L);

        mochi.tools.Clear();
        mochi.matTools.Clear();
        foreach (var tool in toolSet == ToolSet.TrackedHands ? handTools : benchTools)
            if (tool != null) mochi.matTools.Add(tool);
        foreach (var tool in sharedTools)
            if (tool != null) mochi.matTools.Add(tool);

        if (surfaceMaterial != null)
            foreach (var r in mochi.GetComponentsInChildren<MarchingCubes.MarchingCubeVisualizer>(true))
                r.GetComponent<MeshRenderer>().sharedMaterial = surfaceMaterial;

        Debug.Log($"MochiStation: {massKg:F2} kg lump, d={diameter * 100f:F1} cm in a {L * 100f:F0} cm domain, " +
                  $"dx={L / n * 1000f:F1} mm, E_sim={mochi._E:F0}, SigY_sim={mochi._SigY:F0}, g_sim={-9.81f / L:F1}, " +
                  $"P-wave {Mathf.Sqrt(youngsModulus * (1 - poissonRatio) / ((1 + poissonRatio) * (1 - 2 * poissonRatio)) / density):F1} m/s");
    }

    /// <summary>Height of the solver floor in station space, m.</summary>
    public float FloorY => floorHeight;

    /// <summary>Converts a solver-space position (unit cube) to world space.</summary>
    public Vector3 SimToWorld(Vector3 sim) => Sim.transform.TransformPoint(sim);
    /// <summary>Converts a solver-space position to station space (metres, origin at the usu bottom centre).</summary>
    public Vector3 SimToStation(Vector3 sim) => transform.InverseTransformPoint(Sim.transform.TransformPoint(sim));
}
