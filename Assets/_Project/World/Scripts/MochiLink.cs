using System;
using System.Reflection;
using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// Read-only bridge to the team's dough simulation (MochiPound). The World assembly cannot reference
    /// Assembly-CSharp, so the solver, kine and usu are found at runtime by type name:
    /// MochiSolver (IMochiSim: RequestParticleReadback / ReadParticles / ParticleCount / ResetDough),
    /// KineTool (origin = striking face centre) and UsuTool (origin = usu bottom). Nothing is written to them,
    /// except <see cref="ResetDough"/> and hiding the dough renderer during the ending.
    /// Particle positions come back asynchronously (a few frames late) and are converted to world space.
    /// </summary>
    [DefaultExecutionOrder(-40), DisallowMultipleComponent]
    public class MochiLink : MonoBehaviour
    {
        public string solverType = "MochiSolver";
        public string kineType = "KineTool";
        public string usuType = "UsuTool";
        [Tooltip("Particle readback every N frames (each one copies the particle buffer from the GPU).")]
        [Min(1)] public int readbackInterval = 2;

        public Component Solver { get; private set; }
        public Transform Kine { get; private set; }
        public Transform Usu { get; private set; }
        public Renderer DoughRenderer { get; private set; }

        /// <summary>Usu geometry (from UsuTool's fields, defaults if missing).</summary>
        public float UsuBase { get; private set; } = 0.03f;
        public float UsuInnerTop { get; private set; } = 0.15f;
        public float UsuDepth { get; private set; } = 0.13f;
        public float DoughFloorY => Usu != null ? Usu.position.y + UsuBase : transform.position.y;

        /// <summary>Latest particle positions in world space (length = ParticleCount), or null before the first readback.</summary>
        public Vector3[] Particles { get; private set; }
        public int ParticleCount { get; private set; }
        /// <summary>Increments whenever <see cref="Particles"/> got new data.</summary>
        public int Version { get; private set; }
        public Vector3 Centroid { get; private set; }
        public float TopY { get; private set; }
        public bool HasDough => Particles != null && ParticleCount > 0;

        Func<bool> request;
        Func<float[], bool> read;
        Func<int> count;
        MethodInfo resetDough;
        float[] buffer, heights;
        float nextSearch;
        int frame;

        void Update()
        {
            if (Solver == null || Kine == null || Usu == null)
            {
                if (Time.unscaledTime >= nextSearch) { nextSearch = Time.unscaledTime + 1f; Find(); }
                if (Solver == null) return;
            }
            if (DoughRenderer == null)
            {
                var t = Solver.transform.Find("MochiSurface");
                if (t != null) DoughRenderer = t.GetComponent<Renderer>();
            }

            WatchKineRelease();
            int n = count != null ? count() : 0;
            if (n <= 0) return;
            if (buffer == null || buffer.Length != n * 3)
            {
                buffer = new float[n * 3];
                Particles = null;
            }
            if (read != null && read(buffer)) Ingest(n);
            if (request != null && ++frame % readbackInterval == 0) request();
        }

        void Ingest(int n)
        {
            var m = Solver.transform.localToWorldMatrix;
            if (Particles == null || Particles.Length != n) Particles = new Vector3[n];
            Vector3 sum = Vector3.zero; float top = float.NegativeInfinity; int valid = 0;
            for (int i = 0; i < n; i++)
            {
                var p = m.MultiplyPoint3x4(new Vector3(buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2]));
                if (float.IsNaN(p.x) || float.IsNaN(p.y) || float.IsNaN(p.z)) { Particles[i] = Particles[Mathf.Max(0, i - 1)]; continue; }
                Particles[i] = p;
                sum += p; valid++;
                if (p.y > top) top = p.y;
            }
            if (valid == 0) return;
            ParticleCount = n;
            Centroid = sum / valid;
            // Height of the dough's top: the 97th percentile of a subsample, so threads of dough stuck to the
            // lifted kine (tack) do not count as the top.
            int step = Mathf.Max(1, n / 1500), k = 0;
            if (heights == null || heights.Length < n / step + 1) heights = new float[n / step + 1];
            for (int i = 0; i < n; i += step) heights[k++] = Particles[i].y;
            System.Array.Sort(heights, 0, k);
            TopY = k > 0 ? heights[Mathf.Clamp((int)(k * 0.97f), 0, k - 1)] : top;
            BuildDensity(n);
            Version++;
        }

        // ---- coarse occupancy grid around the dough (1.5 cm cells): lets overlays tell the dough body from stray crumbs

        const float Cell = 0.015f;
        const int Dim = 36;
        int[] grid;
        Vector3 gridOrigin;

        void BuildDensity(int n)
        {
            if (grid == null) grid = new int[Dim * Dim * Dim];
            System.Array.Clear(grid, 0, grid.Length);
            gridOrigin = Centroid - Vector3.one * (Cell * Dim * 0.5f);
            for (int i = 0; i < n; i++)
            {
                var q = (Particles[i] - gridOrigin) / Cell;
                int x = (int)q.x, y = (int)q.y, z = (int)q.z;
                if (x < 0 || y < 0 || z < 0 || x >= Dim || y >= Dim || z >= Dim) continue;
                grid[(x * Dim + y) * Dim + z]++;
            }
        }

        /// <summary>Particles in the 3x3x3 cells (4.5 cm cube) around a world point; the dough body gives dozens, crumbs a few.</summary>
        public int DensityAt(Vector3 p)
        {
            if (grid == null) return 0;
            var q = (p - gridOrigin) / Cell;
            int cx = (int)q.x, cy = (int)q.y, cz = (int)q.z, sum = 0;
            for (int x = cx - 1; x <= cx + 1; x++)
            for (int y = cy - 1; y <= cy + 1; y++)
            for (int z = cz - 1; z <= cz + 1; z++)
                if (x >= 0 && y >= 0 && z >= 0 && x < Dim && y < Dim && z < Dim) sum += grid[(x * Dim + y) * Dim + z];
            return sum;
        }

        void Find()
        {
            foreach (var mb in FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            {
                var tn = mb.GetType().Name;
                if (Solver == null && tn == solverType) Bind(mb);
                else if (Kine == null && tn == kineType) Kine = mb.transform;
                else if (Usu == null && tn == usuType)
                {
                    Usu = mb.transform;
                    UsuBase = GetFloat(mb, "baseThickness", UsuBase);
                    UsuInnerTop = GetFloat(mb, "innerRadiusTop", UsuInnerTop);
                    UsuDepth = GetFloat(mb, "depth", UsuDepth);
                }
            }
        }

        void Bind(Component solver)
        {
            Solver = solver;
            var t = solver.GetType();
            const BindingFlags F = BindingFlags.Public | BindingFlags.Instance;
            try
            {
                var mReq = t.GetMethod("RequestParticleReadback", F, null, Type.EmptyTypes, null);
                var mRead = t.GetMethod("ReadParticles", F, null, new[] { typeof(float[]) }, null);
                var pCount = t.GetProperty("ParticleCount", F);
                if (mReq != null) request = (Func<bool>)Delegate.CreateDelegate(typeof(Func<bool>), solver, mReq);
                if (mRead != null) read = (Func<float[], bool>)Delegate.CreateDelegate(typeof(Func<float[], bool>), solver, mRead);
                if (pCount != null) count = (Func<int>)Delegate.CreateDelegate(typeof(Func<int>), solver, pCount.GetGetMethod());
                resetDough = t.GetMethod("ResetDough", F, null, Type.EmptyTypes, null);
            }
            catch (Exception e)
            {
                Debug.LogWarning("[Moon Rabbit] MochiLink could not bind to " + t.Name + ": " + e.Message);
            }
        }

        static float GetFloat(object o, string field, float fallback)
        {
            var f = o.GetType().GetField(field, BindingFlags.Public | BindingFlags.Instance);
            return f != null && f.FieldType == typeof(float) ? (float)f.GetValue(o) : fallback;
        }

        /// <summary>Puts the dough back as a fresh mound (MochiSolver.ResetDough).</summary>
        public void ResetDough()
        {
            if (Solver != null && resetDough != null) resetDough.Invoke(Solver, null);
            Particles = null;
            buffer = null;
        }

        // ---- kine rest (KineDriver.Rest(): standing on its face beside the usu, station space)

        Component kineDriver, kineGrab;
        PropertyInfo grabHeld;
        bool wasHeld;

        [Tooltip("When the kine is let go below the usu's base (inside the pedestal or the ground) or this far from the usu (m), " +
                 "it is put back to rest so nobody starts with a buried kine.")]
        public float kineLostDistance = 1.5f;

        void BindKine()
        {
            if (Kine == null || kineDriver != null) return;
            foreach (var c in Kine.GetComponentsInChildren<Component>(true)) // driver and grab zone may sit on a child
            {
                var tn = c.GetType().Name;
                if (tn == "KineDriver") kineDriver = c;
                else if (tn == "GrabZone") { kineGrab = c; grabHeld = c.GetType().GetProperty("Held", BindingFlags.Public | BindingFlags.Instance); }
            }
        }

        bool KineHeld => kineGrab != null && grabHeld != null && (bool)grabHeld.GetValue(kineGrab);

        /// <summary>Puts the kine back to rest beside the usu (KineDriver.Rest(), External mode) unless someone holds it.</summary>
        public bool RestKine()
        {
            BindKine();
            if (kineDriver == null || KineHeld) return false;
            var t = kineDriver.GetType();
            var mode = t.GetField("mode");
            if (mode != null && mode.FieldType.IsEnum) mode.SetValue(kineDriver, Enum.Parse(mode.FieldType, "External"));
            var rest = t.GetMethod("Rest", BindingFlags.Public | BindingFlags.Instance, null, Type.EmptyTypes, null);
            if (rest == null) return false;
            rest.Invoke(kineDriver, null);
            return true;
        }

        void WatchKineRelease()
        {
            BindKine();
            if (kineGrab == null || Usu == null) return;
            bool held = KineHeld;
            if (wasHeld && !held)
            {
                // let go: if the face ended up under the usu's base or far away, stand it back up beside the usu
                var face = Kine.position;
                var baseY = Usu.position.y;
                var flat = new Vector2(face.x - Usu.position.x, face.z - Usu.position.z);
                if (face.y < baseY - 0.02f || flat.magnitude > kineLostDistance) RestKine();
            }
            wasHeld = held;
        }

        public void SetDoughVisible(bool visible)
        {
            if (DoughRenderer != null) DoughRenderer.enabled = visible;
        }
    }
}
