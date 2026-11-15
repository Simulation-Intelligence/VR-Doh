using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// A coarse height map of the dough over the usu, in station space (metres, +Y up), built from
    /// <see cref="IMochiSim"/>'s particle readback (asynchronous on the native solver, so the simulation
    /// never waits). Each tick reads what the previous request returned, then asks for the next.
    /// Works with either solver: positions go through the sim transform, so the Taichi unit cube and the
    /// native solver's metres both come out in station space.
    /// </summary>
    public class DoughHeight
    {
        readonly MochiStation station;
        readonly Vector2 centre; // usu axis in station space (x, z)
        readonly float half, cell;
        readonly int n;
        readonly float[] top;
        float[] positions = new float[0];
        float nextAt;
        bool requested;

        public bool Valid { get; private set; }

        public DoughHeight(MochiStation station, Vector2 centre, float halfWidth, float cellSize = 0.012f)
        {
            this.station = station;
            this.centre = centre;
            half = halfWidth;
            cell = cellSize;
            n = Mathf.Max(1, Mathf.CeilToInt(2f * halfWidth / cellSize));
            top = new float[n * n];
        }

        public void Tick(float now, float interval)
        {
            var sim = station != null ? station.Sim : null;
            if (sim == null || now < nextAt) return;
            nextAt = now + interval;
            if (requested) Build(sim);
            requested = sim.RequestParticleReadback();
        }

        void Build(IMochiSim sim)
        {
            int count = sim.ParticleCount;
            if (count <= 0) return;
            if (positions.Length != count * 3) positions = new float[count * 3];
            if (!sim.ReadParticles(positions)) return;
            for (int i = 0; i < top.Length; i++) top[i] = float.NegativeInfinity;
            Matrix4x4 m = station.transform.worldToLocalMatrix * sim.transform.localToWorldMatrix;
            for (int i = 0; i < count; i++)
            {
                Vector3 p = m.MultiplyPoint3x4(new Vector3(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]));
                if (float.IsNaN(p.x) || float.IsNaN(p.y) || float.IsNaN(p.z)) continue;
                float fx = (p.x - centre.x + half) / cell, fz = (p.z - centre.y + half) / cell; // centre.y holds the axis z
                if (fx < 0f || fz < 0f || fx >= n || fz >= n) continue;
                int k = (int)fz * n + (int)fx;
                if (p.y > top[k]) top[k] = p.y;
            }
            Valid = true;
        }

        /// <summary>
        /// Highest particle (station-space y) under a disc of the given radius around (x, z), or
        /// negative infinity where there is no dough (or no reading yet).
        /// </summary>
        public float TopUnder(float x, float z, float radius)
        {
            if (!Valid) return float.NegativeInfinity;
            float best = float.NegativeInfinity;
            float fx = (x - centre.x + half) / cell, fz = (z - centre.y + half) / cell;
            int reach = Mathf.CeilToInt(radius / cell) + 1;
            int cx = Mathf.FloorToInt(fx), cz = Mathf.FloorToInt(fz);
            float r2 = (radius + 0.7f * cell) * (radius + 0.7f * cell);
            for (int iz = cz - reach; iz <= cz + reach; iz++)
            {
                if (iz < 0 || iz >= n) continue;
                for (int ix = cx - reach; ix <= cx + reach; ix++)
                {
                    if (ix < 0 || ix >= n) continue;
                    float dx = (ix + 0.5f) * cell - half + centre.x - x, dz = (iz + 0.5f) * cell - half + centre.y - z;
                    if (dx * dx + dz * dz > r2) continue;
                    float t = top[iz * n + ix];
                    if (t > best) best = t;
                }
            }
            return best;
        }
    }
}
