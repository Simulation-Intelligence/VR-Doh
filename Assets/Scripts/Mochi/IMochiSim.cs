using UnityEngine;

/// <summary>
/// What gameplay, the kine driver and the test bench need from a dough simulation, so they work
/// with either the Taichi solver (Mpm3DMarching, PC over Link) or the native one (MochiSolver, standalone).
/// Particle positions are in the local space of <see cref="transform"/>.
/// </summary>
public interface IMochiSim
{
    Transform transform { get; }
    int ParticleCount { get; }
    float GridDx { get; }
    double SimTime { get; }
    float LastFrameSimTime { get; }
    int LastSubstepCount { get; }
    bool RequestParticleReadback();
    bool ReadParticles(float[] positions);
}
