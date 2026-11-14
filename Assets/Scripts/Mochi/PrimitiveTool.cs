using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// A rigid MPM collision tool built from medial primitives in code, in this transform's local space (metres).
/// Cone/capsule: two spheres, third radius 0. Slab: three spheres (a rounded triangle).
/// Keep the transform's scale at 1: VR-Doh's MatTool does not scale radii for fixed primitives.
/// Runs before Mpm3DMarching so the simulation sees this frame's pose.
/// </summary>
[DefaultExecutionOrder(-50)]
public abstract class PrimitiveTool : MatTool
{
    protected readonly List<Primitive> build = new();

    protected abstract void Build();

    protected virtual void Awake()
    {
        build.Clear();
        Build();
        init_primitives = build.ToArray();
        numPrimitives = init_primitives.Length;
        primitives = new Primitive[numPrimitives];
        for (int i = 0; i < numPrimitives; i++)
        {
            primitives[i] = init_primitives[i];
            primitives[i].sphere1 = transform.TransformPoint(init_primitives[i].sphere1);
            primitives[i].sphere2 = transform.TransformPoint(init_primitives[i].sphere2);
            primitives[i].sphere3 = transform.TransformPoint(init_primitives[i].sphere3);
        }
    }

    protected void Capsule(Vector3 a, float ra, Vector3 b, float rb)
    {
        build.Add(new Primitive { sphere1 = a, radii1 = ra, sphere2 = b, radii2 = rb, sphere3 = b, radii3 = 0f });
    }

    protected void Slab(Vector3 a, Vector3 b, Vector3 c, float r)
    {
        build.Add(new Primitive { sphere1 = a, radii1 = r, sphere2 = b, radii2 = r, sphere3 = c, radii3 = r });
    }

    /// <summary>A flat rounded rectangle of thickness 2r in the local XZ plane, centred at y=r.</summary>
    protected void FlatRect(float halfX, float halfZ, float r, float y = -1f)
    {
        if (y < 0f) y = r;
        float x = Mathf.Max(halfX - r, 1e-4f), z = Mathf.Max(halfZ - r, 1e-4f);
        Slab(new Vector3(-x, y, -z), new Vector3(x, y, -z), new Vector3(x, y, z), r);
        Slab(new Vector3(-x, y, -z), new Vector3(x, y, z), new Vector3(-x, y, z), r);
    }

    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) { build.Clear(); Build(); }
        var src = Application.isPlaying ? init_primitives : build.ToArray();
        if (src == null) return;
        Gizmos.color = new Color(1f, 0.6f, 0.1f, 0.6f);
        foreach (var p in src)
        {
            Gizmos.DrawWireSphere(transform.TransformPoint(p.sphere1), p.radii1);
            Gizmos.DrawWireSphere(transform.TransformPoint(p.sphere2), p.radii2);
            if (p.radii3 > 0f) Gizmos.DrawWireSphere(transform.TransformPoint(p.sphere3), p.radii3);
        }
    }
}
