using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// In passthrough the real hands are part of the camera image, so the virtual mochi and usu are
/// drawn over them and a hand reaching into the bowl vanishes. This draws the hand colliders (the
/// same medial spheres the dough collides with) with a cutout material that writes depth and leaves
/// a hole for passthrough, so the real hand shows in front of whatever virtual object is behind it.
/// </summary>
[DefaultExecutionOrder(200)]
public class MochiHandCutout : MonoBehaviour
{
    public MochiSolver solver;
    [Tooltip("Material using the Mochi/PassthroughCutout shader. Loaded from Resources/MochiHandCutout if empty.")]
    public Material cutout;
    [Tooltip("Extra spheres drawn along each bone between its end spheres, to close the gaps.")]
    [Range(0, 4)] public int fill = 2;
    [Tooltip("Scale of the drawn spheres relative to the collider. Slightly under 1 keeps the hole inside the real hand's outline.")]
    public float radiusScale = 0.95f;
    [Tooltip("Also cut out tools that are not tracked hands (for testing in the Editor).")]
    public bool includeOtherTools = false;

    Mesh sphere;
    readonly List<Matrix4x4> batch = new(1023);

    void Start()
    {
        if (solver == null) solver = FindAnyObjectByType<MochiSolver>();
        if (cutout == null) cutout = Resources.Load<Material>("MochiHandCutout");
        if (cutout == null) { Debug.LogWarning("MochiHandCutout: no cutout material."); enabled = false; return; }
        cutout.enableInstancing = true;
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere = go.GetComponent<MeshFilter>().sharedMesh;   // unit diameter
        Destroy(go);
    }

    void LateUpdate()
    {
        if (solver == null || sphere == null) return;
        batch.Clear();
        foreach (var tool in solver.activeTools)
        {
            if (tool == null || tool.primitives == null) continue;
            if (!(tool is MatHand) && !(includeOtherTools && !(tool is KineTool) && !(tool is UsuTool))) continue;
            int count = Mathf.Min(tool.numPrimitives, tool.primitives.Length);
            for (int i = 0; i < count; i++)
            {
                var p = tool.primitives[i];
                bool slab = p.radii3 > 0f;
                Add(p.sphere1, p.radii1); Add(p.sphere2, p.radii2);
                Between(p.sphere1, p.radii1, p.sphere2, p.radii2);
                if (slab)
                {
                    Add(p.sphere3, p.radii3);
                    Between(p.sphere2, p.radii2, p.sphere3, p.radii3);
                    Between(p.sphere3, p.radii3, p.sphere1, p.radii1);
                    Add((p.sphere1 + p.sphere2 + p.sphere3) / 3f, (p.radii1 + p.radii2 + p.radii3) / 3f);
                }
            }
        }
        Flush();
    }

    void Between(Vector3 a, float ra, Vector3 b, float rb)
    {
        for (int k = 1; k <= fill; k++)
        {
            float t = k / (fill + 1f);
            Add(Vector3.Lerp(a, b, t), Mathf.Lerp(ra, rb, t));
        }
    }

    void Add(Vector3 centre, float radius)
    {
        if (radius <= 0f) return;
        batch.Add(Matrix4x4.TRS(centre, Quaternion.identity, Vector3.one * (2f * radius * radiusScale)));
        if (batch.Count == 1023) Flush();
    }

    void Flush()
    {
        if (batch.Count == 0) return;
        Graphics.DrawMeshInstanced(sphere, 0, cutout, batch, null, UnityEngine.Rendering.ShadowCastingMode.Off, false);
        batch.Clear();
    }
}
