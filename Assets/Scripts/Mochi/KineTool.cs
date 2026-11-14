using UnityEngine;

/// <summary>
/// The kine (mochitsuki mallet) head as an MPM collider. Local origin is the centre of the flat
/// striking face, local -Y is the strike direction, the head extends along +Y.
/// </summary>
public class KineTool : PrimitiveTool
{
    [Tooltip("Head radius, m. Traditional kine heads are roughly 10-15 cm across.")]
    public float headRadius = 0.06f;
    [Tooltip("Head length along the strike axis, m.")]
    public float headLength = 0.20f;
    [Tooltip("Rounding radius of the face edge, m. Smaller is flatter but needs a finer grid to resolve.")]
    public float faceRounding = 0.015f;

    protected override void Build()
    {
        // Flat face: two overlapping triangles (a hexagram) of small spheres approximating a disc.
        float r = faceRounding, c = headRadius - r;
        for (int k = 0; k < 2; k++)
        {
            float a0 = k * Mathf.PI / 3f;
            Vector3 P(float a) => new Vector3(Mathf.Cos(a) * c, r, Mathf.Sin(a) * c);
            Slab(P(a0), P(a0 + 2f * Mathf.PI / 3f), P(a0 + 4f * Mathf.PI / 3f), r);
        }
        // Body: a capsule filling the rest of the cylinder, its lower cap hidden behind the face.
        float rb = headRadius * 0.92f;
        Capsule(new Vector3(0, rb + r * 0.5f, 0), rb, new Vector3(0, Mathf.Max(headLength - rb, rb + r), 0), rb);
    }
}
