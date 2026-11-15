using UnityEngine;

/// <summary>
/// The usu (mortar) as both a collider and a visual. Local origin is where the usu stands on the table.
/// The inner bowl is approximated by stacked rings of capsules; the floor is the MPM domain floor,
/// which MochiStation places at <see cref="baseThickness"/> above the table.
/// </summary>
public class UsuTool : PrimitiveTool
{
    public float baseThickness = 0.03f;
    [Tooltip("Inner radius of the bowl at the floor, m.")]
    public float innerRadiusBottom = 0.125f;
    [Tooltip("Inner radius of the bowl at the rim, m.")]
    public float innerRadiusTop = 0.15f;
    [Tooltip("Bowl depth from floor to rim, m.")]
    public float depth = 0.13f;
    public float outerRadius = 0.205f;
    public int segments = 16;
    public int rings = 4;
    [Tooltip("Thickness of the collision wall, m.")]
    public float wallRadius = 0.025f;
    public Material woodMaterial;
    public bool buildVisual = true;

    protected override void Build()
    {
        for (int ring = 0; ring < rings; ring++)
        {
            float f = rings == 1 ? 0f : ring / (float)(rings - 1);
            float y = baseThickness + wallRadius * 0.6f + f * (depth - wallRadius * 0.6f);
            float rInner = Mathf.Lerp(innerRadiusBottom, innerRadiusTop, f);
            float rc = rInner + wallRadius; // capsule centre line sits one wall-radius outside the inner surface
            for (int i = 0; i < segments; i++)
            {
                float a0 = i * 2f * Mathf.PI / segments, a1 = (i + 1) * 2f * Mathf.PI / segments;
                Capsule(new Vector3(Mathf.Cos(a0) * rc, y, Mathf.Sin(a0) * rc), wallRadius,
                        new Vector3(Mathf.Cos(a1) * rc, y, Mathf.Sin(a1) * rc), wallRadius);
            }
        }
    }

    protected override void Awake()
    {
        base.Awake();
        if (buildVisual && Application.isPlaying) BuildVisual();
    }

    [ContextMenu("Build Visual")]
    public void BuildVisual()
    {
        var existing = transform.Find("UsuVisual");
        if (existing != null) Destroy(existing.gameObject);
        var go = new GameObject("UsuVisual");
        go.transform.SetParent(transform, false);
        go.AddComponent<MeshFilter>().sharedMesh = BuildLatheMesh();
        var mr = go.AddComponent<MeshRenderer>();
        mr.sharedMaterial = woodMaterial;
    }

    /// <summary>Revolves the usu cross-section (outer wall, rim, inner bowl, floor) around +Y.</summary>
    public Mesh BuildLatheMesh(int around = 48)
    {
        float floorY = baseThickness, rimY = baseThickness + depth + 0.02f;
        // Profile from the outer base, up the outside, over the rim, down the bowl, to the centre of the floor.
        Vector2[] prof =
        {
            new(0f, 0f), new(outerRadius * 0.96f, 0f), new(outerRadius, 0.015f),
            new(outerRadius, rimY - 0.01f), new(outerRadius - 0.01f, rimY),
            new(innerRadiusTop + 0.01f, rimY), new(innerRadiusTop, rimY - 0.01f),
            new(Mathf.Lerp(innerRadiusBottom, innerRadiusTop, 0.5f), floorY + depth * 0.5f),
            new(innerRadiusBottom, floorY + 0.015f), new(innerRadiusBottom - 0.02f, floorY), new(0f, floorY),
        };
        var verts = new System.Collections.Generic.List<Vector3>();
        var tris = new System.Collections.Generic.List<int>();
        int n = prof.Length;
        for (int j = 0; j <= around; j++)
        {
            float a = j * 2f * Mathf.PI / around;
            float c = Mathf.Cos(a), s = Mathf.Sin(a);
            foreach (var p in prof) verts.Add(new Vector3(p.x * c, p.y, p.x * s));
        }
        for (int j = 0; j < around; j++)
            for (int i = 0; i < n - 1; i++)
            {
                int a = j * n + i, b = (j + 1) * n + i;
                tris.Add(a); tris.Add(a + 1); tris.Add(b);
                tris.Add(b); tris.Add(a + 1); tris.Add(b + 1);
            }
        // The built-in Standard shader culls back faces, so add a reversed copy to make the shell two-sided.
        int vc = verts.Count, tc = tris.Count;
        for (int i = 0; i < vc; i++) verts.Add(verts[i]);
        for (int i = 0; i < tc; i += 3) { tris.Add(tris[i] + vc); tris.Add(tris[i + 2] + vc); tris.Add(tris[i + 1] + vc); }
        var m = new Mesh { name = "Usu" };
        m.SetVertices(verts);
        m.SetTriangles(tris, 0);
        m.RecalculateNormals();
        m.RecalculateBounds();
        return m;
    }
}
