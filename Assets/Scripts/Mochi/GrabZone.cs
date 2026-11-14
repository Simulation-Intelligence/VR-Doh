using UnityEngine;

/// <summary>
/// A region that MochiGrabber can grab. Grabbing it moves <see cref="moveTarget"/>.
///  - Segment: a capsule around a line in this transform's space (e.g. the kine handle). 6-DoF.
///  - Rim: the lip of a bowl (ring around local +Y). Position + yaw only, so the usu stays upright.
/// </summary>
public class GrabZone : MonoBehaviour
{
    public enum Shape { Segment, Rim }
    public Shape shape = Shape.Segment;
    public Transform moveTarget;
    [Tooltip("Keep the target upright (yaw only).")]
    public bool uprightOnly = false;

    [Header("Segment (local space)")]
    public Vector3 segmentA = Vector3.zero;
    public Vector3 segmentB = Vector3.forward;
    public float segmentRadius = 0.06f;

    [Header("Rim (local space)")]
    public float rimInnerRadius = 0.15f;
    public float rimOuterRadius = 0.21f;
    public float rimY = 0.18f;
    [Tooltip("How far above/below the rim height still counts, m.")]
    public float rimTolerance = 0.06f;

    [Header("Feedback")]
    public Renderer[] highlight;
    public Color hoverTint = new Color(1.25f, 1.2f, 1.0f, 1f);

    public bool Held { get; set; }

    void Start()
    {
        if (highlight == null || highlight.Length == 0) highlight = GetComponentsInChildren<Renderer>(true);
    }
    public event System.Action<GrabZone> Grabbed, Released;
    MaterialPropertyBlock mpb;
    bool hovered;

    /// <summary>Distance from a world point to the zone (&lt;= 0 means inside).</summary>
    public float Distance(Vector3 world)
    {
        Vector3 p = transform.InverseTransformPoint(world);
        if (shape == Shape.Segment)
        {
            Vector3 ab = segmentB - segmentA;
            float t = Mathf.Clamp01(Vector3.Dot(p - segmentA, ab) / Mathf.Max(ab.sqrMagnitude, 1e-6f));
            return (p - (segmentA + t * ab)).magnitude - segmentRadius;
        }
        float r = new Vector2(p.x, p.z).magnitude;
        float dr = r < rimInnerRadius ? rimInnerRadius - r : (r > rimOuterRadius ? r - rimOuterRadius : 0f);
        float dy = Mathf.Max(Mathf.Abs(p.y - rimY) - rimTolerance, 0f);
        return Mathf.Sqrt(dr * dr + dy * dy) - 0.0001f;
    }

    public void SetHover(bool on)
    {
        if (on == hovered || highlight == null) return;
        hovered = on;
        mpb ??= new MaterialPropertyBlock();
        foreach (var r in highlight)
        {
            if (r == null) continue;
            if (on)
            {
                var c = r.sharedMaterial != null && r.sharedMaterial.HasProperty("_Color") ? r.sharedMaterial.color : Color.white;
                mpb.SetColor("_Color", new Color(c.r * hoverTint.r, c.g * hoverTint.g, c.b * hoverTint.b, c.a));
                r.SetPropertyBlock(mpb);
            }
            else r.SetPropertyBlock(null);
        }
    }

    public void NotifyGrabbed() { Held = true; Grabbed?.Invoke(this); }
    public void NotifyReleased() { Held = false; Released?.Invoke(this); }

    void OnDrawGizmosSelected()
    {
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = new Color(0.2f, 0.8f, 1f, 0.7f);
        if (shape == Shape.Segment)
        {
            Gizmos.DrawWireSphere(segmentA, segmentRadius);
            Gizmos.DrawWireSphere(segmentB, segmentRadius);
            Gizmos.DrawLine(segmentA, segmentB);
        }
        else
        {
            for (int i = 0; i < 32; i++)
            {
                float a0 = i * Mathf.PI / 16f, a1 = (i + 1) * Mathf.PI / 16f;
                foreach (float r in new[] { rimInnerRadius, rimOuterRadius })
                    Gizmos.DrawLine(new Vector3(Mathf.Cos(a0) * r, rimY, Mathf.Sin(a0) * r), new Vector3(Mathf.Cos(a1) * r, rimY, Mathf.Sin(a1) * r));
            }
        }
    }
}
