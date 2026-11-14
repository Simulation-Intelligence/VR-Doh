using UnityEngine;

/// <summary>
/// A flat palm (heel of hand plus palm, no fingers) as an MPM collider, for scripted kneading tests.
/// Local origin at the centre of the palm surface, local -Y is the palm normal (pressing direction),
/// local +Z points toward the fingers.
/// </summary>
public class PalmTool : PrimitiveTool
{
    public float width = 0.085f;
    public float length = 0.10f;
    public float thickness = 0.03f;

    protected override void Build()
    {
        FlatRect(width * 0.5f, length * 0.5f, thickness * 0.5f);
    }
}
