
using UnityEngine;

public class MpmCapsule : MpmTool
{
    public float radius = 0.02f;
    public float length = 0.5f;
    void Awake()
    {
        // Set the valus of numCapsules and init_capsules in Awake() method
        numCapsules = 1;
        init_capsules = new Capsule[numCapsules];

        // Initialize positions and radius for each Capsule
        init_capsules[0].start = new Vector3(0, 0, 0);
        init_capsules[0].end = new Vector3(length, 0, 0);
        init_capsules[0].radius = radius;
    }
}