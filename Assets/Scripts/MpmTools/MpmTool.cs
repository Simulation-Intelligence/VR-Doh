using UnityEngine;

public class MpmTool : MonoBehaviour
{
    public struct Capsule
    {
        public Vector3 start;
        public Vector3 end;
        public float radius;
    }
    public int numCapsules;
    protected Capsule[] init_capsules;
    public Capsule[] capsules;

    void Start()
    {
        capsules = new Capsule[numCapsules];
        for (int i = 0; i < numCapsules; i++)
        {
            capsules[i] = init_capsules[i];
        }
    }
    void Update()
    {
        UpdateCapsules();
    }
    // Virutal method to be overriden by child classes
    protected virtual void UpdateCapsules()
    {
        TransformFixedCapsules();
    }
    void TransformFixedCapsules()
    {
        for (int i = 0; i < numCapsules; i++)
        {
            // Update capsule position
            capsules[i].start = transform.TransformPoint(init_capsules[i].start);
            capsules[i].end = transform.TransformPoint(init_capsules[i].end);
        }
    }
}