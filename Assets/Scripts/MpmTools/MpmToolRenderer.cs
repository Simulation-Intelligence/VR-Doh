using UnityEngine;

[RequireComponent(typeof(MpmTool))]
public class ToolRender : MonoBehaviour
{
    public MpmTool mpmTool; // Reference to the MpmTool component
    public Material material; // Material used for rendering
    
    // Array to store the capsule assemblies (each consisting of two spheres and a cylinder)
    private GameObject[] capsuleObjects;

    void Start()
    {
        // Get the MpmTool component
        if (mpmTool == null)
        {
            mpmTool = GetComponent<MpmTool>();
        }

        // Initialize the capsule assembly array
        capsuleObjects = new GameObject[mpmTool.numCapsules];

        // Create child objects for each capsule
        for (int i = 0; i < mpmTool.numCapsules; i++)
        {
            GameObject capsule = new GameObject("Capsule_" + i);
            capsule.transform.SetParent(transform); // Set child gameObject
            
            // Create two spheres
            GameObject sphere1 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere1.transform.SetParent(capsule.transform);
            sphere1.GetComponent<Renderer>().material = material; 

            GameObject sphere2 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere2.transform.SetParent(capsule.transform);
            sphere2.GetComponent<Renderer>().material = material; 

            // Create a cylinder
            GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cylinder.transform.SetParent(capsule.transform);
            cylinder.GetComponent<Renderer>().material = material;
            
            capsuleObjects[i] = capsule;
        }
    }
    void Update()
    {
        if (mpmTool == null || capsuleObjects == null)
        {
            return;
        }

        // Update each capsule's position and size
        for (int i = 0; i < mpmTool.numCapsules; i++)
        {
            MpmTool.Capsule capsule = mpmTool.capsules[i];
            UpdateCapsuleObject(capsuleObjects[i], capsule.start, capsule.end, capsule.radius);
        }
    }
    
    // Update the capsule assembly object
    void UpdateCapsuleObject(GameObject capsuleObject, Vector3 start, Vector3 end, float radius)
    {
        // Calculate the midpoint and direction
        Vector3 center = (start + end) / 2.0f;
        Vector3 direction = (end - start).normalized;
        float height = (end - start).magnitude;

        // Update two spheres
        Transform sphere1 = capsuleObject.transform.GetChild(0);
        Transform sphere2 = capsuleObject.transform.GetChild(1);
        sphere1.position = start;
        sphere2.position = end;
        sphere1.localScale = sphere2.localScale = Vector3.one * radius * 2;

        // Update the cylinder
        Transform cylinder = capsuleObject.transform.GetChild(2);
        cylinder.position = center;
        cylinder.up = direction; // Set direction
        // Note: In Unity, the cylinder height is along the y-axis
        cylinder.localScale = new Vector3(radius * 2, height / 2, radius * 2); 
    }
}
