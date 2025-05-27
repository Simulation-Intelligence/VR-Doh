using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MedialCones : MonoBehaviour
{
    private Mesh mesh;
    public MATCoordinator matCoordinator;
    public GameObject conePrefab; 

    void Start()
    {
        
    }
    void Update()
    {
        
    }
    public void GenerateMedialCones()
    {
        matCoordinator = GetComponentInParent<MATCoordinator>();
        if (matCoordinator == null)
        {
            Debug.LogError("Impossibile trovare il componente MATCoordinator nell'oggetto corrente o nei genitori.");
            return;
        }
        List<Vector3> coneVerts = new List<Vector3>();
        List<int> coneFaces = new List<int>();
        for (int i = 0; i < matCoordinator.Edges.Length; i += 2)
        {
            int i1 = matCoordinator.Edges[i];
            Vector3 v1 = matCoordinator.Vertices[i1];
            float r1 = matCoordinator.Radii[i1];
            int i2 = matCoordinator.Edges[i + 1];
            Vector3 v2 = matCoordinator.Vertices[i2];
            float r2 = matCoordinator.Radii[i2];
            MATUtils.GenerateConicalSurface(v1, r1, v2, r2, 32, out coneVerts, out coneFaces);
            mesh = new Mesh
                {
                    name = "Slab Mesh",
                    vertices = coneVerts.ToArray(),
                    triangles = coneFaces.ToArray()
                };
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                Vector3[] normals = mesh.normals;
                for (int j = 0; j < normals.Length; j++)
                {
                    normals[j] = -normals[j];
                }
                mesh.normals = normals;
                GameObject newSlab = Instantiate(conePrefab, transform);
                newSlab.GetComponent<MeshFilter>().mesh = mesh;
        }
    }
}
