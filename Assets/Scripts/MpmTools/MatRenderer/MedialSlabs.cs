using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MedialSlabs : MonoBehaviour
{
    private Mesh mesh;
    public MATCoordinator matCoordinator;
    public GameObject slabPrefab;

    void Start()
    {
        
    }
    void Update()
    {
        
    }
    public void GenerateMedialSlabs()
    {
        matCoordinator = GetComponentInParent<MATCoordinator>();
        if (matCoordinator == null)
        {
            Debug.LogError("Impossibile trovare il componente MATCoordinator nell'oggetto corrente o nei genitori.");
            return;
        }
        int degeneratedSlabs = 0;
        List<Vector3> slabVerts = new List<Vector3>();
        List<int> slabFaces = new List<int>();
        for (int i = 0; i < matCoordinator.Triangles.Length; i += 3)
        {
            int i1 = matCoordinator.Triangles[i];
            Vector3 v1 = matCoordinator.Vertices[i1];
            float r1 = matCoordinator.Radii[i1];
            int i2 = matCoordinator.Triangles[i + 1];
            Vector3 v2 = matCoordinator.Vertices[i2];
            float r2 = matCoordinator.Radii[i2];
            int i3 = matCoordinator.Triangles[i + 2];
            Vector3 v3 = matCoordinator.Vertices[i3];
            float r3 = matCoordinator.Radii[i3];

            bool result = MATUtils.GenerateSlab(v1, r1, v2, r2, v3, r3, out slabVerts, out slabFaces);
            if (!result)
            {
                degeneratedSlabs++;
            }
            else
            {
                mesh = new Mesh
                {
                    name = "Slab Mesh",
                    vertices = slabVerts.ToArray(),
                    triangles = slabFaces.ToArray()
                };
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                Vector3[] normals = mesh.normals;
                for (int j = 0; j < normals.Length; j++)
                {
                    normals[j] = -normals[j];
                }
                mesh.normals = normals;
                GameObject newSlab = Instantiate(slabPrefab, transform);
                newSlab.GetComponent<MeshFilter>().mesh = mesh;
            }
        }
        Debug.LogFormat("Degenerated Slabs: {0}", degeneratedSlabs);
        
    }
}
