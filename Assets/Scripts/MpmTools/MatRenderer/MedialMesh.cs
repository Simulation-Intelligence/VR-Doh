using System;
using System.Collections.Generic;
using Unity.Burst.Intrinsics;
using UnityEngine;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MedialMesh : MonoBehaviour
{
    private Mesh mesh;
    public MATCoordinator matCoordinator;
    public GameObject linePrefab;

    void Start()
    {
        
    }
    public void GenerateMedialMesh()
    {
        // Trova il componente MATCoordinator nell'oggetto corrente o genitore
        matCoordinator = GetComponentInParent<MATCoordinator>();

        if (matCoordinator == null)
        {
            Debug.LogError("Impossibile trovare il componente MATCoordinator nell'oggetto corrente o nei genitori.");
            return;
        }

        mesh = new Mesh
        {
            name = "Medial Mesh",
            vertices = matCoordinator.Vertices,
            triangles = matCoordinator.Triangles
        };
        AddLines();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();

        GetComponent<MeshFilter>().mesh = mesh;
    }
    private void AddLines()
    {
        for (int i = 0; i < matCoordinator.Edges.Length; i += 2)
        {
            GameObject newLine = Instantiate(linePrefab, transform);
            // Trova il componente LineRenderer
            LineRenderer lineRenderer = newLine.GetComponent<LineRenderer>();

            // Assegna le posizioni al LineRenderer
            lineRenderer.positionCount = 2;
            Vector3[] linePositions = {matCoordinator.Vertices[matCoordinator.Edges[i]], matCoordinator.Vertices[matCoordinator.Edges[i + 1]]};
            lineRenderer.SetPositions(linePositions);
        }
    }
}
