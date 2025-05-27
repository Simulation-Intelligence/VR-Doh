using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class MATCoordinator : MonoBehaviour
{
    #region PrivateVariables
    private Vector3[] vertices = null;
    private int[] triangles = null;
    private int[] edges = null;
    private float[] radii = null;
    #endregion

    #region Properties
    [SerializeField]
    private string filePath = "E:/VR Project/Tachi_mpm/Taichi-UnityExample/scripts/data/hand_mat_data/hand_ma_v_53_e_78_f_26.ma";
    public MedialMesh medialMesh;
    public MedialSpheres medialSpheres;
    public MedialSlabs medialSlabs;
    public MedialCones medialCones;
    public Vector3[] Vertices
    {
        get { return vertices; }
        set
        {
            vertices = value;
        }
    }
    public int[] Triangles
    {
        get { return triangles; }
        set
        {
            triangles = value;
        }
    }
    public int[] Edges
    {
        get { return edges; }
        set
        {
            edges = value;
        }
    }
    public float[] Radii
    {
        get { return radii; }
        set
        {
            radii = value;
        }
    }
    #endregion

    #region MonoBehaviour
    void Start()
    {
        ReadMaFile();
        if (vertices.Length > 0)
        {
            if (medialMesh != null)
            {
                medialMesh.GenerateMedialMesh();
            }
            if (medialSpheres != null)
            {
                medialSpheres.GenerateMedialSpheres();
            }
            if (medialSlabs != null && triangles.Length > 0)
            {
                medialSlabs.GenerateMedialSlabs();
            }
            if (medialCones != null)
            {
                medialCones.GenerateMedialCones();
            }
        }
    }

    void Update()
    {
        
    }
    #endregion
    
    #region Methods
    public void ReadMaFile()
    {
        string[] lines = File.ReadAllLines(filePath);
        
        if (lines.Length < 1)
        {
            throw new ArgumentException("Il file .ma è vuoto.");
        }

        // Leggi la prima riga per ottenere il numero di vertici, linee e triangoli
        string[] counts = lines[0].Split(' ');
        int numVertices = int.Parse(counts[0]);
        int numEdges = int.Parse(counts[1]);
        int numTriangles = int.Parse(counts[2]);

        vertices = new Vector3[numVertices];
        triangles = new int[numTriangles * 3]; // Ogni triangolo ha 3 vertici
        edges = new int[numEdges * 2]; // Ogni linea ha 2 vertici
        radii = new float[numVertices];

        int vertexIndex = 0;
        int triangleIndex = 0;
        int edgeIndex = 0;

        for (int i = 1; i < lines.Length; i++)
        {
            string line = lines[i];
            string[] parts = line.Split(' ');

            if (parts.Length < 3)
            {
                throw new FormatException("Riga non valida nel file .ma: " + line);
            }

            char type = parts[0][0];
            switch (type)
            {
                case 'v':
                    float x = float.Parse(parts[1], CultureInfo.InvariantCulture);
                    float y = float.Parse(parts[2], CultureInfo.InvariantCulture);
                    float z = float.Parse(parts[3], CultureInfo.InvariantCulture);
                    vertices[vertexIndex] = new Vector3(x, y, z);
                    radii[vertexIndex] = float.Parse(parts[4], CultureInfo.InvariantCulture);
                    vertexIndex++;
                    break;
                case 'e':
                    int v1 = int.Parse(parts[1]);
                    int v2 = int.Parse(parts[2]);
                    edges[edgeIndex] = v1;
                    edges[edgeIndex + 1] = v2;
                    edgeIndex += 2;
                    break;
                case 'f':
                    int vA = int.Parse(parts[1]);
                    int vB = int.Parse(parts[2]);
                    int vC = int.Parse(parts[3]);
                    triangles[triangleIndex] = vA;
                    triangles[triangleIndex + 1] = vB;
                    triangles[triangleIndex + 2] = vC;
                    triangleIndex += 3;
                    break;
                default:
                    throw new FormatException("Tipo non valido nel file .ma: " + type);
            }
        }
    }
    #endregion
}