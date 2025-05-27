using UnityEngine;
using System.Collections.Generic;

public class MeshVertexInfo : MonoBehaviour
{
    [SerializeField]
    private MeshFilter[] meshFilters; // 在编辑器中传入多个MeshFilter
    public Transform positionOffset;
    public float[] combinedVertices;
    private float[] combinedPreviousVertices;
    public float[] combinedVelocities;
    public int subdivisions = 1;

    void Awake()
    {
        // 计算所有顶点的总数
        int totalVertexCount = 0;
        for (int i = 0; i < meshFilters.Length; i++)
        {
            if (meshFilters[i] != null && meshFilters[i].mesh != null)
            {
                for (int j = 0; j < subdivisions; j++)
                {
                    SubdivideMesh(meshFilters[i].mesh);
                }
                totalVertexCount += meshFilters[i].mesh.vertexCount;
            }
        }

        // 初始化float数组
        combinedVertices = new float[totalVertexCount * 3];
        combinedPreviousVertices = new float[totalVertexCount * 3];
        combinedVelocities = new float[totalVertexCount * 3];

        // 填充初始顶点位置
        int index = 0;
        foreach (MeshFilter meshFilter in meshFilters)
        {
            if (meshFilter != null && meshFilter.mesh != null)
            {
                Vector3[] vertices = meshFilter.mesh.vertices;
                Transform transform = meshFilter.transform;
                for (int i = 0; i < vertices.Length; i++)
                {
                    Vector3 vertex = transform.TransformPoint(vertices[i]);
                    combinedVertices[index * 3] = vertex.x - positionOffset.position.x;
                    combinedVertices[index * 3 + 1] = vertex.y - positionOffset.position.y;
                    combinedVertices[index * 3 + 2] = vertex.z - positionOffset.position.z;

                    combinedPreviousVertices[index * 3] = vertex.x;
                    combinedPreviousVertices[index * 3 + 1] = vertex.y;
                    combinedPreviousVertices[index * 3 + 2] = vertex.z;

                    index++;
                }
            }
        }
    }

    void Update()
    {
        int vertexIndex = 0;
        for (int i = 0; i < meshFilters.Length; i++)
        {
            if (meshFilters[i] != null && meshFilters[i].mesh != null)
            {
                Vector3[] currentVertices = meshFilters[i].mesh.vertices;
                Transform transform = meshFilters[i].transform;
                // 更新顶点位置和速度
                for (int j = 0; j < currentVertices.Length; j++)
                {
                    Vector3 vertex = transform.TransformPoint(currentVertices[j]);
                    combinedVelocities[vertexIndex * 3] = (vertex.x - combinedPreviousVertices[vertexIndex * 3]) / Time.deltaTime;
                    combinedVelocities[vertexIndex * 3 + 1] = (vertex.y - combinedPreviousVertices[vertexIndex * 3 + 1]) / Time.deltaTime;
                    combinedVelocities[vertexIndex * 3 + 2] = (vertex.z - combinedPreviousVertices[vertexIndex * 3 + 2]) / Time.deltaTime;

                    combinedPreviousVertices[vertexIndex * 3] = vertex.x;
                    combinedPreviousVertices[vertexIndex * 3 + 1] = vertex.y;
                    combinedPreviousVertices[vertexIndex * 3 + 2] = vertex.z;

                    combinedVertices[vertexIndex * 3] = vertex.x - positionOffset.position.x;
                    combinedVertices[vertexIndex * 3 + 1] = vertex.y - positionOffset.position.y;
                    combinedVertices[vertexIndex * 3 + 2] = vertex.z - positionOffset.position.z;

                    vertexIndex++;
                }
            }
        }

    }
    void SubdivideMesh(Mesh mesh)
    {
        Vector3[] oldVertices = mesh.vertices;
        int[] oldTriangles = mesh.triangles;

        List<Vector3> newVertices = new(oldVertices);
        List<int> newTriangles = new();

        Dictionary<long, int> midpointCache = new();

        for (int i = 0; i < oldTriangles.Length; i += 3)
        {
            int v0 = oldTriangles[i];
            int v1 = oldTriangles[i + 1];
            int v2 = oldTriangles[i + 2];

            int a = GetMidpointIndex(midpointCache, newVertices, oldVertices, v0, v1);
            int b = GetMidpointIndex(midpointCache, newVertices, oldVertices, v1, v2);
            int c = GetMidpointIndex(midpointCache, newVertices, oldVertices, v2, v0);

            newTriangles.Add(v0);
            newTriangles.Add(a);
            newTriangles.Add(c);

            newTriangles.Add(v1);
            newTriangles.Add(b);
            newTriangles.Add(a);

            newTriangles.Add(v2);
            newTriangles.Add(c);
            newTriangles.Add(b);

            newTriangles.Add(a);
            newTriangles.Add(b);
            newTriangles.Add(c);
        }

        mesh.Clear();
        mesh.vertices = newVertices.ToArray();
        mesh.triangles = newTriangles.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
    private int GetMidpointIndex(Dictionary<long, int> cache, List<Vector3> vertices, Vector3[] oldVertices, int i0, int i1)
    {
        bool isSmallerFirst = i0 < i1;
        long smallerIndex = isSmallerFirst ? i0 : i1;
        long largerIndex = isSmallerFirst ? i1 : i0;
        long key = (smallerIndex << 32) + largerIndex;

        if (cache.TryGetValue(key, out int index))
        {
            return index;
        }

        Vector3 midpoint = (oldVertices[i0] + oldVertices[i1]) * 0.5f;
        index = vertices.Count;
        vertices.Add(midpoint);

        cache[key] = index;
        return index;
    }
}