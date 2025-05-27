using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MATUtils
{
    public static bool GenerateSlab(Vector3 v1, float r1, Vector3 v2, float r2, Vector3 v3, float r3,out List<Vector3> slabVerts,out List<int> slabFaces, float threshold = 1e-4f)
    {
        slabVerts = new List<Vector3>();
        slabFaces = new List<int>();
        // Calculate v12, v13, and n
        Vector3 v12 = v1 - v2;
        Vector3 v13 = v1 - v3;
        Vector3 n = Vector3.Normalize(Vector3.Cross(v12, v13));

        // Calculate tangent points and distances
        Vector3[] tangentPoints1 = IntersectPointOfCones(v1, r1, v2, r2, v3, r3, n);
        float d2v1 = Vector3.Distance(tangentPoints1[0], v1) - r1;

        Vector3[] tangentPoints2 = IntersectPointOfCones(v2, r2, v1, r1, v3, r3, n);
        float d2v2 = Vector3.Distance(tangentPoints2[0], v2) - r2;

        Vector3[] tangentPoints3 = IntersectPointOfCones(v3, r3, v1, r1, v2, r2, n);
        float d2v3 = Vector3.Distance(tangentPoints3[0], v3) - r3;

        if (d2v1 > threshold || d2v2 > threshold || d2v3 > threshold)
        {
            return false;
        }

        // Add vertices to the slabVerts list
        slabVerts.Add(tangentPoints1[0]);
        slabVerts.Add(tangentPoints2[0]);
        slabVerts.Add(tangentPoints3[0]);
        slabVerts.Add(tangentPoints1[1]);
        slabVerts.Add(tangentPoints2[1]);
        slabVerts.Add(tangentPoints3[1]);

        // Calculate vertex count
        int vcount = slabVerts.Count;

        // Add faces to the slabFaces list
        slabFaces.Add(vcount - 1);
        slabFaces.Add(vcount - 3);
        slabFaces.Add(vcount - 2);
        slabFaces.Add(vcount - 4);
        slabFaces.Add(vcount - 5);
        slabFaces.Add(vcount - 6);

        return true;
    }
    public static void GenerateConicalSurface(Vector3 v1, float r1, Vector3 v2, float r2, int resolution,out List<Vector3> coneVerts,out List<int> coneFaces)
    {
        coneVerts = new List<Vector3>();
        coneFaces = new List<int>();
        
        if (r1 < 1e-3f && r2 < 1e-3f)
        {
            return;
        }

        Vector3 c12 = v2 - v1;
        float phi = ComputeAngle(r1, r2, c12);

        c12 = Vector3.Normalize(c12);

        // Start direction
        Vector3 startDir = new Vector3(0.0f, 1.0f, 0.0f); // Pick randomly

        if (Math.Abs(Vector3.Dot(c12, startDir)) > 0.999f)
        {
            startDir = new Vector3(1.0f, 0.0f, 0.0f); // If parallel to c12, pick a new direction
        }

        // Correct the startDir (perpendicular to c12)
        startDir = Vector3.Normalize(Vector3.Cross(c12, startDir));

        int start_index = coneVerts.Count;
        int local_vcount = 0;
        Matrix4x4 mat = RotateMat(v1, c12, (float)(2.0f * Math.PI / resolution));

        for (int i = 0; i < resolution; i++)
        {
            float cos = (float)Math.Cos(phi);
            float sin = (float)Math.Sin(phi);

            Vector3 pos;

            if (r1 < 1e-3f)
            {
                pos = v1;
            }
            else
            {
                pos = v1 + (c12 * cos + startDir * sin) * r1;
            }

            coneVerts.Add(pos);

            if (r2 < 1e-3f)
            {
                pos = v2;
            }
            else
            {
                pos = v2 + (c12 * cos + startDir * sin) * r2;
            }

            coneVerts.Add(pos);
            local_vcount += 2;

            // Rotate
            Vector4 result = mat * new Vector4(startDir.x, startDir.y, startDir.z, 0);
            startDir = new Vector3(result.x, result.y, result.z);
        }

        for (int i = 0; i < local_vcount - 2; i += 2)
        {
            coneFaces.Add(start_index + i);
            coneFaces.Add(start_index + i + 3);
            coneFaces.Add(start_index + i + 1);
            coneFaces.Add(start_index + i);
            coneFaces.Add(start_index + i + 2);
            coneFaces.Add(start_index + i + 3);
        }

        // Close conical surface
        coneFaces.Add(start_index + local_vcount - 2);
        coneFaces.Add(start_index + 1);
        coneFaces.Add(start_index + local_vcount - 1);
        coneFaces.Add(start_index + local_vcount - 2);
        coneFaces.Add(start_index);
        coneFaces.Add(start_index + 1);
        return;
    }
    private static Vector3[] IntersectPointOfCones(Vector3 v1, float r1, Vector3 v2, float r2, Vector3 v3, float r3, Vector3 norm)
    {
        if (r1 < 1e-3f)
        {
            return new Vector3[2] { v1, v1 };
        }

        Vector3 v12 = v2 - v1;
        float phi_12 = ComputeAngle(r1, r2, v12);

        Vector3 v13 = v3 - v1;
        float phi_13 = ComputeAngle(r1, r3, v13);

        Vector3 p12 = v1 + Vector3.Normalize(v12) * (float)Math.Cos(phi_12) * r1;
        Vector3 p13 = v1 + Vector3.Normalize(v13) * (float)Math.Cos(phi_13) * r1;
        Matrix4x4 mat = RotateMat(p12, v12, (float)Math.PI / 2);
        Vector4 result = mat * new Vector4(norm.x, norm.y, norm.z, 0);
        Vector3 dir_12 = new Vector3(result.x, result.y, result.z);
        Vector3 intersect_p = PlaneLineIntersection(v13, p13, dir_12, p12);

        Vector3 v1p = intersect_p - v1;
        float dotProduct = Vector3.Dot(v1p, v1p);
        float scaled_n_length = (float)Math.Sqrt(Math.Max(r1 * r1 - dotProduct, 1e-5f));
        Vector3 scaled_n = scaled_n_length * norm;

        return new Vector3[2] { intersect_p + scaled_n, intersect_p - scaled_n };
    }
    private static Vector3 PlaneLineIntersection(Vector3 n, Vector3 p, Vector3 d, Vector3 a)
    {
        float vpt = Vector3.Dot(d, n);

        if (vpt == 0)
        {
            Debug.LogError("SLAB GENERATION::Parallel Error");
            return Vector3.zero;
        }

        float t = (Vector3.Dot(p - a, n)) / vpt;
        return a + d * t;
    }
    private static float ComputeAngle(float r1, float r2, Vector3 c21)
    {
        float r21_2 = (float)Math.Max(Math.Pow(r1 - r2, 2), 0.0f);
        if (r21_2 == 0)
        {
            return (float)Math.PI / 2;
        }

        float dotProduct = Vector3.Dot(c21, c21);
        float squareRootArg = Math.Max(dotProduct - r21_2, 0) / r21_2;
        float phi = (float)Math.Atan(Math.Sqrt(squareRootArg));

        if (r1 < r2)
        {
            phi = (float)Math.PI - phi;
        }

        return phi;
    }
    private static Matrix4x4 RotateMat(Vector3 point, Vector3 vector, float angle)
    {
        float u = vector.x, v = vector.y, w = vector.z;
        float a = point.x, b = point.y, c = point.z;
        float cos = (float)Math.Cos(angle);
        float sin = (float)Math.Sin(angle);

        float u2 = u * u;
        float v2 = v * v;
        float w2 = w * w;
        float uv = u * v;
        float uw = u * w;
        float vw = v * w;

        return new Matrix4x4(
            new Vector4(u2 + (v2 + w2) * cos, uv * (1 - cos) - w * sin, uw * (1 - cos) + v * sin, (a * (v2 + w2) - u * (b * v + c * w)) * (1 - cos) + (b * w - c * v) * sin),
            new Vector4(uv * (1 - cos) + w * sin, v2 + (u2 + w2) * cos, vw * (1 - cos) - u * sin, (b * (u2 + w2) - v * (a * u + c * w)) * (1 - cos) + (c * u - a * w) * sin),
            new Vector4(uw * (1 - cos) - v * sin, vw * (1 - cos) + u * sin, w2 + (u2 + v2) * cos, (c * (u2 + v2) - w * (a * u + b * v)) * (1 - cos) + (a * v - b * u) * sin),
            new Vector4(0, 0, 0, 1)
        );
    }
}
