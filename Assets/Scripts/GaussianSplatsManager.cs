using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class GaussianSplatManager : MonoBehaviour
{
    public struct RichPoint
    {
        public Vector3 pos;
        public float[] n;
        public float[] shs;
        public float opacity;
        public Vector3 scale;
        public Quaternion rot;
    }

    public string filePath;
    public float eps = 0.01f;
    public int splat_num;

    public float[] positions;
    public float[] shsList;
    public float[] opacities;
    public float[] scales;
    public float[] rotations;

    public Vector3 min;
    public Vector3 max;

    public void Awake()
    {
        LoadPly(filePath);
        ScaleToUnitCube();
    }

    public void LoadPly(string filename)
    {
        if (string.IsNullOrEmpty(filename))
        {
            Debug.LogError("File path is not set.");
            return;
        }

        List<RichPoint> points = new();
        min = new(float.MaxValue, float.MaxValue, float.MaxValue);
        max = new(float.MinValue, float.MinValue, float.MinValue);

        using BinaryReader reader = new(File.Open(filename, FileMode.Open));

        // Read and parse the PLY header
        string line = ReadLine(reader);
        if (line != "ply")
            throw new Exception("Invalid PLY file.");

        while ((line = ReadLine(reader)) != "end_header")
        {
            if (line.StartsWith("element vertex"))
            {
                string[] parts = line.Split(' ');
                splat_num = int.Parse(parts[2]);
            }
        }

        positions = new float[splat_num * 3]; // 3 floats per position (x, y, z)
        shsList = new float[splat_num * 48];  // 48 floats for SH coefficients for D = 3
        opacities = new float[splat_num];
        scales = new float[splat_num * 3];    // 3 floats per scale (x, y, z)
        rotations = new float[splat_num * 4]; // 4 floats per quaternion (x, y, z, w)

        // Read data from the PLY file
        for (int i = 0; i < splat_num; i++)
        {
            Vector3 pos = new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            float[] n = new float[3];
            for (int j = 0; j < 3; j++)
                n[j] = reader.ReadSingle();

            float[] shs = new float[48]; // (D + 1)^2 * 3 for D = 3
            for (int j = 0; j < shs.Length; j++)
                shs[j] = reader.ReadSingle();

            float opacity = reader.ReadSingle();
            Vector3 scale = new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            Quaternion rot = new(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

            points.Add(new RichPoint { pos = pos, n = n, shs = shs, opacity = opacity, scale = scale, rot = rot });

            min = Vector3.Min(min, pos);
            max = Vector3.Max(max, pos);
        }

        // Morton order sorting
        List<(ulong, int)> mortonCodes = new();
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 relativePos = new(
                (points[i].pos.x - min.x) / (max.x - min.x),
                (points[i].pos.y - min.y) / (max.y - min.y),
                (points[i].pos.z - min.z) / (max.z - min.z)
            );
            Vector3Int scaled = Vector3Int.FloorToInt(relativePos * ((1 << 21) - 1));
            ulong mortonCode = CalculateMortonCode(scaled);
            mortonCodes.Add((mortonCode, i));
        }
        //mortonCodes.Sort((a, b) => a.Item1.CompareTo(b.Item1));

        for (int k = 0; k < mortonCodes.Count; k++)
        {
            int i = mortonCodes[k].Item2;
            RichPoint point = points[i];

            positions[k * 3] = point.pos.x;
            positions[k * 3 + 1] = point.pos.y;
            positions[k * 3 + 2] = point.pos.z;

            // Handle SH coefficients correctly
            shsList[k * 48] = point.shs[0];
            shsList[k * 48 + 1] = point.shs[1];
            shsList[k * 48 + 2] = point.shs[2];
            int SH_N = 16;
            for (int j = 1; j < SH_N; j++)
            {
                shsList[k * 48 + j * 3 + 0] = point.shs[(j - 1) + 3];
                shsList[k * 48 + j * 3 + 1] = point.shs[(j - 1) + SH_N + 2];
                shsList[k * 48 + j * 3 + 2] = point.shs[(j - 1) + 2 * SH_N + 1];
            }

            opacities[k] = Sigmoid(point.opacity);

            scales[k * 3] = Mathf.Exp(point.scale.x);
            scales[k * 3 + 1] = Mathf.Exp(point.scale.y);
            scales[k * 3 + 2] = Mathf.Exp(point.scale.z);

            float length = Mathf.Sqrt(point.rot.x * point.rot.x + point.rot.y * point.rot.y +
                                      point.rot.z * point.rot.z + point.rot.w * point.rot.w);
            rotations[k * 4] = point.rot.x / length;
            rotations[k * 4 + 1] = point.rot.y / length;
            rotations[k * 4 + 2] = point.rot.z / length;
            rotations[k * 4 + 3] = point.rot.w / length;

        }
    }

    private ulong CalculateMortonCode(Vector3Int pos)
    {
        ulong mortonCode = 0;
        for (int i = 0; i < 21; i++)
        {
            mortonCode |= (ulong)(pos.x & (1 << i)) << (2 * i + 0);
            mortonCode |= (ulong)(pos.y & (1 << i)) << (2 * i + 1);
            mortonCode |= (ulong)(pos.z & (1 << i)) << (2 * i + 2);
        }
        return mortonCode;
    }

    private string ReadLine(BinaryReader reader)
    {
        List<byte> line = new();
        byte b;
        while ((b = reader.ReadByte()) != 10) // LF = 10
        {
            line.Add(b);
        }
        return System.Text.Encoding.ASCII.GetString(line.ToArray());
    }
    public void ScaleToUnitCube()
    {
        Vector3 center = (min + max) / 2f;
        Vector3 size = max - min;

        // Scale factor based on the largest dimension
        float scaleFactor = (1f - 2 * eps) / Mathf.Max(size.x, size.y, size.z);

        Vector3 newCenter = new(0.5f, 0.5f, 0.5f);

        for (int i = 0; i < splat_num; i++)
        {
            // Scale positions and translate them to the new center
            positions[i * 3] = (positions[i * 3] - center.x) * scaleFactor + newCenter.x;
            positions[i * 3 + 1] = (positions[i * 3 + 1] - center.y) * scaleFactor + newCenter.y;
            positions[i * 3 + 2] = (positions[i * 3 + 2] - center.z) * scaleFactor + newCenter.z;

            // Scale scales (already exponentiated)
            scales[i * 3] *= scaleFactor;
            scales[i * 3 + 1] *= scaleFactor;
            scales[i * 3 + 2] *= scaleFactor;
        }

        // Update min and max after scaling
        min = (min - center) * scaleFactor + newCenter;
        max = (max - center) * scaleFactor + newCenter;
    }
    private float Sigmoid(float x)
    {
        return 1.0f / (1.0f + Mathf.Exp(-x));
    }
}
