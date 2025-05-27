using UnityEngine;
using System;

public class VolumeTextureUpdater : MonoBehaviour
{
    public int width = 32;  // X 轴尺寸
    public int height = 32; // Y 轴尺寸
    public int depth = 32;  // Z 轴尺寸
    public GameObject targetObject; // 目标 GameObject
    public string texturePropertyName = "_volumeTex"; // 材质中使用的纹理属性名

    public float[] densityData; // 三维数组存储密度数据
    public ComputeBuffer computeBuffer;
    public ComputeShader computeShader;
    public float max_density = 100;
    public RenderTexture volumeTex;   // RenderTexture 对象
    public Material targetMaterial; // 目标材质

    void Start()
    {
        // 初始化密度数据和 RenderTexture
        densityData = new float[width * height * depth];
        computeBuffer = new ComputeBuffer(width * height * depth, sizeof(float));

        // 创建 RenderTexture
        volumeTex = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat)
        {
            dimension = UnityEngine.Rendering.TextureDimension.Tex3D,
            volumeDepth = depth,
            enableRandomWrite = true,
            wrapMode = TextureWrapMode.Clamp
        };
        volumeTex.Create(); // 确保 RenderTexture 已创建

        // 获取目标对象的材质
        targetMaterial = targetObject.GetComponent<Renderer>().material;
        if (targetMaterial == null)
        {
            Debug.LogError("Target object does not have a material.");
            return;
        }
    }

    void Update()
    {
        // 在每一帧或需要时更新密度数据
        // UpdateDensityData();

        // 更新 RenderTexture 的数据
        RunComputeShader();
    }

    void UpdateDensityData()
    {
        // 定义球体的半径
        float radius = 0.5f * Mathf.Min(width, height, depth);
        // 定义球体的中心
        Vector3 center = new Vector3(width / 2.0f, height / 2.0f, depth / 2.0f);

        for (int z = 0; z < depth; z++)
        {
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    // 计算当前点到中心的距离
                    Vector3 pos = new Vector3(x, y, z);
                    float distance = Vector3.Distance(pos, center);

                    // 如果点在球体内，设置密度为 1，否则为 0
                    densityData[x + y * width + z * width * height] = (distance <= radius) ? 1.0f : 0.0f;
                }
            }
        }
    }

    void RunComputeShader()
    {
        int kernelHandle = computeShader.FindKernel("CSMain");

        // 设置 ComputeShader 参数
        computeShader.SetBuffer(kernelHandle, "dataBuffer", computeBuffer);
        computeShader.SetTexture(kernelHandle, "Result", volumeTex);
        computeShader.SetInt("textureSize", width);

        computeShader.SetFloat("normalizationFactor", max_density);

        // 执行 ComputeShader
        int threadGroups = Mathf.CeilToInt(width / 8.0f);
        computeShader.Dispatch(kernelHandle, threadGroups, threadGroups, threadGroups);

        // 将 RenderTexture 设置到材质的指定属性
        targetMaterial.SetTexture(texturePropertyName, volumeTex);
    }
}
