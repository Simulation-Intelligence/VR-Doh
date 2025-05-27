using UnityEngine;
using System.IO;

public class NeRFDataGenerator : MonoBehaviour
{
    public GameObject model; // 球心的模型
    public int numberOfSamples = 100; // 样本数量
    public float sphereRadius = 10f; // 球体半径
    public int Resolution = 1024; // 分辨率
    public Camera captureCamera; // 用于拍摄的相机
    public string savePath = "Assets/NeRFImages/images"; // 保存路径

    void Start()
    {
        GenerateNeRFData();
    }

    void GenerateNeRFData()
    {
        if (!Directory.Exists(savePath))
        {
            Directory.CreateDirectory(savePath);
        }

        for (int i = 0; i < numberOfSamples; i++)
        {
            Vector3 randomPosition = Random.onUnitSphere * sphereRadius + model.transform.position;
            captureCamera.transform.position = randomPosition;
            captureCamera.transform.LookAt(model.transform);

            RenderTexture renderTexture = new RenderTexture(Resolution, Resolution, 24);
            captureCamera.targetTexture = renderTexture;
            Texture2D screenShot = new Texture2D(Resolution, Resolution, TextureFormat.RGB24, false);

            captureCamera.Render();
            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(new Rect(0, 0, Resolution, Resolution), 0, 0);
            captureCamera.targetTexture = null;
            RenderTexture.active = null;
            Destroy(renderTexture);

            byte[] bytes = screenShot.EncodeToPNG();
            string filename = Path.Combine(savePath, "image" + model.name + i.ToString("D3") + ".png");
            File.WriteAllBytes(filename, bytes);

            Debug.Log("Saved: " + filename);
        }
    }
}
