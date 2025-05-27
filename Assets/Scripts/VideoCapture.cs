using UnityEngine;
using System.IO;
using System.Diagnostics;

public class CameraVideoCapture : MonoBehaviour
{
    public GameObject objectToRotate; // 需要旋转的物体
    public Camera captureCamera; // 用于拍摄的相机
    public Vector3 rotationAxis = Vector3.up; // 物体旋转的轴（默认是Y轴）

    public float rotationSpeed = 10f; // 旋转速度
    public int ResolutionWidth = 1024; // 横向分辨率
    public int ResolutionHeight = 768; // 纵向分辨率
    public string savePath = "Assets/VideoFrames"; // 保存路径
    public bool createSubfolders = false; // 是否创建子文件夹，按物体名称生成

    private int frameCount = 0; // 帧计数器

    public int captureFrameCount = 240; // 捕获帧数

    private bool isCapturing = true; // 是否正在捕获

    void Start()
    {
        // 创建保存路径
        if (!Directory.Exists(savePath))
        {
            Directory.CreateDirectory(savePath);
        }

        // 如果创建子文件夹，则按物体名称创建
        if (createSubfolders)
        {
            string subfolder = savePath + "/images_" + objectToRotate.name;
            if (!Directory.Exists(subfolder))
            {
                Directory.CreateDirectory(subfolder);
            }
            savePath = subfolder;
        }
    }

    void Update()
    {
        if (isCapturing)
        {
            // 让物体沿某一轴旋转（可以修改为任意轴）
            objectToRotate.transform.Rotate(rotationAxis * rotationSpeed * Time.deltaTime); // 默认绕Y轴旋转

            // 每一帧生成一个图片
            CaptureFrame();
        }
    }

    void CaptureFrame()
    {
        // 相机始终指向物体
        captureCamera.transform.LookAt(objectToRotate.transform);

        // 创建RenderTexture并设置为相机目标纹理
        RenderTexture renderTexture = new RenderTexture(ResolutionWidth, ResolutionHeight, 24);
        captureCamera.targetTexture = renderTexture;

        // 创建一个临时的Texture2D来保存捕获的图像
        Texture2D screenShot = new Texture2D(ResolutionWidth, ResolutionHeight, TextureFormat.RGB24, false);

        // 渲染相机视图并读取像素
        captureCamera.Render();
        RenderTexture.active = renderTexture;
        screenShot.ReadPixels(new Rect(0, 0, ResolutionWidth, ResolutionHeight), 0, 0);
        captureCamera.targetTexture = null; // 清除目标纹理
        RenderTexture.active = null; // 清除活动纹理

        // 清理RenderTexture
        Destroy(renderTexture);

        // 编码为PNG并保存到指定路径
        byte[] bytes = screenShot.EncodeToPNG();
        string filename = Path.Combine(savePath, "frame_" + frameCount.ToString("D3") + ".png");
        File.WriteAllBytes(filename, bytes);

        // 输出保存信息
        UnityEngine.Debug.Log("Saved frame " + frameCount + " to " + filename);

        // 更新帧计数器
        frameCount++;

        // 如果捕获帧数达到指定数量，则停止捕获
        if (frameCount >= captureFrameCount)
        {
            isCapturing = false;
            UnityEngine.Debug.Log("Capture finished.");
            CreateVideoFromFrames();
        }
    }

    void CreateVideoFromFrames()
    {
        // 定义FFmpeg命令的路径，确保你已安装FFmpeg，并将其添加到环境变量中
        string ffmpegPath = "ffmpeg"; // 如果FFmpeg已添加到系统路径，直接使用"ffmpeg"即可
        string inputPattern = Path.Combine(savePath, "frame_%03d.png"); // 图像序列的路径
        string outputVideo = Path.Combine(savePath, "output_video.mp4"); // 输出视频的路径

        // 创建FFmpeg命令
        string arguments = $"-framerate 60 -i \"{inputPattern}\" -c:v libx264 -pix_fmt yuv420p \"{outputVideo}\"";

        // 执行FFmpeg命令
        RunProcess(ffmpegPath, arguments);

        // 删除原有图片
        DeleteCapturedFrames();
    }

    void RunProcess(string fileName, string arguments)
    {
        // 创建一个新的进程来运行FFmpeg
        Process process = new Process();
        process.StartInfo.FileName = fileName;
        process.StartInfo.Arguments = arguments;
        process.StartInfo.CreateNoWindow = true;
        process.StartInfo.UseShellExecute = false;
        process.StartInfo.RedirectStandardOutput = true;
        process.StartInfo.RedirectStandardError = true;

        // 捕获FFmpeg的输出和错误信息
        process.OutputDataReceived += (sender, e) => UnityEngine.Debug.Log("FFmpeg Output: " + e.Data);
        process.ErrorDataReceived += (sender, e) => UnityEngine.Debug.Log("FFmpeg Error: " + e.Data);

        process.Start();

        // 启动进程并异步读取输出和错误信息
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        // 等待进程结束
        process.WaitForExit();

        UnityEngine.Debug.Log("FFmpeg process finished.");
    }

    // 删除保存的图像文件
    void DeleteCapturedFrames()
    {
        string[] files = Directory.GetFiles(savePath, "frame_*.png");
        foreach (string file in files)
        {
            try
            {
                File.Delete(file); // 删除文件
                UnityEngine.Debug.Log("Deleted: " + file);
            }
            catch (IOException e)
            {
                UnityEngine.Debug.LogError("Failed to delete " + file + ": " + e.Message);
            }
        }
    }
}
