using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Oculus.Interaction.Input;

public class HandTrackingManager : MonoBehaviour
{
    public OVRHand leftHand;   // 拖入左手 Oculus Hand prefab
    public OVRHand rightHand;  // 拖入右手 Oculus Hand prefab

    public OVRSkeleton leftSkeleton;   // 左手的骨骼数据
    public OVRSkeleton rightSkeleton;  // 右手的骨骼数据

    public bool isRecording = false;  // 是否处于记录模式
    public bool isPlaying = false;    // 是否处于播放模式

    private List<FrameData> recordedFrames = new List<FrameData>();
    private int currentFrameIndex = 0;  // 当前播放的帧索引

    void Start()
    {
        if (isPlaying)
        {
            LoadRecording();  // 加载录制的数据
        }
    }

    void Update()
    {
        if (isRecording && leftHand.IsTracked && rightHand.IsTracked)
        {
            RecordFrame();  // 记录当前左右手数据
        }
        else if (isPlaying && currentFrameIndex < recordedFrames.Count)
        {
            PlayFrame();  // 播放当前帧数据
        }
    }

    // 记录当前帧的左右手数据
    private void RecordFrame()
    {
        var frame = new FrameData();

        // 记录左手的骨骼数据
        foreach (var joint in leftSkeleton.Bones)
        {
            frame.leftHandJoints.Add(new JointData
            {
                position = joint.Transform.position,
                rotation = joint.Transform.rotation
            });
        }

        // 记录右手的骨骼数据
        foreach (var joint in rightSkeleton.Bones)
        {
            frame.rightHandJoints.Add(new JointData
            {
                position = joint.Transform.position,
                rotation = joint.Transform.rotation
            });
        }

        recordedFrames.Add(frame);  // 保存这一帧数据
    }

    // 播放当前帧数据
    private void PlayFrame()
    {
        var frame = recordedFrames[currentFrameIndex];

        // 应用左手数据
        ApplyFrameToHand(frame.leftHandJoints, leftSkeleton);

        // 应用右手数据
        ApplyFrameToHand(frame.rightHandJoints, rightSkeleton);

        currentFrameIndex++;

        // 循环播放（可选）
        if (currentFrameIndex >= recordedFrames.Count)
        {
            currentFrameIndex = 0;
        }
    }

    // 将帧数据应用到对应的手部
    private void ApplyFrameToHand(List<JointData> joints, OVRSkeleton skeleton)
    {
        for (int i = 0; i < skeleton.Bones.Count; i++)
        {
            skeleton.Bones[i].Transform.position = joints[i].position;
            skeleton.Bones[i].Transform.rotation = joints[i].rotation;
        }
    }

    // 保存录制的数据到 JSON 文件
    public void SaveRecording()
    {
        string json = JsonUtility.ToJson(new FrameCollection { frames = recordedFrames });
        File.WriteAllText(Application.persistentDataPath + "/handTracking.json", json);
        Debug.Log("Recording saved to: " + Application.persistentDataPath);
    }

    // 加载 JSON 文件中的数据
    public void LoadRecording()
    {
        string path = Application.persistentDataPath + "/handTracking.json";
        if (File.Exists(path))
        {
            string json = File.ReadAllText(path);
            FrameCollection loadedFrames = JsonUtility.FromJson<FrameCollection>(json);
            recordedFrames = loadedFrames.frames;
            Debug.Log("Loaded " + recordedFrames.Count + " frames.");
        }
        else
        {
            Debug.LogError("Recording file not found.");
        }
    }
}

// 帧数据结构
[System.Serializable]
public class JointData
{
    public Vector3 position;
    public Quaternion rotation;
}

[System.Serializable]
public class FrameData
{
    public List<JointData> leftHandJoints = new List<JointData>();  // 左手的关节数据
    public List<JointData> rightHandJoints = new List<JointData>(); // 右手的关节数据
}

[System.Serializable]
public class FrameCollection
{
    public List<FrameData> frames = new List<FrameData>();  // 所有帧数据
}
