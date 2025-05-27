using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine.Rendering;  // To handle file writing and reading

public class SmoothHand : MonoBehaviour
{
    public enum HandType { LeftHand, RightHand }
    public HandType handType;
    public OVRHand oculus_hand;
    public OVRSkeleton oculus_skeleton;
    [SerializeField]
    private SkinnedMeshRenderer _skinnedMeshRenderer;
    [Header("Specify a New Material")]
    public Material _newMaterial;
    [SerializeField]
    private List<Transform> leftHandJoints = new List<Transform>();
    [SerializeField]
    private List<Transform> rightHandJoints = new List<Transform>();
    public List<Transform> SmoothLeftHandJoints => leftHandJoints;
    public List<Transform> SmoothRightHandJoints => rightHandJoints;

    public enum MovingAverage { None, Simple, Weighted, TimeScaled }
    public MovingAverage movingAverage = MovingAverage.Weighted;

    [Range(1, 20)]
    public int windowSize = 5;
    [Range(0.01f, 1.0f)]
    public float timeWindow = 0.25f;
    private float[] weights;
    private Dictionary<int, List<Vector3>> positionQueue = new Dictionary<int, List<Vector3>>();
    private Dictionary<int, List<Quaternion>> rotationQueue = new Dictionary<int, List<Quaternion>>();
    private Dictionary<int, List<float>> timeStepQueue = new Dictionary<int, List<float>>();
    // New variables for recording data
    public bool record_data = false;
    public bool use_record_data = false;
    public string filePath = "hand_joints_data.txt"; // File path to store hand joints data

    StreamReader reader;

    int cnt = 0;

    [SerializeField]
    int maxFrame = int.MaxValue;
    void Awake()
    {
        if (_newMaterial != null)
        {
            _skinnedMeshRenderer.material = _newMaterial;
        }

        if (handType == HandType.LeftHand)
        {
            if (oculus_hand == null | oculus_skeleton == null)
            {
                oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRHand>();
                oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRSkeleton>();
            }
        }
        else if (handType == HandType.RightHand)
        {
            if (oculus_hand == null | oculus_skeleton == null)
            {
                oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRHand>();
                oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRSkeleton>();
            }
        }

        // Initialize queues
        int numBones = handType == HandType.LeftHand ? leftHandJoints.Count : rightHandJoints.Count;
        switch (movingAverage)
        {
            case MovingAverage.Simple or MovingAverage.Weighted:
                for (int i = 0; i < numBones; i++)
                {
                    positionQueue[i] = new List<Vector3>(new Vector3[windowSize]);
                    rotationQueue[i] = new List<Quaternion>(new Quaternion[windowSize]);
                }
                // Weights for moving average smoothing
                weights = GetLinearWeights(windowSize);
                break;
            case MovingAverage.TimeScaled:
                for (int i = 0; i < numBones; i++)
                {
                    positionQueue[i] = new List<Vector3>();
                    rotationQueue[i] = new List<Quaternion>();
                    timeStepQueue[i] = new List<float>();
                }
                break;
        }
        if (record_data)
        {
            if (File.Exists(filePath))
            {
                File.Delete(filePath);  // Delete the file if it already exists
            }
        }
        if (use_record_data && File.Exists(filePath))
            reader = new StreamReader(filePath);
    }

    void Update()
    {
        cnt++;
        if (use_record_data && File.Exists(filePath)) // Use recorded data if the flag is true
        {
            LoadRecordedData();
        }
        else if (oculus_hand.IsTracked && oculus_hand.HandConfidence == OVRHand.TrackingConfidence.High)
        {
            int numBones = oculus_skeleton.Bones.Count;
            float currentTime = Time.time;

            for (int i = 0; i < numBones; i++)
            {
                OVRBone bone = oculus_skeleton.Bones[i];

                // Update queue data
                Vector3 newPosition = bone.Transform.position;
                Quaternion newRotation = bone.Transform.rotation;
                switch (movingAverage)
                {
                    case MovingAverage.Simple or MovingAverage.Weighted:
                        UpdateQueue(positionQueue[i], newPosition);
                        UpdateQueue(rotationQueue[i], newRotation);
                        break;
                    case MovingAverage.TimeScaled:
                        UpdateQueue(positionQueue[i], rotationQueue[i], timeStepQueue[i], newPosition, newRotation, currentTime);
                        break;
                }

                Vector3 smoothedPosition = newPosition;
                Quaternion smoothedRotation = newRotation;

                // Moving average smoothing
                switch (movingAverage)
                {
                    case MovingAverage.Simple:
                        smoothedPosition = SimpleMovingAverage(positionQueue[i]);
                        smoothedRotation = SimpleMovingAverage(rotationQueue[i]);
                        break;
                    case MovingAverage.Weighted:
                        smoothedPosition = WeightedMovingAverage(positionQueue[i], weights);
                        smoothedRotation = WeightedMovingAverage(rotationQueue[i], weights);
                        break;
                    case MovingAverage.TimeScaled:
                        smoothedPosition = TimeWeightedMovingAverage(positionQueue[i], timeStepQueue[i], currentTime);
                        smoothedRotation = TimeWeightedMovingAverage(rotationQueue[i], timeStepQueue[i], currentTime);
                        break;
                }

                // Update position and rotation
                if (handType == HandType.LeftHand)
                {
                    leftHandJoints[i].position = smoothedPosition;
                    leftHandJoints[i].rotation = smoothedRotation;
                }
                else if (handType == HandType.RightHand)
                {
                    rightHandJoints[i].position = smoothedPosition;
                    rightHandJoints[i].rotation = smoothedRotation;
                }

                if (record_data)
                {
                    //RecordHandJointsData(smoothedPosition, smoothedRotation, i);
                    RecordHandJointsData(newPosition, newRotation, i);
                }
            }
        }
    }
    void RecordHandJointsData(Vector3 position, Quaternion rotation, int i)
    {
        // Record position and rotation data for left or right hand joints into a text file
        string data = $"Bone {i} : Position = {position}, Rotation = {rotation}";
        File.AppendAllText(filePath, data + Environment.NewLine);
    }

    void LoadRecordedData()
    {
        if (cnt >= maxFrame)
        {
            return;
        }
        // Load the hand joints data from the txt file and apply it to the joint positions
        int count = handType == HandType.LeftHand ? leftHandJoints.Count : rightHandJoints.Count;
        string[] lines = new string[count];
        //check if next line is null
        if (reader.Peek() == -1)
        {
            // 处理文件末尾的逻辑，例如重新打开文件
            Console.WriteLine("End of file reached. Restarting...");
            OpenReader();  // 重新打开文件并重置读取器位置
        }
        for (int i = 0; i < count; i++)
        {
            lines[i] = reader.ReadLine();
        }
        for (int i = 0; i < count; i++)
        {
            string[] parts = lines[i].Split(new string[] { "Position = ", ", Rotation = " }, StringSplitOptions.None);
            if (parts.Length >= 2)
            {
                Vector3 position = StringToVector3(parts[1]);
                Quaternion rotation = StringToQuaternion(parts[2]);
                if (handType == HandType.LeftHand)
                {
                    leftHandJoints[i].position = position;
                    leftHandJoints[i].rotation = rotation;
                }
                else if (handType == HandType.RightHand)
                {
                    rightHandJoints[i].position = position;
                    rightHandJoints[i].rotation = rotation;
                }
            }
        }
    }
    private void OpenReader()
    {
        if (reader != null)
        {
            reader.Close();  // Close the existing reader if it was opened
        }

        // Open the reader to start from the beginning of the file
        reader = new StreamReader(filePath);
    }
    Vector3 StringToVector3(string str)
    {
        string[] values = str.Trim(new char[] { '(', ')' }).Split(',');
        return new Vector3(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]));
    }
    Quaternion StringToQuaternion(string str)
    {
        string[] values = str.Trim(new char[] { '(', ')' }).Split(',');
        return new Quaternion(float.Parse(values[0]), float.Parse(values[1]), float.Parse(values[2]), float.Parse(values[3]));
    }

    private Vector3 TimeWeightedMovingAverage(List<Vector3> posQueue, List<float> timeQueue, float currentTime)
    {
        Vector3 weightedSum = Vector3.zero;
        float totalWeight = 0.0f;
        List<float> weights = new List<float>();
        for (int i = 0; i < posQueue.Count; i++)
        {
            float timeDiff = currentTime - timeQueue[i];
            float w = Mathf.Exp(-timeDiff / timeWindow);
            // weightedSum += posQueue[i] * w;
            weights.Add(w);
            totalWeight += w;
        }
        for (int i = 0; i < posQueue.Count; i++)
        {
            float norm_w = weights[i] / totalWeight;
            weightedSum += posQueue[i] * norm_w;
        }
        return weightedSum;
    }

    private Quaternion TimeWeightedMovingAverage(List<Quaternion> rotQueue, List<float> timeQueue, float currentTime)
    {
        Vector3 averageForward = Vector3.zero;
        Vector3 averageUpwards = Vector3.zero;
        float totalWeight = 0.0f;
        List<float> weights = new List<float>();
        for (int i = 0; i < rotQueue.Count; i++)
        {
            float timeDiff = currentTime - timeQueue[i];
            float w = Mathf.Exp(-timeDiff / timeWindow);
            // averageForward += (rotQueue[i] * Vector3.forward) * w;
            // averageUpwards += (rotQueue[i] * Vector3.up) * w;
            weights.Add(w);
            totalWeight += w;
        }
        for (int i = 0; i < rotQueue.Count; i++)
        {
            float norm_w = weights[i] / totalWeight;
            averageForward += (rotQueue[i] * Vector3.forward) * norm_w;
            averageUpwards += (rotQueue[i] * Vector3.up) * norm_w;
        }
        return Quaternion.LookRotation(averageForward, averageUpwards);
    }

    private void UpdateQueue(List<Vector3> posQueue, List<Quaternion> rotQueue, List<float> timeQueue, Vector3 newPosition, Quaternion newRotation, float currentTime)
    {
        // Update the queue based on a time duration
        posQueue.Add(newPosition);
        rotQueue.Add(newRotation);
        timeQueue.Add(currentTime);

        while (timeQueue.Count > 0 && currentTime - timeQueue[0] > timeWindow)
        {
            posQueue.RemoveAt(0);
            rotQueue.RemoveAt(0);
            timeQueue.RemoveAt(0);
        }
    }

    private void UpdateQueue<T>(List<T> queue, T newValue)
    {
        // Update the queue based on a fixed frame number
        queue.Add(newValue);
        if (queue.Count > windowSize)
        {
            queue.RemoveAt(0);
        }
    }

    private Vector3 SimpleMovingAverage(List<Vector3> window)
    {
        float w = 1.0f / window.Count;
        Vector3 result = Vector3.zero;
        for (int i = 0; i < window.Count; i++)
        {
            result += window[i] * w;
        }
        return result;
    }

    private Quaternion SimpleMovingAverage(List<Quaternion> window)
    {
        Vector3 averageForward = Vector3.zero;
        Vector3 averageUpwards = Vector3.zero;
        float w = 1.0f / window.Count;
        for (int i = 0; i < window.Count; i++)
        {
            averageForward += (window[i] * Vector3.forward) * w;
            averageUpwards += (window[i] * Vector3.up) * w;
        }
        return Quaternion.LookRotation(averageForward, averageUpwards);
    }

    private Quaternion SlerpMovingAverage(List<Quaternion> window)
    {
        Quaternion result = window[0];
        for (int i = 0; i < window.Count; i++)
        {
            float t = 1.0f / window.Count;
            float w = 1.0f / (i + 1);
            result = Quaternion.Slerp(result, window[i], t);
        }
        return result;
    }

    private Vector3 WeightedMovingAverage(List<Vector3> window, float[] weights)
    {
        Vector3 sum = Vector3.zero;
        for (int i = 0; i < window.Count; i++)
        {
            sum += window[i] * weights[i];
        }
        return sum;
    }

    private Quaternion WeightedMovingAverage(List<Quaternion> window, float[] weights)
    {
        Vector3 averageForward = Vector3.zero;
        Vector3 averageUpwards = Vector3.zero;
        for (int i = 0; i < window.Count; i++)
        {
            averageForward += (window[i] * Vector3.forward) * weights[i];
            averageUpwards += (window[i] * Vector3.up) * weights[i];
        }
        return Quaternion.LookRotation(averageForward, averageUpwards);
    }

    private float[] GetLinearWeights(int windowSize)
    {
        // Increase wights linearly
        float[] result = new float[windowSize];
        result[0] = 1.0f;
        float sum = 1.0f;
        for (int i = 1; i < windowSize; i++)
        {
            result[i] = 1 + result[i - 1];
            sum += result[i];
        }
        for (int i = 0; i < windowSize; i++)
        {
            result[i] /= sum;
        }
        return result;
    }
}