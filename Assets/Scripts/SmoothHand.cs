using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine.Rendering;  // To handle file writing and reading

// Runs after OVRHand/OVRSkeleton (order 0) refresh this frame's bones and before MatHand (60) and the MPM sim (100) read them.
[DefaultExecutionOrder(50)]
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

    public enum MovingAverage { None, Simple, Weighted, TimeScaled, OneEuro }
    public MovingAverage movingAverage = MovingAverage.Weighted;

    [Range(1, 20)]
    public int windowSize = 5;
    [Range(0.01f, 1.0f)]
    public float timeWindow = 0.25f;
    private float[] weights;

    [Header("One-Euro filter (MovingAverage = OneEuro)")]
    [Tooltip("Cutoff frequency at rest, Hz. Lower = steadier when still.")]
    public float minCutoff = 2.0f;
    [Tooltip("How fast the cutoff rises with speed (Hz per m/s). Higher = less lag on fast moves.")]
    public float beta = 25f;
    [Tooltip("Cutoff for the speed estimate, Hz.")]
    public float derivativeCutoff = 1.0f;
    [Tooltip("Keep following the hand when tracking confidence drops (fast moves), with heavier smoothing, instead of freezing it in place.")]
    public bool acceptLowConfidence = true;
    [Tooltip("Multiplier on minCutoff/beta while confidence is low.")]
    public float lowConfidenceScale = 0.4f;
    Vector3[] euroPos, euroVel; Quaternion[] euroRot; bool[] euroInit;
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
            case MovingAverage.OneEuro:
                for (int i = 0; i < numBones; i++) positionQueue[i] = null; // keeps positionQueue.Count == numBones for the loop below
                break;
        }
        euroPos = new Vector3[numBones]; euroVel = new Vector3[numBones]; euroRot = new Quaternion[numBones]; euroInit = new bool[numBones];
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
        else if (oculus_hand.IsTracked && (oculus_hand.HandConfidence == OVRHand.TrackingConfidence.High || acceptLowConfidence))
        {
            bool lowConfidence = oculus_hand.HandConfidence != OVRHand.TrackingConfidence.High;
            float dt = Mathf.Max(Time.unscaledDeltaTime, 1e-4f);
            // Drive the 24 legacy-OVR-rigged joints, whatever skeleton version the runtime reports
            // (Meta XR SDK v74+ defaults to the 26-joint OpenXR skeleton).
            int numBones = Mathf.Min(positionQueue.Count, HandBoneCompat.LegacyBoneCount);
            float currentTime = Time.time;

            for (int i = 0; i < numBones; i++)
            {
                // Update queue data
                if (!HandBoneCompat.TryGetLegacyPose(oculus_skeleton, i, out Vector3 newPosition, out Quaternion newRotation))
                    continue;
                if (movingAverage == MovingAverage.OneEuro)
                {
                    OneEuro(i, newPosition, newRotation, dt, lowConfidence, out Vector3 fp, out Quaternion fr);
                    var joints = handType == HandType.LeftHand ? leftHandJoints : rightHandJoints;
                    joints[i].SetPositionAndRotation(fp, fr);
                    if (record_data) RecordHandJointsData(newPosition, newRotation, i);
                    continue;
                }
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
    static float Alpha(float cutoff, float dt) { float tau = 1f / (2f * Mathf.PI * cutoff); return 1f / (1f + tau / dt); }

    /// <summary>One-Euro filter (Casiez et al. 2012): heavy smoothing at rest, little lag at speed.</summary>
    void OneEuro(int i, Vector3 x, Quaternion q, float dt, bool low, out Vector3 outPos, out Quaternion outRot)
    {
        if (!euroInit[i] || (x - euroPos[i]).sqrMagnitude > 0.25f)   // first sample, or a >0.5 m jump (tracking re-acquired)
        {
            euroInit[i] = true; euroPos[i] = x; euroVel[i] = Vector3.zero; euroRot[i] = q;
            outPos = x; outRot = q; return;
        }
        float scale = low ? lowConfidenceScale : 1f;
        Vector3 rawVel = (x - euroPos[i]) / dt;
        euroVel[i] = Vector3.Lerp(euroVel[i], rawVel, Alpha(derivativeCutoff, dt));
        float cutoff = minCutoff * scale + beta * scale * euroVel[i].magnitude;
        float a = Alpha(cutoff, dt);
        euroPos[i] = Vector3.Lerp(euroPos[i], x, a);
        if (Quaternion.Dot(euroRot[i], q) < 0f) q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
        euroRot[i] = Quaternion.Slerp(euroRot[i], q, a);
        outPos = euroPos[i]; outRot = euroRot[i];
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