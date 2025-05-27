using System;
using System.Linq;
using UnityEngine;
using System.Collections.Generic;
using Oculus.Interaction;
using Oculus.Interaction.Input;

public class MatHand : MatTool
{
    private HandJoint handJoint;
    [SerializeField]
    private HandJointId _handJointId;
    private OVRHand oculus_hand;
    private OVRSkeleton oculus_skeleton;

    // Map between primitive index and joint index
    Dictionary<int, int> primitivesJointMap = new Dictionary<int, int>();
    private Dictionary<int, Vector3[]> primitiveOffsets = new Dictionary<int, Vector3[]>();
    public struct PrimitiveBinding
    {
        public int jointStartIndex;
        public int jointEndIndex;
        public float interpolation;
    }
    private Dictionary<int, PrimitiveBinding> primitivesJointBindings = new Dictionary<int, PrimitiveBinding>();
    [SerializeField]
    private List<Transform> leftHandJoints = new List<Transform>();
    [SerializeField]
    private List<Transform> rightHandJoints = new List<Transform>();

    // These primitives are removed from the hand_mat.json
    private int[] removedPrimitives = new int[] { 2, 8, 12, 19, 42, 48, 58, 65, 98 };

    void Awake()
    {
        // Oculus hands
        if (handType == HandType.LeftHand)
        {
            oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRSkeleton>();
            _handJointsData = smoothHand.SmoothLeftHandJoints; // Inherited from the parent class
        }
        else if (handType == HandType.RightHand)
        {
            oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRSkeleton>();
            _handJointsData = smoothHand.SmoothRightHandJoints; // Inherited from the parent class
        }

        // Initialization with hand_mat.json
        bool isLeftHand = handType == HandType.LeftHand;
        LoadPrimitivesFromJson("Prefabs/Tools/hand_mat_2", isLeftHand);

        RemoveNoUsedPrimitives();
        CalculateInteractionForLerp(isLeftHand);
        AdjustPrimitivesJointBindings();
        CalculateOffsetForLerp(isLeftHand);
    }

    // protected override void UpdatePrimitives()
    // {
    //     // Update mat hand without hand tracking
    //     if (oculus_hand.IsTracked)
    //     {
    //         foreach (var bone in oculus_skeleton.Bones)
    //         {
    //             var jointId = _handJointId.ToString().Replace("Hand", "Hand_");
    //             if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId))
    //             {
    //                 transform.position = bone.Transform.position;
    //                 transform.rotation = bone.Transform.rotation * _rotationOffset;

    //                 for (int i = 0; i < numPrimitives; i++)
    //                 {
    //                     // Update primitive position using the oculus Hand Joint Component
    //                     primitives[i].sphere1 = transform.TransformPoint(init_primitives[i].sphere1);
    //                     primitives[i].sphere2 = transform.TransformPoint(init_primitives[i].sphere2);
    //                     primitives[i].sphere3 = transform.TransformPoint(init_primitives[i].sphere3);

    //                     // Multiply by localScale to get the correct radius
    //                     primitives[i].radii1 = init_primitives[i].radii1 * transform.localScale.x;
    //                     primitives[i].radii2 = init_primitives[i].radii2 * transform.localScale.x;
    //                     primitives[i].radii3 = init_primitives[i].radii3 * transform.localScale.x;
    //                 }
    //                 break;
    //             }
    //         }
    //     }
    // }

    // protected override void UpdatePrimitives()
    // {
    //     // Update mat hand without hand tracking
    //     if (oculus_hand.IsTracked)
    //     {
    //         int numBones = oculus_skeleton.Bones.Count;
    //         if (numBones > 0)
    //         {
    //             foreach (var entry in primitivesJointMap)
    //             {
    //                 int primitiveIndex = entry.Key;
    //                 int jointIndex = entry.Value;
    //                 OVRBone bone = oculus_skeleton.Bones[jointIndex];

    //                 Vector3[] offsets = primitiveOffsets[primitiveIndex];
    //                 primitives[primitiveIndex].sphere1 = bone.Transform.parent.TransformPoint(offsets[0]);
    //                 primitives[primitiveIndex].sphere2 = bone.Transform.parent.TransformPoint(offsets[1]);
    //                 primitives[primitiveIndex].sphere3 = bone.Transform.parent.TransformPoint(offsets[2]);
    //             }
    //         }
    //     }
    // }

    protected override void UpdatePrimitives()
    {
        // Update mat hand without hand tracking
        //if (oculus_hand.IsTracked)
        {
            // int numBones = oculus_skeleton.Bones.Count;
            int numBones = _handJointsData.Count;
            if (numBones > 0)
            {
                foreach (var entry in primitivesJointBindings)
                {
                    int primitiveIndex = entry.Key;
                    PrimitiveBinding binding = entry.Value;
                    // Transform jointStart = oculus_skeleton.Bones[binding.jointStartIndex].Transform;
                    Transform jointStart = _handJointsData[binding.jointStartIndex];
                    // Transform jointEnd = (binding.jointEndIndex >= 0) ? oculus_skeleton.Bones[binding.jointEndIndex].Transform : jointStart;
                    Transform jointEnd = (binding.jointEndIndex >= 0) ? _handJointsData[binding.jointEndIndex] : jointStart;
                    
                    // Lerp between jointStart and jointEnd
                    Vector3 primitiveLerpPosition = Vector3.Lerp(jointStart.position, jointEnd.position, binding.interpolation);
                    Vector3[] offsets = primitiveOffsets[primitiveIndex];
                    primitives[primitiveIndex].sphere1 = jointEnd.rotation * offsets[0] + primitiveLerpPosition;
                    primitives[primitiveIndex].sphere2 = jointEnd.rotation * offsets[1] + primitiveLerpPosition;
                    primitives[primitiveIndex].sphere3 = jointEnd.rotation * offsets[2] + primitiveLerpPosition;
                }
            }
        }
    }

    void RemoveNoUsedPrimitives()
    {
        List<Primitive> primitiveList = new List<Primitive>(init_primitives);
        var sortedRemovedPrimitives = new List<int>(removedPrimitives);
        sortedRemovedPrimitives.Sort((a, b) => b.CompareTo(a));
        foreach (int index in sortedRemovedPrimitives)
        {
            if (index < primitiveList.Count)
            {
                primitiveList.RemoveAt(index);
            }
        }
        init_primitives = primitiveList.ToArray();
        numPrimitives = init_primitives.Length;
        Debug.Log("Remaining primitives count: " + numPrimitives);
    }

    void AdjustPrimitivesJointMap()
    {
        var keys = primitivesJointMap.Keys.ToList();
        for (int i = 0; i < keys.Count; i++)
        {
            int primitiveIndex = keys[i];
            if (primitiveIndex == 69)
            {
                primitivesJointMap[primitiveIndex] = 20;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 28)
            {
                primitivesJointMap[primitiveIndex] = 4;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 44)
            {
                primitivesJointMap[primitiveIndex] = 6;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 93)
            {
                primitivesJointMap[primitiveIndex] = 12;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 94)
            {
                primitivesJointMap[primitiveIndex] = 9;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 71)
            {
                primitivesJointMap[primitiveIndex] = 16;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 64)
            {
                primitivesJointMap[primitiveIndex] = 12;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 18)
            {
                primitivesJointMap[primitiveIndex] = 16;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 92)
            {
                primitivesJointMap[primitiveIndex] = 12;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 65)
            {
                primitivesJointMap[primitiveIndex] = 9;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 66)
            {
                primitivesJointMap[primitiveIndex] = 12;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 38)
            {
                primitivesJointMap[primitiveIndex] = 2;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
            if (primitiveIndex == 19)
            {
                primitivesJointMap[primitiveIndex] = 16;
                // Debug.Log("Primitive " + primitiveIndex + " is closest to " + primitivesJointMap[primitiveIndex]);
            }
        }
    }

    void AdjustPrimitivesJointBindings()
    {
        var keys = primitivesJointBindings.Keys.ToList();
        for (int i = 0; i < keys.Count; i++)
        {
            int primitiveIndex = keys[i];
            if (primitiveIndex == 3)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 11;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 18)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 16;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 28)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 4;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 30)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 13;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 43)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 44)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 6;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 56)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 64 || primitiveIndex == 65 || primitiveIndex == 66)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 12;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 69)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 20;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 71)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 16;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 87)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 89)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 6;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 92)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 93)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 12;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 94)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
            if (primitiveIndex == 103)
            {
                PrimitiveBinding binding = primitivesJointBindings[primitiveIndex];
                binding.jointStartIndex = 9;
                binding.jointEndIndex = GetParentJointIndex(binding.jointStartIndex);
                primitivesJointBindings[primitiveIndex] = binding;
            }
        }
    }

    void CalculateOffsetForLerp(bool isLeftHand)
    {
        List<Transform> handJoints = isLeftHand ? leftHandJoints : rightHandJoints;
        foreach (var entry in primitivesJointBindings)
        {
            int primitiveIndex = entry.Key;
            PrimitiveBinding binding = entry.Value;
            Transform jointStart = handJoints[binding.jointStartIndex];
            Transform jointEnd = handJoints[binding.jointEndIndex];
            Vector3[] offsets = new Vector3[3];
            Vector3 interpolatedPosition = Vector3.Lerp(jointStart.position, jointEnd.position, binding.interpolation);
            offsets[0] = Quaternion.Inverse(jointEnd.rotation) * (init_primitives[primitiveIndex].sphere1 - interpolatedPosition);
            offsets[1] = Quaternion.Inverse(jointEnd.rotation) * (init_primitives[primitiveIndex].sphere2 - interpolatedPosition);
            offsets[2] = Quaternion.Inverse(jointEnd.rotation) * (init_primitives[primitiveIndex].sphere3 - interpolatedPosition);
            primitiveOffsets[primitiveIndex] = offsets;
        }
    }
    
    void CalculateInteractionForLerp(bool isLeftHand)
    {
        Dictionary<int, float> minDistances = new Dictionary<int, float>();
        primitivesJointMap.Clear();
        primitivesJointBindings.Clear();
        List<Transform> handJoints = isLeftHand ? leftHandJoints : rightHandJoints;
        for (int i = 0; i < rightHandJoints.Count; i++)
        {
            Vector3 start = handJoints[i].position;
            Vector3 end = handJoints[i].parent.position;
            int parentIndex = GetParentJointIndex(i);
            for (int j = 0; j < numPrimitives; j++)
            {
                var primitive = init_primitives[j];
                bool intersect = LinePrimitiveIntersection(start, end, primitive, out float distance);
                if (!minDistances.ContainsKey(j) || distance < minDistances[j])
                {
                    minDistances[j] = distance;
                    float interpolation = CalculateInterpolation(primitive, start, end);
                    primitivesJointBindings[j] = new PrimitiveBinding
                    {
                        jointStartIndex = i,
                        jointEndIndex = parentIndex,
                        interpolation = interpolation
                    };
                }
            }
        }
        // foreach (var e in primitivesJointBindings.OrderBy(e => e.Key))
        // {
        //     Debug.Log("Primitive " + e.Key + " is closest to start index " + e.Value.jointStartIndex + " and end index " + e.Value.jointEndIndex + " with interpolation " + e.Value.interpolation);
        // }
        Debug.Log("Intersection calculation done with:" + primitivesJointBindings.Count);
    }

    float CalculateInterpolation(Primitive primitive, Vector3 start, Vector3 end)
    {
        Vector3 primitiveCenter = (primitive.sphere1 + primitive.sphere2 + primitive.sphere3) / 3;
        Vector3 line = (end - start).normalized;
        float length = Vector3.Distance(start, end);
        float projection = Vector3.Dot(primitiveCenter - start, line);
        return Mathf.Clamp01(projection / length);
    }

    int GetParentJointIndex(int jointIndex)
    {
        Transform currentJoint = rightHandJoints[jointIndex];
        Transform parentJoint = currentJoint.parent;
        for (int i = 0; i < rightHandJoints.Count; i++)
        {
            if (rightHandJoints[i] == parentJoint)
            {
                return i;
            }
        }
        return -1;
    }

    void CalculateOffset()
    {
        foreach (var entry in primitivesJointMap)
        {
            int primitiveIndex = entry.Key;
            int jointIndex = entry.Value;
            Transform bone = rightHandJoints[jointIndex];
            Vector3[] offsets = new Vector3[3];
            offsets[0] = bone.parent.InverseTransformPoint(init_primitives[primitiveIndex].sphere1);
            offsets[1] = bone.parent.InverseTransformPoint(init_primitives[primitiveIndex].sphere2);
            offsets[2] = bone.parent.InverseTransformPoint(init_primitives[primitiveIndex].sphere3);
            primitiveOffsets[primitiveIndex] = offsets;
        }
    }

    void CalculateIntersection()
    {
        Dictionary<int, float> minDistances = new Dictionary<int, float>();
        primitivesJointMap.Clear();
        for (int i = 0; i < rightHandJoints.Count; i++)
        {
            Vector3 start = rightHandJoints[i].position;
            Vector3 end = rightHandJoints[i].parent.position;
            for (int j = 0; j < numPrimitives; j++)
            {
                var primitive = init_primitives[j];
                bool intersect = LinePrimitiveIntersection(start, end, primitive, out float distance);
                if (intersect)
                {
                    if (!minDistances.ContainsKey(j) || distance < minDistances[j])
                    {
                        minDistances[j] = distance;
                        primitivesJointMap[j] = i;
                    }
                }
            }
        }
        foreach (var e in primitivesJointMap.OrderBy(e => e.Key))
        {
            Debug.Log("Primitive " + e.Key + " is closest to rightHandJoint index " + e.Value);
        }
        Debug.Log("Intersection calculation done with:" + minDistances.Count);
    }

    bool LinePrimitiveIntersection(Vector3 start, Vector3 end, Primitive primitive, out float shortestDistance)
    {
        shortestDistance = float.MaxValue;
        bool intersected = false;
        Vector3[] spheres = { primitive.sphere1, primitive.sphere2, primitive.sphere3 };
        float[] radii = { primitive.radii1, primitive.radii2, primitive.radii3 };

        for (int i = 0; i < 3; i++)
        {
            Vector3 sphere = spheres[i];
            float radius = radii[i];
            if (radius == 0)
            {
                continue;
            }
            if (LineSphereIntersection(start, end, sphere, radius, out float distance))
            {
                intersected = true;
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                }
            }
        }
        return intersected;
    }

    bool LineSphereIntersection(Vector3 start, Vector3 end, Vector3 sphereCenter, float sphereRadius, out float distance)
    {
        Vector3 line = end - start;
        Vector3 lineToSphere = sphereCenter - start;
        float t = Vector3.Dot(lineToSphere, line) / Vector3.Dot(line, line);
        t = Mathf.Clamp01(t);
        Vector3 closestPoint = start + t * line;
        distance = Vector3.Distance(closestPoint, sphereCenter);
        return distance <= sphereRadius;
    }
}