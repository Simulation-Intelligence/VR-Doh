using System;
using System.Linq;
using UnityEngine;
using System.Collections.Generic;
using Oculus.Interaction;
using Oculus.Interaction.Input;

public class MatScissorCylinder : MatTool
{
    [SerializeField]
    private HandJointId _handJointId1, _handJointId2;
    private OVRHand oculus_hand;
    private OVRSkeleton oculus_skeleton;

    private Vector3 sphere11 = new Vector3(0, 0, 0);
    private float radii11 = 0.04f;
    private Vector3 sphere12 = new Vector3(1.0f, 0, 0.0f);
    private float radii12 = 0.04f;
    // Note: Set sphere3 and radii3 to 0.0f to treat it as a cone in the system
    private Vector3 sphere13 = new Vector3(0, 0, 0);
    private float radii13 = 0.0f;

    private Vector3 sphere21 = new Vector3(0, 0, 0);
    private float radii21 = 0.04f;
    private Vector3 sphere22 = new Vector3(1.0f, 0, 0.0f);
    private float radii22 = 0.04f;
    // Note: Set sphere3 and radii3 to 0.0f to treat it as a cone in the system
    private Vector3 sphere23 = new Vector3(0, 0, 0);
    private float radii23 = 0.0f;

    // Map between primitive index and joint index
    void Awake()
    {
        numPrimitives = 2;
        init_primitives = new Primitive[numPrimitives];

        // Initialize positions and radius
        init_primitives[0].sphere1 = sphere11;
        init_primitives[0].radii1 = radii11;
        init_primitives[0].sphere2 = sphere12;
        init_primitives[0].radii2 = radii12;
        init_primitives[0].sphere3 = sphere13;
        init_primitives[0].radii3 = radii13;

        init_primitives[1].sphere1 = sphere21;
        init_primitives[1].radii1 = radii21;
        init_primitives[1].sphere2 = sphere22;
        init_primitives[1].radii2 = radii22;
        init_primitives[1].sphere3 = sphere23;
        init_primitives[1].radii3 = radii23;
        // Oculus hands
        if (handType == HandType.LeftHand)
        {
            oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRSkeleton>();
        }
        else if (handType == HandType.RightHand)
        {
            oculus_hand = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRSkeleton>();
        }
    }

    protected override void UpdatePrimitives()
    {
        // Update mat hand without hand tracking
        if (oculus_hand.IsTracked)
        {
            Transform transform1 = new GameObject().transform;
            Transform transform2 = new GameObject().transform;
            var jointId1 = _handJointId1.ToString().Replace("Hand", "Hand_");
            var jointId2 = _handJointId2.ToString().Replace("Hand", "Hand_");
            foreach (var bone in oculus_skeleton.Bones)
            {

                if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId1))
                {

                    transform1.position = bone.Transform.position;
                    transform1.rotation = bone.Transform.rotation;
                    transform1.localScale = transform.localScale;
                    //
                }
                if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId2))
                {

                    transform2.position = bone.Transform.position;
                    transform2.rotation = bone.Transform.rotation;
                    transform2.localScale = transform.localScale;

                }

            }

            Vector3 point1 = transform1.position;
            Vector3 point2 = transform2.position;
            Vector3 p12 = point2 - point1;
            Vector3 normal = Vector3.Cross(transform1.right, p12);
            Vector3 perpendicular = Vector3.Cross(normal, p12);
            Vector3 midPoint = (point1 + point2) / 2;
            float len = p12.magnitude / 2;
            float new_len = Mathf.Sqrt(transform.localScale.x * transform.localScale.x / 4 - len * len);
            Vector3 cross_point = midPoint + perpendicular.normalized * new_len;
            {
                Vector3 xAxis = cross_point - point1;
                Vector3 zAxis = normal;
                Vector3 yAxis = Vector3.Cross(zAxis, xAxis).normalized;
                zAxis = Vector3.Cross(xAxis, yAxis).normalized;  // 确保 Z 轴正交
                Quaternion targetRotation = Quaternion.LookRotation(zAxis, yAxis);
                transform1.rotation = targetRotation;
                UpdatePrimitive(ref primitives[0], init_primitives[0], transform1);
            }
            {
                Vector3 xAxis = cross_point - point2;
                Vector3 zAxis = normal;
                Vector3 yAxis = Vector3.Cross(zAxis, xAxis).normalized;
                zAxis = Vector3.Cross(xAxis, yAxis).normalized;  // 确保 Z 轴正交
                Quaternion targetRotation = Quaternion.LookRotation(zAxis, yAxis);
                transform2.rotation = targetRotation;
                UpdatePrimitive(ref primitives[1], init_primitives[1], transform2);
            }

        }
    }
}