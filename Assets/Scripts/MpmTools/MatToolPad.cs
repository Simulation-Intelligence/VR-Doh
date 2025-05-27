using System;
using UnityEngine;
using Oculus.Interaction;
using Oculus.Interaction.Input;

public class MatToolPad : MatTool
{
    // Modified cooridinate system and radius, i.e., the x value of sphere2 is inversed
    // A pad consists of two slabs
    private Vector3 sphere1_1 = new Vector3(0, 0, 0);
    private float radii1_1 = 0.1f;
    private Vector3 sphere1_2 = new Vector3(-1.0f, 0, 0);
    private float radii1_2 = 0.1f;
    private Vector3 sphere1_3 = new Vector3(0, 0, 1.0f);
    private float radii1_3 = 0.1f;

    private Vector3 sphere2_1 = new Vector3(-1.0f, 0, 0);
    private float radii2_1 = 0.1f;
    private Vector3 sphere2_2 = new Vector3(-1.0f, 0, 1.0f);
    private float radii2_2 = 0.1f;
    private Vector3 sphere2_3 = new Vector3(0, 0, 1.0f);
    private float radii2_3 = 0.1f;
    
    private HandJoint handJoint;
    [SerializeField]
    private HandJointId _handJointId;
    private OVRHand oculus_hand;
    private OVRSkeleton oculus_skeleton;

    void Awake()
    {
        numPrimitives = 2;
        init_primitives = new Primitive[numPrimitives];
        
        // Initialize positions and radius
        init_primitives[0].sphere1 = sphere1_1;
        init_primitives[0].radii1 = radii1_1;
        init_primitives[0].sphere2 = sphere1_2;
        init_primitives[0].radii2 = radii1_2;
        init_primitives[0].sphere3 = sphere1_3;
        init_primitives[0].radii3 = radii1_3;

        init_primitives[1].sphere1 = sphere2_1;
        init_primitives[1].radii1 = radii2_1;
        init_primitives[1].sphere2 = sphere2_2;
        init_primitives[1].radii2 = radii2_2;
        init_primitives[1].sphere3 = sphere2_3;
        init_primitives[1].radii3 = radii2_3;

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
    }
    
    protected override void UpdatePrimitives()
    {
        // Update Gameobject Transform
        if (oculus_hand.IsTracked)
        {
            var jointId = _handJointId.ToString().Replace("Hand", "Hand_");

            // (1) Use the OVRSkeleton to update the primitive position (Unsmoothed)
            // foreach (var bone in oculus_skeleton.Bones)
            // {
            //     if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId))
            //     {
            //         transform.position = bone.Transform.position;
            //         transform.rotation = bone.Transform.rotation;
                    
            //         for (int i = 0; i < numPrimitives; i++)
            //         {
            //             // Update primitive position using the oculus Hand Joint Component
            //             primitives[i].sphere1 = transform.TransformPoint(init_primitives[i].sphere1);
            //             primitives[i].sphere2 = transform.TransformPoint(init_primitives[i].sphere2);
            //             primitives[i].sphere3 = transform.TransformPoint(init_primitives[i].sphere3);
                        
            //             // Multiply by localScale to get the correct radius
            //             primitives[i].radii1 = init_primitives[i].radii1 * transform.localScale.x;
            //             primitives[i].radii2 = init_primitives[i].radii2 * transform.localScale.x;
            //             primitives[i].radii3 = init_primitives[i].radii3 * transform.localScale.x;
            //         }
            //         break;
            //     }
            // }

            // (2) Use the SmoothHand to update the primitive position
            for (int i = 0; i < oculus_skeleton.Bones.Count; i++)
            {
                OVRBone bone = oculus_skeleton.Bones[i];
                if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId))
                {
                    // Rotate 90 degrees around the x-axis to align with the hand joint
                    transform.position = _handJointsData[i].position;
                    transform.rotation = _handJointsData[i].rotation * _rotationOffset;

                    for (int j = 0; j < numPrimitives; j++)
                    {
                        // Update primitive position using the oculus Hand Joint Component
                        UpdatePrimitive(ref primitives[j], init_primitives[j], transform);
                    }
                    break;
                }
            }
        }
    }
}