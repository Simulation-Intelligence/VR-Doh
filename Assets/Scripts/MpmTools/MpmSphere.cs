using System;
using UnityEngine;
using Oculus.Interaction;
using Oculus.Interaction.Input;

public class MpmSphere : MpmTool
{
    public float radius = 0.1f;
    private HandJoint handJoint;
    public enum HandType
    {
        LeftHand,
        RightHand
    }
    public HandType handType;
    [SerializeField]
    private HandJointId _handJointId;
    private OVRHand oculus_hand;
    private OVRSkeleton oculus_skeleton;
    private Transform sphereTransform;
    void Awake()
    {
        // Set the valus of numCapsules and init_capsules in Awake() method
        numCapsules = 1;
        init_capsules = new Capsule[numCapsules];
        
        // Initialize positions and radius for each Capsule
        init_capsules[0].start = new Vector3(0, 0, 0);
        init_capsules[0].end = new Vector3(0, 0, 0);
        init_capsules[0].radius = radius;

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
    protected override void UpdateCapsules()
    {
        // Update the position of the sphere
        if (oculus_hand.IsTracked)
        {
            foreach (var bone in oculus_skeleton.Bones)
            {
                var jointId = _handJointId.ToString().Replace("Hand", "Hand_");
                if (bone.Id == (OVRSkeleton.BoneId)Enum.Parse(typeof(OVRSkeleton.BoneId), jointId))
                {
                    for (int i = 0; i < numCapsules; i++)
                    {
                        capsules[i].start = bone.Transform.position;
                        capsules[i].end = bone.Transform.position;
                    }
                    break;
                }
            }
        }
    }
}