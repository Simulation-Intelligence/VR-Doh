using UnityEngine;

public class MpmHand : MpmTool
{
    // Hand as a tool
    public enum HandType
    {
        LeftHand,
        RightHand
    }
    public HandType handType;
    [SerializeField]
    private OVRHand oculus_hand;
    [SerializeField]
    private OVRSkeleton oculus_skeleton;

    public static readonly float[] preset_capsule_radius =  { 0,
                                              0,
                                              0,
                                              0.015382f,
                                              0.013382f,
                                              0.01028295f,
                                              0.014f,
                                              0.01029526f,
                                              0.008038102f,
                                              0.019f,
                                              0.011f,
                                              0.008030958f,
                                              0.01608828f,
                                              0.009922137f,
                                              0.007611672f,
                                              0.013f,
                                              0.013f,
                                              0.008483353f,
                                              0.006764194f,
                                              0.0090f,
                                              0.007636196f,
                                              0.007629411f,
                                              0.007231089f,
                                              0.006425985f };


    void Awake()
    {
        // Set the valus of numCapsules and init_capsules in Awake() method
        numCapsules = 24;
        init_capsules = new Capsule[numCapsules];
        for (int i = 0; i < numCapsules; i++)
        {
            init_capsules[i] = new Capsule() { radius = preset_capsule_radius[i] };
        }
        
        // Oculus hands
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
    }

    protected override void UpdateCapsules()
    {
        if (oculus_hand.IsTracked && oculus_hand.HandConfidence == OVRHand.TrackingConfidence.High)
        {
            int numBones = oculus_skeleton.Bones.Count;
            //UnityEngine.Debug.Log("Num of Bones while tracking: " + numBones);
            if (numBones > 0)
            {
                for (int j = 0; j < numBones; j++)
                {
                    OVRBone bone = oculus_skeleton.Bones[j];
                    capsules[j].start = bone.Transform.position;
                    capsules[j].end = bone.Transform.parent.position;
                    
                    // Adjust capsules for the finger tips
                    if (j == 19 || j == 20 || j == 21 || j == 22 || j == 23)
                    {
                        Vector3 start = bone.Transform.position;
                        Vector3 end = bone.Transform.parent.position;
                        Vector3 direction = (end - start).normalized;
                        capsules[j].start = start + direction * preset_capsule_radius[j];
                        // capsules[j].end = end - direction * preset_capsule_radius[j];
                    }
                }
            }
        }
    }
}