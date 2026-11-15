using UnityEngine;

/// <summary>
/// Lets VR-Doh code, which was written against the legacy 24-bone OVR hand skeleton
/// (OVRSkeleton.BoneId.Hand_*), run on the OpenXR hand skeleton (26 joints, XRHand_*)
/// that Meta XR SDK v74+ uses by default and that the Interaction SDK requires.
///
/// The two enums overlap numerically (e.g. Hand_ThumbTip == XRHand_RingDistal == 19),
/// so comparing a bone's Id against a Hand_* constant on an OpenXR skeleton silently
/// returns the wrong joint. Always go through Map/Find instead.
/// </summary>
public static class HandBoneCompat
{
    // Indexed by legacy OVR BoneId (0..23). Legacy-only joints (ForearmStub, Thumb0 trapezium)
    // map to their nearest OpenXR joint.
    static readonly OVRSkeleton.BoneId[] s_LegacyToXR =
    {
        OVRSkeleton.BoneId.XRHand_Wrist,              // 0  Hand_WristRoot
        OVRSkeleton.BoneId.XRHand_Wrist,              // 1  Hand_ForearmStub (no OpenXR equivalent)
        OVRSkeleton.BoneId.XRHand_ThumbMetacarpal,    // 2  Hand_Thumb0      (no OpenXR equivalent)
        OVRSkeleton.BoneId.XRHand_ThumbMetacarpal,    // 3  Hand_Thumb1
        OVRSkeleton.BoneId.XRHand_ThumbProximal,      // 4  Hand_Thumb2
        OVRSkeleton.BoneId.XRHand_ThumbDistal,        // 5  Hand_Thumb3
        OVRSkeleton.BoneId.XRHand_IndexProximal,      // 6  Hand_Index1
        OVRSkeleton.BoneId.XRHand_IndexIntermediate,  // 7  Hand_Index2
        OVRSkeleton.BoneId.XRHand_IndexDistal,        // 8  Hand_Index3
        OVRSkeleton.BoneId.XRHand_MiddleProximal,     // 9  Hand_Middle1
        OVRSkeleton.BoneId.XRHand_MiddleIntermediate, // 10 Hand_Middle2
        OVRSkeleton.BoneId.XRHand_MiddleDistal,       // 11 Hand_Middle3
        OVRSkeleton.BoneId.XRHand_RingProximal,       // 12 Hand_Ring1
        OVRSkeleton.BoneId.XRHand_RingIntermediate,   // 13 Hand_Ring2
        OVRSkeleton.BoneId.XRHand_RingDistal,         // 14 Hand_Ring3
        OVRSkeleton.BoneId.XRHand_LittleMetacarpal,   // 15 Hand_Pinky0
        OVRSkeleton.BoneId.XRHand_LittleProximal,     // 16 Hand_Pinky1
        OVRSkeleton.BoneId.XRHand_LittleIntermediate, // 17 Hand_Pinky2
        OVRSkeleton.BoneId.XRHand_LittleDistal,       // 18 Hand_Pinky3
        OVRSkeleton.BoneId.XRHand_ThumbTip,           // 19 Hand_ThumbTip
        OVRSkeleton.BoneId.XRHand_IndexTip,           // 20 Hand_IndexTip
        OVRSkeleton.BoneId.XRHand_MiddleTip,          // 21 Hand_MiddleTip
        OVRSkeleton.BoneId.XRHand_RingTip,            // 22 Hand_RingTip
        OVRSkeleton.BoneId.XRHand_LittleTip,          // 23 Hand_PinkyTip
    };

    public const int LegacyBoneCount = 24;

    // Joint-local basis of each convention, expressed as (distal, dorsal):
    //   OVR right: +X, +Y   OVR left: -X, -Y   OpenXR (both hands, Unity space): +Z, +Y
    static readonly Quaternion s_XRToLegacyRight = Quaternion.Inverse(Quaternion.LookRotation(Vector3.right, Vector3.up));
    static readonly Quaternion s_XRToLegacyLeft = Quaternion.Inverse(Quaternion.LookRotation(Vector3.left, Vector3.down));

    public static bool IsOpenXR(OVRSkeleton skeleton)
    {
        if (skeleton == null) return false;
        var t = skeleton.GetSkeletonType();
        return t == OVRSkeleton.SkeletonType.XRHandLeft || t == OVRSkeleton.SkeletonType.XRHandRight;
    }

    static bool IsLeft(OVRSkeleton skeleton)
    {
        var t = skeleton.GetSkeletonType();
        return t == OVRSkeleton.SkeletonType.XRHandLeft || t == OVRSkeleton.SkeletonType.HandLeft;
    }

    /// <summary>Translates a legacy Hand_* id into the id used by this skeleton.</summary>
    public static OVRSkeleton.BoneId Map(OVRSkeleton skeleton, OVRSkeleton.BoneId legacyId)
    {
        int i = (int)legacyId;
        if (!IsOpenXR(skeleton) || i < 0 || i >= s_LegacyToXR.Length) return legacyId;
        return s_LegacyToXR[i];
    }

    /// <summary>Finds the tracked transform for a legacy Hand_* id, or null if not available.</summary>
    public static Transform Find(OVRSkeleton skeleton, OVRSkeleton.BoneId legacyId)
    {
        if (skeleton == null || skeleton.Bones == null) return null;
        var id = Map(skeleton, legacyId);
        var bones = skeleton.Bones;
        for (int i = 0; i < bones.Count; i++)
            if (bones[i].Id == id) return bones[i].Transform;
        return null;
    }

    /// <summary>
    /// World-space pose a legacy-rigged (OVR) hand bone should take, given the live skeleton.
    /// Converts OpenXR joint orientation into the OVR joint orientation the VR-Doh hand mesh was skinned with.
    /// </summary>
    public static bool TryGetLegacyPose(OVRSkeleton skeleton, int legacyIndex, out Vector3 position, out Quaternion rotation)
    {
        position = default;
        rotation = Quaternion.identity;
        var t = Find(skeleton, (OVRSkeleton.BoneId)legacyIndex);
        if (t == null) return false;

        position = t.position;
        rotation = t.rotation;
        if (!IsOpenXR(skeleton)) return true;

        rotation = t.rotation * (IsLeft(skeleton) ? s_XRToLegacyLeft : s_XRToLegacyRight);
        if (legacyIndex == (int)OVRSkeleton.BoneId.Hand_Thumb0)
        {
            // The trapezium has no OpenXR joint: place it between the wrist and the thumb metacarpal.
            var wrist = Find(skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
            if (wrist != null) position = Vector3.Lerp(wrist.position, t.position, 0.5f);
        }
        return true;
    }
}
