using System.Collections.Generic;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// The player's tracked hands, sampled once per frame through <see cref="HandBoneCompat"/> (so the
    /// OpenXR hand skeleton of SDK 209 maps correctly): palm centre (where MochiGrabber puts a held handle),
    /// index fingertip, and whether the hand is the one holding the kine.
    /// </summary>
    public class HandPoints
    {
        public class Hand
        {
            public OVRHand hand;
            public OVRSkeleton skeleton;
            public bool tracked, holding;
            public Vector3 palm, indexTip;
        }

        public readonly List<Hand> Hands = new List<Hand>();
        float nextFind;

        public void Refresh(GrabZone kineZone)
        {
            if (Hands.Count < 2 && Time.unscaledTime >= nextFind)
            {
                nextFind = Time.unscaledTime + 1f;
                foreach (var h in UnityEngine.Object.FindObjectsByType<OVRHand>(FindObjectsInactive.Exclude))
                {
                    bool known = false;
                    foreach (var k in Hands) if (k.hand == h) { known = true; break; }
                    if (!known && Hands.Count < 2) Hands.Add(new Hand { hand = h, skeleton = h.GetComponent<OVRSkeleton>() });
                }
            }
            foreach (var h in Hands)
            {
                h.tracked = false;
                h.holding = false;
                if (h.hand == null || h.skeleton == null || !h.hand.IsTracked || !h.hand.IsDataValid) continue;
                var wrist = HandBoneCompat.Find(h.skeleton, OVRSkeleton.BoneId.Hand_WristRoot);
                var knuckle = HandBoneCompat.Find(h.skeleton, OVRSkeleton.BoneId.Hand_Middle1);
                var tip = HandBoneCompat.Find(h.skeleton, OVRSkeleton.BoneId.Hand_IndexTip);
                if (wrist == null || knuckle == null || tip == null) continue;
                h.tracked = true;
                h.palm = Vector3.Lerp(wrist.position, knuckle.position, 0.7f); // same palm point as MochiGrabber
                h.indexTip = tip.position;
                h.holding = kineZone != null && kineZone.Held && kineZone.Distance(h.palm) < 0.08f;
            }
        }
    }
}
