using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SkeletonRenderer : MonoBehaviour
{
    public enum HandType
    {
        HandLeft,
        HandRight
    }
    public HandType handType;
    private int handIndex = -1;

    public bool renderPhysicsCapsules = true;
    private bool useCustomCapsules = true;
    public bool renderSkeleton = true;

    [SerializeField]
    private OVRHand[] oculus_hands;
    [SerializeField]
    private OVRSkeleton[] oculus_skeletons;
    [SerializeField]
    private List<Transform> leftHandJoints = new List<Transform>();
    [SerializeField]
    private List<Transform> rightHandJoints = new List<Transform>();

    private bool isInitialized = false;

    private List<SegmentVisualization> _segmentVisualizations;
    private List<CapsuleVisualization> _capsuleVisualizations;
    private List<OVRCapsuleVisualization> _ovrCapsuleVisualizations;
    [SerializeField]
    private Material skeletonMaterial;
    [SerializeField]
    private Material capsuleMaterial;

    private float[] _skeleton_capsule_radius;

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

    private void Start()
    {
        if (skeletonMaterial == null)
        {
            skeletonMaterial = new Material(Shader.Find("Diffuse"));
            skeletonMaterial.color = Color.red;
        }
        if (capsuleMaterial == null)
        {
            capsuleMaterial = new Material(Shader.Find("Transparent/Diffuse"));
            capsuleMaterial.color = new Color(1f, 1f, 1f, 0.5f); // Transparent white material
        }

        _segmentVisualizations = new List<SegmentVisualization>();
        _capsuleVisualizations = new List<CapsuleVisualization>();
        _ovrCapsuleVisualizations = new List<OVRCapsuleVisualization>();

        if (handType.ToString() == "HandLeft")
        {
            handIndex = 0;
            for (int i = 0; i < leftHandJoints.Count; i++)
            {
                var start = leftHandJoints[i];
                var end = leftHandJoints[i].parent;
                var capsuleVis = new CapsuleVisualization(i, start.position, end.position, preset_capsule_radius[i], capsuleMaterial);
                _capsuleVisualizations.Add(capsuleVis);
            }
        }
        else if (handType.ToString() == "HandRight")
        {
            handIndex = 1;
            for (int i = 0; i < rightHandJoints.Count; i++)
            {
                var start = rightHandJoints[i];
                var end = rightHandJoints[i].parent;
                var capsuleVis = new CapsuleVisualization(i, start.position, end.position, preset_capsule_radius[i], capsuleMaterial);
                _capsuleVisualizations.Add(capsuleVis);
            }
        }
        Debug.Log("handIndex is " + handIndex);

        // 24 line segments with 24 capsules in total
        _skeleton_capsule_radius = preset_capsule_radius;
    }

    private void Update()
    {
        if (!isInitialized)
        {
            Initialize();
        }
        if (isInitialized)
        {
            //for (int i = 0; i < 1; i++)
            //{
            if (oculus_hands[handIndex].IsTracked && oculus_hands[handIndex].HandConfidence == OVRHand.TrackingConfidence.High)
            {
                if (renderPhysicsCapsules)
                {
                    // Update skeleton capsules
                    if (useCustomCapsules)
                    {
                        for (int j = 0; j < oculus_skeletons[handIndex].Bones.Count; j++)
                        {
                            var start = oculus_skeletons[handIndex].Bones[j].Transform;
                            var end = oculus_skeletons[handIndex].Bones[j].Transform.parent;
                            _capsuleVisualizations[j].Update(start.position, end.position);
                        }
                    }
                    else
                    {
                        for (int j = 0; j < _ovrCapsuleVisualizations.Count; j++)
                        {
                            _ovrCapsuleVisualizations[j].Update(1f);
                        }
                    }
                }

                // Update skeleton line segments
                for (int j = 0; j < oculus_skeletons[handIndex].Bones.Count; j++)
                {
                    _segmentVisualizations[j].Update();
                }
            }
        }
        //}
    }

    private void Initialize()
    {
        if (oculus_hands[handIndex].IsTracked && oculus_hands[handIndex].HandConfidence == OVRHand.TrackingConfidence.High)
        {
            if (useCustomCapsules)
            {
                // Initialize skeleton capsules by customized capsules
                if (renderPhysicsCapsules && oculus_skeletons[handIndex].Bones.Count > 0)
                {
                    Debug.Log("Num of customized capsules in initialization: " + _skeleton_capsule_radius.Length);
                    for (int j = 0; j < oculus_skeletons[handIndex].Bones.Count; j++)
                    {
                        var start = oculus_skeletons[handIndex].Bones[j].Transform;
                        var end = oculus_skeletons[handIndex].Bones[j].Transform.parent;

                        var capsuleVis = new CapsuleVisualization(j, start.position, end.position, _skeleton_capsule_radius[j], capsuleMaterial);
                        _capsuleVisualizations.Add(capsuleVis);
                    }
                    isInitialized = true;
                }
            }
            else
            {
                // Initialize skeleton capsules by Oculus OVRBoneCapsule
                if (renderPhysicsCapsules && oculus_skeletons[handIndex].Capsules.Count > 0)
                {
                    Debug.Log("Num of OVRBoneCapsules in initialization: " + oculus_skeletons[handIndex].Capsules.Count);
                    for (int j = 0; j < oculus_skeletons[handIndex].Capsules.Count; j++)
                    {
                        var capsuleVis = new OVRCapsuleVisualization(oculus_skeletons[handIndex].Capsules[j], capsuleMaterial);
                        _ovrCapsuleVisualizations.Add(capsuleVis);
                    }
                    isInitialized = true;
                }
            }

            // Initialize skeleton line segments
            if (renderSkeleton && oculus_skeletons[handIndex].Bones.Count > 0)
            {
                Debug.Log("Num of Bones in initialization: " + oculus_skeletons[handIndex].Bones.Count);
                for (int j = 0; j < oculus_skeletons[handIndex].Bones.Count; j++)
                {
                    var start = oculus_skeletons[handIndex].Bones[j].Transform;
                    var end = oculus_skeletons[handIndex].Bones[j].Transform.parent;
                    var segmentVis = new SegmentVisualization(start, end, skeletonMaterial);
                    _segmentVisualizations.Add(segmentVis);
                }
                isInitialized = true;
            }
        }
    }

    // Visualize hand skeleton by capsules (custom method)
    public class CapsuleVisualization
    {
        private GameObject CapsuleGO;
        private Vector3 capsuleScale;
        private MeshRenderer Renderer;
        private Material RenderMaterial;
        private float capsult_radius;

        public CapsuleVisualization(int index, Vector3 BoneBegin, Vector3 BoneEnd, float radius, Material renderMat)
        {
            CapsuleGO = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            CapsuleGO.name = "Capsule_" + index;
            CapsuleCollider collider = CapsuleGO.GetComponent<CapsuleCollider>();
            Destroy(collider);
            Renderer = CapsuleGO.GetComponent<MeshRenderer>();
            RenderMaterial = renderMat;
            Renderer.sharedMaterial = RenderMaterial;
            
            capsuleScale = Vector3.one;
            var height = Vector3.Distance(BoneBegin, BoneEnd) + radius * 2.0f;
            capsuleScale.y = height / 2;
            // capsuleScale.y = Vector3.Distance(BoneBegin, BoneEnd) / 2;
            capsuleScale.x = radius * 2;
            capsuleScale.z = radius * 2;
            CapsuleGO.transform.localScale = capsuleScale;

            CapsuleGO.transform.position = (BoneBegin + BoneEnd) / 2;
            Vector3 orientation = (BoneEnd - BoneBegin).normalized;
            CapsuleGO.transform.rotation = Quaternion.LookRotation(orientation) * Quaternion.Euler(90, 0, 0);
        }

        public CapsuleVisualization(float radius, Material renderMat)
        {
            CapsuleGO = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            CapsuleCollider collider = CapsuleGO.GetComponent<CapsuleCollider>();
            Destroy(collider);
            capsult_radius = radius;
            capsuleScale.x = radius * 2;
            capsuleScale.z = radius * 2;
            CapsuleGO.transform.localScale = capsuleScale;

            Renderer = CapsuleGO.GetComponent<MeshRenderer>();
            RenderMaterial = renderMat;
            Renderer.sharedMaterial = RenderMaterial;
        }

        public void Update(Vector3 BoneBegin, Vector3 BoneEnd)
        {
            Vector3 orientation = (BoneEnd - BoneBegin).normalized;
            if (orientation != Vector3.zero)
            {
                CapsuleGO.transform.position = (BoneBegin + BoneEnd) / 2;
                CapsuleGO.transform.rotation = Quaternion.LookRotation(orientation) * Quaternion.Euler(90, 0, 0);
            }
            if (capsult_radius != 0)
            {
                CapsuleGO.transform.localScale = new Vector3(capsuleScale.x, Vector3.Distance(BoneBegin, BoneEnd) / 2 + capsult_radius * 1.0f, capsuleScale.z);
            }
        }
    }

    // Visualize hand skeletion by line segments
    public class SegmentVisualization
    {
        private GameObject BoneGO;
        private Transform BoneBegin;
        private Transform BoneEnd;
        private LineRenderer Line;
        private Material RenderMaterial;
        private const float LINE_RENDERER_WIDTH = 0.004f;

        public SegmentVisualization(Transform begin, Transform end, Material renderMat)
        {
            RenderMaterial = renderMat;

            BoneBegin = begin;
            BoneEnd = end;

            BoneGO = new GameObject(begin.name);
            //BoneGO.transform.SetParent(rootGO.transform, false);

            Line = BoneGO.AddComponent<LineRenderer>();
            Line.sharedMaterial = RenderMaterial;
            Line.useWorldSpace = true;
            Line.positionCount = 2;

            Line.SetPosition(0, BoneBegin.position);
            Line.SetPosition(1, BoneEnd.position);

            Line.startWidth = LINE_RENDERER_WIDTH;
            Line.endWidth = LINE_RENDERER_WIDTH;
        }

        public void Update()
        {
            Line.SetPosition(0, BoneBegin.position);
            Line.SetPosition(1, BoneEnd.position);

            Line.startWidth = LINE_RENDERER_WIDTH;
            Line.endWidth = LINE_RENDERER_WIDTH;

            Line.sharedMaterial = RenderMaterial;
        }
    }

    // Visualize hand skeleton by capsules (oculus default method)
    public class OVRCapsuleVisualization
    {
        private GameObject CapsuleGO;
        private OVRBoneCapsule BoneCapsule;
        private Vector3 capsuleScale;
        private MeshRenderer Renderer;
        private Material RenderMaterial;

        public OVRCapsuleVisualization(OVRBoneCapsule boneCapsule, Material renderMat)
        {
            BoneCapsule = boneCapsule;
            RenderMaterial = renderMat;

            CapsuleGO = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            CapsuleCollider collider = CapsuleGO.GetComponent<CapsuleCollider>();
            Destroy(collider);
            Renderer = CapsuleGO.GetComponent<MeshRenderer>();
            Renderer.sharedMaterial = RenderMaterial;

            capsuleScale = Vector3.one;
            capsuleScale.y = boneCapsule.CapsuleCollider.height / 2;
            capsuleScale.x = boneCapsule.CapsuleCollider.radius * 2;
            capsuleScale.z = boneCapsule.CapsuleCollider.radius * 2;
            CapsuleGO.transform.localScale = capsuleScale;
        }

        public void Update(float scale)
        {
            CapsuleGO.transform.rotation = BoneCapsule.CapsuleCollider.transform.rotation * Quaternion.Euler(0, 0, 90);
            CapsuleGO.transform.position = BoneCapsule.CapsuleCollider.transform.TransformPoint(BoneCapsule.CapsuleCollider.center);
            CapsuleGO.transform.localScale = capsuleScale * scale;
        }
    }
}

