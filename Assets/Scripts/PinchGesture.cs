using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public class PinchGesture : MonoBehaviour
{
    public enum HandType { LeftHand, RightHand }

    public HandType handType = HandType.RightHand;
    private OVRHand hand;
    private OVRSkeleton oculus_skeleton;

    // 手势检测的指尖选择，直接使用 BoneId
    public OVRSkeleton.BoneId firstPinchFinger = OVRSkeleton.BoneId.Hand_ThumbTip;
    public OVRSkeleton.BoneId secondPinchFinger = OVRSkeleton.BoneId.Hand_MiddleTip;

    // 旋转手势的指尖选择
    public OVRSkeleton.BoneId firstRotateFinger = OVRSkeleton.BoneId.Hand_IndexTip;
    public OVRSkeleton.BoneId secondRotateFinger = OVRSkeleton.BoneId.Hand_ThumbTip;

    // 旋转手势的旋转轴计算所需的关节
    public OVRSkeleton.BoneId firstJointBone = OVRSkeleton.BoneId.Hand_Middle1;
    public OVRSkeleton.BoneId secondJointBone = OVRSkeleton.BoneId.Hand_Ring1;

    // 握拳检测所需的指尖和关节
    private List<OVRSkeleton.BoneId> squeezeFingerTips = new List<OVRSkeleton.BoneId>
    {
        OVRSkeleton.BoneId.Hand_ThumbTip,
        OVRSkeleton.BoneId.Hand_IndexTip,
        OVRSkeleton.BoneId.Hand_MiddleTip,
        OVRSkeleton.BoneId.Hand_RingTip,
        OVRSkeleton.BoneId.Hand_PinkyTip
    };

    private List<OVRSkeleton.BoneId> squeezeFingerBases = new List<OVRSkeleton.BoneId>
    {
        OVRSkeleton.BoneId.Hand_Thumb1,
        OVRSkeleton.BoneId.Hand_Index1,
        OVRSkeleton.BoneId.Hand_Middle1,
        OVRSkeleton.BoneId.Hand_Ring1,
        OVRSkeleton.BoneId.Hand_Pinky1
    };

    [SerializeField]
    private SmoothHand smoothHand;
    private List<Transform> _handJointsData;
    public bool UseSmoothHand = true;

    [HideInInspector] public bool isPinching = false;
    [HideInInspector] public bool isRotating = false;

    [HideInInspector] public Vector3 initialPinchPosition;
    [HideInInspector] public Vector3 pinchMovement;
    [HideInInspector] public Vector3 lastPinchPosition;
    [HideInInspector] public Vector3 pinchSpeed;

    [HideInInspector] public Vector3 initialRotatePosition;
    [HideInInspector] public Vector3 rotationAxis;
    [HideInInspector] public float rotationSpeed;

    [HideInInspector] private float previousAngle1 = 0f;
    [HideInInspector] private float previousAngle2 = 0f;

    [HideInInspector] public Vector3 initialDirectionJoint1;
    [HideInInspector] public Vector3 initialDirectionJoint2;

    // 新增握拳检测相关变量
    [HideInInspector] public bool isSqueezing = false;
    [HideInInspector] public Vector3 squeezeCenter;
    [HideInInspector] public Vector3 squeezeDirection;

    public float pinchThreshold = 0.02f;
    public float rotationThreshold = 0.03f;
    public float pinchRadius = 0.05f;

    public float squeezeRadius = 0.02f;

    // 握拳检测阈值
    public float squeezeThresholdLow = 0.04f;

    public float squeezeThresholdHigh = 0.06f;

    [HideInInspector]
    public float squeeze_ratio = 0.5f;

    // Visualize the selection area while pinch translation and rotation
    private GameObject pinchSphere;
    private GameObject rotationSphere;
    public bool RenderPinchSphere = true;
    public bool RenderRotationSphere = true;

    // 可视化握拳中心和朝向
    private GameObject squeezeSphere;
    public bool RenderSqueezeSphere = true;
    // 可视化握拳朝向的圆锥体
    private GameObject squeezeCone;
    public bool RenderSqueezeCone = true;
    public float coneHeight = 0.1f;
    public float coneRadius = 0.02f;

    // 材质
    private Material transparentMaterial;

    void Start()
    {
        if (handType == HandType.LeftHand)
        {
            hand = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/LeftHandAnchor/LeftOVRHand").GetComponent<OVRSkeleton>();
            _handJointsData = smoothHand.SmoothLeftHandJoints;
        }
        else
        {
            hand = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRHand>();
            oculus_skeleton = GameObject.Find("OVRCameraRig/TrackingSpace/RightHandAnchor/RightOVRHand").GetComponent<OVRSkeleton>();
            _handJointsData = smoothHand.SmoothRightHandJoints;
        }
        CreateTransparentMaterial();

    }

    void Update()
    {
        if (hand.IsTracked && oculus_skeleton != null)
        {
            DetectPinch();
            DetectRotation();
            DetectSqueeze(); // 调用握拳检测方法
        }
    }

    void CreateTransparentMaterial()
    {
        // 初始化透明材质
        transparentMaterial = new Material(Shader.Find("Standard"));
        transparentMaterial.SetFloat("_Mode", 3); // Transparent mode
        transparentMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        transparentMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        transparentMaterial.SetInt("_ZWrite", 0);
        transparentMaterial.DisableKeyword("_ALPHATEST_ON");
        transparentMaterial.EnableKeyword("_ALPHABLEND_ON");
        transparentMaterial.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        transparentMaterial.renderQueue = 3000;
    }

    void DetectPinch()
    {
        Transform firstFingerTip = GetBoneTransform(firstPinchFinger);
        Transform secondFingerTip = GetBoneTransform(secondPinchFinger);
        if (firstFingerTip == null || secondFingerTip == null) return;

        float distance = Vector3.Distance(firstFingerTip.position, secondFingerTip.position);

        if (distance < pinchThreshold && !isPinching)
        {
            isPinching = true;
            initialPinchPosition = (firstFingerTip.position + secondFingerTip.position) / 2;
            lastPinchPosition = initialPinchPosition;
            CreateOrUpdatePinchSphere(initialPinchPosition);
        }
        else if (distance >= pinchThreshold && isPinching)
        {
            isPinching = false;
            DestroyPinchSphere();
            pinchSpeed = Vector3.zero;
        }

        if (isPinching)
        {
            Vector3 currentPinchPosition = (firstFingerTip.position + secondFingerTip.position) / 2;
            pinchMovement = currentPinchPosition - initialPinchPosition;
            pinchSpeed = (currentPinchPosition - lastPinchPosition) / Time.deltaTime;
            lastPinchPosition = currentPinchPosition;

            CreateOrUpdatePinchSphere(currentPinchPosition);
        }
    }

    void DetectRotation()
    {
        // 获取旋转手势的两个指尖
        Transform firstRotateTip = GetBoneTransform(firstRotateFinger);
        Transform secondRotateTip = GetBoneTransform(secondRotateFinger);
        if (firstRotateTip == null || secondRotateTip == null) return;

        // 检查指尖之间的距离
        float distance = Vector3.Distance(firstRotateTip.position, secondRotateTip.position);

        if (distance < rotationThreshold && !isRotating)
        {
            // 开始检测旋转
            isRotating = true;
            initialRotatePosition = (firstRotateTip.position + secondRotateTip.position) / 2;

            // 计算旋转轴（基于两个额外关节的中点）
            Transform joint1 = GetBoneTransform(firstJointBone);
            Transform joint2 = GetBoneTransform(secondJointBone);
            if (joint1 != null && joint2 != null)
            {
                Vector3 jointMidpoint = (joint1.position + joint2.position) / 2;
                rotationAxis = (initialRotatePosition - jointMidpoint).normalized;

                // 记录初始方向
                initialDirectionJoint1 = joint1.position - jointMidpoint;
                initialDirectionJoint2 = joint2.position - jointMidpoint;

                // 初始化上一帧角度
                previousAngle1 = 0f;
                previousAngle2 = 0f;
            }
            CreateOrUpdateRotationSphere(initialRotatePosition);
        }
        else if (distance >= rotationThreshold && isRotating)
        {
            // 停止旋转检测
            isRotating = false;
            rotationSpeed = 0;
            previousAngle1 = 0f;
            previousAngle2 = 0f;
            DestroyRotationSphere();
        }

        // 计算瞬时角速度
        if (isRotating)
        {
            Transform joint1 = GetBoneTransform(firstJointBone);
            Transform joint2 = GetBoneTransform(secondJointBone);

            if (joint1 != null && joint2 != null)
            {
                Vector3 jointMidpoint = (joint1.position + joint2.position) / 2;

                // 当前方向向量
                Vector3 currentDirectionJoint1 = joint1.position - jointMidpoint;
                Vector3 currentDirectionJoint2 = joint2.position - jointMidpoint;

                // 计算当前角度
                float currentAngle1 = Vector3.SignedAngle(initialDirectionJoint1, currentDirectionJoint1, rotationAxis);
                float currentAngle2 = Vector3.SignedAngle(initialDirectionJoint2, currentDirectionJoint2, rotationAxis);

                // 计算瞬时角速度
                float angularVelocity1 = (currentAngle1 - previousAngle1) / Time.deltaTime;
                float angularVelocity2 = (currentAngle2 - previousAngle2) / Time.deltaTime;

                // 更新上一帧角度
                previousAngle1 = currentAngle1;
                previousAngle2 = currentAngle2;

                // 计算平均瞬时角速度
                rotationSpeed = (angularVelocity1 + angularVelocity2) / 2.0f / 360.0f;

                Debug.Log($"Rotation Speed: {rotationSpeed} degrees/second");
            }
            CreateOrUpdateRotationSphere(initialRotatePosition);
        }
    }

    void DetectSqueeze()
    {
        int curledFingers = 0;
        float sumDistance = 0;

        for (int i = 0; i < squeezeFingerTips.Count; i++)
        {
            Transform tip = GetBoneTransform(squeezeFingerTips[i]);
            Transform baseBone = GetBoneTransform(squeezeFingerBases[i]);

            if (tip == null || baseBone == null)
                continue;

            float distance = Vector3.Distance(tip.position, baseBone.position);

            sumDistance += distance;

            curledFingers += distance < squeezeThresholdHigh ? 1 : 0;
        }
        float avarageDistance = sumDistance / curledFingers;
        // 假设握拳需要三个手指都弯曲
        if (avarageDistance <= squeezeThresholdHigh && !isSqueezing)
        {
            isSqueezing = true;
            squeeze_ratio = 1 - (avarageDistance - squeezeThresholdLow) / (squeezeThresholdHigh - squeezeThresholdLow);
            CalculateSqueezeDetails();
            //CreateOrUpdateSqueezeSphere(squeezeCenter);
            CreateOrUpdateSqueezeCone(squeezeCenter, squeezeDirection);
        }
        else if (avarageDistance > squeezeThresholdHigh && isSqueezing)
        {
            isSqueezing = false;
            squeezeCenter = Vector3.zero;
            squeezeDirection = Vector3.zero;
            //DestroySqueezeSphere();
            DestroySqueezeCone();
        }

        if (isSqueezing)
        {
            squeeze_ratio = 1 - (avarageDistance - squeezeThresholdLow) / (squeezeThresholdHigh - squeezeThresholdLow);
            CalculateSqueezeDetails();
            //CreateOrUpdateSqueezeSphere(squeezeCenter);
            CreateOrUpdateSqueezeCone(squeezeCenter, squeezeDirection);
        }
    }

    void CalculateSqueezeDetails()
    {
        // 计算握拳的中心位置
        Vector3 sum = Vector3.zero;
        int count = 0;
        foreach (var boneId in squeezeFingerTips)
        {
            Transform tip = GetBoneTransform(boneId);
            if (tip != null)
            {
                sum += tip.position;
                count++;
            }
        }

        if (count > 0)
        {
            squeezeCenter = sum / count;
        }
        else
        {
            squeezeCenter = Vector3.zero;
        }

        // 计算握拳的朝向
        // 用小拇指指尖减去食指指尖的位置来计算朝向
        Transform pinkyTip = GetBoneTransform(OVRSkeleton.BoneId.Hand_PinkyTip);
        Transform indexTip = GetBoneTransform(OVRSkeleton.BoneId.Hand_IndexTip);

        if (pinkyTip != null && indexTip != null)
        {
            // 计算小指指尖到食指指尖的向量
            Vector3 pinchDirection = pinkyTip.position - indexTip.position;

            // 将该向量加到握拳的中心位置
            squeezeCenter += pinchDirection.normalized * pinchDirection.magnitude * 2; // 调整比例可以控制影响的大小
        }

        // 计算握拳的朝向
        // 用小拇指指尖的位置减去食指指尖的位置来计算朝向
        Transform pinky2 = GetBoneTransform(OVRSkeleton.BoneId.Hand_Pinky2);
        Transform index2 = GetBoneTransform(OVRSkeleton.BoneId.Hand_Index2);

        if (pinky2 != null && index2 != null)
        {
            // 计算朝向向量
            squeezeDirection = (pinky2.position - index2.position).normalized;
        }
        else
        {
            // 如果无法获取到小拇指和食指的指尖位置，使用默认朝向
            squeezeDirection = Vector3.down;
        }
    }

    Transform GetBoneTransform(OVRSkeleton.BoneId boneId)
    {
        if (!UseSmoothHand)
        {
            foreach (var bone in oculus_skeleton.Bones)
            {
                if (bone.Id == boneId) return bone.Transform;
            }
        }
        else
        {
            for (int i = 0; i < oculus_skeleton.Bones.Count; i++)
            {
                OVRBone bone = oculus_skeleton.Bones[i];
                if (bone.Id == boneId)
                {
                    return _handJointsData[i];
                }
            }
        }
        return null;
    }

    void CreateOrUpdatePinchSphere(Vector3 position)
    {
        if (pinchSphere == null && RenderPinchSphere)
        {
            pinchSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            pinchSphere.transform.localScale = Vector3.one * (2 * pinchRadius);
            pinchSphere.name = "PinchSphere";

            // 创建一个透明材质
            pinchSphere.GetComponent<Renderer>().material = new Material(transparentMaterial);
            pinchSphere.GetComponent<Renderer>().material.color = new Color(0, 1, 0, 0.2f); // 半透明绿色
        }

        if (pinchSphere != null)
            pinchSphere.transform.position = position;
    }

    void DestroyPinchSphere()
    {
        if (pinchSphere != null)
        {
            Destroy(pinchSphere);
            pinchSphere = null;
        }
    }

    void CreateOrUpdateRotationSphere(Vector3 position)
    {
        if (rotationSphere == null && RenderRotationSphere)
        {
            rotationSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            rotationSphere.transform.localScale = Vector3.one * (2 * pinchRadius);
            rotationSphere.name = "RotationSphere";

            // 创建一个透明材质
            rotationSphere.GetComponent<Renderer>().material = new Material(transparentMaterial);
            rotationSphere.GetComponent<Renderer>().material.color = new Color(0, 0, 1, 0.2f); // 半透明蓝色
        }
        if (rotationSphere != null)
            rotationSphere.transform.position = position;
    }

    void DestroyRotationSphere()
    {
        if (rotationSphere != null)
        {
            Destroy(rotationSphere);
            rotationSphere = null;
        }
    }

    void CreateOrUpdateSqueezeSphere(Vector3 position)
    {
        if (!RenderSqueezeSphere)
            return;

        if (squeezeSphere == null)
        {
            squeezeSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            squeezeSphere.transform.localScale = Vector3.one * (2 * squeezeRadius);
            squeezeSphere.name = "SqueezeSphere";

            // 创建一个透明材质
            squeezeSphere.GetComponent<Renderer>().material = new Material(transparentMaterial);
            squeezeSphere.GetComponent<Renderer>().material.color = new Color(1, 0, 0, 0.2f); // 半透明红色
        }

        if (squeezeSphere != null)
            squeezeSphere.transform.position = position;
    }

    void DestroySqueezeSphere()
    {
        if (squeezeSphere != null)
        {
            Destroy(squeezeSphere);
            squeezeSphere = null;
        }
    }

    void CreateOrUpdateSqueezeCone(Vector3 position, Vector3 direction)
    {
        if (!RenderSqueezeCone)
            return;

        if (squeezeCone == null)
        {
            // 创建一个圆锥体
            squeezeCone = CreateCone(coneRadius, coneHeight, 20); // 20段
            squeezeCone.name = "SqueezeCone";

            // 设置材质
            Renderer renderer = squeezeCone.GetComponent<Renderer>();
            renderer.material = new Material(transparentMaterial);
            renderer.material.color = new Color(1, 0, 0, 0.5f); // 半透明红色
        }

        if (squeezeCone != null)
        {
            // 设置位置
            squeezeCone.transform.position = position;

            // 设置朝向
            if (direction != Vector3.zero)
            {
                squeezeCone.transform.rotation = Quaternion.LookRotation(direction);
            }
        }
    }

    void DestroySqueezeCone()
    {
        if (squeezeCone != null)
        {
            Destroy(squeezeCone);
            squeezeCone = null;
        }
    }

    /// <summary>
    /// 创建一个圆锥体的 GameObject
    /// </summary>
    /// <param name="radius">圆锥底部半径</param>
    /// <param name="height">圆锥高度</param>
    /// <param name="segments">圆锥底部的细分段数</param>
    /// <returns>圆锥体的 GameObject</returns>
    GameObject CreateCone(float radius, float height, int segments)
    {
        GameObject cone = new GameObject("Cone");
        MeshFilter mf = cone.AddComponent<MeshFilter>();
        MeshRenderer mr = cone.AddComponent<MeshRenderer>();

        Mesh mesh = new Mesh();
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        // 顶点
        Vector3 tip = Vector3.zero;
        vertices.Add(tip);

        float angleStep = 360f / segments;
        for (int i = 0; i <= segments; i++)
        {
            float angle = i * angleStep * Mathf.Deg2Rad;
            float x = radius * Mathf.Cos(angle);
            float y = radius * Mathf.Sin(angle);
            Vector3 baseVertex = Vector3.back * height + new Vector3(x, y, 0);
            vertices.Add(baseVertex);
        }

        // 三角形
        for (int i = 1; i <= segments; i++)
        {
            triangles.Add(0); // Tip vertex
            triangles.Add(i);
            triangles.Add(i + 1);
        }

        // 创建底部
        int baseCenterIndex = vertices.Count;
        vertices.Add(Vector3.back * height);

        for (int i = 1; i <= segments; i++)
        {
            triangles.Add(baseCenterIndex);
            triangles.Add(i + 1);
            triangles.Add(i);
        }

        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.RecalculateNormals();

        mf.mesh = mesh;

        return cone;
    }
}
