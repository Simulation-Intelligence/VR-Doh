using UnityEngine;
using System.Collections.Generic;

public class MatTool : MonoBehaviour
{
    public struct Primitive
    {
        public Vector3 sphere1;
        public float radii1;
        public Vector3 sphere2;
        public float radii2;
        public Vector3 sphere3;
        public float radii3;
    }
    [HideInInspector]
    public int numPrimitives;
    public Primitive[] init_primitives;
    public Primitive[] primitives;

    [SerializeField]
    protected Quaternion _rotationOffset = Quaternion.identity;

    // Json Object to store the primitives
    [System.Serializable]
    public class PrimitiveData
    {
        public float[] sphere1;
        public float radii1;
        public float[] sphere2;
        public float radii2;
        public float[] sphere3;
        public float radii3;
    }
    [System.Serializable]
    public class PrimitiveList
    {
        public List<PrimitiveData> primitives;
    }
    
    public enum HandType
    {
        LeftHand,
        RightHand
    }
    public HandType handType;

    // Smoothed hand-tracking
    public SmoothHand smoothHand;
    protected List<Transform> _handJointsData;

    void Start()
    {
        primitives = new Primitive[numPrimitives];
        for (int i = 0; i < numPrimitives; i++)
        {
            primitives[i] = init_primitives[i];
        }
    }
    
    void Update()
    {
        UpdatePrimitives();
    }
    
    // Virutal method to be overriden by child classes
    protected virtual void UpdatePrimitives()
    {
        TransformFixedPrimitives();
    }
    
    void TransformFixedPrimitives()
    {
        for (int i = 0; i < numPrimitives; i++)
        {
            // Update primitive position using the oculus Hand Joint Component
            primitives[i].sphere1 = transform.TransformPoint(init_primitives[i].sphere1);
            primitives[i].sphere2 = transform.TransformPoint(init_primitives[i].sphere2);
            primitives[i].sphere3 = transform.TransformPoint(init_primitives[i].sphere3);
        }
    }
    
    // 定义一个函数来更新 primitive
    protected void UpdatePrimitive(ref Primitive primitive, Primitive init_primitive, Transform transform)
    {
        // 更新 primitive 的球体位置，使用 transform 的 TransformPoint 进行转换
        primitive.sphere1 = transform.TransformPoint(init_primitive.sphere1);
        primitive.sphere2 = transform.TransformPoint(init_primitive.sphere2);
        primitive.sphere3 = transform.TransformPoint(init_primitive.sphere3);

        // 乘以 localScale 来获得正确的半径
        primitive.radii1 = transform.localScale.x * init_primitive.radii1;
        primitive.radii2 = transform.localScale.x * init_primitive.radii2;
        primitive.radii3 = transform.localScale.x * init_primitive.radii3;
    }

    protected void LoadPrimitivesFromJson(string filepath, bool inverse)
    {
        TextAsset jsonFile = Resources.Load<TextAsset>(filepath);

        if (jsonFile != null)
        {
            PrimitiveList primitiveList = JsonUtility.FromJson<PrimitiveList>("{\"primitives\":" + jsonFile.text + "}");
            numPrimitives = primitiveList.primitives.Count;
            init_primitives = new Primitive[numPrimitives];

            for (int i = 0; i < numPrimitives; i++)
            {
                PrimitiveData data = primitiveList.primitives[i];
                Vector3 sphere1, sphere2, sphere3;
                if (inverse)
                {
                    sphere1 = new Vector3(data.sphere1[0] * -1, data.sphere1[1] * -1, data.sphere1[2] * -1);
                    sphere2 = new Vector3(data.sphere2[0] * -1, data.sphere2[1] * -1, data.sphere2[2] * -1);
                    sphere3 = new Vector3(data.sphere3[0] * -1, data.sphere3[1] * -1, data.sphere3[2] * -1);
                }
                else
                {
                    sphere1 = new Vector3(data.sphere1[0], data.sphere1[1], data.sphere1[2]);
                    sphere2 = new Vector3(data.sphere2[0], data.sphere2[1], data.sphere2[2]);
                    sphere3 = new Vector3(data.sphere3[0], data.sphere3[1], data.sphere3[2]);
                }
                init_primitives[i] = new Primitive
                {
                    sphere1 = sphere1,
                    radii1 = data.radii1,
                    sphere2 = sphere2,
                    radii2 = data.radii2,
                    sphere3 = sphere3,
                    radii3 = data.radii3
                };
            }
            Debug.Log("Primitives loaded successfully from JSON.");
        }
        else
        {
            Debug.LogError("Could not load JSON file.");
        }
    }
}