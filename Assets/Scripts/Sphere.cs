using UnityEngine;

[System.Serializable]
public class Sphere : MonoBehaviour
{
    public GameObject sphereObject;
    private Vector3 lastpos = Vector3.zero;
    private Vector3 initialScale;

    private bool shouldenlarge = false;
    private bool shouldshrink = false;
    public float scaleSpeed = 0.1f;
    public float maxScale = 3.0f;
    public float minScale = 0.1f;
    public Sphere(GameObject sphere)
    {
        sphereObject = sphere;
        initialScale = sphereObject.transform.localScale;
    }

    public Vector3 Position
    {
        get { return sphereObject.transform.position; }
        set { sphereObject.transform.position = value; }
    }

    public Vector3 Velocity
    {
        get
        {
            Vector3 curpos = sphereObject.transform.position;//当前点
            Vector3 _speed = (curpos - lastpos) / Time.deltaTime;//与上一个点做计算除去当前帧花的时间。
            lastpos = curpos;//把当前点保存下一次用
            return _speed;

        }
    }

    public float Radius
    {
        get { return sphereObject.transform.localScale.x / 2; }
        set { sphereObject.transform.localScale = Vector3.one * value * 2; }
    }
    public void EnlargeSphere()
    {
        shouldenlarge = true;
    }
    public void KeepSphere()
    {
        shouldenlarge = false;
        shouldshrink = false;
    }

    public void ShrinkSphere()
    {
        shouldshrink = true;
    }
    void Update()
    {
        if (shouldenlarge)
        {
            if (sphereObject.transform.localScale.x < maxScale)
            {
                sphereObject.transform.localScale += Vector3.one * scaleSpeed * Time.deltaTime;
            }
        }
        if (shouldshrink)
        {
            if (sphereObject.transform.localScale.x > minScale)
            {
                sphereObject.transform.localScale -= Vector3.one * scaleSpeed * Time.deltaTime;
            }
        }

    }
}
