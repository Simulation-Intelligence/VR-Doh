using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTrackingScript : MonoBehaviour
{
    // Start is called before the first frame update
    public Camera sceneCamera;
    public OVRHand leftHand;
    public OVRHand rightHand;
    public OVRSkeleton skeleton;

    private Vector3 targetPosition;
    private Quaternion targetRotation;
    private float step;
    private bool isIndexFingerPinching;

    private LineRenderer line;
    private Transform p0;
    private Transform p1;
    private Transform p2;

    private Transform handIndexTipTransform;
    void Start()
    {
        transform.position = sceneCamera.transform.position + sceneCamera.transform.forward * 1.0f;
        line = GetComponent<LineRenderer>();
    }

    // Update is called once per frame
    void Update()
    {
        step = 5.0f * Time.deltaTime;

        if (leftHand.IsTracked)
        {
            isIndexFingerPinching = leftHand.GetFingerIsPinching(OVRHand.HandFinger.Index);
            if (isIndexFingerPinching)
            {
                line.enabled = true;
                // Animate cube smoothly next to left hand
                pinchCube();
                foreach (var b in skeleton.Bones)
                {
                    if (b.Id == OVRSkeleton.BoneId.Hand_IndexTip)
                    {
                        handIndexTipTransform = b.Transform;
                        break;
                    }
                }

                p0 = transform;
                p2 = handIndexTipTransform;
                p1 = sceneCamera.transform;
                p1.position += sceneCamera.transform.forward * 0.8f;

                DrawCurve(p0.position, p1.position, p2.position);
                // ...

            }
            else
            {
                line.enabled = false;
            }
        }
    }
    void pinchCube()
    {
        targetPosition = leftHand.transform.position - leftHand.transform.forward * 0.4f;
        targetRotation = Quaternion.LookRotation(transform.position - leftHand.transform.position);

        transform.position = Vector3.Lerp(transform.position, targetPosition, step);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, step);
    }
    void DrawCurve(Vector3 point_0, Vector3 point_1, Vector3 point_2)
    {
        line.positionCount = 200;
        Vector3 B = new Vector3(0, 0, 0);
        float t = 0f;

        for (int i = 0; i < line.positionCount; i++)
        {
            t += 0.005f;
            B = (1 - t) * (1 - t) * point_0 + 2 * (1 - t) * t * point_1 + t * t * point_2;
            line.SetPosition(i, B);
        }
    }
}
