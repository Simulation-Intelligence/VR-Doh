using UnityEngine;

public class HandGestureControl : MonoBehaviour
{
    public OVRHand leftHand;
    public GameObject sphere;
    public float scaleSpeed = 0.1f;
    public float maxScale = 3.0f;
    public float minScale = 0.1f;

    private float handClosedThreshold = 0.1f; // Threshold for considering the hand as closed (fist)
    private float handOpenThreshold = 0.01f;   // Threshold for considering the hand as open

    void Start()
    {
        if (leftHand == null)
        {
            Debug.LogError("Please assign the left hand OVRHand object in the inspector.");
        }
    }

    void Update()
    {
        if (leftHand.IsTracked)
        {
            float avgPinchStrength = GetAveragePinchStrength(leftHand);

            if (avgPinchStrength > handClosedThreshold)
            {
                // Hand is closed (fist), shrink the sphere
                Vector3 newScale = sphere.transform.localScale - Vector3.one * scaleSpeed * Time.deltaTime;
                newScale.x = Mathf.Clamp(newScale.x, minScale, maxScale);
                newScale.y = Mathf.Clamp(newScale.y, minScale, maxScale);
                newScale.z = Mathf.Clamp(newScale.z, minScale, maxScale);
                sphere.transform.localScale = newScale;
            }
            else if (avgPinchStrength < handOpenThreshold)
            {
                // Hand is open, enlarge the sphere
                Vector3 newScale = sphere.transform.localScale + Vector3.one * scaleSpeed * Time.deltaTime;
                newScale.x = Mathf.Clamp(newScale.x, minScale, maxScale);
                newScale.y = Mathf.Clamp(newScale.y, minScale, maxScale);
                newScale.z = Mathf.Clamp(newScale.z, minScale, maxScale);
                sphere.transform.localScale = newScale;
            }
        }
    }

    private float GetAveragePinchStrength(OVRHand hand)
    {
        float thumbStrength = hand.GetFingerPinchStrength(OVRHand.HandFinger.Thumb);
        float indexStrength = hand.GetFingerPinchStrength(OVRHand.HandFinger.Index);
        float middleStrength = hand.GetFingerPinchStrength(OVRHand.HandFinger.Middle);
        float ringStrength = hand.GetFingerPinchStrength(OVRHand.HandFinger.Ring);
        float pinkyStrength = hand.GetFingerPinchStrength(OVRHand.HandFinger.Pinky);

        return (thumbStrength + indexStrength + middleStrength + ringStrength + pinkyStrength) / 5.0f;
    }
}
