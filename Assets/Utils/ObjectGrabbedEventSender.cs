using UnityEngine;
using Oculus.Interaction;

public class ObjectGrabbedEventSender : GrabFreeTransformer, ITransformer
{
    public delegate void ObjectGrabbed(GameObject source);
    public event ObjectGrabbed onObjectGrabbed;
    public delegate void ObjectMoved(GameObject source);
    public event ObjectMoved onObjectMoved;
    public delegate void ObjectReleased(GameObject source);
    public event ObjectReleased onObjectReleased;
    public bool isGrabbed { get; private set; }

    public new void Initialize(IGrabbable grabbable)
    {
        base.Initialize(grabbable);
    }
    
    public new void BeginTransform()
    {
        base.BeginTransform();
        onObjectGrabbed?.Invoke(gameObject);
        Debug.Log(gameObject.name + "is being grabbed");
        isGrabbed = true;
    }

    public new void UpdateTransform()
    {
        base.UpdateTransform();
        onObjectMoved?.Invoke(gameObject);
    }

    public new void EndTransform()
    {
        //Parent class does nothing with that method so no need to call it
        onObjectReleased?.Invoke(gameObject);
        isGrabbed = false;
    }

}