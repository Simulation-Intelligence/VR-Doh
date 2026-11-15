using System;
using System.Reflection;
using UnityEngine;
using UnityEngine.XR;

namespace MoonRabbit.World
{
    /// <summary>
    /// Demo helper: when the headset is taken off for at least <see cref="minRemovedSeconds"/> and put back on, the story
    /// starts again from the intro (same path as the demo reset: ending cleared, progress 0, Earth dark, rice re-placed,
    /// intro). A quick lift of the headset is ignored. Detection, all optional so the kit compiles without the Meta SDK:
    /// OVRManager.HMDUnmounted / HMDMounted (found by reflection), XR userPresence on the head device, and
    /// OnApplicationPause / OnApplicationFocus (Quest pauses the app when the headset comes off).
    /// </summary>
    [RequireComponent(typeof(MoonProgress))]
    public class HeadsetRestart : MonoBehaviour
    {
        [Tooltip("Restart from the intro when the headset is put back on (someone new is trying it).")]
        public bool restartOnRemount = true;
        [Tooltip("Only when it was off at least this long (s); lifting it for a moment does nothing.")]
        public float minRemovedSeconds = 3f;
        [Tooltip("Wait after putting it on before the intro starts, so tracking and the view have settled (s).")]
        public float settleDelay = 0.5f;

        public bool HeadsetOn { get; private set; } = true;

        MoonProgress progress;
        float removedAt = -1f;      // realtimeSinceStartup when it came off (real time keeps running while paused)
        float restartAt = -1f;
        bool presenceKnown, lastPresence = true;
        Action onUnmounted, onMounted;
        EventInfo unmountedEvent, mountedEvent;

        void Awake() => progress = GetComponent<MoonProgress>();

        void OnEnable()
        {
            // Meta XR SDK: static events on OVRManager, subscribed by reflection (no assembly reference)
            var ovr = FindType("OVRManager");
            if (ovr == null) return;
            unmountedEvent = ovr.GetEvent("HMDUnmounted", BindingFlags.Public | BindingFlags.Static);
            mountedEvent = ovr.GetEvent("HMDMounted", BindingFlags.Public | BindingFlags.Static);
            try
            {
                if (unmountedEvent != null && unmountedEvent.EventHandlerType == typeof(Action))
                    unmountedEvent.AddEventHandler(null, onUnmounted = Removed);
                if (mountedEvent != null && mountedEvent.EventHandlerType == typeof(Action))
                    mountedEvent.AddEventHandler(null, onMounted = PutOn);
            }
            catch (Exception e) { Debug.LogWarning("[Moon Rabbit] HeadsetRestart: could not subscribe to OVRManager: " + e.Message); }
        }

        void OnDisable()
        {
            try
            {
                if (unmountedEvent != null && onUnmounted != null) unmountedEvent.RemoveEventHandler(null, onUnmounted);
                if (mountedEvent != null && onMounted != null) mountedEvent.RemoveEventHandler(null, onMounted);
            }
            catch (Exception) { }
            onUnmounted = onMounted = null;
        }

        static Type FindType(string name)
        {
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                var t = a.GetType(name, false);
                if (t != null) return t;
            }
            return null;
        }

        // Quest pauses / unfocuses the app when the headset comes off. In the Editor these fire when another window
        // takes focus, so they only count in a player build.
        void OnApplicationPause(bool paused) { if (Application.isEditor) return; if (paused) Removed(); else PutOn(); }
        void OnApplicationFocus(bool focus) { if (Application.isEditor) return; if (!focus) Removed(); else PutOn(); }

        /// <summary>The headset came off (or the app lost focus / was paused).</summary>
        public void Removed()
        {
            if (!HeadsetOn) return;
            HeadsetOn = false;
            removedAt = Time.realtimeSinceStartup;
            restartAt = -1f;
        }

        /// <summary>The headset is back on: restart after a short settle if it was off long enough.</summary>
        public void PutOn()
        {
            if (HeadsetOn) return;
            HeadsetOn = true;
            float off = removedAt >= 0f ? Time.realtimeSinceStartup - removedAt : 0f;
            removedAt = -1f;
            if (restartOnRemount && off >= minRemovedSeconds)
                restartAt = Time.realtimeSinceStartup + settleDelay;
        }

        void Update()
        {
            // XR user presence (proximity sensor) where the runtime reports it
            var head = InputDevices.GetDeviceAtXRNode(XRNode.Head);
            if (head.isValid && head.TryGetFeatureValue(CommonUsages.userPresence, out bool present))
            {
                if (presenceKnown && present != lastPresence) { if (present) PutOn(); else Removed(); }
                presenceKnown = true;
                lastPresence = present;
            }

            if (restartAt >= 0f && Time.realtimeSinceStartup >= restartAt)
            {
                restartAt = -1f;
                if (progress != null) progress.RequestDemoReset();
            }
        }
    }
}
