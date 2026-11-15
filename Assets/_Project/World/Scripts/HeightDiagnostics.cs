using System;
using System.Collections;
using UnityEngine;
using UnityEngine.XR;

namespace MoonRabbit.World
{
    /// <summary>
    /// One logcat line with every height that matters, at launch (after tracking settles), after each story reset and
    /// when the headset is put back on: tracking origin, head height in tracking space and in the world, the rig, the
    /// MRUK floor anchor (the real floor), the station (usu) and the nearest scanned table. If the head-in-tracking-space
    /// height jumps between sitting and standing launches, Quest's floor reference is the cause (not the scene).
    /// Read-only: changes nothing.
    /// </summary>
    public class HeightDiagnostics : MonoBehaviour
    {
        public float launchDelay = 6f;

        void Start()
        {
            StartCoroutine(Later("launch", launchDelay));
            var mp = GetComponent<MoonProgress>();
            if (mp != null) mp.onReset.AddListener(() => StartCoroutine(Later("reset:" + mp.LastResetReason, 1f)));
        }

        IEnumerator Later(string why, float delay)
        {
            yield return new WaitForSecondsRealtime(delay);
            Log(why);
        }

        public void Log(string why)
        {
            try
            {
                var cam = Camera.main;
                string origin = "?";
                var subs = new System.Collections.Generic.List<XRInputSubsystem>();
                SubsystemManager.GetSubsystems(subs);
                if (subs.Count > 0) origin = subs[0].GetTrackingOriginMode().ToString();
                float headLocal = cam != null ? cam.transform.localPosition.y : float.NaN;
                float headWorld = cam != null ? cam.transform.position.y : float.NaN;
                float rig = cam != null && cam.transform.parent != null ? cam.transform.parent.position.y : float.NaN;
                var st = GameObject.Find("MochiStation");
                Debug.Log($"[Moon Rabbit] Heights ({why}): origin={origin} headInTracking={headLocal:F2} headWorld={headWorld:F2} " +
                          $"trackingSpaceY={rig:F2} floorAnchorY={FloorAnchorY():F2} stationY={(st != null ? st.transform.position.y : float.NaN):F2}");
            }
            catch (Exception e) { Debug.Log("[Moon Rabbit] Heights: " + e.Message); }
        }

        static float FloorAnchorY()
        {
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                var t = a.GetType("Meta.XR.MRUtilityKit.MRUK", false);
                if (t == null) continue;
                var inst = t.GetProperty("Instance")?.GetValue(null);
                if (inst == null) return float.NaN;
                if (!(t.GetProperty("Rooms")?.GetValue(inst) is IList rooms) || rooms.Count == 0) return float.NaN;
                var floor = rooms[0].GetType().GetProperty("FloorAnchor")?.GetValue(rooms[0]) as Component;
                return floor != null ? floor.transform.position.y : float.NaN;
            }
            return float.NaN;
        }
    }
}
