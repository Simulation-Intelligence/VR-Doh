using UnityEngine;

namespace MoonRabbit.World
{
    /// <summary>
    /// Demo strike counter until gameplay sends real strike events: the kine's striking face (KineTool origin)
    /// coming down fast inside the usu and reaching the dough's top counts as one strike, which adds
    /// 1 / demoStrikesToFull to <see cref="MoonProgress"/>. Turned off with MoonProgress.demoStrikeDetection.
    /// </summary>
    [RequireComponent(typeof(MochiLink))]
    public class DemoStrikeDetector : MonoBehaviour
    {
        [Tooltip("Downward speed of the face (m/s) needed for a strike.")]
        public float minSpeed = 1.5f;
        [Tooltip("Seconds before the next strike can count.")]
        public float cooldown = 0.3f;
        [Tooltip("The face counts as on the dough this far above the dough's highest point (m).")]
        public float contactMargin = 0.015f;
        [Tooltip("How long a fast downward motion stays valid while the face travels on to the dough (s).")]
        public float speedMemory = 0.15f;

        public int StrikeCount { get; private set; }
        public float LastDownSpeed { get; private set; }

        MochiLink link;
        Vector3 prevPos;
        float prevTime = -1f, fastUntil = -1f, lastStrike = -10f;
        bool wasAbove = true;

        void Awake() => link = GetComponent<MochiLink>();

        void LateUpdate()
        {
            var mp = MoonProgress.Instance;
            if (mp == null || !mp.demoStrikeDetection || mp.Locked || link.Kine == null || link.Usu == null) return;

            var pos = link.Kine.position;
            float now = Time.time;
            if (prevTime < 0f) { prevPos = pos; prevTime = now; return; }
            if (pos == prevPos) return; // the kine may move in FixedUpdate: measure only over frames where it moved
            float dt = Mathf.Max(1e-4f, now - prevTime);
            float vDown = -(pos.y - prevPos.y) / dt;
            LastDownSpeed = vDown;
            if (vDown >= minSpeed) fastUntil = now + speedMemory;

            var usu = link.Usu.position;
            var flat = new Vector2(pos.x - usu.x, pos.z - usu.z);
            bool insideBowl = flat.magnitude < link.UsuInnerTop + 0.03f && pos.y < usu.y + link.UsuBase + link.UsuDepth + 0.25f;
            float doughTop = link.HasDough ? link.TopY : link.DoughFloorY + 0.06f;
            bool above = pos.y > doughTop + contactMargin;

            if (insideBowl && wasAbove && !above && now <= fastUntil && now - lastStrike >= cooldown)
            {
                lastStrike = now;
                StrikeCount++;
                mp.RegisterDemoStrike(pos);
            }
            wasAbove = above;
            prevPos = pos;
            prevTime = now;
        }
    }
}
