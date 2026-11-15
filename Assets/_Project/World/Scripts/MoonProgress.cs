using System;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.XR;
#if MR_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace MoonRabbit.World
{
    /// <summary>
    /// Story progress of the night, 0 (crescent, rice) to 1 (full Moon, finished mochi).
    /// Gameplay calls it from anywhere:
    /// <code>
    /// MoonProgress.Instance?.Add(0.05f);          // one good strike
    /// MoonProgress.Instance?.Set(quality);        // or drive it directly (0..1)
    /// MoonProgress.Instance?.NotifyStrike(point); // optional: rice grains hop at the impact point
    /// MoonProgress.Instance.onChanged.AddListener(p => ...);
    /// MoonProgress.Instance.onComplete.AddListener(() => ...);
    /// </code>
    /// When the real strike events are wired up, untick <see cref="demoStrikeDetection"/>.
    /// Demo controls (presentation backup): right controller A = +1/N, hold A 1 s = reset;
    /// keyboard P = +1/N, R = reset, F = finish.
    /// </summary>
    [DefaultExecutionOrder(-50), DisallowMultipleComponent]
    public class MoonProgress : MonoBehaviour
    {
        public static MoonProgress Instance { get; private set; }

        [Range(0f, 1f), SerializeField] float progress;
        public float Value => progress;
        public bool IsComplete { get; private set; }
        /// <summary>True during the ending: Add/Set are ignored until the automatic reset.</summary>
        public bool Locked { get; set; }
        /// <summary>True while the intro plays: strikes and Add/Set do not count yet.</summary>
        public bool IntroActive { get; set; }
        /// <summary>Raised by the trigger / A button / Space during the intro (StoryIntro skips).</summary>
        public event Action SkipIntroRequested;

        [Serializable] public class FloatEvent : UnityEvent<float> { }
        [Serializable] public class PointEvent : UnityEvent<Vector3> { }

        [Tooltip("Called with the new progress (0..1) whenever it changes.")]
        public FloatEvent onChanged = new FloatEvent();
        [Tooltip("Called once when progress reaches 1.")]
        public UnityEvent onComplete = new UnityEvent();
        [Tooltip("Called for every strike (world point of the impact).")]
        public PointEvent onStrike = new PointEvent();
        [Tooltip("Called when progress goes back to 0 (demo reset or after the ending).")]
        public UnityEvent onReset = new UnityEvent();
        [Serializable] public class IntEvent : UnityEvent<int> { }
        [Tooltip("Sound hook: 1..4 when progress first passes 25 / 50 / 75 / 100 %.")]
        public IntEvent onMilestone = new IntEvent();
        [Tooltip("Sound hook: the finished mochi reached the Earth (ending).")]
        public UnityEvent onMochiArrivedEarth = new UnityEvent();
        [Tooltip("Sound hook: intro line n (1-based) starts fading in.")]
        public IntEvent onIntroLine = new IntEvent();

        int milestone;

        [Header("Demo")]
        [Tooltip("Strikes (demo detection or the A button / P key) needed for a full Moon.")]
        [Min(1)] public int demoStrikesToFull = 12;
        [Tooltip("Count strikes from the kine's motion (DemoStrikeDetector). Turn off once gameplay calls Add/Set itself.")]
        public bool demoStrikeDetection = true;
        [Tooltip("Right controller A (+1/N, hold 1 s = reset) and keyboard P / R / F.")]
        public bool demoControls = true;
        public float holdToReset = 1f;

        float aDownTime = -1f;
        bool aWasDown, holdFired, triggerWasDown;

        void OnEnable()
        {
            if (Instance != null && Instance != this) { Debug.LogWarning("[Moon Rabbit] Two MoonProgress components; using the first."); return; }
            Instance = this;
        }

        void OnDisable()
        {
            if (Instance == this) Instance = null;
        }

        /// <summary>Adds to the progress (clamped to 0..1).</summary>
        public void Add(float amount) => Set(progress + amount);

        /// <summary>Sets the progress (clamped to 0..1). Reaching 1 fires onComplete once.</summary>
        public void Set(float value)
        {
            if (Locked || IntroActive) return;
            value = Mathf.Clamp01(value);
            if (value > 0.9995f) value = 1f;
            if (Mathf.Approximately(value, progress)) return;
            progress = value;
            onChanged.Invoke(progress);
            int m = Mathf.FloorToInt(progress * 4f + 1e-4f);
            while (milestone < m) { milestone++; onMilestone.Invoke(milestone); }
            if (progress >= 1f && !IsComplete)
            {
                IsComplete = true;
                onComplete.Invoke();
            }
        }

        /// <summary>One demo step: 1 / demoStrikesToFull.</summary>
        public void AddDemoStep() => Add(1f / Mathf.Max(1, demoStrikesToFull));

        /// <summary>Strike happened at a world point: counts one demo step and lets the rice grains hop.</summary>
        public void RegisterDemoStrike(Vector3 worldPoint)
        {
            NotifyStrike(worldPoint);
            AddDemoStep();
        }

        /// <summary>Visual only (rice grains hop); does not change the progress.</summary>
        public void NotifyStrike(Vector3 worldPoint) => onStrike.Invoke(worldPoint);

        /// <summary>Back to 0 (crescent). Also unlocks after the ending.</summary>
        public void ResetProgress()
        {
            Locked = false;
            IsComplete = false;
            progress = 0f;
            milestone = 0;
            onChanged.Invoke(0f);
            onReset.Invoke();
        }

        void Update()
        {
            if (!demoControls) return;

            // Controller: right-hand primary button (A on Quest Touch).
            bool aDown = false;
            var right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            if (right.isValid) right.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primaryButton, out aDown);

            if (IntroActive)
            {
                // During the intro: trigger (either hand), A or Space skips it; the press is swallowed.
                bool trig = false, trigL = false;
                if (right.isValid) right.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out trig);
                var left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
                if (left.isValid) left.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out trigL);
                bool key = false;
#if MR_INPUT_SYSTEM
                var k0 = Keyboard.current;
                key = k0 != null && (k0.spaceKey.wasPressedThisFrame || k0.enterKey.wasPressedThisFrame);
#elif ENABLE_LEGACY_INPUT_MANAGER
                key = Input.GetKeyDown(KeyCode.Space) || Input.GetKeyDown(KeyCode.Return);
#endif
                bool anyNew = (aDown && !aWasDown) || ((trig || trigL) && !triggerWasDown) || key;
                triggerWasDown = trig || trigL;
                if (anyNew) SkipIntroRequested?.Invoke();
                if (aDown) holdFired = true; // swallow this A press
                aWasDown = aDown;
                return;
            }
            if (aDown && !aWasDown) { aDownTime = Time.unscaledTime; holdFired = false; }
            if (aDown && !holdFired && aDownTime >= 0f && Time.unscaledTime - aDownTime >= holdToReset)
            {
                holdFired = true;
                ResetProgress();
            }
            if (!aDown && aWasDown && !holdFired) AddDemoStep();
            aWasDown = aDown;

#if MR_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.pKey.wasPressedThisFrame) AddDemoStep();
                if (kb.rKey.wasPressedThisFrame) ResetProgress();
                if (kb.fKey.wasPressedThisFrame) Set(1f);
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKeyDown(KeyCode.P)) AddDemoStep();
            if (Input.GetKeyDown(KeyCode.R)) ResetProgress();
            if (Input.GetKeyDown(KeyCode.F)) Set(1f);
#endif
        }
    }
}
