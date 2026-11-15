using System;
using TMPro;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// A small button pressed by touching it with an index fingertip or the tip of a Touch controller
    /// (RhythmMode.PressPoints, which leaves out whatever holds the kine). Fires once per touch.
    /// Local frame: the top face is +Y; the label reads from the -Z side.
    /// </summary>
    [DefaultExecutionOrder(60)] // after RhythmMode collected this frame's press points
    public class FingerButton : MonoBehaviour
    {
        public RhythmMode mode;
        public Vector3 size = new Vector3(0.06f, 0.02f, 0.045f);
        public Action onPress;

        TextMeshPro label;
        Transform cap;
        Material material;
        Color baseColor;
        bool inside;
        float cooldown, flash;

        public void Build(string text, Color color)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube); // built-in Default-Material (Standard)
            go.name = "Cap";
            var col = go.GetComponent<Collider>();
            if (col != null) Destroy(col);
            cap = go.transform;
            cap.SetParent(transform, false);
            cap.localScale = size;
            material = go.GetComponent<Renderer>().material;
            baseColor = color;
            material.color = color;

            var lgo = new GameObject("Label");
            lgo.transform.SetParent(transform, false);
            lgo.transform.localRotation = Quaternion.Euler(90f, 0f, 0f); // lie on the top face, top of the text toward +Z
            label = lgo.AddComponent<TextMeshPro>();
            label.fontSize = 0.13f; // 3D TextMeshPro: ~1.3 cm
            label.alignment = TextAlignmentOptions.Center;
            label.textWrappingMode = TextWrappingModes.Normal;
            label.rectTransform.sizeDelta = new Vector2(size.x * 0.95f, size.z * 0.95f);
            label.color = Color.white;
            label.text = text;
            Layout(0f);
        }

        public void SetLabel(string text) { if (label != null && label.text != text) label.text = text; }
        public void SetColor(Color color) { baseColor = color; }

        void Layout(float dip)
        {
            cap.localPosition = new Vector3(0f, dip, 0f);
            label.transform.localPosition = new Vector3(0f, size.y * 0.5f + 0.0015f + dip, 0f);
        }

        void Update()
        {
            if (cap == null) return;
            cooldown -= Time.deltaTime;
            bool now = Touching();
            if (now && !inside && cooldown <= 0f)
            {
                cooldown = 0.35f;
                flash = 1f;
                onPress?.Invoke();
            }
            inside = now;
            flash = Mathf.Max(0f, flash - Time.deltaTime * 4f);
            Layout(inside ? -0.35f * size.y : 0f);
            material.color = Color.Lerp(baseColor, Color.white, 0.6f * flash);
        }

        bool Touching()
        {
            if (mode == null) return false;
            Vector3 e = 0.5f * size + new Vector3(0.006f, 0.015f, 0.006f);
            foreach (var w in mode.PressPoints)
            {
                Vector3 p = transform.InverseTransformPoint(w);
                if (Mathf.Abs(p.x) < e.x && Mathf.Abs(p.y) < e.y && Mathf.Abs(p.z) < e.z) return true;
            }
            return false;
        }
    }
}
