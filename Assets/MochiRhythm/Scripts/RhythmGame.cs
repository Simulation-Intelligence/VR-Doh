using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace MochiRhythm
{
    /// <summary>
    /// Rhythm mode (port of the three.js version's src/rhythm/rhythm.js). The song's beats become
    /// strike notes. For each note a ring shrinks onto the usu's rim and meets it exactly on the strike;
    /// the rim pulses on every beat.
    ///  - Pounder (you hold the kine, or KineDriver is in Controller mode): your strikes are judged by
    ///    when you hear them. Perfect within 60 ms, Good within 130 ms, a miss after 250 ms. Striking while
    ///    a free hand is under the kine is "Ouch! Hands!".
    ///  - Kneader (nobody holds the kine): the partner strikes every note. Turn the dough between strikes
    ///    ("Turned" scores and keeps the combo) and get your hands out before it lands ("Hands!" breaks it).
    /// </summary>
    public class RhythmGame : MonoBehaviour
    {
        public const double LeadS = 1.6;  // how long a ring takes to close in
        public const double PerfectS = 0.06, GoodS = 0.13, CatchS = 0.25;
        const int RingSegments = 64;
        static readonly Color Gold = new Color(1f, 0.84f, 0.42f);
        static readonly Color Green = new Color(0.61f, 0.89f, 0.48f);
        static readonly Color Red = new Color(1f, 0.48f, 0.42f);
        static readonly Color RimColor = new Color(1f, 0.85f, 0.63f);
        static readonly Color RingColor = new Color(1f, 0.945f, 0.84f);

        public RhythmMode mode;

        public bool Active { get; private set; }
        public int Every { get; private set; } = 1;  // strike on every beat (1) or every other beat (2)
        public int Score { get; private set; }
        public int Combo { get; private set; }
        public int MaxCombo { get; private set; }
        public int Perfects { get; private set; }
        public int Goods { get; private set; }
        public int Misses { get; private set; }
        public int Turns { get; private set; }
        public int HandHits { get; private set; }
        /// <summary>Team credit for the Moon story: each note is worth up to 1 (see Judge / PartnerStrike), no combo multiplier.</summary>
        public float TeamCredit { get; private set; }
        /// <summary>Notes in one play-through of the song, at the current density.</summary>
        public int NotesPerPass => strikes != null ? strikes.Length : 0;
        public double SongDuration => dur;
        public int Mult => Math.Min(4, 1 + Combo / 8);
        /// <summary>Mean signed timing error of your recent strikes (ms; positive = late), or null.</summary>
        public double? MeanErrorMs
        {
            get
            {
                if (recent.Count == 0) return null;
                double s = 0;
                foreach (var e in recent) s += e;
                return 1000.0 * s / recent.Count;
            }
        }

        class Note { public double t; public LineRenderer ring; public bool judged; public bool npc; }
        readonly Dictionary<long, Note> notes = new Dictionary<long, Note>();
        readonly List<long> finished = new List<long>();
        readonly List<KeyValuePair<long, double>> window = new List<KeyValuePair<long, double>>();
        readonly List<KeyValuePair<long, double>> search = new List<KeyValuePair<long, double>>();
        readonly List<double> recent = new List<double>(); // signed errors (s) of recent strikes
        double[] strikes = new double[0], beats = new double[0];
        double dur;

        Material lineMaterial;
        LineRenderer rim;
        readonly List<LineRenderer> pool = new List<LineRenderer>();
        readonly Vector3[] circle = new Vector3[RingSegments];
        float flash;
        Color flashColor = RimColor;

        class Popup { public TextMeshPro text; public float t; }
        readonly List<Popup> popups = new List<Popup>();

        void Awake()
        {
            // built-in RP, both always included in builds: unlit, vertex-coloured, transparent
            var shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("UI/Default");
            lineMaterial = new Material(shader);
            rim = MakeLine("Rim pulse", 0.010f);
            for (int i = 0; i < 8; i++) pool.Add(MakeLine("Approach ring", 0.007f));
        }

        LineRenderer MakeLine(string name, float width)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.sharedMaterial = lineMaterial;
            lr.useWorldSpace = true;
            lr.loop = true;
            lr.positionCount = RingSegments;
            lr.widthMultiplier = width;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.enabled = false;
            return lr;
        }

        void DrawCircle(LineRenderer lr, Vector3 centre, Vector3 axisX, Vector3 axisZ, float radius, Color color)
        {
            for (int i = 0; i < RingSegments; i++)
            {
                float a = i * Mathf.PI * 2f / RingSegments;
                circle[i] = centre + (axisX * Mathf.Cos(a) + axisZ * Mathf.Sin(a)) * radius;
            }
            lr.SetPositions(circle);
            lr.startColor = color;
            lr.endColor = color;
            lr.enabled = true;
        }

        LineRenderer TakeRing()
        {
            foreach (var lr in pool)
            {
                bool used = false;
                foreach (var n in notes.Values) if (n.ring == lr) { used = true; break; }
                if (!used) return lr;
            }
            return null;
        }

        // ---- note bookkeeping (strike times repeat every loop of the song) ----

        void StrikesIn(double from, double to, List<KeyValuePair<long, double>> result)
        {
            result.Clear();
            if (strikes.Length == 0 || !(dur > 0) || double.IsInfinity(from) || double.IsInfinity(to)) return;
            long k0 = Math.Max(0, (long)Math.Floor(from / dur)), k1 = (long)Math.Floor(to / dur);
            for (long k = k0; k <= k1; k++)
                for (int i = 0; i < strikes.Length; i++)
                {
                    double t = strikes[i] + k * dur;
                    if (t >= from && t <= to) result.Add(new KeyValuePair<long, double>(k * 100000 + i, t));
                }
        }

        /// <summary>Next strike time after `after` (song seconds), or +infinity.</summary>
        public double NextStrike(double after)
        {
            double best = double.PositiveInfinity;
            StrikesIn(after, after + Math.Max(4, dur), search);
            foreach (var kv in search) if (kv.Value > after && kv.Value < best) best = kv.Value;
            return best;
        }

        double LastBeatBefore(double t)
        {
            if (beats.Length == 0 || !(dur > 0)) return double.NegativeInfinity;
            double k = Math.Floor(t / dur), local = t - k * dur;
            double b = double.NegativeInfinity;
            foreach (var x in beats) { if (x <= local) b = x; else break; }
            return double.IsNegativeInfinity(b) ? b : b + k * dur;
        }

        // ---- game ----

        /// <param name="everyChoice">0 = auto (every beat unless faster than one strike per 0.75 s), 1 or 2</param>
        public bool StartGame(int everyChoice)
        {
            var a = mode != null && mode.Music != null ? mode.Music.Analysis : null;
            if (a == null) return false;
            Every = everyChoice <= 0 ? BeatAnalyzer.AutoEvery(a) : everyChoice;
            strikes = BeatAnalyzer.StrikesFromBeats(a, Every);
            beats = a.Beats;
            dur = a.Duration;
            Score = 0; Combo = 0; MaxCombo = 0; Perfects = 0; Goods = 0; Misses = 0; Turns = 0; HandHits = 0; TeamCredit = 0f;
            recent.Clear();
            ClearNotes();
            Active = true;
            return true;
        }

        public void StopGame()
        {
            Active = false;
            ClearNotes();
            if (rim != null) rim.enabled = false;
        }

        public void ClearTimingHistory() => recent.Clear();

        void ClearNotes()
        {
            foreach (var n in notes.Values) if (n.ring != null) n.ring.enabled = false;
            notes.Clear();
        }

        void AddCombo(int points)
        {
            Combo++;
            MaxCombo = Math.Max(MaxCombo, Combo);
            Score += points * Mult;
        }

        void Judge(Note note, double err, bool hands, bool turned = false)
        {
            note.judged = true;
            double a = Math.Abs(err);
            string kind = hands ? "miss" : a <= PerfectS ? "perfect" : a <= GoodS ? "good" : "miss";
            if (kind == "perfect") { Perfects++; AddCombo(100); }
            else if (kind == "good") { Goods++; AddCombo(50); }
            else { Misses++; Combo = 0; }
            if (hands) HandHits++;
            if (!hands && a <= CatchS) { recent.Add(err); if (recent.Count > 12) recent.RemoveAt(0); }
            // team credit: the strike (Perfect 1, Good 0.6) plus the dough turned before it (0.5), at most 1 per note
            bool turnedOk = turned && !hands;
            if (turnedOk) Turns++;
            float credit = hands ? 0f : (kind == "perfect" ? 1f : kind == "good" ? 0.6f : 0f) + (turnedOk ? 0.5f : 0f);
            TeamCredit += Mathf.Min(1f, credit);
            string label = hands ? "Ouch! Hands!" : kind == "perfect" ? "Perfect" : kind == "good" ? (err < 0 ? "Good (early)" : "Good (late)") : err < 0 ? "Early" : "Late";
            if (turnedOk) label += " + Turned";
            Color color = kind == "perfect" ? Gold : kind == "good" ? Green : Red;
            flash = 1f;
            flashColor = color;
            ShowPopup(label, color);
        }

        /// <summary>The held kine hit the dough at song time t (as heard); hands = a free hand was under it.</summary>
        public void PlayerStrike(double t, bool hands, bool turned = false)
        {
            if (!Active || double.IsInfinity(t) || double.IsNaN(t)) return;
            Note best = null;
            double bestErr = double.PositiveInfinity;
            foreach (var n in notes.Values)
            {
                if (n.judged) continue;
                double e = t - n.t;
                if (Math.Abs(e) < Math.Abs(bestErr)) { bestErr = e; best = n; }
            }
            if (best != null && Math.Abs(bestErr) <= CatchS) Judge(best, bestErr, hands, turned);
            else
            {
                Combo = 0; // a strike nowhere near a note breaks the combo
                if (hands) HandHits++;
                ShowPopup(hands ? "Ouch! Hands!" : "Off-beat", Red);
            }
        }

        /// <summary>The partner's kine landed on a note. hands = a hand was under it; turned = the dough was turned since the last strike.</summary>
        public void PartnerStrike(bool hands, bool turned)
        {
            if (!Active) return;
            if (hands)
            {
                HandHits++;
                Combo = 0;
                flash = 1f;
                flashColor = Red;
                ShowPopup("Hands!", Red);
            }
            else if (turned)
            {
                Turns++;
                AddCombo(50);
                TeamCredit += 1f; // with the partner pounding, the kneader is the whole team
                flash = 1f;
                flashColor = Green;
                ShowPopup("Turned", Green);
            }
            else
            {
                flash = Mathf.Max(flash, 0.5f);
                flashColor = RimColor;
            }
        }

        void Update()
        {
            float dt = Time.deltaTime;
            UpdatePopups(dt);
            if (!Active || mode == null || mode.Music == null) return;
            double now = mode.Music.SongTime();
            if (double.IsInfinity(now) || double.IsNaN(now)) return;
            bool playerPounds = mode.PlayerPounding;

            // notes entering the approach window
            StrikesIn(now, now + LeadS, window);
            foreach (var kv in window)
                if (!notes.ContainsKey(kv.Key))
                    notes[kv.Key] = new Note { t = kv.Value, ring = TakeRing(), npc = !playerPounds };

            // rings close in; your unhit notes become misses; finished notes leave
            Transform st = mode.Station.transform;
            Vector3 centre = mode.RimCenter;
            float rimR = mode.RimRadius, startR = rimR + mode.ringReach;
            finished.Clear();
            foreach (var kv in notes)
            {
                var n = kv.Value;
                double left = n.t - now;
                if (!n.judged && left < -CatchS)
                {
                    if (playerPounds && !n.npc) Judge(n, CatchS, false);
                    else n.judged = true;
                }
                if (left < -0.35) { finished.Add(kv.Key); continue; }
                if (n.ring != null)
                {
                    float k = (float)Math.Max(0, left / LeadS);
                    float alpha = n.judged ? Mathf.Max(0f, 0.9f + (float)left * 3f) : Mathf.Min(0.9f, (1f - k) * 1.4f);
                    Color c = left > 0 ? RingColor : Gold;
                    c.a = alpha;
                    DrawCircle(n.ring, centre, st.right, st.forward, rimR + (startR - rimR) * k, c);
                }
            }
            foreach (var key in finished)
            {
                if (notes[key].ring != null) notes[key].ring.enabled = false;
                notes.Remove(key);
            }

            // rim: a soft pulse on every beat, a coloured flash on each judgment
            double b = LastBeatBefore(now);
            float pulse = double.IsNegativeInfinity(b) ? 0f : Mathf.Exp(-(float)(now - b) * 7f);
            flash = Mathf.Max(0f, flash - dt * 3f);
            Color rc = Color.Lerp(RimColor, flashColor, flash);
            rc.a = Mathf.Clamp01(0.15f + 0.45f * pulse + 0.5f * flash);
            DrawCircle(rim, centre, st.right, st.forward, rimR, rc);
        }

        // ---- floating judgment text ----

        void ShowPopup(string text, Color color)
        {
            if (mode == null) return;
            var go = new GameObject("Judgment");
            go.transform.SetParent(transform, false);
            go.transform.position = mode.PopupAnchor;
            var tmp = go.AddComponent<TextMeshPro>();
            tmp.text = text;
            tmp.fontSize = 0.6f; // 3D TextMeshPro: ~0.1 m per 1 pt, so ~6 cm
            tmp.fontStyle = FontStyles.Bold;
            tmp.alignment = TextAlignmentOptions.Center;
            tmp.textWrappingMode = TextWrappingModes.NoWrap;
            tmp.color = color;
            tmp.rectTransform.sizeDelta = new Vector2(0.8f, 0.15f);
            Billboard(go.transform);
            popups.Add(new Popup { text = tmp, t = 0f });
            while (popups.Count > 4) { Destroy(popups[0].text.gameObject); popups.RemoveAt(0); }
        }

        void UpdatePopups(float dt)
        {
            for (int i = popups.Count - 1; i >= 0; i--)
            {
                var p = popups[i];
                if (p.text == null) { popups.RemoveAt(i); continue; }
                p.t += dt;
                p.text.transform.position += Vector3.up * (dt * 0.1f);
                Billboard(p.text.transform);
                var c = p.text.color;
                c.a = Mathf.Max(0f, 1f - p.t / 0.9f);
                p.text.color = c;
                if (p.t > 0.9f) { Destroy(p.text.gameObject); popups.RemoveAt(i); }
            }
        }

        /// <summary>Turn a TextMeshPro to face the viewer (its readable side is -Z).</summary>
        public static void Billboard(Transform t)
        {
            var cam = Camera.main;
            if (cam == null) return;
            Vector3 away = t.position - cam.transform.position;
            if (away.sqrMagnitude > 1e-6f) t.rotation = Quaternion.LookRotation(away, Vector3.up);
        }
    }
}
