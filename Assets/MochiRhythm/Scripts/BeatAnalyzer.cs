using System;
using System.Collections.Generic;

namespace MochiRhythm
{
    /// <summary>Tempo and beat times found in one song.</summary>
    public sealed class BeatAnalysis
    {
        public double Bpm;
        public double Period;     // seconds per beat
        public double[] Beats;    // beat times, seconds from the start of the song
        public double[] Strength; // onset strength at each beat
        public double Duration;   // song length, seconds
    }

    /// <summary>
    /// Beat tracking for any song. A line-for-line port of mochi-pounding/src/rhythm/beats.js (the
    /// three.js version), with the same constants, so both versions put the notes in the same places.
    ///  1. Onset strength every ~10 ms: rises in low-band energy (drums, bass) plus rises in high-band
    ///     energy (snares, claps, plucks).
    ///  2. Tempo: autocorrelation of that curve over 60-190 BPM, weighted toward ~110 BPM so the
    ///     tracker prefers the pulse people tap rather than half or double of it.
    ///  3. Beats: dynamic programming over the onset curve (D. Ellis, "Beat Tracking by Dynamic
    ///     Programming", 2007, the method librosa.beat.beat_track uses).
    /// Pure C# with no Unity calls, so it can run on a worker thread.
    /// </summary>
    public static class BeatAnalyzer
    {
        const double HopSeconds = 0.01;

        // JavaScript's Math.round (C#'s Math.Round rounds halves to even)
        static int RoundJs(double x) => (int)Math.Floor(x + 0.5);

        /// <param name="mono">the song mixed down to one channel</param>
        public static BeatAnalysis Analyze(float[] mono, int sampleRate, double minBpm = 60, double maxBpm = 190, double preferBpm = 110)
        {
            if (mono == null) throw new ArgumentNullException(nameof(mono));
            int sr = sampleRate;
            const int SUB = 8;
            int block = Math.Max(1, RoundJs(sr * HopSeconds / SUB));
            int hop = block * SUB;            // samples per analysis frame, an exact number of blocks
            double hopS = (double)hop / sr;   // seconds per frame (use this, not HopSeconds, for times and lags)
            int frames = mono.Length / hop;
            if (frames < 200) throw new InvalidOperationException("song too short to find a beat");

            // ---- 1. onset strength ----
            // Band energies in small blocks (hop/8), then a Hann-weighted 20 ms window every 10 ms, so
            // steady tones read as steady.
            int nBlocks = mono.Length / block;
            double aLow = Math.Exp(-2 * Math.PI * 180 / sr); // one-pole low-pass at ~180 Hz
            double low = 0;
            var bLow = new double[nBlocks];
            var bHigh = new double[nBlocks];
            for (int k = 0, j = 0; k < nBlocks; k++)
            {
                double sl = 0, sh = 0;
                for (int i = 0; i < block; i++, j++)
                {
                    double x = mono[j];
                    low = (1 - aLow) * x + aLow * low;
                    double high = x - low;
                    sl += low * low;
                    sh += high * high;
                }
                bLow[k] = sl;
                bHigh[k] = sh;
            }
            var win = new double[2 * SUB];
            for (int i = 0; i < win.Length; i++) win[i] = 0.5 - 0.5 * Math.Cos(2 * Math.PI * (i + 0.5) / win.Length);
            var eLow = new double[frames];
            var eHigh = new double[frames];
            double mLow = 0, mHigh = 0;
            for (int f = 0; f < frames; f++)
            {
                double sl = 0, sh = 0;
                int k0 = f * SUB - SUB;
                for (int i = 0; i < win.Length; i++)
                {
                    int k = k0 + i;
                    if (k < 0 || k >= nBlocks) continue;
                    sl += win[i] * bLow[k];
                    sh += win[i] * bHigh[k];
                }
                eLow[f] = sl;
                eHigh[f] = sh;
                mLow += sl;
                mHigh += sh;
            }
            // log with a floor (1% of the mean energy) so quiet passages don't flicker
            double fLow = 0.01 * (mLow / frames) + 1e-12, fHigh = 0.01 * (mHigh / frames) + 1e-12;
            for (int f = 0; f < frames; f++)
            {
                eLow[f] = Math.Log(eLow[f] + fLow);
                eHigh[f] = Math.Log(eHigh[f] + fHigh);
            }
            // a rise counts only above the higher of the two previous frames
            var onset = new double[frames];
            for (int f = 2; f < frames; f++)
            {
                onset[f] = Math.Max(0, eLow[f] - Math.Max(eLow[f - 1], eLow[f - 2]))
                         + 0.6 * Math.Max(0, eHigh[f] - Math.Max(eHigh[f - 1], eHigh[f - 2]));
            }
            // tempo is read from the raw curve with only its overall mean removed
            double rawSum = 0;
            for (int f = 0; f < frames; f++) rawSum += onset[f];
            double rawMean = rawSum / frames;
            var centred = new double[frames];
            for (int f = 0; f < frames; f++) centred[f] = onset[f] - rawMean;
            // for placing beats: remove a slow (3 s) local mean, keep the positive part, normalise
            const int W = 150;
            var prefix = new double[frames + 1];
            for (int f = 0; f < frames; f++) prefix[f + 1] = prefix[f] + onset[f];
            var strengthCurve = new double[frames];
            double sum = 0, sum2 = 0;
            for (int f = 0; f < frames; f++)
            {
                int lo0 = Math.Max(0, f - W), hi0 = Math.Min(frames, f + W + 1);
                double v = Math.Max(0, onset[f] - (prefix[hi0] - prefix[lo0]) / (hi0 - lo0));
                strengthCurve[f] = v;
                sum += v;
                sum2 += v * v;
            }
            double mean = sum / frames;
            double std = Math.Sqrt(Math.Max(1e-12, sum2 / frames - mean * mean));
            for (int f = 0; f < frames; f++) onset[f] = strengthCurve[f] / std;

            // ---- 2. tempo by autocorrelation ----
            int lagMin = Math.Max(2, (int)Math.Floor(60 / maxBpm / hopS));
            int lagMax = Math.Min(frames - 1, (int)Math.Ceiling(60 / minBpm / hopS));
            var ac = new double[lagMax + 2];
            for (int lag = lagMin - 1; lag <= lagMax + 1; lag++)
            {
                double s = 0;
                for (int f = lag; f < frames; f++) s += centred[f] * centred[f - lag];
                ac[lag] = s / (frames - lag);
            }
            int best = lagMin;
            double bestScore = double.NegativeInfinity;
            for (int lag = lagMin; lag <= lagMax; lag++)
            {
                double lagBpm = 60 / (lag * hopS);
                double octaves = Math.Log(lagBpm / preferBpm, 2) / 0.9;
                double prior = Math.Exp(-0.5 * octaves * octaves);
                double lagScore = ac[lag] * prior;
                if (lagScore > bestScore) { bestScore = lagScore; best = lag; }
            }
            // parabolic refinement of the peak
            double y0 = ac[best - 1], y1 = ac[best], y2 = ac[best + 1];
            double denom = y0 - 2 * y1 + y2;
            double period = best + (Math.Abs(denom) > 1e-12 ? 0.5 * (y0 - y2) / denom : 0); // in frames
            double bpm = 60 / (period * hopS);

            // ---- 3. beats by dynamic programming ----
            const double tight = 100;
            var dp = new double[frames];
            var back = new int[frames];
            int lo = RoundJs(period / 2), hi = RoundJs(period * 2);
            for (int t = 0; t < frames; t++)
            {
                int bestPrev = -1;
                double bestVal = 0;
                for (int tau = t - hi; tau <= t - lo; tau++)
                {
                    if (tau < 0) continue;
                    double pen = Math.Log((t - tau) / period);
                    double v = dp[tau] - tight * pen * pen;
                    if (bestPrev < 0 || v > bestVal) { bestVal = v; bestPrev = tau; }
                }
                dp[t] = onset[t] + (bestPrev >= 0 ? Math.Max(0, bestVal) : 0);
                back[t] = bestPrev >= 0 && bestVal > 0 ? bestPrev : -1;
            }
            // end on the best-scoring frame in the last period, then walk back
            int end = frames - 1;
            for (int t = Math.Max(0, frames - RoundJs(period)); t < frames; t++) if (dp[t] > dp[end]) end = t;
            var beatFrames = new List<int>();
            for (int t = end; t >= 0; t = back[t])
            {
                beatFrames.Add(t);
                if (back[t] < 0) break;
            }
            beatFrames.Reverse();

            // drop weak beats at the very start and end (silence or fades)
            var strength = new double[beatFrames.Count];
            for (int i = 0; i < beatFrames.Count; i++)
            {
                int bf = beatFrames[i];
                double s = 0;
                for (int k = Math.Max(0, bf - 2); k <= Math.Min(frames - 1, bf + 2); k++) s = Math.Max(s, onset[k]);
                strength[i] = s;
            }
            var sorted = (double[])strength.Clone();
            Array.Sort(sorted);
            double floor = sorted.Length > 0 ? 0.15 * sorted[sorted.Length / 2] : 0;
            int first = 0, last = beatFrames.Count - 1;
            while (first < last && strength[first] < floor) first++;
            while (last > first && strength[last] < floor) last--;
            var beats = new List<double>();
            var beatStrength = new List<double>();
            for (int i = first; i <= last; i++)
            {
                beats.Add(beatFrames[i] * hopS);
                beatStrength.Add(strength[i]);
            }
            return new BeatAnalysis
            {
                Bpm = bpm,
                Period = period * hopS,
                Beats = beats.ToArray(),
                Strength = beatStrength.ToArray(),
                Duration = (double)mono.Length / sr,
            };
        }

        /// <summary>Strike times from the beats: every beat, or every other beat on the stronger half.</summary>
        public static double[] StrikesFromBeats(BeatAnalysis a, int every)
        {
            if (every <= 1) return (double[])a.Beats.Clone();
            double even = 0, odd = 0;
            for (int i = 0; i < a.Strength.Length; i++)
            {
                if ((i & 1) == 1) odd += a.Strength[i];
                else even += a.Strength[i];
            }
            int phase = odd > even ? 1 : 0;
            var result = new List<double>();
            for (int i = 0; i < a.Beats.Length; i++) if (i % 2 == phase) result.Add(a.Beats[i]);
            return result.ToArray();
        }

        /// <summary>A comfortable mallet pace: every beat unless that is faster than one strike per 0.75 s.</summary>
        public static int AutoEvery(BeatAnalysis a) => a.Period < 0.75 ? 2 : 1;
    }
}
