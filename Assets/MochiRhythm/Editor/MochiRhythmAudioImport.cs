using UnityEditor;
using UnityEngine;

namespace MochiRhythm.EditorTools
{
    /// <summary>
    /// Songs in Assets/MochiRhythm/Resources/MochiRhythm/Music are imported Decompress On Load (so the beat
    /// tracker can read their samples with AudioClip.GetData) and not preloaded (so on the headset only the
    /// song you pick is decoded into memory; RhythmMusic loads and unloads it).
    /// </summary>
    public class MochiRhythmAudioImport : AssetPostprocessor
    {
        const string MusicFolder = "Assets/MochiRhythm/Resources/MochiRhythm/Music/";

        void OnPreprocessAudio()
        {
            if (!assetPath.StartsWith(MusicFolder)) return;
            var importer = (AudioImporter)assetImporter;
            var settings = importer.defaultSampleSettings;
            settings.loadType = AudioClipLoadType.DecompressOnLoad;
            // preloadAudioData lives on the sample settings in current Unity; set it by name so this
            // compiles whichever way the field is declared
            object boxed = settings;
            var field = typeof(AudioImporterSampleSettings).GetField("preloadAudioData");
            if (field != null && field.FieldType == typeof(bool)) field.SetValue(boxed, false);
            else
            {
                var prop = typeof(AudioImporterSampleSettings).GetProperty("preloadAudioData");
                if (prop != null && prop.CanWrite && prop.PropertyType == typeof(bool)) prop.SetValue(boxed, false);
            }
            settings = (AudioImporterSampleSettings)boxed;
            importer.defaultSampleSettings = settings;
        }
    }
}
