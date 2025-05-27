#if UNITY_EDITOR

using System;
using System.IO;
using UnityEditor.AssetImporters;
using UnityEngine;


namespace Taichi {
    [ScriptedImporter(1, "tcm")]
    public class AotModuleImporter : ScriptedImporter {
        public override void OnImportAsset(AssetImportContext ctx) {
            var guid = Guid.NewGuid().ToString();
            var tcm = File.ReadAllBytes(ctx.assetPath);

            AotModuleAsset asset = ScriptableObject.CreateInstance<AotModuleAsset>();
            asset.name = "tcm:" + ctx.assetPath;
            asset.Guid = guid;
            asset.ArchiveData = tcm;

            ctx.AddObjectToAsset("archive", asset);
            ctx.SetMainObject(asset);
        }
    }
}

#endif // UNITY_EDITOR
