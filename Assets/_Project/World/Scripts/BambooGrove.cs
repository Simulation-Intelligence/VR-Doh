using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace MoonRabbit.World
{
    /// <summary>
    /// Draws the whole bamboo ring with GPU instancing (Graphics.RenderMeshInstanced):
    /// no GameObject per stalk, a few draw calls in total. Works in URP (beginCameraRendering)
    /// and in the Built-in pipeline (Camera.onPreCull); each pipeline fires only its own callback. Matrices are baked by
    /// "Moon Rabbit > Build Environment" and stored relative to this transform.
    /// If a headset ever shows bamboo in one eye only, untick "Use GPU instancing"
    /// in the builder settings to get plain MeshRenderers instead.
    /// </summary>
    [ExecuteAlways]
    public class BambooGrove : MonoBehaviour
    {
        [Serializable]
        public class Batch
        {
            public Mesh mesh;
            [Tooltip("One material per sub-mesh (stalk, leaves).")]
            public Material[] materials;
            public Matrix4x4[] localMatrices;
        }

        public List<Batch> batches = new List<Batch>();
        public ShadowCastingMode castShadows = ShadowCastingMode.Off;
        public bool receiveShadows = true;
        [Tooltip("Local-space bounds of all stalks (for culling).")]
        public Bounds localBounds = new Bounds(Vector3.zero, Vector3.one * 40f);

        /// <summary>0..1 tint toward <see cref="silverColor"/> (MoonStory sets it at the end of the night). Runtime only.</summary>
        [NonSerialized] public float silver;
        [NonSerialized] public Color silverColor = new Color(0.72f, 0.8f, 0.86f);
        MaterialPropertyBlock silverBlock;

        const int MaxPerCall = 1023;
        readonly List<Matrix4x4[]> worldCache = new List<Matrix4x4[]>();
        Matrix4x4 cachedLocalToWorld;
        bool cacheValid;
        Matrix4x4[] chunk;

        void OnEnable()
        {
            cacheValid = false;
            RenderPipelineManager.beginCameraRendering += OnBeginCamera; // URP / SRP
            Camera.onPreCull += OnPreCullCamera;                          // Built-in pipeline
        }

        void OnDisable()
        {
            RenderPipelineManager.beginCameraRendering -= OnBeginCamera;
            Camera.onPreCull -= OnPreCullCamera;
        }

        void OnValidate() => cacheValid = false;

        void RebuildCache()
        {
            worldCache.Clear();
            var l2w = transform.localToWorldMatrix;
            foreach (var b in batches)
            {
                var src = b?.localMatrices ?? Array.Empty<Matrix4x4>();
                var dst = new Matrix4x4[src.Length];
                for (int i = 0; i < src.Length; i++) dst[i] = l2w * src[i];
                worldCache.Add(dst);
            }
            cachedLocalToWorld = l2w;
            cacheValid = true;
        }

        void OnBeginCamera(ScriptableRenderContext context, Camera cam) => Draw(cam);

        void OnPreCullCamera(Camera cam)
        {
            if (GraphicsSettings.currentRenderPipeline != null) return; // SRP never calls this, but be safe
            Draw(cam);
        }

        void Draw(Camera cam)
        {
            if (!isActiveAndEnabled || batches == null || batches.Count == 0) return;
            if (cam.cameraType == CameraType.Preview || cam.cameraType == CameraType.Reflection) return;
            if ((cam.cullingMask & (1 << gameObject.layer)) == 0) return;

            if (!cacheValid || transform.localToWorldMatrix != cachedLocalToWorld || worldCache.Count != batches.Count)
                RebuildCache();

            var worldBounds = TransformBounds(localBounds, transform.localToWorldMatrix);

            for (int bi = 0; bi < batches.Count; bi++)
            {
                var b = batches[bi];
                var mats = worldCache[bi];
                if (b == null || b.mesh == null || b.materials == null || mats.Length == 0) continue;

                for (int sub = 0; sub < b.mesh.subMeshCount && sub < b.materials.Length; sub++)
                {
                    var mat = b.materials[sub];
                    if (mat == null) continue;
                    var rp = new RenderParams(mat)
                    {
                        camera = cam,
                        layer = gameObject.layer,
                        worldBounds = worldBounds,
                        shadowCastingMode = castShadows,
                        receiveShadows = receiveShadows,
                    };
                    if (silver > 0.001f && mat.HasProperty("_Color"))
                    {
                        silverBlock ??= new MaterialPropertyBlock();
                        silverBlock.SetColor("_Color", Color.Lerp(mat.color, silverColor, Mathf.Clamp01(silver)));
                        rp.matProps = silverBlock;
                    }

                    if (mats.Length <= MaxPerCall)
                    {
                        Graphics.RenderMeshInstanced(rp, b.mesh, sub, mats);
                    }
                    else
                    {
                        for (int start = 0; start < mats.Length; start += MaxPerCall)
                        {
                            int count = Mathf.Min(MaxPerCall, mats.Length - start);
                            if (chunk == null || chunk.Length != count) chunk = new Matrix4x4[count];
                            Array.Copy(mats, start, chunk, 0, count);
                            Graphics.RenderMeshInstanced(rp, b.mesh, sub, chunk);
                        }
                    }
                }
            }
        }

        static Bounds TransformBounds(Bounds b, Matrix4x4 m)
        {
            var center = m.MultiplyPoint3x4(b.center);
            var ext = b.extents;
            var axisX = m.MultiplyVector(new Vector3(ext.x, 0, 0));
            var axisY = m.MultiplyVector(new Vector3(0, ext.y, 0));
            var axisZ = m.MultiplyVector(new Vector3(0, 0, ext.z));
            var size = new Vector3(
                Mathf.Abs(axisX.x) + Mathf.Abs(axisY.x) + Mathf.Abs(axisZ.x),
                Mathf.Abs(axisX.y) + Mathf.Abs(axisY.y) + Mathf.Abs(axisZ.y),
                Mathf.Abs(axisX.z) + Mathf.Abs(axisY.z) + Mathf.Abs(axisZ.z));
            return new Bounds(center, size * 2f);
        }

        void OnDrawGizmosSelected()
        {
            var wb = TransformBounds(localBounds, transform.localToWorldMatrix);
            Gizmos.color = new Color(0.31f, 0.48f, 0.33f, 0.5f);
            Gizmos.DrawWireCube(wb.center, wb.size);
        }
    }
}
