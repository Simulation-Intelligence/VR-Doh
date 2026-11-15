// SPDX-License-Identifier: MIT
#if GS_ENABLE_URP

using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;
using UnityEngine.Rendering.RenderGraphModule;
using UnityEngine.Rendering.Universal;

namespace GaussianSplatting.Runtime
{
    // URP 17 (Unity 6) RenderGraph version. The pre-RenderGraph Execute/OnCameraSetup path was
    // removed from URP in Unity 6.x, so the pass is recorded with an unsafe RenderGraph pass:
    // splats are drawn into a temporary half-float target (sharing the camera depth) and then
    // composited onto the camera color target.
    //
    // ReSharper disable once InconsistentNaming
    class GaussianSplatURPFeature : ScriptableRendererFeature
    {
        class GSRenderPass : ScriptableRenderPass
        {
            const string k_GaussianSplatRTName = "_GaussianSplatRT";
            const string k_ProfilerTag = "GaussianSplatRenderGraph";
            static readonly ProfilingSampler s_ProfilingSampler = new(k_ProfilerTag);
            static readonly int s_GaussianSplatRT = Shader.PropertyToID(k_GaussianSplatRTName);

            class PassData
            {
                internal Camera Camera;
                internal TextureHandle SourceColor;
                internal TextureHandle SourceDepth;
                internal TextureHandle GaussianSplatRT;
            }

            public override void RecordRenderGraph(RenderGraph renderGraph, ContextContainer frameData)
            {
                var cameraData = frameData.Get<UniversalCameraData>();
                var resourceData = frameData.Get<UniversalResourceData>();

                RenderTextureDescriptor rtDesc = cameraData.cameraTargetDescriptor;
                rtDesc.depthBufferBits = 0;
                rtDesc.depthStencilFormat = GraphicsFormat.None;
                rtDesc.msaaSamples = 1;
                rtDesc.graphicsFormat = GraphicsFormat.R16G16B16A16_SFloat;
                TextureHandle splatRT = UniversalRenderer.CreateRenderGraphTexture(renderGraph, rtDesc, k_GaussianSplatRTName, true);

                using var builder = renderGraph.AddUnsafePass(k_ProfilerTag, out PassData passData);
                passData.Camera = cameraData.camera;
                passData.SourceColor = resourceData.activeColorTexture;
                passData.SourceDepth = resourceData.activeDepthTexture;
                passData.GaussianSplatRT = splatRT;

                builder.UseTexture(resourceData.activeColorTexture, AccessFlags.ReadWrite);
                builder.UseTexture(resourceData.activeDepthTexture, AccessFlags.Read);
                builder.UseTexture(splatRT, AccessFlags.ReadWrite);
                builder.AllowPassCulling(false);
                builder.AllowGlobalStateModification(true);

                builder.SetRenderFunc(static (PassData data, UnsafeGraphContext context) =>
                {
                    CommandBuffer cmd = CommandBufferHelpers.GetNativeCommandBuffer(context.cmd);
                    using (new ProfilingScope(cmd, s_ProfilingSampler))
                    {
                        cmd.SetGlobalTexture(s_GaussianSplatRT, data.GaussianSplatRT);
                        CoreUtils.SetRenderTarget(cmd, data.GaussianSplatRT, data.SourceDepth, ClearFlag.Color, new Color(0, 0, 0, 0));

                        // add sorting, view calc and drawing commands for each splat object
                        Material matComposite = GaussianSplatRenderSystem.instance.SortAndRenderSplats(data.Camera, cmd);

                        // compose
                        cmd.BeginSample(GaussianSplatRenderSystem.s_ProfCompose);
                        Blitter.BlitCameraTexture(cmd, data.GaussianSplatRT, data.SourceColor, RenderBufferLoadAction.Load, RenderBufferStoreAction.Store, matComposite, 0);
                        cmd.EndSample(GaussianSplatRenderSystem.s_ProfCompose);
                    }
                });
            }
        }

        GSRenderPass m_Pass;
        bool m_HasCamera;

        public override void Create()
        {
            m_Pass = new GSRenderPass
            {
                renderPassEvent = RenderPassEvent.BeforeRenderingTransparents
            };
        }

        public override void OnCameraPreCull(ScriptableRenderer renderer, in CameraData cameraData)
        {
            m_HasCamera = GaussianSplatRenderSystem.instance.GatherSplatsForCamera(cameraData.camera);
        }

        public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
        {
            if (!m_HasCamera)
                return;
            renderer.EnqueuePass(m_Pass);
        }

        protected override void Dispose(bool disposing)
        {
            m_Pass = null;
        }
    }
}

#endif // #if GS_ENABLE_URP
