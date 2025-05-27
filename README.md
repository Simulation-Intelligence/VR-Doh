# Taichi-UnityExample (exploring MPM)

## Common problems during set up

**Problem:** "InvalidOperationException: Ignored launch because kernel handle is null"

**Solution:** This error occurs when the taichi_c_api.dll and taichi_unity.dll files in Assets/Plugins/X86_64 are incompatible with the AOT modules in Assets/Resources/TaichiModules. To resolve this issue, follow these steps:
1. Copy the correct taichi_c_api.dll from your pip-installed taichi to Assets/Plugins/X86_64.
2. Rebuild the AOT modules in Assets/Resources/TaichiModules using the same version of taichi.

## How to stream to quest3?

To stream to Quest 3, follow these steps:
1. Install Meta Quest Link on your PC.
2. Connect your PC to the VR headset and open the Quest Link app on the headset.
3. Open your Unity project and set up the OpenXR plugin (not the Oculus plugin).
4. Click "Play" in Unity, and you will see the scene in the VR headset.
5. Make sure you have enabled all the settings in the "beta" tab in the Meta Quest Link app.

## Weird Disappearance

**Problem:** When dynamically creating a mesh and assigning it to a MeshFilter through a script, the mesh may disappear when the MeshFilter moves out of the camera's view. This occurs because Unity's culling system automatically hides objects that are outside the camera's frustum to optimize performance.

**Solution:** To prevent the mesh from disappearing when it moves out of the camera's view, you can manually adjust the mesh's bounding box. By setting a larger or more appropriate bounding box, you ensure that the mesh remains visible even when the MeshFilter is outside the initial view.

```csharp
_Mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 114514f);
```

## Problem with Gaussian Renderer

The Gaussian Renderer is from aras-p's UnityGaussianSplatting (https://github.com/aras-p/UnityGaussianSplatting/tree/main), which is a great tool for rendering gaussian. However, you need to make several changes to make it work with Taichi in VR.

### Changes in `SplatUtilities.compute`

1. Add the include for Unity built-in stuff:
    ```c
    #include "UnityCG.cginc"
    ```

2. Change line 81 to:
    ```c
    float4 centerClipPos = mul(UNITY_MATRIX_VP, float4(centerWorldPos, 1));
    ```

3. Change line 95 to:
    ```c
    float3 cov2d = CalcCovariance2D(splat.pos, cov3d0, cov3d1, _MatrixMV, UNITY_MATRIX_P, _VecScreenParams);
    ```

### Scene Camera Settings

- Set the MSAA to "off".

### XR Plugin Management

- Set the PC render mode in OpenXR to "Multi Pass".

### Player Settings

- Set the Graphics API to "Vulkan".
