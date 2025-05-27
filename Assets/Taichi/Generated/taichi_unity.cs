using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace Taichi.Generated {

// handle.native_buffer
[StructLayout(LayoutKind.Sequential)]
public struct TixNativeBufferUnity {
  public IntPtr Inner;
}

// function.import_native_runtime
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern TiRuntime tix_import_native_runtime_unity(

);
public static TiRuntime TixImportNativeRuntimeUnity(

) {
  var rv = tix_import_native_runtime_unity(
  );
  return rv;
}
}

// function.launch_kernel_async
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_launch_kernel_async_unity(
  TiRuntime runtime,
  TiKernel kernel,
  uint arg_count,
  [MarshalAs(UnmanagedType.LPArray)] TiArgument[] args
);
public static void TixLaunchKernelAsyncUnity(
  TiRuntime runtime,
  TiKernel kernel,
  uint arg_count,
  TiArgument[] args
) {
  tix_launch_kernel_async_unity(
    runtime,
    kernel,
    arg_count,
    args
  );
}
}

// function.launch_compute_graph_async
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_launch_compute_graph_async_unity(
  TiRuntime runtime,
  TiComputeGraph compute_graph,
  uint arg_count,
  [MarshalAs(UnmanagedType.LPArray)] TiNamedArgument[] args
);
public static void TixLaunchComputeGraphAsyncUnity(
  TiRuntime runtime,
  TiComputeGraph compute_graph,
  uint arg_count,
  TiNamedArgument[] args
) {
  tix_launch_compute_graph_async_unity(
    runtime,
    compute_graph,
    arg_count,
    args
  );
}
}

// function.copy_memory_to_native_buffer_async
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_to_native_buffer_async_unity(
  TiRuntime runtime,
  TixNativeBufferUnity dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryToNativeBufferAsyncUnity(
  TiRuntime runtime,
  TixNativeBufferUnity dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_to_native_buffer_async_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}

// function.copy_memory_device_to_host
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] byte[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  byte[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] sbyte[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  sbyte[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] short[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  short[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] ushort[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  ushort[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] int[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  int[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] uint[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  uint[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] long[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  long[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] ulong[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  ulong[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] IntPtr[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  IntPtr[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] float[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  float[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_device_to_host_unity(
  TiRuntime runtime,
  [MarshalAs(UnmanagedType.LPArray)] double[] dst,
  ulong dst_offset,
  IntPtr src
);
public static void TixCopyMemoryDeviceToHostUnity(
  TiRuntime runtime,
  double[] dst,
  ulong dst_offset,
  TiMemorySlice src
) {
  IntPtr hglobal_src = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src, hglobal_src, false);
  tix_copy_memory_device_to_host_unity(
    runtime,
    dst,
    dst_offset,
    hglobal_src
  );
  Marshal.FreeHGlobal(hglobal_src);
}
}

// function.copy_memory_host_to_device
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] byte[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  byte[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] sbyte[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  sbyte[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] short[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  short[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] ushort[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  ushort[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] int[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  int[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] uint[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  uint[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] long[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  long[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] ulong[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  ulong[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] IntPtr[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  IntPtr[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] float[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  float[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern void tix_copy_memory_host_to_device_unity(
  TiRuntime runtime,
  IntPtr dst,
  [MarshalAs(UnmanagedType.LPArray)] double[] src,
  ulong src_offset
);
public static void TixCopyMemoryHostToDeviceUnity(
  TiRuntime runtime,
  TiMemorySlice dst,
  double[] src,
  ulong src_offset
) {
  IntPtr hglobal_dst = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst, hglobal_dst, false);
  tix_copy_memory_host_to_device_unity(
    runtime,
    hglobal_dst,
    src,
    src_offset
  );
  Marshal.FreeHGlobal(hglobal_dst);
}
}

// function.submit_async
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_unity")]
#endif
private static extern IntPtr tix_submit_async_unity(
  TiRuntime runtime
);
public static IntPtr TixSubmitAsyncUnity(
  TiRuntime runtime
) {
  var rv = tix_submit_async_unity(
    runtime
  );
  return rv;
}
}

} // namespace Taichi
