using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;

namespace Taichi.Generated {

// alias.bool


// definition.false
static partial class Def {
public const uint TI_FALSE = 0;
}

// definition.true
static partial class Def {
public const uint TI_TRUE = 1;
}

// alias.flags
// using TiFlags = uint;

// definition.null_handle
static partial class Def {
public const uint TI_NULL_HANDLE = 0;
}

// handle.runtime
[StructLayout(LayoutKind.Sequential)]
public struct TiRuntime {
  public IntPtr Inner;
}

// handle.aot_module
[StructLayout(LayoutKind.Sequential)]
public struct TiAotModule {
  public IntPtr Inner;
}

// handle.memory
[StructLayout(LayoutKind.Sequential)]
public struct TiMemory {
  public IntPtr Inner;
}

// handle.kernel
[StructLayout(LayoutKind.Sequential)]
public struct TiKernel {
  public IntPtr Inner;
}

// handle.compute_graph
[StructLayout(LayoutKind.Sequential)]
public struct TiComputeGraph {
  public IntPtr Inner;
}

// enumeration.arch
public enum TiArch {
  TI_ARCH_X64 = 0,
  TI_ARCH_ARM64 = 1,
  TI_ARCH_JS = 2,
  TI_ARCH_CC = 3,
  TI_ARCH_WASM = 4,
  TI_ARCH_CUDA = 5,
  TI_ARCH_METAL = 6,
  TI_ARCH_OPENGL = 7,
  TI_ARCH_DX11 = 8,
  TI_ARCH_OPENCL = 9,
  TI_ARCH_AMDGPU = 10,
  TI_ARCH_VULKAN = 11,
  TI_ARCH_MAX_ENUM = 0x7fffffff,
}

// enumeration.data_type
public enum TiDataType {
  TI_DATA_TYPE_F16 = 0,
  TI_DATA_TYPE_F32 = 1,
  TI_DATA_TYPE_F64 = 2,
  TI_DATA_TYPE_I8 = 3,
  TI_DATA_TYPE_I16 = 4,
  TI_DATA_TYPE_I32 = 5,
  TI_DATA_TYPE_I64 = 6,
  TI_DATA_TYPE_U1 = 7,
  TI_DATA_TYPE_U8 = 8,
  TI_DATA_TYPE_U16 = 9,
  TI_DATA_TYPE_U32 = 10,
  TI_DATA_TYPE_U64 = 11,
  TI_DATA_TYPE_GEN = 12,
  TI_DATA_TYPE_UNKNOWN = 13,
  TI_DATA_TYPE_MAX_ENUM = 0x7fffffff,
}

// enumeration.argument_type
public enum TiArgumentType {
  TI_ARGUMENT_TYPE_I32 = 0,
  TI_ARGUMENT_TYPE_F32 = 1,
  TI_ARGUMENT_TYPE_NDARRAY = 2,
  TI_ARGUMENT_TYPE_MAX_ENUM = 0x7fffffff,
}

// bit_field.memory_usage
[Flags]
public enum TiMemoryUsageFlagBits {
  TI_MEMORY_USAGE_STORAGE_BIT = 1 << 0,
  TI_MEMORY_USAGE_UNIFORM_BIT = 1 << 1,
  TI_MEMORY_USAGE_VERTEX_BIT = 1 << 2,
  TI_MEMORY_USAGE_INDEX_BIT = 1 << 3,
};

// structure.memory_allocate_info
[StructLayout(LayoutKind.Sequential)]
public struct TiMemoryAllocateInfo {
  public ulong size;
  public uint host_write;
  public uint host_read;
  public uint export_sharing;
  public TiMemoryUsageFlagBits usage;
}

// structure.memory_slice
[StructLayout(LayoutKind.Sequential)]
public struct TiMemorySlice {
  public TiMemory memory;
  public ulong offset;
  public ulong size;
}

// structure.nd_shape
[StructLayout(LayoutKind.Sequential)]
public struct TiNdShape {
  public uint dim_count;
  [MarshalAs(UnmanagedType.ByValArray, SizeConst=16)] public uint[] dims;
}

// structure.nd_array
[StructLayout(LayoutKind.Sequential)]
public struct TiNdArray {
  public TiMemory memory;
  public TiNdShape shape;
  public TiNdShape elem_shape;
  public TiDataType elem_type;
}

// union.argument_value
[StructLayout(LayoutKind.Explicit)]
public struct TiArgumentValue {
  [FieldOffset(0)] public int i32;
  [FieldOffset(0)] public float f32;
  [FieldOffset(0)] public TiNdArray ndarray;
}

// structure.argument
[StructLayout(LayoutKind.Sequential)]
public struct TiArgument {
  public TiArgumentType type;
  public TiArgumentValue value;
}

// structure.named_argument
[StructLayout(LayoutKind.Sequential)]
public struct TiNamedArgument {
  public string name;
  public TiArgument argument;
}

// function.create_runtime
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern TiRuntime ti_create_runtime(
  TiArch arch
);
public static TiRuntime TiCreateRuntime(
  TiArch arch
) {
  var rv = ti_create_runtime(
    arch
  );
  return rv;
}
}

// function.destroy_runtime
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_destroy_runtime(
  TiRuntime runtime
);
public static void TiDestroyRuntime(
  TiRuntime runtime
) {
  ti_destroy_runtime(
    runtime
  );
}
}

// function.allocate_memory
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern TiMemory ti_allocate_memory(
  TiRuntime runtime,
  IntPtr allocate_info
);
public static TiMemory TiAllocateMemory(
  TiRuntime runtime,
  TiMemoryAllocateInfo allocate_info
) {
  IntPtr hglobal_allocate_info = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemoryAllocateInfo)));
  Marshal.StructureToPtr(allocate_info, hglobal_allocate_info, false);
  var rv = ti_allocate_memory(
    runtime,
    hglobal_allocate_info
  );
  Marshal.FreeHGlobal(hglobal_allocate_info);
  return rv;
}
}

// function.free_memory
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_free_memory(
  TiRuntime runtime,
  TiMemory memory
);
public static void TiFreeMemory(
  TiRuntime runtime,
  TiMemory memory
) {
  ti_free_memory(
    runtime,
    memory
  );
}
}

// function.map_memory
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern IntPtr ti_map_memory(
  TiRuntime runtime,
  TiMemory memory
);
public static IntPtr TiMapMemory(
  TiRuntime runtime,
  TiMemory memory
) {
  var rv = ti_map_memory(
    runtime,
    memory
  );
  return rv;
}
}

// function.unmap_memory
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_unmap_memory(
  TiRuntime runtime,
  TiMemory memory
);
public static void TiUnmapMemory(
  TiRuntime runtime,
  TiMemory memory
) {
  ti_unmap_memory(
    runtime,
    memory
  );
}
}

// function.copy_memory_device_to_device
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_copy_memory_device_to_device(
  TiRuntime runtime,
  IntPtr dst_memory,
  IntPtr src_memory
);
public static void TiCopyMemoryDeviceToDevice(
  TiRuntime runtime,
  TiMemorySlice dst_memory,
  TiMemorySlice src_memory
) {
  IntPtr hglobal_dst_memory = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(dst_memory, hglobal_dst_memory, false);
  IntPtr hglobal_src_memory = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(TiMemorySlice)));
  Marshal.StructureToPtr(src_memory, hglobal_src_memory, false);
  ti_copy_memory_device_to_device(
    runtime,
    hglobal_dst_memory,
    hglobal_src_memory
  );
  Marshal.FreeHGlobal(hglobal_dst_memory);
  Marshal.FreeHGlobal(hglobal_src_memory);
}
}

// function.launch_kernel
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_launch_kernel(
  TiRuntime runtime,
  TiKernel kernel,
  uint arg_count,
  [MarshalAs(UnmanagedType.LPArray)] TiArgument[] args
);
public static void TiLaunchKernel(
  TiRuntime runtime,
  TiKernel kernel,
  uint arg_count,
  TiArgument[] args
) {
  ti_launch_kernel(
    runtime,
    kernel,
    arg_count,
    args
  );
}
}

// function.launch_compute_graph
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_launch_compute_graph(
  TiRuntime runtime,
  TiComputeGraph compute_graph,
  uint arg_count,
  [MarshalAs(UnmanagedType.LPArray)] TiNamedArgument[] args
);
public static void TiLaunchComputeGraph(
  TiRuntime runtime,
  TiComputeGraph compute_graph,
  uint arg_count,
  TiNamedArgument[] args
) {
  ti_launch_compute_graph(
    runtime,
    compute_graph,
    arg_count,
    args
  );
}
}

// function.submit
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_submit(
  TiRuntime runtime
);
public static void TiSubmit(
  TiRuntime runtime
) {
  ti_submit(
    runtime
  );
}
}

// function.wait
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_wait(
  TiRuntime runtime
);
public static void TiWait(
  TiRuntime runtime
) {
  ti_wait(
    runtime
  );
}
}

// function.load_aot_module
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern TiAotModule ti_load_aot_module(
  TiRuntime runtime,
  string module_path
);
public static TiAotModule TiLoadAotModule(
  TiRuntime runtime,
  string module_path
) {
  var rv = ti_load_aot_module(
    runtime,
    module_path
  );
  return rv;
}
}

// function.destroy_aot_module
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern void ti_destroy_aot_module(
  TiAotModule aot_module
);
public static void TiDestroyAotModule(
  TiAotModule aot_module
) {
  ti_destroy_aot_module(
    aot_module
  );
}
}

// function.get_aot_module_kernel
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern TiKernel ti_get_aot_module_kernel(
  TiAotModule aot_module,
  string name
);
public static TiKernel TiGetAotModuleKernel(
  TiAotModule aot_module,
  string name
) {
  var rv = ti_get_aot_module_kernel(
    aot_module,
    name
  );
  return rv;
}
}

// function.get_aot_module_compute_graph
static partial class Ffi {
#if (UNITY_IOS || UNITY_TVOS || UNITY_WEBGL) && !UNITY_EDITOR
    [DllImport ("__Internal")]
#else
    [DllImport("taichi_c_api")]
#endif
private static extern TiComputeGraph ti_get_aot_module_compute_graph(
  TiAotModule aot_module,
  string name
);
public static TiComputeGraph TiGetAotModuleComputeGraph(
  TiAotModule aot_module,
  string name
) {
  var rv = ti_get_aot_module_compute_graph(
    aot_module,
    name
  );
  return rv;
}
}

} // namespace Taichi
