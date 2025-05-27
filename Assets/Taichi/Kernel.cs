using System;
using Taichi.Generated;
using UnityEngine;

namespace Taichi {
    public class Kernel : IDisposable {
        public readonly AotModule AotModule;
        public readonly TiKernel Handle;
        public readonly string Name;

        public Kernel(AotModule aotModule, TiKernel handle, string name) {
            AotModule = aotModule;
            Handle = handle;
            Name = name;
        }

        public void LaunchAsync(TiArgument[] args) {
            if (Handle.Inner == IntPtr.Zero) {
                throw new InvalidOperationException("Ignored launch because kernel handle is null");
            }
            for (var i = 0; i < args.Length; ++i) {
                if (args[i].type == TiArgumentType.TI_ARGUMENT_TYPE_NDARRAY) {
                    if (args[i].value.ndarray.shape.dims.Length != 16) {
                        throw new ArgumentException("NdArray shape must be 16");
                    }
                    if (args[i].value.ndarray.elem_shape.dims.Length != 16) {
                        throw new ArgumentException("NdArray element shape must be 16");
                    }
                    if (args[i].value.ndarray.memory.Inner == IntPtr.Zero) {
                        Debug.LogWarning("Ignored launch because argument list contains uninitialized `TiMemory`");
                        return;
                    }
                } else {
                    // FIXME: (penguinliong) This is a workaround to avoid
                    // `NullPointerException` when clr tries to marshall these
                    // unused fields.
                    args[i].value.ndarray.shape.dims = new uint[16];
                    args[i].value.ndarray.elem_shape.dims = new uint[16];
                }
            }
            Ffi.TixLaunchKernelAsyncUnity(Runtime.Singleton.Handle, Handle, (uint)args.Length, args);
        }
        public void LaunchAsync(params object[] args) {
            var builder = new ArgumentListBuilder();
            foreach (var arg in args) {
                if (arg.GetType() == typeof(int)) {
                    builder.Add((int)arg);
                } else if (arg.GetType() == typeof(float)) {
                    builder.Add((float)arg);
                } else if (arg.GetType() == typeof(NdArray<int>)) {
                    builder.Add((NdArray<int>)arg);
                } else if (arg.GetType() == typeof(NdArray<float>)) {
                    builder.Add((NdArray<float>)arg);
                } else {
                    throw new ArgumentException("Unsupported data type; try `LaunchAsync(TiArgument[])`?");
                }
            }
            LaunchAsync(builder.ToArray());
        }


        public bool IsDisposed => disposedValue;
        private bool disposedValue;
        protected virtual void Dispose(bool disposing) {
            if (!disposedValue) {
                if (disposing) {
                }

                disposedValue = true;
            }
        }
        ~Kernel() {
            Dispose(disposing: false);
        }
        public void Dispose() {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
