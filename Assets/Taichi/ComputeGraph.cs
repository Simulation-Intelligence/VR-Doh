using System;
using System.Collections.Generic;
using Taichi.Generated;
using UnityEngine;

namespace Taichi {
    public class ComputeGraph : IDisposable {
        public readonly AotModule AotModule;
        public readonly TiComputeGraph Handle;
        public readonly string Name;

        public ComputeGraph(AotModule aotModule, TiComputeGraph handle, string name) {
            AotModule = aotModule;
            Handle = handle;
            Name = name;
        }

        public void LaunchAsync(TiNamedArgument[] namedArgs) {
            if (Handle.Inner == IntPtr.Zero) {
                throw new InvalidOperationException("Ignored launch because kernel handle is null");
            }
            for (var i = 0; i < namedArgs.Length; ++i) {
                if (namedArgs[i].argument.type == TiArgumentType.TI_ARGUMENT_TYPE_NDARRAY) {
                    if (namedArgs[i].argument.value.ndarray.memory.Inner == IntPtr.Zero) {
                        Debug.LogWarning("Ignored launch because argument list contains uninitialized `TiMemory`");
                        return;
                    }
                } else {
                    // FIXME: (penguinliong) This is a workaround to avoid
                    // `NullPointerException` when clr tries to marshall these
                    // unused fields.
                    namedArgs[i].argument.value.ndarray.shape.dims = new uint[16];
                    namedArgs[i].argument.value.ndarray.elem_shape.dims = new uint[16];
                }
            }
            Ffi.TixLaunchComputeGraphAsyncUnity(Runtime.Singleton.Handle, Handle, (uint)namedArgs.Length, namedArgs);
        }
        public void LaunchAsync(Dictionary<string, object> namedArgs) {
            var builder = new NamedArgumentListBuilder();
            foreach (var arg in namedArgs) {
                if (arg.Value.GetType() == typeof(int)) {
                    builder.Add(arg.Key, (int)arg.Value);
                } else if (arg.Value.GetType() == typeof(float)) {
                    builder.Add(arg.Key, (float)arg.Value);
                } else if (arg.Value.GetType() == typeof(NdArray<int>)) {
                    builder.Add(arg.Key, (NdArray<int>)arg.Value);
                } else if (arg.Value.GetType() == typeof(NdArray<float>)) {
                    builder.Add(arg.Key, (NdArray<float>)arg.Value);
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
        ~ComputeGraph() {
            Dispose(disposing: false);
        }
        public void Dispose() {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
