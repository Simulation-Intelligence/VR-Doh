using System;
using Taichi.Generated;
using UnityEngine;
using UnityEngine.Rendering;

namespace Taichi {
    public class Runtime : IDisposable {
        public readonly TiRuntime Handle;

        public Runtime(TiRuntime handle) {
            Handle = handle;
        }



        private static Runtime _Singleton;
        public static Runtime Singleton {
            get {
                if (_Singleton == null) {
                    _Singleton = new Runtime(Ffi.TixImportNativeRuntimeUnity());
                }
                return _Singleton;
            }
        }



        public static void Submit() {
            GL.IssuePluginEvent(Ffi.TixSubmitAsyncUnity(Singleton.Handle), 0);
        }
        public static void Submit(CommandBuffer commandBuffer) {
            commandBuffer.IssuePluginEvent(Ffi.TixSubmitAsyncUnity(Singleton.Handle), 0);
        }



        public bool IsDisposed => disposedValue;
        private bool disposedValue;
        protected virtual void Dispose(bool disposing) {
            if (!disposedValue) {
                if (disposing) {
                }

                Ffi.TiDestroyRuntime(Handle);
                disposedValue = true;
            }
        }
        ~Runtime() {
            Dispose(disposing: false);
        }
        public void Dispose() {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
