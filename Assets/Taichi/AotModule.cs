using System;
using Taichi.Generated;

namespace Taichi {

    public class AotModule : IDisposable {
        public readonly TiAotModule Handle;

        public AotModule(string module_path) {
            Handle = Ffi.TiLoadAotModule(Runtime.Singleton.Handle, module_path);
        }

        public Kernel GetKernel(string name) {
            return new Kernel(this, Ffi.TiGetAotModuleKernel(Handle, name), name);
        }
        public ComputeGraph GetComputeGraph(string name) {
            return new ComputeGraph(this, Ffi.TiGetAotModuleComputeGraph(Handle, name), name);
        }



        public bool IsDisposed => disposedValue;
        private bool disposedValue;
        protected virtual void Dispose(bool disposing) {
            if (!disposedValue) {
                if (disposing) {
                }

                if (Handle.Inner != null) {
                    Ffi.TiDestroyAotModule(Handle);
                }
                disposedValue = true;
            }
        }
        ~AotModule() {
            Dispose(disposing: false);
        }
        public void Dispose() {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
