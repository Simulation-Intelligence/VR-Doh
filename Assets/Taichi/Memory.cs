using System;
using Taichi.Generated;

namespace Taichi {
    public class Memory : IDisposable {
        public readonly int Size;
        public readonly bool HostRead;
        public readonly bool HostWrite;
        public readonly TiMemoryUsageFlagBits Usage;

        public readonly TiMemory Handle;

        public Memory(int size, bool hostRead, bool hostWrite, TiMemoryUsageFlagBits usage) {
            HostRead = hostRead;
            HostWrite = hostWrite;
            Size = size;
            Usage = usage;
            var allocate_info = new TiMemoryAllocateInfo {
                host_read = hostRead ? 1u : 0u,
                host_write = hostWrite ? 1u : 0u,
                size = (ulong)size,
                usage = usage,
            };
            Handle = Ffi.TiAllocateMemory(Runtime.Singleton.Handle, allocate_info);
        }
        public Memory(int size, bool hostRead, bool hostWrite) :
            this(size, hostRead, hostWrite, TiMemoryUsageFlagBits.TI_MEMORY_USAGE_STORAGE_BIT) { }
        public Memory(int size) :
            this(size, false, false) { }



        public void Read<T>(T[] dst) {
            if (!HostRead) {
                throw new InvalidOperationException("Read from non-host-readable memory is not allowed");
            } else {
                if (typeof(T) == typeof(byte)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as byte[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(sbyte)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as sbyte[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(short)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as short[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(ushort)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as ushort[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(int)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as int[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(uint)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as uint[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(long)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as long[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(ulong)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as ulong[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(IntPtr)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as IntPtr[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(float)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as float[], 0, ToTiMemorySlice());
                } else if (typeof(T) == typeof(double)) {
                    Ffi.TixCopyMemoryDeviceToHostUnity(Runtime.Singleton.Handle, dst as double[], 0, ToTiMemorySlice());
                } else {
                    throw new InvalidOperationException("Unsupported type for read access");
                }
            }
        }
        public void Write<T>(T[] src) {
            if (!HostWrite) {
                throw new InvalidOperationException("Write to non-host-writable memory is not allowed");
            } else {
                if (typeof(T) == typeof(byte)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as byte[], 0);
                } else if (typeof(T) == typeof(sbyte)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as sbyte[], 0);
                } else if (typeof(T) == typeof(short)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as short[], 0);
                } else if (typeof(T) == typeof(ushort)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as ushort[], 0);
                } else if (typeof(T) == typeof(int)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as int[], 0);
                } else if (typeof(T) == typeof(uint)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as uint[], 0);
                } else if (typeof(T) == typeof(long)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as long[], 0);
                } else if (typeof(T) == typeof(ulong)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as ulong[], 0);
                } else if (typeof(T) == typeof(IntPtr)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as IntPtr[], 0);
                } else if (typeof(T) == typeof(float)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as float[], 0);
                } else if (typeof(T) == typeof(double)) {
                    Ffi.TixCopyMemoryHostToDeviceUnity(Runtime.Singleton.Handle, ToTiMemorySlice(), src as double[], 0);
                } else {
                    throw new InvalidOperationException("Unsupported type for write access");
                }
            }
        }
        public void CopyTo(Memory memory) {
            Ffi.TiCopyMemoryDeviceToDevice(Runtime.Singleton.Handle, ToTiMemorySlice(), memory.ToTiMemorySlice());
        }
        public void CopyToNativeBufferAsync(IntPtr nativeBufferPtr) {
            Ffi.TixCopyMemoryToNativeBufferAsyncUnity(Runtime.Singleton.Handle, new TixNativeBufferUnity { Inner = nativeBufferPtr }, 0, ToTiMemorySlice());
        }



        public TiMemorySlice ToTiMemorySlice() => ToTiMemorySlice(0, Size);
        public TiMemorySlice ToTiMemorySlice(int offset, int size) {
            if (offset < 0) {
                throw new ArgumentOutOfRangeException(nameof(offset));
            }
            if (size < 0 || size > Size) {
                throw new ArgumentOutOfRangeException(nameof(size));
            }
            return new TiMemorySlice {
                memory = Handle,
                offset = (ulong)offset,
                size = (ulong)size,
            };
        }

        public bool IsDisposed => disposedValue;
        private bool disposedValue;
        protected virtual void Dispose(bool disposing) {
            if (!disposedValue) {
                if (disposing) {
                }

                if (!Runtime.Singleton.IsDisposed && Handle.Inner != null) {
                    Ffi.TiFreeMemory(Runtime.Singleton.Handle, Handle);
                }
                disposedValue = true;
            }
        }
        ~Memory() {
            Dispose(disposing: false);
        }
        public void Dispose() {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }
}
