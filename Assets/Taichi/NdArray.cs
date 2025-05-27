using System.Runtime.InteropServices;
using System;
using Taichi.Generated;

namespace Taichi
{
    public class NdArray<T> : IDisposable where T : struct
    {
        private int[] _Shape;
        private int[] _ElemShape;
        private Memory _Memory;



        public int[] Shape => _Shape;
        public int[] ElemShape => _ElemShape;
        public int Count => Shape2Count(Shape) * Shape2Count(ElemShape);
        public int SizeInBytes => Count * Marshal.SizeOf(typeof(T));



        private static int Shape2Count(int[] shape)
        {
            int size = 1;
            foreach (var dim in shape)
            {
                size *= dim;
            }
            if (size == 0)
            {
                throw new ArgumentException("NdArray shape cannot be zero");
            }
            return size;
        }



        public NdArray(int[] shape, int[] elemShape, bool hostRead, bool hostWrite, TiMemoryUsageFlagBits usage)
        {
            if (shape == null) shape = new int[0];
            if (elemShape == null) elemShape = new int[0];
            _Shape = shape;
            _ElemShape = elemShape;
            _Memory = new Memory(SizeInBytes, hostRead, hostWrite, usage);
        }
        public NdArray(int[] shape, bool hostRead, bool hostWrite, TiMemoryUsageFlagBits usage) : this(shape, null, hostRead, hostWrite, usage) { }
        public NdArray(int[] shape, int[] elemShape, bool hostRead, bool hostWrite) : this(shape, elemShape, hostRead, hostWrite, TiMemoryUsageFlagBits.TI_MEMORY_USAGE_STORAGE_BIT) { }
        public NdArray(int[] shape, bool hostRead, bool hostWrite) : this(shape, hostRead, hostWrite, TiMemoryUsageFlagBits.TI_MEMORY_USAGE_STORAGE_BIT) { }
        public NdArray(int[] shape, int[] elemShape) : this(shape, elemShape, false, false) { }
        public NdArray(params int[] shape) : this(shape, null) { }



        public void CopyFromArray(T[] data)
        {
            if (data.Length * Marshal.SizeOf<T>() != SizeInBytes)
            {
                throw new ArgumentException(nameof(data));
            }
            _Memory.Write(data);
        }

        public static NdArray<T> FromArray(T[] data, int[] shape)
        {
            var rv = new NdArray<T>(shape);
            rv.CopyFromArray(data);
            return rv;
        }

        public void CopyToArray(T[] data)
        {
            if (data.Length == 0)
            {
                return;
            }
            if (data.Length * Marshal.SizeOf<T>() != SizeInBytes)
            {
                throw new ArgumentException(nameof(data));
            }
            _Memory.Read(data);
        }
        public T[] ToArray()
        {
            var rv = new T[Shape2Count(Shape)];
            CopyToArray(rv);
            return rv;
        }



        public void CopyToNativeBufferAsync(IntPtr native_buffer_ptr)
        {
            if (native_buffer_ptr == IntPtr.Zero)
            {
                throw new ArgumentNullException("`native_buffer_ptr` cannot be null");
            }
            _Memory.CopyToNativeBufferAsync(native_buffer_ptr);
        }



        public TiNdArray ToTiNdArray()
        {
            var shape = new uint[16];
            for (int i = 0; i < _Shape.Length; ++i)
            {
                shape[i] = (uint)_Shape[i];
            }
            var elemShape = new uint[16];
            for (int i = 0; i < _ElemShape.Length; ++i)
            {
                elemShape[i] = (uint)_ElemShape[i];
            }

            TiDataType elemType;
            if (typeof(T) == typeof(byte))
            {
                elemType = TiDataType.TI_DATA_TYPE_U8;
            }
            else if (typeof(T) == typeof(sbyte))
            {
                elemType = TiDataType.TI_DATA_TYPE_I8;
            }
            else if (typeof(T) == typeof(short))
            {
                elemType = TiDataType.TI_DATA_TYPE_I16;
            }
            else if (typeof(T) == typeof(ushort))
            {
                elemType = TiDataType.TI_DATA_TYPE_U16;
            }
            else if (typeof(T) == typeof(int))
            {
                elemType = TiDataType.TI_DATA_TYPE_I32;
            }
            else if (typeof(T) == typeof(uint))
            {
                elemType = TiDataType.TI_DATA_TYPE_U32;
            }
            else if (typeof(T) == typeof(long))
            {
                elemType = TiDataType.TI_DATA_TYPE_I64;
            }
            else if (typeof(T) == typeof(ulong))
            {
                elemType = TiDataType.TI_DATA_TYPE_U64;
            }
            else if (typeof(T) == typeof(IntPtr))
            {
                elemType = TiDataType.TI_DATA_TYPE_GEN;
            }
            else if (typeof(T) == typeof(float))
            {
                elemType = TiDataType.TI_DATA_TYPE_F32;
            }
            else if (typeof(T) == typeof(double))
            {
                elemType = TiDataType.TI_DATA_TYPE_F64;
            }
            else
            {
                throw new InvalidOperationException("Unsupported type for read access");
            }

            return new TiNdArray
            {
                shape = new TiNdShape
                {
                    dim_count = (uint)Shape.Length,
                    dims = shape,
                },
                elem_shape = new TiNdShape
                {
                    dim_count = (uint)ElemShape.Length,
                    dims = elemShape,
                },
                elem_type = elemType,
                memory = _Memory.Handle,
            };
        }
        public TiMemorySlice ToTiMemorySlice()
        {
            return _Memory.ToTiMemorySlice(0, SizeInBytes);
        }



        public bool IsDisposed => disposedValue;
        private bool disposedValue;
        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    _Memory.Dispose();
                }
                disposedValue = true;
            }
        }
        ~NdArray()
        {
            Dispose(disposing: false);
        }
        public void Dispose()
        {
            Dispose(disposing: true);
            GC.SuppressFinalize(this);
        }
    }

    public class NdArrayBuilder<T> where T : struct
    {
        private int[] _Shape;
        private int[] _ElemShape;
        private bool _HostRead;
        private bool _HostWrite;

        public NdArrayBuilder<T> Shape(params int[] shape)
        {
            _Shape = shape;
            return this;
        }
        public NdArrayBuilder<T> ElemShape(params int[] elemShape)
        {
            _ElemShape = elemShape;
            return this;
        }
        public NdArrayBuilder<T> HostRead(bool hostRead = true)
        {
            _HostRead = hostRead;
            return this;
        }
        public NdArrayBuilder<T> HostWrite(bool hostWrite = true)
        {
            _HostWrite = hostWrite;
            return this;
        }

        public NdArray<T> Build()
        {
            return new NdArray<T>(_Shape, _ElemShape, _HostRead, _HostWrite, TiMemoryUsageFlagBits.TI_MEMORY_USAGE_STORAGE_BIT);
        }

    }
}
