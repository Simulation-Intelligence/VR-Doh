using UnityEngine;
using UnityEngine.Rendering;

namespace MarchingCubes
{

  //
  // Isosurface mesh builder with the marching cubes algorithm
  //
  sealed class MeshBuilder : System.IDisposable
  {
    #region Public members

    public Mesh Mesh => _mesh;

    public MeshBuilder(int x, int y, int z, int budget, ComputeShader compute)
      => Initialize((x, y, z), budget, compute);

    public MeshBuilder(Vector3Int dims, int budget, ComputeShader compute)
      => Initialize((dims.x, dims.y, dims.z), budget, compute);

    public void Dispose()
      => ReleaseAll();

    public void BuildIsosurface(ComputeBuffer voxels, float target, float scale)
      => RunCompute(voxels, target, scale);

    #endregion

    #region Private members

    (int x, int y, int z) _grids;
    int _triangleBudget;

    public int _smoothingIterations = 0;
    ComputeShader _compute;

    void Initialize((int, int, int) dims, int budget, ComputeShader compute)
    {
      _grids = dims;
      _triangleBudget = budget;
      _compute = compute;

      AllocateBuffers();
      AllocateMesh(3 * _triangleBudget);
    }

    void ReleaseAll()
    {
      ReleaseBuffers();
      ReleaseMesh();
    }

    void RunCompute(ComputeBuffer voxels, float target, float scale)
    {
      _vertexCounterBuffer.SetCounterValue(0);
      _triangleCounterBuffer.SetCounterValue(0);
      _counterBuffer.SetCounterValue(0);

      //initialize vertex mapping buffer
      if (_smoothingIterations > 0)
      {
        int kernelInitVertexMapping = _compute.FindKernel("InitVertexMapping");
        _compute.SetBuffer(kernelInitVertexMapping, "VertexMapping", _vertexMappingBuffer);
        _compute.DispatchThreads(kernelInitVertexMapping, _grids.x * _grids.y * _grids.z * 3, 1, 1);

        //initialize vertex mapping buffer
        int kernelInitNeighborCount = _compute.FindKernel("InitNeighborCount");
        _compute.SetBuffer(kernelInitNeighborCount, "NeighborCount", _neighborCountBuffer);
        _compute.DispatchThreads(kernelInitNeighborCount, _triangleBudget * 3, 1, 1);
      }
      // Isosurface reconstruction
      _compute.SetInts("Dims", _grids);
      _compute.SetInt("MaxNeighbors", MaxNeighbors);
      _compute.SetInt("MaxTriangle", _triangleBudget);
      _compute.SetFloat("Scale", scale);
      _compute.SetFloat("Isovalue", target);
      int kernel;
      if (_smoothingIterations > 0)
      {
        kernel = _compute.FindKernel("MeshReconstruction");
      }
      else
      {
        kernel = _compute.FindKernel("MeshReconstructionNoSmoothing");
      }
      _compute.SetBuffer(kernel, "TriangleTable", _triangleTable);
      _compute.SetBuffer(kernel, "Voxels", voxels);
      _compute.SetBuffer(kernel, "VertexBuffer", _vertexBuffer);
      _compute.SetBuffer(kernel, "IndexBuffer", _indexBuffer);
      _compute.SetBuffer(kernel, "VertexCounter", _vertexCounterBuffer);
      _compute.SetBuffer(kernel, "TriangleCounter", _triangleCounterBuffer);
      _compute.SetBuffer(kernel, "Counter", _counterBuffer);
      _compute.SetBuffer(kernel, "VertexMapping", _vertexMappingBuffer);
      _compute.SetBuffer(kernel, "AdjacencyList", _adjacencyListBuffer);
      _compute.SetBuffer(kernel, "NeighborCount", _neighborCountBuffer);
      _compute.DispatchThreads(kernel, _grids);
      if (_smoothingIterations > 0)
      {
        kernel = _compute.FindKernel("ClearUnusedIndices");
        // Clear unused area of the buffers.
        _compute.SetBuffer(kernel, "VertexBuffer", _vertexBuffer);
        _compute.SetBuffer(kernel, "IndexBuffer", _indexBuffer);
        _compute.SetBuffer(kernel, "VertexCounter", _vertexCounterBuffer);
        _compute.DispatchThreads(kernel, 1024, 1, 1);

        kernel = _compute.FindKernel("ClearUnusedTriangles");
        _compute.SetBuffer(kernel, "IndexBuffer", _indexBuffer);
        _compute.SetBuffer(kernel, "TriangleCounter", _triangleCounterBuffer);
        _compute.DispatchThreads(2, 1024, 1, 1);

        // Laplacian smoothing
        LaplacianSmoothing(_smoothingIterations);
      }
      else
      {
        kernel = _compute.FindKernel("ClearUnused");
        _compute.SetBuffer(kernel, "VertexBuffer", _vertexBuffer);
        _compute.SetBuffer(kernel, "IndexBuffer", _indexBuffer);
        _compute.SetBuffer(kernel, "Counter", _counterBuffer);
        _compute.DispatchThreads(kernel, 1024, 1, 1);
      }


      // Bounding box
      var ext = new Vector3(_grids.x, _grids.y, _grids.z) * scale;
      _mesh.bounds = new Bounds(Vector3.zero, ext);


    }

    void LaplacianSmoothing(int iterations)
    {
      int kernel = _compute.FindKernel("LaplacianSmoothing");
      _compute.SetBuffer(kernel, "VertexBuffer", _vertexBuffer);
      _compute.SetBuffer(kernel, "AdjacencyList", _adjacencyListBuffer);
      _compute.SetBuffer(kernel, "NeighborCount", _neighborCountBuffer);
      for (var i = 0; i < iterations; i++)
        _compute.DispatchThreads(kernel, 3 * _triangleBudget, 1, 1);

    }

    #endregion

    #region Compute buffer objects

    ComputeBuffer _triangleTable;
    ComputeBuffer _vertexCounterBuffer;

    ComputeBuffer _counterBuffer;

    ComputeBuffer _triangleCounterBuffer;

    ComputeBuffer _vertexMappingBuffer;

    ComputeBuffer _adjacencyListBuffer;

    ComputeBuffer _neighborCountBuffer;


    const int MaxNeighbors = 20;

    void AllocateBuffers()
    {
      // Marching cubes triangle table
      _triangleTable = new ComputeBuffer(256, sizeof(ulong));
      _triangleTable.SetData(PrecalculatedData.TriangleTable);

      // Buffer for triangle counting
      _vertexCounterBuffer = new ComputeBuffer(1, 4, ComputeBufferType.Counter);
      _triangleCounterBuffer = new ComputeBuffer(1, 4, ComputeBufferType.Counter);
      _counterBuffer = new ComputeBuffer(1, 4, ComputeBufferType.Counter);


      // Buffer for vertex mapping
      _vertexMappingBuffer = new ComputeBuffer(_grids.x * _grids.y * _grids.z * 3, 4);

      // Buffer for adjacency list
      _adjacencyListBuffer = new ComputeBuffer(MaxNeighbors * _triangleBudget * 3, 4);

      // Buffer for neighbor count
      _neighborCountBuffer = new ComputeBuffer(_triangleBudget * 3, 4);


    }

    void ReleaseBuffers()
    {
      _triangleTable.Dispose();
      _vertexCounterBuffer.Dispose();
      _triangleCounterBuffer.Dispose();
      _vertexMappingBuffer.Dispose();
      _adjacencyListBuffer.Dispose();
      _neighborCountBuffer.Dispose();
    }

    #endregion

    #region Mesh objects

    Mesh _mesh;
    GraphicsBuffer _vertexBuffer;
    GraphicsBuffer _indexBuffer;

    void AllocateMesh(int vertexCount)
    {
      _mesh = new Mesh();

      // We want GraphicsBuffer access as Raw (ByteAddress) buffers.
      _mesh.indexBufferTarget |= GraphicsBuffer.Target.Raw;
      _mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;

      // Vertex position: float32 x 3
      var vp = new VertexAttributeDescriptor
        (VertexAttribute.Position, VertexAttributeFormat.Float32, 3);

      // Vertex normal: float32 x 3
      var vn = new VertexAttributeDescriptor
        (VertexAttribute.Normal, VertexAttributeFormat.Float32, 3);

      // Vertex/index buffer formats
      _mesh.SetVertexBufferParams(vertexCount, vp, vn);
      _mesh.SetIndexBufferParams(vertexCount, IndexFormat.UInt32);

      // Submesh initialization
      _mesh.SetSubMesh(0, new SubMeshDescriptor(0, vertexCount),
                       MeshUpdateFlags.DontRecalculateBounds);

      // GraphicsBuffer references
      _vertexBuffer = _mesh.GetVertexBuffer(0);
      _indexBuffer = _mesh.GetIndexBuffer();
    }

    void ReleaseMesh()
    {
      _vertexBuffer.Dispose();
      _indexBuffer.Dispose();
      Object.Destroy(_mesh);
    }

    #endregion
  }

} // namespace MarchingCubes
