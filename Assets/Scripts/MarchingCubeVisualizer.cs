using UnityEngine;

namespace MarchingCubes
{

    class MarchingCubeVisualizer : MonoBehaviour
    {
        #region Editable attributes
        public Vector3Int _dimensions = new Vector3Int(64, 64, 64);
        public float _gridScale = 1.0f / 64;
        [SerializeField] int _triangleBudget = 65536 * 16;

        #endregion

        #region Project asset references
        public ComputeShader _builderCompute = null;

        #endregion

        #region Target isovalue

        public float TargetValue = 0.4f;
        public bool shouldUpdate = false;

        #endregion

        #region Private members

        int VoxelCount => _dimensions.x * _dimensions.y * _dimensions.z;

        public ComputeBuffer _voxelBuffer;

        public int _smoothingIterations = 0;

        public bool use_own_smoothing = false;
        MeshBuilder _builder;

        #endregion

        #region MonoBehaviour implementation

        public void Init()
        {
            _voxelBuffer = new ComputeBuffer(VoxelCount, sizeof(float));
            _builder = new MeshBuilder(_dimensions, _triangleBudget, _builderCompute);
        }

        public void OnDestroy()
        {
            _voxelBuffer.Dispose();
            _builder.Dispose();
        }

        void Update()
        {
            if (use_own_smoothing)
            {
                _builder._smoothingIterations = _smoothingIterations;
            }
            if (!shouldUpdate)
            {
                return;
            }
            _builder.BuildIsosurface(_voxelBuffer, TargetValue, _gridScale);
            GetComponent<MeshFilter>().sharedMesh = _builder.Mesh;
            shouldUpdate = false;
        }

        public void SetSmoothingIterations(int iterations)
        {
            _builder._smoothingIterations = iterations;
        }

        #endregion
    }

} // namespace MarchingCubes
