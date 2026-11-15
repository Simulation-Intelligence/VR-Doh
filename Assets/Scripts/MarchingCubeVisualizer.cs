using UnityEngine;

namespace MarchingCubes
{

    // After the MPM sim (100) has written this frame's density field, so the mesh isn't a frame behind.
    [DefaultExecutionOrder(110)]
    public class MarchingCubeVisualizer : MonoBehaviour
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

        [Header("Field filter (smooths the splatted density before meshing)")]
        [Tooltip("Gaussian sigma in render voxels. 0 = off. About 1.0-1.5 hides particle graininess without losing folds.")]
        public float filterSigma = 0f;
        [Tooltip("Weight of the new frame when blending with the previous field (1 = no temporal smoothing). Lower values remove shimmer but add visual lag.")]
        [Range(0.1f, 1f)] public float temporalBlend = 1f;
        ComputeShader _filter;
        ComputeBuffer _filterTmp, _filterHistory;
        bool _historyValid;

        #endregion

        #region MonoBehaviour implementation

        public void Init()
        {
            _voxelBuffer = new ComputeBuffer(VoxelCount, sizeof(float));
            _builder = new MeshBuilder(_dimensions, _triangleBudget, _builderCompute);
        }

        public void OnDestroy()
        {
            _voxelBuffer?.Dispose();
            _builder?.Dispose();
            _filterTmp?.Dispose(); _filterTmp = null;
            _filterHistory?.Dispose(); _filterHistory = null;
            _historyValid = false;
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
            _builder.BuildIsosurface(Filter(_voxelBuffer), TargetValue, _gridScale);
            GetComponent<MeshFilter>().sharedMesh = _builder.Mesh;
            shouldUpdate = false;
        }

        ComputeBuffer Filter(ComputeBuffer field)
        {
            if (filterSigma <= 0.01f && temporalBlend >= 0.999f) return field;
            if (_filter == null) _filter = Resources.Load<ComputeShader>("MochiFieldFilter");
            if (_filter == null) return field;
            if (_filterTmp == null || _filterTmp.count != field.count)
            {
                _filterTmp?.Dispose(); _filterHistory?.Dispose();
                _filterTmp = new ComputeBuffer(field.count, sizeof(float));
                _filterHistory = new ComputeBuffer(field.count, sizeof(float));
                _historyValid = false;
            }
            var dims = _dimensions;
            int gx = (dims.x + 7) / 8, gy = (dims.y + 7) / 8, gz = (dims.z + 7) / 8;
            _filter.SetInts("Dims", dims.x, dims.y, dims.z);
            ComputeBuffer result = field;
            if (filterSigma > 0.01f)
            {
                int radius = Mathf.Clamp(Mathf.CeilToInt(filterSigma * 2.5f), 1, 4);
                var w = new float[5]; float total = 0f;
                for (int i = 0; i <= 4; i++) { w[i] = i <= radius ? Mathf.Exp(-0.5f * i * i / (filterSigma * filterSigma)) : 0f; total += i == 0 ? w[i] : 2f * w[i]; }
                for (int i = 0; i <= 4; i++) w[i] /= total;
                _filter.SetVector("W0123", new Vector4(w[0], w[1], w[2], w[3]));
                _filter.SetFloat("W4", w[4]);
                _filter.SetInt("Radius", radius);
                int k = _filter.FindKernel("Blur");
                // x: field -> tmp, y: tmp -> field, z: field -> tmp (the field is rewritten by the sim every frame).
                Pass(k, field, _filterTmp, 1, 0, 0, gx, gy, gz);
                Pass(k, _filterTmp, field, 0, 1, 0, gx, gy, gz);
                Pass(k, field, _filterTmp, 0, 0, 1, gx, gy, gz);
                result = _filterTmp;
            }
            if (temporalBlend < 0.999f)
            {
                int k = _filter.FindKernel("Temporal");
                if (result != _filterTmp) { Pass(_filter.FindKernel("Blur"), field, _filterTmp, 0, 0, 0, gx, gy, gz, true); result = _filterTmp; }
                _filter.SetFloat("Blend", _historyValid ? temporalBlend : 1f);
                _filter.SetBuffer(k, "Dst", result);
                _filter.SetBuffer(k, "History", _filterHistory);
                _filter.Dispatch(k, gx, gy, gz);
                _historyValid = true;
            }
            return result;
        }

        void Pass(int k, ComputeBuffer src, ComputeBuffer dst, int ax, int ay, int az, int gx, int gy, int gz, bool copy = false)
        {
            if (copy) { _filter.SetInt("Radius", 0); _filter.SetVector("W0123", new Vector4(1, 0, 0, 0)); }
            _filter.SetInts("Axis", ax, ay, az);
            _filter.SetBuffer(k, "Src", src);
            _filter.SetBuffer(k, "Dst", dst);
            _filter.Dispatch(k, gx, gy, gz);
        }

        public void SetSmoothingIterations(int iterations)
        {
            _builder._smoothingIterations = iterations;
        }

        #endregion
    }

} // namespace MarchingCubes
