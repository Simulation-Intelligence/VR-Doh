using System.Collections.Generic;
using System.Linq;
using Taichi;
using UnityEngine;

public partial class Fractal : MonoBehaviour
{
    const int WIDTH = 640;
    const int HEIGHT = 320;

    public AotModuleAsset Module;
    private Kernel _Kernel_fractal;
    private ComputeGraph _ComputeGraph_fractal;
    private NdArray<float> _Canvas;

    private MeshRenderer _MeshRenderer;
    private Texture2D _Texture;
    private Color[] _FractalDataColor;

    // Start is called before the first frame update
    void Start()
    {
        var kernels = Module.GetAllKernels().ToDictionary(x => x.Name);
        var cgraphs = Module.GetAllComputeGrpahs().ToDictionary(x => x.Name);
        if (kernels.ContainsKey("fractal"))
        {
            _Kernel_fractal = kernels["fractal"];
        }
        if (cgraphs.ContainsKey("fractal"))
        {
            _ComputeGraph_fractal = cgraphs["fractal"];
        }

        _Canvas = new NdArray<float>(new int[] { WIDTH, HEIGHT }, true, false);
        _Texture = new Texture2D(WIDTH, HEIGHT);

        _FractalDataColor = new Color[WIDTH * HEIGHT];

        _MeshRenderer = GetComponent<MeshRenderer>();
        _MeshRenderer.material.mainTexture = _Texture;
    }

    Color GetColor()
    {
        var iframe = Time.frameCount;

        float pos = iframe % 100;
        float alpha = (pos < 50) ? (pos / 50.0f) : (2.0f - pos / 50.0f);

        var color = new Color();
        color.r = alpha * 0.75f;
        color.g = 0.75f;
        color.b = (1.0f - alpha) * 0.75f;
        color.a = 1.0f;
        return color;
    }

    // Update is called once per frame
    void Update()
    {
        // Note that we are reading data from last frame here.
        var fractal = new float[WIDTH * HEIGHT];
        _Canvas.CopyToArray(fractal);
        for (int i = 0; i < _FractalDataColor.Length; ++i)
        {
            var v = fractal[i];
            _FractalDataColor[i] = new Color(v, v, v);
        }
        _Texture.SetPixels(_FractalDataColor);
        _Texture.Apply();
        _MeshRenderer.material.color = GetColor();

        // Now launch kernels and compute graphs, but it won't be
        // immediately executed on graphics device.
        float t = Time.frameCount * 0.03f;

        if (_ComputeGraph_fractal != null)
        {
            _ComputeGraph_fractal.LaunchAsync(new Dictionary<string, object> {
                { "t", t },
                { "canvas", _Canvas },
            });
        }
        if (_Kernel_fractal != null)
        {
            _Kernel_fractal.LaunchAsync(t, _Canvas);
        }
        // Everything settled. Submit launched kernels and compute graphs to
        // the device for execution. Note that we can only submit ONCE in a
        // frame and we CANNOT wait on the device, because `Update is called
        // on the GAME THREAD yet everything for rendering is created in the
        // RENDER THREAD. `TaichiRuntime.Submit` will submit the commands in
        // the RENDER THREAD.
        Runtime.Submit();
    }
}
