// Hanji (Korean mulberry paper) handscroll behind the intro lines: warm cream paper, half see-through, fine fibres and
// faint blotches (code-made texture), softly torn irregular edges, thin rolled curls along the top and bottom, a paper
// roll at each moving end while it unrolls / rolls up, and gold auspicious-cloud motifs in the four corners.
// Straight alpha with Blend SrcAlpha OneMinusSrcAlpha for colour and One OneMinusSrcAlpha for alpha, so the eye
// buffer's alpha stays <= 1: over passthrough (alpha 0) the room shows through the paper, never black.
// _Size = paper w, h (fully open) and quad w, h (m); _Unroll = 0 (rolled up in the middle) .. 1 (open).
// Stereo-instancing safe.
Shader "MoonRabbit/HanjiScroll"
{
    Properties
    {
        _FiberTex ("Fibres (R), blotches (G), edge noise (B)", 2D) = "gray" {}
        _CloudTex ("Cloud motif (R), one corner", 2D) = "black" {}
        _Paper ("Paper colour", Color) = (0.97,0.93,0.82,1)
        _Opacity ("Paper opacity", Range(0,1)) = 0.62
        _Gold ("Cloud gold", Color) = (0.86,0.66,0.3,1)
        _CloudGlow ("Cloud glow", Range(0,2)) = 0
        _CloudSize ("Cloud motif size (m)", Float) = 0.13
        _Curl ("Top/bottom curl width (m)", Float) = 0.012
        _Roll ("Moving roll radius (m)", Float) = 0.022
        _Torn ("Torn edge depth (m)", Float) = 0.012
        _Size ("Paper w,h / quad w,h (m)", Vector) = (1.6,0.4,1.7,0.5)
        _Unroll ("Unroll", Range(0,1)) = 1
        _Alpha ("Alpha", Range(0,1)) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent-10" "IgnoreProjector"="True" }
        Blend SrcAlpha OneMinusSrcAlpha, One OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            sampler2D _FiberTex, _CloudTex;
            fixed4 _Paper, _Gold;
            float _Opacity, _CloudGlow, _CloudSize, _Curl, _Roll, _Torn, _Unroll, _Alpha;
            float4 _Size;

            struct appdata { float4 vertex : POSITION; float2 uv : TEXCOORD0; UNITY_VERTEX_INPUT_INSTANCE_ID };
            struct v2f { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; UNITY_VERTEX_OUTPUT_STEREO };

            v2f vert (appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float2 p = (i.uv - 0.5) * _Size.zw;                 // metres from the centre (x right, y up)
                float u = saturate(_Unroll);
                float hwFull = _Size.x * 0.5, hh = _Size.y * 0.5;
                float roll = lerp(_Roll, _Roll * 0.3, u);            // the roll thins as the paper comes off it
                float hw = max(hwFull * u, roll);                    // paper edge (also the roll's centre line)
                float ax = abs(p.x), ay = abs(p.y);

                // paper: fibres, blotches, softly torn edges (noise along the outline)
                float3 f = tex2D(_FiberTex, p * 1.6).rgb;
                float tear = (tex2D(_FiberTex, float2(p.x * 3.1, p.y * 3.1 + 0.37)).b - 0.5) * _Torn;
                float dEdge = min(hw - ax, hh - ay) + tear;
                float paperMask = smoothstep(0.0, 0.004 + _Torn * 0.4, dEdge);
                float3 col = _Paper.rgb * (0.93 + 0.1 * f.r) - f.g * 0.05;
                float a = _Opacity * (0.88 + 0.24 * f.r) * paperMask;

                // thin rolled curls along the top and bottom: a little darker, cylinder-shaded, more opaque
                float tc = saturate((ay - (hh - _Curl)) / _Curl);
                float inCurl = step(hh - _Curl, ay) * step(ay, hh + _Curl * 0.25) * step(ax, hw);
                float shade = sin(tc * 3.14159);
                col = lerp(col, _Paper.rgb * (0.62 + 0.3 * shade), inCurl);
                a = lerp(a, 0.9 * step(ax, hw - 0.002), inCurl);

                // the roll at each moving end (and a slim rod when fully open)
                float tr = (ax - (hw - roll)) / (2 * roll);          // 0..1 across the roll
                float inRoll = step(0, tr) * step(tr, 1) * step(ay, hh + _Curl * 0.25);
                float rs = sin(saturate(tr) * 3.14159);
                float3 rollCol = _Paper.rgb * (0.55 + 0.4 * rs) * (0.95 + 0.08 * f.r);
                col = lerp(col, rollCol, inRoll);
                a = lerp(a, 0.95, inRoll);

                // gold auspicious clouds in the four corners (mirrored), once the paper is (nearly) open
                float cs = min(_CloudSize, _Size.y * 0.42);
                float inset = roll * 2 + 0.012;
                float2 cuv = float2((ax - (hw - inset - cs)) / cs, (ay - (hh - _Curl - 0.008 - cs)) / cs);
                float cl = 0;
                if (cuv.x >= 0 && cuv.x <= 1 && cuv.y >= 0 && cuv.y <= 1) cl = tex2D(_CloudTex, cuv).r;
                cl *= smoothstep(0.82, 1.0, u) * (1 - inRoll);
                float3 gold = _Gold.rgb * (0.85 + 0.6 * _CloudGlow);
                col = lerp(col, gold, cl * 0.9);
                a = max(a, cl * 0.92 * paperMask);

                return fixed4(saturate(col), saturate(a) * _Alpha);
            }
            ENDCG
        }
    }
    FallBack Off
}
