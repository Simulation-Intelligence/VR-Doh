// Frosted-glass panel behind a story line (fake frost, no blur): rounded rectangle, soft white fill that is a little
// brighter at the top, faint grain, a thin bright edge and a very soft shadow below. Premultiplied alpha
// (Blend One OneMinusSrcAlpha), so the eye buffer's alpha never goes above 1: over passthrough (alpha 0) the room
// shows through the panel, over the garden the garden does. Drawn on a quad that is bigger than the panel by the
// shadow margin; _Size = panel w, h and quad w, h (m), set per renderer with a property block. Stereo-instancing safe.
Shader "MoonRabbit/GlassPanel"
{
    Properties
    {
        _Color ("Fill (alpha = opacity)", Color) = (1,1,1,0.18)
        _TopLight ("Extra fill at the top", Range(0,1)) = 0.35
        _BorderColor ("Edge (alpha = opacity)", Color) = (1,0.98,0.94,0.55)
        _Border ("Edge width (m)", Float) = 0.004
        _Radius ("Corner radius (m)", Float) = 0.05
        _Grain ("Frost grain", Range(0,0.5)) = 0.12
        _ShadowAlpha ("Shadow opacity", Range(0,1)) = 0.22
        _ShadowSoft ("Shadow softness (m)", Float) = 0.05
        _ShadowOffset ("Shadow offset down (m)", Float) = 0.018
        _Size ("Panel w,h / quad w,h (m)", Vector) = (1,0.3,1.14,0.44)
        _Alpha ("Alpha", Range(0,1)) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" "IgnoreProjector"="True" }
        Blend One OneMinusSrcAlpha
        ZWrite Off
        Cull Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            fixed4 _Color, _BorderColor;
            float _TopLight, _Border, _Radius, _Grain, _ShadowAlpha, _ShadowSoft, _ShadowOffset, _Alpha;
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

            // signed distance to a rounded rectangle with half size b and corner radius r
            float RoundRect(float2 p, float2 b, float r)
            {
                float2 q = abs(p) - b + r;
                return length(max(q, 0)) + min(max(q.x, q.y), 0) - r;
            }

            float Hash(float2 p) { return frac(sin(dot(p, float2(12.9898, 78.233))) * 43758.5453); }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float2 p = (i.uv - 0.5) * _Size.zw;            // metres from the panel's centre
                float2 hb = _Size.xy * 0.5;
                float r = min(_Radius, min(hb.x, hb.y));
                float d = RoundRect(p, hb, r);
                float aa = max(fwidth(d), 1e-4);

                float inside = 1 - smoothstep(-aa, aa, d);
                // fill: soft white, a little brighter towards the top, faint frost grain (fixed to the panel)
                float top = saturate(p.y / max(hb.y, 1e-3) * 0.5 + 0.5);
                float grain = (Hash(floor(p * 900)) - 0.5) * _Grain;
                float aFill = inside * saturate(_Color.a * (1 + _TopLight * top + grain));
                float3 cFill = _Color.rgb * aFill;
                // thin bright edge just inside the outline
                float edge = (1 - smoothstep(-aa, aa, d)) * smoothstep(-_Border - aa, -_Border + aa, d);
                float aEdge = edge * _BorderColor.a * (0.75 + 0.25 * top);
                float3 cEdge = _BorderColor.rgb * aEdge;
                // edge over fill (premultiplied)
                float3 c = cEdge + cFill * (1 - aEdge);
                float a = aEdge + aFill * (1 - aEdge);
                // very soft shadow under the panel (darkens what is behind, outside the panel only)
                float ds = RoundRect(p + float2(0, _ShadowOffset), hb, r);
                float sh = (1 - smoothstep(-_ShadowSoft, _ShadowSoft, ds)) * _ShadowAlpha * (1 - inside);
                a = a + sh * (1 - a);
                return fixed4(c, a) * _Alpha;
            }
            ENDCG
        }
    }
    FallBack Off
}
