// The finished mochi flying to the Moon: soft warm-white ball, fog-free (it travels ~140 m into the fog),
// lit from above with a gentle rim. Single-pass-instanced stereo safe.
Shader "MoonRabbit/MochiOrb"
{
    Properties
    {
        _Color ("Colour", Color) = (1,0.97,0.9,1)
        _Shade ("Shade (lower half)", Color) = (0.62,0.56,0.58,1)
        _RimColor ("Rim", Color) = (1,0.92,0.75,1)
        _RimPower ("Rim Power", Range(0.5,8)) = 2.5
        _Glow ("Glow", Range(0,2)) = 0.12
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry+20" "IgnoreProjector"="True" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            fixed4 _Color, _Shade, _RimColor;
            half _RimPower, _Glow;

            struct appdata { float4 vertex : POSITION; float3 normal : NORMAL; UNITY_VERTEX_INPUT_INSTANCE_ID };
            struct v2f { float4 pos : SV_POSITION; float3 n : TEXCOORD0; float3 v : TEXCOORD1; UNITY_VERTEX_OUTPUT_STEREO };

            v2f vert (appdata i)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(i);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(i.vertex);
                o.n = UnityObjectToWorldNormal(i.normal);
                o.v = WorldSpaceViewDir(i.vertex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float3 n = normalize(i.n), v = normalize(i.v);
                half up = saturate(n.y * 0.5 + 0.5);
                fixed3 c = lerp(_Shade.rgb, _Color.rgb, up);
                half rim = pow(1 - saturate(dot(n, v)), _RimPower);
                c = c * 0.82 + _RimColor.rgb * rim * 0.45 + _Color.rgb * _Glow;
                return fixed4(c, 1);
            }
            ENDCG
        }
    }
    FallBack Off
}
