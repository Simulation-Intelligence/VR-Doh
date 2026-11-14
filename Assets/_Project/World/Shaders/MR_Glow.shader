// Additive, fog-free halo quad (Moon glow, Earth atmosphere, lantern glow).
// Cheap replacement for bloom. Single-pass-instanced stereo safe.
Shader "MoonRabbit/Glow"
{
    Properties
    {
        _MainTex ("Radial Mask", 2D) = "white" {}
        _Color ("Color", Color) = (0.87,0.9,0.94,1)
        _Intensity ("Intensity", Range(0,4)) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" "IgnoreProjector"="True" }
        Blend One One
        ZWrite Off
        Cull Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_ST;
            fixed4 _Color;
            half _Intensity;

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            v2f vert (appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                fixed mask = tex2D(_MainTex, i.uv).r;
                return fixed4(_Color.rgb * mask * _Intensity, 1);
            }
            ENDCG
        }
    }
    FallBack Off
}
