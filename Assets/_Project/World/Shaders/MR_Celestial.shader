// Unlit, fog-free shader for the Moon and Earth (they sit far beyond the fog).
// Vertex colors give the surface pattern; _LightDir controls the lit side,
// so Moon phases can be animated later by rotating _LightDir.
// Single-pass-instanced stereo safe (Quest over Link).
Shader "MoonRabbit/Celestial"
{
    Properties
    {
        _Color ("Tint", Color) = (1,1,1,1)
        _LightDir ("Light Direction (world, toward light)", Vector) = (0,0,-1,0)
        _Ambient ("Dark Side Brightness", Range(0,1)) = 0.12
        _Emission ("Glow", Range(0,2)) = 0.5
        _Terminator ("Terminator Softness", Range(0.01,1)) = 0.25
        _RimColor ("Rim Color", Color) = (0.87,0.9,0.94,1)
        _RimPower ("Rim Power", Range(0.5,8)) = 3
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry+10" "IgnoreProjector"="True" }
        LOD 100
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            fixed4 _Color;
            float4 _LightDir;
            half _Ambient;
            half _Emission;
            half _Terminator;
            fixed4 _RimColor;
            half _RimPower;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 normal : NORMAL;
                fixed4 color  : COLOR;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos   : SV_POSITION;
                float3 n     : TEXCOORD0;
                float3 wpos  : TEXCOORD1;
                fixed4 color : COLOR;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            v2f vert (appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                o.n = UnityObjectToWorldNormal(v.normal);
                o.wpos = mul(unity_ObjectToWorld, v.vertex).xyz;
                o.color = v.color;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float3 n = normalize(i.n);
                float3 l = normalize(_LightDir.xyz);
                float3 v = normalize(_WorldSpaceCameraPos - i.wpos);
                float lit = smoothstep(-_Terminator, _Terminator, dot(n, l));
                fixed3 albedo = i.color.rgb * _Color.rgb;
                fixed3 col = albedo * lerp(_Ambient, 1.0, lit) + albedo * _Emission * lit;
                float rim = pow(1.0 - saturate(dot(n, v)), _RimPower);
                col += _RimColor.rgb * rim * (0.25 + 0.75 * lit);
                return fixed4(col, 1);
            }
            ENDCG
        }
    }
    FallBack Off
}
