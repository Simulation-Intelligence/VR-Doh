// Steamed rice grain for the rice-to-mochi overlay (instanced, one draw call).
// Translucent-looking white with a soft sheen; shaded toward the warm dough light (_WarmPos) with a tint matched to
// dough light (_WarmPos/_WarmColor), because instanced Graphics draws get neither that Spot's add pass nor, in
// Built-in, the main light and ambient SH; RiceToMochi passes all of them in (_MainDir/_MainColor/_Amb*).
// _Melt (0..1, per draw) blends the grain toward the dough's colour and sheen so the borders fade as it melts in.
Shader "MoonRabbit/RiceGrain"
{
    Properties
    {
        _Color ("Rice", Color) = (0.96,0.95,0.92,1)
        _DoughColor ("Dough", Color) = (0.965,0.955,0.925,1)
        _Melt ("Melt", Range(0,1)) = 0
        _Sheen ("Sheen", Range(0,1)) = 0.35
        _Translucency ("Translucency", Range(0,1)) = 0.35
        _WarmPos ("Warm light pos (xyz) range (w)", Vector) = (0,0,0,0)
        _WarmColor ("Warm light on (alpha)", Color) = (0,0,0,1)
        _Lit ("Lit tint (how the warm-lit dough reads, linear)", Color) = (0.95,0.8,0.71,1)
        _MainDir ("Main light direction (toward light)", Vector) = (0,1,0,0)
        _MainColor ("Main light colour x intensity", Color) = (0.6,0.65,0.75,1)
        _AmbSky ("Ambient sky", Color) = (0.3,0.35,0.5,1)
        _AmbEquator ("Ambient equator", Color) = (0.2,0.24,0.35,1)
        _AmbGround ("Ambient ground", Color) = (0.08,0.09,0.12,1)
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile_fog
            #include "UnityCG.cginc"

            fixed4 _Color, _DoughColor, _WarmColor, _MainColor, _AmbSky, _AmbEquator, _AmbGround, _Lit;
            float4 _WarmPos, _MainDir;
            half _Melt, _Sheen, _Translucency;

            struct appdata { float4 vertex : POSITION; float3 normal : NORMAL; UNITY_VERTEX_INPUT_INSTANCE_ID };
            struct v2f
            {
                float4 pos : SV_POSITION;
                float3 n : TEXCOORD0;
                float3 wpos : TEXCOORD1;
                UNITY_FOG_COORDS(2)
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
                UNITY_TRANSFER_FOG(o, o.pos);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float3 n = normalize(i.n);
                float3 v = normalize(_WorldSpaceCameraPos - i.wpos);
                fixed3 albedo = lerp(_Color.rgb, _DoughColor.rgb, _Melt);

                // Look-matched to the dough (its own lighting is subsurface + clipped warm light, not worth
                // reproducing): the warm-lit tint shaded softly toward the warm light, plus a share of moon and ambient.
                float3 L = normalize(_MainDir.xyz);
                half wrap = saturate((dot(n, L) + 0.5) / 1.5);
                half3 amb = n.y > 0 ? lerp(_AmbEquator.rgb, _AmbSky.rgb, n.y) : lerp(_AmbEquator.rgb, _AmbGround.rgb, -n.y);
                float3 Lw = normalize(_WarmPos.xyz - i.wpos);
                half ww = saturate((dot(n, Lw) + 0.4) / 1.4);
                fixed3 c = albedo * _Lit.rgb * _WarmColor.a * (0.62 + 0.28 * ww) + albedo * (amb + _MainColor.rgb * wrap) * 0.8; // brightens with the Moon like the dough

                // light sheen (grains glisten a little; fades as they melt into the matte dough)
                float3 h = normalize(Lw + v);
                c += pow(saturate(dot(n, h)), 40) * _Sheen * (1 - 0.7 * _Melt) * _Lit.rgb;

                // translucent edges
                half rim = pow(1 - saturate(dot(n, v)), 3) * _Translucency * (1 - _Melt);
                c += albedo * rim * 0.25;

                UNITY_APPLY_FOG(i.fogCoord, c);
                return fixed4(c, 1);
            }
            ENDCG
        }
    }
    FallBack Off
}
