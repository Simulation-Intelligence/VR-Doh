// Portal from the real room (passthrough) into the Moon garden. Drawn on the inside of a sphere around the head,
// after the scene: outside the portal it multiplies colour AND alpha by 0, so passthrough (an underlay that shows
// where alpha is 0) replaces the garden; inside the portal it leaves the garden untouched; a soft edge and a faint
// glowing rim in between. No depth test: near garden pieces open with the portal too. What must stay (usu, pedestal,
// dough, kine) is marked in the stencil (bits 32, 64, 128) by MoonRabbit/PortalKeep bubbles and left alone; hands are cut out
// (alpha 0) already.
// _Center = world point the portal opens around (the usu), _Radius = portal half-angle (radians).
Shader "MoonRabbit/PortalVeil"
{
    Properties
    {
        _Center ("Portal centre (world)", Vector) = (0,1,0,0)
        _Radius ("Portal half-angle (rad)", Float) = 0
        _Soft ("Edge softness (rad)", Float) = 0.18
        _RimColor ("Rim colour", Color) = (1,0.85,0.6,1)
        _RimIntensity ("Rim intensity", Float) = 0.6
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent+400" "IgnoreProjector"="True" }
        Cull Front
        ZWrite Off
        ZTest Always
        Stencil { Ref 0 ReadMask 224 Comp Equal }

        CGINCLUDE
        #include "UnityCG.cginc"
        float4 _Center;
        float _Radius, _Soft, _RimIntensity;
        fixed4 _RimColor;
        struct appdata { float4 vertex : POSITION; UNITY_VERTEX_INPUT_INSTANCE_ID };
        struct v2f { float4 pos : SV_POSITION; float3 wpos : TEXCOORD0; UNITY_VERTEX_OUTPUT_STEREO };
        v2f vert (appdata v)
        {
            v2f o;
            UNITY_SETUP_INSTANCE_ID(v);
            UNITY_INITIALIZE_OUTPUT(v2f, o);
            UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
            o.pos = UnityObjectToClipPos(v.vertex);
            o.wpos = mul(unity_ObjectToWorld, v.vertex).xyz;
            return o;
        }
        float Angle(float3 wpos)
        {
            float3 v = normalize(wpos - _WorldSpaceCameraPos);
            float3 c = normalize(_Center.xyz - _WorldSpaceCameraPos);
            return acos(clamp(dot(v, c), -1, 1));
        }
        ENDCG

        // 1) keep the garden inside the portal, fade to passthrough outside (colour and alpha x k)
        Pass
        {
            Blend Zero SrcAlpha
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float k = 1 - smoothstep(_Radius - _Soft, _Radius + _Soft, Angle(i.wpos));
                return fixed4(0, 0, 0, k);
            }
            ENDCG
        }
        // 2) faint glowing rim along the portal's edge (colour only)
        Pass
        {
            Blend One One
            ColorMask RGB
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);
                float d = (Angle(i.wpos) - _Radius) / max(_Soft, 1e-3);
                float rim = exp(-d * d * 4.0) * _RimIntensity * step(0.001, _Radius);
                return fixed4(_RimColor.rgb * rim, 0);
            }
            ENDCG
        }
    }
    FallBack Off
}
