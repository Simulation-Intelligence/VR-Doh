// Marks, in its own stencil bit (_KeepBit: 32 pedestal, 64 usu bowl, 128 kine), the pixels whose visible surface lies inside this mesh (a sphere around the usu, one
// around the kine), so the passthrough portal veil (MoonRabbit/PortalVeil) leaves them alone. Draws no colour.
// Front faces in front of the surface set the bit; back faces in front of the surface (surface beyond the volume)
// clear it again. Drawn just before the veil. Also used as an extra material slot on a mesh itself (the pedestal):
// then it marks exactly the pixels where that mesh is the visible surface.
Shader "MoonRabbit/PortalKeep"
{
    Properties { _KeepBit ("Stencil bit", Int) = 64 }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent+390" "IgnoreProjector"="True" }
        ZWrite Off
        ZTest LEqual
        ColorMask 0

        CGINCLUDE
        #include "UnityCG.cginc"
        struct appdata { float4 vertex : POSITION; UNITY_VERTEX_INPUT_INSTANCE_ID };
        struct v2f { float4 pos : SV_POSITION; UNITY_VERTEX_OUTPUT_STEREO };
        v2f vert (appdata v)
        {
            v2f o;
            UNITY_SETUP_INSTANCE_ID(v);
            UNITY_INITIALIZE_OUTPUT(v2f, o);
            UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
            o.pos = UnityObjectToClipPos(v.vertex);
            return o;
        }
        fixed4 frag (v2f i) : SV_Target { UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i); return 0; }
        ENDCG

        Pass
        {
            Cull Back
            Stencil { Ref [_KeepBit] WriteMask [_KeepBit] Comp Always Pass Replace }
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            ENDCG
        }
        Pass
        {
            Cull Front
            Stencil { Ref 0 WriteMask [_KeepBit] Comp Always Pass Replace }
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            ENDCG
        }
    }
    FallBack Off
}
