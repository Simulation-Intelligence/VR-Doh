// Punches a hole in the virtual scene so passthrough shows through: writes depth and clears colour
// and alpha. Drawn on the tracked hand meshes before everything else, it lets the real hands stay
// visible in front of the mochi and the usu instead of being painted over by them.
Shader "Mochi/PassthroughCutout"
{
    SubShader
    {
        Tags { "RenderType" = "Opaque" "Queue" = "Geometry-100" }
        Pass
        {
            ZWrite On
            ZTest LEqual
            Cull Back
            Blend Off
            ColorMask RGBA

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            v2f vert(appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.pos = UnityObjectToClipPos(v.vertex);
                return o;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                return fixed4(0, 0, 0, 0);
            }
            ENDCG
        }
    }
}
