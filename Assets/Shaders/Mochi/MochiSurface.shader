// Mochi surface for the Built-in render pipeline.
// Physically-based base (StandardSpecular, so GI / probes / shadows still work) plus:
//  - wrap diffuse tinted toward a warm subsurface colour (soft, not plastic)
//  - cheap view-dependent translucency where light is behind the dough
//  - a broad satin sheen at grazing angles (the starchy "skin")
//  - a second, sharp specular lobe for wet patches (_Wetness, or the global wetness volume)
//  - faint world-space micro-relief so flat areas don't look like plastic
Shader "Mochi/Surface"
{
    Properties
    {
        _Color ("Base colour", Color) = (0.965, 0.955, 0.925, 1)
        _SubsurfaceColor ("Subsurface colour", Color) = (1.0, 0.86, 0.74, 1)
        _Wrap ("Wrap lighting", Range(0, 1)) = 0.55
        _SubsurfaceStrength ("Subsurface strength", Range(0, 1)) = 0.45
        _Translucency ("Back-light translucency", Range(0, 2)) = 0.6
        _TranslucencyPower ("Translucency focus", Range(1, 16)) = 4
        _Smoothness ("Satin smoothness", Range(0, 1)) = 0.42
        _Specular ("Specular level", Range(0, 0.1)) = 0.035
        _Sheen ("Grazing sheen", Range(0, 1)) = 0.35
        _SheenColor ("Sheen colour", Color) = (1, 0.98, 0.95, 1)
        _Wetness ("Wetness (global)", Range(0, 1)) = 0.15
        _WetSmoothness ("Wet smoothness", Range(0, 1)) = 0.9
        _MicroScale ("Micro-relief scale (1/m)", Float) = 220
        _MicroStrength ("Micro-relief strength", Range(0, 1)) = 0.07
        _AO ("Mottling", Range(0, 1)) = 0.0
    }
    SubShader
    {
        Tags { "RenderType" = "Opaque" "Queue" = "Geometry" }
        LOD 300
        Cull Back

        CGPROGRAM
        #pragma surface surf Mochi fullforwardshadows vertex:vert addshadow
        #pragma target 4.5
        #include "UnityPBSLighting.cginc"

        fixed4 _Color, _SubsurfaceColor, _SheenColor;
        half _Wrap, _SubsurfaceStrength, _Translucency, _TranslucencyPower;
        half _Smoothness, _Specular, _Sheen, _Wetness, _WetSmoothness;
        float _MicroScale; half _MicroStrength, _AO;

        // Optional wetness volume written by MochiWetness (station space, 0..1). Off when _MochiWetVolumeOn == 0.
        sampler3D _MochiWetVolume;
        float4x4 _MochiWorldToWetVolume;
        float _MochiWetVolumeOn;

        struct Input
        {
            float3 worldPos;
            float3 worldNormal;
        };

        struct SurfaceOutputMochi
        {
            fixed3 Albedo;
            fixed3 Specular;
            float3 Normal;
            half3 Emission;
            half Smoothness;
            half Occlusion;
            fixed Alpha;
            half Wet;
            float3 BumpN;   // world-space shading normal (the marching-cubes mesh has no tangents)
        };

        void vert(inout appdata_full v) { }

        // --- small value noise for micro-relief -------------------------------------------
        float hash13(float3 p) { p = frac(p * 0.1031); p += dot(p, p.zyx + 31.32); return frac((p.x + p.y) * p.z); }
        float vnoise(float3 p)
        {
            float3 i = floor(p), f = frac(p); f = f * f * (3 - 2 * f);
            float n000 = hash13(i), n100 = hash13(i + float3(1,0,0)), n010 = hash13(i + float3(0,1,0)), n110 = hash13(i + float3(1,1,0));
            float n001 = hash13(i + float3(0,0,1)), n101 = hash13(i + float3(1,0,1)), n011 = hash13(i + float3(0,1,1)), n111 = hash13(i + float3(1,1,1));
            return lerp(lerp(lerp(n000, n100, f.x), lerp(n010, n110, f.x), f.y), lerp(lerp(n001, n101, f.x), lerp(n011, n111, f.x), f.y), f.z);
        }

        void surf(Input IN, inout SurfaceOutputMochi o)
        {
            float3 wn = normalize(IN.worldNormal);

            // Micro-relief: perturb the world normal with the gradient of 3D value noise.
            float3 p = IN.worldPos * _MicroScale;
            const float e = 0.35;
            float n0 = vnoise(p);
            float3 g = float3(vnoise(p + float3(e,0,0)), vnoise(p + float3(0,e,0)), vnoise(p + float3(0,0,e))) - n0;
            g += 0.5 * (float3(vnoise(p*2.7 + float3(e,0,0)), vnoise(p*2.7 + float3(0,e,0)), vnoise(p*2.7 + float3(0,0,e))) - vnoise(p*2.7));

            half wet = _Wetness;
            if (_MochiWetVolumeOn > 0.5)
            {
                float3 uvw = mul(_MochiWorldToWetVolume, float4(IN.worldPos, 1)).xyz;
                if (all(uvw > 0) && all(uvw < 1)) wet = saturate(wet + tex3Dlod(_MochiWetVolume, float4(uvw, 0)).r);
            }

            o.BumpN = normalize(wn - (g - dot(g, wn) * wn) * _MicroStrength * (1 - 0.7 * wet));

            o.Albedo = _Color.rgb * lerp(1.0, 0.93, wet);           // wet dough reads a touch darker
            o.Specular = _Specular.xxx;
            o.Smoothness = _Smoothness;
            o.Occlusion = 1 - _AO * saturate(0.5 - n0) ;             // faint mottling in creases
            o.Alpha = 1;
            o.Wet = wet;
        }

        inline half4 LightingMochi(SurfaceOutputMochi s, half3 viewDir, UnityGI gi)
        {
            // Base: energy-conserving StandardSpecular (direct + indirect).
            SurfaceOutputStandardSpecular ss;
            ss.Albedo = s.Albedo; ss.Specular = s.Specular; ss.Normal = s.BumpN; ss.Emission = 0;
            ss.Smoothness = s.Smoothness; ss.Occlusion = s.Occlusion; ss.Alpha = 1;
            half4 c = LightingStandardSpecular(ss, viewDir, gi);

            half3 L = gi.light.dir, N = s.BumpN, V = viewDir;
            half3 lightCol = gi.light.color;
            half ndl = dot(N, L);

            // Wrap diffuse fills in the terminator with a warm subsurface tint.
            half wrapped = saturate((ndl + _Wrap) / (1 + _Wrap)) - saturate(ndl);
            c.rgb += s.Albedo * _SubsurfaceColor.rgb * lightCol * wrapped * _SubsurfaceStrength;

            // Back-light translucency (light passing through thin edges / the top of a cone).
            half3 Lt = normalize(L + N * 0.4);
            half trans = pow(saturate(dot(V, -Lt)), _TranslucencyPower) * _Translucency;
            c.rgb += s.Albedo * _SubsurfaceColor.rgb * lightCol * trans * 0.5;

            // Satin sheen at grazing angles.
            half nv = saturate(dot(N, V));
            half sheen = pow(1 - nv, 4) * _Sheen;
            c.rgb += _SheenColor.rgb * sheen * (lightCol * saturate(ndl + 0.3) + gi.indirect.diffuse);

            // Wet lobe: a sharp highlight layered on top.
            if (s.Wet > 0.001)
            {
                half3 H = normalize(L + V);
                half r = 1 - _WetSmoothness; half a2 = max(r * r * r * r, 1e-4);
                half nh = saturate(dot(N, H));
                half d = nh * nh * (a2 - 1) + 1;
                half D = a2 / (UNITY_PI * d * d);
                half F = 0.02 + 0.98 * pow(1 - saturate(dot(H, V)), 5);
                c.rgb += lightCol * saturate(ndl) * D * F * 0.25 * s.Wet;
                c.rgb += gi.indirect.specular * F * s.Wet * 0.6;
            }
            return c;
        }

        inline void LightingMochi_GI(SurfaceOutputMochi s, UnityGIInput data, inout UnityGI gi)
        {
            Unity_GlossyEnvironmentData g = UnityGlossyEnvironmentSetup(lerp(s.Smoothness, _WetSmoothness, s.Wet), data.worldViewDir, s.BumpN, s.Specular);
            gi = UnityGlobalIllumination(data, s.Occlusion, s.BumpN, g);
        }
        ENDCG
    }
    FallBack "Standard"
}
