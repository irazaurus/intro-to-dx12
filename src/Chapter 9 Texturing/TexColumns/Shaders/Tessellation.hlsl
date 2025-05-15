//***************************************************************************************
// Tessellation.hlsl by Frank Luna (C) 2015 All Rights Reserved.
//***************************************************************************************


// Include structures and functions for lighting.
#include "LightingUtil.hlsl"

// Defaults for number of lights.
#ifndef NUM_DIR_LIGHTS
    #define NUM_DIR_LIGHTS 3
#endif

#ifndef NUM_POINT_LIGHTS
    #define NUM_POINT_LIGHTS 0
#endif

#ifndef NUM_SPOT_LIGHTS
    #define NUM_SPOT_LIGHTS 0
#endif

Texture2D gDiffuseMap : register(t0);
Texture2D gNormalMap : register(t1);
Texture2D gDisplacementMap : register(t2);

SamplerState gsamPointWrap : register(s0);
SamplerState gsamPointClamp : register(s1);
SamplerState gsamLinearWrap : register(s2);
SamplerState gsamLinearClamp : register(s3);
SamplerState gsamAnisotropicWrap : register(s4);
SamplerState gsamAnisotropicClamp : register(s5);

// Constant data that varies per frame.
cbuffer cbPerObject : register(b0)
{
    float4x4 gWorld;
    float4x4 gTexTransform;
};

// Constant data that varies per material.
cbuffer cbPass : register(b1)
{
    float4x4 gView;
    float4x4 gInvView;
    float4x4 gProj;
    float4x4 gInvProj;
    float4x4 gViewProj;
    float4x4 gInvViewProj;
    float3 gEyePosW;
    float cbPerObjectPad1;
    float2 gRenderTargetSize;
    float2 gInvRenderTargetSize;
    float gNearZ;
    float gFarZ;
    float gTotalTime;
    float gDeltaTime;
    float4 gAmbientLight;

	// Indices [0, NUM_DIR_LIGHTS) are directional lights;
	// indices [NUM_DIR_LIGHTS, NUM_DIR_LIGHTS+NUM_POINT_LIGHTS) are point lights;
	// indices [NUM_DIR_LIGHTS+NUM_POINT_LIGHTS, NUM_DIR_LIGHTS+NUM_POINT_LIGHT+NUM_SPOT_LIGHTS)
	// are spot lights for a maximum of MaxLights per object.
    Light gLights[MaxLights];
};

cbuffer cbMaterial : register(b2)
{
    float4 gDiffuseAlbedo;
    float3 gFresnelR0;
    float gRoughness;
    float4x4 gMatTransform;
};

struct Vertex
{
    float3 Tangent : TANGENT;
    float3 PosL : POSITION;
    float3 NormalL : NORMAL;
    float2 TexC : TEXCOORD;
};

struct HullOut
{
    float3 Tangent : TANGENT;
    float3 PosL : POSITION;
    float3 NormalW : NORMAL;
    float2 TexC : TEXCOORD;
};

struct DomainOut
{
    float3 Tangent : TANGENT;
    float3 PosL : POSITION;
    float4 PosH : SV_POSITION;
    float3 NormalW : NORMAL;
    float2 TexC : TEXCOORD;
};
 
struct PatchTess
{
    float EdgeTess[3] : SV_TessFactor;
    float InsideTess : SV_InsideTessFactor;
};

Vertex VS(Vertex vin)
{
    return vin;
}

PatchTess ConstantHS(InputPatch<Vertex, 3> patch, uint patchID : SV_PrimitiveID)
{
    PatchTess pt;
	
    float3 centerL = 0.25f * (patch[0].PosL + patch[1].PosL + patch[2].PosL);
    float3 centerW = mul(float4(centerL, 1.0f), gWorld).xyz;
	
    float d = distance(centerW, gEyePosW);

	// Tessellate the patch based on distance from the eye such that
	// the tessellation is 0 if d >= d1 and 64 if d <= d0.  The interval
	// [d0, d1] defines the range we tessellate in.
	
    const float d0 = 1.0f;
    const float d1 = 100.0f;
    float tess = 64.0f * saturate((d1 - d) / (d1 - d0));

	// Uniformly tessellate the patch.

    pt.EdgeTess[0] = tess;
    pt.EdgeTess[1] = tess;
    pt.EdgeTess[2] = tess;
	
    pt.InsideTess = tess;
	
    return pt;
}

[domain("tri")]
[partitioning("integer")]
[outputtopology("triangle_cw")]
[outputcontrolpoints(3)]
[patchconstantfunc("ConstantHS")]
[maxtessfactor(64.0f)]
HullOut HS(InputPatch<Vertex, 3> p,
           uint i : SV_OutputControlPointID,
           uint patchId : SV_PrimitiveID)
{
    HullOut hout;
	
    hout.PosL = p[i].PosL;
    hout.NormalW = p[i].NormalL;
    hout.TexC = p[i].TexC;
    hout.Tangent = p[i].Tangent;
	
    return hout;
}

[domain("tri")]
DomainOut DS(PatchTess patchTess,
             float3 bary : SV_DomainLocation,
             const OutputPatch<HullOut, 3> tri)
{
    DomainOut dout;
    
    float3 p = bary.x * tri[0].PosL +
               bary.y * tri[1].PosL +
               bary.z * tri[2].PosL;
    
    if (bary.x + bary.y + bary.z != 1.0f)
    {
        p = (0.0f, 20.0f, 0.0f);

    }
    
    float2 t = bary.x * tri[0].TexC +
               bary.y * tri[1].TexC +
               bary.z * tri[2].TexC;
    t = float2(abs(t.x) - (uint) t.x, abs(t.y) - (uint) t.y);
    
    float3 norm = bary.x * tri[0].NormalW +
               bary.y * tri[1].NormalW +
               bary.z * tri[2].NormalW;

    // Displacement mapping
    uint width, height;
    gDisplacementMap.GetDimensions(width, height);
    float disp = gDisplacementMap.Load(int3(t.x * width, t.y * height, 0)).r;
    if (abs(disp) < 1e-5f)
        disp = 1.0f;
    p.y += disp * 1.0f;

    dout.PosL = p;
    float4 posW = mul(float4(p, 1.0f), gWorld);
    dout.PosH = mul(posW, gViewProj);
    dout.NormalW = norm;
    dout.Tangent = tri[0].Tangent;
    dout.TexC = t;

    return dout;
}


float4 PS(DomainOut pin) : SV_Target
{
    float4 diffuseAlbedo = gDiffuseMap.Sample(gsamAnisotropicWrap, pin.TexC) * gDiffuseAlbedo;
	
#ifdef ALPHA_TEST
	// Discard pixel if texture alpha < 0.1.  We do this test as soon 
	// as possible in the shader so that we can potentially exit the
	// shader early, thereby skipping the rest of the shader code.
	clip(diffuseAlbedo.a - 0.1f);
#endif
    
	// TBN
    float3 bitangent = (cross(pin.NormalW, pin.Tangent));
    bitangent = normalize(mul(bitangent, (float3x3) gWorld));
    float3 tangent = normalize(mul(pin.Tangent, (float3x3) gWorld));
    float3 normal = normalize(mul(pin.NormalW, (float3x3) gWorld));
    float3x3 TBN = float3x3(pin.Tangent, bitangent, pin.NormalW);
	
	// normal from texture
    float3 normalMap = gNormalMap.Sample(gsamAnisotropicWrap, pin.TexC).rgb;
    normalMap = normalMap * 2.0f - 1.0f;
    normalMap = normalize(mul(normalMap, TBN));
	
    // Vector from point being lit to eye. 
    float3 toEyeW = gEyePosW - pin.PosL;
    float distToEye = length(toEyeW);
    toEyeW /= distToEye; // normalize

    // Light terms.
    float4 ambient = gAmbientLight * diffuseAlbedo;

    const float shininess = 1.0f - gRoughness;
    Material mat = { diffuseAlbedo, gFresnelR0, shininess };
    float3 shadowFactor = 1.0f;
    float4 directLight = ComputeLighting(gLights, mat, pin.PosL,
        normalMap, toEyeW, shadowFactor);
    
    // this is shit
    //if (!any(directLight))
    //    directLight = (0.1f, 0.1f, 0.1f, 0.1f);

    float4 litColor = ambient + directLight;

#ifdef FOG
	float fogAmount = saturate((distToEye - gFogStart) / gFogRange);
	litColor = lerp(litColor, gFogColor, fogAmount);
#endif

    // Common convention to take alpha from diffuse albedo.
    litColor.a = diffuseAlbedo.a;

    return litColor;
}
