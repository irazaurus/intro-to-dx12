#include "Common.hlsl"

Texture2D gDiffuseMap : register(t0);
Texture2D gNormalMap : register(t1);
Texture2D gDisplacementMap : register(t2);
Texture2DArray gShadowMap : register(t3);

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
    float3 PosW : POSITION1;
    float4 PosH : SV_POSITION;
    float3 NormalW : NORMAL;
    float2 TexC : TEXCOORD;
};
 
struct PatchTess
{
    float EdgeTess[3] : SV_TessFactor;
    float InsideTess : SV_InsideTessFactor;
};

struct GBufferData
{
    float4 diffuse : SV_TARGET0;
    float4 zwzanashih_RGBA32F : SV_TARGET1;
    float4 normal : SV_TARGET2;
    float4 materialAlbedo : SV_TARGET3;
    float4 MaterialFresnelRoughness : SV_TARGET4;
};

Vertex VS(Vertex vin)
{
    return vin;
}

PatchTess ConstantHS(InputPatch<Vertex, 3> patch, uint patchID : SV_PrimitiveID)
{
    PatchTess pt;
    float tess = 1;

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
    dout.PosW = posW;
    dout.PosH = mul(posW, gViewProj);
    dout.NormalW = norm;
    dout.Tangent = tri[0].Tangent;
    dout.TexC = t;

    return dout;
}

GBufferData DeferredPS(DomainOut pin)
{
    GBufferData pout;
    
    float3 normalMap = gNormalMap.Sample(gsamAnisotropicWrap, pin.TexC).rgb;
    if (length(normalMap) != 0.f)
    {    
	// TBN
        float3 bitangent = (cross(pin.NormalW, pin.Tangent));
        bitangent = normalize(mul(bitangent, (float3x3) gWorld));
        float3 tangent = normalize(mul(pin.Tangent, (float3x3) gWorld));
        float3 normal = normalize(mul(pin.NormalW, (float3x3) gWorld));
        float3x3 TBN = float3x3(pin.Tangent, bitangent, pin.NormalW);
    
	// normal from texture
        normalMap = normalMap * 2.0f - 1.0f;
        normalMap = normalize(mul(normalMap, TBN));
    }
    else
        normalMap = normalize(pin.NormalW);
    
    float4 diffuseAlbedo = gDiffuseMap.Sample(gsamAnisotropicWrap, pin.TexC);

    pout.diffuse = diffuseAlbedo;
    pout.zwzanashih_RGBA32F = float4(0.f, 0.f, 0.f, pin.PosH.z);
    pout.normal = float4(normalMap, 0.f);
    pout.materialAlbedo = gDiffuseAlbedo;
    pout.MaterialFresnelRoughness = float4(gFresnelR0, gRoughness);

    return pout;
}