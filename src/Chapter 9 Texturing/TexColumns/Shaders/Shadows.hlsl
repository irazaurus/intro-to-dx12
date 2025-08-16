#include "Common.hlsl"

struct VertexIn
{
	float3 PosL    : POSITION;
	float2 TexC    : TEXCOORD;
};

struct VertexOut
{
	float4 PosW    : POSITION;
	float2 TexC    : TEXCOORD;
};

struct GSOut
{
    float4 PosCS : SV_Position;
    uint ArrInd : SV_RenderTargetArrayIndex;
    float2 TexC : TEXCOORD;
};

VertexOut VS(VertexIn vin)
{
	VertexOut vout = (VertexOut)0.0f;
	
    float4 posW = mul(float4(vin.PosL, 1.0f), gWorld);
    vout.PosW = posW;
	
    return vout;
}

[instance(6)]
[maxvertexcount(3)]
void GS(triangle VertexOut p[3], in uint id : SV_GSInstanceID, inout TriangleStream<GSOut> stream)
{
   // draw 4 cascades for directional lights
        
   // first 4 instances only
    if (id > 3)
        return;
        
    for (int i = 0; i < 3; i++)
    {
        GSOut Out;
        Out.PosCS = mul(float4(p[i].PosW.xyz, 1.f), gLightViewProj[id]);
        Out.TexC = p[i].TexC;
        Out.ArrInd = id;
        stream.Append(Out);
    }
}

// This is only used for alpha cut out geometry, so that shadows 
// show up correctly.  Geometry that does not need to sample a
// texture can use a NULL pixel shader for depth pass.
void PS(GSOut pin) 
{
	// Fetch the material data.
	MaterialData matData = gMaterialData[gMaterialIndex];
	float4 diffuseAlbedo = matData.DiffuseAlbedo;
    uint diffuseMapIndex = matData.DiffuseMapIndex;
	
	// Dynamically look up the texture in the array.
	diffuseAlbedo *= gTextureMaps[diffuseMapIndex].Sample(gsamAnisotropicWrap, pin.TexC);

#ifdef ALPHA_TEST
    // Discard pixel if texture alpha < 0.1.  We do this test as soon 
    // as possible in the shader so that we can potentially exit the
    // shader early, thereby skipping the rest of the shader code.
    clip(diffuseAlbedo.a - 0.1f);
#endif

}


