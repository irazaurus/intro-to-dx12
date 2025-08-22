#include "LightingUtil.hlsl"

Texture2DArray gShadowMap   : register(t0);
Texture2D gDiffuse          : register(t1);
Texture2D gZW               : register(t2);
Texture2D gNormal           : register(t3);
Texture2D gMaterialAlbedo   : register(t4);
Texture2D gMaterialFresnelRoughness : register(t5);

SamplerState gsamPointWrap : register(s0);
SamplerState gsamPointClamp : register(s1);
SamplerState gsamLinearWrap : register(s2);
SamplerState gsamLinearClamp : register(s3);
SamplerState gsamAnisotropicWrap : register(s4);
SamplerState gsamAnisotropicClamp : register(s5);
SamplerComparisonState gsamShadow : register(s6);

cbuffer cbPass : register(b0)
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
};

cbuffer LightConstants : register(b1)
{
    Light light;
    float3 LColor;
    int LightType; //0 - directional; 1 - point; 2 - spot
    float4x4 LWorld;
    float4x4 LViewProj[6];
    float4x4 LShadowTransform[6];
};

float3 RestoreWorldPosition(float2 UV, float depth)
{
    //magic DirectX texcoord mutations
    float4 clipPos;
    clipPos.x = UV.x * 2.0f - 1.0f;
    clipPos.y = 1.0f - UV.y * 2.0f;
    clipPos.z = depth;
    clipPos.w = 1.0f;

    //transform into world space
    float4 viewPos = mul(clipPos, gInvViewProj);
    viewPos.xyz /= viewPos.w;

    return viewPos.xyz;
}

// calculates shadow factor for shadow mapping
float CalcShadowFactor(float4 posW, int cascadeID)
{
    float4 shadowPosH = mul(posW, LShadowTransform[cascadeID]);
    
    // Complete projection by doing division by w.
    shadowPosH.xyz /= shadowPosH.w;

    // Depth in NDC space.
    float depth = shadowPosH.z;

    uint width, height, numMips, numLayers;
    gShadowMap.GetDimensions(0, width, height, numLayers, numMips);

    // Texel size.
    float dx = 1.0f / (float) width;

    float percentLit = 0.0f;
    const float2 offsets[9] =
    {
        float2(-dx, -dx), float2(0.0f, -dx), float2(dx, -dx),
        float2(-dx, 0.0f), float2(0.0f, 0.0f), float2(dx, 0.0f),
        float2(-dx, +dx), float2(0.0f, +dx), float2(dx, +dx)
    };

    [unroll]
    for (int i = 0; i < 9; ++i)
    {
        percentLit += gShadowMap.SampleCmpLevelZero(gsamShadow,
            float3(shadowPosH.xy + offsets[i], cascadeID), depth).r;
    }
    
    return percentLit / 9.0f;
}


struct VertexOut
{
    float4 PosH : SV_Position;
};

VertexOut VS(uint vertexID : SV_VertexID)
{
    //full-screen quad
    float2 verts[3] =
    {
        float2(-1, -1),
        float2(-1, 3),
        float2(3, -1)
    };
    
    VertexOut vo;
    vo.PosH = float4(verts[vertexID], 0, 1);
    return vo;
}

float4 PS(VertexOut vo) : SV_Target
{
    float2 uv = vo.PosH.xy / gRenderTargetSize;
    uint2 pixelC = vo.PosH.xy;
    float4 diffuseAlbedo = gDiffuse.Load(int3(pixelC, 0));
    float4 zw = gZW.Load(int3(pixelC, 0));
    float4 normal = gNormal.Load(int3(pixelC, 0));
    float4 matAlbedo = gMaterialAlbedo.Load(int3(pixelC, 0));
    float4 matFrR = gMaterialFresnelRoughness.Load(int3(pixelC, 0));
    
    // Vector from point being lit to eye. 
    float3 posW = RestoreWorldPosition(uv, zw.a);
    float3 toEyeW = gEyePosW - posW;
    float distToEye = length(toEyeW);
    toEyeW /= distToEye; // normalize

    const float shininess = 1.0f - matFrR.a;
    Material mat = { diffuseAlbedo, matFrR.rgb, shininess };
    float shadowFactor = 1.0f;
    
    float4 currentLight;
    
    if (LightType == 0) // direction
        {
            for (uint cascade = 0; cascade < 4; cascade++)
            {
                float factor = CalcShadowFactor(float4(posW, 1.0f), cascade);
                if (factor < 0.3f)
                {
                    shadowFactor = factor;
                    break;
                }
            }
            currentLight = float4(ComputeDirectionalLight(light, mat, normal.rgb, toEyeW) * shadowFactor, 1.0f);
    } 
    else if (LightType == 1) // point
        {
            float3 lightToPixel = posW - light.Position;
            float distToLight = length(lightToPixel);
            lightToPixel /= distToLight;
    
            // determine shadow map cube face index
            float3 absDir = abs(lightToPixel);
            uint faceIndex = 0;
            if (absDir.x >= absDir.y && absDir.x >= absDir.z)
                faceIndex = (lightToPixel.x > 0) ? 0 : 1;
            else if (absDir.y >= absDir.z)
                faceIndex = (lightToPixel.y > 0) ? 2 : 3;
            else
                faceIndex = (lightToPixel.z > 0) ? 4 : 5;
            
            shadowFactor = CalcShadowFactor(float4(posW, 1.0f), faceIndex);
            currentLight = float4(ComputePointLight(light, mat, posW, normal.rgb, toEyeW) * shadowFactor, 1.0f);
    } 
    else // spot
        {
            shadowFactor = CalcShadowFactor(float4(posW, 1.0f), 0);
            currentLight = float4(ComputeSpotLight(light, mat, posW, normal.rgb, toEyeW) * shadowFactor, 1.0f);
        }
    
    currentLight = currentLight * float4(LColor, 1.f);
    
    return currentLight;
}

float4 AmbientPS(VertexOut vo) : SV_Target
{
    uint2 pixelC = vo.PosH.xy;
    float4 diffuse = gDiffuse.Load(int3(pixelC, 0));
    
    return diffuse * gAmbientLight;
}