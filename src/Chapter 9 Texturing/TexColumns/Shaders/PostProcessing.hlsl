Texture2D gInputImage : register(t0);
Texture2D<float> gDepthMap : register(t1);

SamplerState gSampler : register(s0);

cbuffer PostProcessSettings : register(b3)
{
    // Blur settings
    float2 gBlurDirection;
    float gMaxBlurRadius;
    float gDepthBlurThreshold;
    
    // Chromatic Aberration settings
    float2 gChromaticDirection;
    float gChromaticIntensity;
    float gChromaticDistanceScale;
    
    float gEffectIntensity; // 0 - no effects, 1 - full effects
    int gEffectType; // 0 - blur, 1 - aberration, 2 - all
};

struct VertexOut
{
    float4 PosH : SV_POSITION;
    float2 TexC : TEXCOORD;
};

VertexOut VS(uint vid : SV_VertexID)
{
    VertexOut vout;
    
    // Generating fullscreen triangle
    float2 texcoord = float2((vid << 1) & 2, vid & 2);
    vout.PosH = float4(texcoord * float2(2.0f, -2.0f) + float2(-1.0f, 1.0f), 0.0f, 1.0f);
    vout.TexC = texcoord;
    
    return vout;
}

float4 ChromaticAberration(float2 texCoord, float intensity, float2 direction)
{
    float2 texOffset = float2(1.0f / 1280.0f, 1.0f / 720.0f) * intensity;
    
    float2 offsetR = direction * texOffset * 1.5f;
    float2 offsetG = direction * texOffset * 0.5f;
    float2 offsetB = -direction * texOffset * 1.0f;
    
    float r = gInputImage.Sample(gSampler, texCoord + offsetR).r;
    float g = gInputImage.Sample(gSampler, texCoord + offsetG).g;
    float b = gInputImage.Sample(gSampler, texCoord + offsetB).b;
    
    return float4(r, g, b, 1.0f);
}

float4 DepthBlur(float2 texCoord, float blurRadius, float2 direction)
{
    if (blurRadius <= 0.0f)
        return gInputImage.Sample(gSampler, texCoord);
    
    float2 texOffset = float2(1.0f / 1280.0f, 1.0f / 720.0f) * blurRadius;
    
    // Gauss
    const float weights[5] = { 0.227027f, 0.1945946f, 0.1216216f, 0.054054f, 0.016216f };
    
    float4 color = gInputImage.Sample(gSampler, texCoord) * weights[0];
    
    for (int i = 1; i < 5; ++i)
    {
        float2 offset = direction * texOffset * i;
        color += gInputImage.Sample(gSampler, texCoord + offset) * weights[i];
        color += gInputImage.Sample(gSampler, texCoord - offset) * weights[i];
    }
    
    return color;
}

float4 PS(VertexOut pin) : SV_Target
{
    float4 ñolor = gInputImage.Sample(gSampler, pin.TexC);
    
    if (gEffectIntensity <= 0.0f)
        return ñolor;
    
    // Blur radius
    float depth = gDepthMap.Sample(gSampler, pin.TexC);
    float blurFactor = saturate((depth - gDepthBlurThreshold) / (1.0f - gDepthBlurThreshold));
    float blurRadius = lerp(0.0f, gMaxBlurRadius, blurFactor) * gEffectIntensity;
    
    switch (gEffectType)
    {
        case 0: // Blur only
            return DepthBlur(pin.TexC, blurRadius, gBlurDirection);
            
        case 1: // Chromatic Aberration only
            return lerp(ñolor,
                      ChromaticAberration(pin.TexC, gChromaticIntensity, gChromaticDirection),
                      gEffectIntensity);
            
        case 2: // Blur + Chromatic Aberration
            float4 blurred = DepthBlur(pin.TexC, blurRadius, gBlurDirection);
            float4 chromatic = ChromaticAberration(pin.TexC, gChromaticIntensity, gChromaticDirection);
            return lerp(ñolor, lerp(blurred, chromatic, 0.5f), gEffectIntensity);
            
        default:
            return ñolor;
    }
}