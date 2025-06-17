Texture2D gInputImage : register(t0);

SamplerState gSampler : register(s0);

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

float4 ChromaticAberration(VertexOut pin, float Intensity, float2 Direction) : SV_Target
{
    float2 texOffset = float2(1.0f / 1280.0f, 1.0f / 720.0f) * Intensity;
    
    float2 offsetR = Direction * texOffset * 1.5f;
    float2 offsetG = Direction * texOffset * 0.5f;
    float2 offsetB = -Direction * texOffset * 1.0f;
    
    float r = gInputImage.Sample(gSampler, pin.TexC + offsetR).r;
    float g = gInputImage.Sample(gSampler, pin.TexC + offsetG).g;
    float b = gInputImage.Sample(gSampler, pin.TexC + offsetB).b;
    
    return float4(r, g, b, 1.0f);
}

float4 PS(VertexOut pin) : SV_Target
{
    // Default color
    float4 color = ChromaticAberration(pin, 2.0, float2(1.0, -1.0));
    
    return color;
}