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

float4 PS(VertexOut pin) : SV_Target
{
    // Default color
    float4 color = gInputImage.Sample(gSampler, pin.TexC);
    
    color.rgb = 1.0 - color.rgb;
    
    // Basic blur effect
    /*
    float2 texelSize = 1.0 / float2(1280, 720);
    float4 blurColor = float4(0, 0, 0, 0);
    for (int x = -2; x <= 2; x++)
    {
        for (int y = -2; y <= 2; y++)
        {
            blurColor += gInputImage.Sample(gSampler, pin.TexC + float2(x, y) * texelSize);
        }
    }
    color = blurColor / 25.0;
    */
    
    return color;
}