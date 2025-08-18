//***************************************************************************************
// TexColumnsApp.cpp by Frank Luna (C) 2015 All Rights Reserved.
//***************************************************************************************

#include "../../Common/d3dApp.h"
#include "../../Common/MathHelper.h"
#include "../../Common/UploadBuffer.h"
#include "../../Common/GeometryGenerator.h"
#include "../../Common/Camera.h"
#include "FrameResource.h"
#include "ShadowMap.h"

using Microsoft::WRL::ComPtr;
using namespace DirectX;
using namespace DirectX::PackedVector;

#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "D3D12.lib")

// #define DEBUG_VIEW
// #define DEBUG
// #define POSTEFFECTS

const int gNumFrameResources = 3;

// Lightweight structure stores parameters to draw a shape.  This will
// vary from app-to-app.
struct RenderItem
{
	RenderItem() = default;
	RenderItem(const RenderItem& rhs) = delete;

	// World matrix of the shape that describes the object's local space
	// relative to the world space, which defines the position, orientation,
	// and scale of the object in the world.
	XMFLOAT4X4 World = MathHelper::Identity4x4();

	XMFLOAT4X4 TexTransform = MathHelper::Identity4x4();

	// Dirty flag indicating the object data has changed and we need to update the constant buffer.
	// Because we have an object cbuffer for each FrameResource, we have to apply the
	// update to each FrameResource.  Thus, when we modify obect data we should set 
	// NumFramesDirty = gNumFrameResources so that each frame resource gets the update.
	int NumFramesDirty = gNumFrameResources;

	// Index into GPU constant buffer corresponding to the ObjectCB for this render item.
	UINT ObjCBIndex = -1;

	Material* Mat = nullptr;
	MeshGeometry* Geo = nullptr;

	// Primitive topology.
	D3D12_PRIMITIVE_TOPOLOGY PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

	// DrawIndexedInstanced parameters.
	UINT IndexCount = 0;
	UINT StartIndexLocation = 0;
	int BaseVertexLocation = 0;
};

enum class RenderLayer : int
{
	Opaque = 0,
	Debug,
	Sky,
	Count
};

class TexColumnsApp : public D3DApp
{
public:
	TexColumnsApp(HINSTANCE hInstance);
	TexColumnsApp(const TexColumnsApp& rhs) = delete;
	TexColumnsApp& operator=(const TexColumnsApp& rhs) = delete;
	~TexColumnsApp();

	virtual bool Initialize()override;

private:
	virtual void CreateRtvAndDsvDescriptorHeaps()override;
	virtual void OnResize()override;
	virtual void Update(const GameTimer& gt)override;
	virtual void Draw(const GameTimer& gt)override;

	virtual void OnMouseDown(WPARAM btnState, int x, int y)override;
	virtual void OnMouseUp(WPARAM btnState, int x, int y)override;
	virtual void OnMouseMove(WPARAM btnState, int x, int y)override;

	void OnKeyboardInput(const GameTimer& gt);
	void AnimateMaterials(const GameTimer& gt);
	void AnimateLights(const GameTimer& gt);
	void UpdateObjectCBs(const GameTimer& gt);
	void UpdateMaterialCBs(const GameTimer& gt);
	void UpdateMainPassCB(const GameTimer& gt);
	void UpdatePostProcessCB(const GameTimer& gt);
	void UpdateShadowTransform(const GameTimer& gt);

	void LoadTexture(std::string name, std::wstring filename);
	void LoadTextures();
	void BuildRootSignature();
	void BuildDescriptorHeaps();
	void BuildShadersAndInputLayout();
	void BuildShapeGeometry();
	void BuildPSOs();
	void BuildFrameResources();
	void BuildMaterials();
	void BuildRenderItem(std::string name, std::string material, XMMATRIX translate, int layer = (int)RenderLayer::Opaque, float scale = 1.f, float scaleTex = 1.f);
	void BuildRenderItems();
	void DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems);
	void DrawSceneToShadowMap();
	void BuildPostProcessResources();
	void BuildPostProcessPSO();
	void BuildPostProcessRootSignature();

	std::array<const CD3DX12_STATIC_SAMPLER_DESC, 7> GetStaticSamplers();

private:

	std::vector<std::unique_ptr<FrameResource>> mFrameResources;
	FrameResource* mCurrFrameResource = nullptr;
	int mCurrFrameResourceIndex = 0;

	UINT mCbvSrvDescriptorSize = 0;

	std::unordered_map<std::string, ComPtr<ID3D12RootSignature>> mRootSignature;

	ComPtr<ID3D12DescriptorHeap> mSrvDescriptorHeap = nullptr;

	std::unordered_map<std::string, std::unique_ptr<MeshGeometry>> mGeometries;
	std::unordered_map<std::string, std::unique_ptr<Material>> mMaterials;
	std::vector<std::unique_ptr<Texture>> mTextures;
	std::unordered_map<std::string, ComPtr<ID3DBlob>> mShaders;
	std::unordered_map<std::string, ComPtr<ID3D12PipelineState>> mPSOs;

	std::vector<D3D12_INPUT_ELEMENT_DESC> mInputLayout;

	// List of all the render items.
	std::vector<std::unique_ptr<RenderItem>> mAllRitems;
	int ObjCBIndex = 0;

	// Render items divided by PSO.
	std::vector<RenderItem*> mRitemLayer[(int)RenderLayer::Count];

	PassConstants mMainPassCB;

	Camera mCamera;
	POINT mLastMousePos;

	// For post-processing
	ComPtr<ID3D12Resource> mPostProcessRenderTarget;
	ComPtr<ID3D12Resource> mPostProcessRenderTargetUpload;
	ComPtr<ID3D12DescriptorHeap> mPostProcessRTVHeap;
	ComPtr<ID3D12DescriptorHeap> mPostProcessSRVHeap;
	ComPtr<ID3D12PipelineState> mPostProcessPSO;
	ComPtr<ID3D12RootSignature> mPostProcessRootSignature;
	std::unordered_map<std::string, ComPtr<ID3DBlob>> mPostProcessShaders;

	// For lighting
	std::unique_ptr<ShadowMap> mShadowMap;
	DirectX::BoundingSphere mSceneBounds;

	float mLightNearZ = 0.0f;
	float mLightFarZ = 0.0f;
	XMFLOAT3 mLightPosW;

	float mLightRotationAngle = 0.0f;
	XMFLOAT3 mBaseLightDirections[3] = {
		XMFLOAT3(0.57735f, -0.57735f, 0.57735f),
		XMFLOAT3(-0.57735f, -0.57735f, 0.57735f),
		XMFLOAT3(0.0f, -0.707f, -0.707f)
	};
	XMFLOAT3 mRotatedLightDirections[3];

	UINT mSkyTexHeapIndex = 0;
	UINT mShadowMapHeapIndex = 0;
	UINT mNullCubeSrvIndex = 0;
	UINT mNullTexSrvIndex = 0;
	CD3DX12_GPU_DESCRIPTOR_HANDLE mNullSrv;
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
	PSTR lpCmdLine, int nCmdShow)
{
	// Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

	try
	{
		TexColumnsApp theApp(hInstance);
		if (!theApp.Initialize())
			return 0;

		return theApp.Run();
	}
	catch (DxException& e)
	{
		MessageBox(nullptr, e.ToString().c_str(), L"HR Failed", MB_OK);
		return 0;
	}
}

TexColumnsApp::TexColumnsApp(HINSTANCE hInstance)
	: D3DApp(hInstance)
{
}

TexColumnsApp::~TexColumnsApp()
{
	if (md3dDevice != nullptr)
		FlushCommandQueue();
}

bool TexColumnsApp::Initialize()
{
	if (!D3DApp::Initialize())
		return false;

	// Reset the command list to prep for initialization commands.
	ThrowIfFailed(mCommandList->Reset(mDirectCmdListAlloc.Get(), nullptr));

	// Get the increment size of a descriptor in this heap type.  This is hardware specific, 
	// so we have to query this information.
	mCbvSrvDescriptorSize = md3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	mSceneBounds.Center = XMFLOAT3(0.0f, 0.0f, 0.0f);
	mSceneBounds.Radius = 100.0f;

	mCamera.SetPosition(-30.0f, 70.0f, -20.0f);
	mCamera.Pitch(-5.3f);
	mCamera.RotateY(0.7f);
	mShadowMap = std::make_unique<ShadowMap>(
		md3dDevice.Get(), 2048, 2048);

	LoadTextures();
	BuildRootSignature();
	BuildDescriptorHeaps();
	BuildShadersAndInputLayout();
	BuildShapeGeometry();
	BuildMaterials();
	BuildRenderItems();

#if defined(POSTEFFECTS)
	BuildPostProcessResources();
	BuildPostProcessRootSignature();
	BuildPostProcessPSO();
#endif

	BuildFrameResources();
	BuildPSOs();

	// Execute the initialization commands.
	ThrowIfFailed(mCommandList->Close());
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

	// Wait until initialization is complete.
	FlushCommandQueue();

	return true;
}

void TexColumnsApp::CreateRtvAndDsvDescriptorHeaps()
{
	// Add +6 RTV for cube render target.
	D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc;
	rtvHeapDesc.NumDescriptors = SwapChainBufferCount;
	rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
	rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
	rtvHeapDesc.NodeMask = 0;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(
		&rtvHeapDesc, IID_PPV_ARGS(mRtvHeap.GetAddressOf())));

	// Add +1 DSV for shadow map.
	D3D12_DESCRIPTOR_HEAP_DESC dsvHeapDesc;
	dsvHeapDesc.NumDescriptors = 2;
	dsvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
	dsvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
	dsvHeapDesc.NodeMask = 0;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(
		&dsvHeapDesc, IID_PPV_ARGS(mDsvHeap.GetAddressOf())));
}

void TexColumnsApp::OnResize()
{
	D3DApp::OnResize();

	// The window resized, so update the aspect ratio and recompute the projection matrix.
	mCamera.SetLens(0.25f * MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);
	mGBuffer->Resize(mClientWidth, mClientHeight, md3dDevice.Get());

	// copy gbuffer resources into the srv heap
	if (mSrvDescriptorHeap != nullptr)
	{
		auto srvGBuffer = CD3DX12_CPU_DESCRIPTOR_HANDLE(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());
		srvGBuffer.Offset(mGBuffer->Channel0SRVHeapIndex, mCbvSrvUavDescriptorSize);
		md3dDevice->CopyDescriptorsSimple(mGBuffer->NumBuffers, srvGBuffer,
			mGBuffer->m_SRVDescriptorHeap->GetCPUDescriptorHandleForHeapStart(),
			D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
	}
}

void TexColumnsApp::Update(const GameTimer& gt)
{
	OnKeyboardInput(gt);

	// Cycle through the circular frame resource array.
	mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % gNumFrameResources;
	mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

	// Has the GPU finished processing the commands of the current frame resource?
	// If not, wait until the GPU has completed commands up to this fence point.
	if (mCurrFrameResource->Fence != 0 && mFence->GetCompletedValue() < mCurrFrameResource->Fence)
	{
		HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);
		ThrowIfFailed(mFence->SetEventOnCompletion(mCurrFrameResource->Fence, eventHandle));
		WaitForSingleObject(eventHandle, INFINITE);
		CloseHandle(eventHandle);
	}

	AnimateLights(gt);
	AnimateMaterials(gt);
	UpdateShadowTransform(gt);
	UpdateObjectCBs(gt);
	UpdateMaterialCBs(gt);
	UpdateMainPassCB(gt);
	UpdatePostProcessCB(gt);
}

void TexColumnsApp::Draw(const GameTimer& gt)
{
	auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;
	auto passCB = mCurrFrameResource->PassCB->Resource();

	// Reuse the memory associated with command recording.
	// We can only reset when the associated command lists have finished execution on the GPU.
	ThrowIfFailed(cmdListAlloc->Reset());

	// A command list can be reset after it has been added to the command queue via ExecuteCommandList.
	// Reusing the command list reuses memory.
	ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), nullptr));
	mCommandList->SetGraphicsRootSignature(mRootSignature["default"].Get());
	
	// draw geometry for deferred render
	mCommandList->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_3_CONTROL_POINT_PATCHLIST);
	ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };

	mCommandList->RSSetViewports(1, &mScreenViewport);
	mCommandList->RSSetScissorRects(1, &mScissorRect);
	mCommandList->SetPipelineState(mPSOs["deferredGeometry"].Get());

	D3D12_CPU_DESCRIPTOR_HANDLE rtvs[5] = {
		 mGBuffer->DiffuseRTV,
		 mGBuffer->ZWzanashihRTV,
		 mGBuffer->NormalRTV,
		 mGBuffer->MaterialAlbedoRTV,
		 mGBuffer->MaterialFresnelRoughnessRTV
	};

	mCommandList->OMSetRenderTargets(5, rtvs, false, &DepthStencilView());

	mGBuffer->TransitToOpaqueRenderingState(mCommandList);
	mGBuffer->ClearRTVs(mCommandList);

	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);
	mCommandList->SetGraphicsRootConstantBufferView(2, passCB->GetGPUVirtualAddress());

	DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque]);

	mGBuffer->TransitToLightsRenderingState(mCommandList);
	mGBuffer->TransitToTonemappingState(mCommandList);
	mGBuffer->TransitFromShaderResourceToCommon(mCommandList);

	// draw shadow map
	mCommandList->RSSetViewports(1, &mShadowMap->Viewport());
	mCommandList->RSSetScissorRects(1, &mShadowMap->ScissorRect());
	mCommandList->SetPipelineState(mPSOs["shadow_opaque"].Get());

	// Transition render target to dsv
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
		mShadowMap->Resource(),
		D3D12_RESOURCE_STATE_GENERIC_READ,
		D3D12_RESOURCE_STATE_DEPTH_WRITE));
	// Clear depth stencil
	mCommandList->ClearDepthStencilView(mShadowMap->Dsv(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

	// Specify the buffers we are going to render to.
	mCommandList->OMSetRenderTargets(0, nullptr, true, &mShadowMap->Dsv());

	mCommandList->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);

	mCommandList->SetGraphicsRootConstantBufferView(2, passCB->GetGPUVirtualAddress());

	DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque]);

	// Transition dsv to rtv
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
		mShadowMap->Resource(),
		D3D12_RESOURCE_STATE_DEPTH_WRITE,
		D3D12_RESOURCE_STATE_GENERIC_READ));

	// draw opaque items
	mCommandList->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_3_CONTROL_POINT_PATCHLIST);

	mCommandList->RSSetViewports(1, &mScreenViewport);
	mCommandList->RSSetScissorRects(1, &mScissorRect);
	mCommandList->SetPipelineState(mPSOs["opaque"].Get());

#if defined(POSTEFFECTS)
	// Transition render target to render target state
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
		mPostProcessRenderTarget.Get(),
		D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
		D3D12_RESOURCE_STATE_RENDER_TARGET));
	// Clear render target
	CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(mPostProcessRTVHeap->GetCPUDescriptorHandleForHeapStart());
	mCommandList->ClearRenderTargetView(rtvHandle, Colors::LightSteelBlue, 0, nullptr);
# else
	// Indicate a state transition on the resource usage.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
		D3D12_RESOURCE_STATE_PRESENT, 
		D3D12_RESOURCE_STATE_RENDER_TARGET));
	// Clear back buffer
	mCommandList->ClearRenderTargetView(CurrentBackBufferView(), Colors::LightSteelBlue, 0, nullptr);
#endif

	// Clear depth buffer.
	mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

#if defined(POSTEFFECTS)
	// Set render target
	mCommandList->OMSetRenderTargets(1, &rtvHandle, true, &DepthStencilView());
# else
	// Specify the buffers we are going to render to.
	mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());
#endif

	mCommandList->SetGraphicsRootDescriptorTable(4, mShadowMap->Srv()); // Shadow Map
	mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);
	mCommandList->SetGraphicsRootConstantBufferView(2, passCB->GetGPUVirtualAddress());

	DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque]);

#if defined(POSTEFFECTS)
	// Transition render target to shader resource for post-processing
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
		mPostProcessRenderTarget.Get(),
		D3D12_RESOURCE_STATE_RENDER_TARGET,
		D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE));

	// Transition back buffer to render target for post-processing
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(
		CurrentBackBuffer(),
		D3D12_RESOURCE_STATE_PRESENT,
		D3D12_RESOURCE_STATE_RENDER_TARGET));

	// Post-processing pass
	mCommandList->SetPipelineState(mPostProcessPSO.Get());
	mCommandList->SetGraphicsRootSignature(mPostProcessRootSignature.Get());

	ID3D12DescriptorHeap* postProcessHeaps[] = { mPostProcessSRVHeap.Get() };
	mCommandList->SetDescriptorHeaps(_countof(postProcessHeaps), postProcessHeaps);

	CD3DX12_GPU_DESCRIPTOR_HANDLE srvHandle(
		mPostProcessSRVHeap->GetGPUDescriptorHandleForHeapStart());
	mCommandList->SetGraphicsRootDescriptorTable(0, srvHandle); // Texture
	srvHandle.Offset(1, mCbvSrvDescriptorSize);
	mCommandList->SetGraphicsRootDescriptorTable(1, srvHandle); // Depth buffer
	auto postProcessCB = mCurrFrameResource->PostProcessCB->Resource();
    mCommandList->SetGraphicsRootConstantBufferView(2, 
        postProcessCB->GetGPUVirtualAddress()); // PostProcess Settings

	// Set back buffer as render target
	CD3DX12_CPU_DESCRIPTOR_HANDLE backBufferRtvHandle(
		mRtvHeap->GetCPUDescriptorHandleForHeapStart(),
		mCurrBackBuffer,
		mCbvSrvDescriptorSize);
	mCommandList->OMSetRenderTargets(1, &backBufferRtvHandle, true, nullptr);

	// Draw fullscreen quad
	mCommandList->IASetVertexBuffers(0, 0, nullptr);
	mCommandList->IASetIndexBuffer(nullptr);
	mCommandList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	mCommandList->DrawInstanced(3, 1, 0, 0);
#endif

	// Indicate a state transition on the resource usage.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
		D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

	// Done recording commands.
	ThrowIfFailed(mCommandList->Close());

	// Add the command list to the queue for execution.
	ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
	mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

	// Swap the back and front buffers
	ThrowIfFailed(mSwapChain->Present(0, 0));
	mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;

	// Advance the fence value to mark commands up to this fence point.
	mCurrFrameResource->Fence = ++mCurrentFence;

	// Add an instruction to the command queue to set a new fence point. 
	// Because we are on the GPU timeline, the new fence point won't be 
	// set until the GPU finishes processing all the commands prior to this Signal().
	mCommandQueue->Signal(mFence.Get(), mCurrentFence);
}

void TexColumnsApp::OnMouseDown(WPARAM btnState, int x, int y)
{
	mLastMousePos.x = x;
	mLastMousePos.y = y;

	SetCapture(mhMainWnd);
}

void TexColumnsApp::OnMouseUp(WPARAM btnState, int x, int y)
{
	ReleaseCapture();
}

void TexColumnsApp::OnMouseMove(WPARAM btnState, int x, int y)
{
	if ((btnState & MK_LBUTTON) != 0)
	{
		// Make each pixel correspond to a quarter of a degree.
		float dx = XMConvertToRadians(0.25f * static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(0.25f * static_cast<float>(y - mLastMousePos.y));

		mCamera.Pitch(dy);
		mCamera.RotateY(dx);
	}

	mLastMousePos.x = x;
	mLastMousePos.y = y;
}

void TexColumnsApp::OnKeyboardInput(const GameTimer& gt)
{
	const float dt = gt.DeltaTime();

	if (GetAsyncKeyState('W') & 0x8000)
		mCamera.Walk(20.0f * dt);

	if (GetAsyncKeyState('S') & 0x8000)
		mCamera.Walk(-20.0f * dt);

	if (GetAsyncKeyState('A') & 0x8000)
		mCamera.Strafe(-20.0f * dt);

	if (GetAsyncKeyState('D') & 0x8000)
		mCamera.Strafe(20.0f * dt);

	mCamera.UpdateViewMatrix();
}

void TexColumnsApp::AnimateMaterials(const GameTimer& gt)
{

}

void TexColumnsApp::AnimateLights(const GameTimer& gt)
{
	mLightRotationAngle += 0.1f * gt.DeltaTime();

	XMMATRIX R = XMMatrixRotationY(mLightRotationAngle);
	for (int i = 0; i < 3; ++i)
	{
		XMVECTOR lightDir = XMLoadFloat3(&mBaseLightDirections[i]);
		lightDir = XMVector3TransformNormal(lightDir, R);
		XMStoreFloat3(&mRotatedLightDirections[i], lightDir);
	}
}

void TexColumnsApp::UpdateObjectCBs(const GameTimer& gt)
{
	auto currObjectCB = mCurrFrameResource->ObjectCB.get();
	for (auto& e : mAllRitems)
	{
		// Only update the cbuffer data if the constants have changed.  
		// This needs to be tracked per frame resource.
		if (e->NumFramesDirty > 0)
		{
			XMMATRIX world = XMLoadFloat4x4(&e->World);
			XMMATRIX texTransform = XMLoadFloat4x4(&e->TexTransform);

			ObjectConstants objConstants;
			XMStoreFloat4x4(&objConstants.World, XMMatrixTranspose(world));
			XMStoreFloat4x4(&objConstants.TexTransform, XMMatrixTranspose(texTransform));
			objConstants.MaterialIndex = e->Mat->MatCBIndex;

			currObjectCB->CopyData(e->ObjCBIndex, objConstants);

			// Next FrameResource need to be updated too.
			e->NumFramesDirty--;
		}
	}
}

void TexColumnsApp::UpdateMaterialCBs(const GameTimer& gt)
{
	auto currMaterialCB = mCurrFrameResource->MaterialCB.get();
	for (auto& e : mMaterials)
	{
		// Only update the cbuffer data if the constants have changed.  If the cbuffer
		// data changes, it needs to be updated for each FrameResource.
		Material* mat = e.second.get();
		if (mat->NumFramesDirty > 0)
		{
			XMMATRIX matTransform = XMLoadFloat4x4(&mat->MatTransform);

			MaterialConstants matConstants;
			matConstants.DiffuseAlbedo = mat->DiffuseAlbedo;
			matConstants.FresnelR0 = mat->FresnelR0;
			matConstants.Roughness = mat->Roughness;
			XMStoreFloat4x4(&matConstants.MatTransform, XMMatrixTranspose(matTransform));
			matConstants.DiffuseMapIndex = mat->DiffuseSrvHeapIndex;
			matConstants.NormalMapIndex = mat->NormalSrvHeapIndex;

			currMaterialCB->CopyData(mat->MatCBIndex, matConstants);

			// Next FrameResource need to be updated too.
			mat->NumFramesDirty--;
		}
	}
}

void TexColumnsApp::UpdatePostProcessCB(const GameTimer& gt)
{
	auto currPostProcessCB = mCurrFrameResource->PostProcessCB.get();
	PostProcessSettings postProcessSettings;

	postProcessSettings.FocusDistance = 0.95f;
	postProcessSettings.FocusRange = 0.1f;
	postProcessSettings.NearBlurStrength = 5.0f;
	postProcessSettings.FarBlurStrength = 5.0f;
	postProcessSettings.ChromaticDirection = XMFLOAT2(-1.0f, -1.0f);
	postProcessSettings.ChromaticIntensity = 2.0f;
	postProcessSettings.ChromaticDistanceScale = 1.5f;
	postProcessSettings.EffectIntensity = 1.0f;
	postProcessSettings.EffectType = 0;

	currPostProcessCB->CopyData(0, postProcessSettings);
}

void TexColumnsApp::UpdateShadowTransform(const GameTimer& gt)
{
		float SphereRadiuses[4] = { 50, 70, 100, 150 };

		//for each cascade
		for (int i = 0; i < 4; i++)
		{
			// Only the first "main" light casts a shadow. Why? Idk, you tell me.
			XMVECTOR lightDir = XMLoadFloat3(&mRotatedLightDirections[0]);
			XMVECTOR lightPos = mCamera.GetPosition() - 2.0f * SphereRadiuses[i] * lightDir;
			XMVECTOR targetPos = mCamera.GetPosition();
			XMVECTOR lightUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
			XMMATRIX lightView = XMMatrixLookAtLH(lightPos, targetPos, lightUp);

			XMStoreFloat3(&mLightPosW, lightPos);

			// Transform bounding sphere to light space.
			XMFLOAT3 sphereCenterLS;
			XMStoreFloat3(&sphereCenterLS, XMVector3TransformCoord(targetPos, lightView));

			// Ortho frustum in light space encloses scene.
			float l = sphereCenterLS.x - SphereRadiuses[i];
			float b = sphereCenterLS.y - SphereRadiuses[i];
			float n = sphereCenterLS.z - SphereRadiuses[i];
			float r = sphereCenterLS.x + SphereRadiuses[i];
			float t = sphereCenterLS.y + SphereRadiuses[i];
			float f = sphereCenterLS.z + SphereRadiuses[i];

			mLightNearZ = n;
			mLightFarZ = f;
			XMMATRIX lightProj = XMMatrixOrthographicOffCenterLH(l, r, b, t, n, f);

			// Transform NDC space [-1,+1]^2 to texture space [0,1]^2
			XMMATRIX T(
				0.5f, 0.0f, 0.0f, 0.0f,
				0.0f, -0.5f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.5f, 0.5f, 0.0f, 1.0f);

			XMMATRIX S = lightView * lightProj;
			XMMATRIX S1 = S * T;
			XMStoreFloat4x4(&mMainPassCB.LightViewProj[i], XMMatrixTranspose(S));
			XMStoreFloat4x4(&mMainPassCB.ShadowTransform[i], XMMatrixTranspose(S1));
		}
}

void TexColumnsApp::UpdateMainPassCB(const GameTimer& gt)
{
	XMMATRIX view = mCamera.GetView();
	XMMATRIX proj = mCamera.GetProj();

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

	XMStoreFloat4x4(&mMainPassCB.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mMainPassCB.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mMainPassCB.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mMainPassCB.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mMainPassCB.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mMainPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
	mMainPassCB.EyePosW = mCamera.GetPosition3f();
	mMainPassCB.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCB.NearZ = 1.0f;
	mMainPassCB.FarZ = 1000.0f;
	mMainPassCB.TotalTime = gt.TotalTime();
	mMainPassCB.DeltaTime = gt.DeltaTime();
	mMainPassCB.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
	mMainPassCB.Lights[0].Direction = mRotatedLightDirections[0];
	mMainPassCB.Lights[0].Strength = { 0.9f, 0.8f, 0.7f };
	mMainPassCB.Lights[1].Direction = mRotatedLightDirections[1];
	mMainPassCB.Lights[1].Strength = { 0.4f, 0.4f, 0.4f };
	mMainPassCB.Lights[2].Direction = mRotatedLightDirections[2];
	mMainPassCB.Lights[2].Strength = { 0.2f, 0.2f, 0.2f };

	auto currPassCB = mCurrFrameResource->PassCB.get();
	currPassCB->CopyData(0, mMainPassCB);
}

void TexColumnsApp::LoadTexture(std::string name, std::wstring filename)
{
	auto tex = std::make_unique<Texture>();
	tex->Name = name;
	tex->Filename = filename;
	ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
		mCommandList.Get(), tex->Filename.c_str(),
		tex->Resource, tex->UploadHeap));
	mTextures.push_back(std::move(tex));
}

void TexColumnsApp::LoadTextures()
{
	// Bricks
	LoadTexture("bricksTex", L"../../Textures/tile.dds");
	LoadTexture("bricksNormTex", L"../../Textures/tile_nmap.dds");
	LoadTexture("bricksDispTex", L"../../Textures/checkboard.dds");

	// Baronyx
	LoadTexture("baryonyx_diffuse", L"../../Textures/baryonyx_diffuse.dds");
	LoadTexture("baryonyx_norm", L"../../Textures/baryonyx_diffuse.dds");
	LoadTexture("baryonyx_disp", L"../../Textures/baryonyx_diffuse.dds");

	// Defaults
	LoadTexture("defaultNorm", L"../../Textures/default_nmap.dds");
	LoadTexture("defaultDiffuse", L"../../Textures/white1x1.dds");

	// Last texture - sky map
	LoadTexture("skyCubeMap", L"../../Textures/desertcube1024.dds");
}

void TexColumnsApp::BuildRootSignature()
{
	CD3DX12_DESCRIPTOR_RANGE texTable;
	texTable.Init(
		D3D12_DESCRIPTOR_RANGE_TYPE_SRV,
		3,  // number of descriptors
		0); // register t0

	CD3DX12_DESCRIPTOR_RANGE texTable1;
	texTable1.Init(
		D3D12_DESCRIPTOR_RANGE_TYPE_SRV,
		1,  // number of descriptors
		3); // register t3

	// Root parameter can be a table, root descriptor or root constants.
	CD3DX12_ROOT_PARAMETER slotRootParameter[5];

	// Perfomance TIP: Order from most frequent to least frequent.
	slotRootParameter[0].InitAsDescriptorTable(1, &texTable, D3D12_SHADER_VISIBILITY_ALL);
	slotRootParameter[1].InitAsConstantBufferView(0); // register b0
	slotRootParameter[2].InitAsConstantBufferView(1); // register b1
	slotRootParameter[3].InitAsConstantBufferView(2); // register b2
	slotRootParameter[4].InitAsDescriptorTable(1, &texTable1, D3D12_SHADER_VISIBILITY_ALL);

	auto staticSamplers = GetStaticSamplers();

	// A root signature is an array of root parameters.
	CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc(5, slotRootParameter,
		(UINT)staticSamplers.size(), staticSamplers.data(),
		D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

	// create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
	ComPtr<ID3DBlob> serializedRootSig = nullptr;
	ComPtr<ID3DBlob> errorBlob = nullptr;
	HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1,
		serializedRootSig.GetAddressOf(), errorBlob.GetAddressOf());

	if (errorBlob != nullptr)
	{
		::OutputDebugStringA((char*)errorBlob->GetBufferPointer());
	}
	ThrowIfFailed(hr);

	ThrowIfFailed(md3dDevice->CreateRootSignature(
		0,
		serializedRootSig->GetBufferPointer(),
		serializedRootSig->GetBufferSize(),
		IID_PPV_ARGS(mRootSignature["default"].GetAddressOf())));
}

void TexColumnsApp::BuildDescriptorHeaps()
{
	//
	// Create the SRV heap.
	//
	D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
	srvHeapDesc.NumDescriptors = 200;
	srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&mSrvDescriptorHeap)));

	//
	// Fill out the heap with actual descriptors.
	//
	CD3DX12_CPU_DESCRIPTOR_HANDLE hDescriptor(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());


	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;

	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;

	// texture descriptors except last
	for (int i = 0; i < mTextures.size() - 1; i++)
	{
		auto& tex = mTextures[i]->Resource;
		srvDesc.Format = tex->GetDesc().Format;
		srvDesc.Texture2D.MipLevels = tex->GetDesc().MipLevels;
		md3dDevice->CreateShaderResourceView(tex.Get(), &srvDesc, hDescriptor);
		hDescriptor.Offset(1, mCbvSrvDescriptorSize);
	}

	// last texture = sky box
	auto& skyCubeMap = mTextures.back()->Resource; 

	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
	srvDesc.TextureCube.MostDetailedMip = 0;
	srvDesc.TextureCube.MipLevels = skyCubeMap->GetDesc().MipLevels;
	srvDesc.TextureCube.ResourceMinLODClamp = 0.0f;

	srvDesc.Format = skyCubeMap->GetDesc().Format;
	srvDesc.Texture2D.MipLevels = skyCubeMap->GetDesc().MipLevels;
	md3dDevice->CreateShaderResourceView(skyCubeMap.Get(), &srvDesc, hDescriptor);

	mSkyTexHeapIndex = (UINT)mTextures.size() - 1;
	mShadowMapHeapIndex = mSkyTexHeapIndex + 1;

	mNullCubeSrvIndex = mShadowMapHeapIndex + 1;
	mNullTexSrvIndex = mNullCubeSrvIndex + 1;
	mGBuffer->Channel0SRVHeapIndex = mNullCubeSrvIndex + 1;

	// copy gbuffer resources into the srv heap
	auto srvGBuffer = CD3DX12_CPU_DESCRIPTOR_HANDLE(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());
	srvGBuffer.Offset(mGBuffer->Channel0SRVHeapIndex, mCbvSrvUavDescriptorSize);
	md3dDevice->CopyDescriptorsSimple(mGBuffer->NumBuffers, srvGBuffer,
		mGBuffer->m_SRVDescriptorHeap->GetCPUDescriptorHandleForHeapStart(),
		D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	auto srvCpuStart = mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
	auto srvGpuStart = mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart();
	auto dsvCpuStart = mDsvHeap->GetCPUDescriptorHandleForHeapStart();

	// null srv
	auto nullSrv = CD3DX12_CPU_DESCRIPTOR_HANDLE(srvCpuStart, mNullCubeSrvIndex, mCbvSrvUavDescriptorSize);
	mNullSrv = CD3DX12_GPU_DESCRIPTOR_HANDLE(srvGpuStart, mNullCubeSrvIndex, mCbvSrvUavDescriptorSize);

	md3dDevice->CreateShaderResourceView(nullptr, &srvDesc, nullSrv);
	nullSrv.Offset(1, mCbvSrvUavDescriptorSize);

	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.MipLevels = 1;
	srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;
	md3dDevice->CreateShaderResourceView(nullptr, &srvDesc, nullSrv);

	mShadowMap->BuildDescriptors(
		CD3DX12_CPU_DESCRIPTOR_HANDLE(srvCpuStart, mShadowMapHeapIndex, mCbvSrvUavDescriptorSize),
		CD3DX12_GPU_DESCRIPTOR_HANDLE(srvGpuStart, mShadowMapHeapIndex, mCbvSrvUavDescriptorSize),
		CD3DX12_CPU_DESCRIPTOR_HANDLE(dsvCpuStart, 1, mDsvDescriptorSize));
}

void TexColumnsApp::BuildShadersAndInputLayout()
{
	const D3D_SHADER_MACRO alphaTestDefines[] =
	{
		"ALPHA_TEST", "1",
		NULL, NULL
	};

	mShaders["standardVS"] = d3dUtil::CompileShader(L"Shaders\\Tessellation.hlsl", nullptr, "VS", "vs_5_0");
	mShaders["tessHS"] = d3dUtil::CompileShader(L"Shaders\\Tessellation.hlsl", nullptr, "HS", "hs_5_0");
	mShaders["tessDS"] = d3dUtil::CompileShader(L"Shaders\\Tessellation.hlsl", nullptr, "DS", "ds_5_0");
	mShaders["opaquePS"] = d3dUtil::CompileShader(L"Shaders\\Tessellation.hlsl", nullptr, "PS", "ps_5_0");
	mShaders["deferredPS"] = d3dUtil::CompileShader(L"Shaders\\Tessellation.hlsl", nullptr, "DeferredPS", "ps_5_0");
	
	mShaders["shadowVS"] = d3dUtil::CompileShader(L"Shaders\\Shadows.hlsl", nullptr, "VS", "vs_5_1");
	mShaders["shadowGS"] = d3dUtil::CompileShader(L"Shaders\\Shadows.hlsl", nullptr, "GS", "gs_5_1");
	mShaders["shadowOpaquePS"] = d3dUtil::CompileShader(L"Shaders\\Shadows.hlsl", nullptr, "PS", "ps_5_1");
	mShaders["shadowAlphaTestedPS"] = d3dUtil::CompileShader(L"Shaders\\Shadows.hlsl", alphaTestDefines, "PS", "ps_5_1");

	mShaders["debugVS"] = d3dUtil::CompileShader(L"Shaders\\ShadowDebug.hlsl", nullptr, "VS", "vs_5_1");
	mShaders["debugPS"] = d3dUtil::CompileShader(L"Shaders\\ShadowDebug.hlsl", nullptr, "PS", "ps_5_1");

	mShaders["skyVS"] = d3dUtil::CompileShader(L"Shaders\\Sky.hlsl", nullptr, "VS", "vs_5_1");
	mShaders["skyPS"] = d3dUtil::CompileShader(L"Shaders\\Sky.hlsl", nullptr, "PS", "ps_5_1");

	mInputLayout =
	{
		{ "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
		{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 36, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	};
}

void TexColumnsApp::BuildShapeGeometry()
{
	GeometryGenerator geoGen;
	std::vector<GeometryGenerator::MeshData> allMeshData;

	// if you want to generate new model -- generate it here
	allMeshData.push_back( geoGen.CreateGrid(50.0f, 50.0f, 50, 50, 1.0f) );           // grid
	allMeshData.push_back( geoGen.CreateBox(10.0f, 10.0f, 10.0f, 3) );                // box
	allMeshData.push_back( geoGen.LoadModel("..\\..\\Models\\trex.obj"));             // trex
	allMeshData.push_back( geoGen.LoadModel("..\\..\\Models\\Baryonyx.obj"));         // baronyx

	//
	// We are concatenating all the geometry into one big vertex/index buffer.  So
	// define the regions in the buffer each submesh covers.
	//

	// Cache the vertex offsets to each object in the concatenated vertex and index buffer.
	std::vector<UINT> vertexOffsets;
	vertexOffsets.push_back(0);
	std::vector<UINT> indexOffsets;
	indexOffsets.push_back(0);
	for (size_t i = 1; i < allMeshData.size(); i++)
	{
		vertexOffsets.push_back(vertexOffsets.at(i - 1) + (UINT) allMeshData.at(i - 1).Vertices.size());
		indexOffsets.push_back(indexOffsets.at(i - 1) + (UINT) allMeshData.at(i - 1).Indices32.size());
	}
	
	// generating submeshes
	size_t totalVertexCount = 0;
	std::vector<SubmeshGeometry> allSubmeshes;
	for (size_t i = 0; i < allMeshData.size(); i++)
	{
		SubmeshGeometry submesh;
		auto& mesh = allMeshData.at(i);
		submesh.IndexCount = (UINT)mesh.Indices32.size();
		submesh.StartIndexLocation = indexOffsets.at(i);
		submesh.BaseVertexLocation = vertexOffsets.at(i);
		allSubmeshes.push_back(submesh);
		totalVertexCount += mesh.Vertices.size();
	}

	// pack the vertices of all the meshes into one vertex buffer
	std::vector<Vertex> vertices(totalVertexCount);
	UINT k = 0;
	for (GeometryGenerator::MeshData mesh : allMeshData) {
		for (size_t i = 0; i < mesh.Vertices.size(); ++i, ++k)
		{
			vertices[k].Tangent = mesh.Vertices[i].TangentU;
			vertices[k].Pos = mesh.Vertices[i].Position;
			vertices[k].Normal = mesh.Vertices[i].Normal;
			vertices[k].TexC = mesh.Vertices[i].TexC;
		}
	}

	std::vector<std::uint16_t> indices;
	for (GeometryGenerator::MeshData mesh : allMeshData)
		indices.insert(indices.end(), std::begin(mesh.GetIndices16()), std::end(mesh.GetIndices16()));

	const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
	const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "shapeGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

	geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
		mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	for (size_t i = 0; i < allMeshData.size(); i++)
	{
		geo->DrawArgs[allMeshData.at(i).name] = allSubmeshes.at(i);
	}

	mGeometries[geo->Name] = std::move(geo);
}

void TexColumnsApp::BuildPSOs()
{
	D3D12_GRAPHICS_PIPELINE_STATE_DESC opaquePsoDesc;

	//
	// PSO for opaque objects.
	//
	ZeroMemory(&opaquePsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	opaquePsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	opaquePsoDesc.pRootSignature = mRootSignature["default"].Get();
	opaquePsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["standardVS"]->GetBufferPointer()),
		mShaders["standardVS"]->GetBufferSize()
	};
	opaquePsoDesc.HS =
	{
		reinterpret_cast<BYTE*>(mShaders["tessHS"]->GetBufferPointer()),
		mShaders["tessHS"]->GetBufferSize()
	};
	opaquePsoDesc.DS =
	{
		reinterpret_cast<BYTE*>(mShaders["tessDS"]->GetBufferPointer()),
		mShaders["tessDS"]->GetBufferSize()
	};
	opaquePsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["opaquePS"]->GetBufferPointer()),
		mShaders["opaquePS"]->GetBufferSize()
	};
	opaquePsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
#ifdef DEBUG_VIEW
	opaquePsoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
#else
	opaquePsoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID;
#endif // DEBUG_VIEW
	opaquePsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	opaquePsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	opaquePsoDesc.SampleMask = UINT_MAX;
	opaquePsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_PATCH;
	opaquePsoDesc.NumRenderTargets = 1;
	opaquePsoDesc.RTVFormats[0] = mBackBufferFormat;
	opaquePsoDesc.SampleDesc.Count = m4xMsaaState ? 4 : 1;
	opaquePsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
	opaquePsoDesc.DSVFormat = mDepthStencilFormat;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&opaquePsoDesc, IID_PPV_ARGS(&mPSOs["opaque"])));

	//
	// PSO for shadow map pass.
	//
	D3D12_GRAPHICS_PIPELINE_STATE_DESC smapPsoDesc;
	ZeroMemory(&smapPsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	smapPsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
	smapPsoDesc.pRootSignature = mRootSignature["default"].Get();
	smapPsoDesc.VS =
	{
	  reinterpret_cast<BYTE*>(mShaders["shadowVS"]->GetBufferPointer()),
	  mShaders["shadowVS"]->GetBufferSize()
	};
	smapPsoDesc.GS =
	{
	  reinterpret_cast<BYTE*>(mShaders["shadowGS"]->GetBufferPointer()),
	  mShaders["shadowGS"]->GetBufferSize()
	};
	smapPsoDesc.PS = { nullptr, 0 };

	smapPsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	smapPsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	smapPsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	smapPsoDesc.SampleMask = UINT_MAX;
	smapPsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	smapPsoDesc.SampleDesc.Count = 1;
	smapPsoDesc.SampleDesc.Quality = 0;
	smapPsoDesc.DSVFormat = mDepthStencilFormat;
	smapPsoDesc.RTVFormats[0] = DXGI_FORMAT_UNKNOWN;
	smapPsoDesc.NumRenderTargets = 0;
	smapPsoDesc.RasterizerState.DepthBias = 1000;
	smapPsoDesc.RasterizerState.DepthBiasClamp = 0.0f;
	smapPsoDesc.RasterizerState.SlopeScaledDepthBias = 1.0f;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&smapPsoDesc, IID_PPV_ARGS(&mPSOs["shadow_opaque"])));

	//
	// PSO for debug layer.
	//
	D3D12_GRAPHICS_PIPELINE_STATE_DESC debugPsoDesc = smapPsoDesc;
	debugPsoDesc.pRootSignature = mRootSignature["default"].Get();
	debugPsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["debugVS"]->GetBufferPointer()),
		mShaders["debugVS"]->GetBufferSize()
	};
	debugPsoDesc.GS = {nullptr, 0};
	debugPsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["debugPS"]->GetBufferPointer()),
		mShaders["debugPS"]->GetBufferSize()
	};
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&debugPsoDesc, IID_PPV_ARGS(&mPSOs["debug"])));

	//
	// PSO for sky.
	//
	D3D12_GRAPHICS_PIPELINE_STATE_DESC skyPsoDesc = opaquePsoDesc;

	// The camera is inside the sky sphere, so just turn off culling.
	skyPsoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;

	// Make sure the depth function is LESS_EQUAL and not just LESS.  
	// Otherwise, the normalized depth values at z = 1 (NDC) will 
	// fail the depth test if the depth buffer was cleared to 1.
	skyPsoDesc.DepthStencilState.DepthFunc = D3D12_COMPARISON_FUNC_LESS_EQUAL;
	skyPsoDesc.pRootSignature = mRootSignature["default"].Get();
	skyPsoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mShaders["skyVS"]->GetBufferPointer()),
		mShaders["skyVS"]->GetBufferSize()
	};
	skyPsoDesc.DS = { nullptr, 0 };
	skyPsoDesc.HS = { nullptr, 0 };
	skyPsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["skyPS"]->GetBufferPointer()),
		mShaders["skyPS"]->GetBufferSize()
	};
	skyPsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&skyPsoDesc, IID_PPV_ARGS(&mPSOs["sky"])));

	//
	//	PSO for deferred geometry pass
	//
	D3D12_GRAPHICS_PIPELINE_STATE_DESC deferredGeometryPsoDesc = opaquePsoDesc;
	deferredGeometryPsoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mShaders["deferredPS"]->GetBufferPointer()),
		mShaders["deferredPS"]->GetBufferSize()
	};
	deferredGeometryPsoDesc.DepthStencilState.DepthEnable = false;
	deferredGeometryPsoDesc.NumRenderTargets = 5;
	deferredGeometryPsoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;		   // diffuse
	deferredGeometryPsoDesc.RTVFormats[1] = DXGI_FORMAT_R32G32B32A32_FLOAT;    // zwzanashih
	deferredGeometryPsoDesc.RTVFormats[2] = DXGI_FORMAT_R16G16B16A16_SNORM;	   // normal
	deferredGeometryPsoDesc.RTVFormats[3] = DXGI_FORMAT_R8G8B8A8_UNORM;        // diffuse albedo
	deferredGeometryPsoDesc.RTVFormats[4] = DXGI_FORMAT_R8G8B8A8_UNORM;        // fresnel & roughness
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&deferredGeometryPsoDesc, IID_PPV_ARGS(&mPSOs["deferredGeometry"])));
}

void TexColumnsApp::BuildFrameResources()
{
	for (int i = 0; i < gNumFrameResources; ++i)
	{
		mFrameResources.push_back(std::make_unique<FrameResource>(md3dDevice.Get(),
			2, (UINT)mAllRitems.size(), (UINT)mMaterials.size()));
	}
}

void TexColumnsApp::BuildMaterials()
{
	auto bricks0 = std::make_unique<Material>();
	bricks0->Name = "bricks0";
	bricks0->MatCBIndex = 0;
	bricks0->DiffuseSrvHeapIndex = 0;
	bricks0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	bricks0->FresnelR0 = XMFLOAT3(0.02f, 0.02f, 0.02f);
	bricks0->Roughness = 0.1f;

	auto stone0 = std::make_unique<Material>();
	stone0->Name = "stone0";
	stone0->MatCBIndex = 1;
	stone0->DiffuseSrvHeapIndex = 0;
	stone0->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	stone0->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	stone0->Roughness = 0.3f;

	auto gorg = std::make_unique<Material>();
	gorg->Name = "gorg";
	gorg->MatCBIndex = 2;
	gorg->DiffuseSrvHeapIndex = 3;
	gorg->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	gorg->FresnelR0 = XMFLOAT3(0.05f, 0.05f, 0.05f);
	gorg->Roughness = 0.3f;

	auto sky = std::make_unique<Material>();
	sky->Name = "sky";
	sky->MatCBIndex = 3;
	sky->DiffuseSrvHeapIndex = 0;
	sky->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	sky->FresnelR0 = XMFLOAT3(0.1f, 0.1f, 0.1f);
	sky->Roughness = 1.0f;

	mMaterials["bricks0"] = std::move(bricks0);
	mMaterials["stone0"] = std::move(stone0);
	mMaterials["gorg"] = std::move(gorg);
	mMaterials["sky"] = std::move(sky);
}

void TexColumnsApp::BuildRenderItem(std::string name, std::string material, XMMATRIX translate, int layer, float scale, float scaleTex)
{
	auto ptr = std::make_unique<RenderItem>();
	XMStoreFloat4x4(&ptr->World, XMMatrixScaling(scale, scale, scale) * translate);
	XMStoreFloat4x4(&ptr->TexTransform, XMMatrixScaling(scaleTex, scaleTex, scaleTex));
	ptr->ObjCBIndex = ObjCBIndex++;
	ptr->Mat = mMaterials[material].get();
	ptr->Geo = mGeometries["shapeGeo"].get();
	if (layer == 0)
		ptr->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_3_CONTROL_POINT_PATCHLIST;
	else
		ptr->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	ptr->IndexCount = ptr->Geo->DrawArgs[name].IndexCount;
	ptr->StartIndexLocation = ptr->Geo->DrawArgs[name].StartIndexLocation;
	ptr->BaseVertexLocation = ptr->Geo->DrawArgs[name].BaseVertexLocation;

	mRitemLayer[layer].push_back(ptr.get());
	mAllRitems.push_back(std::move(ptr));
}

void TexColumnsApp::BuildRenderItems()
{
	BuildRenderItem("sphere", "sky", XMMatrixIdentity(), (int) RenderLayer::Sky, 5000.0f);
	BuildRenderItem("quad", "bricks0", XMMatrixIdentity(), (int)RenderLayer::Debug);

	BuildRenderItem("box", "stone0", XMMatrixTranslation(15.f, 0.f, 0.f));
	BuildRenderItem("grid", "stone0", XMMatrixTranslation(0.f, -5.f, 10.f));
	BuildRenderItem("Baryonyx", "gorg", XMMatrixTranslation(0.f, -5.f, 20.f));
	BuildRenderItem("Baryonyx", "gorg", XMMatrixTranslation(-30.f, -5.f, 40.f));
	BuildRenderItem("Baryonyx", "gorg", XMMatrixTranslation(30.f, -5.f, 0.f));
}

void TexColumnsApp::DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems)
{
	UINT objCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(ObjectConstants));
	UINT matCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(MaterialConstants));

	auto objectCB = mCurrFrameResource->ObjectCB->Resource();
	auto matCB = mCurrFrameResource->MaterialCB->Resource();

	// For each render item...
	for (const auto& ri : ritems)
	{
		Material* mat = ri->Mat;
		UINT textureIndex = mat->DiffuseSrvHeapIndex;

		// register texture in t0
		CD3DX12_GPU_DESCRIPTOR_HANDLE texHandle(
			mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart(),
			textureIndex,  // Смещение в куче дескрипторов
			mCbvSrvDescriptorSize
		);
		cmdList->SetGraphicsRootDescriptorTable(0, texHandle);

		cmdList->IASetVertexBuffers(0, 1, &ri->Geo->VertexBufferView());
		cmdList->IASetIndexBuffer(&ri->Geo->IndexBufferView());
		//cmdList->IASetPrimitiveTopology(ri->PrimitiveType);

		//CD3DX12_GPU_DESCRIPTOR_HANDLE tex(mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());
		//tex.Offset(ri->Mat->DiffuseSrvHeapIndex, mCbvSrvDescriptorSize);

		D3D12_GPU_VIRTUAL_ADDRESS objCBAddress = objectCB->GetGPUVirtualAddress() + ri->ObjCBIndex * objCBByteSize;
		D3D12_GPU_VIRTUAL_ADDRESS matCBAddress = matCB->GetGPUVirtualAddress() + ri->Mat->MatCBIndex * matCBByteSize;

		//cmdList->SetGraphicsRootDescriptorTable(0, tex);
		cmdList->SetGraphicsRootConstantBufferView(1, objCBAddress);
		cmdList->SetGraphicsRootConstantBufferView(3, matCBAddress);

		cmdList->DrawIndexedInstanced(ri->IndexCount, 1, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
	}
}

void TexColumnsApp::DrawSceneToShadowMap()
{
	mCommandList->RSSetViewports(1, &mShadowMap->Viewport());
	mCommandList->RSSetScissorRects(1, &mShadowMap->ScissorRect());

	// Change to DEPTH_WRITE.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(mShadowMap->Resource(),
		D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE));

	UINT passCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(PassConstants));

	// Clear the back buffer and depth buffer.
	mCommandList->ClearDepthStencilView(mShadowMap->Dsv(),
		D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

	// Set null render target because we are only going to draw to
	// depth buffer.  Setting a null render target will disable color writes.
	// Note the active PSO also must specify a render target count of 0.
	mCommandList->OMSetRenderTargets(0, nullptr, false, &mShadowMap->Dsv());

	// Bind the pass constant buffer for the shadow map pass.
	auto passCB = mCurrFrameResource->PassCB->Resource();
	D3D12_GPU_VIRTUAL_ADDRESS passCBAddress = passCB->GetGPUVirtualAddress() + 1 * passCBByteSize;
	mCommandList->SetGraphicsRootConstantBufferView(1, passCBAddress);

	mCommandList->SetPipelineState(mPSOs["shadow_opaque"].Get());

	DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque]);

	// Change back to GENERIC_READ so we can read the texture in a shader.
	mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(mShadowMap->Resource(),
		D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_GENERIC_READ));
}

void TexColumnsApp::BuildPostProcessResources()
{
	// Create render target for post-processing
	D3D12_RESOURCE_DESC renderTargetDesc;
	ZeroMemory(&renderTargetDesc, sizeof(D3D12_RESOURCE_DESC));
	renderTargetDesc.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	renderTargetDesc.Alignment = 0;
	renderTargetDesc.Width = mClientWidth;
	renderTargetDesc.Height = mClientHeight;
	renderTargetDesc.DepthOrArraySize = 1;
	renderTargetDesc.MipLevels = 1;
	renderTargetDesc.Format = mBackBufferFormat;
	renderTargetDesc.SampleDesc.Count = 1;
	renderTargetDesc.SampleDesc.Quality = 0;
	renderTargetDesc.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN;
	renderTargetDesc.Flags = D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET;

	D3D12_CLEAR_VALUE clearValue;
	clearValue.Format = mBackBufferFormat;
	memcpy(clearValue.Color, Colors::LightSteelBlue, sizeof(float) * 4);

	ThrowIfFailed(md3dDevice->CreateCommittedResource(
		&CD3DX12_HEAP_PROPERTIES(D3D12_HEAP_TYPE_DEFAULT),
		D3D12_HEAP_FLAG_NONE,
		&renderTargetDesc,
		D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE,
		&clearValue,
		IID_PPV_ARGS(&mPostProcessRenderTarget)));

	// Create RTV heap
	D3D12_DESCRIPTOR_HEAP_DESC rtvHeapDesc;
	rtvHeapDesc.NumDescriptors = 1;
	rtvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
	rtvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
	rtvHeapDesc.NodeMask = 0;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(
		&rtvHeapDesc, IID_PPV_ARGS(&mPostProcessRTVHeap)));

	// Create RTV
	CD3DX12_CPU_DESCRIPTOR_HANDLE rtvHandle(
		mPostProcessRTVHeap->GetCPUDescriptorHandleForHeapStart());
	md3dDevice->CreateRenderTargetView(
		mPostProcessRenderTarget.Get(), nullptr, rtvHandle);

	// Create SRV heap
	D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
	srvHeapDesc.NumDescriptors = 2;
	srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
	srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	ThrowIfFailed(md3dDevice->CreateDescriptorHeap(
		&srvHeapDesc, IID_PPV_ARGS(&mPostProcessSRVHeap)));

	// Create SRV for render target
	CD3DX12_CPU_DESCRIPTOR_HANDLE srvHandle(
		mPostProcessSRVHeap->GetCPUDescriptorHandleForHeapStart());
	D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
	srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
	srvDesc.Format = mBackBufferFormat;
	srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MostDetailedMip = 0;
	srvDesc.Texture2D.MipLevels = 1;
	md3dDevice->CreateShaderResourceView(
		mPostProcessRenderTarget.Get(), &srvDesc, srvHandle);

	// Create SRV for depth buffer
	srvHandle.Offset(1, mCbvSrvDescriptorSize);
	srvDesc.Format = DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
	md3dDevice->CreateShaderResourceView(
		mDepthStencilBuffer.Get(), &srvDesc, srvHandle);
}

void TexColumnsApp::BuildPostProcessPSO()
{
	// Compile shaders with error handling
	try
	{
		mPostProcessShaders["postVS"] = d3dUtil::CompileShader(L"Shaders\\PostProcessing.hlsl", nullptr, "VS", "vs_5_0");
		mPostProcessShaders["postPS"] = d3dUtil::CompileShader(L"Shaders\\PostProcessing.hlsl", nullptr, "PS", "ps_5_0");
	}
	catch (DxException& e)
	{
		ThrowIfFailed(S_SERBDNT);
	}

	D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc;
	ZeroMemory(&psoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
	psoDesc.InputLayout = { nullptr, 0 };
	psoDesc.pRootSignature = mPostProcessRootSignature.Get();
	psoDesc.VS =
	{
		reinterpret_cast<BYTE*>(mPostProcessShaders["postVS"]->GetBufferPointer()),
		mPostProcessShaders["postVS"]->GetBufferSize()
	};
	psoDesc.PS =
	{
		reinterpret_cast<BYTE*>(mPostProcessShaders["postPS"]->GetBufferPointer()),
		mPostProcessShaders["postPS"]->GetBufferSize()
	};
	psoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	psoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	psoDesc.DepthStencilState.DepthEnable = false;
	psoDesc.DepthStencilState.StencilEnable = false;
	psoDesc.DSVFormat = mDepthStencilFormat;
	psoDesc.SampleMask = UINT_MAX;
	psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	psoDesc.NumRenderTargets = 1;
	psoDesc.RTVFormats[0] = mBackBufferFormat;
	psoDesc.SampleDesc.Count = 1;
	psoDesc.SampleDesc.Quality = 0;
	ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(&mPostProcessPSO)));
}

void TexColumnsApp::BuildPostProcessRootSignature()
{
	CD3DX12_DESCRIPTOR_RANGE srvTable[2];
	srvTable[0].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 0); // t0, base texture
	srvTable[1].Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 1); // t1, depth buffer

	CD3DX12_ROOT_PARAMETER slotRootParameter[3];
	slotRootParameter[0].InitAsDescriptorTable(1, &srvTable[0], D3D12_SHADER_VISIBILITY_PIXEL);
	slotRootParameter[1].InitAsDescriptorTable(1, &srvTable[1], D3D12_SHADER_VISIBILITY_PIXEL);
	slotRootParameter[2].InitAsConstantBufferView(0, 0, D3D12_SHADER_VISIBILITY_PIXEL); // PostProcessSettings (b0)

	CD3DX12_STATIC_SAMPLER_DESC sampler(
		0, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP, // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP, // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc(3, slotRootParameter,
		1, &sampler,
		D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

	ComPtr<ID3DBlob> serializedRootSig = nullptr;
	ComPtr<ID3DBlob> errorBlob = nullptr;
	HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1,
		serializedRootSig.GetAddressOf(), errorBlob.GetAddressOf());

	if (errorBlob != nullptr)
	{
		OutputDebugStringA((char*)errorBlob->GetBufferPointer());
	}
	ThrowIfFailed(hr);

	ThrowIfFailed(md3dDevice->CreateRootSignature(
		0,
		serializedRootSig->GetBufferPointer(),
		serializedRootSig->GetBufferSize(),
		IID_PPV_ARGS(&mPostProcessRootSignature)));
}

std::array<const CD3DX12_STATIC_SAMPLER_DESC, 7> TexColumnsApp::GetStaticSamplers()
{
	// Applications usually only need a handful of samplers.  So just define them all up front
	// and keep them available as part of the root signature.  

	const CD3DX12_STATIC_SAMPLER_DESC pointWrap(
		0, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC pointClamp(
		1, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearWrap(
		2, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC linearClamp(
		3, // shaderRegister
		D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicWrap(
		4, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressW
		0.0f,                             // mipLODBias
		8);                               // maxAnisotropy

	const CD3DX12_STATIC_SAMPLER_DESC anisotropicClamp(
		5, // shaderRegister
		D3D12_FILTER_ANISOTROPIC, // filter
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressW
		0.0f,                              // mipLODBias
		8);                                // maxAnisotropy

	const CD3DX12_STATIC_SAMPLER_DESC shadow(
		6, // shaderRegister
		D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT, // filter
		D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressU
		D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressV
		D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressW
		0.0f,                               // mipLODBias
		16,                                 // maxAnisotropy
		D3D12_COMPARISON_FUNC_LESS_EQUAL,
		D3D12_STATIC_BORDER_COLOR_OPAQUE_WHITE);

	return {
		pointWrap, pointClamp,
		linearWrap, linearClamp,
		anisotropicWrap, anisotropicClamp,
		shadow};
}

