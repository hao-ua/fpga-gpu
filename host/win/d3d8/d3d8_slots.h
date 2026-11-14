/* d3d8_slots.h — Direct3D 8's method tables in order: O is a method of d3d8_main.c, S the method of that name in d3d9.dll's table, M one that d3d9.dll calls otherwise.
 * d3d8_main.c builds the tables from these lists, d3d8_check.c proves them against the SDK's d3d8.h at compile time. d3d8.h and d3d9.h cannot share a file. */
#ifndef D3D8_SLOTS_H
#define D3D8_SLOTS_H

#define D8_D3D(O, S) \
    O(QueryInterface) S(AddRef) S(Release) S(RegisterSoftwareDevice) S(GetAdapterCount) O(GetAdapterIdentifier) O(GetAdapterModeCount) O(EnumAdapterModes) S(GetAdapterDisplayMode) \
    S(CheckDeviceType) S(CheckDeviceFormat) O(CheckDeviceMultiSampleType) S(CheckDepthStencilMatch) O(GetDeviceCaps) S(GetAdapterMonitor) O(CreateDevice)

#define D8_DEVICE(O, S, M) \
    O(QueryInterface) S(AddRef) S(Release) S(TestCooperativeLevel) S(GetAvailableTextureMem) O(ResourceManagerDiscardBytes) S(GetDirect3D) O(GetDeviceCaps) O(GetDisplayMode) S(GetCreationParameters) \
    S(SetCursorProperties) S(SetCursorPosition) S(ShowCursor) O(CreateAdditionalSwapChain) O(Reset) S(Present) O(GetBackBuffer) O(GetRasterStatus) O(SetGammaRamp) O(GetGammaRamp) \
    O(CreateTexture) O(CreateVolumeTexture) O(CreateCubeTexture) O(CreateVertexBuffer) O(CreateIndexBuffer) O(CreateRenderTarget) O(CreateDepthStencilSurface) O(CreateImageSurface) \
    O(CopyRects) S(UpdateTexture) O(GetFrontBuffer) O(SetRenderTarget) O(GetRenderTarget) S(GetDepthStencilSurface) S(BeginScene) S(EndScene) S(Clear) \
    S(SetTransform) S(GetTransform) S(MultiplyTransform) S(SetViewport) S(GetViewport) S(SetMaterial) S(GetMaterial) S(SetLight) S(GetLight) S(LightEnable) S(GetLightEnable) S(SetClipPlane) S(GetClipPlane) \
    O(SetRenderState) S(GetRenderState) S(BeginStateBlock) S(EndStateBlock) O(ApplyStateBlock) O(CaptureStateBlock) O(DeleteStateBlock) S(CreateStateBlock) S(SetClipStatus) S(GetClipStatus) \
    S(GetTexture) S(SetTexture) O(GetTextureStageState) O(SetTextureStageState) S(ValidateDevice) O(GetInfo) S(SetPaletteEntries) S(GetPaletteEntries) S(SetCurrentTexturePalette) S(GetCurrentTexturePalette) \
    S(DrawPrimitive) O(DrawIndexedPrimitive) S(DrawPrimitiveUP) S(DrawIndexedPrimitiveUP) O(ProcessVertices) O(CreateVertexShader) O(SetVertexShader) O(GetVertexShader) O(DeleteVertexShader) \
    M(SetVertexShaderConstant, SetVertexShaderConstantF) M(GetVertexShaderConstant, GetVertexShaderConstantF) O(GetVertexShaderDeclaration) O(GetVertexShaderFunction) \
    O(SetStreamSource) O(GetStreamSource) O(SetIndices) O(GetIndices) S(CreatePixelShader) S(SetPixelShader) O(GetPixelShader) O(DeletePixelShader) \
    M(SetPixelShaderConstant, SetPixelShaderConstantF) M(GetPixelShaderConstant, GetPixelShaderConstantF) O(GetPixelShaderFunction) S(DrawRectPatch) S(DrawTriPatch) S(DeletePatch)

#define D8_RESOURCE_HEAD(O, S) O(QueryInterface) S(AddRef) S(Release) S(GetDevice) S(SetPrivateData) S(GetPrivateData) S(FreePrivateData)
#define D8_RESOURCE(O, S) D8_RESOURCE_HEAD(O, S) S(SetPriority) S(GetPriority) S(PreLoad) S(GetType)

#define D8_SURFACE(O, S) D8_RESOURCE_HEAD(O, S) S(GetContainer) O(GetDesc) S(LockRect) S(UnlockRect)
#define D8_TEXTURE(O, S) D8_RESOURCE(O, S) S(SetLOD) S(GetLOD) S(GetLevelCount) O(GetLevelDesc) S(GetSurfaceLevel) S(LockRect) S(UnlockRect) S(AddDirtyRect)
#define D8_CUBE(O, S) D8_RESOURCE(O, S) S(SetLOD) S(GetLOD) S(GetLevelCount) O(GetLevelDesc) S(GetCubeMapSurface) S(LockRect) S(UnlockRect) S(AddDirtyRect)
#define D8_VOLUME(O, S) D8_RESOURCE_HEAD(O, S) S(GetContainer) O(GetDesc) S(LockBox) S(UnlockBox)
#define D8_VOLTEX(O, S) D8_RESOURCE(O, S) S(SetLOD) S(GetLOD) S(GetLevelCount) O(GetLevelDesc) S(GetVolumeLevel) S(LockBox) S(UnlockBox) S(AddDirtyBox)
#define D8_BUFFER(O, S) D8_RESOURCE(O, S) S(Lock) S(Unlock) S(GetDesc)

/* the structures that changed, as d3d8_main.c declares them */
#define D8_SIZE_CAPS 212
#define D8_SIZE_PP 52
#define D8_SIZE_DESC 32
#define D8_SIZE_VDESC 32
#define D8_SIZE_ID 1068
#endif
