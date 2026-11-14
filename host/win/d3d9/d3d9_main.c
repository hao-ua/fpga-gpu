/* d3d9_main.c — Direct3DCreate9 and IDirect3D9: one adapter, the HDMI modes, what the hardware can do. */
#define INITGUID                                /* this file holds the IID_* definitions */
#include "d3d9_internal.h"

typedef struct d3obj { const IDirect3D9Vtbl *vtbl; LONG refs; } d3obj;

/* the sizes a game may ask for: board B's HDMI modes (gpu2core.h). 1 024 x 768 is its own mode on a board B that has mode 4; one without it shows it 1:1
   in 1 280 x 800 (g2_mode_select) */
static const struct { UINT w, h; } MODES[] = { { 640, 480 }, { 800, 600 }, { 1024, 768 }, { 1280, 720 }, { 1280, 800 } };
#define NMODES (sizeof MODES / sizeof MODES[0])

static HRESULT WINAPI o_QueryInterface(IDirect3D9 *t, REFIID id, void **o)
{
    if (IsEqualGUID(id, &IID_IUnknown) || IsEqualGUID(id, &IID_IDirect3D9)) { IDirect3D9_AddRef(t); *o = t; return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}
static ULONG WINAPI o_AddRef(IDirect3D9 *t) { return (ULONG)InterlockedIncrement(&((d3obj *)t)->refs); }
static ULONG WINAPI o_Release(IDirect3D9 *t) { LONG r = InterlockedDecrement(&((d3obj *)t)->refs); if (!r) free(t); return (ULONG)r; }
static HRESULT WINAPI o_RegisterSoftwareDevice(IDirect3D9 *t, void *f) { (void)t; (void)f; return D3DERR_INVALIDCALL; }
static UINT WINAPI o_GetAdapterCount(IDirect3D9 *t) { (void)t; return 1; }
static HRESULT WINAPI o_GetAdapterIdentifier(IDirect3D9 *t, UINT a, DWORD f, D3DADAPTER_IDENTIFIER9 *o)
{
    (void)t; (void)f;
    if (a) return D3DERR_INVALIDCALL;
    memset(o, 0, sizeof *o);
    strcpy(o->Driver, "d3d9.dll");
    strcpy(o->Description, "GPU2 two-board FPGA GPU");
    strcpy(o->DeviceName, "\\\\.\\DISPLAY1");
    o->VendorId = 0x1D0F; o->DeviceId = 0x0002; o->WHQLLevel = 1;
    return D3D_OK;
}
static UINT WINAPI o_GetAdapterModeCount(IDirect3D9 *t, UINT a, D3DFORMAT f) { (void)t; if (a) return 0; return f == D3DFMT_X8R8G8B8 || f == D3DFMT_R5G6B5 || f == D3DFMT_X1R5G5B5 ? NMODES : 0; }
static HRESULT WINAPI o_EnumAdapterModes(IDirect3D9 *t, UINT a, D3DFORMAT f, UINT i, D3DDISPLAYMODE *m)
{
    (void)t;
    if (a || i >= NMODES) return D3DERR_INVALIDCALL;
    m->Width = MODES[i].w; m->Height = MODES[i].h; m->RefreshRate = 60; m->Format = f;
    return D3D_OK;
}
static HRESULT WINAPI o_GetAdapterDisplayMode(IDirect3D9 *t, UINT a, D3DDISPLAYMODE *m)
{
    (void)t;
    if (a) return D3DERR_INVALIDCALL;
    m->Width = (UINT)GetSystemMetrics(SM_CXSCREEN); m->Height = (UINT)GetSystemMetrics(SM_CYSCREEN); m->RefreshRate = 60; m->Format = D3DFMT_X8R8G8B8;
    return D3D_OK;
}
static HRESULT WINAPI o_CheckDeviceType(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT df, D3DFORMAT bf, BOOL w)
{
    (void)t; (void)w;
    if (d3_trace()) g2e_log("d3d9: CheckDeviceType %d, %d", (int)df, (int)bf);
    if (a || ty != D3DDEVTYPE_HAL) return D3DERR_NOTAVAILABLE;
    if (df != D3DFMT_X8R8G8B8 && df != D3DFMT_R5G6B5 && df != D3DFMT_X1R5G5B5) return D3DERR_NOTAVAILABLE;     /* 5:5:5 renders and scans out as 5:6:5, which holds it */
    return bf == D3DFMT_UNKNOWN || bf == D3DFMT_A8R8G8B8 || bf == D3DFMT_X8R8G8B8 || bf == D3DFMT_R5G6B5 || bf == D3DFMT_X1R5G5B5 || bf == D3DFMT_A1R5G5B5 ? D3D_OK : D3DERR_NOTAVAILABLE;
}
int d3_trace(void) { static int v = -1; if (v < 0) v = g2e_get_int("GPU2_TRACE_D3D", 0); return v; }

static HRESULT WINAPI o_CheckDeviceFormat(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT af, DWORD u, D3DRESOURCETYPE r, D3DFORMAT f)
{
    uint32_t g;
    (void)t; (void)af;
    D3T("d3d9: CheckDeviceFormat usage %08lx type %d format %d", (unsigned long)u, (int)r, (int)f);
    if (a || ty != D3DDEVTYPE_HAL) return D3DERR_NOTAVAILABLE;
    if ((r == D3DRTYPE_VOLUMETEXTURE || r == D3DRTYPE_VOLUME) && ((u & (D3DUSAGE_DEPTHSTENCIL | D3DUSAGE_RENDERTARGET | D3DUSAGE_AUTOGENMIPMAP)) || !d3_has_3d())) return D3DERR_NOTAVAILABLE;
    if (u & D3DUSAGE_DEPTHSTENCIL) return (r == D3DRTYPE_TEXTURE ? d3_fmt_dtex(f) : d3_fmt_ds(f)) ? D3D_OK : D3DERR_NOTAVAILABLE;
    if (u & D3DUSAGE_RENDERTARGET) return d3_fmt_rt(f) ? D3D_OK : D3DERR_NOTAVAILABLE;
    if (u & D3DUSAGE_QUERY_VERTEXTEXTURE) return D3DERR_NOTAVAILABLE;
    if ((u & (D3DUSAGE_QUERY_SRGBREAD | D3DUSAGE_QUERY_SRGBWRITE)) && !d3_fmt_srgb(f)) return D3DERR_NOTAVAILABLE;
    if ((u & D3DUSAGE_AUTOGENMIPMAP) && d3_fmt_bytes(f) == 0) return D3DOK_NOAUTOGEN;   /* DXT: a chain would have to be compressed */
    return d3_fmt_g2(f, &g) ? D3D_OK : D3DERR_NOTAVAILABLE;
}
static HRESULT WINAPI o_CheckDeviceMultiSampleType(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT f, BOOL w, D3DMULTISAMPLE_TYPE ms, DWORD *q)
{
    (void)t; (void)a; (void)ty; (void)f; (void)w;
    if (q) *q = 1;
    return ms == D3DMULTISAMPLE_NONE ? D3D_OK : D3DERR_NOTAVAILABLE;
}
static HRESULT WINAPI o_CheckDepthStencilMatch(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT af, D3DFORMAT rf, D3DFORMAT df) { (void)t; (void)a; (void)ty; (void)af; (void)rf; return d3_fmt_ds(df) ? D3D_OK : D3DERR_NOTAVAILABLE; }
static HRESULT WINAPI o_CheckDeviceFormatConversion(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT s, D3DFORMAT o) { (void)t; (void)a; (void)ty; return s == o ? D3D_OK : D3DERR_NOTAVAILABLE; }

/* volume textures only where board B has 3D textures (d3_has_3d): a B without them has no program path for Direct3D's volumes, so the caps do not offer them */
void d3_caps(D3DCAPS9 *c)
{
    int vol = d3_has_3d();
    memset(c, 0, sizeof *c);
    c->DeviceType = D3DDEVTYPE_HAL;
    c->Caps2 = D3DCAPS2_CANAUTOGENMIPMAP | D3DCAPS2_DYNAMICTEXTURES;
    c->Caps3 = D3DCAPS3_ALPHA_FULLSCREEN_FLIP_OR_DISCARD;
    c->PresentationIntervals = D3DPRESENT_INTERVAL_ONE | D3DPRESENT_INTERVAL_IMMEDIATE;
    c->CursorCaps = D3DCURSORCAPS_COLOR | D3DCURSORCAPS_LOWRES;
    c->DevCaps = D3DDEVCAPS_DRAWPRIMTLVERTEX | D3DDEVCAPS_EXECUTESYSTEMMEMORY | D3DDEVCAPS_HWRASTERIZATION | D3DDEVCAPS_HWTRANSFORMANDLIGHT | D3DDEVCAPS_PUREDEVICE | D3DDEVCAPS_TEXTUREVIDEOMEMORY | D3DDEVCAPS_DRAWPRIMITIVES2 | D3DDEVCAPS_DRAWPRIMITIVES2EX;
    c->PrimitiveMiscCaps = D3DPMISCCAPS_BLENDOP | D3DPMISCCAPS_CULLCCW | D3DPMISCCAPS_CULLCW | D3DPMISCCAPS_CULLNONE | D3DPMISCCAPS_COLORWRITEENABLE | D3DPMISCCAPS_MASKZ | D3DPMISCCAPS_SEPARATEALPHABLEND | D3DPMISCCAPS_TSSARGTEMP | D3DPMISCCAPS_PERSTAGECONSTANT;
    c->RasterCaps = D3DPRASTERCAPS_SCISSORTEST | D3DPRASTERCAPS_ZTEST | D3DPRASTERCAPS_FOGVERTEX | D3DPRASTERCAPS_FOGTABLE | D3DPRASTERCAPS_MIPMAPLODBIAS | D3DPRASTERCAPS_DEPTHBIAS | D3DPRASTERCAPS_SLOPESCALEDEPTHBIAS;
    c->ZCmpCaps = c->AlphaCmpCaps = 0xFF;
    c->SrcBlendCaps = c->DestBlendCaps = D3DPBLENDCAPS_ZERO | D3DPBLENDCAPS_ONE | D3DPBLENDCAPS_SRCCOLOR | D3DPBLENDCAPS_INVSRCCOLOR | D3DPBLENDCAPS_SRCALPHA | D3DPBLENDCAPS_INVSRCALPHA | D3DPBLENDCAPS_DESTALPHA | D3DPBLENDCAPS_INVDESTALPHA |
                                         D3DPBLENDCAPS_DESTCOLOR | D3DPBLENDCAPS_INVDESTCOLOR | D3DPBLENDCAPS_SRCALPHASAT | D3DPBLENDCAPS_BLENDFACTOR;
    c->ShadeCaps = D3DPSHADECAPS_COLORGOURAUDRGB | D3DPSHADECAPS_SPECULARGOURAUDRGB | D3DPSHADECAPS_ALPHAGOURAUDBLEND | D3DPSHADECAPS_FOGGOURAUD;
    c->TextureCaps = D3DPTEXTURECAPS_ALPHA | D3DPTEXTURECAPS_MIPMAP | D3DPTEXTURECAPS_CUBEMAP | D3DPTEXTURECAPS_MIPCUBEMAP | (vol ? D3DPTEXTURECAPS_VOLUMEMAP | D3DPTEXTURECAPS_MIPVOLUMEMAP : 0) | D3DPTEXTURECAPS_PROJECTED | D3DPTEXTURECAPS_PERSPECTIVE;      /* POW2 not set: the TMU wraps and mips any size */
    c->TextureFilterCaps = c->CubeTextureFilterCaps = D3DPTFILTERCAPS_MINFPOINT | D3DPTFILTERCAPS_MINFLINEAR | D3DPTFILTERCAPS_MAGFPOINT | D3DPTFILTERCAPS_MAGFLINEAR | D3DPTFILTERCAPS_MIPFPOINT | D3DPTFILTERCAPS_MIPFLINEAR;
    c->TextureAddressCaps = D3DPTADDRESSCAPS_WRAP | D3DPTADDRESSCAPS_MIRROR | D3DPTADDRESSCAPS_CLAMP | D3DPTADDRESSCAPS_BORDER | D3DPTADDRESSCAPS_MIRRORONCE | D3DPTADDRESSCAPS_INDEPENDENTUV;
    if (vol) { c->VolumeTextureFilterCaps = c->TextureFilterCaps; c->VolumeTextureAddressCaps = c->TextureAddressCaps; }
    c->LineCaps = D3DLINECAPS_TEXTURE | D3DLINECAPS_ZTEST | D3DLINECAPS_BLEND | D3DLINECAPS_ALPHACMP | D3DLINECAPS_FOG;
    c->MaxTextureWidth = c->MaxTextureHeight = MAX_TEX_SIZE;
    c->MaxVolumeExtent = vol ? G2R_VOL_D : 0;                   /* the same for all three: the TMU's depth stops at 255 */
    c->MaxTextureRepeat = 8192; c->MaxTextureAspectRatio = MAX_TEX_SIZE; c->MaxAnisotropy = 1; c->MaxVertexW = 1e10f;
    c->GuardBandLeft = c->GuardBandTop = -8192.0f; c->GuardBandRight = c->GuardBandBottom = 8192.0f;
    c->StencilCaps = D3DSTENCILCAPS_KEEP | D3DSTENCILCAPS_ZERO | D3DSTENCILCAPS_REPLACE | D3DSTENCILCAPS_INCRSAT | D3DSTENCILCAPS_DECRSAT | D3DSTENCILCAPS_INVERT | D3DSTENCILCAPS_INCR | D3DSTENCILCAPS_DECR | D3DSTENCILCAPS_TWOSIDED;
    c->FVFCaps = 8 | D3DFVFCAPS_PSIZE;
    c->TextureOpCaps = 0x03FFFFFF;                              /* every op */
    c->MaxTextureBlendStages = MAX_STAGES; c->MaxSimultaneousTextures = MAX_STAGES;
    c->VertexProcessingCaps = D3DVTXPCAPS_TEXGEN | D3DVTXPCAPS_MATERIALSOURCE7 | D3DVTXPCAPS_DIRECTIONALLIGHTS | D3DVTXPCAPS_POSITIONALLIGHTS | D3DVTXPCAPS_LOCALVIEWER | D3DVTXPCAPS_TWEENING | D3DVTXPCAPS_TEXGEN_SPHEREMAP;
    c->MaxActiveLights = MAX_LIGHTS; c->MaxUserClipPlanes = MAX_CLIP; c->MaxVertexBlendMatrices = 4; c->MaxVertexBlendMatrixIndex = MAX_WM - 1; c->MaxPointSize = MAX_POINT;
    c->MaxPrimitiveCount = 0xFFFFF; c->MaxVertexIndex = 0xFFFFF; c->MaxStreams = MAX_STREAMS; c->MaxStreamStride = 508;
    c->VertexShaderVersion = D3DVS_VERSION(2, 0); c->MaxVertexShaderConst = VS_CONSTS;
    c->PixelShaderVersion = D3DPS_VERSION(2, 0); c->PixelShader1xMaxValue = 8.0f;
    c->DevCaps2 = D3DDEVCAPS2_STREAMOFFSET;
    c->MaxNpatchTessellationLevel = 0; c->MasterAdapterOrdinal = 0; c->AdapterOrdinalInGroup = 0; c->NumberOfAdaptersInGroup = 1;
    c->DeclTypes = D3DDTCAPS_UBYTE4 | D3DDTCAPS_UBYTE4N | D3DDTCAPS_SHORT2N | D3DDTCAPS_SHORT4N | D3DDTCAPS_USHORT2N | D3DDTCAPS_USHORT4N;
    c->NumSimultaneousRTs = 1;
    c->StretchRectFilterCaps = D3DPTFILTERCAPS_MINFPOINT | D3DPTFILTERCAPS_MAGFPOINT;
    c->VS20Caps.Caps = D3DVS20CAPS_PREDICATION; c->VS20Caps.NumTemps = 32; c->VS20Caps.StaticFlowControlDepth = 4; c->VS20Caps.DynamicFlowControlDepth = 24;
    c->PS20Caps.Caps = D3DPS20CAPS_ARBITRARYSWIZZLE | D3DPS20CAPS_GRADIENTINSTRUCTIONS | D3DPS20CAPS_PREDICATION | D3DPS20CAPS_NODEPENDENTREADLIMIT | D3DPS20CAPS_NOTEXINSTRUCTIONLIMIT; c->PS20Caps.NumTemps = 32; c->PS20Caps.NumInstructionSlots = 512; c->PS20Caps.StaticFlowControlDepth = 4; c->PS20Caps.DynamicFlowControlDepth = 24;
    c->VertexTextureFilterCaps = 0; c->MaxVShaderInstructionsExecuted = 65535; c->MaxPShaderInstructionsExecuted = 65535;
    c->MaxVertexShader30InstructionSlots = 0; c->MaxPixelShader30InstructionSlots = 0;
}

static HRESULT WINAPI o_GetDeviceCaps(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DCAPS9 *c)
{
    (void)t;
    if (a || ty != D3DDEVTYPE_HAL) return D3DERR_NOTAVAILABLE;
    d3_caps(c);
    return D3D_OK;
}
static HMONITOR WINAPI o_GetAdapterMonitor(IDirect3D9 *t, UINT a) { POINT p = { 0, 0 }; (void)t; (void)a; return MonitorFromPoint(p, MONITOR_DEFAULTTOPRIMARY); }
static HRESULT WINAPI o_CreateDevice(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, HWND focus, DWORD fl, D3DPRESENT_PARAMETERS *pp, IDirect3DDevice9 **o)
{
    g2e_log("d3d9: CreateDevice type %d, %u x %u, format %d, depth %d, windowed %d, multisample %d, swap effect %d", (int)ty, pp->BackBufferWidth, pp->BackBufferHeight, (int)pp->BackBufferFormat, (int)pp->AutoDepthStencilFormat, pp->Windowed, (int)pp->MultiSampleType, (int)pp->SwapEffect);
    if (a || ty != D3DDEVTYPE_HAL) return D3DERR_NOTAVAILABLE;
    if (pp->MultiSampleType != D3DMULTISAMPLE_NONE) return D3DERR_NOTAVAILABLE;
    return d3_device_new(t, focus, fl, pp, o);
}

const IDirect3D9Vtbl d3_d3d_vtbl = {
    o_QueryInterface, o_AddRef, o_Release, o_RegisterSoftwareDevice, o_GetAdapterCount, o_GetAdapterIdentifier, o_GetAdapterModeCount, o_EnumAdapterModes, o_GetAdapterDisplayMode,
    o_CheckDeviceType, o_CheckDeviceFormat, o_CheckDeviceMultiSampleType, o_CheckDepthStencilMatch, o_CheckDeviceFormatConversion, o_GetDeviceCaps, o_GetAdapterMonitor, o_CreateDevice
};

IDirect3D9 *WINAPI Direct3DCreate9(UINT sdk)
{
    d3obj *o = (d3obj *)calloc(1, sizeof *o);
    if (!o) return NULL;
    g2e_log("d3d9: Direct3DCreate9(%u)", sdk);
    o->vtbl = &d3_d3d_vtbl; o->refs = 1;
    return (IDirect3D9 *)o;
}

/* D3DPERF_* markers: the debug runtime's, nothing here uses them */
int WINAPI D3DPERF_BeginEvent(DWORD c, const WCHAR *n) { (void)c; (void)n; return 0; }
int WINAPI D3DPERF_EndEvent(void) { return 0; }
void WINAPI D3DPERF_SetMarker(DWORD c, const WCHAR *n) { (void)c; (void)n; }
void WINAPI D3DPERF_SetRegion(DWORD c, const WCHAR *n) { (void)c; (void)n; }
BOOL WINAPI D3DPERF_QueryRepeatFrame(void) { return FALSE; }
void WINAPI D3DPERF_SetOptions(DWORD o) { (void)o; }
DWORD WINAPI D3DPERF_GetStatus(void) { return 0; }

BOOL WINAPI DllMain(HINSTANCE h, DWORD why, LPVOID r)
{
    (void)r;
    if (why == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(h);
        SetProcessDPIAware();
    }
    return TRUE;
}
