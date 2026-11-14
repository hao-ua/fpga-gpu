/* d3d8_main.c — d3d8.dll: Direct3D 8 on the code of d3d9.dll, 32-bit as Direct3D 8 is.
 * The objects are d3d9.dll's behind Direct3D 8's method tables: a game's IDirect3DSurface8 * is the d3surf, so identity and references need no wrappers and a method that means the same runs the same code.
 * This file holds what differs: structures that changed, the states Direct3D 9 moved to samplers, the base vertex that came with SetIndices, vertex shaders with their declaration, handles.
 * Handles: state blocks and pixel shaders are pointers (x86), a vertex shader handle is a pointer with bit 0 set, which no FVF has. */
#include "d3d9_internal.h"
#include "d3d8_slots.h"

#define D9 (&d3_dev_vtbl)
#define DEV(t) ((d3dev *)(t))
typedef IDirect3DDevice9 dev9;

/* the structures that changed: d3d8_check.c compares their sizes with the SDK's */
typedef struct pp8 {
    UINT BackBufferWidth, BackBufferHeight;
    D3DFORMAT BackBufferFormat;
    UINT BackBufferCount;
    D3DMULTISAMPLE_TYPE MultiSampleType;
    DWORD SwapEffect;
    HWND hDeviceWindow;
    BOOL Windowed, EnableAutoDepthStencil;
    D3DFORMAT AutoDepthStencilFormat;
    DWORD Flags;
    UINT FullScreen_RefreshRateInHz, FullScreen_PresentationInterval;
} pp8;
typedef struct desc8 { D3DFORMAT Format; D3DRESOURCETYPE Type; DWORD Usage; D3DPOOL Pool; UINT Size; D3DMULTISAMPLE_TYPE MultiSampleType; UINT Width, Height; } desc8;
typedef struct vdesc8 { D3DFORMAT Format; D3DRESOURCETYPE Type; DWORD Usage; D3DPOOL Pool; UINT Size, Width, Height, Depth; } vdesc8;
#pragma pack(push, 4)                                   /* as the SDK's headers: 8-byte members on 4 */
typedef struct id8 { char Driver[512], Description[512]; LARGE_INTEGER DriverVersion; DWORD VendorId, DeviceId, SubSysId, Revision; GUID DeviceIdentifier; DWORD WHQLLevel; } id8;
#pragma pack(pop)
_Static_assert(sizeof(pp8) == D8_SIZE_PP && sizeof(desc8) == D8_SIZE_DESC && sizeof(vdesc8) == D8_SIZE_VDESC && sizeof(id8) == D8_SIZE_ID, "d3d8 structures");
_Static_assert(offsetof(D3DCAPS9, DevCaps2) == D8_SIZE_CAPS, "D3DCAPS9 begins with D3DCAPS8");

#define SWAP_COPY_VSYNC 4                               /* Direct3D 8 only */
#define RS_ZBIAS 47
#define PRASTER_ZBIAS 0x00004000u
#define CAPS2_CANRENDERWINDOWED 0x00080000u
#define MODES_PER_FORMAT 2

static const GUID G_D3D = { 0x1DD9E8DA, 0x1C77, 0x4D40, { 0xB0, 0xCF, 0x98, 0xFE, 0xFD, 0xFF, 0x95, 0x12 } };
static const GUID G_Device = { 0x7385E5DF, 0x8FE8, 0x41D5, { 0x86, 0xB6, 0xD7, 0xB4, 0x85, 0x47, 0xB6, 0xCF } };
static const GUID G_Resource = { 0x1B36BB7B, 0x09B7, 0x410A, { 0xB4, 0x45, 0x7D, 0x14, 0x30, 0xD7, 0xB3, 0x3F } };
static const GUID G_VB = { 0x8AEEEAC7, 0x05F9, 0x44D4, { 0xB5, 0x91, 0x00, 0x0B, 0x0D, 0xF1, 0xCB, 0x95 } };
static const GUID G_IB = { 0x0E689C9A, 0x053D, 0x44A0, { 0x9D, 0x92, 0xDB, 0x0E, 0x3D, 0x75, 0x0F, 0x86 } };
static const GUID G_Surface = { 0xB96EEBCA, 0xB326, 0x4EA5, { 0x88, 0x2F, 0x2F, 0xF5, 0xBA, 0xE0, 0x21, 0xDD } };
static const GUID G_Base = { 0xB4211CFA, 0x51B9, 0x4A9F, { 0xAB, 0x78, 0xDB, 0x99, 0xB2, 0xBB, 0x67, 0x8E } };
static const GUID G_Texture = { 0xE4CDD575, 0x2866, 0x4F01, { 0xB1, 0x2E, 0x7E, 0xEC, 0xE1, 0xEC, 0x93, 0x58 } };
static const GUID G_Volume = { 0xBD7349F5, 0x14F1, 0x42E4, { 0x9C, 0x79, 0x97, 0x23, 0x80, 0xDB, 0x40, 0xC0 } };
static const GUID G_VolumeTex = { 0x4B8AAAFA, 0x140F, 0x42BA, { 0x91, 0x31, 0x59, 0x7E, 0xAF, 0xAA, 0x2E, 0xAD } };
static const GUID G_Cube = { 0x3EE5B968, 0x2ACA, 0x4C34, { 0x8B, 0xB5, 0x7E, 0x0C, 0x3D, 0x19, 0xB7, 0x50 } };
static const GUID G_Chain = { 0x928C088B, 0x76B9, 0x4C6B, { 0xA5, 0x36, 0xA5, 0x90, 0x85, 0x38, 0x76, 0xCD } };

static HRESULT qi8(void *self, REFIID id, const GUID *a, const GUID *b, const GUID *c, void **o)
{
    if (!o) return E_POINTER;
    if (IsEqualGUID(id, &IID_IUnknown) || IsEqualGUID(id, a) || (b && IsEqualGUID(id, b)) || (c && IsEqualGUID(id, c))) { ((IUnknown *)self)->lpVtbl->AddRef((IUnknown *)self); *o = self; return S_OK; }
    *o = NULL;
    return E_NOINTERFACE;
}

/* ---------------------------------------------------------------- present parameters, caps */
static void pp_to9(const pp8 *a, D3DPRESENT_PARAMETERS *b)
{
    memset(b, 0, sizeof *b);
    b->BackBufferWidth = a->BackBufferWidth; b->BackBufferHeight = a->BackBufferHeight; b->BackBufferFormat = a->BackBufferFormat; b->BackBufferCount = a->BackBufferCount;
    b->MultiSampleType = a->MultiSampleType;
    b->SwapEffect = a->SwapEffect == SWAP_COPY_VSYNC ? D3DSWAPEFFECT_COPY : (D3DSWAPEFFECT)a->SwapEffect;
    b->hDeviceWindow = a->hDeviceWindow; b->Windowed = a->Windowed; b->EnableAutoDepthStencil = a->EnableAutoDepthStencil; b->AutoDepthStencilFormat = a->AutoDepthStencilFormat; b->Flags = a->Flags;
    b->FullScreen_RefreshRateInHz = a->FullScreen_RefreshRateInHz;
    b->PresentationInterval = a->FullScreen_PresentationInterval;
    if (a->Windowed) b->PresentationInterval = a->SwapEffect == SWAP_COPY_VSYNC ? D3DPRESENT_INTERVAL_ONE : D3DPRESENT_INTERVAL_IMMEDIATE;      /* Direct3D 8 waits in a window only for COPY_VSYNC */
}
static void pp_back(const D3DPRESENT_PARAMETERS *b, pp8 *a) { a->BackBufferWidth = b->BackBufferWidth; a->BackBufferHeight = b->BackBufferHeight; a->BackBufferFormat = b->BackBufferFormat; a->BackBufferCount = b->BackBufferCount; }

/* D3DCAPS9 begins with D3DCAPS8. Shaders: what d3d9_sm.c translates of Direct3D 8's versions */
static void caps8(void *o)
{
    D3DCAPS9 c;
    d3_caps(&c);
    c.Caps2 |= CAPS2_CANRENDERWINDOWED;
    c.RasterCaps = (c.RasterCaps & ~(DWORD)(D3DPRASTERCAPS_DEPTHBIAS | D3DPRASTERCAPS_SLOPESCALEDEPTHBIAS | D3DPRASTERCAPS_SCISSORTEST)) | PRASTER_ZBIAS;
    c.VertexShaderVersion = D3DVS_VERSION(1, 1); c.MaxVertexShaderConst = 96;
    c.PixelShaderVersion = D3DPS_VERSION(1, 1);
    memcpy(o, &c, D8_SIZE_CAPS);
}

/* ---------------------------------------------------------------- IDirect3D8 */
static HRESULT WINAPI o_QueryInterface(IDirect3D9 *t, REFIID id, void **o) { return qi8(t, id, &G_D3D, NULL, NULL, o); }
static HRESULT WINAPI o_GetAdapterIdentifier(IDirect3D9 *t, UINT a, DWORD f, id8 *o)
{
    D3DADAPTER_IDENTIFIER9 i;
    HRESULT r = d3_d3d_vtbl.GetAdapterIdentifier(t, a, f, &i);
    if (r != D3D_OK) return r;
    memset(o, 0, sizeof *o);
    strcpy(o->Driver, "d3d8.dll");
    strcpy(o->Description, i.Description);
    o->VendorId = i.VendorId; o->DeviceId = i.DeviceId; o->WHQLLevel = i.WHQLLevel;
    return D3D_OK;
}
/* Direct3D 8 enumerates sizes and formats together */
static UINT WINAPI o_GetAdapterModeCount(IDirect3D9 *t, UINT a) { return d3_d3d_vtbl.GetAdapterModeCount(t, a, D3DFMT_X8R8G8B8) * MODES_PER_FORMAT; }
static HRESULT WINAPI o_EnumAdapterModes(IDirect3D9 *t, UINT a, UINT i, D3DDISPLAYMODE *m) { return d3_d3d_vtbl.EnumAdapterModes(t, a, i % MODES_PER_FORMAT ? D3DFMT_R5G6B5 : D3DFMT_X8R8G8B8, i / MODES_PER_FORMAT, m); }
static HRESULT WINAPI o_CheckDeviceMultiSampleType(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, D3DFORMAT f, BOOL w, D3DMULTISAMPLE_TYPE ms) { return d3_d3d_vtbl.CheckDeviceMultiSampleType(t, a, ty, f, w, ms, NULL); }
static HRESULT WINAPI o_GetDeviceCaps(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, void *c)
{
    D3DCAPS9 x;
    HRESULT r = d3_d3d_vtbl.GetDeviceCaps(t, a, ty, &x);
    if (r == D3D_OK) caps8(c);
    return r;
}
static HRESULT WINAPI o_CreateDevice(IDirect3D9 *t, UINT a, D3DDEVTYPE ty, HWND focus, DWORD fl, pp8 *p, IDirect3DDevice9 **o)
{
    D3DPRESENT_PARAMETERS q;
    HRESULT r;
    pp_to9(p, &q);
    r = d3_d3d_vtbl.CreateDevice(t, a, ty, focus, fl, &q, o);
    if (r == D3D_OK) pp_back(&q, p);
    return r;
}

/* ---------------------------------------------------------------- the device: what is not d3d9.dll's method itself */
static HRESULT WINAPI d_QueryInterface(dev9 *t, REFIID id, void **o) { return qi8(t, id, &G_Device, NULL, NULL, o); }
static HRESULT WINAPI d_ResourceManagerDiscardBytes(dev9 *t, DWORD n) { (void)n; return D9->EvictManagedResources(t); }
static HRESULT WINAPI d_GetDeviceCaps(dev9 *t, void *c) { (void)t; caps8(c); return D3D_OK; }
static HRESULT WINAPI d_GetDisplayMode(dev9 *t, D3DDISPLAYMODE *m) { return D9->GetDisplayMode(t, 0, m); }
static HRESULT WINAPI d_CreateAdditionalSwapChain(dev9 *t, pp8 *p, IDirect3DSwapChain9 **o)
{
    D3DPRESENT_PARAMETERS q;
    HRESULT r;
    pp_to9(p, &q);
    r = D9->CreateAdditionalSwapChain(t, &q, o);
    if (r == D3D_OK) pp_back(&q, p);
    return r;
}
static HRESULT WINAPI d_Reset(dev9 *t, pp8 *p)
{
    D3DPRESENT_PARAMETERS q;
    HRESULT r;
    pp_to9(p, &q);
    r = D9->Reset(t, &q);
    if (r == D3D_OK) pp_back(&q, p);
    return r;
}
static HRESULT WINAPI d_GetBackBuffer(dev9 *t, UINT i, D3DBACKBUFFER_TYPE ty, IDirect3DSurface9 **o) { return D9->GetBackBuffer(t, 0, i, ty, o); }
static HRESULT WINAPI d_GetRasterStatus(dev9 *t, D3DRASTER_STATUS *o) { return D9->GetRasterStatus(t, 0, o); }
static void WINAPI d_SetGammaRamp(dev9 *t, DWORD f, const D3DGAMMARAMP *r) { D9->SetGammaRamp(t, 0, f, r); }
static void WINAPI d_GetGammaRamp(dev9 *t, D3DGAMMARAMP *r) { D9->GetGammaRamp(t, 0, r); }
static HRESULT WINAPI d_CreateTexture(dev9 *t, UINT w, UINT h, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DTexture9 **o) { return D9->CreateTexture(t, w, h, l, u, f, p, o, NULL); }
static HRESULT WINAPI d_CreateVolumeTexture(dev9 *t, UINT w, UINT h, UINT de, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DVolumeTexture9 **o) { return D9->CreateVolumeTexture(t, w, h, de, l, u, f, p, o, NULL); }
static HRESULT WINAPI d_CreateCubeTexture(dev9 *t, UINT e, UINT l, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DCubeTexture9 **o) { return D9->CreateCubeTexture(t, e, l, u, f, p, o, NULL); }
static HRESULT WINAPI d_CreateVertexBuffer(dev9 *t, UINT n, DWORD u, DWORD fvf, D3DPOOL p, IDirect3DVertexBuffer9 **o) { return D9->CreateVertexBuffer(t, n, u, fvf, p, o, NULL); }
static HRESULT WINAPI d_CreateIndexBuffer(dev9 *t, UINT n, DWORD u, D3DFORMAT f, D3DPOOL p, IDirect3DIndexBuffer9 **o) { return D9->CreateIndexBuffer(t, n, u, f, p, o, NULL); }
static HRESULT WINAPI d_CreateRenderTarget(dev9 *t, UINT w, UINT h, D3DFORMAT f, D3DMULTISAMPLE_TYPE ms, BOOL lk, IDirect3DSurface9 **o) { return D9->CreateRenderTarget(t, w, h, f, ms, 0, lk, o, NULL); }
static HRESULT WINAPI d_CreateDepthStencilSurface(dev9 *t, UINT w, UINT h, D3DFORMAT f, D3DMULTISAMPLE_TYPE ms, IDirect3DSurface9 **o) { return D9->CreateDepthStencilSurface(t, w, h, f, ms, 0, FALSE, o, NULL); }
static HRESULT WINAPI d_CreateImageSurface(dev9 *t, UINT w, UINT h, D3DFORMAT f, IDirect3DSurface9 **o)
{
    uint32_t g;
    if (!d3_fmt_g2(f, &g)) return D3DERR_INVALIDCALL;
    return D9->CreateOffscreenPlainSurface(t, w, h, f, D3DPOOL_SYSTEMMEM, o, NULL);
}

/* a rectangle between any two surfaces: on the board with the board's copy, in memory by the shadow, across the two by lock */
static int on_board(const d3surf *s) { return s->back || (s->usage & D3DUSAGE_RENDERTARGET); }
static HRESULT copy_rect(dev9 *t, d3surf *s, const RECT *r, d3surf *o, const POINT *p)
{
    LONG w = r->right - r->left, h = r->bottom - r->top, y;
    RECT rd;
    D3DLOCKED_RECT a, b;
    int bytes = d3_fmt_bytes(s->fmt);
    SetRect(&rd, p->x, p->y, p->x + w, p->y + h);
    if (on_board(s) && on_board(o)) return D9->StretchRect(t, (IDirect3DSurface9 *)s, r, (IDirect3DSurface9 *)o, &rd, D3DTEXF_NONE);
    if (!on_board(s) && !on_board(o)) return D9->UpdateSurface(t, (IDirect3DSurface9 *)s, r, (IDirect3DSurface9 *)o, p);
    if (!bytes) return D3DERR_INVALIDCALL;
    if (d3_surface_vtbl.LockRect((IDirect3DSurface9 *)s, &a, r, D3DLOCK_READONLY) != D3D_OK) return D3DERR_INVALIDCALL;
    if (d3_surface_vtbl.LockRect((IDirect3DSurface9 *)o, &b, &rd, 0) != D3D_OK) { d3_surface_vtbl.UnlockRect((IDirect3DSurface9 *)s); return D3DERR_INVALIDCALL; }
    for (y = 0; y < h; y++) memcpy((uint8_t *)b.pBits + (ptrdiff_t)y * b.Pitch, (const uint8_t *)a.pBits + (ptrdiff_t)y * a.Pitch, (size_t)w * bytes);
    d3_surface_vtbl.UnlockRect((IDirect3DSurface9 *)o);
    d3_surface_vtbl.UnlockRect((IDirect3DSurface9 *)s);
    return D3D_OK;
}
static HRESULT WINAPI d_CopyRects(dev9 *t, d3surf *s, const RECT *sr, UINT n, d3surf *o, const POINT *dp)
{
    RECT all;
    POINT zero = { 0, 0 };
    UINT i;
    if (s->fmt != o->fmt) return D3DERR_INVALIDCALL;
    if (!sr || !n) {                                            /* the whole surface, at the first point if there is one */
        SetRect(&all, 0, 0, (int)s->w, (int)s->h);
        return copy_rect(t, s, &all, o, dp ? dp : &zero);
    }
    for (i = 0; i < n; i++) {
        POINT p;
        HRESULT r;
        p.x = dp ? dp[i].x : sr[i].left; p.y = dp ? dp[i].y : sr[i].top;
        r = copy_rect(t, s, &sr[i], o, &p);
        if (r != D3D_OK) return r;
    }
    return D3D_OK;
}
static HRESULT WINAPI d_GetFrontBuffer(dev9 *t, d3surf *o) { return d3_front_read(DEV(t), o); }
/* a NULL target keeps the target, a NULL depth surface takes the depth surface away */
static HRESULT WINAPI d_SetRenderTarget(dev9 *t, IDirect3DSurface9 *rt, IDirect3DSurface9 *ds)
{
    HRESULT r = rt ? D9->SetRenderTarget(t, 0, rt) : D3D_OK;
    return r == D3D_OK ? D9->SetDepthStencilSurface(t, ds) : r;
}
static HRESULT WINAPI d_GetRenderTarget(dev9 *t, IDirect3DSurface9 **o) { return D9->GetRenderTarget(t, 0, o); }

/* ZBIAS 0 .. 16 pulls toward the viewer: one unit of a 16-bit depth buffer per step, so it does not round away there */
static HRESULT WINAPI d_SetRenderState(dev9 *t, D3DRENDERSTATETYPE s, DWORD v)
{
    HRESULT r = D9->SetRenderState(t, s, v);
    if (s == RS_ZBIAS) {
        float b = -(float)v / 65536.0f;
        DWORD w;
        memcpy(&w, &b, 4);
        D9->SetRenderState(t, D3DRS_DEPTHBIAS, w);
    }
    return r;
}
static HRESULT WINAPI d_ApplyStateBlock(dev9 *t, DWORD b) { (void)t; if (!b) return D3DERR_INVALIDCALL; d3_block_apply((d3block *)b); return D3D_OK; }
static HRESULT WINAPI d_CaptureStateBlock(dev9 *t, DWORD b) { (void)t; if (!b) return D3DERR_INVALIDCALL; d3_block_capture((d3block *)b); return D3D_OK; }
static HRESULT WINAPI d_DeleteStateBlock(dev9 *t, DWORD b) { (void)t; if (!b) return D3DERR_INVALIDCALL; IDirect3DStateBlock9_Release((IDirect3DStateBlock9 *)b); return D3D_OK; }

/* what Direct3D 9 moved from the stages to the samplers, in Direct3D 8's numbers */
static int tss_sampler(DWORD ty)
{
    switch (ty) {
    case 13: return D3DSAMP_ADDRESSU;
    case 14: return D3DSAMP_ADDRESSV;
    case 15: return D3DSAMP_BORDERCOLOR;
    case 16: return D3DSAMP_MAGFILTER;
    case 17: return D3DSAMP_MINFILTER;
    case 18: return D3DSAMP_MIPFILTER;
    case 19: return D3DSAMP_MIPMAPLODBIAS;
    case 20: return D3DSAMP_MAXMIPLEVEL;
    case 21: return D3DSAMP_MAXANISOTROPY;
    case 25: return D3DSAMP_ADDRESSW;
    default: return 0;
    }
}
static HRESULT WINAPI d_GetTextureStageState(dev9 *t, DWORD st, DWORD ty, DWORD *v) { int s = tss_sampler(ty); return s ? D9->GetSamplerState(t, st, (D3DSAMPLERSTATETYPE)s, v) : D9->GetTextureStageState(t, st, (D3DTEXTURESTAGESTATETYPE)ty, v); }
static HRESULT WINAPI d_SetTextureStageState(dev9 *t, DWORD st, DWORD ty, DWORD v) { int s = tss_sampler(ty); return s ? D9->SetSamplerState(t, st, (D3DSAMPLERSTATETYPE)s, v) : D9->SetTextureStageState(t, st, (D3DTEXTURESTAGESTATETYPE)ty, v); }
static HRESULT WINAPI d_GetInfo(dev9 *t, DWORD id, void *o, DWORD n) { (void)t; (void)id; (void)o; (void)n; return S_FALSE; }      /* none of the optional statistics */

static HRESULT WINAPI d_DrawIndexedPrimitive(dev9 *t, D3DPRIMITIVETYPE p, UINT mi, UINT nv, UINT start, UINT c) { return D9->DrawIndexedPrimitive(t, p, DEV(t)->st.ib_base, mi, nv, start, c); }
static HRESULT WINAPI d_ProcessVertices(dev9 *t, UINT a, UINT b, UINT c, IDirect3DVertexBuffer9 *o, DWORD f) { return D9->ProcessVertices(t, a, b, c, o, NULL, f); }
static HRESULT WINAPI d_SetStreamSource(dev9 *t, UINT i, IDirect3DVertexBuffer9 *b, UINT stride) { return D9->SetStreamSource(t, i, b, 0, stride); }
static HRESULT WINAPI d_GetStreamSource(dev9 *t, UINT i, IDirect3DVertexBuffer9 **b, UINT *stride) { UINT off; return D9->GetStreamSource(t, i, b, &off, stride); }
static HRESULT WINAPI d_SetIndices(dev9 *t, IDirect3DIndexBuffer9 *b, UINT base) { HRESULT r = D9->SetIndices(t, b); if (r == D3D_OK) DEV(t)->st.ib_base = (int)base; return r; }
static HRESULT WINAPI d_GetIndices(dev9 *t, IDirect3DIndexBuffer9 **b, UINT *base) { *base = (UINT)DEV(t)->st.ib_base; return D9->GetIndices(t, b); }

/* ---------------------------------------------------------------- vertex shaders */
typedef struct vs8 {
    IDirect3DVertexShader9 *vs;                 /* NULL: a declaration for the fixed function */
    IDirect3DVertexDeclaration9 *decl;
    DWORD *dtok, *ftok;                         /* as the application gave them */
    UINT nd, nf;
} vs8;

/* the fixed function's meaning of the registers, usable by shaders too: nothing else names an input in Direct3D 8 */
static const struct { BYTE usage, index; } REG8[17] = {
    { D3DDECLUSAGE_POSITION, 0 }, { D3DDECLUSAGE_BLENDWEIGHT, 0 }, { D3DDECLUSAGE_BLENDINDICES, 0 }, { D3DDECLUSAGE_NORMAL, 0 }, { D3DDECLUSAGE_PSIZE, 0 }, { D3DDECLUSAGE_COLOR, 0 }, { D3DDECLUSAGE_COLOR, 1 },
    { D3DDECLUSAGE_TEXCOORD, 0 }, { D3DDECLUSAGE_TEXCOORD, 1 }, { D3DDECLUSAGE_TEXCOORD, 2 }, { D3DDECLUSAGE_TEXCOORD, 3 }, { D3DDECLUSAGE_TEXCOORD, 4 }, { D3DDECLUSAGE_TEXCOORD, 5 }, { D3DDECLUSAGE_TEXCOORD, 6 }, { D3DDECLUSAGE_TEXCOORD, 7 },
    { D3DDECLUSAGE_POSITION, 1 }, { D3DDECLUSAGE_NORMAL, 1 }
};
#define VSD_STREAM 1
#define VSD_STREAMDATA 2
#define VSD_CONSTMEM 4
#define VSD_EXT 5
#define VSD_END 0xFFFFFFFFu
#define VSD_SKIP (1u << 28)
#define VSD_TESS_STREAM (1u << 28)

/* the declaration's tokens as elements, ends with Direct3D 9's end element. regs[] gets each register it loads */
static int decl_parse(const DWORD *p, D3DVERTEXELEMENT9 *e, int *regs, UINT *tokens)
{
    static const BYTE BYTES[8] = { 4, 8, 12, 16, 4, 4, 4, 8 };
    static const D3DVERTEXELEMENT9 end = D3DDECL_END();
    const DWORD *s = p;
    int n = 0, off = 0, stream = 0;
    for (; *p != VSD_END; p++) {
        DWORD t = *p, ty = t >> 29;
        if (ty == VSD_STREAM) { if (!(t & VSD_TESS_STREAM)) { stream = (int)(t & 15); off = 0; } }
        else if (ty == VSD_STREAMDATA && (t & VSD_SKIP)) off += 4 * (int)(t >> 16 & 15);
        else if (ty == VSD_STREAMDATA) {
            int reg = (int)(t & 31), type = (int)(t >> 16 & 7);
            if (reg > 16 || n >= 32) return -1;
            e[n].Stream = (WORD)stream; e[n].Offset = (WORD)off; e[n].Type = (BYTE)type; e[n].Method = D3DDECLMETHOD_DEFAULT; e[n].Usage = REG8[reg].usage; e[n].UsageIndex = REG8[reg].index;
            regs[n++] = reg;
            off += BYTES[type];
        }
        else if (ty == VSD_CONSTMEM) p += 4 * (t >> 25 & 15);
        else if (ty == VSD_EXT) p += t >> 24 & 31;
    }
    e[n] = end;
    *tokens = (UINT)(p - s) + 1;
    return n;
}

/* D3DVSD_CONST: registers loaded when the shader is set */
static void decl_constants(dev9 *t, const DWORD *p)
{
    for (; *p != VSD_END; p++) {
        DWORD c = *p, ty = c >> 29;
        if (ty == VSD_CONSTMEM) {
            UINT n = c >> 25 & 15;
            D9->SetVertexShaderConstantF(t, c & 127, (const float *)(p + 1), n);
            p += 4 * n;
        } else if (ty == VSD_EXT) p += c >> 24 & 31;
    }
}

static UINT shader_tokens(const DWORD *f)
{
    UINT n = 1;
    while (f[n] != 0x0000FFFF) {
        if ((f[n] & 0xFFFF) == 0xFFFE) n += f[n] >> 16;
        n++;
    }
    return n + 1;
}

/* Direct3D 9's vs 1.x says what its inputs are in dcl instructions, Direct3D 8's in the declaration */
static DWORD *with_dcl(const DWORD *f, UINT nf, const int *regs, int n)
{
    DWORD *o = (DWORD *)malloc(((size_t)nf + 3 * (size_t)n) * 4), *q = o;
    int i;
    if (!o) return NULL;
    *q++ = f[0];
    for (i = 0; i < n; i++) {
        *q++ = 31;                                              /* dcl */
        *q++ = 0x80000000u | REG8[regs[i]].usage | (DWORD)REG8[regs[i]].index << 16;
        *q++ = 0x80000000u | 1u << 28 | 15u << 16 | (DWORD)regs[i];     /* v<n>, all components */
    }
    memcpy(q, f + 1, ((size_t)nf - 1) * 4);
    return o;
}

static void vs8_free(vs8 *v)
{
    if (v->vs) IDirect3DVertexShader9_Release(v->vs);
    if (v->decl) IDirect3DVertexDeclaration9_Release(v->decl);
    free(v->dtok); free(v->ftok); free(v);
}

static HRESULT WINAPI d_CreateVertexShader(dev9 *t, const DWORD *decl, const DWORD *func, DWORD *handle, DWORD usage)
{
    D3DVERTEXELEMENT9 e[33];
    int regs[32], n;
    vs8 *v = (vs8 *)calloc(1, sizeof *v);
    HRESULT r;
    (void)usage;
    if (!v) return E_OUTOFMEMORY;
    n = decl_parse(decl, e, regs, &v->nd);
    if (n < 0) { g2e_log("d3d8: a vertex declaration this library does not take"); free(v); return D3DERR_INVALIDCALL; }
    v->dtok = (DWORD *)malloc((size_t)v->nd * 4);
    if (!v->dtok) { free(v); return E_OUTOFMEMORY; }
    memcpy(v->dtok, decl, (size_t)v->nd * 4);
    r = D9->CreateVertexDeclaration(t, e, &v->decl);
    if (r == D3D_OK && func) {
        DWORD *f9;
        v->nf = shader_tokens(func);
        v->ftok = (DWORD *)malloc((size_t)v->nf * 4);
        f9 = with_dcl(func, v->nf, regs, n);
        if (!v->ftok || !f9) r = E_OUTOFMEMORY;
        else { memcpy(v->ftok, func, (size_t)v->nf * 4); r = D9->CreateVertexShader(t, f9, &v->vs); }
        free(f9);
    }
    if (r != D3D_OK) { vs8_free(v); return r; }
    *handle = (DWORD)v | 1;
    ((d3decl *)v->decl)->shader8 = *handle;
    D3T("d3d8: CreateVertexShader %d inputs, %u function tokens", n, v->nf);
    return D3D_OK;
}
#define VS(h) ((vs8 *)((h) & ~(DWORD)1))
static HRESULT WINAPI d_SetVertexShader(dev9 *t, DWORD h)
{
    if (h & 1) {
        vs8 *v = VS(h);
        D9->SetVertexShader(t, v->vs);
        D9->SetVertexDeclaration(t, v->decl);
        decl_constants(t, v->dtok);
        return D3D_OK;
    }
    D9->SetVertexShader(t, NULL);
    return D9->SetFVF(t, h);
}
static HRESULT WINAPI d_GetVertexShader(dev9 *t, DWORD *h) { const d3state *s = &DEV(t)->st; *h = s->fvf ? s->fvf : s->decl ? s->decl->shader8 : 0; return D3D_OK; }
static HRESULT WINAPI d_DeleteVertexShader(dev9 *t, DWORD h) { (void)t; if (!(h & 1)) return D3DERR_INVALIDCALL; vs8_free(VS(h)); return D3D_OK; }
static HRESULT give_tokens(const DWORD *tok, UINT n, void *o, DWORD *size)
{
    DWORD bytes = n * 4;
    if (!o) { *size = bytes; return D3D_OK; }
    if (*size < bytes) { *size = bytes; return D3DERR_MOREDATA; }
    memcpy(o, tok, bytes);
    *size = bytes;
    return D3D_OK;
}
static HRESULT WINAPI d_GetVertexShaderDeclaration(dev9 *t, DWORD h, void *o, DWORD *size) { (void)t; if (!(h & 1)) return D3DERR_INVALIDCALL; return give_tokens(VS(h)->dtok, VS(h)->nd, o, size); }
static HRESULT WINAPI d_GetVertexShaderFunction(dev9 *t, DWORD h, void *o, DWORD *size) { (void)t; if (!(h & 1)) return D3DERR_INVALIDCALL; return give_tokens(VS(h)->ftok, VS(h)->nf, o, size); }

/* ---------------------------------------------------------------- pixel shaders: a handle takes no reference */
static HRESULT WINAPI d_GetPixelShader(dev9 *t, DWORD *h) { *h = (DWORD)DEV(t)->st.ps; return D3D_OK; }
static HRESULT WINAPI d_DeletePixelShader(dev9 *t, DWORD h) { (void)t; if (!h) return D3DERR_INVALIDCALL; IDirect3DPixelShader9_Release((IDirect3DPixelShader9 *)h); return D3D_OK; }
static HRESULT WINAPI d_GetPixelShaderFunction(dev9 *t, DWORD h, void *o, DWORD *size) { const d3shader *s = (const d3shader *)h; (void)t; if (!h) return D3DERR_INVALIDCALL; return give_tokens(s->tok, s->ntok, o, size); }

/* ---------------------------------------------------------------- resources */
static HRESULT WINAPI s_QueryInterface(IDirect3DSurface9 *t, REFIID id, void **o) { return qi8(t, id, &G_Surface, &G_Resource, NULL, o); }
static HRESULT WINAPI s_GetDesc(IDirect3DSurface9 *t, desc8 *o)
{
    D3DSURFACE_DESC d;
    d3_surface_vtbl.GetDesc(t, &d);
    o->Format = d.Format; o->Type = d.Type; o->Usage = d.Usage; o->Pool = d.Pool; o->MultiSampleType = d.MultiSampleType; o->Width = d.Width; o->Height = d.Height;
    o->Size = (UINT)d3_shadow_bytes((const d3surf *)t);
    return D3D_OK;
}
static HRESULT WINAPI t_QueryInterface(IDirect3DTexture9 *t, REFIID id, void **o) { return qi8(t, id, &G_Texture, &G_Base, &G_Resource, o); }
static HRESULT WINAPI t_GetLevelDesc(d3tex *t, UINT l, desc8 *o) { if ((int)l >= t->levels) return D3DERR_INVALIDCALL; return s_GetDesc((IDirect3DSurface9 *)t->surf[l], o); }
static HRESULT WINAPI c_QueryInterface(IDirect3DCubeTexture9 *t, REFIID id, void **o) { return qi8(t, id, &G_Cube, &G_Base, &G_Resource, o); }
#define c_GetLevelDesc t_GetLevelDesc
static HRESULT WINAPI l_QueryInterface(IDirect3DVolume9 *t, REFIID id, void **o) { return qi8(t, id, &G_Volume, NULL, NULL, o); }
static HRESULT WINAPI l_GetDesc(IDirect3DVolume9 *t, vdesc8 *o)
{
    D3DVOLUME_DESC d;
    const d3vol *v = (const d3vol *)t;
    d3_volume_vtbl.GetDesc(t, &d);
    o->Format = d.Format; o->Type = d.Type; o->Usage = d.Usage; o->Pool = d.Pool; o->Width = d.Width; o->Height = d.Height; o->Depth = d.Depth;
    o->Size = (UINT)v->slice * v->d;
    return D3D_OK;
}
static HRESULT WINAPI x_QueryInterface(IDirect3DVolumeTexture9 *t, REFIID id, void **o) { return qi8(t, id, &G_VolumeTex, &G_Base, &G_Resource, o); }
static HRESULT WINAPI x_GetLevelDesc(d3tex *t, UINT l, vdesc8 *o) { if ((int)l >= t->levels) return D3DERR_INVALIDCALL; return l_GetDesc((IDirect3DVolume9 *)t->lv[l], o); }
static HRESULT WINAPI v_QueryInterface(IDirect3DVertexBuffer9 *t, REFIID id, void **o) { return qi8(t, id, &G_VB, &G_Resource, NULL, o); }
static HRESULT WINAPI i_QueryInterface(IDirect3DIndexBuffer9 *t, REFIID id, void **o) { return qi8(t, id, &G_IB, &G_Resource, NULL, o); }
/* IDirect3DSwapChain8: Present has no flags, GetBackBuffer is the same */
static HRESULT WINAPI w_QueryInterface(IDirect3DSwapChain9 *t, REFIID id, void **o) { return qi8(t, id, &G_Chain, NULL, NULL, o); }
static HRESULT WINAPI w_Present(IDirect3DSwapChain9 *t, const RECT *s, const RECT *o, HWND w, const RGNDATA *r) { return d3_chain_vtbl.Present(t, s, o, w, r, 0); }

/* ---------------------------------------------------------------- the tables */
#define ONE(n) + 1
#define ONE2(n, m) + 1
static const void *g_d3d[D8_D3D(ONE, ONE)], *g_dev[D8_DEVICE(ONE, ONE, ONE2)], *g_surf[D8_SURFACE(ONE, ONE)], *g_tex[D8_TEXTURE(ONE, ONE)], *g_cube[D8_CUBE(ONE, ONE)], *g_vol[D8_VOLUME(ONE, ONE)], *g_voltex[D8_VOLTEX(ONE, ONE)], *g_vb[D8_BUFFER(ONE, ONE)], *g_ib[D8_BUFFER(ONE, ONE)];
static const void *g_chain[5];

static void tables(void)
{
    const void **p;
    if (g_dev[0]) return;
#define O(n) *p++ = (const void *)o_##n;
#define S(n) *p++ = (const void *)d3_d3d_vtbl.n;
    p = g_d3d; D8_D3D(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)d_##n;
#define S(n) *p++ = (const void *)d3_dev_vtbl.n;
#define M(n, m) *p++ = (const void *)d3_dev_vtbl.m;
    p = g_dev; D8_DEVICE(O, S, M)
#undef O
#undef S
#undef M
#define O(n) *p++ = (const void *)s_##n;
#define S(n) *p++ = (const void *)d3_surface_vtbl.n;
    p = g_surf; D8_SURFACE(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)t_##n;
#define S(n) *p++ = (const void *)d3_texture_vtbl.n;
    p = g_tex; D8_TEXTURE(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)c_##n;
#define S(n) *p++ = (const void *)d3_cube_vtbl.n;
    p = g_cube; D8_CUBE(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)l_##n;
#define S(n) *p++ = (const void *)d3_volume_vtbl.n;
    p = g_vol; D8_VOLUME(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)x_##n;
#define S(n) *p++ = (const void *)d3_voltex_vtbl.n;
    p = g_voltex; D8_VOLTEX(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)v_##n;
#define S(n) *p++ = (const void *)d3_vb_vtbl.n;
    p = g_vb; D8_BUFFER(O, S)
#undef O
#undef S
#define O(n) *p++ = (const void *)i_##n;
#define S(n) *p++ = (const void *)d3_ib_vtbl.n;
    p = g_ib; D8_BUFFER(O, S)
#undef O
#undef S
    g_chain[0] = (const void *)w_QueryInterface; g_chain[1] = (const void *)d3_chain_vtbl.AddRef; g_chain[2] = (const void *)d3_chain_vtbl.Release; g_chain[3] = (const void *)w_Present; g_chain[4] = (const void *)d3_chain_vtbl.GetBackBuffer;
    d3_vt_surface = g_surf; d3_vt_texture = g_tex; d3_vt_cube = g_cube; d3_vt_volume = g_vol; d3_vt_voltex = g_voltex; d3_vt_vb = g_vb; d3_vt_ib = g_ib; d3_vt_device = g_dev; d3_vt_chain = g_chain;
}

void *WINAPI Direct3DCreate8(UINT sdk)
{
    IDirect3D9 *o;
    tables();
    o = Direct3DCreate9(D3D_SDK_VERSION);
    if (o) o->lpVtbl = (IDirect3D9Vtbl *)g_d3d;
    g2e_log("d3d8: Direct3DCreate8(%u)", sdk);
    return o;
}

/* what the system's d3d8.dll exports besides: the version token is what these two can say about a shader here */
static HRESULT validate(const DWORD *f, DWORD lo, DWORD hi, const char *what, BOOL want, char **errors)
{
    char m[80];
    HRESULT r = E_FAIL;
    m[0] = 0;
    if (f && *f >= lo && *f <= hi) r = S_OK;
    else if (want) snprintf(m, sizeof m, "%s: unsupported version token\n", what);
    if (errors && (*errors = (char *)HeapAlloc(GetProcessHeap(), 0, strlen(m) + 1))) strcpy(*errors, m);
    return r;
}
HRESULT WINAPI ValidateVertexShader(const DWORD *f, const DWORD *decl, const D3DCAPS9 *caps, BOOL want, char **errors) { (void)decl; (void)caps; return validate(f, 0xFFFE0100u, 0xFFFE0101u, "vertex shader", want, errors); }
HRESULT WINAPI ValidatePixelShader(const DWORD *f, const D3DCAPS9 *caps, BOOL want, char **errors) { (void)caps; return validate(f, 0xFFFF0100u, 0xFFFF0104u, "pixel shader", want, errors); }
void WINAPI DebugSetMute(void) { }
int WINAPI Direct3D8EnableMaximizedWindowedModeShim(UINT on) { (void)on; return 0; }
