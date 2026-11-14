/* d3d8_check.c — compiled, never linked: d3d8_slots.h against the SDK's d3d8.h. A wrong slot or size stops the build. */
#define CINTERFACE
#include <stddef.h>
#include <windows.h>
#include <d3d8.h>
#include "d3d8_slots.h"

#define CHECK(T, LIST, ...) \
    enum { LIST(__VA_ARGS__) T##_n }

#define E_DEV(n) e_dev_##n,
#define E_DEV2(n, m) e_dev_##n,
enum { D8_DEVICE(E_DEV, E_DEV, E_DEV2) n_dev };
#define A_DEV(n) _Static_assert(offsetof(IDirect3DDevice8Vtbl, n) == e_dev_##n * sizeof(void *), "IDirect3DDevice8." #n);
#define A_DEV2(n, m) A_DEV(n)
D8_DEVICE(A_DEV, A_DEV, A_DEV2)
_Static_assert(sizeof(IDirect3DDevice8Vtbl) == n_dev * sizeof(void *), "IDirect3DDevice8 length");

#define E_D3D(n) e_d3d_##n,
enum { D8_D3D(E_D3D, E_D3D) n_d3d };
#define A_D3D(n) _Static_assert(offsetof(IDirect3D8Vtbl, n) == e_d3d_##n * sizeof(void *), "IDirect3D8." #n);
D8_D3D(A_D3D, A_D3D)
_Static_assert(sizeof(IDirect3D8Vtbl) == n_d3d * sizeof(void *), "IDirect3D8 length");

#define E_SURF(n) e_surf_##n,
enum { D8_SURFACE(E_SURF, E_SURF) n_surf };
#define A_SURF(n) _Static_assert(offsetof(IDirect3DSurface8Vtbl, n) == e_surf_##n * sizeof(void *), "IDirect3DSurface8." #n);
D8_SURFACE(A_SURF, A_SURF)
_Static_assert(sizeof(IDirect3DSurface8Vtbl) == n_surf * sizeof(void *), "IDirect3DSurface8 length");

#define E_TEX(n) e_tex_##n,
enum { D8_TEXTURE(E_TEX, E_TEX) n_tex };
#define A_TEX(n) _Static_assert(offsetof(IDirect3DTexture8Vtbl, n) == e_tex_##n * sizeof(void *), "IDirect3DTexture8." #n);
D8_TEXTURE(A_TEX, A_TEX)
_Static_assert(sizeof(IDirect3DTexture8Vtbl) == n_tex * sizeof(void *), "IDirect3DTexture8 length");

#define E_CUBE(n) e_cube_##n,
enum { D8_CUBE(E_CUBE, E_CUBE) n_cube };
#define A_CUBE(n) _Static_assert(offsetof(IDirect3DCubeTexture8Vtbl, n) == e_cube_##n * sizeof(void *), "IDirect3DCubeTexture8." #n);
D8_CUBE(A_CUBE, A_CUBE)
_Static_assert(sizeof(IDirect3DCubeTexture8Vtbl) == n_cube * sizeof(void *), "IDirect3DCubeTexture8 length");

#define E_VOL(n) e_vol_##n,
enum { D8_VOLUME(E_VOL, E_VOL) n_vol };
#define A_VOL(n) _Static_assert(offsetof(IDirect3DVolume8Vtbl, n) == e_vol_##n * sizeof(void *), "IDirect3DVolume8." #n);
D8_VOLUME(A_VOL, A_VOL)
_Static_assert(sizeof(IDirect3DVolume8Vtbl) == n_vol * sizeof(void *), "IDirect3DVolume8 length");

#define E_VT(n) e_vt_##n,
enum { D8_VOLTEX(E_VT, E_VT) n_vt };
#define A_VT(n) _Static_assert(offsetof(IDirect3DVolumeTexture8Vtbl, n) == e_vt_##n * sizeof(void *), "IDirect3DVolumeTexture8." #n);
D8_VOLTEX(A_VT, A_VT)
_Static_assert(sizeof(IDirect3DVolumeTexture8Vtbl) == n_vt * sizeof(void *), "IDirect3DVolumeTexture8 length");

#define E_BUF(n) e_buf_##n,
enum { D8_BUFFER(E_BUF, E_BUF) n_buf };
#define A_VB(n) _Static_assert(offsetof(IDirect3DVertexBuffer8Vtbl, n) == e_buf_##n * sizeof(void *), "IDirect3DVertexBuffer8." #n);
#define A_IB(n) _Static_assert(offsetof(IDirect3DIndexBuffer8Vtbl, n) == e_buf_##n * sizeof(void *), "IDirect3DIndexBuffer8." #n);
D8_BUFFER(A_VB, A_VB)
D8_BUFFER(A_IB, A_IB)
_Static_assert(sizeof(IDirect3DVertexBuffer8Vtbl) == n_buf * sizeof(void *), "IDirect3DVertexBuffer8 length");
_Static_assert(sizeof(IDirect3DIndexBuffer8Vtbl) == n_buf * sizeof(void *), "IDirect3DIndexBuffer8 length");

_Static_assert(sizeof(D3DCAPS8) == D8_SIZE_CAPS, "D3DCAPS8");
_Static_assert(sizeof(D3DPRESENT_PARAMETERS) == D8_SIZE_PP, "D3DPRESENT_PARAMETERS");
_Static_assert(sizeof(D3DSURFACE_DESC) == D8_SIZE_DESC, "D3DSURFACE_DESC");
_Static_assert(sizeof(D3DVOLUME_DESC) == D8_SIZE_VDESC, "D3DVOLUME_DESC");
_Static_assert(sizeof(D3DADAPTER_IDENTIFIER8) == D8_SIZE_ID, "D3DADAPTER_IDENTIFIER8");
