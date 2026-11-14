/* d3d9_cursor.c — a Windows cursor as the words of B's 64 x 64 hardware cursor. Games of Direct3D 8's age leave the pointer to Windows, which shows it on the PC's screen only:
 * the device puts the same shape on HDMI (cursor_mirror in d3d9_dev.c). */
#include "d3d9_internal.h"

static uint32_t *bits32(HDC dc, HBITMAP b, int w, int h)
{
    BITMAPINFO bi;
    uint32_t *p = (uint32_t *)malloc((size_t)w * h * 4);
    memset(&bi, 0, sizeof bi);
    bi.bmiHeader.biSize = sizeof bi.bmiHeader; bi.bmiHeader.biWidth = w; bi.bmiHeader.biHeight = -h; bi.bmiHeader.biPlanes = 1; bi.bmiHeader.biBitCount = 32; bi.bmiHeader.biCompression = BI_RGB;
    if (p && !GetDIBits(dc, b, 0, (UINT)h, p, &bi, DIB_RGB_COLORS)) { free(p); return NULL; }
    return p;
}

/* w[64 * 64]; *mode G2_CUR_COLOR or G2_CUR_ANDXOR. < 0: nothing to show. A cursor is: a monochrome mask of double height (AND over XOR), a colour bitmap with a mask, or 32-bit colour with alpha */
int d3_cursor_words(HCURSOR c, uint32_t *w, int *mode, int *hx, int *hy)
{
    ICONINFO ii;
    BITMAP bm;
    HDC dc;
    uint32_t *mask = NULL, *col = NULL;
    int bw, bh, x, y, alpha = 0, rc = -1;
    if (!GetIconInfo(c, &ii)) return -1;
    *hx = (int)ii.xHotspot > 63 ? 63 : (int)ii.xHotspot; *hy = (int)ii.yHotspot > 63 ? 63 : (int)ii.yHotspot;
    GetObject(ii.hbmMask, sizeof bm, &bm);
    bw = bm.bmWidth; bh = ii.hbmColor ? bm.bmHeight : bm.bmHeight / 2;
    dc = GetDC(NULL);
    mask = bits32(dc, ii.hbmMask, bw, ii.hbmColor ? bh : bh * 2);       /* a monochrome pixel reads as 0 or 0x00FFFFFF */
    if (ii.hbmColor) col = bits32(dc, ii.hbmColor, bw, bh);
    ReleaseDC(NULL, dc);
    if (col) for (y = 0; y < bw * bh; y++) if (col[y] >> 24) alpha = 1;
    if (mask && (col || !ii.hbmColor)) {
        *mode = alpha ? G2_CUR_COLOR : G2_CUR_ANDXOR;
        for (y = 0; y < 64; y++) {
            for (x = 0; x < 64; x++) {
                uint32_t o = alpha ? 0 : 1u << 24;              /* outside the shape: transparent */
                if (x < bw && y < bh) {
                    size_t i = (size_t)y * bw + x;
                    if (alpha) o = col[i];
                    else o = (mask[i] & 1) << 24 | (col ? col[i] & 0xFFFFFF : mask[(size_t)(y + bh) * bw + x] & 0xFFFFFF);
                }
                w[y * 64 + x] = o;
            }
        }
        rc = 0;
    }
    free(mask); free(col);
    DeleteObject(ii.hbmMask);
    if (ii.hbmColor) DeleteObject(ii.hbmColor);
    return rc;
}
