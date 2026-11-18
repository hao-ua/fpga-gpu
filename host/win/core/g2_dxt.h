/* DXT1 / DXT3 / DXT5 and ETC1 to RGBA8 on the host: the TMU samples no compressed format, so GLES and Direct3D 9 expand at upload. Output: R G B A bytes, w x h pixels, tight; ETC1's alpha is 255. */
#ifndef G2_DXT_H
#define G2_DXT_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { G2_DXT1 = 1, G2_DXT1A = 2, G2_DXT3 = 3, G2_DXT5 = 5 };   /* DXT1A: colour 3 of a 3-colour block is transparent black */
void g2_dxt_decode(int kind, const uint8_t *src, int w, int h, uint8_t *rgba);
uint32_t g2_dxt_bytes(int kind, int w, int h);
void g2_etc1_decode(const uint8_t *src, int w, int h, uint8_t *rgba);
uint32_t g2_etc1_bytes(int w, int h);
/* B's 16-bit storage formats, G2_FMT_RGB565 / RGBA4444 / RGBA5551, R in the top bits as GL's UNSIGNED_SHORT_5_6_5 / 4_4_4_4 / 5_5_5_1: to and from R G B A bytes, rounding to the nearest. A format
 * without alpha reads 255 */
void g2_px16_unpack(uint32_t fmt, const uint16_t *in, size_t n, uint8_t *rgba);
void g2_px16_pack(uint32_t fmt, const uint8_t *rgba, size_t n, uint16_t *out);
#ifdef __cplusplus
}
#endif
#endif
