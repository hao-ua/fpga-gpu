# Features

**On this branch the GPU is the one-board build.** It supports everything in sections 1–3 except what
[section 5](#5-the-one-board-build) lists. In short:
- OpenGL ES 2.0, OpenGL 2.0 and Direct3D 9 shader model 2.0 stay;
- SIN / COS and integer division are lowered by the compilers;
- targets and video modes go up to 800×600;
- everything else costs only speed.

What the hardware does, what each Windows library offers on top of it, and what is refused. The libraries implement
only what the hardware can do exactly, or what the PC can do exactly beside it; they never fake a feature.

## 1. Hardware

### Geometry (board A)
- **Primitives:** triangles, strips and fans; points (size up to 64 pixels); lines, line strips and line loops (width
  up to 64). Index counts are 32-bit.
- **Attribute formats:** byte, unsigned byte, short, unsigned short, fixed, float and FP24, normalised or not.
- **Clipping:**
  - against w ≥ 2⁻⁷, near and far, and a ±2040-pixel x/y guard band;
  - a 0 ≤ z ≤ w volume, selected per draw, for reversed depth;
  - viewports smaller than the target are cut through the scissor;
  - each draw is bounded by its scissor rectangle.
- **Culling:** back, front or both, with either winding; degenerate triangles are dropped.

### Fragments, depth and stencil (board B)
- **Depth and stencil:**
  - Depth is a D24S8 word, or a 16-bit depth mode without stencil.
  - 8 compare functions, depth write mask, polygon offset.
  - Early depth rejection on B.
  - Depth and stencil persist across passes (bands are loaded and flushed).
  - Stencil is 8-bit and two-sided, with reference, value mask, write mask and all ES 2.0 operations.
- **Blending:**
  - separate colour and alpha factors and equations (including min / max);
  - a constant colour;
  - exact rounding;
  - 4×4 ordered dither;
  - a per-channel colour mask;
  - dithered clears.
  - One colour target and one sample; there is no logic op in hardware.
- **Render targets:**
  - RGBA8, RGBA4444, RGBA5551 or RGB565;
  - up to 1280 pixels wide;
  - the row pitch is a multiple of 32 bytes.

### Textures (board B, `tmu24`)
- **Formats:** RGBA8, RGBA4444, RGBA5551 and RGB565, decoded exactly.
  - L, A, LA and RGB are expanded by the libraries.
  - DXT1/3/5 and ETC1 are decoded on the PC.
- **Filtering:** nearest and linear, nearest-mipmap and trilinear, LOD bias and explicit LOD.
- **Wrap modes:** REPEAT, CLAMP_TO_EDGE and MIRRORED_REPEAT, for non-power-of-two sizes as well. Border and mirror-once
  addressing are done in shader code the libraries generate.
- **Cube maps** (six square faces).
- **Incomplete textures** sample as (0, 0, 0, 1), as OpenGL ES 2.0 requires. A texture is incomplete when, for
  example, its filter uses mipmaps but levels are missing, or a cube map's faces differ in size.
- **Limits:** up to 2048 × 2048 and 12 levels. A 64 KB 4-way texture cache.
- **Not in hardware:** 3D textures, anisotropic filtering, MSAA.

### Shader cores
- **Number formats:** FP24 floats (sign, 7-bit exponent, 16-bit mantissa); 24-bit integers.
- **Instructions:** 64-bit words. Each vec4 instruction runs as one micro-op per written component, with free swizzles
  and modifiers on every source.
- **Operations:**
  - float: ADD SUB MUL MAD MIN MAX SET SETP MOV MOVI SEL, DP3 / DP4;
  - integer: IADD ISUB ISLT ISGE ISEQ ISNE IMUL IDIV IMOD UDIV UMOD;
  - conversions FTOI / ITOF / FTOU / UTOF, and rounding TRUNC FLR CEIL RNDE;
  - quarter-rate RCP RSQRT LOG2 EXP2 SIN COS, plus SQRT, RCPM and RSQM. A CLAMP unit is built but switched off.
- **Control flow:** IF / ELSE / ENDIF, LOOP / ENDL / BREAK / CONT and BRA, per lane. A stack overflow stops the warp
  and sets an error flag.
- **Fragment core (board B):**
  - 8 warps of 16 lanes (4 quads), 32 vec4 registers, 64 uniforms, 4096 instructions;
  - an IF stack of 8 and a loop stack of 4;
  - 8 varyings and 8 samplers per draw, up to 8 texture lookups in flight;
  - TEX with implicit LOD, bias or explicit LOD; coarse DDX / DDY; DISCARD; depth write;
  - gl_FragCoord, gl_FrontFacing, gl_PointCoord.
- **Vertex core (board A):**
  - 4 warps of 16 vertices, 16 attributes, 256 uniforms with run-time indexing;
  - outputs gl_Position, gl_PointSize and 16 varyings (B takes 8);
  - no texture, derivative or discard instructions, so no vertex textures.

### 2D and display (board B)
- **Image operations:** READ, FILL and COPY_IMAGE.
  - They convert between RGBA8 and RGB565 and copy 8-bit palette images.
  - They do nearest-neighbour stretch, mirror and source colour-key ranges, and overlap-safe 1:1 copies.
  - FILL takes a channel mask.
- **Scanout:**
  - RGBA8, RGB565 or 8-bit palette (256 × 24-bit, double-banked, switched at vsync); no gamma table.
  - Modes 640×480, 800×600 (the default), 1024×768, 1280×720 and 1280×800 at 60 Hz.
- **Cursor:** 64 × 64, colour with 8-bit alpha or monochrome AND / XOR. Position, shape and mode change at vsync;
  position updates have an unsequenced fast path.

## 2. Windows libraries (`host/win`)

All five are user-mode drop-in DLLs on one core library (`host/win/core`).

### `ddraw.dll` — DirectDraw 1–7, 2D (x86, x64)
- **Surfaces:** 8, 16 and 32 bits per pixel; 5:5:5 is stored as RGB565.
- **Blits:** copy (including overlapping), stretch, mirror, source and destination colour key, colour fill, clip lists,
  GDI through GetDC, and flip chains.
- **Where blits run:**
  - Every blit is done on a PC shadow copy, and the board repeats it with COPY / FILL_IMAGE where it can.
  - Destination keys, key ranges, 555 keys and raster ops other than SRCCOPY / BLACKNESS / WHITENESS stay on the PC.
- **Sizes:** a display size that is not a board video mode is shown at the largest integer scale, centred;
  `GPU2_SCALE=stretch` fills the screen instead.
- **Not done:** Direct3D 7 through QueryInterface, overlays, gamma outside 8-bit modes. Windowed mode is written but
  untested.

### `opengl32.dll` — OpenGL 2.0, GLSL 1.10 (x86, x64)
- **Entry points:** all 336 GL 1.1 entry points.
  - Accumulation, selection, feedback and the bitwise logic ops run on the PC, exactly.
  - Fixed function becomes generated shaders; GLSL 1.10 is translated onto the ES compiler.
- **Limits:**
  - 8 texture units, 8 lights, 6 clip planes;
  - textures up to 2048 (3D up to 128);
  - 896 vertex / 192 fragment uniform components and 32 varying floats;
  - 1 draw buffer, renderbuffers up to 1280, no multisample buffers.
- **Extensions:**
  - framebuffer object, blit and packed depth-stencil;
  - texture border clamp, S3TC (decoded on the PC), depth textures and shadow compare;
  - EXT_texture3D (sampled in shader code, since today's board B has no 3D textures);
  - non-power-of-two textures, shader texture LOD;
  - pixel buffer objects, point sprites.
- **Refused:** GL 2.1 and GLSL 1.20, sRGB, occlusion queries, MSAA, multiple render targets, anisotropy, float or
  integer textures, vertex textures.
- **Not exact:** GL_POLYGON_SMOOTH, glPixelStore byte swap / LSB-first, cube-map borders, `noise()` (returns 0),
  DEPTH_COMPONENT16 textures (render at 24 bits). `#version 120` is accepted, but only ES 1.00 syntax compiles.

### `libGLESv2.dll` + `libEGL.dll` — OpenGL ES 2.0, GLSL ES 1.00 (x86, x64)
- **Exports:** ANGLE's export ordinals; libEGL only forwards.
- **EGL configs:** six, RGBA8 or RGB565, each with D24S8, D16 or no depth.
- **Limits:** 16 attributes, 224 vertex / 48 fragment uniform vectors, 8 varyings, 8 texture units, 0 vertex texture
  units, textures up to 2048, renderbuffers up to 1280.
- **Extensions:**
  - OES: rgb8_rgba8, depth24, packed_depth_stencil, element_index_uint, texture_npot, standard_derivatives,
    fragment_precision_high, vertex_array_object, mapbuffer, depth_texture, fbo_render_mipmap, texture_3D only when
    board B has 3D (not today);
  - EXT: shader_texture_lod, frag_depth, discard_framebuffer, blend_minmax, BGRA8888 / read_format_bgra,
    texture_storage, robustness;
  - compressed formats DXT1/3/5 and ETC1 (decoded to RGB565);
  - ANGLE instanced arrays, framebuffer blit and translated shader source;
  - NV_fence, get_program_binary.
- **Refused:** OpenGL ES 3.0, which needs 32-bit float and integer, MRT and MSAA. `#version 300 es` is rejected.

### `d3d9.dll` — Direct3D 9, shader model 2.0 with the 2_x extras (x86, x64)
- **Shaders:**
  - vs_2_0 / ps_2_0 with predication, arbitrary swizzle, gradient instructions, 32 temporaries and 512 slots, with no
    dependent-read or texture-instruction limits.
  - Shader bytecode vs/ps 1.x–3.0 and all of ps 1.x is translated.
- **Caps:**
  - 256 vertex constants, 8 texture stages and samplers, 8 lights, 4 clip planes;
  - vertex blending (4 matrices, 28 indexed);
  - point size up to 64, 16 streams, 1 render target.
- **Also supported:**
  - two-sided stencil, depth bias and slope-scaled bias;
  - depth textures (INTZ, DF24, DF16, D24S8 with PCF);
  - mipmap generation (on the PC);
  - border and mirror-once addressing, LOD bias, PREMODULATE;
  - signed formats; flat shading; wireframe and point fill (done on the PC);
  - instancing, ProcessVertices, timestamp and vertex-cache queries;
  - the hardware cursor through SetCursorProperties, else the Windows pointer is mirrored.
  - sRGB texture reads and sRGB writes are done in generated shader code for 8-bit formats (unlike the GL libraries).
- **Shader model 3.0 is out of reach:** 64 fragment constants against 224 required, 8 varyings against 10, FP24 floats,
  8 samplers against 16.
- **Refused (the caps say so):** volume textures (today's board B has none), MSAA, occlusion queries, vertex textures,
  N-patches and RT patches.
- **Depth:** kept reversed inside (far at 0), for far-distance precision. `GPU2_REVZ`
  overrides.

### `d3d8.dll` — Direct3D 8 (x86 only)
- d3d9's objects behind the Direct3D 8 interfaces.
- vs/ps 1.1 and 96 vertex constants.
- A constant `a0` is resolved on the PC; a per-vertex `a0` uses the vertex core's indexed uniform load.

### Shader compiler (`host/win/shc`)
- A word-for-word C port of `tools/glslc.py`, which is its specification.
- Functions are inlined, and early return works per lane.
- Varyings are packed as GLSL ES Appendix A.7 says.
- Hardware loops are used.
- Uninitialised variables start at 0.
- It emits SQRT / RCPM / RSQM only on a board B that reports them.

## 3. Games and applications

| title | API | status |
|---|---|---|
| OpenArena | OpenGL | plays; timedemo at the game's default settings 28.6 fps at 640×480, 21.1 at 800×600 (A 148 / B 142) |
| Need for Speed: Hot Pursuit 2 | Direct3D 8 | races run, 22–36 fps at 800×600 (A 147 / B 142); far-depth flicker fixed |
| Half-Life | OpenGL 1.x | about 60 fps at 640×480; the flashed board A can hang on the first mission's tram ride (fixed in the RTL, not yet in the flash) |
| Heroes Chronicles (Heroes III engine) | DirectDraw | menus and play at 800×600×16 |
| Heroes of Might and Magic IV | DirectDraw | 1024×768×16, native video mode |
| Contraption Maker | OpenGL ES | menus correct, 5.4 fps at 1280×800 |
| Crayon Physics | Direct3D 9 | first screen correct, 8.6 fps |
| Super Splatters | Direct3D 9 | intro plays, 4.6 fps |

## 4. Known open issues

- **Far depth in OpenGL / OpenGL ES:** reversed depth is done only in Direct3D, so far geometry keeps about 1 % depth
  resolution at Quake scale.
- **Board B's command-processor error count** rises by one per run of each library test program. Not investigated.
- **Board link:** the CRC and parity counters creep slowly. No long soak test has been run.
- **Lost event:** board A once lost a present event after about 71 minutes of uptime. Not reproduced or investigated
  since.
- **Board B hang** in an NFS HP2 race with 128×128 render targets: cause unknown. A similar OpenArena hang was fixed.
- **Shader compiler limits:**
  - no spilling past 32 registers;
  - a uniform array copied whole inside a struct assignment is refused;
  - `texture2DLod` is accepted in fragment shaders without the extension;
  - `for(; i++ < N;)` is not treated as a counted loop.
- **Timing margin:** board B meets timing by only +0.016 ns at 95.7 % of its slices.

## 5. The one-board build

Everything in sections 1 and 2 holds on the one-board build (`board_ab_top`, see [ARCHITECTURE.md](ARCHITECTURE.md)
section 10) unless listed here. The cuts make it slower, not less capable: OpenGL ES 2.0, OpenGL 2.0 and Direct3D 9
shader model 2.0 stay as they are.

- **Same as the two boards:**
  - the shader number formats and instruction set;
  - 32 registers, 4096 fragment instructions, the uniforms, varyings, attributes and samplers listed above;
  - the texture formats, filters, wrap modes, cube maps and sizes up to 2048;
  - depth, stencil, blending, dither and the colour mask;
  - the 2D image operations, the palette and the cursor;
  - the host protocol and every library entry point.
- **Operations without a hardware unit,** lowered by the compilers (`tools/glslc.py`, `host/win/shc`) when STATUS says
  the unit is absent:
  - SIN and COS become a range reduction and a polynomial;
  - integer division and remainder (IDIV, IMOD, UDIV, UMOD) become float operations;
  - SQRT, RCPM and RSQM become RSQRT or RCP with a multiply, as on any board B without them.

  Programs that use them get longer: `cubes.vert` grows from 71 to 148 instructions.
- **Sizes:**
  - render targets and video modes up to 800 pixels wide: 640×480 and 800×600 only;
  - 2¹⁷ triangle records per frame (the two boards: 2¹⁸).
- **Speed only:**
  - 4 lanes per warp in both cores;
  - 4 fragment warps of one quad;
  - one quarter-rate unit per core;
  - 4 TEX instructions live instead of 8;
  - 100 MHz.

  Measured in [PERF_TESTS.md](PERF_TESTS.md).
- **Debug:** board A's geometry profile counters are off, so the libraries' `GPU2_STATS` geometry split and
  `fb_bench`'s PROFILE lines read 0 %. JTAG reads board A's STATUS only.
- **Host software:** this branch's libraries and `host/fb` tools.
  - They read STATUS flags bit 29 and move to the one-board heaps: board A's buffers in 256–448 MB, with a 64 MB vertex
    ring and up to 124 MB kept on board A; board B's from 464 MB.
  - master's libraries keep the two boards' heaps and run over board B's draw buckets: OpenArena stops after about a
    minute.
- **Games:** OpenArena's timedemo runs to the end, 7.2 fps at 640×480 at the game's default settings (S 158, with a test
  build of the library holding the one-board heaps). No other game has been run on one board.
- **Open on one board:**
  - The libraries do not know the 800-pixel limit: they still offer renderbuffers and modes up to 1280, and the board
    refuses the wider ones.
  - Only one board's PCB has run it, for minutes at a time; there has been no long run.
  - Timing is met by +0.001 ns at 99.1 % of the slices, so any growth of the RTL needs timing work.
  - The heap bounds and bit 29 (eb90593) have run in simulation only: no build has them yet.
