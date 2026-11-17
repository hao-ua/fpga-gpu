#!/bin/sh
# SPDX-License-Identifier: MIT
#
# apply.sh <mesa-tree>: copy the fpgagpu Gallium driver (overlay/) into a Mesa source tree and register it with the
# build, the DRI target and the pipe loader. Safe to run again: a registration already present is left alone. Stops
# with an error when an anchor line is not found exactly once (the tree is not the Mesa version this was written for).
set -eu

if [ $# -ne 1 ] || [ ! -f "$1/meson.options" ] || [ ! -d "$1/src/gallium" ]; then
	echo "usage: $0 <mesa source tree>" >&2
	exit 1
fi
MESA=$(cd "$1" && pwd)
HERE=$(cd "$(dirname "$0")" && pwd)

cp -R "$HERE/overlay/." "$MESA/"
echo "copied overlay/ into $MESA"

# the shader compiler's back end (optimizer, register allocator, assembler) is the Windows libraries' own
SHC="$HERE/../../win/shc"
mkdir -p "$MESA/src/gallium/drivers/fpgagpu/shc"
for f in shc_internal.h g2_shc.h shc_opt.c shc_asm.c shc_util.c; do
	cp "$SHC/$f" "$MESA/src/gallium/drivers/fpgagpu/shc/$f"
done
echo "copied the shc back end from $SHC"

python3 - "$MESA" <<'PY'
import sys

root = sys.argv[1]


def edit(path, anchor, lines, where):
    """where: 'before' / 'after' the anchor line, or 'replace' it; lines: the new lines. Skips when lines[0] is
    already a line of the file."""
    full = root + "/" + path
    with open(full, encoding="utf-8") as f:
        text = f.read()
    have = text.split("\n")
    if lines[0] in have:
        print("  %-55s already registered" % path)
        return
    hits = [i for i, l in enumerate(have) if l == anchor]
    if len(hits) != 1:
        sys.exit("apply.sh: %s: anchor line %r found %d times (want 1)" % (path, anchor, len(hits)))
    i = hits[0]
    if where == "before":
        have[i:i] = lines
    elif where == "after":
        have[i + 1:i + 1] = lines
    else:
        have[i:i + 1] = lines
    with open(full, "w", encoding="utf-8") as f:
        f.write("\n".join(have))
    print("  %-55s registered" % path)


edit("meson.options",
     "    'asahi', 'crocus', 'd3d12', 'ethosu', 'etnaviv', 'freedreno', 'i915', 'iris',",
     ["    'asahi', 'crocus', 'd3d12', 'ethosu', 'etnaviv', 'fpgagpu', 'freedreno', 'i915', 'iris',"],
     "replace")

edit("meson.build",
     "with_gallium_lima = gallium_drivers.contains('lima')",
     ["with_gallium_fpgagpu = gallium_drivers.contains('fpgagpu')"],
     "after")

edit("src/gallium/meson.build",
     "if with_gallium_rocket",
     ["if with_gallium_fpgagpu",
      "  subdir('winsys/fpgagpu/drm')",
      "  subdir('drivers/fpgagpu')",
      "else",
      "  driver_fpgagpu = declare_dependency()",
      "endif"],
     "before")

edit("src/gallium/targets/dri/meson.build",
     "    driver_asahi, driver_crocus, driver_rocket, driver_ethosu",
     ["    driver_asahi, driver_crocus, driver_rocket, driver_ethosu, driver_fpgagpu"],
     "replace")

edit("src/gallium/auxiliary/target-helpers/drm_helper.h",
     "#ifdef GALLIUM_KMSRO",
     ["#ifdef GALLIUM_FPGAGPU",
      "#include \"fpgagpu/drm/fpgagpu_drm_public.h\"",
      "",
      "static struct pipe_screen *",
      "pipe_fpgagpu_create_screen(int fd, const struct pipe_screen_config *config)",
      "{",
      "   struct pipe_screen *screen;",
      "",
      "   screen = fpgagpu_drm_screen_create(fd, config);",
      "   return screen ? debug_screen_wrap(screen) : NULL;",
      "}",
      "DRM_DRIVER_DESCRIPTOR(fpgagpu, NULL, 0)",
      "",
      "#else",
      "DRM_DRIVER_DESCRIPTOR_STUB(fpgagpu)",
      "#endif",
      ""],
     "before")

edit("src/gallium/auxiliary/target-helpers/drm_helper_public.h",
     "extern const struct drm_driver_descriptor kmsro_driver_descriptor;",
     ["extern const struct drm_driver_descriptor fpgagpu_driver_descriptor;"],
     "before")

# before kmsro: the pipe loader takes the first descriptor whose name matches, and falls back to kmsro
edit("src/gallium/auxiliary/pipe-loader/pipe_loader_drm.c",
     "   &kmsro_driver_descriptor,",
     ["   &fpgagpu_driver_descriptor,"],
     "before")

edit("src/gallium/targets/dril/meson.build",
     "             [with_gallium_lima, ['lima']],",
     ["             [with_gallium_fpgagpu, ['fpgagpu']],"],
     "after")

edit("src/gallium/targets/dril/dril_target.c",
     "DEFINE_LOADER_DRM_ENTRYPOINT(lima)",
     ["DEFINE_LOADER_DRM_ENTRYPOINT(fpgagpu)"],
     "after")

# OpenGL 2.0 with GLSL 1.10: Mesa raises an OpenGL 2.0 context's GLSL to 1.20, while fpgagpu offers what its hardware
# does, GLSL 1.10 (glsl_feature_level); a driver whose GLSL is 1.20 or more is not changed
edit("src/mesa/main/version.c",
     "         FALLTHROUGH; /* GLSL 1.20 is the minimum we support */",
     ["         /* the driver's GLSL when it is 1.10 (fpgagpu: OpenGL 2.0 with GLSL 1.10), else 1.20 */",
      "         ctx->Const.GLSLVersion = MAX2(110, MIN2(ctx->Const.GLSLVersion, 120));",
      "         break;"],
     "replace")

edit("src/mesa/main/getstring.c",
     "      case 120:",
     ["      case 110:",
      "         return (const GLubyte *) \"1.10\";"],
     "before")

# texture swizzle in OpenGL ES 2.0 contexts too: X's glamor keeps alpha-only pictures (glyphs) in GL_R8 textures and
# reads their alpha through GL_TEXTURE_SWIZZLE_A = GL_RED on any GLES; without it every glyph samples alpha 1
edit("src/mesa/main/extensions_table.h",
     "EXT(EXT_texture_swizzle                     , EXT_texture_swizzle                    , GLL, GLC,  x ,  x , 2008)",
     ["EXT(EXT_texture_swizzle                     , EXT_texture_swizzle                    , GLL, GLC,  x , ES2, 2008)"],
     "replace")

# occlusion queries: fpgagpu's board counts no samples (QUERY_COUNTER_BITS 0), yet its queries answer whether a draw
# ran inside them (1, never 0 where samples passed); Mesa makes a counterless driver's queries dummies that answer 0
edit("src/mesa/main/queryobj.c",
     "      return !st->screen->caps.occlusion_query;",
     ["      /* fpgagpu: no samples counter, but its queries say whether a draw ran (never 0 where samples passed) */",
      "      return !st->screen->caps.occlusion_query && strcmp(st->screen->get_name(st->screen), \"fpgagpu\") != 0;"],
     "replace")
PY
echo "done: configure with -Dgallium-drivers=fpgagpu,..."
