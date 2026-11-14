/* ff_dump.c — the fixed-function programs gl_ff.c generates for chosen states, compiled as gl_ff_program compiles them, with no board:
 * their instruction counts, and with -v the GLSL and the fragment program's assembly. Links the opengl32 objects (build.ps1 -Target gl).
 *   ff_dump.exe [-v] [-n] [-o dir] [state ...]      no state: all of them; -o writes each fragment program to dir/<state>.frag (a packed one its .vert and .pack too);
 *                                                   -n stage by stage, the varyings one a row (what the link's packing is measured against) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gl_internal.h"
#include "g2_shc.h"

typedef struct st { const char *name, *what; void (*set)(ff_key *k); } st;

static void unit(ff_key *k, int u, int base, int env) { k->u[u].target = 1; k->u[u].base = (uint8_t)base; k->u[u].env = (uint8_t)env; k->tex_units++; }

/* alpha_fn: GL_NEVER .. GL_ALWAYS & 7 */
static void s_none(ff_key *k) { (void)k; }
static void s_mod(ff_key *k) { unit(k, 0, BF_RGBA, 0); }
static void s_mod_rgb(ff_key *k) { unit(k, 0, BF_RGB, 0); }
static void s_rep(ff_key *k) { unit(k, 0, BF_RGBA, 2); }
static void s_mod_gt0(ff_key *k) { s_mod(k); k->alpha_fn = 4; }
static void s_mod_ge(ff_key *k) { s_mod(k); k->alpha_fn = 6; }
static void s_mod_lt(ff_key *k) { s_mod(k); k->alpha_fn = 1; }
static void s_lm(ff_key *k) { unit(k, 0, BF_RGBA, 0); unit(k, 1, BF_RGB, 0); }
static void s_lm_rgb(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_RGB, 0); }
static void s_lm_add(ff_key *k) { unit(k, 0, BF_RGBA, 0); unit(k, 1, BF_RGB, 4); }
static void s_lm_ge(ff_key *k) { s_lm(k); k->alpha_fn = 6; }
static void s_mod_clamp(ff_key *k) { s_mod(k); k->u[0].bs = k->u[0].bt = 1; }
static void s_mod_clamp_mip(ff_key *k) { s_mod_clamp(k); k->u[0].blod = 1; }
static void s_mod_bias(ff_key *k) { s_mod(k); k->u[0].lod = 1; }
static void s_mod_minmax(ff_key *k) { s_mod(k); k->u[0].lod = 2; }
static void s_mod_fog(ff_key *k) { s_mod(k); k->fog = 1; }
static void s_rgb_decal(ff_key *k) { unit(k, 0, BF_RGB, 1); }
static void s_rgb_rep(ff_key *k) { unit(k, 0, BF_RGB, 2); }
static void s_rgb_blend(ff_key *k) { unit(k, 0, BF_RGB, 3); }
static void s_rgb_add(ff_key *k) { unit(k, 0, BF_RGB, 4); }
static void s_rgba_decal(ff_key *k) { unit(k, 0, BF_RGBA, 1); }
static void s_rgb_then_rgba(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_RGBA, 0); }
static void s_rgb_rgba_rgb(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_RGBA, 0); unit(k, 2, BF_RGB, 3); }
static void s_rgb_alpha(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_ALPHA, 0); }
static void s_rgb_int(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_INTENSITY, 3); }
static void s_rgb_comb(ff_key *k)
{
    unit(k, 0, BF_RGB, 0); unit(k, 1, BF_RGBA, 5);
    k->u[1].comb_rgb = 4; k->u[1].comb_a = 1; k->u[1].scale_rgb = 2; k->u[1].scale_a = 1;
    k->u[1].src_rgb[0] = 0; k->u[1].src_rgb[1] = 3; k->u[1].src_rgb[2] = 2; k->u[1].src_a[0] = 3; k->u[1].src_a[1] = 0;
}
static void s_rgb_sec(ff_key *k) { s_mod_rgb(k); k->sec = 1; }
static void s_rgb_fog(ff_key *k) { s_mod_rgb(k); k->fog = 2; }
static void s_rgb_ge(ff_key *k) { s_mod_rgb(k); k->alpha_fn = 6; }
static void s_mod_le(ff_key *k) { s_mod(k); k->alpha_fn = 3; }
static void s_mod_eq(ff_key *k) { s_mod(k); k->alpha_fn = 2; }
static void s_mod_ne(ff_key *k) { s_mod(k); k->alpha_fn = 5; }
static void s_mod_never(ff_key *k) { s_mod(k); k->alpha_fn = 0; }
static void s_mod_ge_inv(ff_key *k) { s_mod(k); k->alpha_fn = 6; k->logic = 3; }
static void s_mask(ff_key *k) { unit(k, 0, BF_RGBA, 0); unit(k, 1, BF_RGB, 0); k->mask0 = 1; }
static void s_lm_rgb_gt(ff_key *k) { s_lm_rgb(k); k->alpha_fn = 4; }
static void s_rgb_alpha_rep(ff_key *k) { unit(k, 0, BF_RGB, 0); unit(k, 1, BF_ALPHA, 2); }
static void s_mod_sec(ff_key *k) { s_mod(k); k->sec = 1; }
static void s_mod_sec_fog(ff_key *k) { s_mod(k); k->sec = 1; k->fog = 3; }
static void s_none_fog(ff_key *k) { k->fog = 1; }
static void s_lm_rgb_fog_ge(ff_key *k) { s_lm_rgb(k); k->fog = 1; k->alpha_fn = 6; }
static void s_clip_rgb(ff_key *k) { s_mod_rgb(k); k->clip_mask = 1; }
static void s_stip_rgb(ff_key *k) { s_mod_rgb(k); k->poly_stip = 1; }
static void s_mask_fog(ff_key *k) { s_mask(k); k->fog = 1; }
static void s_clip_lm(ff_key *k) { s_lm_rgb(k); k->clip_mask = 3; }

static const st STATES[] = {
    { "none", "no texture: the colour", s_none },
    { "mod", "RGBA texture x colour (MODULATE)", s_mod },
    { "mod_rgb", "RGB texture x colour", s_mod_rgb },
    { "rep", "REPLACE", s_rep },
    { "mod_gt0", "MODULATE, alpha test GREATER", s_mod_gt0 },
    { "mod_ge", "MODULATE, alpha test GEQUAL", s_mod_ge },
    { "mod_lt", "MODULATE, alpha test LESS", s_mod_lt },
    { "lm", "RGBA texture x RGB lightmap x colour (two units MODULATE)", s_lm },
    { "lm_rgb", "RGB texture x RGB lightmap x colour", s_lm_rgb },
    { "lm_add", "RGBA texture x colour + RGB texture (unit 1 ADD)", s_lm_add },
    { "lm_ge", "two units MODULATE, alpha test GEQUAL", s_lm_ge },
    { "mod_clamp", "MODULATE, GL_CLAMP linear (border in the program)", s_mod_clamp },
    { "mod_clamp_mip", "MODULATE, GL_CLAMP linear with mipmaps", s_mod_clamp_mip },
    { "mod_bias", "MODULATE, a LOD bias", s_mod_bias },
    { "mod_minmax", "MODULATE, MIN_LOD / MAX_LOD", s_mod_minmax },
    { "mod_fog", "MODULATE, linear fog", s_mod_fog },
    { "rgb_decal", "RGB DECAL", s_rgb_decal },
    { "rgb_rep", "RGB REPLACE", s_rgb_rep },
    { "rgb_blend", "RGB BLEND", s_rgb_blend },
    { "rgb_add", "RGB ADD", s_rgb_add },
    { "rgba_decal", "RGBA DECAL", s_rgba_decal },
    { "rgb_then_rgba", "RGB MODULATE, then RGBA MODULATE", s_rgb_then_rgba },
    { "rgb_rgba_rgb", "RGB MODULATE, RGBA MODULATE, RGB BLEND", s_rgb_rgba_rgb },
    { "rgb_alpha", "RGB MODULATE, then ALPHA MODULATE", s_rgb_alpha },
    { "rgb_int", "RGB MODULATE, then INTENSITY BLEND", s_rgb_int },
    { "rgb_comb", "RGB MODULATE, then COMBINE reading the previous alpha", s_rgb_comb },
    { "rgb_sec", "RGB MODULATE, colour sum", s_rgb_sec },
    { "rgb_fog", "RGB MODULATE, EXP fog", s_rgb_fog },
    { "rgb_ge", "RGB MODULATE, alpha test GEQUAL", s_rgb_ge },
    { "mod_le", "MODULATE, alpha test LEQUAL", s_mod_le },
    { "mod_eq", "MODULATE, alpha test EQUAL", s_mod_eq },
    { "mod_ne", "MODULATE, alpha test NOTEQUAL", s_mod_ne },
    { "mod_never", "MODULATE, alpha test NEVER", s_mod_never },
    { "mod_ge_inv", "MODULATE, alpha test GEQUAL, logic op COPY_INVERTED", s_mod_ge_inv },
    { "mask", "unit 0 a mask, RGB unit 1", s_mask },
    { "lm_rgb_gt", "RGB texture x RGB lightmap x colour, alpha test GREATER", s_lm_rgb_gt },
    { "rgb_alpha_rep", "RGB MODULATE, then ALPHA REPLACE", s_rgb_alpha_rep },
    { "mod_sec", "MODULATE, colour sum", s_mod_sec },
    { "mod_sec_fog", "MODULATE, colour sum, EXP2 fog", s_mod_sec_fog },
    { "none_fog", "no texture, linear fog", s_none_fog },
    { "lm_rgb_fog_ge", "RGB texture x RGB lightmap x colour, fog, alpha test GEQUAL", s_lm_rgb_fog_ge },
    { "clip_rgb", "a clip plane, RGB MODULATE", s_clip_rgb },
    { "stip_rgb", "polygon stipple, RGB MODULATE", s_stip_rgb },
    { "mask_fog", "unit 0 a mask, RGB unit 1, fog", s_mask_fog },
    { "clip_lm", "two clip planes, RGB texture x RGB lightmap x colour", s_clip_lm },
};

static const char *out_dir;
static int no_pack;                                     /* -n: every program stage by stage, as before the link packed coordinates (the comparison) */

static void put(const char *name, const char *ext, const char *text)
{
    char path[512];
    FILE *o;
    snprintf(path, sizeof path, "%s/%s.%s", out_dir, name, ext);
    if ((o = fopen(path, "wb")) != NULL) { fputs(text, o); fclose(o); }
}

/* as gl_ff_program builds it: a tightened program through g2sc_link (its varyings packed by Appendix A.7), the rest compiled stage by stage. -o writes what compare.ps1 compiles the same way: the texts, and a packed
 * program's vertex text too, with the .pack the link used */
static void dump(const st *s, int verbose)
{
    ff_key k;
    char *vs, *fs;
    int tight, i, rows = 0;
    g2sc_result rv, rf;
    g2sc_linked lk;
    memset(&k, 0, sizeof k);
    k.alpha_fn = 7;
    s->set(&k);
    if (no_pack) {
        gl_ureq vu[256], fu[256];
        int nvu, nfu;
        gl_ff_text(&k, &vs, &fs, vu, &nvu, fu, &nfu);
        tight = 0;
    } else tight = gl_ff_text_tight(&k, &vs, &fs);
    memset(&rv, 0, sizeof rv); memset(&rf, 0, sizeof rf);
    if (tight) {
        if (g2sc_link(vs, fs, &lk) < 0) { printf("%-14s does not link: %s\n", s->name, lk.error); free(vs); free(fs); return; }
        rv = lk.vs; rf = lk.fs;
        for (i = 0; i < 32; i++) rows += lk.fsmask >> i & 1;
        printf("%-14s VS %2d  FS %2d  %s; %d rows%s\n", s->name, rv.nwords, rf.nwords, s->what, rows, lk.vpacked ? ", packed" : "");
    } else if (g2sc_compile(vs, G2SC_VS, &rv) < 0 || g2sc_compile(fs, G2SC_FS, &rf) < 0) printf("%-14s does not compile: %s %s\n", s->name, rv.error, rf.error);
    else printf("%-14s VS %2d  FS %2d  %s\n", s->name, rv.nwords, rf.nwords, s->what);
    if (verbose) printf("--- fs\n%s--- fs assembly\n%s\n\n", fs, rf.sasm ? rf.sasm : "");
    if (out_dir) {
        put(s->name, "frag", fs);
        if (tight) {
            char pk[1024];
            int n = 0;
            put(s->name, "vert", vs);
            if (lk.vpacked) {
                n = snprintf(pk, sizeof pk, "V %d", 64);
                for (i = 0; i < 64 && n < (int)sizeof pk - 8; i++) n += snprintf(pk + n, sizeof pk - (size_t)n, " %d", lk.vmap[i]);
                snprintf(pk + n, sizeof pk - (size_t)n, "\n");
                put(s->name, "frag.pack", pk); put(s->name, "vert.pack", pk);
            }
        }
    }
    if (tight) g2sc_link_free(&lk);
    else { g2sc_free(&rv); g2sc_free(&rf); }
    free(vs); free(fs);
}

int main(int argc, char **argv)
{
    int verbose = 0, a = 1, i, any = 0;
    if (a < argc && !strcmp(argv[a], "-v")) { verbose = 1; a++; }
    if (a < argc && !strcmp(argv[a], "-n")) { no_pack = 1; a++; }
    if (a + 1 < argc && !strcmp(argv[a], "-o")) { out_dir = argv[a + 1]; a += 2; }
    for (; a < argc; a++) {
        for (i = 0; i < (int)(sizeof STATES / sizeof STATES[0]); i++) if (!strcmp(argv[a], STATES[i].name)) { dump(&STATES[i], verbose); any = 1; break; }
        if (i == (int)(sizeof STATES / sizeof STATES[0])) { fprintf(stderr, "no state %s\n", argv[a]); return 1; }
    }
    if (!any) for (i = 0; i < (int)(sizeof STATES / sizeof STATES[0]); i++) dump(&STATES[i], verbose);
    return 0;
}
