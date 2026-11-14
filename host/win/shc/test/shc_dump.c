/* shc_dump — every result of the C compiler as canonical text, one block per file. oracle.py prints the same text from tools/glslc.py + tools/isa64.py; compare.ps1 compares them line by line.
 *   shc_dump [--xops n] [--order a,b,c] [--unread mask] [--strict] file.vert|file.frag ...      (.vert / .vs = vertex stage, anything else = fragment; --xops: g2sc_set_target)
 * A file X.pack beside a shader is its Appendix A.7 layout (pack_check.py): a line "U urows slot..." and a line "V slot...", and a line "D bits": the samplers that read depth textures. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g2_shc.h"

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "rb");
    long n;
    char *s;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    s = (char *)malloc((size_t)n + 1);
    if (s) { n = (long)fread(s, 1, (size_t)n, f); s[n] = 0; }
    fclose(f);
    return s;
}

static const char *base_name(const char *p)
{
    const char *a = strrchr(p, '/'), *b = strrchr(p, '\\');
    if (b > a) a = b;
    return a ? a + 1 : p;
}

static int is_vs(const char *p)
{
    size_t n = strlen(p);
    return (n > 5 && !strcmp(p + n - 5, ".vert")) || (n > 3 && !strcmp(p + n - 3, ".vs"));
}

static int read_pack(const char *shader, g2sc_pack *p, int *u, int *v)
{
    char path[1024];
    FILE *f;
    char tag[4];
    int have = 0;
    snprintf(path, sizeof path, "%s.pack", shader);
    if (!(f = fopen(path, "r"))) return 0;
    memset(p, 0, sizeof *p);
    while (fscanf(f, "%3s", tag) == 1) {
        int n, i;
        if (tag[0] == 'U') {
            if (fscanf(f, "%d %d", &p->urows, &n) != 2) break;
            for (i = 0; i < n; i++) if (fscanf(f, "%d", &u[i]) != 1) break;
            p->umap = u; have = 1;
        } else if (tag[0] == 'D') {
            if (fscanf(f, "%u", &p->depth_units) != 1) break;
            have = 1;
        } else {
            if (fscanf(f, "%d", &n) != 1) break;
            for (i = 0; i < n; i++) if (fscanf(f, "%d", &v[i]) != 1) break;
            p->vmap = v; p->nvmap = n; have = 1;
        }
    }
    fclose(f);
    return have;
}

static void syms(const char *tag, const g2sc_sym *y, int n)
{
    int i;
    printf("%s %d\n", tag, n);
    for (i = 0; i < n; i++) {
        printf("%s %d %s", y[i].lname, y[i].loc, y[i].ltype);
        if (y[i].has_array) printf(" %d", y[i].array);
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    const char *order[32];
    char *list = NULL;
    uint32_t unread = 0;
    int a = 1, i, nord = 0, strict = 0;
    if (a + 1 < argc && !strcmp(argv[a], "--xops")) { g2sc_set_target((uint32_t)strtoul(argv[a + 1], NULL, 0)); a += 2; }     /* oracle.py --xops */
    if (a + 1 < argc && !strcmp(argv[a], "--order")) {
        char *t;
        list = argv[a + 1];
        a += 2;
        for (t = strtok(list, ","); t && nord < 32; t = strtok(NULL, ",")) order[nord++] = t;
    }
    if (a + 1 < argc && !strcmp(argv[a], "--unread")) { unread = (uint32_t)strtoul(argv[a + 1], NULL, 0); a += 2; }
    if (a < argc && !strcmp(argv[a], "--strict")) { strict = 1; a++; }
    for (; a < argc; a++) {
        g2sc_result r;
        g2sc_pack pk;
        static int um[4096], vm[256];
        char *src = slurp(argv[a]);
        int stage = is_vs(argv[a]) ? G2SC_VS : G2SC_FS, rc;
        printf("FILE %s\n", base_name(argv[a]));
        if (!src) { printf("ERROR unreadable\nEND\n"); continue; }
        memset(&r, 0, sizeof r);
        if (read_pack(argv[a], &pk, um, vm)) rc = g2sc_compile_packed(src, stage, NULL, 0, &pk, &r);
        else rc = nord ? g2sc_compile_ex(src, stage, order, nord, &r) : unread && stage == G2SC_VS ? g2sc_compile_vs(src, unread, &r) : strict ? g2sc_compile_strict(src, stage, &r) : g2sc_compile(src, stage, &r);
        free(src);
        if (rc < 0) { printf("ERROR %s\nEND\n", r.error); g2sc_free(&r); continue; }
        if (r.oversize) {                                       /* a library packs it at the link; Python's assembler is what the oracle ran */
            uint64_t *w = NULL;
            int n = 0;
            char e[512];
            if (g2sc_assemble(r.sasm, stage == G2SC_VS, 8, 4, &w, &n, e, sizeof e) < 0) { printf("ERROR %s\nEND\n", e); g2sc_free(&r); continue; }
            free(r.words);
            r.words = w; r.nwords = n;
        }
        printf("WORDS %d\n", r.nwords);
        for (i = 0; i < r.nwords; i++) printf("%016llx\n", (unsigned long long)r.words[i]);
        printf("SASM\n%s\nENDSASM\n", r.sasm);
        printf("CONST %d %d\n", r.const_base, r.nconst);
        for (i = 0; i < r.nconst; i++) {
            union { double d; unsigned long long u; } v[4];
            int k;
            for (k = 0; k < 4; k++) v[k].d = r.const_pool[i][k];
            printf("%016llx %016llx %016llx %016llx\n", v[0].u, v[1].u, v[2].u, v[3].u);
        }
        syms("ATTR", r.attribs, r.nattribs);
        syms("UNI", r.uniforms, r.nuniforms);
        syms("VARY", r.varyings, r.nvaryings);
        printf("SAMP %d\n", r.nsamplers);
        for (i = 0; i < r.nsamplers; i++) {
            printf("%s %d", r.samplers[i].lname, r.samplers[i].loc);
            if (r.samplers[i].has_array) printf(" %d", r.samplers[i].array);
            printf("\n");
        }
        printf("END\n");
        g2sc_free(&r);
    }
    return 0;
}
