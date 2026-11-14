/* g2_env: see g2_env.h */
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g2_env.h"

#define MAX_INI 64
static struct { char key[32], val[224]; } g_ini[MAX_INI];
static int g_nini;
static LONG g_ini_done;
static FILE *g_logf;
static LONG g_log_done;

static void exe_dir(char *out, size_t n)
{
    char *p;
    GetModuleFileNameA(NULL, out, (DWORD)n);
    p = strrchr(out, '\\');
    if (p) p[1] = 0;
}

static void load_ini(void)
{
    char path[MAX_PATH + 16], line[300];
    FILE *f;
    if (InterlockedExchange(&g_ini_done, 1)) return;
    exe_dir(path, MAX_PATH);
    strcat(path, "gpu2.ini");
    f = fopen(path, "r");
    if (!f) return;
    while (fgets(line, sizeof line, f) && g_nini < MAX_INI) {
        char *eq = strchr(line, '='), *e;
        if (line[0] == '#' || line[0] == ';' || !eq) continue;
        *eq = 0;
        e = eq + 1 + strcspn(eq + 1, "\r\n");
        *e = 0;
        snprintf(g_ini[g_nini].key, sizeof g_ini[0].key, "%s", line);
        snprintf(g_ini[g_nini].val, sizeof g_ini[0].val, "%s", eq + 1);
        g_nini++;
    }
    fclose(f);
}

const char *g2e_get(const char *name, const char *dflt)
{
    const char *v = getenv(name);
    int i;
    if (v && *v) return v;
    load_ini();
    for (i = 0; i < g_nini; i++) if (!strcmp(g_ini[i].key, name)) return g_ini[i].val;
    return dflt;
}

int g2e_get_int(const char *name, int dflt)
{
    const char *v = g2e_get(name, NULL);
    return v ? atoi(v) : dflt;
}

void g2e_log(const char *fmt, ...)
{
    va_list ap;
    if (!InterlockedExchange(&g_log_done, 1)) {
        const char *p = g2e_get("GPU2_LOG", "");
        if (*p) g_logf = fopen(p, "a");
    }
    if (!g_logf) return;
    {
        SYSTEMTIME t;
        GetLocalTime(&t);
        fprintf(g_logf, "%02d:%02d:%02d.%03d [%lu] ", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, GetCurrentThreadId());
    }
    va_start(ap, fmt);
    vfprintf(g_logf, fmt, ap);
    va_end(ap);
    fputc('\n', g_logf);
    fflush(g_logf);
}

static void core_log(const char *line) { g2e_log("gpu2core: %s", line); }

g2_board *g2e_connect(const char *who)
{
    const char *ip = g2e_get("GPU2_IP", "192.168.1.10");
    int port = g2e_get_int("GPU2_PORT", 5000);
    g2_board *b;
    g2_set_log(core_log);
    b = g2_open(ip, port, NULL);
    if (!b) { g2e_log("%s: cannot open a socket", who); return NULL; }
    if (g2_hello(b, 1) < 0 || g2_check_boards(b, g2e_get_int("GPU2_MIN_A", 62)) < 0) {
        g2e_log("%s: the boards at %s:%d are not usable: %s", who, ip, port, g2_error_text(b));
        g2_close(b);
        return NULL;
    }
    g2e_log("%s: connected to %s:%d", who, ip, port);
    return b;
}

static LONG WINAPI on_crash(EXCEPTION_POINTERS *e)
{
    HMODULE m = NULL;
    char name[MAX_PATH] = "?";
    void *addr = e->ExceptionRecord->ExceptionAddress;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)addr, &m);
    if (m) GetModuleFileNameA(m, name, sizeof name);
    g2e_log("CRASH code %08lX at %p: %s + 0x%llx (addr2line -e that file)", e->ExceptionRecord->ExceptionCode, addr, name,
            (unsigned long long)((char *)addr - (char *)m));
    {   /* no debugger on this toolchain: whatever on the stack points into our own code is most likely the way here */
#ifdef _WIN64
        uintptr_t *sp = (uintptr_t *)e->ContextRecord->Rsp;
#else
        uintptr_t *sp = (uintptr_t *)e->ContextRecord->Esp;
#endif
        uintptr_t *top = (uintptr_t *)((NT_TIB *)NtCurrentTeb())->StackBase;
        int n = 0;
        for (; sp < top && n < 16; sp++) {
            HMODULE c = NULL;
            char mod[MAX_PATH];
            if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)*sp, &c) || !c) continue;
            if (!GetModuleFileNameA(c, mod, sizeof mod) || strstr(mod, "\\System32\\") || strstr(mod, "\\SysWOW64\\") || strstr(mod, "\\system32\\")) continue;
            g2e_log("  on the stack: %s + 0x%llx", mod, (unsigned long long)(*sp - (uintptr_t)c));
            n++;
        }
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

void g2e_crash_handler(void) { SetUnhandledExceptionFilter(on_crash); }
