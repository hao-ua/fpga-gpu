#!/usr/bin/env python3
"""oracle — the same canonical text as shc_dump.c, from the Python compiler (tools/glslc.py + tools/isa64.py). Python is the specification; the C port must match it word for word.
    python3 host/win/shc/test/oracle.py [--xops n] [--noops n] [--order a,b,c] [--unread mask] [--bind a=1,b=1] [--strict] file.vert file.frag ...      (run from the repository root)
--order renames nothing: it reorders the varying declarations of the source first, which is what g2sc_compile_ex must be equivalent to. --unread is g2sc_compile_vs, on the vertex shaders.
--bind is g2sc_compile_bound (not strict) on the vertex shaders without a .pack, --order or --unread."""
import os, re, signal, struct, sys
sys.path.insert(0, 'tools')
import glslc, isa64


def too_long(*_):
    raise RuntimeError('oracle timeout')


def bits(x):
    return '%016x' % struct.unpack('<Q', struct.pack('<d', float(x)))[0]


def syms(tag, tab):
    print(tag, len(tab))
    for k, v in tab.items():
        print(k, v[0], v[1], *(v[2:3] if len(v) > 2 else ()))


def reorder(src, order):
    """the declarations of the varyings in `order` first, in that order: moved to the first such declaration"""
    decl = {}
    for m in re.finditer(r'^[ \t]*varying\s+(?:\w+\s+)*?(\w+)\s+(\w+)\s*;[^\n]*\n', src, re.M):
        decl[m.group(2)] = m.group(0)
    first = min((src.index(t) for t in decl.values()), default=0)
    head, tail = src[:first], src[first:]
    for t in decl.values():
        tail = tail.replace(t, '', 1)
    text = ''.join(decl[n] for n in order if n in decl) + ''.join(t for n, t in decl.items() if n not in order)
    return head + text + tail


def read_pack(path):
    """the Appendix A.7 layout beside a shader, as shc_dump.c reads it"""
    try:
        t = open(path + '.pack').read().split()
    except OSError:
        return None
    p, i = {}, 0
    while i < len(t):
        if t[i] == 'U':
            n = int(t[i + 2]); p['urows'] = int(t[i + 1]); p['umap'] = [int(x) for x in t[i + 3:i + 3 + n]]; i += 3 + n
        elif t[i] == 'D':
            p['depth'] = int(t[i + 1]); i += 2
        else:
            n = int(t[i + 1]); p['vmap'] = [int(x) for x in t[i + 2:i + 2 + n]]; i += 2 + n
    return p


def main():
    args = sys.argv[1:]
    xops = 0
    if args and args[0] == '--xops':                # board B's fragment-core operations (g2sc_set_target): 1 SQRT / RCPM / RSQM, 2 CLAMP
        xops = int(args[1], 0)
        args = args[2:]
    noops = 0
    if args and args[0] == '--noops':               # the units both stages' cores lack (g2sc_set_absent): 1 the integer divider, 2 SIN / COS
        noops = int(args[1], 0)
        args = args[2:]
    order = None
    if args and args[0] == '--order':
        order = args[1].split(',')
        args = args[2:]
    unread = 0
    if args and args[0] == '--unread':
        unread = int(args[1], 0)
        args = args[2:]
    attrs = None
    if args and args[0] == '--bind':                # g2sc_compile_bound, on the vertex shaders: name=location,...
        attrs = {n: int(v, 0) for n, v in (b.split('=') for b in args[1].split(','))}
        args = args[2:]
    strict = bool(args) and args[0] == '--strict'
    if strict:
        args = args[1:]
    signal.signal(signal.SIGALRM, too_long)
    for path in args:
        print('FILE', os.path.basename(path), flush=True)
        stage = 'vs' if path.endswith(('.vert', '.vs')) else 'fs'
        signal.alarm(20)
        try:
            src = open(path, newline='').read()
            if order:
                src = reorder(src, order)
            pack = read_pack(path)
            bound = attrs is not None and stage == 'vs' and not pack and not order and not unread
            r = glslc.compile_stage(src, stage, isa64=True, forward=True, unread=unread if stage == 'vs' else 0, strict=strict and not pack and not bound, pack=pack, xops=xops, noops=noops,
                                    attrs=attrs if bound else None)
            words, _ = isa64.assemble(r['sasm'].splitlines(), vs=stage == 'vs')
        except Exception as e:
            print('ERROR', str(e).replace('\n', ' '))
            print('END')
            continue
        print('WORDS', len(words))
        for w in words:
            print('%016x' % (w & 0xFFFFFFFFFFFFFFFF))
        print('SASM')
        print(r['sasm'])
        print('ENDSASM')
        print('CONST', r['const_base'], len(r['const_pool']))
        for v in r['const_pool']:
            print(' '.join(bits(c) for c in v))
        syms('ATTR', r['attribs'])
        syms('UNI', r['uniforms'])
        syms('VARY', r['varyings'])
        print('SAMP', len(r['samplers']))
        for k, v in r['samplers'].items():
            print(k, v, *([r['sampler_sizes'][k]] if k in r['sampler_sizes'] else []))
        print('END')


main()
