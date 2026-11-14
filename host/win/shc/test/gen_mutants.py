#!/usr/bin/env python3
"""gen_mutants — broken and bent shaders for the oracle: every seed shader, mutated at the token level with a fixed seed. Most mutants fail to compile, which is the point: both compilers must fail the same way, and neither may crash or hang.
    python3 host/win/shc/test/gen_mutants.py OUT_DIR COUNT SEED seed_dir [seed_dir ...]"""
import os, random, re, sys

out, count, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
files = []
for d in sys.argv[4:]:
    for root, _, names in os.walk(d):
        files += [os.path.join(root, n) for n in names if n.endswith(('.vert', '.frag'))]
files.sort()
rnd = random.Random(seed)
TOK = re.compile(r'[A-Za-z_]\w*|\d+\.\d*(?:[eE][-+]?\d+)?|\.\d+|\d+|==|!=|<=|>=|&&|\|\||\+\+|--|[-+*/]=|\S|\s+')
OPS = ['+', '-', '*', '/', '<', '>', '<=', '>=', '==', '!=', '&&', '||', '=', '+=', '*=', '?', ':', ',', ';', '(', ')', '{', '}', '[', ']', '.']
TYPES = ['float', 'int', 'bool', 'vec2', 'vec3', 'vec4', 'mat2', 'mat3', 'mat4', 'ivec2', 'bvec3', 'void', 'sampler2D']
SWZ = ['.x', '.xy', '.xyz', '.wzyx', '.rgb', '.st', '.a', '.xx', '.yxy']
NUM = ['0', '1', '2', '0.0', '1.0', '0.5', '-1.0', '1e10', '1e-10', '3.14159', '65535.0', '0.1', '1000000.0']
KW = ['if', 'else', 'for', 'while', 'return', 'discard', 'break', 'continue', 'const', 'uniform', 'varying', 'attribute', 'struct', 'void', 'main']


def mutate(src):
    t = TOK.findall(src)
    real = [i for i, x in enumerate(t) if not x.isspace()]
    for _ in range(rnd.choice((1, 1, 1, 2, 3))):
        i = rnd.choice(real)
        k = rnd.randrange(9)
        if k == 0: t[i] = ''
        elif k == 1: t[i] = rnd.choice(OPS)
        elif k == 2: t[i] = rnd.choice(TYPES)
        elif k == 3: t[i] = t[rnd.choice(real)]
        elif k == 4: t[i] += rnd.choice(SWZ)
        elif k == 5: t[i] = rnd.choice(NUM)
        elif k == 6: t[i] = rnd.choice(KW)
        elif k == 7:
            j = rnd.choice(real)
            t[i], t[j] = t[j], t[i]
        else: t[i] = t[i] + ' ' + t[i]
    return ''.join(t)


os.makedirs(out, exist_ok=True)
sources = [(os.path.splitext(f)[1], open(f, newline='').read()) for f in files]
sources = [(e, s.replace('\r\n', '\n')) for e, s in sources if s.strip() and len(s) < 6000]
for n in range(count):
    ext, src = rnd.choice(sources)
    with open(os.path.join(out, 'm%05d%s' % (n, ext)), 'w', newline='\n') as f:
        f.write(mutate(src))
print(len(sources), 'seeds,', count, 'mutants in', out)
