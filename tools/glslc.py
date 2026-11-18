#!/usr/bin/env python3
"""glslc — GLSL ES 1.00 -> gpu2 shader ISA compiler.

Compiles a practical subset of the OpenGL ES 2.0 shading language (GLSL ES 1.00) to the gpu2
vec4 ISA, emitting .sasm text that tools/shader_ref.py assembles to iram words. Targets the
NATIVE write-mask ISA extension (OP.xyzw) + source modifiers (-r/!r), so vector construction and
masked/swizzled writes are single instructions; swizzle READS use DP4-with-basis extraction
(the 32-bit ISA has no source swizzle). Literals / basis vectors live in a compiler-managed uniform pool
appended after the user uniforms; compile() returns the pool so the host can upload it.

isa64=True (compile_stage / --isa64): the 64-bit swizzling ISA of board B's scalar core
(assemble with tools/isa64.py). Swizzle reads and masked / permuted writes are one
MOV with a per-operand swizzle (no basis constants, no DP4 extraction), dot(vec2) is a 2-step chain,
?: is the exact SEL, int / is IDIV, floor / ceil are FLR / CEIL, sqrt(0) = 0 and length(0) = 0 via SEL,
texture bias / LOD modes and cube maps are passed to the TMU (.w = bias / lod), gl_PointCoord is LDV 10.
Scalars (float / int / bool values and the constant pool's scalar constants) are PACKED four to a register on
this ISA: the code generator still treats every scalar as a broadcast vec4 (its lanes are equal by
construction), and the register allocator gives each scalar virtual register one COMPONENT of a physical one;
the operand rewriter then writes a scalar destination as `rN.c` (one micro-op) and reads it as the splat `rN.c`
(scalar loads read `.x` of their uniform / varying slot). A scalar op costs one micro-op instead of four; the
vec4 lowering below is unchanged (the 32-bit ISA has no per-component issue).

Value model: every value is a vec4 register + a GLSL type. A `float`/`bool`/`int` is BROADCAST to
all 4 lanes; vecN uses lanes 0..N-1; matN is a list of N column registers. The ISA is component-
wise, so broadcast floats make scalar math fall out for free.

Coverage (documented, extensible):
  types    float vec2 vec3 vec4  mat2 mat3 mat4  bool int  sampler2D samplerCube sampler3D (GL_OES_texture_3D)  void
  decls    attribute uniform varying const, locals, function defs (inlined)
  exprs    + - * /  unary -  swizzle(read+write)  constructors  indexing(mat col, const)  calls
           comparisons  ?: (uniform/const condition)  assignment (= += -= *= /=)
  control  if/else (-> IF/ELSE/ENDIF)   for with constant bounds (unrolled)
  special  gl_Position gl_PointSize (isa64 vertex core: STO 1) gl_FragColor gl_FragCoord gl_FrontFacing gl_PointCoord
  builtins radians degrees sin cos tan asin acos atan  pow exp log exp2 log2 sqrt inversesqrt
           abs sign floor ceil fract mod min max clamp mix step smoothstep
           length distance dot cross normalize reflect refract faceforward  matrixCompMult
           texture2D texture2DProj textureCube texture3D texture3DProj  dFdx dFdy fwidth
Not yet: while/do, non-const-bound loops, arrays of samplers, structs, per-fragment discard cond
         beyond `if(cond) discard;`. These raise a clear CompileError.
"""
import sys, os, re, math
sys.path.insert(0, os.path.dirname(__file__))

class CompileError(Exception): pass

# ---------------- types ----------------
SCALAR = {'float':1,'int':1,'bool':1}
VEC = {'vec2':2,'vec3':3,'vec4':4,'bvec2':2,'bvec3':3,'bvec4':4,'ivec2':2,'ivec3':3,'ivec4':4}
MAT = {'mat2':2,'mat3':3,'mat4':4}
SAMPLERS=('sampler2D','samplerCube','sampler3D')
def is_vec(t): return t in VEC
def is_mat(t): return t in MAT
def ncomp(t): return SCALAR.get(t) or VEC.get(t) or 4      # scalar=1, vecN=N
def is_scalar(t): return t in SCALAR
def _cidx(c): return 'xyzw'.index(c) if c in 'xyzw' else 'rgba'.index(c) if c in 'rgba' else 'stpq'.index(c)

# ---------------- preprocessor ----------------
# GLSL ES 1.00 §3.4, which is C++'s: preprocessing tokens, macros replaced token by token (a token remembers the macros it came out
# of, so none expands inside itself), #if expressions of ints with C's precedence. A token is [kind, text, line, ws, hide]: kind
# 'id' 'num' 'op' 'other', line its physical line, ws white space (or a comment) before it, hide the macros it came out of.
_PP_TOK = re.compile(r"""
   (?P<id>[A-Za-z_]\w*)
 | (?P<num>\.?\d(?:[eE][+-]|[\w.])*)
 | (?P<op><<=|>>=|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||\^\^|\+=|-=|\*=|/=|%=|&=|\^=|\|=|\#\#|[-+*/%<>=!&|^~?:.,;(){}\[\]\#])
 | (?P<other>.)
""", re.VERBOSE | re.S)
_PP_WS = ' \t\r\f\v'

def _pp_lines(src):
    """the source as logical lines (first physical line, tokens) and its physical line count: a comment is white space, so one
    that runs over lines joins them"""
    lines=[]; cur=[]; i=0; n=len(src); line=1; start=1; ws=False
    while i<n:
        c=src[i]
        if c=='\n':
            lines.append((start,cur)); cur=[]; line+=1; start=line; ws=False; i+=1; continue
        if c in _PP_WS: ws=True; i+=1; continue
        if src.startswith('//',i):
            while i<n and src[i]!='\n': i+=1
            ws=True; continue
        if src.startswith('/*',i):
            j=src.find('*/',i+2)
            if j<0: raise CompileError("a comment does not end")
            line+=src.count('\n',i,j); i=j+2; ws=True; continue
        m=_PP_TOK.match(src,i)
        cur.append([m.lastgroup, m.group(), line, ws, frozenset()]); ws=False; i=m.end()
    lines.append((start,cur))
    return lines, line

def _pp_text(toks):
    return ''.join((' ' if k else '')+t[1] for k,t in enumerate(toks))

def _pp_int(v):
    """an integer literal of a #if, None if the token is none"""
    if re.fullmatch(r'0[xX][0-9a-fA-F]+', v): return int(v,16)
    if re.fullmatch(r'0[0-7]*', v): return int(v,8)
    if re.fullmatch(r'[1-9]\d*', v): return int(v)
    return None

def _pp_w32(x): return ((x+0x80000000) & 0xFFFFFFFF) - 0x80000000     # the ints of a #if: 32 bits, two's complement

EXTENSIONS=('GL_OES_standard_derivatives','GL_EXT_shader_texture_lod','GL_EXT_frag_depth','GL_OES_texture_3D')   # fragment language; the library advertises the same list
EXT_ANY=('GL_OES_texture_3D',)   # the vertex language has these too: a sampler there compiles, and the link refuses it
_PP_LEVELS=(('||',),('&&',),('|',),('^',),('&',),('==','!='),('<','>','<=','>='),('<<','>>'),('+','-'),('*','/','%'))

def preprocess(src, stage='vs', exts=None, strict=False):
    """GLSL ES 1.00's preprocessor: object- and function-like #define, #undef, #if / #ifdef / #ifndef / #elif / #else / #endif,
    #error, #pragma, #extension (exts collects the ones enabled), #version (100, first), #line, __LINE__ __FILE__ __VERSION__
    GL_ES. strict (a program from an application): every error §3.4 names; otherwise what cannot be read is let pass."""
    def err(msg):
        raise CompileError(msg)
    lines,nlines=_pp_lines(src)
    defines={}                                         # name -> (params | None, replacement tokens)
    def const(name, v): defines[name]=(None,[['num',v,0,False,frozenset()]])
    const('GL_ES','1'); const('__VERSION__','100')
    if stage=='fs':
        const('GL_FRAGMENT_PRECISION_HIGH','1')      # highp exists in this fragment language: FP24 meets it
        for x in EXTENSIONS: const(x,'1')
    for x in EXT_ANY: const(x,'1')
    fixed=('__LINE__','__FILE__','__VERSION__','GL_ES')      # predefined: no #define or #undef takes them
    st={'lineoff':0, 'file':0}

    def call_end(toks, i):
        """toks[i] names a function-like macro: the index of its '(' and of the ')' that closes it; None when no '(' follows,
        'open' when the call does not end in toks"""
        j=i+1
        if j>=len(toks): return 'open'
        if toks[j][1]!='(': return None
        depth=0
        for k in range(j,len(toks)):
            if toks[k][1]=='(': depth+=1
            elif toks[k][1]==')':
                depth-=1
                if depth==0: return j,k
        return 'open'

    def expand(toks, ifexpr=False):
        """macro replacement of a token list; ifexpr: `defined X` and `defined ( X )` become 1 or 0, also where a
        replacement brings them"""
        toks=[list(t) for t in toks]; out=[]; i=0
        while i<len(toks):
            t=toks[i]; k,v=t[0],t[1]
            if ifexpr and k=='id' and v=='defined':
                j=i+1; paren=j<len(toks) and toks[j][1]=='('
                if paren: j+=1
                if j>=len(toks) or toks[j][0]!='id': err("defined needs a macro name")
                nm=toks[j][1]; j+=1
                if paren:
                    if j>=len(toks) or toks[j][1]!=')': err("defined( needs a )")
                    j+=1
                out.append(['num','1' if nm in defines or nm in ('__LINE__','__FILE__') else '0',t[2],t[3],frozenset()]); i=j; continue
            if k!='id' or v in t[4]:
                out.append(t); i+=1; continue
            if v=='__LINE__':
                out.append(['num',str(t[2]+st['lineoff']),t[2],t[3],frozenset()]); i+=1; continue
            if v=='__FILE__':
                out.append(['num',str(st['file']),t[2],t[3],frozenset()]); i+=1; continue
            if v not in defines:
                out.append(t); i+=1; continue
            params,body=defines[v]
            if params is None:
                rep=[[b[0],b[1],t[2],(t[3] if n==0 else b[3]),t[4]|{v}] for n,b in enumerate(body)]
                toks[i:i+1]=rep; continue
            ce=call_end(toks,i)
            if not isinstance(ce,tuple):
                out.append(t); i+=1; continue
            lp,rp=ce
            args=[[]]; depth=0
            for a in toks[lp+1:rp]:
                if a[1]=='(': depth+=1
                elif a[1]==')': depth-=1
                if a[1]==',' and depth==0: args.append([]); continue
                args[-1].append(a)
            if args==[[]] and not params: args=[]
            if len(args)!=len(params):
                if strict: err(f"macro {v} takes {len(params)} arguments, not {len(args)}")
                args=(args+[[] for _ in params])[:len(params)]
            args=[expand(a) for a in args]
            hide=t[4]|{v}; line=toks[rp][2]; rep=[]
            for n,b in enumerate(body):
                ws=t[3] if n==0 else b[3]
                if b[0]=='id' and b[1] in params:
                    a=args[params.index(b[1])]
                    rep.extend([x[0],x[1],x[2],(ws if m==0 else x[3]),x[4]|hide] for m,x in enumerate(a))
                else: rep.append([b[0],b[1],line,ws,hide])
            toks[i:rp+1]=rep
        return out

    def eval_if(toks):
        """a #if / #elif expression: 1 or 0"""
        x=expand(toks, True); pos=[0]
        if not x: err("#if needs an expression")
        def peek(): return x[pos[0]][1] if pos[0]<len(x) else None
        def unary(ev):
            if pos[0]>=len(x): err("#if: the expression ends early")
            k,v=x[pos[0]][0],x[pos[0]][1]; pos[0]+=1
            if v in ('+','-','~','!'):
                a=unary(ev)
                return a if v=='+' else _pp_w32(-a) if v=='-' else _pp_w32(~a) if v=='~' else int(not a)
            if v=='(':
                a=binary(0,ev)
                if peek()!=')': err("#if: ) expected")
                pos[0]+=1; return a
            if k=='num':
                a=_pp_int(v)
                if a is None: err(f"#if: {v} is no integer")
                return _pp_w32(a)
            if k=='id':
                if ev: err(f"#if: {v!r} is not defined")
                return 0
            err(f"#if: unexpected {v!r}")
        def binary(lvl, ev):
            if lvl==len(_PP_LEVELS): return unary(ev)
            a=binary(lvl+1,ev)
            while peek() in _PP_LEVELS[lvl]:
                op=peek(); pos[0]+=1
                if op=='||': b=binary(lvl+1, ev and not a); a=int(bool(a) or bool(b)); continue
                if op=='&&': b=binary(lvl+1, ev and bool(a)); a=int(bool(a) and bool(b)); continue
                b=binary(lvl+1,ev)
                if op in ('/','%'):
                    if b==0:
                        if ev: err("#if: division by zero")
                        a=0; continue
                    q=abs(a)//abs(b)
                    if (a<0)!=(b<0): q=-q
                    a=_pp_w32(q) if op=='/' else _pp_w32(a-b*q); continue
                if op in ('<<','>>'): a=_pp_w32(a<<(b&31)) if op=='<<' else a>>(b&31); continue
                a={'==':int(a==b),'!=':int(a!=b),'<':int(a<b),'>':int(a>b),'<=':int(a<=b),'>=':int(a>=b),
                   '|':a|b,'^':a^b,'&':a&b,'+':_pp_w32(a+b),'-':_pp_w32(a-b),'*':_pp_w32(a*b)}[op]
            return a
        r=binary(0,True)
        if pos[0]!=len(x): err(f"#if: unexpected {x[pos[0]][1]!r} after the expression")
        return int(bool(r))
    def if_value(toks):
        try: return eval_if(toks)
        except CompileError:
            if strict: raise
            return 0

    def name_of(toks, what):
        """the one name a directive takes"""
        if strict and (len(toks)!=1 or toks[0][0]!='id'): err(f"#{what} takes one macro name")
        return toks[0][1] if toks else ''
    def no_more(toks, what):
        if strict and toks: err(f"#{what}: unexpected {toks[0][1]!r}")
    def same_body(a, b):
        return [(x[0],x[1],x[3] if n else False) for n,x in enumerate(a)]==[(x[0],x[1],x[3] if n else False) for n,x in enumerate(b)]

    def define(toks):
        if not toks or toks[0][0]!='id': err("#define needs a macro name")
        nm=toks[0][1]; params=None; i=1
        if i<len(toks) and toks[i][1]=='(' and not toks[i][3]:          # a function-like macro: '(' right after the name
            params=[]; i+=1
            if i<len(toks) and toks[i][1]==')': i+=1
            else:
                while True:
                    if i>=len(toks) or toks[i][0]!='id':
                        if strict: err(f"#define {nm}: a parameter must be a name")
                        while i<len(toks) and toks[i][1]!=')': i+=1
                        i+=1; break
                    if strict and toks[i][1] in params: err(f"#define {nm}: the parameter {toks[i][1]} twice")
                    params.append(toks[i][1]); i+=1
                    if i<len(toks) and toks[i][1]==',': i+=1; continue
                    if i<len(toks) and toks[i][1]==')': i+=1; break
                    if strict: err(f"#define {nm}: ) expected")
                    break
        body=toks[i:]
        if strict and (nm.startswith('GL_') or nm in fixed): err(f"macro name {nm!r} is reserved")
        if strict and nm in defines and (defines[nm][0]!=params or not same_body(defines[nm][1],body)): err(f"macro {nm} is defined again otherwise")
        defines[nm]=(params,[list(b) for b in body])

    out=['']*nlines; stack=[]                          # (parent active, this group taken, a group of the chain taken, #else met)
    begun=False                                        # a directive or code went before: #version comes first
    code=False                                         # code went before: #extension comes first
    li=0
    while li<len(lines):
        start,toks=lines[li]; li+=1
        active=all(s[1] for s in stack)
        if toks and toks[0][1]=='#':
            d=toks[1:]
            key=d[0][1] if d and d[0][0]=='id' else ''
            rest=d[1:]
            if d and not key and strict and active: err(f"bad directive #{d[0][1]}")
            if key in ('if','ifdef','ifndef'):
                if not active: t=False
                elif key=='if': t=bool(if_value(rest))
                else:
                    nm=name_of(rest, key); t=(nm in defines or nm in ('__LINE__','__FILE__'))==(key=='ifdef')
                stack.append((active, t, t, False))
            elif key=='elif':
                if not stack: err("#elif without #if")
                pa,_,any_,el=stack[-1]
                if el and strict: err("#elif after #else")
                t=pa and not any_ and bool(if_value(rest))
                stack[-1]=(pa, t, any_ or t, el)
            elif key=='else':
                if not stack: err("#else without #if")
                pa,_,any_,el=stack[-1]
                if el and strict: err("#else after #else")
                no_more(rest, 'else')
                stack[-1]=(pa, pa and not any_, True, True)
            elif key=='endif':
                if not stack: err("#endif without #if")
                no_more(rest, 'endif')
                stack.pop()
            elif active:
                if key=='version':
                    v=_pp_text(rest)
                    if begun or v!='100': err('#version must come first' if v=='100' else f'#version {v} is not supported: GLSL ES 1.00 only')
                elif key=='define': define(rest)
                elif key=='undef':
                    nm=name_of(rest, 'undef')
                    if strict and (nm.startswith('GL_') or nm in fixed): err(f"macro name {nm!r} is reserved")
                    defines.pop(nm, None)
                elif key=='error': err('#error '+_pp_text(rest))
                elif key=='extension':
                    m=re.fullmatch(r'(\w+) : (\w+)', _pp_text(rest))
                    if not m or m.group(2) not in ('require','enable','warn','disable'): err(f"bad #extension: {_pp_text(rest)}")
                    if strict and code: err("#extension comes before the code")
                    nm,beh=m.group(1),m.group(2)
                    if nm=='all':
                        if beh in ('require','enable'): err(f"#extension all : {beh}")
                        if exts is not None:
                            if beh=='disable': exts.clear()
                            else: exts.update(EXTENSIONS if stage=='fs' else EXT_ANY)
                    elif not (nm in EXT_ANY or (stage=='fs' and nm in EXTENSIONS)):
                        if beh=='require': err(f"extension {nm} is not supported")
                    elif exts is not None:
                        if beh=='disable': exts.discard(nm)
                        else: exts.add(nm)
                elif key=='line':
                    x=expand(rest); vals=[min(int(t[1]),0x7FFFFFFF) if t[0]=='num' and re.fullmatch(r'\d+', t[1]) else None for t in x]
                    if not 1<=len(vals)<=2 or None in vals: err(f"bad #line: {_pp_text(rest)}")
                    nxt=lines[li][0] if li<len(lines) else nlines+1
                    st['lineoff']=vals[0]-nxt                                 # the next line is line vals[0]
                    if len(vals)==2: st['file']=vals[1]
                elif key=='pragma' or not d: pass
                elif strict: err(f"unknown directive #{key}")
            if key: begun=True
            continue
        if not active: continue
        if toks: begun=True; code=True
        while li<len(lines) and not (lines[li][1] and lines[li][1][0][1]=='#'):    # a call of a function-like macro may go on over lines
            ids=[n for n,t in enumerate(toks) if t[0]=='id' and t[1] in defines and defines[t[1]][0] is not None]
            if not any(call_end(toks,n)=='open' for n in ids): break
            nxt=[list(t) for t in lines[li][1]]
            if nxt: nxt[0][3]=True
            toks=toks+nxt; li+=1
        out[start-1]=_pp_text(expand(toks))
    if stack: err("#if without #endif")
    return '\n'.join(out)

# ---------------- lexer ----------------
KEYWORDS = {'attribute','uniform','varying','const','void','float','int','bool',
            'vec2','vec3','vec4','bvec2','bvec3','bvec4','ivec2','ivec3','ivec4',
            'mat2','mat3','mat4','sampler2D','samplerCube',
            'if','else','for','return','discard','true','false','in','out','inout','precision',
            'highp','mediump','lowp','struct','while','do','continue','break','invariant'}
RESERVED = {'asm','class','union','enum','typedef','template','this','packed','goto','switch','default','inline','noinline',
            'volatile','public','static','extern','external','interface','flat','long','short','double','half','fixed',
            'unsigned','superp','input','output','hvec2','hvec3','hvec4','dvec2','dvec3','dvec4','fvec2','fvec3','fvec4',
            'sampler1D','sampler3D','sampler1DShadow','sampler2DShadow','sampler2DRect','sampler3DRect',
            'sampler2DRectShadow','sizeof','cast','namespace','using'}      # §3.6: a program that names one is in error
TOKEN_RE = re.compile(r"""
   (?P<ws>\s+)
 | (?P<comment>//[^\n]*|/\*.*?\*/)
 | (?P<float>\d+\.\d*([eE][+-]?\d+)?|\.\d+([eE][+-]?\d+)?|\d+[eE][+-]?\d+|\d+\.)
 | (?P<hex>0[xX][0-9a-fA-F]+)
 | (?P<int>\d+)
 | (?P<ident>[A-Za-z_]\w*)
 | (?P<op><<|>>|<=|>=|==|!=|&&|\|\||\+\+|--|\+=|-=|\*=|/=|\^\^|[-+*/%<>=!&|^~?:.,;(){}\[\]])
""", re.VERBOSE|re.DOTALL)

class IntLit(float):                               # a value of type int (it stays a float for everything else: ('num', 3.0) == ('num', IntLit(3)))
    pass
class BoolLit(float):                              # true / false
    pass
class OneLit(float):                               # the 1 of ++x: of the type of x
    pass

def lex(src, exts=()):
    toks=[]; i=0
    while i < len(src):
        m = TOKEN_RE.match(src, i)
        if not m: raise CompileError(f"lex error at {src[i:i+16]!r}")
        i = m.end()
        k = m.lastgroup
        if k in ('ws','comment'): continue
        v = m.group()
        if k=='ident' and (v in KEYWORDS or v=='sampler3D'): toks.append(('kw', v))    # 3D textures with or without GL_OES_texture_3D
        elif k=='ident': toks.append(('id', v))
        elif k=='float': toks.append(('num', float(v)))
        elif k=='hex': toks.append(('num', IntLit(int(v,16))))
        elif k=='int':
            if len(v)>1 and v[0]=='0':
                if not all(c in '01234567' for c in v): raise CompileError(f"bad octal literal {v}")
                toks.append(('num', IntLit(int(v,8))))
            else: toks.append(('num', IntLit(float(v))))
        else: toks.append(('op', v))
    toks.append(('eof',''))
    return toks

# ---------------- AST (tuples: (kind, ...)) ----------------
class Parser:
    def __init__(self, toks, strict=False):
        self.t=toks; self.i=0; self.strict=strict
        self.sscope=[{}]                           # struct names by scope: name -> the name its definition goes by (an inner struct may hide an outer one)
        self.nstruct=0                             # definitions so far: a hidden name and an anonymous struct get a unique one
        self.hoisted=[]                            # struct definitions met inside functions: every struct is global to the code generator
        self.prec={}                               # id(declaration) -> its precision qualifier (Sema: the fragment language has no default for float)
        self.parr={}                               # id(parameter) -> its array size expression
        self.fragdata=set()                        # id of the gl_FragColor nodes that were gl_FragData[0] (Sema: a shader writes one of the two)
    def peek(self): return self.t[self.i]
    def next(self): tok=self.t[self.i]; self.i+=1; return tok
    def at(self, v): k,val=self.peek(); return val==v
    def eat(self, v):
        k,val=self.peek()
        if val!=v: raise CompileError(f"expected {v!r} got {val!r}")
        self.i+=1; return val
    def accept(self, v):
        if self.at(v): self.i+=1; return True
        return False
    def ident(self):
        """the name a declaration gives: an identifier (§3.7); a program from an application that names a keyword or a number fails"""
        k,v=self.next()
        if k=='num' or (self.strict and k!='id'): raise CompileError("expected a name, got " + ('<number>' if k=='num' else repr(v)))
        return v

    # ---- top level ----
    def parse(self):
        decls=[]
        if self.strict:
            for k,v in self.t:
                if k=='id' and v in RESERVED: raise CompileError(f"{v!r} is a reserved word")
        while self.peek()[0]!='eof':
            decls.extend(self.top_decl())          # top_decl returns a list (comma-separated declarators)
        return decls
    def sname(self, v):
        """the name a struct type goes by, None if v names no struct"""
        for sc in reversed(self.sscope):
            if v in sc: return sc[v]
        return None
    def type_name(self):
        v=self.next()[1]; return self.sname(v) or v
    def parse_struct(self):
        self.next()                              # 'struct'
        self.nstruct+=1; name=None
        if not self.at('{'): name=self.ident()
        self.eat('{'); members=[]
        while not self.at('}'):
            p=self.skip_precision()
            mty=self.type_name()
            while True:                          # comma-separated members of the same type
                mname=self.ident(); marr=None
                if self.accept('['): marr=self.assign(); self.eat(']')
                m=(mty, mname, marr); self.prec[id(m)]=p; members.append(m)
                if not self.accept(','): break
            self.eat(';')
        self.eat('}')
        if self.strict and name in self.sscope[-1]: raise CompileError(f"struct {name} is already defined")
        real=name if name is not None and self.sname(name) is None else f'{name or "_anon"}@{self.nstruct}'
        if name is not None: self.sscope[-1][name]=real
        return real, members
    def decl_names(self, make):
        """the declarators after a type: name[size] = init, ... ; make(name, size, init) builds each"""
        out=[]
        while True:
            name=self.ident(); arr=None; init=None
            if self.accept('['): arr=self.assign(); self.eat(']')   # assign(): a comma separates declarators
            if self.accept('='): init=self.assign()
            out.append(make(name, arr, init))
            if not self.accept(','): break
        self.eat(';'); return out
    def top_decl(self):
        k,v=self.peek()
        if v=='precision':                       # precision qualifier: a default for the type
            self.next(); q=self.next()[1]; ty=self.next()[1]; self.eat(';')
            return [('precision', q, ty)]
        if v=='struct':
            sname,members=self.parse_struct(); out=[('struct', sname, members)]
            if self.accept(';'): return out
            return out+self.decl_names(lambda n,a,i: ('gdecl','var',sname,n,a,i))    # struct S{..} a, b;
        if v=='invariant':                       # every program here is computed one way: nothing to do
            self.next()
            if not self.at('varying'):           # invariant gl_Position, v; : names declared before
                while not self.accept(';'):
                    k,w=self.next()
                    if self.strict and k!='id' and w!=',': raise CompileError("invariant qualifies varyings and the outputs gl_Position and gl_PointSize")
                return []
            k,v=self.peek()
        if v in ('attribute','uniform','varying','const'):
            qual=self.next()[1]
            p=self.skip_precision(); out=[]
            if self.at('struct'):                          # uniform struct S {..} u;
                ty,members=self.parse_struct(); out.append(('struct', ty, members))
            else: ty=self.type_name()
            def make(n,a,i):                               # comma-separated declarators: uniform vec3 a, b;
                d=('gdecl', qual, ty, n, a, i); self.prec[id(d)]=p; return d
            return out+self.decl_names(make)
        # function def or global: <type> name ( ... ) { ... }  OR <type> name;
        p=self.skip_precision()
        ty=self.type_name(); name=self.ident()
        if self.at('('):
            params=self.params()
            if self.accept(';'): d=('proto', ty, name, params)
            else: d=('func', ty, name, params, self.block())
            self.prec[id(d)]=p
            return [d]
        out=[]                                             # a plain global: kept with the outputs, which main and every function see
        while True:
            arr=None; init=None
            if self.accept('['): arr=self.assign(); self.eat(']')
            if self.accept('='): init=self.assign()
            d=('gdecl', 'var', ty, name, arr, init); self.prec[id(d)]=p; out.append(d)
            if not self.accept(','): break
            name=self.ident()
        self.eat(';'); return out
    def skip_precision(self):
        q=None
        while self.peek()[1] in ('highp','mediump','lowp'): q=self.next()[1]
        return q
    def params(self):
        self.eat('(')
        ps=[]
        if self.at('void') and self.t[self.i+1][1]==')': self.next()   # f(void)
        if not self.at(')'):
            while True:
                q='in'; c=False; pr=None; last=-1
                while self.peek()[1] in ('in','out','inout','const','highp','mediump','lowp'):
                    t=self.next()[1]
                    o=0 if t=='const' else 1 if t in ('in','out','inout') else 2      # §4.6.4: const, then in / out / inout, then a precision
                    if self.strict and o<=last: raise CompileError(f"parameter qualifier {t!r} out of order")
                    last=o
                    if t in ('in','out','inout'): q=t
                    elif t=='const': c=True
                    else: pr=t
                ty=self.type_name(); nm=''
                if not self.at(',') and not self.at(')') and not self.at('['): nm=self.ident()   # a prototype may leave the name out
                arr=None
                if self.accept('['): arr=self.assign(); self.eat(']')
                p=(q,ty,nm); self.prec[id(p)]=pr; self.parr[id(p)]=(arr,c); ps.append(p)
                if not self.accept(','): break
        self.eat(')')
        return ps
    # ---- statements ----
    def scoped(self, s):
        """§6.2: the statement of an if or an else is a scope of its own: a declaration there goes in a block"""
        return ('block', [s]) if s[0] in ('ldecl','multi','lstruct') else s
    def block(self):
        self.eat('{'); stmts=[]; self.sscope.append({})
        while not self.at('}'): stmts.append(self.stmt())
        self.eat('}'); self.sscope.pop(); return ('block', stmts)
    def stmt(self):
        k,v=self.peek()
        if v=='{': return self.block()
        if v==';': self.next(); return ('block', [])      # the empty statement
        if v=='precision':
            self.next(); q=self.next()[1]; ty=self.next()[1]; self.eat(';')
            return ('precision', q, ty)
        if v=='struct':                                   # a struct defined in a function: known to every function, its name to this scope
            sname,members=self.parse_struct(); self.hoisted.append(('struct', sname, members)); d=('lstruct', sname, members)
            if self.accept(';'): return d
            return ('multi', [d]+self.decl_names(lambda n,a,i: ('ldecl', False, sname, n, i, a)))
        if v=='if':
            self.next(); self.eat('('); c=self.expr(); self.eat(')')
            th=self.scoped(self.stmt()); el=None
            if self.accept('else'): el=self.scoped(self.stmt())
            return ('if', c, th, el)
        if v=='for':
            self.next(); self.eat('(')
            self.sscope.append({})
            init=self.stmt()                      # decl or expr; consumes ';'
            cond=('num', BoolLit(1.0)) if self.at(';') else self.expr()   # for (;;): true
            self.eat(';')
            post=None if self.at(')') else self.expr()
            self.eat(')')
            body=self.stmt(); self.sscope.pop()
            return ('for', init, cond, post, body)
        if v=='while':
            self.next(); self.eat('('); c=self.expr(); self.eat(')'); body=self.stmt()
            return ('while', c, body)
        if v=='do':
            self.next(); body=self.stmt()
            self.eat('while'); self.eat('('); c=self.expr(); self.eat(')'); self.eat(';')
            return ('dowhile', c, body)
        if v=='return':
            self.next(); e=None
            if not self.at(';'): e=self.expr()
            self.eat(';'); return ('return', e)
        if v=='discard':
            self.next(); self.eat(';'); return ('discard',)
        if v=='break':    self.next(); self.eat(';'); return ('break',)
        if v=='continue': self.next(); self.eat(';'); return ('continue',)
        if v in ('const','highp','mediump','lowp') or v in VEC or v in MAT or v in ('float','int','bool') or v in SAMPLERS or self.sname(v) is not None:
            if self.t[self.i+1][1]=='(':           # a constructor starts an expression: float(x) + 1.0;
                e=self.expr(); self.eat(';'); return ('expr', e)
            isc = self.accept('const')
            p=self.skip_precision()
            ty=self.type_name()
            def make(n,a,i):                               # comma-separated: float a=1.0, b, c=3.0;
                d=('ldecl', isc, ty, n, i, a); self.prec[id(d)]=p; return d
            out=self.decl_names(make)
            return out[0] if len(out)==1 else ('multi', out)
        e=self.expr(); self.eat(';'); return ('expr', e)
    # ---- expressions (precedence climbing) ----
    def expr(self):                                    # comma/sequence operator (lowest precedence)
        e=self.assign()
        if self.at(','):
            seq=[e]
            while self.accept(','): seq.append(self.assign())
            return ('seq', seq)
        return e
    def assign(self):
        left=self.ternary()
        k,v=self.peek()
        if v in ('=','+=','-=','*=','/='):
            self.next(); right=self.assign()
            return ('assign', v, left, right)
        return left
    def ternary(self):
        c=self.binary(0)
        if self.accept('?'):
            a=self.assign(); self.eat(':'); b=self.assign()
            return ('ternary', c, a, b)
        return c
    PREC=[['||'],['^^'],['&&'],['==','!='],['<','>','<=','>='],['+','-'],['*','/','%']]
    def binary(self, lvl):
        if lvl>=len(self.PREC): return self.unary()
        left=self.binary(lvl+1)
        while self.peek()[1] in self.PREC[lvl]:
            op=self.next()[1]; right=self.binary(lvl+1)
            left=('bin', op, left, right)
        return left
    def unary(self):
        k,v=self.peek()
        if v in ('-','!','+'):
            self.next(); return ('unary', v, self.unary())
        if v in ('++','--'):                       # ++x is x += 1, and its value
            self.next(); return ('assign', '+=' if v=='++' else '-=', self.unary(), ('num',OneLit(1.0)))
        return self.postfix()
    def postfix(self):
        e=self.primary()
        while True:
            k,v=self.peek()
            if v=='.':
                self.next(); field=self.next()[1]; e=('swizzle', e, field)
            elif v=='[':
                self.next(); idx=self.expr(); self.eat(']')
                if e==('var','gl_FragData') and idx==('num',0.0):           # MAX_DRAW_BUFFERS is 1
                    e=tuple(['var','gl_FragColor']); self.fragdata.add(id(e))
                else: e=('index', e, idx)
            elif v=='(' and e[0]=='var':          # function/constructor call
                args=self.arglist(); e=('call', self.sname(e[1]) or e[1], args)
            elif v in ('++','--'):
                self.next(); e=('postop', e, v)
            else: break
        return e
    def arglist(self):
        self.eat('('); args=[]
        if not self.at(')'):
            while True:
                args.append(self.assign())
                if not self.accept(','): break
        self.eat(')'); return args
    def primary(self):
        k,v=self.peek()
        if k=='num': self.next(); return ('num', v)
        if v=='true': self.next(); return ('num', BoolLit(1.0))
        if v=='false': self.next(); return ('num', BoolLit(0.0))
        if v in VEC or v in MAT:                   # constructor: vec3(...) etc.
            self.next(); args=self.arglist(); return ('call', v, args)
        if k=='id' or v in ('float','int','bool'):
            self.next(); return ('var', v)
        if v=='(':
            self.next(); e=self.expr(); self.eat(')'); return e
        raise CompileError(f"unexpected token {v!r} in expression")


# ---------------- constant expressions ----------------
_FOLD1={'radians':math.radians,'degrees':math.degrees,'sin':math.sin,'cos':math.cos,'tan':math.tan,'asin':math.asin,
        'acos':math.acos,'atan':math.atan,'exp':math.exp,'log':math.log,'sqrt':math.sqrt,'abs':abs,'floor':math.floor,
        'ceil':math.ceil,'exp2':lambda x:2.0**x,'log2':math.log2,'inversesqrt':lambda x:1.0/math.sqrt(x),
        'fract':lambda x:x-math.floor(x),'sign':lambda x:float((x>0)-(x<0))}
_FOLD2={'pow':math.pow,'min':min,'max':max,'atan':math.atan2,'mod':lambda x,y:x-y*math.floor(x/y),
        'step':lambda e,x:0.0 if x<e else 1.0}
def _ints(a,b): return isinstance(a,IntLit) and isinstance(b,IntLit)
def fold_expr(e, get):
    """the number a constant expression comes to, None if it is not one: a value of type int is an IntLit.
    get(name): the value of a scalar known at compile time, or None."""
    k=e[0]
    if k=='num': return e[1]
    if k=='var': return get(e[1])
    if k=='unary':
        a=fold_expr(e[2],get)
        if a is None: return None
        if e[1]=='-': return IntLit(-a) if isinstance(a,IntLit) else -a
        if e[1]=='+': return a
        if e[1]=='!': return float(not a)
        return None
    if k=='bin':
        a=fold_expr(e[2],get); b=fold_expr(e[3],get)
        if a is None or b is None: return None
        if _ints(a,b) and e[1] in ('+','-','*','/'):
            if e[1]=='/': return IntLit(int(a/b)) if b else IntLit(0.0)          # toward zero
            return IntLit({'+':a+b,'-':a-b,'*':a*b}[e[1]])
        return {'+':a+b,'-':a-b,'*':a*b,'/':(a/b if b else 0.0),'%':(a%b if b else 0.0),
                '<':float(a<b),'>':float(a>b),'<=':float(a<=b),'>=':float(a>=b),
                '==':float(a==b),'!=':float(a!=b),'&&':float(bool(a) and bool(b)),
                '||':float(bool(a) or bool(b)),'^^':float(bool(a)!=bool(b))}.get(e[1])
    if k=='ternary':
        c=fold_expr(e[1],get)
        return None if c is None else fold_expr(e[2] if c else e[3], get)
    if k=='call':
        n=e[1]; a=e[2]
        if n in ('float','int','bool') and len(a)==1:
            v=fold_expr(a[0],get)
            if v is None: return None
            return IntLit(int(v)) if n=='int' else (float(bool(v)) if n=='bool' else float(v))
        try:
            if len(a)==1 and n in _FOLD1:
                v=fold_expr(a[0],get)
                return None if v is None else float(_FOLD1[n](v))
            if len(a)==2 and n in _FOLD2:
                v=fold_expr(a[0],get); w=fold_expr(a[1],get)
                return None if v is None or w is None else float(_FOLD2[n](v,w))
            if len(a)==3 and n=='clamp':
                v=fold_expr(a[0],get); w=fold_expr(a[1],get); x=fold_expr(a[2],get)
                return None if v is None or w is None or x is None else float(min(max(v,w),x))
            if len(a)==3 and n=='mix':
                v=fold_expr(a[0],get); w=fold_expr(a[1],get); x=fold_expr(a[2],get)
                return None if v is None or w is None or x is None else float(v*(1.0-x)+w*x)
        except (ValueError,ZeroDivisionError,OverflowError): return None
    return None

# ---------------- semantic pass: GLSL ES 1.00's types, overload resolution, the errors the language requires ----------------
# strict=True (a program from an application: libGLESv2) reports every error. strict=False (the programs the libraries
# generate, and the hand-written benches) only resolves overloads, and takes what it cannot type as ERR.
ERR=('?',None)
def _fam(b):
    """a basic type's family, the size of its vector"""
    if b in ('float','int','bool'): return b,1
    if b in VEC: return {'v':'float','i':'int','b':'bool'}[b[0]],VEC[b]
    if b in MAT: return 'float',MAT[b]
    return None,0
def _vecof(f,n): return f if n==1 else {'float':'vec','int':'ivec','bool':'bvec'}[f]+str(n)
SIGS=[s.split() for s in """
    G radians G|G degrees G|G sin G|G cos G|G tan G|G asin G|G acos G|G atan G|G atan G G|G pow G G|G exp G|G log G|G exp2 G|G log2 G
    G sqrt G|G inversesqrt G|G abs G|G sign G|G floor G|G ceil G|G fract G|G mod G G|G mod G float|G min G G|G min G float
    G max G G|G max G float|G clamp G G G|G clamp G float float|G mix G G G|G mix G G float|G step G G|G step float G
    G smoothstep G G G|G smoothstep float float G|float length G|float distance G G|float dot G G|vec3 cross vec3 vec3
    G normalize G|G faceforward G G G|G reflect G G|G refract G G float|M matrixCompMult M M
    R lessThan V V|R lessThan I I|R lessThanEqual V V|R lessThanEqual I I|R greaterThan V V|R greaterThan I I
    R greaterThanEqual V V|R greaterThanEqual I I|R equal V V|R equal I I|R equal L L|R notEqual V V|R notEqual I I|R notEqual L L
    bool any L|bool all L|R not L
    vec4 texture2D sampler2D vec2|vec4 texture2D sampler2D vec2 float|vec4 texture2DProj sampler2D vec3|vec4 texture2DProj sampler2D vec3 float
    vec4 texture2DProj sampler2D vec4|vec4 texture2DProj sampler2D vec4 float|vec4 textureCube samplerCube vec3|vec4 textureCube samplerCube vec3 float
    vec4 texture2DLod sampler2D vec2 float|vec4 texture2DProjLod sampler2D vec3 float|vec4 texture2DProjLod sampler2D vec4 float
    vec4 textureCubeLod samplerCube vec3 float
    G dFdx G|G dFdy G|G fwidth G
    vec4 texture2DLodEXT sampler2D vec2 float|vec4 texture2DProjLodEXT sampler2D vec3 float|vec4 texture2DProjLodEXT sampler2D vec4 float
    vec4 textureCubeLodEXT samplerCube vec3 float|vec4 texture2DGradEXT sampler2D vec2 vec2 vec2
    vec4 texture2DProjGradEXT sampler2D vec3 vec2 vec2|vec4 texture2DProjGradEXT sampler2D vec4 vec2 vec2
    vec4 textureCubeGradEXT samplerCube vec3 vec3 vec3
    vec4 texture3D sampler3D vec3|vec4 texture3D sampler3D vec3 float|vec4 texture3DProj sampler3D vec4|vec4 texture3DProj sampler3D vec4 float
    vec4 texture3DLod sampler3D vec3 float|vec4 texture3DProjLod sampler3D vec4 float
""".replace('\n','|').split('|') if s.strip()]
_DERIV=('dFdx','dFdy','fwidth')
_TEXFN={'texture2D','texture2DProj','textureCube','texture2DLod','texture2DProjLod','textureCubeLod','texture3D','texture3DProj','texture3DLod','texture3DProjLod'}
GL_MAX={'gl_MaxVertexAttribs':16,'gl_MaxVertexUniformVectors':224,'gl_MaxVaryingVectors':8,'gl_MaxVertexTextureImageUnits':0,
        'gl_MaxCombinedTextureImageUnits':8,'gl_MaxTextureImageUnits':8,'gl_MaxFragmentUniformVectors':48,'gl_MaxDrawBuffers':1}
DEPTHRANGE=('gl_DepthRangeParameters',[('float','near',None),('float','far',None),('float','diff',None)])

class Sym:
    __slots__=('ty','qual','val')
    def __init__(self, ty, qual='', val=None): self.ty=ty; self.qual=qual; self.val=val     # qual: const attribute uniform varying in out inout ro (read only)
class Fn:
    __slots__=('name','ret','params','sig','defined','node','calls')
    def __init__(self,name,ret,params,sig): self.name=name; self.ret=ret; self.params=params; self.sig=sig; self.defined=False; self.node=None; self.calls=[]

class Sema:
    def __init__(self, stage, parser, exts, strict):
        self.stage=stage; self.p=parser; self.exts=exts; self.strict=strict
        self.scopes=[{}]; self.funcs={}; self.structs={}; self.defprec=[{}]
        self.callmap={}          # id(call node) -> Fn
        self.ints=set()          # id(expression) of the expressions of type int
        self.fnames={}           # id(function node) -> the name the code generator knows it by
        self.depthrange=False    # the program reads gl_DepthRange
        self.cur=None; self.loops=0
        self.gnames={}           # a global name -> 'function', 'variable' or 'struct': §4.2.7, they share one name space
        self.fragout=set()       # gl_FragColor / gl_FragData, the ones the program writes
        g=self.scopes[0]
        if stage=='vs': g.update({'gl_Position':Sym(('vec4',None)),'gl_PointSize':Sym(('float',None))})
        else:
            g.update({'gl_FragCoord':Sym(('vec4',None),'ro'),'gl_FrontFacing':Sym(('bool',None),'ro'),'gl_PointCoord':Sym(('vec2',None),'ro'),
                      'gl_FragColor':Sym(('vec4',None)),'gl_FragData':Sym(('vec4',1))})
            g['gl_FragDepthEXT']=Sym(('float',None))     # the core writes depth, with or without GL_EXT_frag_depth
            if not strict: g['gl_FragDepth']=Sym(('float',None))
        for k,v in GL_MAX.items(): g[k]=Sym(('int',None),'const',IntLit(v))
        g['gl_DepthRange']=Sym(('gl_DepthRangeParameters',None),'uniform')
        self.structs['gl_DepthRangeParameters']=DEPTHRANGE[1]
        self.defprec[0].update({'int':'mediump','sampler2D':'lowp','samplerCube':'lowp'})
        if stage=='vs': self.defprec[0]['float']='highp'; self.defprec[0]['int']='highp'
    def err(self, msg):
        if self.strict: raise CompileError(msg)
        return ERR
    def get(self, name):
        for sc in reversed(self.scopes):
            if name in sc: return sc[name].val
        return None
    def sym(self, name):
        for sc in reversed(self.scopes):
            if name in sc: return sc[name]
        return None
    def declare(self, name, ty, qual='', val=None):
        if name.startswith('gl_'): self.err(f"{name!r}: the gl_ prefix is reserved")
        if name in self.scopes[-1] and name!='': self.err(f"{name!r} is already declared in this scope")
        self.scopes[-1][name]=Sym(ty,qual,val)
    def need_prec(self, base, prec):
        """§4.5.3: the fragment language has no default precision for float"""
        f,_=_fam(base)
        if base in self.structs or f!='float' or prec is not None: return
        for d in reversed(self.defprec):
            if 'float' in d: return
        self.err("no precision for a float type: the fragment language has no default")
    def base_ok(self, b):
        return b in SCALAR or b in VEC or b in MAT or b in self.structs or b in SAMPLERS
    def size(self, arr):
        """an array's size: an integral constant expression above zero"""
        if arr is None: return None
        t=self.ty(arr)
        if t not in (('int',None),ERR): self.err("an array size must be an integer")
        v=fold_expr(arr, self.get)
        if v is None: return self.err("an array size must be a constant expression") and -1
        if v<=0: self.err("an array size must be above zero")
        return int(v)
    def has_sampler(self, b):
        return b in SAMPLERS or any(self.has_sampler(m[0]) for m in self.structs.get(b,()))
    def has_opaque(self, b):
        """a type == cannot take: holds a sampler or an array"""
        if b in SAMPLERS: return True
        for mt,mn,ma in self.structs.get(b,()):
            if ma is not None or self.has_opaque(mt): return True
        return False
    def has_array(self, b):
        """a struct type with an array among its members, at any depth"""
        return any(ma is not None or self.has_array(mt) for mt,mn,ma in self.structs.get(b,()))
    def gname(self, name, kind):
        was=self.gnames.setdefault(name, kind)
        if was!=kind: self.err(f"{name!r} is already declared as a {was}")
    def builtin_takes(self, name, sig):
        """a built-in function of this name takes exactly these parameters"""
        if not any(sg[1]==name for sg in SIGS): return False
        s=self.strict; self.strict=False
        try: return self.ty_builtin(None, name, list(sig))!=ERR
        finally: self.strict=s
    def wrote(self, e):
        """a write to e: §7.2, a shader writes gl_FragColor or gl_FragData, not both"""
        while e[0] in ('swizzle','index'): e=e[1]
        if e[0]=='var' and e[1] in ('gl_FragColor','gl_FragData'):
            self.fragout.add('gl_FragData' if e[1]=='gl_FragData' or id(e) in self.p.fragdata else 'gl_FragColor')
    # ---- top level ----
    def run(self, ast):
        order=[]
        for d in ast:
            k=d[0]
            if k=='precision': self.prec_stmt(d)
            elif k=='struct': self.struct(d); self.gname(d[1], 'struct')
            elif k=='gdecl': self.gdecl(d); self.gname(d[3], 'variable')
            elif k in ('proto','func'): order.append(self.func(d)); self.gname(d[2], 'function')
        if len(self.fragout)>1: self.err("a shader writes gl_FragColor or gl_FragData, not both")
        main=self.funcs.get('main')
        if main is None or not any(f.defined for f in main):
            if self.strict: raise CompileError("no main()")
        else:
            for f in main:
                if f.ret!='void' or f.params: self.err("main must be void main()")
        for name,fs in self.funcs.items():                         # an overload goes by its own name in the code generator
            for f in fs:
                f.name=name if len(fs)==1 else name+'('+','.join(f"{t[0]}{'' if t[1] is None else '[%d]'%t[1]}" for t in f.sig)+')'
                if f.node is not None: self.fnames[id(f.node)]=f.name
        if self.strict: self.no_recursion()
    def no_recursion(self):
        state={}
        def visit(f):
            if state.get(id(f))==1: raise CompileError(f"recursive call to {f.name!r} (forbidden in GLSL ES)")
            if state.get(id(f)): return
            state[id(f)]=1
            for g in f.calls: visit(g)
            state[id(f)]=2
        for fs in self.funcs.values():
            for f in fs: visit(f)
    def struct(self, d):
        if d[1] in self.structs: self.err(f"struct {d[1]} is already defined")
        for m in d[2]:
            if not self.base_ok(m[0]) or m[0]=='void': self.err(f"struct {d[1]}: bad member type {m[0]}")
            self.need_prec(m[0], self.p.prec.get(id(m)))
            if m[2] is not None: self.size(m[2])
        self.structs[d[1]]=d[2]
    def prec_stmt(self, d):
        if d[1] not in ('highp','mediump','lowp'): self.err("bad precision statement")
        if d[2] not in ('float','int')+SAMPLERS: self.err(f"no default precision for {d[2]}")
        self.defprec[-1][d[2]]=d[1]
    def gdecl(self, d):
        _,qual,ty,name,arr,init=d
        if not self.base_ok(ty) or ty=='void': self.err(f"bad type {ty}")
        self.need_prec(ty, self.p.prec.get(id(d)))
        n=self.size(arr)
        f,_w=_fam(ty)
        if qual=='attribute':
            if self.stage!='vs': self.err("attribute is for vertex shaders")
            if f!='float' or arr is not None: self.err("an attribute is a float, a vector or a matrix of floats")
        elif qual=='varying':
            if f!='float': self.err("a varying is a float, a vector or a matrix of floats, or an array of them")
        elif qual=='var' or qual=='const':
            if ty in SAMPLERS: self.err("a sampler must be a uniform")
        val=None
        if qual=='const':
            if init is None: self.err(f"const {name!r} needs an initializer")
        elif qual in ('attribute','varying','uniform') and init is not None: self.err(f"{qual} {name!r} cannot have an initializer")
        if init is not None:
            self.init_check(ty,n,init,qual=='const')
            if arr is not None: self.err("an array cannot have an initializer")
            if qual=='const' and ty in SCALAR:
                val=fold_expr(init, self.get)
                if val is not None and ty=='int': val=IntLit(int(val))
            if qual=='var' and not self.is_const(init): self.err("a global's initializer must be a constant expression")
        self.declare(name,(ty,n),'' if qual=='var' else qual,val)
    def init_check(self, ty, n, init, must_const):
        t=self.ty(init)
        if not self.same(t,(ty,n)): self.err(f"cannot initialize a {ty} with a {t[0]}")
        if must_const and not self.is_const(init): self.err("the initializer of a const must be a constant expression")
    def same(self, a, b): return a==b or a[0]=='?' or b[0]=='?'
    def func(self, d):
        isdef=d[0]=='func'
        ret,name,params=d[1],d[2],d[3]
        if ret!='void' and not self.base_ok(ret): self.err(f"bad return type {ret}")
        if ret!='void': self.need_prec(ret, self.p.prec.get(id(d)))
        sig=[]
        for p in params:
            a,c=self.p.parr.get(id(p),(None,False))
            if p[1]=='void' or not self.base_ok(p[1]): self.err(f"bad parameter type {p[1]}")
            if c and p[0] in ('out','inout'): self.err("const out is not a parameter")
            if p[1] in SAMPLERS and p[0]!='in': self.err("a sampler is an in parameter")
            self.need_prec(p[1], self.p.prec.get(id(p)))
            sig.append((p[1],self.size(a)))
        sig=tuple(sig)
        if name.startswith('gl_'): self.err(f"{name!r}: the gl_ prefix is reserved")
        if ret!='void' and self.has_array(ret): self.err(f"{name}: a function cannot return a struct with an array")
        if self.strict and self.builtin_takes(name, sig): self.err(f"{name}: a built-in function cannot be declared again")
        fs=self.funcs.setdefault(name,[])
        fn=next((f for f in fs if f.sig==sig),None)
        if fn is None: fn=Fn(name,ret,params,sig); fs.append(fn)
        else:
            if fn.ret!=ret: self.err(f"{name}: two return types")
            if not isdef: self.err(f"{name} is already declared")
            for p,q in zip(params, fn.params):
                if p[0]!=q[0] or self.p.parr.get(id(p),(None,False))[1]!=self.p.parr.get(id(q),(None,False))[1]:
                    self.err(f"{name}: a parameter's qualifiers differ from its declaration")
        if isdef:
            if fn.defined: self.err(f"{name} is already defined")
            fn.defined=True; fn.node=d; fn.params=params
            self.scopes.append({}); self.defprec.append({}); self.cur=fn
            for p,s in zip(params,sig):
                if p[2]: self.declare(p[2],s,'ro' if self.p.parr.get(id(p),(None,False))[1] else p[0])
            self.stmt(d[4])
            self.cur=None; self.defprec.pop(); self.scopes.pop()
        return fn
    # ---- statements ----
    def stmt(self, s):
        if s is None: return
        k=s[0]
        if k=='block':
            self.scopes.append({}); self.defprec.append({})
            for x in s[1]: self.stmt(x)
            self.defprec.pop(); self.scopes.pop()
        elif k=='multi':
            for x in s[1]: self.stmt(x)
        elif k=='ldecl':
            _,isc,ty,name,init,arr=s
            if not self.base_ok(ty) or ty=='void': self.err(f"bad type {ty}")
            if self.has_sampler(ty): self.err("a sampler must be a uniform")
            self.need_prec(ty, self.p.prec.get(id(s)))
            n=self.size(arr); val=None
            if isc and init is None: self.err(f"const {name!r} needs an initializer")
            if init is not None:
                if arr is not None: self.err("an array cannot have an initializer")
                self.init_check(ty,n,init,isc)
                if isc and ty in SCALAR:
                    val=fold_expr(init, self.get)
                    if val is not None and ty=='int': val=IntLit(int(val))
            self.declare(name,(ty,n),'const' if isc else '',val)
        elif k=='precision': self.prec_stmt(s)
        elif k=='lstruct': self.struct(s)
        elif k=='expr': self.ty(s[1])
        elif k=='if':
            self.cond(s[1]); self.stmt(s[2]); self.stmt(s[3])
        elif k=='for':
            self.scopes.append({}); self.stmt(s[1]); self.cond(s[2])
            if s[3] is not None: self.ty(s[3])
            self.loops+=1
            if self.strict and s[4][0]=='block':                   # §6.3: the body shares the scope of the for statement
                self.defprec.append({})
                for x in s[4][1]: self.stmt(x)
                self.defprec.pop()
            else: self.stmt(s[4])
            self.loops-=1; self.scopes.pop()
        elif k=='while':
            self.cond(s[1]); self.loops+=1; self.stmt(s[2]); self.loops-=1
        elif k=='dowhile':
            self.loops+=1; self.stmt(s[2]); self.loops-=1; self.cond(s[1])
        elif k=='return':
            rt=self.cur.ret if self.cur is not None else 'void'
            if s[1] is None:
                if rt!='void': self.err("return with no value in a function that has a type")
            else:
                t=self.ty(s[1])
                if rt=='void' or not self.same(t,(rt,None)): self.err(f"return of a {t[0]} from a function of type {rt}")
        elif k=='discard':
            if self.stage!='fs': self.err("discard is for fragment shaders")
        elif k in ('break','continue'):
            if not self.loops: self.err(f"{k} outside a loop")
    def cond(self, e):
        t=self.ty(e)
        if not self.same(t,('bool',None)): self.err(f"a condition must be a bool, not a {t[0]}")
    # ---- expressions ----
    def is_lvalue(self, e):
        k=e[0]
        if k=='var':
            s=self.sym(e[1])
            if s is None: return True
            if s.qual in ('const','attribute','uniform','ro'): return False
            if s.qual=='varying' and self.stage=='fs': return False
            return True
        if k=='swizzle':
            if len(set(e[2]))!=len(e[2]) and self.ty(e[1])[0] not in self.structs: return False       # v.xx = ...
            return self.is_lvalue(e[1])
        if k=='index': return self.is_lvalue(e[1])
        return False
    def is_const(self, e):
        k=e[0]
        if k=='num': return True
        if k=='var':
            s=self.sym(e[1]); return s is None or s.qual=='const'
        if k in ('swizzle','unary'): return self.is_const(e[1] if k=='swizzle' else e[2])
        if k=='index': return self.is_const(e[1]) and self.is_const(e[2])
        if k=='bin': return self.is_const(e[2]) and self.is_const(e[3])
        if k=='ternary': return all(self.is_const(x) for x in e[1:])
        if k=='call':
            n=e[1]
            if n in self.funcs or n in _TEXFN or n in _DERIV or 'EXT' in n: return False
            return all(self.is_const(a) for a in e[2])
        return False
    def ty(self, e):
        t=self.ty1(e)
        if t==('int',None): self.ints.add(id(e))            # the code generator's registers do not always say so: 1 ?: 2
        return t
    def ty1(self, e):
        k=e[0]
        if k=='num':
            if isinstance(e[1],OneLit): return ERR
            return ('int',None) if isinstance(e[1],IntLit) else ('bool',None) if isinstance(e[1],BoolLit) else ('float',None)
        if k=='var':
            s=self.sym(e[1])
            if s is None: return self.err(f"undefined identifier {e[1]!r}")
            if e[1]=='gl_DepthRange': self.depthrange=True
            return s.ty
        if k=='swizzle': return self.ty_field(e)
        if k=='index': return self.ty_index(e)
        if k=='call': return self.ty_call(e)
        if k=='postop':
            t=self.ty(e[1]); self.wrote(e[1])
            if t[0]=='?': return t
            if not self.is_lvalue(e[1]): self.err(f"{e[2]} needs a variable it can write")
            if _fam(t[0])[0] not in ('int','float') or t[1] is not None: self.err(f"{e[2]} of a {t[0]}")
            return t
        if k=='unary':
            t=self.ty(e[2])
            if t[0]=='?': return t
            f,_=_fam(t[0])
            if e[1]=='!':
                if t!=('bool',None): self.err("! needs a bool")
            elif f not in ('int','float') or t[1] is not None: self.err(f"{e[1]} of a {t[0]}")
            return t
        if k=='bin': return self.ty_bin(e[1], self.ty(e[2]), self.ty(e[3]))
        if k=='ternary':
            self.cond(e[1]); a=self.ty(e[2]); b=self.ty(e[3])
            if not self.same(a,b): self.err("?: needs two sides of one type")
            if a[1] is not None: self.err("?: of arrays")
            return a if a[0]!='?' else b
        if k=='assign':
            t=self.ty(e[2]); r=self.ty(e[3]); self.wrote(e[2])
            if t[0]!='?' and not self.is_lvalue(e[2]): self.err("assignment to something that cannot be written")
            if e[1]=='=':
                if not self.same(t,r): self.err(f"cannot assign a {r[0]} to a {t[0]}")
                if t[1] is not None or self.has_sampler(t[0]): self.err("cannot assign this type")     # 5.8: entire structures are l-values, whole arrays are not
                return t
            q=self.ty_bin(e[1][0], t, r)
            if not self.same(q,t): self.err(f"{e[1]} changes the type of a {t[0]}")
            return t
        if k=='seq':
            t=ERR
            for x in e[1]: t=self.ty(x)
            return t
        return ERR
    def ty_field(self, e):
        t=self.ty(e[1]); f=e[2]
        if t[0]=='?': return t
        if t[1] is not None: return self.err("a field of an array")
        if t[0] in self.structs:
            for mt,mn,ma in self.structs[t[0]]:
                if mn==f: return (mt, self.size(ma) if ma is not None else None)
            return self.err(f"struct has no member {f!r}")
        fam,n=_fam(t[0])
        if t[0] not in VEC:
            return self.err(f"a field of a {t[0]}")
        for names in ('xyzw','rgba','stpq'):
            if all(c in names for c in f):
                if len(f)>4 or any(names.index(c)>=n for c in f): return self.err(f"swizzle .{f} out of range")
                return (_vecof(fam,len(f)),None)
        return self.err(f"bad swizzle .{f}")
    def ty_index(self, e):
        t=self.ty(e[1]); i=self.ty(e[2])
        if not self.same(i,('int',None)): self.err("an index must be an int")
        v=fold_expr(e[2], self.get)
        if t[0]=='?': return t
        if t[1] is not None:
            if v is not None and not 0<=v<t[1]: self.err(f"index {int(v)} out of range")
            return (t[0],None)
        if t[0] in MAT or t[0] in VEC:
            fam,n=_fam(t[0])
            if v is not None and not 0<=v<n: self.err(f"index {int(v)} out of range")
            return (f'vec{n}',None) if t[0] in MAT else (fam,None)
        return self.err(f"cannot index a {t[0]}")
    def ty_bin(self, op, a, b):
        if a[0]=='?' or b[0]=='?':
            return ('bool',None) if op in ('<','>','<=','>=','==','!=','&&','||','^^') else (a if a[0]!='?' else b)
        if op in ('&&','||','^^'):
            if a!=('bool',None) or b!=('bool',None): self.err(f"{op} needs bools")
            return ('bool',None)
        if op in ('==','!='):
            if a!=b: self.err(f"{op} of a {a[0]} and a {b[0]}")
            if a[1] is not None or self.has_opaque(a[0]): self.err(f"{op} of this type")
            return ('bool',None)
        if op in ('<','>','<=','>='):
            if a!=b or a[1] is not None or a[0] not in ('int','float'): self.err(f"{op} of a {a[0]} and a {b[0]}")
            return ('bool',None)
        if op not in ('+','-','*','/'): return self.err(f"the operator {op} is reserved")
        fa,na=_fam(a[0]); fb,nb=_fam(b[0])
        if a[1] is not None or b[1] is not None or fa is None or fb is None or fa=='bool' or fb=='bool' or fa!=fb:
            return self.err(f"{op} of a {a[0]} and a {b[0]}")
        if a[0]==b[0]: return a
        if a[0] in SCALAR: return b
        if b[0] in SCALAR: return a
        if op=='*':
            if a[0] in MAT and b[0] in VEC and na==nb: return b
            if a[0] in VEC and b[0] in MAT and na==nb: return a
        return self.err(f"{op} of a {a[0]} and a {b[0]}")
    def ty_call(self, e):
        n=e[1]; args=e[2]
        ts=[self.ty(a) for a in args]
        if n in ('float','int','bool') or n in VEC or n in MAT: return self.ty_ctor(n,args,ts)
        if n in self.structs:
            ms=self.structs[n]
            if len(ms)!=len(ts): return self.err(f"{n}: {len(ms)} members, {len(ts)} arguments")
            for m,t in zip(ms,ts):
                if not self.same(t,(m[0],self.size(m[2]) if m[2] is not None else None)): self.err(f"{n}: member {m[1]} is not a {t[0]}")
            return (n,None)
        if n in self.funcs:
            sig=tuple(ts)
            for f in self.funcs[n]:
                if f.sig==sig or (not self.strict and len(f.sig)==len(sig) and all(self.same(x,y) for x,y in zip(f.sig,sig))):
                    self.callmap[id(e)]=f
                    if self.cur is not None: self.cur.calls.append(f)
                    for p,a in zip(f.params,args):
                        if p[0] in ('out','inout'):
                            self.wrote(a)
                            if not self.is_lvalue(a): self.err(f"{n}: an {p[0]} argument must be a variable")
                    return (f.ret,None)
            return self.err(f"no function {n}({', '.join(t[0] for t in ts)})")
        return self.ty_builtin(e,n,ts)
    def ty_ctor(self, n, args, ts):
        if not args: return self.err(f"{n}() needs arguments")
        total=0; want=1 if n in SCALAR else VEC.get(n) or MAT[n]**2; mats=0
        for i,t in enumerate(ts):
            if t[0]=='?': return (n,None)
            f,c=_fam(t[0])
            if f is None or t[1] is not None: return self.err(f"{n}() of a {t[0]}")
            if t[0] in MAT: c=c*c; mats+=1
            if total>=want: return self.err(f"{n}(): too many arguments")
            total+=c
        if mats and len(ts)>1 and n in MAT: return self.err(f"{n}(): a matrix goes alone")
        if total<want and not (len(ts)==1 and (ts[0][0] in SCALAR or (n in MAT and ts[0][0] in MAT))):
            return self.err(f"{n}(): too few arguments")
        return (n,None)
    def ty_builtin(self, e, n, ts):
        fs=self.stage=='fs'
        # What the fragment core does is there whether or not ES 1.00 asks for it: the derivatives and the *LodEXT / *GradEXT reads
        # without their #extension lines, the *Lod reads in a fragment shader. The vertex core has no quads and no texture unit
        if self.strict:
            if n in _DERIV and not fs: return self.err(f"{n} is for fragment shaders")
            if n.endswith('EXT') and not fs: return self.err(f"{n} is for fragment shaders")
            if n in ('texture2D','texture2DProj','textureCube','texture3D','texture3DProj') and len(ts)==3 and not fs: return self.err(f"{n} with a bias is for fragment shaders")
        known=False
        for sg in SIGS:
            if sg[1]!=n: continue
            known=True
            if len(sg)-2!=len(ts): continue
            w=None; ok=True
            for p,t in zip(sg[2:],ts):
                if t[0]=='?': continue
                if t[1] is not None: ok=False; break
                f,c=_fam(t[0])
                fam={'G':'float','V':'float','I':'int','L':'bool'}.get(p)
                if p=='M':
                    if t[0] not in MAT: ok=False; break
                elif fam is not None:
                    if f!=fam or t[0] in MAT or (p!='G' and c==1): ok=False; break
                else:
                    if t[0]!=p: ok=False; break
                    continue
                if w is None: w=c
                elif w!=c: ok=False; break
            if not ok: continue
            r=sg[0]
            if r=='G': return (_vecof('float',w or 1),None)
            if r=='R': return (_vecof('bool',w or 2),None)
            if r=='M': return (f'mat{w or 2}',None)
            return (r,None)
        return self.err(f"no function {n}({', '.join(t[0] for t in ts)})" if known else f"unknown function/builtin {n!r}")


# ---------------- codegen ----------------
BASIS = {0:'ex',1:'ey',2:'ez',3:'ew'}
class Reg:
    __slots__=('n','ty','const','tmp','cvec')
    def __init__(self,n,ty,tmp=True): self.n=n; self.ty=ty; self.tmp=tmp; self.const=None; self.cvec=None   # const: a known scalar value (kf); cvec: a float vector's known components (construct)
class MatVal:
    def __init__(self, cols, ty): self.cols=cols; self.ty=ty   # cols: list[Reg vecN]
class Sampler:
    def __init__(self, unit, ty, size=None): self.unit=unit; self.ty=ty; self.size=size   # sampler2D/samplerCube/sampler3D -> texture unit 0..7; size: an array of samplers
class StructVal:
    def __init__(self, sname, members): self.sname=sname; self.members=members; self.ty=sname   # members: name->value
class ArrayVal:
    # ety[size] array. kind 'uniform': elements at uniform slots base + i*per (loaded on demand); dyn = an int-repr
    #                   register added to the slot at run time (an array member of a run-time-indexed struct, VS only).
    #                   kind 'local' : elements are the Regs/MatVals in `elems` (mutable via arr[i]=...).
    def __init__(self, ety, size, kind, base=None, elems=None, dyn=None):
        self.ety=ety; self.size=size; self.kind=kind; self.base=base; self.elems=elems; self.dyn=dyn
        self.ty=f'{ety}[{size}]'

class Scope(dict):
    """name -> (value, type); .const: name -> the value of a scalar tracked at compile time. An inner declaration hides an outer one of either kind."""
    def __init__(self): super().__init__(); self.const={}

_SPECIAL = {'vs':{'gl_Position':'vec4','gl_PointSize':'float'}, 'fs':{'gl_FragColor':'vec4','gl_FragDepth':'float','gl_FragDepthEXT':'float'}}   # the outputs a program may also read or write in part

class Codegen:
    def __init__(self, stage):
        self.stage=stage         # 'vs' or 'fs'
        self.code=[]             # .sasm lines (virtual registers; physical assigned by allocate_registers)
        self.vcount=0            # virtual register counter
        self.consts={}           # tuple(4 floats) -> uniform slot
        self.const_order=[]      # slots in order
        self.creg={}             # tuple(4 floats) -> pinned Reg (loaded once, reused)
        self.cvreg={}            # (type, 4 floats) -> pinned Reg of a constant vector (kvec)
        self.remat_consts=False  # True -> rematerialize constants (fresh LDP per use) to cut register pressure
        self.fold_cvec=True      # False -> a vector of known components built a lane at a time from the scalar constants (no kvec)
        self.uniforms={}         # name -> (slot, type)  (user uniforms)
        self.samplers={}         # name -> texture unit (sampler2D/samplerCube/sampler3D)
        self.sampler_sizes={}    # name -> size, for an array of samplers (units base .. base + size - 1)
        self.next_sampler=0
        self.sampler_slot={}     # the uniform slot of a sampler in a uniform struct -> its unit (_struct_samplers)
        self.attribs={}          # name -> (index, type)
        self.varyings={}         # name -> (index, type)
        self.locals=[]           # scope stack of {name: (Reg|MatVal, type)}
        self.const_env={}        # global scalar consts; a scalar local of known value is in its Scope's .const -> constant propagation
        self.cond_decls=set()    # ids of the declarations written under runtime control flow -> must be real (masked) regs, not const
        self.hw_for={}           # id of a constant-bounded for statement that is a hardware loop (_long_for) -> its pass count
        self.loopctl=[]          # stack of {brk,cont} flag regs for loops containing break/continue
        self.next_uniform=0
        self.next_attr=0
        self.next_vary=0
        self.zero=None
        self.nozero=False        # a declaration without a value is not zeroed (_zero): the outputs' shadows
        self.label_n=0
        self.isa64=False         # True: the 64-bit swizzling ISA (per-operand swizzles, SEL, IDIV, FLR / CEIL, TEX modes)
        self.xops=0              # board B's fragment-core operations (STATUS b_flags bits 28 / 29 >> 28): 1 SQRT / RCPM / RSQM, 2 CLAMP; fragment stage only
        self.hw_idiv=True        # False: the core has no integer divider (noops bit 0): int / is _idiv_lowered
        self.svregs=set()        # virtual registers holding a SCALAR (lane-uniform) value: packed one per component on isa64
        self.vs_tex=False        # a vertex program samples a texture
        self.umap=None           # Appendix A.7, a program over the limits: uniform slot -> row * 4 + lane
        self.vmap=None           # the same of a varying slot, -1 = the link passes it on to nothing
        self.depth_units=0       # bit per sampler: a depth texture is bound, a read gives (z, z, z, 1)
    # ---- emit ----
    def emit(self,*a):
        if self.umap is not None and a[0] in ('LDP','LDPX') and isinstance(a[2],int): return self._load_at(self.umap[a[2]], *a)
        self.code.append(' '.join(str(x) for x in a))
    def _load_at(self, at, op, d, slot, *rest):
        """a slot that shares its row: the row, then its own lanes down to .x"""
        row,comp=at>>2,at&3
        sc=int(d[1:]) in self.svregs                     # a scalar's load is one lane: CSE must not take it for the row
        if not comp and not sc: self.code.append(' '.join(str(x) for x in (op,d,row)+rest)); return
        r=self.alloc('vec4')
        self.code.append(' '.join(str(x) for x in (op,f'r{r.n}',row)+rest))
        self.code.append(f"MOV {d} r{r.n}.{'xyzw'[comp] if sc else 'xyzw'[comp:]+'w'*comp}")
    def _vload(self, d, slot):
        if self.vmap is None: self.emit('LDV', d, slot)
        else: self._load_at(max(self.vmap[slot],0) if slot<len(self.vmap) else 0, 'LDV', d, slot)
    def newlabel(self,p='L'): self.label_n+=1; return f'{p}{self.label_n}'
    # ---- registers ----
    def alloc(self, ty='vec4'):
        self.vcount+=1
        if ty in SCALAR: self.svregs.add(self.vcount)
        return Reg(self.vcount, ty)          # virtual reg; physical assigned post-codegen by liveness
    def freer(self, r): pass                 # no-op: liveness allocation frees registers, not the codegen
    # ---- constant pool (basis vectors + literals -> uniforms) ----
    def const_slot(self, x,y,z,w):
        key=(float(x),float(y),float(z),float(w))
        if key not in self.consts:
            self.consts[key]=len(self.const_order); self.const_order.append(key)
        return self.consts[key]     # index into the constant pool (final uniform slot = const_base + index)
    def kload(self, x,y,z,w):
        # Default: load each unique constant ONCE, pin the reg + reuse it (fewest instructions). This keeps a
        # register live for the constant's whole span, which for constant-heavy shaders can exceed the 31-reg
        # file. remat_consts=True instead emits a FRESH short-lived LDP at every use (rematerialization): an
        # LDP writes the active lanes and the use reads them under the same mask, so a per-use reload is always
        # SIMT-correct with NO hoisting needed — it trades more LDP instructions for far lower register pressure.
        if not self.remat_consts:
            key=(float(x),float(y),float(z),float(w))
            if key in self.creg: return self.creg[key]
        self.const_slot(x,y,z,w)
        r=self.alloc()
        if not self.remat_consts: r.tmp=False          # pinned (never freed)
        self.emit('LDP', f'r{r.n}', self.ckey_index(x,y,z,w))
        if not self.remat_consts:
            key=(float(x),float(y),float(z),float(w)); self.creg[key]=r
        return r
    def ckey_index(self, x,y,z,w):
        return f'@{self.const_slot(x,y,z,w)}'    # resolved to const_base+index after user uniforms are known
    # basis / common constants
    def basis(self, c):     # e_c broadcast constant
        v=[1.0 if i==c else 0.0 for i in range(4)]; return self.kload(*v)
    def kf(self, f):        # scalar constant broadcast (type float: broadcast to all lanes; packed on isa64)
        r=self.kload(f,f,f,f); r.ty='float'; r.const=float(f); self.svregs.add(r.n); return r
    def kvec(self, ty, vals):
        """a float vector of known components: one constant row, the lanes past ty's the last component (a splat is a splat), loaded
        once (remat_consts: at every use) into a register of its own, not kf's (a scalar's register is one lane of a packed one)"""
        v=tuple(float(x) for x in vals)+(float(vals[-1]),)*(4-len(vals))
        key=(ty,)+v
        if not self.remat_consts and key in self.cvreg: return self.cvreg[key]
        r=self.alloc(ty)
        self.emit('LDP', f'r{r.n}', self.ckey_index(*v))
        r.cvec=tuple(float(x) for x in vals)
        if not self.remat_consts: r.tmp=False; self.cvreg[key]=r
        return r
    def _known(self, a):
        """a value's components known at compile time (a matrix column by column), as _cvec takes them, else None"""
        if isinstance(a,MatVal):
            out=[]
            for c in a.cols:
                k=self._known(c)
                if k is None: return None
                out+=k
            return out
        if not isinstance(a,Reg): return None
        if is_scalar(a.ty): return None if a.const is None else [a.const]
        return None if a.cvec is None else list(a.cvec[:ncomp(a.ty)])
    def zeroreg(self):
        if self.zero is None:
            self.zero=self.kf(0.0); self.zero.tmp=False
        return self.zero

    # ---- swizzle read: v.<swz> -> new reg (result type by len) ----
    def swizzle_read(self, v, swz):
        if isinstance(v, MatVal): raise CompileError("cannot swizzle a matrix")
        v=self._asf(v)                                    # int scalar .x etc. -> float
        comps=[ 'xyzw'.index(c) if c in 'xyzw' else 'rgba'.index(c) if c in 'rgba' else 'stpq'.index(c)
                for c in swz ]
        n=len(comps)
        ty='float' if n==1 else f'vec{n}'
        # single-component swizzle (v.x/.y/.z/.w) is a SCALAR -> broadcast the component to all lanes
        # (extract does a DP4 with a basis vector). A plain MOV would leave lanes 1..3 holding v.y/z/w,
        # which then corrupt any componentwise use (s*vec, vec4(s), s*s into different output lanes).
        if n==1:
            return self.extract(v, comps[0])
        # identity (v.xyzw for vec4, v.xyz for vec3 ...) -> just reuse (but must copy if needed elsewhere).
        # Safe only for multi-component swizzles: lanes stay in their natural positions.
        if comps==list(range(n)) and ncomp(v.ty)>=n:
            r=self.alloc(ty); self.emit('MOV', f'r{r.n}', f'r{v.n}'); return r
        r=self.alloc(ty)
        if self.isa64:                            # one MOV with the source swizzle (lanes >= n repeat the last)
            self.emit('MOV', f'r{r.n}', f'r{v.n}.{"".join("xyzw"[c] for c in comps)}'); return r
        for d,src in enumerate(comps):
            ext=self.extract(v, src)              # v[src] broadcast
            self.emit(f'MOV.{"xyzw"[d]}', f'r{r.n}', f'r{ext.n}')
            self.freer(ext)
        return r
    def extract(self, v, comp):                   # v[comp] -> broadcast float reg
        v=self._asf(v)
        r=self.alloc('float')
        if self.isa64:                            # a splat swizzle: MOV rd, v.c (1 instruction, no constant)
            self.emit('MOV', f'r{r.n}', f'r{v.n}.{"xyzw"[comp]}'); return r
        b=self.basis(comp)
        self.emit('DP4', f'r{r.n}', f'r{v.n}', f'r{b.n}'); self.freer(b); return r
    def swz_src(self, v, comp):                   # operand text for v[comp] broadcast (isa64: v.c, else an extracted reg)
        if self.isa64: return f'r{self._asf(v).n}.{"xyzw"[comp]}'
        return f'r{self.extract(v, comp).n}'

    # ---- integer representation (int-declared scalars: 2's-comp bit pattern, IADD/ISUB/ISLT/ISGE) ----
    # int-repr keeps loop counters off the FP-ADD latency-8 recurrence (IADD is latency 1). Values cross
    # into the float domain via ITOF at every float-consuming site; a float becomes int via FTOI (truncate).
    def _asf(self, x):                            # ensure float representation (ITOF if int-repr)
        if isinstance(x,Reg) and x.ty=='int':
            r=self.alloc('float'); self.emit('ITOF', f'r{r.n}', f'r{x.n}'); return r
        return x
    def _asi(self, x):                            # ensure int representation (FTOI truncate-toward-zero if float)
        if isinstance(x,Reg) and x.ty!='int':
            r=self.alloc('int'); self.emit('FTOI', f'r{r.n}', f'r{x.n}'); return r
        return x
    def _asi_lit(self, val):                      # integer literal -> int-repr reg
        f=self.kf(float(val)); r=self.alloc('int'); self.emit('FTOI', f'r{r.n}', f'r{f.n}'); return r
    def _local_type(self, name):                  # declared type of a real (reg-backed) local, else None
        for sc in reversed(self.locals):
            if name in sc:
                v=sc[name]; return v[1] if isinstance(v,tuple) else None
        return None
    def int_binop(self, op, a, b):
        if op in ('+','-'):
            r=self.alloc('int'); self.emit('IADD' if op=='+' else 'ISUB', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        else:                                     # relational -> ISLT/ISGE/ISEQ/ISNE (exact signed compare -> 1.0/0.0)
            r=self.alloc('bool'); M={'<':('ISLT',a,b),'>':('ISLT',b,a),'<=':('ISGE',b,a),'>=':('ISGE',a,b),'==':('ISEQ',a,b),'!=':('ISNE',a,b)}
            mn,x,y=M[op]; self.emit(mn, f'r{r.n}', f'r{x.n}', f'r{y.n}')
        self.freer(a); self.freer(b); return r

    # ---- masked write: lhs.<mask> = rhs  (lhs a Reg) ----
    def masked_write(self, lhs, mask, rhs):
        comps=['xyzw'.index(c) if c in 'xyzw' else 'rgba'.index(c) if c in 'rgba' else 'stpq'.index(c) for c in mask] if mask else list(range(ncomp(rhs.ty)))
        if not mask:                              # whole-object assign
            self.emit('MOV', f'r{lhs.n}', f'r{rhs.n}'); return
        # rhs component i -> lhs lane comps[i]. If in natural order & contiguous-from-x, direct.
        if comps==list(range(len(comps))):
            m=''.join('xyzw'[c] for c in comps)
            self.emit(f'MOV.{m}', f'r{lhs.n}', f'r{rhs.n}')
        elif self.isa64:                          # one masked MOV; the swizzle is indexed by the destination lane
            if rhs.n==lhs.n:                      # v.yx = v.xy: copy first (ISA rule: no read of a lane this
                t=self.alloc(rhs.ty); self.emit('MOV', f'r{t.n}', f'r{rhs.n}'); rhs=t   # instruction wrote earlier)
            m=''.join('xyzw'[c] for c in sorted(comps))
            swz=''.join('xyzw'[comps.index(l)] if l in comps else 'x' for l in range(4))
            self.emit(f'MOV.{m}', f'r{lhs.n}', f'r{rhs.n}.{swz}')
        else:
            for i,lane in enumerate(comps):
                src=self.extract(rhs, i)          # rhs[i] broadcast
                self.emit(f'MOV.{"xyzw"[lane]}', f'r{lhs.n}', f'r{src.n}')
                self.freer(src)

    # ---- build a matN (list of column regs) ----
    def construct_mat(self, ty, args):
        args=[self._asf(a) for a in args]                 # int components -> float
        n=ncomp2(ty)
        if len(args)==1 and isinstance(args[0],MatVal):   # from a matrix: its components where they exist, the identity elsewhere
            m=args[0]; k=ncomp2(m.ty); cols=[]
            kn=self._known(m) if self.fold_cvec else None
            if kn is not None:                            # known: a constant row a column
                return MatVal([self.kvec(f'vec{n}', [kn[j*k+i] if j<k and i<k else (1.0 if i==j else 0.0) for i in range(n)]) for j in range(n)], ty)
            for j in range(n):
                col=self.alloc(f'vec{n}')
                for i in range(n):
                    src=self.swz_src(m.cols[j],i) if j<k and i<k else f'r{self.kf(1.0 if i==j else 0.0).n}'
                    self.emit(f'MOV.{"xyzw"[i]}', f'r{col.n}', src)
                cols.append(col)
            return MatVal(cols, ty)
        if len(args)==1 and is_scalar(args[0].ty):        # matN(f) = f on the diagonal, 0 elsewhere
            f=args[0]
            if f.const is not None and self.fold_cvec: return MatVal([self.kvec(f'vec{n}', [f.const if i==j else 0.0 for i in range(n)]) for j in range(n)], ty)
            z=self.zeroreg(); cols=[]
            for j in range(n):
                col=self.alloc(f'vec{n}'); self.emit('MOV', f'r{col.n}', f'r{z.n}')
                self.emit(f'MOV.{"xyzw"[j]}', f'r{col.n}', f'r{f.n}'); cols.append(col)
            return MatVal(cols, ty)
        if len(args)==n and all(isinstance(a,Reg) and is_vec(a.ty) and ncomp(a.ty)==n for a in args):   # n column vectors
            return MatVal(list(args), ty)
        kn=[] if self.fold_cvec else None
        for a in args if self.fold_cvec else ():
            k=self._known(a)
            if k is None: kn=None; break
            kn+=k
        if kn is not None and len(kn)>=n*n:               # every component known: a constant row a column
            return MatVal([self.kvec(f'vec{n}', kn[j*n:(j+1)*n]) for j in range(n)], ty)
        srcs=[]                                           # anything else: the components in order, column-major
        for a in args:
            if isinstance(a,MatVal):
                for c in a.cols: srcs+=[self.swz_src(c,i) for i in range(ncomp(c.ty))]
            elif not isinstance(a,Reg): raise CompileError(f"unsupported {ty} constructor ({len(args)} args)")
            elif is_scalar(a.ty): srcs.append(f'r{a.n}')
            else: srcs+=[self.swz_src(a,i) for i in range(ncomp(a.ty))]
        if len(srcs)<n*n: raise CompileError(f"unsupported {ty} constructor ({len(args)} args)")
        cols=[]
        for j in range(n):
            col=self.alloc(f'vec{n}')
            for i in range(n): self.emit(f'MOV.{"xyzw"[i]}', f'r{col.n}', srcs[j*n+i])
            cols.append(col)
        return MatVal(cols, ty)

    # ---- build a vecN from a list of scalar/vector arg Regs ----
    def _convert(self, r, ty, args):
        """ivecN / bvecN from values of another basic type: an int vector's lanes are integral floats, a bool's are 1.0 / 0.0"""
        f=ty[0]
        if f=='i' and any(_fam(a.ty)[0]=='float' for a in args):
            if self.isa64: t=self.alloc(ty); self.emit('TRUNC', f'r{t.n}', f'r{r.n}'); return t
            t=self.alloc(ty); self.emit('FTOI', f'r{t.n}', f'r{r.n}'); self.emit('ITOF', f'r{t.n}', f'r{t.n}'); return t
        if f=='b' and any(_fam(a.ty)[0]!='bool' for a in args):
            t=self.alloc(ty); self.emit('SET', 'SNE', f'r{t.n}', f'r{r.n}', f'r{self.zeroreg().n}'); return t
        return r
    def construct(self, ty, args):
        flat=[]
        for a in args: flat+=a.cols if isinstance(a,MatVal) else [a]     # a matrix goes in column by column
        orig=flat; args=[self._asf(a) for a in flat]                     # int components -> float
        cv=self._cvec(ty, args) if self.fold_cvec else None
        if cv is not None: return self.kvec(ty, cv)                       # every component known: one constant row
        n=ncomp(ty); r=self.alloc(ty); lane=0
        # single scalar -> broadcast to all N
        if len(args)==1 and is_scalar(args[0].ty):
            self.emit('MOV', f'r{r.n}', f'r{args[0].n}'); r.cvec=self._cvec(ty, args); return self._convert(r, ty, orig)
        for a in args:
            an=ncomp(a.ty)
            # A leading vector arg lands at its NATURAL lanes (0..an-1) -> copy with a single masked MOV
            # instead of per-component extract(). extract() is DP4-with-basis (sums all 4 lanes), and
            # x*0=NaN for a NaN/Inf in an unused upper lane (e.g. pow() of a vec3 leaves .w=NaN), which
            # would then poison every extracted component. The direct MOV reads only the wanted lanes.
            if an>1 and lane==0 and lane+an<=n:
                self.emit(f'MOV.{"xyzw"[:an]}', f'r{r.n}', f'r{a.n}'); lane+=an; continue
            for c in range(an):
                if lane>=n: break
                if an>1 and self.isa64:           # the component through the operand swizzle
                    self.emit(f'MOV.{"xyzw"[lane]}', f'r{r.n}', f'r{a.n}.{"xyzw"[c]}'); lane+=1; continue
                src=self.extract(a,c) if an>1 else a
                self.emit(f'MOV.{"xyzw"[lane]}', f'r{r.n}', f'r{src.n}')
                if an>1: self.freer(src)
                lane+=1
        r.cvec=self._cvec(ty, args)
        return self._convert(r, ty, orig)
    def _cvec(self, ty, args):
        """the components of a float vector built from known values (constants, or vectors of them), else None"""
        if ty not in ('vec2','vec3','vec4'): return None
        vals=[]
        for a in args:
            if not isinstance(a,Reg): return None
            if is_scalar(a.ty):
                if a.const is None: return None
                vals.append(a.const)
            elif a.cvec is not None: vals+=list(a.cvec[:ncomp(a.ty)])
            else: return None
        if len(args)==1 and is_scalar(args[0].ty): vals=vals*ncomp(ty)
        return tuple(vals[:ncomp(ty)]) if len(vals)>=ncomp(ty) else None


# --------------- expression + statement compiler ---------------
class Compiler(Codegen):
    def __init__(self, stage): super().__init__(stage); self.funcs={}; self._inlining=set(); self.cur_ret=None; self.structs={}; self.global_consts={}; self.shadow=[]; self.callmap={}; self.ints=set()

    def lookup(self, name):
        for sc in reversed(self.locals):
            if name in sc: return sc[name]
        if name in self.samplers:
            return ('sampler', self.samplers[name], 'sampler2D', self.sampler_sizes.get(name))
        if name in self.uniforms:
            u=self.uniforms[name]
            if len(u)==3: return ('uarray', u[0], u[1], u[2])   # (base_slot, elem_ty, size)
            return ('uniform', u[0], u[1])
        if name in self.attribs:
            idx,ty=self.attribs[name]; return ('attrib', idx, ty)
        if name in self.varyings:
            v=self.varyings[name]
            if len(v)==3: return ('varray', v[0], v[1], v[2])     # (first index, elem_ty, size)
            return ('varying', v[0], v[1])
        return None

    # load a named value into registers -> Reg or MatVal
    def load_var(self, name):
        # FS built-in inputs: gl_FragCoord = varin[8] (x,y,z,1/w), gl_FrontFacing = varin[9]. The RTL
        # (band_datapath) computes+writes these; the FS core derives out_reads_* from the LDV imm, so
        # emitting LDV 8/9 is exactly what turns the hardware compute on.
        if name=='gl_FragCoord':
            if self.stage!='fs': raise CompileError("gl_FragCoord is only available in the fragment shader")
            reg=self.alloc('vec4'); reg.tmp=False; self.emit('LDV', f'r{reg.n}', 8); return reg
        if name=='gl_FrontFacing':
            if self.stage!='fs': raise CompileError("gl_FrontFacing is only available in the fragment shader")
            reg=self.alloc('vec4'); self.emit('LDV', f'r{reg.n}', 9)
            return self.extract(reg, 0)                 # facing flag (bool) broadcast to all lanes
        if name=='gl_PointCoord':                        # board B's core: LDV 10 = (s, t, 0, 1)
            if self.stage!='fs': raise CompileError("gl_PointCoord is only available in the fragment shader")
            if not self.isa64: raise CompileError("gl_PointCoord needs the board-B core (compile with isa64)")
            reg=self.alloc('vec2'); self.emit('LDV', f'r{reg.n}', 10); return reg
        if name in self.global_consts and not any(name in sc for sc in self.locals[1:]):   # non-scalar global const -> materialize from its initializer,
            saved=self.locals; self.locals=saved[:1]                                       # which sees the globals, not the caller's names
            v=self.eval(self.global_consts[name][1]); self.locals=saved
            return v
        r=self.lookup(name)
        if r is None: raise CompileError(f"undefined identifier {name!r}")
        if r[0]=='sampler':
            return Sampler(r[1], r[2], r[3])
        if r[0]=='uarray':
            return ArrayVal(r[2], r[3], 'uniform', base=r[1])   # uniform array: load element on index
        if r[0]=='uniform':
            slot,ty=r[1],r[2]
            if ty in self.structs: return self._load_uniform_struct(slot, ty)   # uniform struct -> StructVal
            if is_mat(ty):
                cols=[self.alloc(f'vec{ncomp2(ty)}') for _ in range(ncomp2(ty))]
                for i,c in enumerate(cols): self.emit('LDP', f'r{c.n}', slot+i)
                return MatVal(cols, ty)
            reg=self.alloc(ty); self.emit('LDP', f'r{reg.n}', slot); reg.tmp=False
            if ty=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')   # host stores int uniform as a float -> int-repr
            return reg
        if r[0]=='attrib':
            if is_mat(r[2]):                            # a matrix attribute: one location per column
                cols=[self.alloc(f'vec{ncomp2(r[2])}') for _ in range(ncomp2(r[2]))]
                for i,c in enumerate(cols): self.emit('LDV', f'r{c.n}', r[1]+i)
                return MatVal(cols, r[2])
            reg=self.alloc(r[2]); self.emit('LDV', f'r{reg.n}', r[1])
            if r[2]=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')
            return reg
        if r[0]=='varray':
            return ArrayVal(r[2], r[3], 'varying', base=r[1])
        if r[0]=='varying':
            if is_mat(r[2]):
                cols=[self.alloc(f'vec{ncomp2(r[2])}') for _ in range(ncomp2(r[2]))]
                for i,c in enumerate(cols): self._vload(f'r{c.n}', r[1]+i)
                return MatVal(cols, r[2])
            reg=self.alloc(r[2]); self._vload(f'r{reg.n}', r[1])
            if r[2]=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')
            return reg
        val,ty=r                                   # local (Reg / MatVal / ArrayVal / StructVal)
        return val

    # ---- struct helpers ----
    def _arrsize(self, ma): return None if ma is None else int(self.fold_const(ma))
    def _type_nslots(self, ty, asz=None):          # uniform slots occupied by a type (x array size)
        if ty in self.structs:
            n=sum(self._type_nslots(mt, self._arrsize(ma)) for mt,mn,ma in self.structs[ty])
        else:
            n = ncomp2(ty) if is_mat(ty) else 1
        return n * (asz or 1)
    def _struct_samplers(self, path, ty, asz, slot):
        """a sampler in a uniform struct: a texture unit of its own, under the name a front end gives it (s.tex, a[1].tex),
        and the uniform slot it takes, which a load of the struct turns into that unit"""
        for e in (range(asz) if asz is not None else [None]):
            p=path if e is None else f'{path}[{e}]'
            for mt,mn,ma in self.structs[ty]:
                n=self._arrsize(ma); q=f'{p}.{mn}'
                if mt in SAMPLERS:
                    self.samplers[q]=self.next_sampler; self.sampler_slot[slot]=self.next_sampler
                    if n is not None: self.sampler_sizes[q]=n
                    self.next_sampler+=n if n is not None else 1
                elif mt in self.structs: self._struct_samplers(q, mt, n, slot)
                slot+=self._type_nslots(mt, n)
    def _load_uniform_member(self, slot, ty, asz):
        if ty in SAMPLERS: return Sampler(self.sampler_slot[slot], 'sampler2D', asz)      # a sampler in a uniform struct: its unit
        if asz is not None: return ArrayVal(ty, asz, 'uniform', base=slot)        # array member
        if ty in self.structs: return self._load_uniform_struct(slot, ty)         # nested struct
        if is_mat(ty):
            cols=[self.alloc(f'vec{ncomp2(ty)}') for _ in range(ncomp2(ty))]
            for i,c in enumerate(cols): self.emit('LDP', f'r{c.n}', slot+i)
            return MatVal(cols, ty)
        reg=self.alloc(ty); reg.tmp=False; self.emit('LDP', f'r{reg.n}', slot)
        if ty=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')       # int member stored as float -> int-repr
        return reg
    def _load_uniform_struct(self, base, sname):
        members={}; off=0
        for mt,mn,ma in self.structs[sname]:
            asz=self._arrsize(ma)
            members[mn]=self._load_uniform_member(base+off, mt, asz)
            off += self._type_nslots(mt, asz)
        return StructVal(sname, members)
    # ---- run-time uniform indexing (ES2 Appendix A.5: a vertex shader indexes uniform arrays with any integer
    # expression; the 64-bit vertex core's LDPX rd, slot, rI loads params[(slot + rI) mod 256] per lane) ----
    def _iscale(self, r, n):
        """int-repr register r x n (the element's stride in slots): IADD chains for 2..4, IMUL otherwise"""
        if n==1: return r
        if n in (2,3,4):
            t=self.alloc('int'); self.emit('IADD', f'r{t.n}', f'r{r.n}', f'r{r.n}')
            if n==2: return t
            u=self.alloc('int'); self.emit('IADD', f'r{u.n}', f'r{t.n}', f'r{t.n if n==4 else r.n}'); return u
        m=self._asi_lit(n); u=self.alloc('int'); self.emit('IMUL', f'r{u.n}', f'r{r.n}', f'r{m.n}'); return u
    def _uniform_dyn_index(self, base, idx_expr):
        """uniform array element at a run-time index: the slot offset = index x element slots (+ the enclosing
        struct's run-time offset) in one int-repr register, the constant base in LDPX's immediate"""
        ireg=self._asi(self.eval(idx_expr))                       # int-repr index (FTOI truncates a float, as int())
        off=self._iscale(ireg, self._type_nslots(base.ety))
        if base.dyn is not None:
            t=self.alloc('int'); self.emit('IADD', f'r{t.n}', f'r{base.dyn.n}', f'r{off.n}'); off=t
        off.tmp=False
        return self._load_uniform_member_dyn(base.base, base.ety, None, off)
    def _load_uniform_member_dyn(self, slot, ty, asz, dyn):
        """as _load_uniform_member, every slot offset by the run-time register dyn (LDPX instead of LDP)"""
        if asz is not None: return ArrayVal(ty, asz, 'uniform', base=slot, dyn=dyn)   # array member: indexed later
        if ty in self.structs:
            members={}; off=0
            for mt,mn,ma in self.structs[ty]:
                a=self._arrsize(ma); members[mn]=self._load_uniform_member_dyn(slot+off, mt, a, dyn); off+=self._type_nslots(mt, a)
            return StructVal(ty, members)
        if is_mat(ty):
            cols=[self.alloc(f'vec{ncomp2(ty)}') for _ in range(ncomp2(ty))]
            for j,c in enumerate(cols): self.emit('LDPX', f'r{c.n}', slot+j, f'r{dyn.n}')
            return MatVal(cols, ty)
        reg=self.alloc(ty); reg.tmp=False; self.emit('LDPX', f'r{reg.n}', slot, f'r{dyn.n}')
        if ty=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')       # int element stored as a float -> int-repr
        return reg
    def _decl_local_struct(self, sname, initval):
        # local struct: scalar/vec members get fresh mutable regs (copied from init); mat/array/nested kept
        members={}
        for mt,mn,ma in self.structs[sname]:
            iv=initval.members[mn] if initval is not None else None
            if ma is not None or is_mat(mt) or mt in self.structs:
                members[mn]=self._clone(iv) if self.isa64 else iv  # 32-bit: aggregate members kept by reference
            else:
                r=self.alloc(mt); r.tmp=False
                if iv is not None:
                    iv=self._asi(iv) if mt=='int' else self._asf(iv)
                    self.emit('MOV', f'r{r.n}', f'r{iv.n}')
                members[mn]=r
        return StructVal(sname, members)
    def _zero(self, v):
        """A variable declared without a value starts at 0. GLSL ES leaves the read of one undefined; WebGL
        zeroes them and sources written there rely on it (float z, d, i; for (...; i++ < 20.; )) — without it they would
        read whatever the register last held. One write per register; forward_opt drops it where the variable is written
        before it is read. A member that is itself an aggregate has no registers until it is written (None)."""
        if isinstance(v,MatVal):
            for c in v.cols: self._zero(c)
        elif isinstance(v,StructVal):
            for m in v.members.values(): self._zero(m)
        elif isinstance(v,ArrayVal):
            if v.kind=='local':
                for e in v.elems: self._zero(e)
        elif isinstance(v,Reg):
            self.emit('FTOI' if v.ty=='int' else 'MOV', f'r{v.n}', f'r{self.zeroreg().n}')     # int: its own representation
    def _fresh(self, ty):
        """fresh registers of a type: an element of a local array"""
        if is_mat(ty): return MatVal([self._fresh(f'vec{ncomp2(ty)}') for _ in range(ncomp2(ty))], ty)
        if ty in self.structs: return self._decl_local_struct(ty, None)
        r=self.alloc(ty); r.tmp=False; return r
    def _like(self, v):
        """fresh registers of v's shape; a uniform array stays, it is read only"""
        if isinstance(v,MatVal): return MatVal([self._like(x) for x in v.cols], v.ty)
        if isinstance(v,StructVal): return StructVal(v.sname, {k:self._like(m) for k,m in v.members.items()})
        if isinstance(v,ArrayVal) and v.kind=='local': return ArrayVal(v.ety, v.size, 'local', elems=[self._like(x) for x in v.elems])
        if isinstance(v,Reg): r=self.alloc(v.ty); r.tmp=False; return r
        return v
    def _copy_val(self, d, s):
        """a whole value, register by register: the lane mask holds and nothing is shared"""
        if d is s or s is None: return
        if isinstance(d,MatVal):
            for a,b in zip(d.cols,s.cols): self._copy_val(a,b)
        elif isinstance(d,StructVal):
            for k in d.members:
                if d.members[k] is None: d.members[k]=self._like(s.members[k])
                self._copy_val(d.members[k], s.members[k])
        elif isinstance(d,ArrayVal):
            if d.kind!='local': raise CompileError("cannot copy a uniform array as a whole")
            for i,a in enumerate(d.elems): self._copy_val(a, self._arr_elem(s,i))
        else:
            s=self._asi(s) if d.ty=='int' else self._asf(s)
            self.emit('MOV', f'r{d.n}', f'r{s.n}')
    def _clone(self, v):
        d=self._like(v); self._copy_val(d, v); return d
    def _arr_elem(self, base, i):
        """element i of an array, at a constant index"""
        if base.kind=='local': return base.elems[i]
        if base.kind=='varying':
            if is_mat(base.ety):                                # one varying vector per column
                n=ncomp2(base.ety); cols=[self.alloc(f'vec{n}') for _ in range(n)]
                for j,c in enumerate(cols): self._vload(f'r{c.n}', base.base+i*n+j)
                return MatVal(cols, base.ety)
            reg=self.alloc(base.ety); self._vload(f'r{reg.n}', base.base+i); return reg
        if base.dyn is not None:                                # a constant index under a run-time struct offset
            return self._load_uniform_member_dyn(base.base + i*self._type_nslots(base.ety), base.ety, None, base.dyn)
        if base.ety in self.structs:                            # uniform array of structs -> element StructVal
            nsl=self._type_nslots(base.ety)
            return self._load_uniform_struct(base.base + i*nsl, base.ety)
        per=ncomp2(base.ety) if is_mat(base.ety) else 1         # uniform array: load element i's slot(s)
        if is_mat(base.ety):
            cols=[self.alloc(f'vec{ncomp2(base.ety)}') for _ in range(ncomp2(base.ety))]
            for j,c in enumerate(cols): self.emit('LDP', f'r{c.n}', base.base+i*per+j)
            return MatVal(cols, base.ety)
        reg=self.alloc(base.ety); reg.tmp=False; self.emit('LDP', f'r{reg.n}', base.base+i*per)
        if base.ety=='int': self.emit('FTOI', f'r{reg.n}', f'r{reg.n}')   # stored as a float
        return reg
    def _sel_val(self, c, a, b):
        """c ? a : b of a matrix or a struct"""
        if isinstance(a,MatVal): return MatVal([self._sel_val(c,x,y) for x,y in zip(a.cols,b.cols)], a.ty)
        if isinstance(a,StructVal): return StructVal(a.sname, {k:self._sel_val(c,a.members[k],b.members[k]) for k in a.members})
        r=self.alloc(a.ty); self.emit('SEL', f'r{r.n}', f'r{c.n}', f'r{a.n}', f'r{b.n}'); return r
    # ---- a run-time index that is not LDPX's (a vector, a matrix, a local or varying array, a uniform array in the
    # fragment core, which has no indirect load): every element read, then one SEL per element ----
    def _dyn_elems(self, base):
        if isinstance(base,MatVal): return list(base.cols)
        if isinstance(base,ArrayVal): return [self._arr_elem(base,i) for i in range(base.size)]
        if isinstance(base,Reg): return [self.extract(base,i) for i in range(ncomp(base.ty))]
        raise CompileError("an index into this needs a constant expression")
    def _ieq(self, ireg, i):
        c=self.alloc('bool'); self.emit('ISEQ', f'r{c.n}', f'r{ireg.n}', f'r{self._asi_lit(i).n}'); return c
    def _dyn_read(self, base, idx):
        """base[idx] per lane: an index past the end reads element 0"""
        if not self.isa64: raise CompileError("array/vector index must be a constant expression (ES2 allows a run-time index on uniform arrays in the vertex shader only)")
        ireg=self._asi(self.eval(idx)); el=self._dyn_elems(base); r=el[0]
        for i in range(1,len(el)): r=self._sel_val(self._ieq(ireg,i), el[i], r)
        return r
    def _dyn_write(self, base, idx, rv):
        """base[idx] = rv per lane: each element takes rv where the index is its own"""
        if not self.isa64: raise CompileError("array index must be a constant expression")
        ireg=self._asi(self.eval(idx))
        if isinstance(base,Reg):
            for i in range(ncomp(base.ty)):
                n=self.alloc('float'); self.emit('SEL', f'r{n.n}', f'r{self._ieq(ireg,i).n}', f'r{rv.n}', self.swz_src(base,i))
                self.emit(f'MOV.{"xyzw"[i]}', f'r{base.n}', f'r{n.n}')
            return
        if isinstance(base,ArrayVal) and base.kind!='local': raise CompileError("cannot write this array")
        el=base.cols if isinstance(base,MatVal) else base.elems
        for i,d in enumerate(el):
            self._copy_val(d, self._sel_val(self._ieq(ireg,i), rv, d))

    def fold_const(self, e):
        """Return a Python number if `e` is a compile-time constant, else None. Drives loop unrolling + bounds evaluation."""
        return fold_expr(e, self._const_get)

    def _fd(self, e):
        """gl_FragData[i] is gl_FragColor: one draw buffer. i may be any constant expression, a loop's index too"""
        if e[0]=='index' and e[1]==('var','gl_FragData'):
            if self.fold_const(e[2])!=0: raise CompileError("gl_FragData index out of range: MAX_DRAW_BUFFERS is 1")
            return ('var','gl_FragColor')
        return e
    def _effects(self, e):
        """evaluating e does more than give a value: it writes something, or calls something that may"""
        if isinstance(e,list): return any(self._effects(x) for x in e)
        if not isinstance(e,tuple) or not e: return False
        if e[0] in ('assign','postop'): return True
        if e[0]=='call' and (self.callmap.get(id(e)) is not None or e[1] in self.funcs): return True
        return any(self._effects(x) for x in e[1:])
    def _branch(self, c, then, other):
        """c ? then : other with only one side run, per lane; then, other: expressions, or a value for other"""
        self.emit('SETP', 'SNE', f'r{c.n}', f'r{self.zeroreg().n}')
        lelse=self.newlabel('Lelse'); lend=self.newlabel('Lend')
        self.emit('IF', lelse)
        rv=self._asf(self.eval(then)); d=self._like(rv); self._copy_val(d, rv)
        self.emit(f'{lelse}:'); self.emit('ELSE', lend)
        self._copy_val(d, self._asf(self.eval(other)) if isinstance(other,tuple) else other)
        self.emit(f'{lend}:'); self.emit('ENDIF')
        return d

    def eval(self, e):
        k=e[0]
        if k=='seq':                                 # comma operator: evaluate all, value = last
            r=None
            for sub in e[1]:
                if isinstance(r,Reg): self.freer(r)
                r=self.eval(sub)
            return r
        if k=='__reg__': return e[1]                 # a pre-materialized Reg embedded in a synthetic AST
        if k=='num':
            return self.kf(e[1])
        if k=='var':
            cv=self._const_get(e[1])
            if cv is not None: return self.kf(float(cv))
            return self.load_var(e[1])
        if k=='swizzle':
            base=self.eval(e[1])
            if isinstance(base, StructVal):        # `.field` is a struct member, not a vector swizzle
                if e[2] not in base.members: raise CompileError(f"struct {base.sname} has no member {e[2]!r}")
                return base.members[e[2]]
            return self.swizzle_read(base, e[2])
        if k=='index':
            fd=self._fd(e)
            if fd is not e: return self.eval(fd)
            base=self.eval(e[1])
            ci=self.fold_const(e[2])                    # a compile-time constant (literal or unrolled loop var), or
            if ci is None:                              # a run-time index
                if isinstance(base, ArrayVal) and base.kind=='uniform' and self.stage=='vs' and self.isa64:
                    return self._uniform_dyn_index(base, e[2])      # ES2 Appendix A.5 (the 64-bit core's LDPX)
                if isinstance(base, Sampler): raise CompileError("a sampler's index must be a constant expression")
                return self._dyn_read(base, e[2])
            i=int(ci)
            if isinstance(base, Sampler):
                if base.size is None: raise CompileError("indexing a sampler that is not an array")
                if i<0 or i>=base.size: raise CompileError(f"array index {i} out of range [0,{base.size})")
                return Sampler(base.unit+i, base.ty)
            if isinstance(base, ArrayVal):
                if i<0 or i>=base.size: raise CompileError(f"array index {i} out of range [0,{base.size})")
                return self._arr_elem(base, i)
            if isinstance(base, MatVal): return base.cols[i]
            return self.extract(base, i)
        if k=='unary':
            op=e[1]; a=self.eval(e[2])
            if op=='+': return a
            if op=='-' and isinstance(a,MatVal):
                cols=[]
                for c in a.cols: r=self.alloc(c.ty); self.emit('MOV', f'r{r.n}', f'-r{c.n}'); cols.append(r)
                return MatVal(cols, a.ty)
            if op=='-':                                    # negate = MOV sign-flip (is_simp, lat 1) — much
                if self.fold_cvec and isinstance(a,Reg) and a.ty=='float' and a.const is not None and a.const!=0.0: return self.kf(-a.const)   # known: a constant (not 0: -0.0 is a value of its own)
                if self.fold_cvec and isinstance(a,Reg) and a.ty in ('vec2','vec3','vec4') and a.cvec is not None and all(x!=0.0 for x in a.cvec): return self.kvec(a.ty, [-x for x in a.cvec])
                if isinstance(a,Reg) and a.ty=='int':      # int negate: 0 - a (sign-flip is a float trick)
                    r=self.alloc('int'); self.emit('ISUB', f'r{r.n}', f'r{self._asi_lit(0).n}', f'r{a.n}'); self.freer(a); return r
                r=self.alloc(a.ty); self.emit('MOV', f'r{r.n}', f'-r{a.n}'); self.freer(a); return r  # cheaper than SUB via madd
            if op=='!':
                one=self.kf(1.0); r=self.alloc('float'); self.emit('SUB', f'r{r.n}', f'r{one.n}', f'r{a.n}'); return r
        if k=='bin':
            return self.binop(e[1], e[2], e[3])
        if k=='call':
            fn=self.callmap.get(id(e))
            return self.call(fn.name if fn is not None else e[1], e[2])
        if k=='ternary':
            c=self.eval(e[1])
            if self.isa64 and (self._effects(e[2]) or self._effects(e[3])): return self._branch(c, e[2], e[3])   # only one side runs
            a=self._asf(self.eval(e[2])); b=self._asf(self.eval(e[3]))
            if self.isa64 and isinstance(a,(MatVal,StructVal)): return self._sel_val(c,a,b)
            if self.isa64:                         # exact per-lane select (Inf / NaN in the unselected side is harmless)
                r=self.alloc(a.ty if ncomp(a.ty)>=ncomp(b.ty) else b.ty)
                self.emit('SEL', f'r{r.n}', f'r{c.n}', f'r{a.n}', f'r{b.n}'); return r
            # 32-bit ISA: select via mix (c in {0,1}: mix(b,a,c) = c ? a : b)
            return self.builtin_mix(b,a,c)
        if k=='assign':
            return self.do_assign(e)
        if k=='postop':
            h=self._const_home(e[1][1]) if e[1][0]=='var' else None
            if h is not None:                              # compile-time loop var: step the constant
                old=h[e[1][1]]; nv=old+(1 if e[2]=='++' else -1)
                h[e[1][1]]=IntLit(nv) if isinstance(old,IntLit) else nv
                return self.kf(float(old))
            cur=self.eval(e[1])
            if isinstance(cur,MatVal):
                one=self.kf(1.0); r=MatVal([], cur.ty)
                for c in cur.cols:
                    x=self.alloc(c.ty); self.emit('ADD' if e[2]=='++' else 'SUB', f'r{x.n}', f'r{c.n}', f'r{one.n}'); r.cols.append(x)
                old=self._clone(cur)
                self.store(e[1], r); return old
            if isinstance(cur,Reg) and cur.ty=='int':      # int-repr counter: IADD/ISUB 1 (latency 1, off the recurrence)
                r=self.alloc('int'); self.emit('IADD' if e[2]=='++' else 'ISUB', f'r{r.n}', f'r{cur.n}', f'r{self._asi_lit(1).n}')
            else:
                one=self.kf(1.0); r=self.alloc(cur.ty)
                self.emit('ADD' if e[2]=='++' else 'SUB', f'r{r.n}', f'r{cur.n}', f'r{one.n}')
            old=self.alloc(cur.ty); self.emit('MOV', f'r{old.n}', f'r{cur.n}')   # cur is the variable itself: the value before the step
            self.store(e[1], r); return old
        raise CompileError(f"cannot eval {k}")

    def _arith(self, op, a, b, ty):
        """a op b, lane by lane, with the shapes the caller settled"""
        r=self.alloc(ty)
        if op=='+': self.emit('ADD', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='-': self.emit('SUB', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='*': self.emit('MUL', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        else:
            inv=self.alloc(b.ty); self.emit('RCP', f'r{inv.n}', f'r{b.n}'); self.emit('MUL', f'r{r.n}', f'r{a.n}', f'r{inv.n}')
        return r
    def mat_op(self, op, a, b):
        """§5.9: * is the linear product of matrices and vectors; everything else, and any matrix with a scalar, is by component"""
        ma=isinstance(a,MatVal); mb=isinstance(b,MatVal)
        if op=='*' and (ma and mb or (ma and not is_scalar(b.ty)) or (mb and not is_scalar(a.ty))): return self.mat_mul(a,b,op)
        m=a if ma else b
        if op not in ('+','-','*','/'): raise CompileError(f"op {op} unsupported")
        return MatVal([self._arith(op, a.cols[j] if ma else a, b.cols[j] if mb else b, m.cols[j].ty) for j in range(len(m.cols))], m.ty)
    def _leaves(self, a, b):
        if isinstance(a,MatVal):
            for x,y in zip(a.cols,b.cols): yield x,y
        elif isinstance(a,StructVal):
            for k in a.members: yield from self._leaves(a.members[k], b.members[k])
        else: yield self._asf(a),self._asf(b)
    def agg_eq(self, op, a, b):
        """== and != of matrices and structs: one bool"""
        acc=None
        for x,y in self._leaves(a,b):
            c=self._setc('SEQ', x, y)
            if ncomp(x.ty)>1: c=self.bi_all(c)
            if acc is None: acc=c
            else: t=self.alloc('float'); self.emit('MIN', f'r{t.n}', f'r{acc.n}', f'r{c.n}'); acc=t
        r=self.alloc('bool')
        if op=='==': self.emit('MOV', f'r{r.n}', f'r{acc.n}')
        else: self.emit('SUB', f'r{r.n}', f'r{self.kf(1.0).n}', f'r{acc.n}')
        return r

    def binop(self, op, ea, eb):
        # constant folding + algebraic identities (fewer instructions; DCE cleans up any dead operand)
        if ea[0]=='num' and eb[0]=='num' and op in ('+','-','*','/'):
            return self.kf(float(fold_expr(('bin',op,ea,eb), self._const_get)))
        if op in ('&&','||') and self.isa64 and self._effects(eb):      # the right side runs only where the left leaves it open
            a=self._asf(self.eval(ea))
            if op=='&&': return self._branch(a, eb, self.kf(0.0))
            n=self.alloc('float'); self.emit('SET', 'SEQ', f'r{n.n}', f'r{a.n}', f'r{self.zeroreg().n}')
            return self._branch(n, eb, self.kf(1.0))
        if op=='*':
            if eb==('num',1.0): return self.eval(ea)
            if ea==('num',1.0): return self.eval(eb)
            if ea==('num',0.0) or eb==('num',0.0):                     # x*0 -> 0 (ES2 exprs are side-effect-free)
                x=self.eval(eb if ea==('num',0.0) else ea)
                if not isinstance(x,MatVal): return self.kf(0.0)
            # x*2 -> x+x : ADD and MUL cost the same here (both is_madd, lat 8), but x+x needs no constant.
            xe = ea if eb==('num',2.0) else eb if ea==('num',2.0) else None
            if xe is not None:
                x=self.eval(xe)
                if isinstance(x,Reg):
                    r=self.alloc(x.ty)
                    self.emit('IADD' if x.ty=='int' else 'ADD', f'r{r.n}', f'r{x.n}', f'r{x.n}'); return r   # 2x = x+x
        if op in ('+','-') and eb==('num',0.0): return self.eval(ea)
        if op=='+' and ea==('num',0.0): return self.eval(eb)
        a=self.eval(ea); b=self.eval(eb)
        # a register known to hold 0.0 as the literal above (a parameter bound to 0.0: sdPlane(p, n, 0.0)'s `+ h`)
        if op in ('+','-') and self._fzero(b) and isinstance(a,Reg) and a.ty in ('float','vec2','vec3','vec4'): return a
        if op=='+' and self._fzero(a) and isinstance(b,Reg) and b.ty in ('float','vec2','vec3','vec4'): return b
        if op in ('==','!=') and (isinstance(a,StructVal) or (isinstance(a,MatVal) and isinstance(b,MatVal))): return self.agg_eq(op,a,b)
        # integer scalar fast path: +,-,relational stay in int-repr (IADD/ISUB/ISLT/ISGE). A num literal
        # paired with an int operand is materialized as int; a real float operand promotes the int to float.
        if self.isa64 and id(ea) in self.ints and id(eb) in self.ints: a=self._asi(a); b=self._asi(b)
        ai=isinstance(a,Reg) and a.ty=='int'; bi=isinstance(b,Reg) and b.ty=='int'
        if (ai or bi) and op in ('/','*') and self.isa64:        # IDIV (truncates toward zero), IMUL
            if not bi and eb[0]=='num' and float(eb[1]).is_integer(): b=self._asi_lit(eb[1]); bi=True
            if not ai and ea[0]=='num' and float(ea[1]).is_integer(): a=self._asi_lit(ea[1]); ai=True
            if ai and bi:
                if op=='/' and not self.hw_idiv: r=self._idiv_lowered(a, b, 'int'); self.freer(a); self.freer(b); return r
                r=self.alloc('int'); self.emit('IDIV' if op=='/' else 'IMUL', f'r{r.n}', f'r{a.n}', f'r{b.n}'); self.freer(a); self.freer(b); return r
        if (ai or bi) and op in ('+','-','<','>','<=','>=','==','!='):
            if not bi and eb[0]=='num' and float(eb[1]).is_integer(): b=self._asi_lit(eb[1]); bi=True
            if not ai and ea[0]=='num' and float(ea[1]).is_integer(): a=self._asi_lit(ea[1]); ai=True
            if ai and bi and (self.isa64 or op not in ('==','!=')): return self.int_binop(op, a, b)
            a=self._asf(a); b=self._asf(b)                       # mixed int/float -> promote, use float ops
        elif ai or bi:                                           # *, /, ==, != etc.: compute in float
            a=self._asf(a); b=self._asf(b)
        # matrices
        if isinstance(a, MatVal) or isinstance(b, MatVal):
            return self.mat_op(op,a,b)
        if op=='/' and self.isa64 and (a.ty[:4]=='ivec' or b.ty[:4]=='ivec'):      # an int vector's lanes are floats; / truncates
            t=a.ty if a.ty[:4]=='ivec' else b.ty; x=self.alloc(t); y=self.alloc(t); q=self.alloc(t); r=self.alloc(t)
            self.emit('FTOI', f'r{x.n}', f'r{a.n}'); self.emit('FTOI', f'r{y.n}', f'r{b.n}')
            if not self.hw_idiv: q=self._idiv_lowered(x, y, t)
            else: self.emit('IDIV', f'r{q.n}', f'r{x.n}', f'r{y.n}')
            self.emit('ITOF', f'r{r.n}', f'r{q.n}'); return r
        rt = a.ty if ncomp(a.ty)>=ncomp(b.ty) else b.ty
        r=self.alloc(rt)
        if op=='+': self.emit('ADD', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='-': self.emit('SUB', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='*': self.emit('MUL', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='/':
            inv=self.alloc(b.ty); self.emit('RCP', f'r{inv.n}', f'r{b.n}')
            self.emit('MUL', f'r{r.n}', f'r{a.n}', f'r{inv.n}'); self.freer(inv)
            n=ncomp(rt)
            if n in (2,3) and not self.isa64:   # vec2/vec3 divide: RCP fills the divisor's UNUSED upper lanes with +inf (RCP(0)),
                z=self.zeroreg()   # so zero them in the result — else a later swizzle-extract (DP4 w/ a basis
                self.emit(f'MOV.{"xyzw"[n:]}', f'r{r.n}', f'r{z.n}')   # vector) does inf*0 = NaN at FP16 (e.g. fragCoord/iResolution.xy)
                                   # (isa64 reads swizzled lanes directly: unused lanes never enter a sum)
        elif op in ('<','>','<=','>=','==','!='):
            if op in ('==','!=') and ncomp(a.ty)>1:      # vector ==/!= -> ONE bool (all-equal / any-not-eq)
                cw=self.bi_equal(a,b) if op=='==' else self.bi_notEqual(a,b)
                red=self.bi_all(cw) if op=='==' else self.bi_any(cw)
                self.emit('MOV', f'r{r.n}', f'r{red.n}'); r.ty='bool'; self.freer(cw); self.freer(red)
            else:
                self.cmp(r,a,b,op)
        elif op=='&&': self.emit('MUL', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='||': self.emit('MAX', f'r{r.n}', f'r{a.n}', f'r{b.n}')
        elif op=='^^': self.cmp(r,a,b,'!=')
        else: raise CompileError(f"op {op} unsupported")
        self.freer(a); self.freer(b); return r
    def _fzero(self, v): return isinstance(v,Reg) and v.ty=='float' and v.const==0.0
    def cmp(self, r,a,b,op):
        # EXACT compare -> 1.0/0.0 via the SET instruction (float_cmp), no multiply-clamp approximation.
        # a>b == b<a ; a<=b == b>=a. Relational ops are scalar in GLSL ES (broadcast, so per-lane is fine).
        SUB={'<':('SLT',a,b),'>':('SLT',b,a),'<=':('SGE',b,a),'>=':('SGE',a,b),'==':('SEQ',a,b),'!=':('SNE',a,b)}
        cc,x,y=SUB[op]; self.emit('SET', cc, f'r{r.n}', f'r{x.n}', f'r{y.n}')

    def mat_mul(self, a, b, op):
        if isinstance(a,MatVal) and isinstance(b,MatVal):   # M * N = columns M*(N.col_j)
            n=ncomp2(a.ty); return MatVal([self.mat_mul(a, b.cols[j], '*') for j in range(n)], a.ty)
        if isinstance(a,MatVal) and isinstance(b,Reg):      # M * v = sum_j v[j]*col_j
            n=ncomp2(a.ty); acc=self.alloc(f'vec{n}')
            first=True
            for j in range(n):
                vj=self.swz_src(b,j)                             # v[j] broadcast (isa64: the operand swizzle)
                if first: self.emit('MUL', f'r{acc.n}', f'r{a.cols[j].n}', vj); first=False
                else:
                    t=self.alloc(f'vec{n}'); self.emit('MUL', f'r{t.n}', f'r{a.cols[j].n}', vj)
                    self.emit('ADD', f'r{acc.n}', f'r{acc.n}', f'r{t.n}'); self.freer(t)
            return acc
        if isinstance(a,Reg) and isinstance(b,MatVal):      # v * M = row-vector x matrix: (v*M)[j] = dot(v, M.col_j)
            n=ncomp2(b.ty); r=self.alloc(f'vec{n}')          # == transpose(M)*v (GLSL ES column-major identity)
            for j in range(n):
                dj=self.bi_dot(a, b.cols[j])                 # scalar (broadcast to all lanes)
                self.emit(f'MOV.{"xyzw"[j]}', f'r{r.n}', f'r{dj.n}'); self.freer(dj)
            return r
        raise CompileError("only mat*vec / vec*mat supported for matrices")

    def inline_call(self, name, argexprs):
        """Inline a user-defined function (GLSL ES forbids recursion, so inlining always terminates).
        `in` params are copied by value; `out`/`inout` alias the caller's lvalue so writes propagate back.
        `return expr` writes a per-call result register and BRAnches to the call's epilogue label."""
        if name in self._inlining: raise CompileError(f"recursive call to {name!r} (forbidden in GLSL ES)")
        rt,params,body=self.funcs[name]
        if len(params)!=len(argexprs): raise CompileError(f"{name}(): expected {len(params)} args, got {len(argexprs)}")
        binds=Scope(); copyout=[]; outs=[]                           # bind args in the CALLER's scope first
        wrt=self._written([body], {})                                # a parameter the body never writes keeps a known value (const, cvec)
        bases=[self._base(ae) for (pq,_,_),ae in zip(params,argexprs) if pq in ('out','inout')]
        for (pq,pty,pname),ae in zip(params, argexprs):              # (arg eval may nest same-name calls: not recursion)
            agg=None; shared=False
            if pq in ('out','inout'):
                if ae[0]=='var' and self._const_home(ae[1]) is not None: self._promote(ae[1])      # a register to write
                b=self._base(ae)                                     # §6.1.1 copies in and out: an alias would show when another
                shared=b is not None and (bases.count(b)>1 or self._sees(body, b, {name}, b in {p[2] for p in params}))   # argument or the function sees it
                if shared and not self._no_alias(ae):
                    lv=self.get_lvalue_reg(ae)
                    if isinstance(lv,(MatVal,ArrayVal,StructVal)): agg=lv
            if agg is not None:                                      # an array, struct or matrix of its own, written back at the end
                pr=self._clone(agg) if pq=='inout' else self._like(agg)
                binds[pname]=(pr,pty); copyout.append((ae,pr))
                if pq=='out': outs.append(pr)
            elif pq in ('out','inout') and (self._no_alias(ae) or self._repr_differs(ae, pty) or shared):   # no register to alias: a copy, written back at the end
                pr=self.alloc(pty); pr.tmp=False
                if pq=='inout':
                    av=self.eval(ae); av=self._asi(av) if pty=='int' else self._asf(av)
                    self.emit('MOV', f'r{pr.n}', f'r{av.n}')
                binds[pname]=(pr,pty); copyout.append((ae,pr))
                if pq=='out': outs.append(pr)
            elif pq in ('out','inout'):
                binds[pname]=(self.get_lvalue_reg(ae), pty)          # alias caller's storage (promote if const)
                if pq=='out': outs.append(binds[pname][0])
            else:
                av=self.eval(ae)
                if isinstance(av,(MatVal,ArrayVal,Sampler,StructVal)): binds[pname]=(self._clone(av) if self.isa64 else av, pty)
                else:
                    av=self._asi(av) if pty=='int' else self._asf(av)    # match the parameter's representation
                    pr=self.alloc(pty); pr.tmp=False; self.emit('MOV', f'r{pr.n}', f'r{av.n}'); binds[pname]=(pr,pty)
                    if pname not in wrt and pty==av.ty: pr.const=av.const; pr.cvec=av.cvec
        for o in outs: self._zero(o)                                 # an out parameter starts at 0 as a local does (_zero); after
        #                                                              every argument is read: f(x, x) with in and out the same variable
        retreg=self.alloc(rt if rt!='void' else 'float'); retreg.tmp=False
        holder=[None]                                                # captures an aggregate (struct/mat/array) return
        lret=self.newlabel('Lret'); saved=self.cur_ret; flow=self._flow_open(body); self.cur_ret=(retreg,lret,rt,holder,flow)
        self._inlining.add(name)                                     # guard the BODY (recursion = body calls itself)
        outer=self.locals; self.locals=[outer[0], binds]             # a function sees its own names and, in [0], the outputs kept in locals
        self.run_stmt(body)
        self.locals=outer
        self._inlining.discard(name)
        if flow: self.emit('ENDL')
        self.emit(f'{lret}:')
        self.cur_ret=saved
        for ae,pr in copyout: self.store(ae, pr)
        return holder[0] if holder[0] is not None else retreg
    def _base(self, e):
        """the variable an lvalue writes: v of v.xy, a[i], s.m"""
        while e[0] in ('swizzle','index'): e=e[1]
        return e[1] if e[0]=='var' else None
    def _sees(self, e, name, seen, own=False):
        """does a function body read or write `name`, also through the functions it calls (a local of the name counts too);
        own: the function has a parameter of the name, so only the functions it calls can see the caller's"""
        if isinstance(e,list): return any(self._sees(x, name, seen, own) for x in e)
        if not isinstance(e,tuple) or not e: return False
        if e[0]=='var': return e[1]==name and not own
        if e[0]=='call':
            fn=self.callmap[id(e)].name if id(e) in self.callmap else e[1]
            if fn in self.funcs and fn not in seen:
                seen.add(fn)
                if self._sees(self.funcs[fn][2], name, seen): return True
        return any(self._sees(x, name, seen, own) for x in e[1:])
    def _repr_differs(self, e, pty):
        """a local whose register holds its value in the other representation than the parameter's: a promoted int is a float"""
        lt=self._local_type(e[1]) if e[0]=='var' else None
        return lt is not None and (lt=='int')!=(pty=='int')
    def _no_alias(self, e):
        """an out / inout argument without a register of its own: part of a vector"""
        if e[0]=='index': return isinstance(self.eval_lvalue(e[1]), Reg)
        return e[0]=='swizzle' and not isinstance(self.eval_lvalue(e[1]), StructVal)
    def _early_return(self, s, under):
        """a return under an if or in a loop (under: any return at all)"""
        if not isinstance(s,tuple): return False
        k=s[0]
        if k=='return': return under
        if k in ('block','multi'): return any(self._early_return(x, under) for x in s[1])
        if k=='if': return self._early_return(s[2],True) or (s[3] is not None and self._early_return(s[3],True))
        if k=='for': return self._early_return(s[4],True)
        if k in ('while','dowhile'): return self._early_return(s[2],True)
        return False
    def _flow_open(self, body):
        """A return under an if or in a loop leaves in some lanes only, so on the SIMT cores it cannot be a BRA (the others
        would be lost and the mask stack left open): the body runs as a LOOP of one iteration and the return is a per-lane
        BREAK, which ELSE, ENDIF and ENDL honour. Inside a hardware loop that BREAK leaves only that loop: `left` marks the
        lanes for one more BREAK after its ENDL (_hw_loop)."""
        if not (self.isa64 and self._early_return(body, False)): return None
        f={'depth':0,'any':False,'left':self.alloc('float')}; f['left'].tmp=False
        self.emit('MOV', f'r{f["left"].n}', f'r{self.kf(0.0).n}')
        self.emit('LOOP', 1)
        return f
    def _hw_loop(self, d):
        f=self.cur_ret[4] if self.cur_ret is not None else None
        if f is None: return
        f['depth']+=d
        if d<0 and f['any']:
            self.emit('SETP','SNE', f'r{f["left"].n}', f'r{self.zeroreg().n}'); self.emit('BREAK')
    def run_main(self, body):
        """main: a return ends the program, past the code that writes the outputs kept in locals"""
        if not (self.isa64 and self._early_return(body, True)): self.run_stmt(body); return
        retreg=self.alloc('float'); retreg.tmp=False
        lret=self.newlabel('Lret'); flow=self._flow_open(body); self.cur_ret=(retreg,lret,'void',[None],flow)
        self.run_stmt(body)
        if flow: self.emit('ENDL')
        self.emit(f'{lret}:')
        self.cur_ret=None

    def call(self, name, argexprs):
        if name in ('float','int','bool') and len(argexprs)==1:      # scalar cast constructor
            cv=self.fold_const(('call',name,argexprs))
            if cv is not None: return self.kf(float(cv))
            a=self.eval(argexprs[0])
            if isinstance(a,MatVal): a=self.extract(a.cols[0],0)     # of a vector or a matrix: its first component
            elif isinstance(a,Reg) and ncomp(a.ty)>1: a=self.extract(a,0)
            a=self._asf(a)                                           # normalize arg to float-repr first
            # int() -> int-repr (FTOI truncates toward zero); float()/bool() -> float-repr.
            r=self.alloc('float' if name=='bool' else name)
            if name=='int':                                          # truncate toward zero -> int-repr
                self.emit('FTOI', f'r{r.n}', f'r{a.n}'); self.freer(a); return r
            if name=='bool' and self.isa64:
                self.emit('SET', 'SNE', f'r{r.n}', f'r{a.n}', f'r{self.zeroreg().n}')
            elif name=='bool':                                       # x!=0 -> 1.0/0.0
                self.emit('MAX', f'r{r.n}', f'!r{a.n}', f'r{a.n}')   # |x|
                self.emit('MUL', f'r{r.n}', f'r{r.n}', f'r{self.kf(1e6).n}')
                self.emit('MIN', f'r{r.n}', f'r{r.n}', f'r{self.kf(1.0).n}')
            else:
                self.emit('MOV', f'r{r.n}', f'r{a.n}')
            self.freer(a); return r
        if name in VEC or name in MAT:
            args=[self.eval(a) for a in argexprs]
            r=self.construct_mat(name, args) if name in MAT else self.construct(name, args)
            for a in args:
                if isinstance(a, Reg): self.freer(a)
            return r
        if name in self.structs:                                     # struct constructor: Light(pos,color,..)
            args=[self.eval(a) for a in argexprs]
            mem=self.structs[name]
            if len(args)!=len(mem): raise CompileError(f"{name}(): expected {len(mem)} members, got {len(args)}")
            return StructVal(name, {mn:av for (mt,mn,ma),av in zip(mem,args)})
        if name in self.funcs:                                       # user-defined function -> inline it
            return self.inline_call(name, argexprs)
        args=[self.eval(a) for a in argexprs]
        r=self.builtin(name, args)
        for a in args:
            if a is not r and isinstance(a, Reg): self.freer(a)
        return r

    # ---- builtins ----
    def builtin(self, name, a):
        f=getattr(self, 'bi_'+name, None)
        if f is None: raise CompileError(f"unknown function/builtin {name!r}")
        a=[self._asf(x) for x in a]                       # builtins operate in float; promote int args
        return f(*a)
    def _un(self, op, x):                          # elementwise 1-arg alu op
        r=self.alloc(x.ty); self.emit(op, f'r{r.n}', f'r{x.n}'); return r
    def bi_abs(self,x):  r=self.alloc(x.ty); self.emit('MOV', f'r{r.n}', f'!r{x.n}'); return r   # abs = MOV clear-sign (lat 1)
    def bi_floor(self,x):
        if self.isa64: return self._un('FLR', x)                 # exact float rounding unit (no FTOI saturation)
        t=self.alloc(x.ty); self.emit('FTOI', f'r{t.n}', f'r{x.n}'); self.emit('ITOF', f'r{t.n}', f'r{t.n}')  # trunc->0
        # trunc rounds toward 0; floor rounds toward -inf. They differ only for NEGATIVE non-integers, where
        # trunc(x)=floor(x)+1. correction = (x < trunc) ? 1 : 0, computed EXACTLY with SET SLT.
        c=self.alloc(x.ty); self.emit('SET', 'SLT', f'r{c.n}', f'r{x.n}', f'r{t.n}')   # (x < trunc) ? 1 : 0
        r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{t.n}', f'r{c.n}')          # floor = trunc - correction
        self.freer(t); self.freer(c); return r
    def bi_fract(self,x):
        fl=self.bi_floor(x); r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{x.n}', f'r{fl.n}'); self.freer(fl); return r
    def bi_sign(self,x):                       # sign = (x>0) - (x<0), EXACT via SET SLT (gives -1/0/+1)
        z=self.zeroreg()
        p=self.alloc(x.ty); self.emit('SET', 'SLT', f'r{p.n}', f'r{z.n}', f'r{x.n}')   # 0 < x  -> x>0
        n=self.alloc(x.ty); self.emit('SET', 'SLT', f'r{n.n}', f'r{x.n}', f'r{z.n}')   # x < 0
        r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{p.n}', f'r{n.n}')
        self.freer(p); self.freer(n); return r
    def bi_min(self,x,y): r=self.alloc(x.ty); self.emit('MIN', f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
    def bi_max(self,x,y): r=self.alloc(x.ty); self.emit('MAX', f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
    # ---- vector relational (component-wise -> bvecN, EXACT via SET) + any/all/not ----
    def _setc(self,cc,x,y): r=self.alloc(x.ty); self.emit('SET', cc, f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
    def bi_lessThan(self,x,y):         return self._setc('SLT', x, y)
    def bi_lessThanEqual(self,x,y):    return self._setc('SGE', y, x)   # x<=y == y>=x
    def bi_greaterThan(self,x,y):      return self._setc('SLT', y, x)   # x>y  == y<x
    def bi_greaterThanEqual(self,x,y): return self._setc('SGE', x, y)
    def bi_equal(self,x,y):            return self._setc('SEQ', x, y)
    def bi_notEqual(self,x,y):         return self._setc('SNE', x, y)
    def _reduce(self,op,x):            # fold components 0..n-1 with MAX(any)/MIN(all); inputs are 0/1
        n=ncomp(x.ty); acc=self.extract(x,0)
        for i in range(1,n):
            e=self.extract(x,i); t=self.alloc('float'); self.emit(op, f'r{t.n}', f'r{acc.n}', f'r{e.n}')
            self.freer(acc); self.freer(e); acc=t
        return acc
    def bi_any(self,x): return self._reduce('MAX', x)   # OR of bools
    def bi_all(self,x): return self._reduce('MIN', x)   # AND of bools
    def bi_not(self,x): one=self.kf(1.0); r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{one.n}', f'r{x.n}'); return r
    def bi_clamp(self,x,lo,hi):
        if self.xops&2:                       # board B's CLAMP (xops bit 1): MIN(MAX(x, lo), hi) in one micro-op a component, bit for bit
            r=self.alloc(x.ty); self.emit('CLAMP', f'r{r.n}', f'r{x.n}', f'r{lo.n}', f'r{hi.n}'); return r
        t=self.bi_max(x,lo); r=self.bi_min(t,hi); self.freer(t); return r
    def bi_mix(self,x,y,a): return self.builtin_mix(x,y,a)
    def builtin_mix(self,x,y,a):                   # x*(1-a)+y*a
        one=self.kf(1.0); ia=self.alloc(a.ty); self.emit('SUB', f'r{ia.n}', f'r{one.n}', f'r{a.n}')
        t1=self.alloc(x.ty); self.emit('MUL', f'r{t1.n}', f'r{x.n}', f'r{ia.n}')
        t2=self.alloc(y.ty); self.emit('MUL', f'r{t2.n}', f'r{y.n}', f'r{a.n}')
        r=self.alloc(x.ty); self.emit('ADD', f'r{r.n}', f'r{t1.n}', f'r{t2.n}')
        self.freer(ia); self.freer(t1); self.freer(t2); return r
    def bi_step(self,edge,x):                  # step(edge,x) = (x >= edge) ? 1 : 0, EXACT via SET SGE
        r=self.alloc(x.ty); self.emit('SET', 'SGE', f'r{r.n}', f'r{x.n}', f'r{edge.n}'); return r
    def bi_smoothstep(self,e0,e1,x):
        z=self.zeroreg(); one=self.kf(1.0)
        if self.isa64 and e0.const is not None and e1.const is not None and e1.const != e0.const:
            # constant edges (the common case): t = (x - e0) / (e1 - e0) is ONE MAD with host-folded constants
            # k = 1 / (e1 - e0), b = -e0 k (no RCP, no subtract), then the clamp and t t (3 - 2 t) as a MAD
            k=1.0/(e1.const-e0.const); kk=self.kf(k); kb=self.kf(-e0.const*k)
            t=self.alloc(x.ty); self.emit('MAD', f'r{t.n}', f'r{x.n}', f'r{kk.n}', f'r{kb.n}')
        else:
            de=self.alloc(e1.ty); self.emit('SUB', f'r{de.n}', f'r{e1.n}', f'r{e0.n}')
            t=self.alloc(x.ty); self.emit('SUB', f'r{t.n}', f'r{x.n}', f'r{e0.n}')
            inv=self.alloc(e1.ty); self.emit('RCP', f'r{inv.n}', f'r{de.n}'); self.emit('MUL', f'r{t.n}', f'r{t.n}', f'r{inv.n}')
        self.emit('MAX', f'r{t.n}', f'r{t.n}', f'r{z.n}'); self.emit('MIN', f'r{t.n}', f'r{t.n}', f'r{one.n}')
        two=self.kf(2.0); three=self.kf(3.0)
        tt=self.alloc(x.ty); self.emit('MUL', f'r{tt.n}', f'r{t.n}', f'r{t.n}')      # t*t
        p=self.alloc(x.ty)
        if self.isa64: self.emit('MAD', f'r{p.n}', f'r{t.n}', f'-r{two.n}', f'r{three.n}')                  # 3 - 2t
        else: self.emit('MUL', f'r{p.n}', f'r{two.n}', f'r{t.n}'); self.emit('SUB', f'r{p.n}', f'r{three.n}', f'r{p.n}')
        r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{tt.n}', f'r{p.n}')
        return r
    def bi_mod(self,x,y):
        q=self.alloc(x.ty); inv=self.alloc(y.ty); self.emit('RCP', f'r{inv.n}', f'r{y.n}')
        self.emit('MUL', f'r{q.n}', f'r{x.n}', f'r{inv.n}'); fl=self.bi_floor(q)
        r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{fl.n}', f'r{y.n}'); self.emit('SUB', f'r{r.n}', f'r{x.n}', f'r{r.n}')
        self.freer(q); self.freer(inv); self.freer(fl); return r
    def _pad4(self,v,n):                      # copy v with lanes n..3 zeroed (vecN registers leave the
        if n>=4: return v                     # unused upper lanes UNDEFINED, which would poison a DP4 sum)
        z=self.alloc('vec4'); self.emit('MOV', f'r{z.n}', f'r{self.zeroreg().n}')
        self.emit(f'MOV.{"xyzw"[:n]}', f'r{z.n}', f'r{v.n}'); return z
    def _basis(self, v, n):
        """k when v's first n components are known to be the unit vector e_k (construct's cvec), else None"""
        cv=v.cvec if isinstance(v,Reg) else None
        if cv is None or len(cv)<n: return None
        ones=[k for k in range(n) if cv[k]==1.0]
        return ones[0] if len(ones)==1 and all(cv[k]==0.0 for k in range(n) if k!=ones[0]) else None
    def bi_dot(self,x,y):
        n=ncomp(x.ty)
        if self.isa64 and n>1:                # dot(p, vec3(0.0, 1.0, 0.0)) is p.y: one micro-op, not a DP3's four
            for v,o in ((y,x),(x,y)):
                k=self._basis(v, n)
                if k is not None: return self.extract(o, k)
        r=self.alloc('float')
        if n==1:                              # a float's lanes are all the one value: DP4 would count it four times
            self.emit('MUL', f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
        if n==3:                              # DP3 sums exactly 3 lanes -> vec3 is safe as-is
            self.emit('DP3', f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
        if n==2:
            if self.isa64:                    # 2-step chain on the swizzled lanes: x.x*y.x + x.y*y.y (= DP4 with zero lanes)
                t=self.alloc('float'); self.emit('MUL', f'r{t.n}', f'r{x.n}.x', f'r{y.n}.x')
                self.emit('MAD', f'r{r.n}', f'r{x.n}.y', f'r{y.n}.y', f'r{t.n}'); self.freer(t); return r
            # 32-bit: DP4 sums 4 lanes; zero the vec2's undefined lanes 2,3 first
            xz=self._pad4(x,2); yz=xz if y is x else self._pad4(y,2)
            self.emit('DP4', f'r{r.n}', f'r{xz.n}', f'r{yz.n}'); return r
        self.emit('DP4', f'r{r.n}', f'r{x.n}', f'r{y.n}'); return r
    def bi_length(self,x):
        d=self.bi_dot(x,x)
        if self.xops&1:                       # board B's SQRT (xops bit 0): RSQRT, MUL, SEL below in one micro-op, bit for bit
            r=self.alloc('float'); self.emit('SQRT', f'r{r.n}', f'r{d.n}'); return r
        if self.isa64:                        # d * rsqrt(d), and 0 where d == 0 (exact select; no clamp that
            rs=self.alloc('float'); self.emit('RSQRT', f'r{rs.n}', f'r{d.n}')     # falsified short vectors)
            p=self.alloc('float'); self.emit('MUL', f'r{p.n}', f'r{d.n}', f'r{rs.n}')
            r=self.alloc('float'); self.emit('SEL', f'r{r.n}', f'r{d.n}', f'r{p.n}', f'r{d.n}')
            self.freer(d); self.freer(rs); self.freer(p); return r
        dg=self.bi_max(d, self.kf(1e-4))   # rsqrt(0)=inf -> d*inf=NaN at length(0); clamp the rsqrt input off 0
        rs=self.alloc('float'); self.emit('RSQRT', f'r{rs.n}', f'r{dg.n}')            # (d itself, not dg, feeds the
        r=self.alloc('float'); self.emit('MUL', f'r{r.n}', f'r{d.n}', f'r{rs.n}')     #  multiply, so length(0)=0)
        self.freer(d); self.freer(dg); self.freer(rs); return r
    def bi_distance(self,x,y):
        t=self.alloc(x.ty); self.emit('SUB', f'r{t.n}', f'r{x.n}', f'r{y.n}'); r=self.bi_length(t); self.freer(t); return r
    def bi_sqrt(self,x):
        if self.xops&1: r=self.alloc(x.ty); self.emit('SQRT', f'r{r.n}', f'r{x.n}'); return r
        rs=self.alloc(x.ty); self.emit('RSQRT', f'r{rs.n}', f'r{x.n}'); p=self.alloc(x.ty); self.emit('MUL', f'r{p.n}', f'r{x.n}', f'r{rs.n}'); self.freer(rs)
        if not self.isa64: return p
        r=self.alloc(x.ty); self.emit('SEL', f'r{r.n}', f'r{x.n}', f'r{p.n}', f'r{x.n}'); self.freer(p); return r   # sqrt(0) = 0, not 0*inf
    def bi_inversesqrt(self,x): return self._un('RSQRT', x)
    def bi_normalize(self,x):
        d=self.bi_dot(x,x); rs=self.alloc('float'); self.emit('RSQRT', f'r{rs.n}', f'r{d.n}')
        r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{x.n}', f'r{rs.n}'); self.freer(d); self.freer(rs); return r
    def bi_cross(self,a,b):
        ayzx=self.swizzle_read(a,'yzx'); bzxy=self.swizzle_read(b,'zxy')
        azxy=self.swizzle_read(a,'zxy'); byzx=self.swizzle_read(b,'yzx')
        t1=self.alloc('vec3'); self.emit('MUL', f'r{t1.n}', f'r{ayzx.n}', f'r{bzxy.n}')
        t2=self.alloc('vec3'); self.emit('MUL', f'r{t2.n}', f'r{azxy.n}', f'r{byzx.n}')
        r=self.alloc('vec3'); self.emit('SUB', f'r{r.n}', f'r{t1.n}', f'r{t2.n}')
        for z in (ayzx,bzxy,azxy,byzx,t1,t2): self.freer(z)
        return r
    def bi_reflect(self,I,N):
        d=self.bi_dot(N,I); two=self.kf(2.0); t=self.alloc(I.ty)
        self.emit('MUL', f'r{t.n}', f'r{two.n}', f'r{d.n}'); self.emit('MUL', f'r{t.n}', f'r{t.n}', f'r{N.n}')
        r=self.alloc(I.ty); self.emit('SUB', f'r{r.n}', f'r{I.n}', f'r{t.n}'); self.freer(d); self.freer(t); return r
    def bi_refract(self,I,N,eta):
        # ES2 spec: k = 1 - eta^2 (1 - dot(N,I)^2);  k<0 -> 0 vector;  else eta*I - (eta*dot(N,I)+sqrt(k))*N
        d=self.bi_dot(N,I)                                                     # scalar dot(N,I)
        dd=self.alloc('float'); self.emit('MUL', f'r{dd.n}', f'r{d.n}', f'r{d.n}')   # d*d
        one=self.kf(1.0); omdd=self.alloc('float'); self.emit('SUB', f'r{omdd.n}', f'r{one.n}', f'r{dd.n}')  # 1-d*d
        e2=self.alloc('float'); self.emit('MUL', f'r{e2.n}', f'r{eta.n}', f'r{eta.n}')                       # eta^2
        t=self.alloc('float'); self.emit('MUL', f'r{t.n}', f'r{e2.n}', f'r{omdd.n}')
        k=self.alloc('float'); self.emit('SUB', f'r{k.n}', f'r{one.n}', f'r{t.n}')                            # k
        kpos=self.alloc('float'); self.emit('SET', 'SGE', f'r{kpos.n}', f'r{k.n}', f'r{self.zeroreg().n}')    # (k>=0)?1:0
        kc=self.alloc('float'); self.emit('MAX', f'r{kc.n}', f'r{k.n}', f'r{self.zeroreg().n}')               # max(k,0)
        # sqrt is x*rsqrt(x), which is 0*inf (garbage) at x=0. When k<0 (kpos=0) the result is zeroed anyway,
        # so push the sqrt argument to >=1 there to keep it finite (avoids NaN*0 leaking through).
        neg=self.alloc('float'); self.emit('SUB', f'r{neg.n}', f'r{one.n}', f'r{kpos.n}')                     # 1-kpos = 1 iff k<0
        self.emit('ADD', f'r{kc.n}', f'r{kc.n}', f'r{neg.n}')
        sq=self.bi_sqrt(kc)
        coef=self.alloc('float'); self.emit('MUL', f'r{coef.n}', f'r{eta.n}', f'r{d.n}'); self.emit('ADD', f'r{coef.n}', f'r{coef.n}', f'r{sq.n}')  # eta*d+sqrt(k)
        ei=self.alloc(I.ty); self.emit('MUL', f'r{ei.n}', f'r{eta.n}', f'r{I.n}')          # eta*I
        cn=self.alloc(N.ty); self.emit('MUL', f'r{cn.n}', f'r{coef.n}', f'r{N.n}')          # coef*N
        r=self.alloc(I.ty); self.emit('SUB', f'r{r.n}', f'r{ei.n}', f'r{cn.n}')
        self.emit('MUL', f'r{r.n}', f'r{r.n}', f'r{kpos.n}')                                 # zero when k<0
        for z in (d,dd,omdd,e2,t,k,kpos,neg,kc,sq,coef,ei,cn): self.freer(z)
        return r
    def bi_matrixCompMult(self,a,b):
        n=ncomp2(a.ty); cols=[]
        for j in range(n):
            c=self.alloc(f'vec{n}'); self.emit('MUL', f'r{c.n}', f'r{a.cols[j].n}', f'r{b.cols[j].n}'); cols.append(c)
        return MatVal(cols, a.ty)
    def bi_radians(self,x): k=self.kf(math.pi/180.0); r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{x.n}', f'r{k.n}'); return r
    def bi_degrees(self,x): k=self.kf(180.0/math.pi); r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{x.n}', f'r{k.n}'); return r
    def bi_exp2(self,x): return self._un('EXP2', x)
    def bi_log2(self,x): return self._un('LOG2', x)
    def bi_exp(self,x): k=self.kf(1.0/math.log(2)); t=self.alloc(x.ty); self.emit('MUL', f'r{t.n}', f'r{x.n}', f'r{k.n}'); r=self._un('EXP2',t); self.freer(t); return r
    def bi_log(self,x): l=self._un('LOG2',x); k=self.kf(math.log(2)); r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{l.n}', f'r{k.n}'); self.freer(l); return r
    def bi_pow(self,x,y): l=self._un('LOG2',x); t=self.alloc(x.ty); self.emit('MUL', f'r{t.n}', f'r{y.n}', f'r{l.n}'); r=self._un('EXP2',t); self.freer(l); self.freer(t); return r
    # HW sin/cos: the float_sfu quarter-wave LUT (ISA op SIN=40/COS=41) takes the angle in RADIANS and does
    # the /2pi range-reduction + quadrant fold internally, so a whole trig call is ONE SFU op (vs ~18 for the
    # software Taylor). Elementwise across the vec lanes, exactly like RCP/RSQRT/EXP2. hw_sincos=False (a core without the
    # unit, noops bit 1): _sincos_lowered on isa64, the Taylor _poly_sin on the 32-bit ISA.
    def bi_sin(self,x): return self._un('SIN', x) if getattr(self,'hw_sincos',True) else self._sincos_lowered(x, cos=False) if self.isa64 else self._poly_sin(x, cos=False)
    def bi_cos(self,x): return self._un('COS', x) if getattr(self,'hw_sincos',True) else self._sincos_lowered(x, cos=True) if self.isa64 else self._poly_sin(x, cos=True)
    # sin / cos without the SIN / COS unit (isa64, noops bit 1), 17 instructions. f = floor(x / 2pi + 1/2) whole turns (cos:
    # + 3/4) come off in three parts, r = x - f C1 - f C2 - f C3 (C1, C2 short, so f C1 and f C2 are exact for |f| < 2^12:
    # |x| < 25000), then the phase in quarter turns a = r * 2/pi (cos: + 1), in [-2, 2] up to a floor off by one at a turn's
    # edge; the clamp to [-3, 3] only matters for huge |x|, whose phase is noise in FP24. The triangle
    # s = max(min(a, 2 - a), -2 - a) folds [-3, 3] to [-1, 1] (exact) with sin(pi a / 2) = sin(pi s / 2); then
    # P(s) = s (c1 + s2 (c3 + s2 (c5 + s2 c7))). The coefficients are FP24 values chosen so that, evaluated as here, P(1) = 1
    # exactly, P(s) <= 1 for every FP24 s, and |P(s) - sin(pi s / 2)| <= 2.2e-5 on [0, 1] (relative 7e-6 for small s). So
    # sin(0) = 0 and cos(0) = 1 exactly, |result| <= 1, and up to |x| = 1e4 the error is <= 4e-5 (sin) / 7e-5 (cos).
    _K2PI = 83443 / 2**17                          # 2/pi in FP24 (1/2pi is the same / 4)
    _SINW = (25 / 2**2, 17 / 2**9, -76527 / 2**32)                                # C1 + C2 + C3 = 2pi
    _SINC = (102943 / 2**16, -84661 / 2**17, 20837 / 2**18, -18297 / 2**22)     # c1, c3, c5, c7
    def _sincos_lowered(self, x, cos):
        x=self._asf(x); ty=x.ty
        u=self.alloc(ty); self.emit('MAD', f'r{u.n}', f'r{x.n}', f'r{self.kf(self._K2PI / 4).n}', f'r{self.kf(0.75 if cos else 0.5).n}')
        f=self._un('FLR', u)
        r=self.alloc(ty); w1,w2,w3=(self.kf(v) for v in self._SINW)
        self.emit('MAD', f'r{r.n}', f'r{f.n}', f'-r{w1.n}', f'r{x.n}')
        self.emit('MAD', f'r{r.n}', f'r{f.n}', f'-r{w2.n}', f'r{r.n}'); self.emit('MAD', f'r{r.n}', f'r{f.n}', f'-r{w3.n}', f'r{r.n}')
        k=self.kf(self._K2PI); a=self.alloc(ty)
        if cos: self.emit('MAD', f'r{a.n}', f'r{r.n}', f'r{k.n}', f'r{self.kf(1.0).n}')
        else: self.emit('MUL', f'r{a.n}', f'r{r.n}', f'r{k.n}')
        three=self.kf(3.0); two=self.kf(2.0)
        self.emit('MIN', f'r{a.n}', f'r{a.n}', f'r{three.n}'); self.emit('MAX', f'r{a.n}', f'r{a.n}', f'-r{three.n}')
        b=self.alloc(ty); self.emit('SUB', f'r{b.n}', f'r{two.n}', f'r{a.n}')
        c=self.alloc(ty); self.emit('SUB', f'r{c.n}', f'-r{two.n}', f'r{a.n}')
        s=self.alloc(ty); self.emit('MIN', f'r{s.n}', f'r{a.n}', f'r{b.n}'); self.emit('MAX', f'r{s.n}', f'r{s.n}', f'r{c.n}')
        s2=self.alloc(ty); self.emit('MUL', f'r{s2.n}', f'r{s.n}', f'r{s.n}')
        c1,c3,c5,c7=(self.kf(v) for v in self._SINC)
        p=self.alloc(ty); self.emit('MAD', f'r{p.n}', f'r{s2.n}', f'r{c7.n}', f'r{c5.n}')
        self.emit('MAD', f'r{p.n}', f'r{p.n}', f'r{s2.n}', f'r{c3.n}'); self.emit('MAD', f'r{p.n}', f'r{p.n}', f'r{s2.n}', f'r{c1.n}')
        y=self.alloc(ty); self.emit('MUL', f'r{y.n}', f'r{p.n}', f'r{s.n}')
        for z in (u,f,r,a,b,c,s,s2,p): self.freer(z)
        return y
    # a / b truncated toward zero, 24-bit two's complement ints (type ty: int, or ivecN in int representation), without the
    # divider (noops bit 0) and with only the integer ops the cores have (no IOP), 27 instructions, exact over the whole range:
    # |a| and |b| by compare and select; an estimate of |a| / |b| through RCP(|b|) and its exact remainder; a second estimate
    # from the remainder, floored after taking 1/2 off, which leaves the remainder in (0, 2 |b|); one correction (q + 1 when
    # r - |b| >= 0, a signed compare that holds for |b| = 2^23 too); the sign by an exact select (as IDIV: -2^23 / -1 wraps).
    # b = 0 gives some finite value (IDIV's -1 is not kept: GLSL leaves x / 0 undefined).
    def _idiv_lowered(self, a, b, ty):
        fty='float' if ty=='int' else 'vec'+ty[-1]
        z=self.zeroreg()                                                                # 0.0, whose bits are the int 0
        na=self.alloc(fty); self.emit('ISLT', f'r{na.n}', f'r{a.n}', f'r{z.n}')
        nb=self.alloc(fty); self.emit('ISLT', f'r{nb.n}', f'r{b.n}', f'r{z.n}')
        A=self.alloc(ty); self.emit('ISUB', f'r{A.n}', f'r{z.n}', f'r{a.n}'); self.emit('SEL', f'r{A.n}', f'r{na.n}', f'r{A.n}', f'r{a.n}')
        B=self.alloc(ty); self.emit('ISUB', f'r{B.n}', f'r{z.n}', f'r{b.n}'); self.emit('SEL', f'r{B.n}', f'r{nb.n}', f'r{B.n}', f'r{b.n}')
        fa=self.alloc(fty); self.emit('UTOF', f'r{fa.n}', f'r{A.n}')                    # |a| (-2^23 reads 2^23 unsigned)
        fb=self.alloc(fty); self.emit('UTOF', f'r{fb.n}', f'r{B.n}')
        rb=self.alloc(fty); self.emit('RCP', f'r{rb.n}', f'r{fb.n}')
        t=self.alloc(fty); self.emit('MUL', f'r{t.n}', f'r{fa.n}', f'r{rb.n}')
        q=self.alloc(ty); self.emit('FTOU', f'r{q.n}', f'r{t.n}')                       # floor(|a| / |b|) within a few hundred
        p=self.alloc(ty); self.emit('IMUL', f'r{p.n}', f'r{q.n}', f'r{B.n}')
        r=self.alloc(ty); self.emit('ISUB', f'r{r.n}', f'r{A.n}', f'r{p.n}')            # its remainder, exact
        fq=self.alloc(fty); self.emit('ITOF', f'r{fq.n}', f'r{r.n}')
        half=self.kf(0.5)
        t2=self.alloc(fty); self.emit('MAD', f'r{t2.n}', f'r{fq.n}', f'r{rb.n}', f'-r{half.n}'); self.emit('FLR', f'r{t2.n}', f'r{t2.n}')
        d=self.alloc(ty); self.emit('FTOI', f'r{d.n}', f'r{t2.n}')
        self.emit('IADD', f'r{q.n}', f'r{q.n}', f'r{d.n}')
        self.emit('IMUL', f'r{p.n}', f'r{d.n}', f'r{B.n}'); self.emit('ISUB', f'r{r.n}', f'r{r.n}', f'r{p.n}')    # remainder in (0, 2 |b|)
        self.emit('ISUB', f'r{r.n}', f'r{r.n}', f'r{B.n}')
        c=self.alloc(fty); self.emit('ISGE', f'r{c.n}', f'r{r.n}', f'r{z.n}')           # 1.0 if the remainder is |b| or more
        self.emit('FTOI', f'r{d.n}', f'r{c.n}'); self.emit('IADD', f'r{q.n}', f'r{q.n}', f'r{d.n}')
        s=self.alloc(fty); self.emit('SET', 'SNE', f'r{s.n}', f'r{na.n}', f'r{nb.n}')    # 1.0 if the signs differ
        self.emit('ISUB', f'r{p.n}', f'r{z.n}', f'r{q.n}'); self.emit('SEL', f'r{q.n}', f'r{s.n}', f'r{p.n}', f'r{q.n}')
        for x in (na,nb,A,B,fa,fb,rb,t,p,r,fq,t2,d,c,s): self.freer(x)
        return q
    def _poly_sin(self, x, cos):
        # sin/cos via range reduction to [-pi,pi] + a 7th-order Taylor. Transcendental precision is
        # implementation-defined in ES2, so this is an approximation (good to ~1e-3 for typical angles).
        TWO_PI=6.2831853072; INV_2PI=1.0/TWO_PI
        xr=x
        if cos:
            hp=self.kf(1.5707963268); xr=self.alloc(x.ty); self.emit('ADD', f'r{xr.n}', f'r{x.n}', f'r{hp.n}')
        t=self.alloc(x.ty); k=self.kf(INV_2PI); self.emit('MUL', f'r{t.n}', f'r{xr.n}', f'r{k.n}')  # turns
        half=self.kf(0.5); self.emit('ADD', f'r{t.n}', f'r{t.n}', f'r{half.n}')                     # turns + .5
        # round-to-nearest whole turns = FLOOR(turns+.5). MUST be floor (round toward -inf), NOT FTOI
        # (truncate toward zero): for negative arguments trunc != floor, which leaves the reduced angle
        # OUTSIDE [-pi,pi] where the Taylor series diverges (-> huge values, e.g. the atan branch-cut seam).
        ki=self.bi_floor(t)
        self.emit('SUB', f'r{t.n}', f'r{t.n}', f'r{ki.n}')          # frac in [0,1)
        self.emit('SUB', f'r{t.n}', f'r{t.n}', f'r{half.n}')        # -> [-.5,.5)
        tp=self.kf(TWO_PI); s=self.alloc(x.ty); self.emit('MUL', f'r{s.n}', f'r{t.n}', f'r{tp.n}')  # s in [-pi,pi]
        s2=self.alloc(x.ty); self.emit('MUL', f'r{s2.n}', f'r{s.n}', f'r{s.n}')
        # 9th-order Horner: p = ((((c4*s2+c3)*s2+c2)*s2+c1)*s2+1) ; sin = s*p  (err ~7e-3 near +/-pi)
        c1=self.kf(-1.0/6); c2=self.kf(1.0/120); c3=self.kf(-1.0/5040); c4=self.kf(1.0/362880); one=self.kf(1.0)
        p=self.alloc(x.ty); self.emit('MUL', f'r{p.n}', f'r{c4.n}', f'r{s2.n}'); self.emit('ADD', f'r{p.n}', f'r{p.n}', f'r{c3.n}')
        self.emit('MUL', f'r{p.n}', f'r{p.n}', f'r{s2.n}'); self.emit('ADD', f'r{p.n}', f'r{p.n}', f'r{c2.n}')
        self.emit('MUL', f'r{p.n}', f'r{p.n}', f'r{s2.n}'); self.emit('ADD', f'r{p.n}', f'r{p.n}', f'r{c1.n}')
        self.emit('MUL', f'r{p.n}', f'r{p.n}', f'r{s2.n}'); self.emit('ADD', f'r{p.n}', f'r{p.n}', f'r{one.n}')
        r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{s.n}', f'r{p.n}')
        for z in (t,ki,s,s2,p): self.freer(z)
        if cos: self.freer(xr)
        return r
    def _atan1(self, x):                              # atan(x), branchless, minimax over [0,1] + 1/x reduction
        ax=self.bi_abs(x)
        axe=self.alloc(x.ty); self.emit('ADD', f'r{axe.n}', f'r{ax.n}', f'r{self.kf(1e-6).n}')  # avoid 1/0 at x=0
        inv=self.alloc(x.ty); self.emit('RCP', f'r{inv.n}', f'r{axe.n}')
        z=self.bi_min(ax, inv)                        # reduced to [0,1]
        z2=self.alloc(x.ty); self.emit('MUL', f'r{z2.n}', f'r{z.n}', f'r{z.n}')
        p=self.kf(0.0208351)
        for c in (-0.0851330, 0.1801410, -0.3302995, 0.9998660):
            t=self.alloc(x.ty); self.emit('MUL', f'r{t.n}', f'r{p.n}', f'r{z2.n}')
            p=self.alloc(x.ty); self.emit('ADD', f'r{p.n}', f'r{t.n}', f'r{self.kf(c).n}')
        at=self.alloc(x.ty); self.emit('MUL', f'r{at.n}', f'r{z.n}', f'r{p.n}')   # atan(z)
        hp=self.kf(math.pi/2); comp=self.alloc(x.ty); self.emit('SUB', f'r{comp.n}', f'r{hp.n}', f'r{at.n}')
        full=self.builtin_mix(at, comp, self.bi_step(self.kf(1.0), ax))           # |x|>1 -> pi/2-atan(1/x)
        sg=self.bi_sign(x); r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{full.n}', f'r{sg.n}'); return r
    def bi_atan(self, *a):
        if self.isa64: return self._atan64(*a)
        if len(a)==1: return self._atan1(a[0])
        y,x=a; inv=self.alloc(x.ty); self.emit('RCP', f'r{inv.n}', f'r{x.n}')
        q=self.alloc(y.ty); self.emit('MUL', f'r{q.n}', f'r{y.n}', f'r{inv.n}')   # y/x
        base=self._atan1(q)
        # the quadrant: + pi (y >= 0) / - pi (y < 0) when x < 0 STRICTLY (x == 0: y / x = +-inf already gives +-pi/2;
        # an `x <= 0` test would add pi there and return the wrong quadrant on the x = 0 line)
        xneg=self._setc('SLT', x, self.zeroreg())                                # 1 if x < 0
        ypos=self._setc('SGE', y, self.zeroreg())                                # 1 if y >= 0 (atan(0, x < 0) = +pi)
        sg=self.alloc(y.ty); self.emit('MUL', f'r{sg.n}', f'r{self.kf(2.0).n}', f'r{ypos.n}')
        self.emit('SUB', f'r{sg.n}', f'r{sg.n}', f'r{self.kf(1.0).n}')          # +1 / -1
        corr=self.alloc(y.ty); self.emit('MUL', f'r{corr.n}', f'r{sg.n}', f'r{self.kf(math.pi).n}')
        self.emit('MUL', f'r{corr.n}', f'r{corr.n}', f'r{xneg.n}')                # +/-pi when x<0
        r=self.alloc(y.ty); self.emit('ADD', f'r{r.n}', f'r{base.n}', f'r{corr.n}'); return r
    def _atan64(self, *a):
        """atan(y, x) / atan(x) on the 64-bit ISA: no 1/x reduction (it gives the wrong quadrant on the x = 0
        line), the octant form instead — r = min(|x|, |y|) / max(|x|, |y|) in [0, 1], atan(r) by the degree-9
        minimax polynomial as MADs, then the three exact SELs: |y| > |x| -> pi/2 - at, x < 0 -> pi - at, y < 0 -> -at.
        21 instructions; atan(0, 0) is NaN (undefined in GLSL ES)."""
        if len(a)==1:
            y, x = a[0], self.kf(1.0)
            if y.ty!='float':                            # a 1 of y's shape: what is worked out from x takes its type (min, max, SET)
                one=self.alloc(y.ty); self.emit('MOV', f'r{one.n}', f'r{x.n}'); x=one
        else: y, x = a
        ay=self.bi_abs(y); ax=self.bi_abs(x)
        mn=self.bi_min(ax, ay); mx=self.bi_max(ax, ay)
        inv=self.alloc(y.ty); self.emit('RCP', f'r{inv.n}', f'r{mx.n}')
        r=self.alloc(y.ty); self.emit('MUL', f'r{r.n}', f'r{mn.n}', f'r{inv.n}')
        s=self.alloc(y.ty); self.emit('MUL', f'r{s.n}', f'r{r.n}', f'r{r.n}')
        p=self.kf(0.0208351)
        for c in (-0.0851330, 0.1801410, -0.3302995, 0.9998660):
            q=self.alloc(y.ty); self.emit('MAD', f'r{q.n}', f'r{p.n}', f'r{s.n}', f'r{self.kf(c).n}'); p=q
        at=self.alloc(y.ty); self.emit('MUL', f'r{at.n}', f'r{p.n}', f'r{r.n}')     # atan(r), r in [0, 1]
        z=self.zeroreg()
        c1=self._setc('SLT', ax, ay); t1=self.alloc(y.ty); self.emit('SUB', f'r{t1.n}', f'r{self.kf(math.pi/2).n}', f'r{at.n}')
        a1=self.alloc(y.ty); self.emit('SEL', f'r{a1.n}', f'r{c1.n}', f'r{t1.n}', f'r{at.n}')
        if len(a)==1: a2=a1
        else:
            c2=self._setc('SLT', x, z); t2=self.alloc(y.ty); self.emit('SUB', f'r{t2.n}', f'r{self.kf(math.pi).n}', f'r{a1.n}')
            a2=self.alloc(y.ty); self.emit('SEL', f'r{a2.n}', f'r{c2.n}', f'r{t2.n}', f'r{a1.n}')
        c3=self._setc('SLT', y, z); t3=self.alloc(y.ty); self.emit('SUB', f'r{t3.n}', f'r{z.n}', f'r{a2.n}')
        res=self.alloc(y.ty); self.emit('SEL', f'r{res.n}', f'r{c3.n}', f'r{t3.n}', f'r{a2.n}')
        return res
    def bi_asin(self,x):
        x2=self.alloc(x.ty); self.emit('MUL', f'r{x2.n}', f'r{x.n}', f'r{x.n}')
        d=self.alloc(x.ty); self.emit('SUB', f'r{d.n}', f'r{self.kf(1.0).n}', f'r{x2.n}')
        self.emit('ADD', f'r{d.n}', f'r{d.n}', f'r{self.kf(1e-6).n}')             # avoid /0 at |x|=1
        rs=self.alloc(x.ty); self.emit('RSQRT', f'r{rs.n}', f'r{d.n}')
        t=self.alloc(x.ty); self.emit('MUL', f'r{t.n}', f'r{x.n}', f'r{rs.n}'); return self._atan1(t)  # asin=atan(x/sqrt(1-x^2))
    def bi_acos(self,x):
        a=self.bi_asin(x); r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{self.kf(math.pi/2).n}', f'r{a.n}'); return r
    def bi_ceil(self,x):
        if self.isa64: return self._un('CEIL', x)
        nx=self.alloc(x.ty); z=self.zeroreg(); self.emit('SUB', f'r{nx.n}', f'r{z.n}', f'r{x.n}')
        f=self.bi_floor(nx); r=self.alloc(x.ty); self.emit('SUB', f'r{r.n}', f'r{z.n}', f'r{f.n}'); self.freer(nx); self.freer(f); return r
    def bi_faceforward(self,N,I,Nref):
        # ES2: N if dot(Nref, I) < 0, else -N (strictly less: dot == 0 gives -N). s = (d < 0) ? 1 : 0, m = 2s - 1.
        d=self.bi_dot(Nref,I); z=self.zeroreg(); s=self.alloc('float'); self.emit('SET', 'SLT', f'r{s.n}', f'r{d.n}', f'r{z.n}')
        two=self.kf(2.0); m=self.alloc('float'); self.emit('MUL', f'r{m.n}', f'r{two.n}', f'r{s.n}')
        one=self.kf(1.0); self.emit('SUB', f'r{m.n}', f'r{m.n}', f'r{one.n}')   # +1 if d<0 else -1
        r=self.alloc(N.ty); self.emit('MUL', f'r{r.n}', f'r{N.n}', f'r{m.n}'); return r
    def _tex(self, samp, coord, mode=None, w=None, cube=False):
        r=self._tex1(samp, coord, mode, w, cube)
        unit = samp.unit if isinstance(samp, Sampler) else 0
        if self.stage=='vs' or not self.depth_units>>unit&1: return r
        # the TMU reads the D24S8 word as RGBA bytes: R the stencil, then the depth, high byte first. All ones must come to 1 whatever the additions round
        g=self.extract(r,1); b=self.extract(r,2); a=self.extract(r,3)
        z=self.alloc('float'); self.emit('MUL', f'r{z.n}', f'r{g.n}', f'r{self.kf(0.99609375).n}')
        t=self.alloc('float'); self.emit('MUL', f'r{t.n}', f'r{b.n}', f'r{self.kf(0.003891).n}'); self.emit('ADD', f'r{z.n}', f'r{z.n}', f'r{t.n}')
        self.emit('MUL', f'r{t.n}', f'r{a.n}', f'r{self.kf(0.000016).n}'); self.emit('ADD', f'r{z.n}', f'r{z.n}', f'r{t.n}')
        o=self.bi_min(z, self.kf(1.0))
        d=self.alloc('vec4'); self.emit('MOV', f'r{d.n}', f'r{o.n}'); self.emit('MOV.w', f'r{d.n}', f'r{self.kf(1.0).n}')
        return d
    def _tex1(self, samp, coord, mode=None, w=None, cube=False):
        # isa64: TEX rd, coord, #unit[, BIAS | LOD] with the bias / lod in coord.w; cube maps take the direction
        # as .xyz (the sampler's kind is in the TMU's descriptor). 32-bit ISA: plain TEX on coord.xy (bias / lod
        # cannot be passed). cube: the textureCube built-ins (the coordinate set below).
        unit = samp.unit if isinstance(samp, Sampler) else 0
        r=self.alloc('vec4')
        if self.isa64 and self.stage=='vs':              # MAX_VERTEX_TEXTURE_IMAGE_UNITS is 0: legal to compile, the link refuses it
            self.vs_tex=True; self.emit('MOV', f'r{r.n}', f'r{self.zeroreg().n}'); return r
        if self.isa64 and mode is not None:
            # the coordinate set (isa64 TEX option): the core fetches only these components into the TMU
            # request (one micro-op each) — a 2D sampler needs s, t (+ the bias / lod in w), a cube map x, y, z (+ w)
            t=self.alloc('vec4'); self.emit('MOV', f'r{t.n}', f'r{coord.n}'); self.emit('MOV.w', f'r{t.n}', f'r{w.n}')
            if cube: self.emit('TEX', f'r{r.n}', f'r{t.n}', f'#{unit}', mode)
            else: self.emit('TEX', f'r{r.n}', f'r{t.n}', f'#{unit}', mode, '.xyw')
            self.freer(t); return r
        if self.isa64: self.emit('TEX', f'r{r.n}', f'r{coord.n}', f'#{unit}', '.xyz' if cube else '.xy'); return r
        self.emit('TEX', f'r{r.n}', f'r{coord.n}', f'#{unit}'); return r
    def bi_textureCube(self,samp,dir,bias=None):
        return self._tex(samp, dir, 'BIAS' if bias is not None else None, bias, cube=True)
    def bi_texture2DProj(self,samp,uv,bias=None):
        # ES2: projective texture. Sample coords = uv.xy / uv.q, where q is the LAST component
        # (vec3 -> .z, vec4 -> .w). Do the divide in-shader (the TMU takes .xy of the passed reg).
        q=self.extract(uv, ncomp(uv.ty)-1)                       # last component
        inv=self.alloc('float'); self.emit('RCP', f'r{inv.n}', f'r{q.n}')
        st=self.alloc('vec2'); self.emit('MUL', f'r{st.n}', f'r{uv.n}', f'r{inv.n}')   # (uv.xy/q) in lanes 0,1
        res=self.bi_texture2D(samp, st, bias); self.freer(q); self.freer(inv); self.freer(st); return res
    def bi_texture2DLod(self,samp,uv,lod): return self._tex(samp, uv, 'LOD', lod)
    def bi_textureCubeLod(self,samp,dir,lod): return self._tex(samp, dir, 'LOD', lod, cube=True)
    def bi_texture3D(self,samp,c,bias=None): return self.bi_textureCube(samp,c,bias)     # s, t, r like a direction: the image decides the kind
    def bi_texture3DLod(self,samp,c,lod): return self.bi_textureCubeLod(samp,c,lod)
    def bi_texture3DProj(self,samp,c,bias=None): return self.bi_textureCube(samp,self._proj(c,'vec3'),bias)
    def bi_texture3DProjLod(self,samp,c,lod): return self.bi_textureCubeLod(samp,self._proj(c,'vec3'),lod)
    def _proj(self, uv, ty='vec2'):
        """uv.xy (ty vec3: .xyz) / q, q the last component"""
        q=self.extract(uv, ncomp(uv.ty)-1)
        inv=self.alloc('float'); self.emit('RCP', f'r{inv.n}', f'r{q.n}')
        st=self.alloc(ty); self.emit('MUL', f'r{st.n}', f'r{uv.n}', f'r{inv.n}'); return st
    def bi_texture2DProjLod(self,samp,uv,lod): return self._tex(samp, self._proj(uv), 'LOD', lod)
    def bi_texture2DLodEXT(self,samp,uv,lod): return self.bi_texture2DLod(samp,uv,lod)
    def bi_texture2DProjLodEXT(self,samp,uv,lod): return self.bi_texture2DProjLod(samp,uv,lod)
    def bi_textureCubeLodEXT(self,samp,dir,lod): return self.bi_textureCubeLod(samp,dir,lod)
    def _texsize(self, samp):
        """(width, height) of the texture on this unit: a uniform the library fills, as the TMU takes a level, not gradients"""
        name=f'_g2_ts{samp.unit}'
        if name not in self.uniforms: self.uniforms[name]=(self.next_uniform,'vec2'); self.next_uniform+=1
        r=self.alloc('vec2'); self.emit('LDP', f'r{r.n}', self.uniforms[name][0]); return r
    def _grad_lod(self, dx, dy, sz):
        """lambda = log2(rho), rho the longer of the two gradients in texels (ES 3.0 §3.8.9)"""
        x=self.alloc(dx.ty); self.emit('MUL', f'r{x.n}', f'r{dx.n}', f'r{sz.n}')
        y=self.alloc(dy.ty); self.emit('MUL', f'r{y.n}', f'r{dy.n}', f'r{sz.n}')
        m=self.bi_max(self.bi_dot(x,x), self.bi_dot(y,y)); l=self._un('LOG2', m)
        r=self.alloc('float'); self.emit('MUL', f'r{r.n}', f'r{l.n}', f'r{self.kf(0.5).n}'); return r
    def bi_texture2DGradEXT(self,samp,uv,dx,dy):
        return self._tex(samp, uv, 'LOD', self._grad_lod(dx, dy, self._texsize(samp)))
    def bi_texture2DProjGradEXT(self,samp,uv,dx,dy):
        return self._tex(samp, self._proj(uv), 'LOD', self._grad_lod(dx, dy, self._texsize(samp)))
    def bi_textureCubeGradEXT(self,samp,dir,dx,dy):
        # a face's s = (sc / |ma| + 1) / 2: across a pixel the coordinate on the face moves by the gradient over 2 |ma|
        ax=self.bi_abs(dir); ma=self.bi_max(self.bi_max(self.extract(ax,0), self.extract(ax,1)), self.extract(ax,2))
        tw=self.alloc('float'); self.emit('ADD', f'r{tw.n}', f'r{ma.n}', f'r{ma.n}')
        inv=self.alloc('float'); self.emit('RCP', f'r{inv.n}', f'r{tw.n}')
        sz=self._texsize(samp); k=self.alloc('float'); self.emit('MUL', f'r{k.n}', f'r{sz.n}.x', f'r{inv.n}')
        return self._tex(samp, dir, 'LOD', self._grad_lod(dx, dy, k), cube=True)
    def bi_tan(self,x): s=self.bi_sin(x); c=self.bi_cos(x); inv=self.alloc(x.ty); self.emit('RCP', f'r{inv.n}', f'r{c.n}'); r=self.alloc(x.ty); self.emit('MUL', f'r{r.n}', f'r{s.n}', f'r{inv.n}'); return r
    def bi_texture2D(self,samp,uv,bias=None):      # optional LOD bias (3rd arg): TEX BIAS mode (isa64)
        return self._tex(samp, uv, 'BIAS' if bias is not None else None, bias)
    def bi_dFdx(self,x): return self._un('DDX', x)
    def bi_dFdy(self,x): return self._un('DDY', x)
    def bi_fwidth(self,x):
        dx=self.bi_dFdx(x); dy=self.bi_dFdy(x); adx=self.bi_abs(dx); ady=self.bi_abs(dy)
        r=self.alloc(x.ty); self.emit('ADD', f'r{r.n}', f'r{adx.n}', f'r{ady.n}')
        for z in (dx,dy,adx,ady): self.freer(z); return r

    # ---- assignment / statements ----
    def do_assign(self, e):
        _,op,lhs,rhs=e
        h=self._const_home(lhs[1]) if lhs[0]=='var' else None
        if h is not None:                                       # updating a compile-time-tracked scalar
            cv=self.fold_const(rhs if op=='=' else ('bin', op[0], lhs, rhs))
            if cv is not None:
                h[lhs[1]]=IntLit(int(cv)) if isinstance(h[lhs[1]],IntLit) else cv
                return self.kf(float(h[lhs[1]]))
            self._promote(lhs[1])                               # became runtime
        # compound assign (i+=k etc.) reuses binop so the int fast-path (IADD/ISUB) and float paths both apply.
        rv = self.eval(rhs) if op=='=' else self.binop(op[0], lhs, rhs)
        self.store(lhs, rv)
        return rv
    def store(self, lhs, rv):
        # lhs: var / swizzle(var) / special output
        lhs=self._fd(lhs)
        if lhs[0]=='var':
            name=lhs[1]
            if name not in self.shadow and self._store_special(name, rv): return
            if name in self.varyings and self.stage=='vs' and name not in self.shadow:
                self.emit('STV', self.varyings[name][0], f'r{self._asf(rv).n}'); return
            if isinstance(rv,(MatVal,StructVal,ArrayVal)) and (self.isa64 or name in self.shadow):
                self._copy_val(self.eval_lvalue(lhs), rv); return
            tt=self._local_type(name)                        # convert rv to the slot's representation
            if   tt=='int': rv=self._asi(rv)
            elif tt is not None: rv=self._asf(rv)
            slot=self.get_local_reg(name, rv.ty)
            self.emit('MOV', f'r{slot.n}', f'r{rv.n}'); return
        if lhs[0]=='swizzle':
            base=self._fd(lhs[1]); mask=lhs[2]
            while base[0]=='swizzle' and not isinstance(self.eval_lvalue(base[1]),StructVal):       # v.wz.y = rv
                mask=''.join(base[2][_cidx(c)] for c in mask); base=self._fd(base[1])
            target=self.eval_lvalue(base)
            if isinstance(target, StructVal):                # l.member = rv
                if mask not in target.members: raise CompileError(f"struct {target.sname} has no member {mask!r}")
                cur=target.members[mask]
                if isinstance(cur, Reg):
                    rv=self._asi(rv) if cur.ty=='int' else self._asf(rv)
                    self.emit('MOV', f'r{cur.n}', f'r{rv.n}')
                elif self.isa64:
                    if cur is None: cur=target.members[mask]=self._like(rv)
                    self._copy_val(cur, rv)
                else: target.members[mask]=rv                # aggregate member (mat/array/nested) re-bind
                return
            self.masked_write(target, mask, self._asf(rv)); return     # a vector holds its ints as floats: i.x += 1 is an IADD
        if lhs[0]=='index':                                  # a[i] = rv   (local array / matrix column / vector component)
            ci=self.fold_const(lhs[2])
            if lhs[1][0]=='swizzle' and not isinstance(self.eval_lvalue(lhs[1][1]),StructVal):       # v.zy[1] = rv
                if ci is None: raise CompileError("array index must be a constant expression")
                self.store(('swizzle', lhs[1][1], lhs[1][2][int(ci)]), rv); return
            base=self.eval_lvalue(lhs[1])
            if ci is None: self._dyn_write(base, lhs[2], self._asf(rv) if isinstance(rv,Reg) else rv); return
            if isinstance(base,Reg):
                self.masked_write(base, 'xyzw'[int(ci)], self._asf(rv)); return
            tgt=self.eval_lvalue(lhs)
            if isinstance(rv,(MatVal,StructVal)):
                if type(tgt) is not type(rv): raise CompileError("cannot assign a matrix to an array element here")
                self._copy_val(tgt, rv); return
            rv=self._asi(rv) if tgt.ty=='int' else self._asf(rv)
            self.emit('MOV', f'r{tgt.n}', f'r{rv.n}'); return
        raise CompileError("unsupported assignment target")
    def _store_special(self, name, rv):
        if name=='gl_Position': self.emit('STO', 0, f'r{self._asf(rv).n}'); return True
        if name=='gl_PointSize':
            # the gl_PointSize row of the 64-bit vertex core (STO 1; the point setup reads .x — a
            # float is broadcast to every lane). The 32-bit single-board core has no point size output (its STO
            # index is unused, so STO 1 would overwrite gl_Position): refuse, as gl_PointCoord does.
            if self.stage!='vs': raise CompileError("gl_PointSize is only available in the vertex shader")
            if not self.isa64: raise CompileError("gl_PointSize needs the 64-bit vertex core (compile with isa64)")
            self.emit('STO', 1, f'r{self._asf(rv).n}'); return True
        if name=='gl_FragColor': self.emit('STO', 0, f'r{self._asf(rv).n}'); return True
        if name in ('gl_FragDepth','gl_FragDepthEXT'): self.emit('STOZ', f'r{self._asf(rv).n}'); return True
        return False
    # A vertex shader may read a varying back or write part of one (v = a; v.y = 1.0 - v.y). STV writes a whole
    # varying and LDV reads attributes, so those, and arrays and matrices, live in a local until the end of main.
    # So do gl_Position, gl_FragColor and the like when read or written in part (STO too writes a whole register).
    def _vary_uses(self, n, used):
        if isinstance(n,list):
            for x in n: self._vary_uses(x, used)
        elif isinstance(n,tuple) and n:
            if n[0]=='assign' and n[1]=='=' and n[2][0]=='var': self._vary_uses(n[3], used)   # a whole write: no use
            elif n[0]=='var':
                if n[1] in self.varyings or (self.isa64 and n[1] in _SPECIAL[self.stage]): used.add(n[1])
                elif self.isa64 and n[1]=='gl_FragData': used.add('gl_FragColor')
            else:
                for x in n[1:]: self._vary_uses(x, used)
    def shadow_varyings(self, body):
        if self.stage!='vs' and not self.isa64: return
        used=set(); self._vary_uses(body, used)
        for fn in self.funcs.values(): self._vary_uses(fn[2], used)
        self.nozero=True                                 # outputs: not variables of the source's, left as they were
        for name,v in self.varyings.items():
            if self.stage=='vs' and (len(v)==3 or is_mat(v[1]) or name in used or self.vmap is not None):
                self.shadow.append(name)
                self.nozero=not (len(v)==3 or is_mat(v[1]))     # an array or a matrix starts at 0: a write at a run-time index reads every element
                self.run_stmt(('ldecl', False, v[1], name, None, ('num',float(v[2])) if len(v)==3 else None))
                self.nozero=True
        if self.isa64:
            for name,ty in _SPECIAL[self.stage].items():
                if name in used:
                    self.shadow.append(name)
                    self.run_stmt(('ldecl', False, ty, name, None, None))
        self.nozero=False
    def flush_varyings(self):
        rows={}                                          # Appendix A.7: the vectors of a row, put together here
        for name in self.shadow:
            val=self.locals[0][name][0]
            if self._store_special(name, val): continue
            slot=self.varyings[name][0]
            for el in (val.elems if isinstance(val,ArrayVal) else [val]):
                for r in (el.cols if isinstance(el,MatVal) else [el]):
                    if self.vmap is None: self.emit('STV', slot, f'r{self._asf(r).n}')
                    elif slot<len(self.vmap) and self.vmap[slot]>=0:
                        r=self._asf(r); rows.setdefault(self.vmap[slot]>>2,[]).append((self.vmap[slot]&3, r))
                    slot+=1
        for row in sorted(rows):                         # lane x first (the row can become its register), then zeros where nothing is
            v=self.alloc('vec4'); cover=0
            for comp,r in sorted(rows[row], key=lambda e: e[0]):
                n=ncomp(r.ty); self.masked_write(v, 'xyzw'[comp:comp+n], r); cover|=((1<<n)-1)<<comp
            if cover!=15: self.emit('MOV.'+''.join('xyzw'[c] for c in range(4) if not cover>>c&1), f'r{v.n}', f'r{self.zeroreg().n}')
            rows[row]=v
        for row in sorted(rows): self.emit('STV', row, f'r{rows[row].n}')
    def get_lvalue_reg(self, e):
        # like eval_lvalue, but first PROMOTE a compile-time-tracked scalar to a real register (needed when
        # it's passed as an out/inout argument, whose callee writes through it).
        if e[0]=='var' and self._const_home(e[1]) is not None: return self._promote(e[1])
        return self.eval_lvalue(e)
    def _const_home(self, name):
        """the table that tracks `name` as a compile-time scalar, unless a real local or a parameter hides it"""
        for sc in reversed(self.locals):
            if name in sc.const: return sc.const
            if name in sc: return None
        return self.const_env if name in self.const_env else None
    def _const_get(self, name):
        h=self._const_home(name)
        return h[name] if h is not None else None
    def _promote(self, name):
        """a compile-time scalar becomes a register, in the scope that declares it"""
        h=self._const_home(name); home=self.locals[-1]
        for sc in reversed(self.locals):
            if sc.const is h: home=sc; break
        val=h.pop(name)
        r=self.alloc('float'); r.tmp=False; home[name]=(r,'float')
        self.emit('MOV', f'r{r.n}', f'r{self.kf(float(val)).n}'); return r
    def _lv_struct(self, e):
        """the value of a local the lvalue e names, an aggregate member of a local struct made on the way (None: no such local)"""
        if e[0]=='var':
            for sc in reversed(self.locals):
                if e[1] in sc: return sc[e[1]][0]
            return None
        if e[0]=='swizzle':
            st=self._lv_struct(e[1])
            if not isinstance(st,StructVal) or e[2] not in st.members: return None
            if st.members[e[2]] is None: st.members[e[2]]=self._member_new(st.sname, e[2])
            return st.members[e[2]]
        if e[0]=='index':
            a=self._lv_struct(e[1]); ci=self.fold_const(e[2])
            if isinstance(a,ArrayVal) and a.kind=='local' and ci is not None and 0<=int(ci)<len(a.elems): return a.elems[int(ci)]
        return None
    def _member_new(self, sname, mname):
        """registers for an aggregate member of a local struct, at its first write: 0, as the struct's other members"""
        for mt,mn,ma in self.structs[sname]:
            if mn==mname:
                n=self._arrsize(ma)
                v=ArrayVal(mt, n, 'local', elems=[self._fresh(mt) for _ in range(n)]) if ma is not None else self._fresh(mt)
                self._zero(v); return v
        return None
    def eval_lvalue(self, e):
        e=self._fd(e)
        if e[0]=='var':
            name=e[1]
            for sc in reversed(self.locals):
                if name in sc: return sc[name][0]
        if e[0]=='index':                                    # a[const] -> the element's Reg (local arr / mat col)
            ci=self.fold_const(e[2])
            if ci is None: raise CompileError("array index must be a constant expression")
            self._lv_struct(e[1])                            # s.b[0] = x: s.b gets its registers at its first write
            arr=self.eval(e[1]); i=int(ci)
            if isinstance(arr,ArrayVal) and arr.kind!='local' and e[1][0]=='swizzle':
                # a struct copied from a uniform shares its read-only array until the first write, which would otherwise
                # cost a register per element in every shader that only reads it
                st=self.eval_lvalue(e[1][1])
                if isinstance(st,StructVal):
                    l=ArrayVal(arr.ety, arr.size, 'local', elems=[self._fresh(arr.ety) for _ in range(arr.size)])
                    self._copy_val(l, arr); st.members[e[1][2]]=l; arr=l
            if isinstance(arr,ArrayVal) and arr.kind=='local': return arr.elems[i]
            if isinstance(arr,MatVal): return arr.cols[i]
            raise CompileError("cannot take address of this index target")
        if e[0]=='swizzle':                                  # struct member as lvalue: l.member
            bv=self.eval_lvalue(e[1])
            if isinstance(bv, StructVal): return bv.members[e[2]]
        raise CompileError("lvalue must be a local variable")
    def get_local_reg(self, name, ty):
        for sc in reversed(self.locals):
            if name in sc: return sc[name][0]
        r=self.alloc(ty); r.tmp=False; self.locals[-1][name]=(r,ty); return r

    def scan_body(self, body):
        self._scan=[{}]; self.scan_cond_assigns(body)
    def _arms(self, e):
        """what a ?: side or the right of && and || writes is written under a lane mask too"""
        if isinstance(e,list):
            for x in e: self._arms(x)
        elif isinstance(e,tuple) and e:
            if e[0]=='ternary': self._cond_expr(e[2]); self._cond_expr(e[3])
            elif e[0]=='bin' and e[1] in ('&&','||'): self._cond_expr(e[3])
            for x in e[1:]: self._arms(x)
    def scan_cond_assigns(self, stmt, incond=False):
        """Mark the declarations written under RUNTIME control flow. Constant propagation is invalid across it (the write
        is conditional), so such scalars must be real masked registers. By declaration, not by name: _scan holds the
        declarations in scope, and two loops called i have nothing to do with each other."""
        if stmt is None or not isinstance(stmt,tuple): return
        k=stmt[0]
        if k=='ldecl': self._arms(stmt[4])
        elif k in ('expr','return'): self._arms(stmt[1])
        elif k in ('if','while','dowhile'): self._arms(stmt[1])
        elif k=='for': self._arms(stmt[2]); self._arms(stmt[3])
        if k=='block':
            self._scan.append({})
            for s in stmt[1]: self.scan_cond_assigns(s, incond)
            self._scan.pop()
        elif k=='multi':
            for s in stmt[1]: self.scan_cond_assigns(s, incond)
        elif k=='ldecl':
            if incond: self._cond_expr(stmt[4])
            self._scan[-1][stmt[3]]=stmt
        elif k=='if':
            if incond: self._cond_expr(stmt[1])
            self.scan_cond_assigns(stmt[2], True); self.scan_cond_assigns(stmt[3], True)
        elif k=='for':
            self._scan.append({})
            runtime=self._refs_runtime(stmt[2]) or self._refs_runtime(stmt[3])     # runtime bound or step -> real (non-unrolled) loop
            if not runtime and (self.stage=='fs' or self.isa64) and self._long_for(stmt):
                self.hw_for[id(stmt)]=self._for_trips(stmt[1], stmt[2], stmt[3]); runtime=True     # a long constant loop: the hardware loop too (_long_for)
            self.scan_cond_assigns(stmt[1], incond)
            if runtime:                                               # loop var must be a reg (not const-tracked)
                self._mark_decls(stmt[1])
                self._cond_expr(stmt[3])                              # post (i++) writes the counter
            # a const-bounded loop still UNROLLS (loop var stays compile-time), but break/continue make the
            # body conditional -> its assignments can't be const-propagated.
            self.scan_cond_assigns(stmt[4], incond or runtime or self._has_loopctl(stmt[4]))
            self._scan.pop()
        elif k=='while':
            rt=self._refs_runtime(stmt[1]) or self._has_loopctl(stmt[2])
            self.scan_cond_assigns(stmt[2], incond or rt)
        elif k=='dowhile':
            self.scan_cond_assigns(stmt[2], True)                     # do-while is always a runtime loop
        elif incond: self._cond_expr(stmt)
    # A constant-bounded for loop is unrolled: its index is a literal in every pass, which a uniform array's or a sampler
    # array's index needs. Unrolled, a loop with break / continue runs every pass under lane masks with the index reloaded
    # as a constant per pass, so 100 marching steps would be 8 000 instructions and 100 constants — more than the fragment
    # core's 64 parameter vectors (a ray marcher's `for (int i = 0; i < 100; i++) { .. break; }` would not
    # compile). So a LONG constant loop is a hardware loop too (LOOP / ENDL with a per-lane BREAK, the index a register):
    # more than HW_FOR_CTL passes with a break or continue, more than HW_FOR_PLAIN without, unless the body needs the index
    # as a constant. Decided here, in the pre-pass, from the loop's text alone (the pre-pass has no values yet), and kept by
    # statement in hw_for: the pre-pass's marks and the code generator must agree. The count is an estimate that decides
    # only which of two correct forms is generated.
    HW_FOR_CTL, HW_FOR_PLAIN = 16, 64
    def _lit(self, e):
        """a literal, a signed one, or a global scalar constant that no local hides; else None"""
        if not isinstance(e,tuple) or not e: return None
        if e[0]=='num': return float(e[1])
        if e[0]=='unary' and e[1] in ('-','+'):
            v=self._lit(e[2])
            return None if v is None else (-v if e[1]=='-' else v)
        if e[0]=='var' and e[1] in self.const_env and not any(e[1] in sc for sc in self._scan): return float(self.const_env[e[1]])
        return None
    def _lit_now(self, e):
        """_lit for the code generator: the value in the scopes as they are (a parameter hides a global constant here, which
        the pre-pass cannot see)"""
        v=self.fold_const(e)
        return None if v is None else float(v)
    def _for_trips(self, init, cond, post, now=False):
        """the pass count of `for (int|float i = a; i < b; i++)` and its kin (<= > >= !=, --, += c, -= c), None if the loop
        is not of that form or runs beyond 4096 passes. now: with the code generator's values (_lit_now)"""
        lit=self._lit_now if now else self._lit
        if not (isinstance(init,tuple) and init[0]=='ldecl' and init[2] in ('int','float') and init[4] is not None and init[5] is None): return None
        name=init[3]; v=lit(init[4])
        if v is None or post is None: return None
        if not (cond[0]=='bin' and cond[1] in ('<','<=','>','>=','!=') and cond[2]==('var',name)): return None
        lim=lit(cond[3])
        if post[0]=='postop' and post[1]==('var',name): step=1.0 if post[2]=='++' else -1.0
        elif post[0]=='assign' and post[1] in ('+=','-=') and post[2]==('var',name):
            step=lit(post[3])
            if step is not None and post[1]=='-=': step=-step
        else: return None
        if lim is None or step is None: return None
        op=cond[1]; n=0
        while n<=4096 and ((op=='<' and v<lim) or (op=='<=' and v<=lim) or (op=='>' and v>lim) or (op=='>=' and v>=lim) or (op=='!=' and v!=lim)):
            v+=step; n+=1
        return n if n<=4096 else None
    def _needs_const(self, e, name, inside=False):
        """does the loop's body need its index `name` as a constant: in an array index, or handed to a function of the shader
        (which may index with its parameter)? By name: a nested declaration of the same name only keeps the loop unrolled"""
        if isinstance(e,list): return any(self._needs_const(x, name, inside) for x in e)
        if not isinstance(e,tuple) or not e: return False
        if e[0]=='var': return inside and e[1]==name
        if e[0]=='index': return self._needs_const(e[1], name, inside) or self._needs_const(e[2], name, True)
        if e[0]=='call':
            fn=self.callmap[id(e)].name if id(e) in self.callmap else e[1]
            return self._needs_const(e[2], name, inside or fn in self.funcs)
        return any(self._needs_const(x, name, inside) for x in e[1:])
    def _long_for(self, stmt):
        n=self._for_trips(stmt[1], stmt[2], stmt[3])
        if n is None or n<=(self.HW_FOR_CTL if self._has_loopctl(stmt[4]) else self.HW_FOR_PLAIN): return False
        return not self._needs_const(stmt[4], stmt[1][3])
    def _mark_decls(self, s):
        if isinstance(s,tuple) and s:
            if s[0]=='ldecl': self.cond_decls.add(id(s))
            elif s[0]=='multi':
                for x in s[1]: self._mark_decls(x)
    def _mark_base(self, e):
        while e[0] in ('swizzle','index'): e=e[1]
        if e[0]=='var':
            for sc in reversed(self._scan):
                if e[1] in sc: self.cond_decls.add(id(sc[e[1]])); return
    def _cond_expr(self, e):
        """everything an expression run under a lane mask can write"""
        if isinstance(e,list):
            for x in e: self._cond_expr(x)
        elif isinstance(e,tuple) and e:
            if e[0]=='assign': self._mark_base(e[2])
            elif e[0]=='postop': self._mark_base(e[1])
            elif e[0]=='call':
                fn=self.callmap[id(e)].name if id(e) in self.callmap else e[1]
                if fn in self.funcs:
                    for (pq,_,_),a in zip(self.funcs[fn][1], e[2]):
                        if pq in ('out','inout'): self._mark_base(a)
            for x in e[1:]: self._cond_expr(x)

    def run_stmt(self, s):
        k=s[0]
        if k=='multi':                             # comma-separated declarators: run in the CURRENT scope
            for st in s[1]: self.run_stmt(st)
        elif k=='block':
            self.locals.append(Scope())
            for st in s[1]: self.run_stmt(st)
            self.locals.pop()
        elif k=='ldecl':
            _,isc,ty,name,init,arr = s if len(s)==6 else (*s,None)
            if ty in SAMPLERS: return
            self.locals[-1].const.pop(name,None)
            zero=init is None and not self.nozero                    # no value given: 0 (_zero); not the outputs' shadows
            if ty in self.structs and arr is None:                   # local struct: fresh member regs
                sv=self._decl_local_struct(ty, self.eval(init) if init is not None else None)
                if zero: self._zero(sv)
                self.locals[-1][name]=(sv, ty); return
            if arr is not None:                                       # local array: size regs (per element)
                n=int(self.fold_const(arr))
                elems=[self._fresh(ty) for _ in range(n)]
                if init is not None and init[0]=='arrinit':          # array initializer {a,b,c}
                    for j,ie in enumerate(init[1]): rv=self.eval(ie); self.emit('MOV', f'r{elems[j].n}', f'r{rv.n}')
                av=ArrayVal(ty,n,'local',elems=elems)
                if zero: self._zero(av)
                self.locals[-1][name]=(av, f'{ty}[{n}]'); return
            if ty in SCALAR and init is not None and id(s) not in self.cond_decls:
                cv=self.fold_const(init)
                if cv is not None:                                    # compile-time scalar (loop counter etc.)
                    self.locals[-1].pop(name,None); self.locals[-1].const[name]=IntLit(int(cv)) if ty=='int' else cv; return
            if is_mat(ty):
                if init is not None:
                    v=self.eval(init)                                     # matrix local: registers of its own (32-bit: the MatVal)
                    self.locals[-1][name]=(self._clone(v) if self.isa64 else v, ty); return
                n=ncomp2(ty); cols=[self.alloc(f'vec{n}') for _ in range(n)]   # uninitialized matN: N column regs,
                for c in cols: c.tmp=False                                     # written later via m[i]=... (masked/full MOV)
                mv=MatVal(cols, ty)
                if zero: self._zero(mv)
                self.locals[-1][name]=(mv, ty); return
            r=self.alloc(ty); r.tmp=False
            if init is not None:                                     # the initializer still sees an outer x: float x = x + 1.0;
                rv=self.eval(init)
                rv=self._asi(rv) if ty=='int' else self._asf(rv)     # match the declared representation
                self.emit('MOV', f'r{r.n}', f'r{rv.n}'); self.freer(rv)
            elif zero: self._zero(r)
            self.locals[-1][name]=(r,ty)
        elif k=='expr':
            rv=self.eval(s[1]); self.freer(rv)
        elif k in ('precision','lstruct'):
            pass
        elif k=='assign':
            self.do_assign(s)
        elif k=='if':
            self.run_if(s)
        elif k=='for':
            self.run_for(s)
        elif k=='while':
            self.run_while(s)
        elif k=='dowhile':
            self.run_dowhile(s)
        elif k=='discard':
            self.emit('DISCARD')
        elif k=='break':
            if self.loopctl:
                lc=self.loopctl[-1]
                if lc['mode']=='bra': self.emit('BRA', lc['lbreak'])                 # VS: jump out of the loop
                elif lc['mode']=='hw':                                               # FS hardware loop: BREAK op
                    self.emit('SETP','SNE', f'r{self.kf(1.0).n}', f'r{self.zeroreg().n}'); self.emit('BREAK')
                else: self.emit('MOV', f'r{lc["brk"].n}', f'r{self.kf(1.0).n}')       # FS unrolled: per-lane mask
        elif k=='continue':
            if self.loopctl:
                lc=self.loopctl[-1]
                if lc['mode']=='bra': self.emit('BRA', lc['lcont'])                  # VS: jump to iteration end
                elif lc['mode']=='hw':
                    self.emit('SETP','SNE', f'r{self.kf(1.0).n}', f'r{self.zeroreg().n}'); self.emit('CONT')
                else: self.emit('MOV', f'r{lc["cont"].n}', f'r{self.kf(1.0).n}')      # FS unrolled: continue mask
        elif k=='return':
            if self.cur_ret is not None:                 # inside an inlined function, or main (run_main)
                f=self.cur_ret[4]
                if s[1] is not None:
                    rv=self.eval(s[1])
                    if isinstance(rv,(StructVal,MatVal,ArrayVal)):
                        if f is None: self.cur_ret[3][0]=rv              # aggregate: capture
                        else:                                            # one result per lane: registers of its own
                            if self.cur_ret[3][0] is None: self.cur_ret[3][0]=self._like(rv)
                            self._copy_val(self.cur_ret[3][0], rv)
                    else:
                        rv=self._asi(rv) if self.cur_ret[2]=='int' else self._asf(rv)      # match return type repr
                        self.emit('MOV', f'r{self.cur_ret[0].n}', f'r{rv.n}')
                if f is None: self.emit('BRA', self.cur_ret[1])  # branch to the call's epilogue label
                else:
                    if f['depth']: self.emit('MOV', f'r{f["left"].n}', f'r{self.kf(1.0).n}'); f['any']=True
                    self.emit('SETP','SNE', f'r{self.kf(1.0).n}', f'r{self.zeroreg().n}'); self.emit('BREAK')
        elif k=='noop':
            pass
        else:
            # expr-statement wrapped
            rv=self.eval(s); self.freer(rv)
    def _lone_ctl(self, s):
        """BREAK or CONT when a statement is nothing but a break / continue (in blocks of one statement), else None"""
        while isinstance(s,tuple) and s and s[0]=='block' and len(s[1])==1: s=s[1][0]
        if not isinstance(s,tuple) or not s: return None
        return 'BREAK' if s[0]=='break' else 'CONT' if s[0]=='continue' else None
    def _ctl_if(self, c, op):
        """`if (c) break;` in a hardware loop: the BREAK takes the predicate, so SETP c + BREAK, no IF around it. `a || b`
        is a pair each: the lanes a takes out are gone when b is worked out, which is the short circuit (CONT the same)"""
        if c[0]=='bin' and c[1]=='||':
            self._ctl_if(c[2], op); self._ctl_if(c[3], op); return
        cond=self.eval(c)
        self.emit('SETP', 'SNE', f'r{cond.n}', f'r{self.zeroreg().n}')
        self.emit(op)
    def run_if(self, s):
        _,c,th,el=s
        if el is None and self.loopctl and self.loopctl[-1]['mode']=='hw':
            op=self._lone_ctl(th)
            if op: self._ctl_if(c, op); return
        cond=self.eval(c)
        self.emit('SETP', 'SNE', f'r{cond.n}', f'r{self.zeroreg().n}')   # pred = (cond != 0)
        self.freer(cond)
        lelse=self.newlabel('Lelse'); lend=self.newlabel('Lend')
        if self.stage=='fs' or self.isa64:
            # FS is SIMT: per-lane divergence -> mask stack (IF/ELSE/ENDIF), both paths execute masked. So is the
            # 64-bit vertex core (16 vertices per warp, no BRP).
            self.emit('IF', lelse); self.run_stmt(th); self.emit(f'{lelse}:')
            self.emit('ELSE', lend)
            if el is not None: self.run_stmt(el)
            self.emit(f'{lend}:'); self.emit('ENDIF')
        else:
            # the 32-bit VS processes one vertex -> a real scalar branch. BRP F skips 'then' when pred is false.
            self.emit('BRP', lelse, 'F')                 # if !cond -> jump to else
            self.run_stmt(th)
            self.emit('BRA', lend)
            self.emit(f'{lelse}:')
            if el is not None: self.run_stmt(el)
            self.emit(f'{lend}:')
    def _has_loopctl(self, stmt):
        """Does this statement contain a break/continue for THIS loop (not swallowed by a nested loop)?"""
        if not isinstance(stmt,tuple): return False
        k=stmt[0]
        if k in ('break','continue'): return True
        if k in ('for','while'): return False               # belongs to the inner loop
        if k=='block' or k=='multi': return any(self._has_loopctl(s) for s in stmt[1])
        if k=='if': return self._has_loopctl(stmt[2]) or (stmt[3] is not None and self._has_loopctl(stmt[3]))
        return False
    def _run_guarded_body(self, body, brk, cont):
        """Run each top-level body statement guarded by `if(brk==0 && cont==0){ st }`. break sets brk (all
        later iterations skip), continue sets cont (rest of THIS iteration skips). The if-guard reuses the
        FS mask stack / VS branch, so per-lane break/continue is correct in fragment shaders."""
        stmts = body[1] if body[0]=='block' else [body]
        guard = ('bin','&&', ('bin','==',('__reg__',brk),('num',0.0)), ('bin','==',('__reg__',cont),('num',0.0)))
        for st in stmts: self.run_if(('if', guard, st, None))
    def _unroll(self, cond, post, body, ctl, kind):
        """Unroll a const-bounded loop. Without break/continue: plain. With: FS (and the 64-bit VS) uses runtime
        mask flags (per-lane correct); the 32-bit VS (scalar, no divergence) uses BRA out of the unrolled sequence."""
        brk=None; lbreak=None
        simt = self.stage=='fs' or self.isa64
        if ctl and simt:
            brk=self.alloc('float'); brk.tmp=False; self.emit('MOV', f'r{brk.n}', f'r{self.kf(0.0).n}')
        elif ctl:
            lbreak=self.newlabel('Lbrk')
        it=0
        while it<4096:
            c=self.fold_const(cond)
            if c is None: raise CompileError(f"{kind} loop needs constant-evaluable bounds (ES2)")
            if not c: break
            if ctl and simt:
                cont=self.alloc('float'); cont.tmp=False; self.emit('MOV', f'r{cont.n}', f'r{self.kf(0.0).n}')
                self.loopctl.append({'brk':brk,'cont':cont,'mode':'mask'})
                self._run_guarded_body(body, brk, cont); self.loopctl.pop()
            elif ctl:
                lcont=self.newlabel('Lcont')
                self.loopctl.append({'lbreak':lbreak,'lcont':lcont,'mode':'bra'})
                self.run_stmt(body); self.emit(f'{lcont}:'); self.loopctl.pop()
            else:
                self.run_stmt(body)
            if post is not None: self.run_stmt(('expr', post))
            it+=1
        else:
            raise CompileError(f"{kind} loop did not terminate within 4096 unrolled iterations — a "
                               f"const-bounded loop must be statically finite (use a runtime bound for "
                               f"an unbounded loop with break)")
        if lbreak is not None: self.emit(f'{lbreak}:')
    def _refs_runtime(self, e):        # does an expr read a uniform/varying/attribute (a runtime value)?
        if isinstance(e, list): return any(self._refs_runtime(x) for x in e)
        if not isinstance(e, tuple): return False
        if e[0]=='var': return e[1] in self.uniforms or e[1] in self.varyings or e[1] in self.attribs
        return any(self._refs_runtime(x) for x in e[1:])
    def _runtime_loop(self, cond, post, body):
        """A real (non-unrolled) loop for a runtime-bounded condition. 32-bit VS: BRP/BRA back-edge (scalar).
        FS and the 64-bit VS: hardware LOOP/ENDL + BREAK (SIMT, per-lane exit)."""
        if self.stage=='vs' and not self.isa64:
            ltop=self.newlabel('Ltop'); lend=self.newlabel('Lend'); lcont=self.newlabel('Lcont')
            self.emit(f'{ltop}:')
            cv=self.eval(cond); self.emit('SETP','SNE', f'r{cv.n}', f'r{self.zeroreg().n}')
            self.emit('BRP', lend, 'F')                              # exit when condition false
            self.loopctl.append({'lbreak':lend,'lcont':lcont,'mode':'bra'})
            self.run_stmt(body); self.loopctl.pop()
            self.emit(f'{lcont}:')
            if post is not None: self.run_stmt(('expr',post))
            self.emit('BRA', ltop); self.emit(f'{lend}:')            # back-edge
        else:
            # FS hardware LOOP/ENDL + BREAK (per-lane exit). The post (i++) runs at the loop TOP guarded by a
            # "not first iteration" flag, so `continue` (which masks a lane until ENDL) can't skip the update.
            first=None
            if post is not None:
                first=self.alloc('float'); first.tmp=False; self.emit('MOV', f'r{first.n}', f'r{self.kf(1.0).n}')
            self.emit('LOOP', 4096)                                  # SIMT loop; BREAK exits per-lane early (safety cap)
            self.loopctl.append({'mode':'hw'}); self._hw_loop(1)
            if post is not None:
                self.run_if(('if', ('bin','==',('__reg__',first),('num',0.0)), ('expr',post), None))  # post except 1st
                self.emit('MOV', f'r{first.n}', f'r{self.kf(0.0).n}')
            cv=self.eval(cond); nc=self.alloc('float')
            self.emit('SET','SEQ', f'r{nc.n}', f'r{cv.n}', f'r{self.zeroreg().n}')   # nc = (cond==0) = !cond
            self.emit('SETP','SNE', f'r{nc.n}', f'r{self.zeroreg().n}')
            self.emit('BREAK')                                       # break lanes where condition is false
            self.run_stmt(body)
            self.loopctl.pop(); self.emit('ENDL'); self._hw_loop(-1)
    def _names(self, e, name):
        """does a statement or expression read the variable `name` anywhere (by name: a nested one of that name counts too)"""
        if isinstance(e,list): return any(self._names(x, name) for x in e)
        if not isinstance(e,tuple) or not e: return False
        if e[0]=='var': return e[1]==name
        return any(self._names(x, name) for x in e[1:])
    def _counted_loop(self, n, body):
        """A long constant loop whose body never reads its index: `LOOP n` counts the passes in the
        core (shader_core_fs2 lt_cnt: ENDL goes round while passes are left and a lane is in), so no index register, no first-pass
        flag, no exit test: 11 micro-ops a pass less. Every lane is on the same pass, a BREAK takes a lane out, a CONT to the next."""
        for nm in self._written([body], {}):
            if self._const_home(nm) is not None: self._promote(nm)
        self.emit('LOOP', n)
        self.loopctl.append({'mode':'hw'}); self._hw_loop(1)
        self.run_stmt(body)
        self.loopctl.pop(); self.emit('ENDL'); self._hw_loop(-1)
    def _run_loop(self, init, cond, post, body, kind, hw=None):
        """hw: the pass count of a long constant-bounded for loop that is a hardware loop (_long_for, from the pre-pass)"""
        self.locals.append(Scope())                       # loop vars already marked in the pre-pass
        # _long_for: init is `int|float i = <literal>`. The count again from the values here: a parameter of the bound's name
        # (long_loops.frag param) hides the constant the pre-pass counted with, and the loop then tests its bound as it goes
        if hw is not None and not self._names(body, init[3]) and self._for_trips(init, cond, post, True)==hw:
            self._counted_loop(hw, body); self.locals.pop(); return
        if init is not None: self.run_stmt(init)
        # A const-bounded FOR loop unrolls EVEN with break/continue, so the loop index is a known literal
        # each pass -> uniform array[i] resolves (required by ES2: array indices are constant-index-exprs).
        # Runtime-bounded loops, and while-loops with break (possibly while(true)), use the hardware loop;
        # so does a long constant-bounded for loop whose body does not need the index as a constant (hw: _long_for).
        always=cond[0]=='num' and bool(cond[1]) and post is None or (cond[0]=='num' and bool(cond[1]) and kind=='for')
        unknown=self.fold_const(cond) is None
        if self._refs_runtime(cond) or ((kind=='while' or always) and self._has_loopctl(body)) or unknown or hw is not None:
            # what the loop writes must be real registers — its CONDITION's writes too (in `for (; i++ < 20.; )`
            # a tracked constant i would compare that constant in every pass and run to the 4096-pass cap)
            for n in self._written([cond, post, body], {}):
                if self._const_home(n) is not None: self._promote(n)
            self._runtime_loop(cond, post, body)
        else:
            self._unroll(cond, post, body, self._has_loopctl(body), kind)
        self.locals.pop()
    def _written(self, e, out):
        """the names an expression or statement assigns"""
        if isinstance(e,list):
            for x in e: self._written(x, out)
        elif isinstance(e,tuple) and e:
            if e[0] in ('assign','postop'):
                t=e[2] if e[0]=='assign' else e[1]
                while t[0] in ('swizzle','index'): t=t[1]
                if t[0]=='var': out[t[1]]=1
            elif e[0]=='call':
                fn=self.callmap[id(e)].name if id(e) in self.callmap else e[1]
                if fn in self.funcs:
                    for (pq,_,_),a in zip(self.funcs[fn][1], e[2]):
                        if pq in ('out','inout'): self._written(('assign','=',a,None), out)
            for x in e[1:]: self._written(x, out)
        return out
    def run_for(self, s):   _,init,cond,post,body=s; self._run_loop(init, cond, post, body, 'for', self.hw_for.get(id(s)))
    def run_while(self, s): _,cond,body=s;           self._run_loop(None, cond, None, body, 'while')
    def run_dowhile(self, s):
        _,cond,body=s
        self.locals.append(Scope())
        for n in self._written([cond], {}):               # a condition that writes a tracked constant (as _run_loop)
            if self._const_home(n) is not None: self._promote(n)
        self._do_loop(cond, body)
        self.locals.pop()
    def _has_continue(self, stmt):
        """a `continue` of THIS loop in the statement (not one of a nested loop)"""
        if not isinstance(stmt,tuple): return False
        k=stmt[0]
        if k=='continue': return True
        if k in ('for','while','dowhile'): return False
        if k=='block' or k=='multi': return any(self._has_continue(s) for s in stmt[1])
        if k=='if': return self._has_continue(stmt[2]) or (stmt[3] is not None and self._has_continue(stmt[3]))
        return False
    def _do_loop(self, cond, body):
        """do { body } while(cond): body ALWAYS runs at least once, cond tested after. The 32-bit VS uses a scalar
        bottom-tested back-edge; FS and the 64-bit VS (SIMT) test cond at the loop TOP guarded by a 'not first
        iteration' flag so a per-lane `continue` (masked until ENDL) still gets its condition evaluated next round."""
        if self.stage=='vs' and not self.isa64:
            ltop=self.newlabel('Ldo'); lend=self.newlabel('Ldend'); lcont=self.newlabel('Ldcont')
            self.emit(f'{ltop}:')
            self.loopctl.append({'lbreak':lend,'lcont':lcont,'mode':'bra'})
            self.run_stmt(body); self.loopctl.pop()
            self.emit(f'{lcont}:')                                    # continue -> re-test the condition
            cv=self.eval(cond); self.emit('SETP','SNE', f'r{cv.n}', f'r{self.zeroreg().n}')
            self.emit('BRP', lend, 'F')                              # exit when cond false
            self.emit('BRA', ltop); self.emit(f'{lend}:')            # back-edge is a BRA (loop-region detected -> loop liveness)
        elif not self._has_continue(body):
            # no `continue` in the body: the body, then the condition, once a pass, as written. The form
            # below evaluates the condition before the FIRST body too and only ignores its result — one evaluation too
            # many when the condition writes (do { } while (i++ < n))
            self.emit('LOOP', 4096)
            self.loopctl.append({'mode':'hw'}); self._hw_loop(1)
            self.run_stmt(body)
            cv=self.eval(cond); z=self.zeroreg(); nc=self.alloc('float')
            self.emit('SET','SEQ', f'r{nc.n}', f'r{cv.n}', f'r{z.n}')                          # !cond
            self.emit('SETP','SNE', f'r{nc.n}', f'r{z.n}'); self.emit('BREAK')
            self.loopctl.pop(); self.emit('ENDL'); self._hw_loop(-1)
        else:
            # with a `continue`: the test at the loop top, where a CONT lane comes back, and worked out only where a pass has
            # run: an IF on the first-pass flag around it, so a condition that writes runs once a pass, as written
            first=self.alloc('float'); first.tmp=False; self.emit('MOV', f'r{first.n}', f'r{self.kf(1.0).n}')
            self.emit('LOOP', 4096)
            self.loopctl.append({'mode':'hw'}); self._hw_loop(1)
            z=self.zeroreg()
            brk=self.alloc('float'); self.emit('MOV', f'r{brk.n}', f'r{z.n}')                     # no break where nothing has run
            self.emit('SETP','SEQ', f'r{first.n}', f'r{z.n}')                                      # past the first pass
            lelse=self.newlabel('Lelse'); lend=self.newlabel('Lend')
            self.emit('IF', lelse)
            cv=self.eval(cond); self.emit('SET','SEQ', f'r{brk.n}', f'r{cv.n}', f'r{z.n}')        # !cond
            self.emit(f'{lelse}:'); self.emit('ELSE', lend); self.emit(f'{lend}:'); self.emit('ENDIF')
            self.emit('MOV', f'r{first.n}', f'r{z.n}')
            self.emit('SETP','SNE', f'r{brk.n}', f'r{z.n}'); self.emit('BREAK')
            self.run_stmt(body)
            self.loopctl.pop(); self.emit('ENDL'); self._hw_loop(-1)

def ncomp2(t): return MAT[t]

# =============== optimization + register allocation passes (operate on the emitted .sasm) ===============
# A register token is [-!]*rN[.swz]: the optional swizzle suffix (isa64 mode) rides along through every pass.
_REGTOK = re.compile(r'^([-!]*)r(\d+)(\.[xyzw]{1,4})?$')
_ALU_DST = {'ADD','SUB','MUL','MAD','MIN','MAX','DP3','DP4','MOV','RCP','RSQRT','LOG2','EXP2',
            'FTOI','ITOF','IADD','ISUB','ISLT','ISGE','DDX','DDY','LDP','LDV','LDPX','TEX',
            'SEL','IDIV','FLR','CEIL','TRUNC','RNDE','SIN','COS',
            'ISEQ','ISNE','IMUL','IMOD','UDIV','UMOD','FTOU','UTOF','MOVI',
            'SQRT','RCPM','RSQM','CLAMP'}   # ops whose toks[1] is a GPR dest
_SIDE = {'STO','STV','STOZ','DISCARD','SETP','IF','ELSE','ENDIF','BRA','BRP','LOOP','ENDL','END','BREAK','CONT','NOP'}

def _dst(toks):
    base=toks[0].split('.')[0]
    if base=='SET':                                  # SET cc rd ra rb -> dest is toks[2]
        m=_REGTOK.match(toks[2]); return int(m.group(2)) if m else None
    if base in _ALU_DST:
        m=_REGTOK.match(toks[1]); return int(m.group(2)) if m else None
    return None
def _srcs(toks):
    base=toks[0].split('.')[0]
    if base=='SET':                                  # SET cc rd ra rb -> sources are toks[3],toks[4]
        return [int(m.group(2)) for t in toks[3:5] for m in [_REGTOK.match(t)] if m]
    d = 1 if _dst(toks) is not None else -1
    out=[]
    for i,t in enumerate(toks):
        if i==0 or i==d: continue
        m=_REGTOK.match(t)
        if m: out.append(int(m.group(2)))
    return out

def _loop_regions(lines):
    """[(lo,hi)] loop spans: a backward BRA (VS back-edge) or a LOOP..ENDL (FS). Used so DCE / register
    allocation know a value read at the loop top is still live at the back-edge."""
    labelpos={};
    for i,ln in enumerate(lines):
        if ln.endswith(':'): labelpos[ln[:-1]]=i
    loops=[]; stk=[]
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        t=ln.split()
        if t[0]=='BRA' and t[1] in labelpos and labelpos[t[1]]<i: loops.append((labelpos[t[1]], i))
        elif t[0]=='LOOP': stk.append(i)
        elif t[0]=='ENDL' and stk: loops.append((stk.pop(), i))
    return loops

def dce(lines):
    """Dead-code elimination: drop any instruction whose GPR dest is never read later and that has no
    side effect (STO/STV/STOZ/DISCARD keep). Masked writes keep the dest live (they read old lanes), so
    construction chains survive. Loop-aware: a dest read anywhere in an enclosing loop is live (back-edge).
    Iterates to a fixpoint (removing a dead op can free its inputs).

    Scalable form: each sweep precomputes every register's read positions ONCE (O(n)) and BATCH-deletes all
    currently-dead instructions, instead of deleting one and restarting the fixpoint. Deleting a dead
    instruction only removes reads (monotone: dead stays dead, never resurrects a kept op), so batch removal
    reaches the identical fixpoint as the one-at-a-time original — just far fewer passes on big shaders."""
    lines=list(lines); changed=True
    while changed:
        changed=False
        loops=_loop_regions(lines)
        reads_at={}                                        # reg -> ascending list of indices that READ it
        for idx,ln in enumerate(lines):
            if ln.endswith(':'): continue
            toks=ln.split()
            for r in _srcs(toks): reads_at.setdefault(r,[]).append(idx)
            d=_dst(toks)
            if d is not None and '.' in toks[0]:           # masked write reads the old dest lanes
                reads_at.setdefault(d,[]).append(idx)
        maxread={r:v[-1] for r,v in reads_at.items()}      # v built in ascending order -> last is the max
        dead=set()
        for i,ln in enumerate(lines):
            if ln.endswith(':'): continue
            toks=ln.split(); base=toks[0].split('.')[0]
            if base in _SIDE: continue
            d=_dst(toks)
            if d is None: continue
            used = maxread.get(d,-1) > i                    # read anywhere after i
            if not used:                                   # also live if read earlier in an enclosing loop
                for lo,hi in loops:
                    if lo<=i<=hi and any(lo<=x<i for x in reads_at.get(d,())): used=True; break
            if not used: dead.add(i)
        if dead:
            lines=[ln for k,ln in enumerate(lines) if k not in dead]; changed=True
    return lines

_CTRL = {'IF','ELSE','ENDIF','BRP','BRA','LOOP','ENDL','SETP','STO','STV','STOZ','DISCARD','END','TEX'}

def _regnum(tok):
    m=_REGTOK.match(tok); return int(m.group(2)) if m else None
def _hasmod(tok):
    m=_REGTOK.match(tok); return bool(m and m.group(1))
def _hasswz(tok):
    m=_REGTOK.match(tok); return bool(m and m.group(3))
def _occ(lines):
    o={}
    for ln in lines:
        if ln.endswith(':'): continue
        toks=ln.split()
        for r in _srcs(toks): o[r]=o.get(r,0)+1
        d=_dst(toks)
        if d is not None: o[d]=o.get(d,0)+1
    return o

def strip_dead_bra(code):
    """Drop a `BRA L` immediately followed by `L:` (a no-op branch to the next instruction — the common
    tail-return case after function inlining)."""
    out=[]
    for i,l in enumerate(code):
        t=l.split()
        if t and t[0]=='BRA' and i+1<len(code) and code[i+1]==f'{t[1]}:': continue
        out.append(l)
    return out

_LICM_OPS = {'ADD','SUB','MUL','MAD','MIN','MAX','DP3','DP4','MOV','RCP','RSQRT','LOG2','EXP2','SIN','COS','FLR','CEIL',
             'TRUNC','RNDE','SEL','SET','FTOI','ITOF','FTOU','UTOF','IADD','ISUB','IMUL','ISLT','ISGE','ISEQ','ISNE','LDP','MOVI',
             'SQRT','RCPM','RSQM','CLAMP'}

def _licm_loop(lines, lo, hi):
    """the lines of LOOP lo .. ENDL hi that can run once before it, or an empty set. A line goes when it is at the loop's own
    level (every lane in the loop reaches it each pass: not under an IF or an inner loop), what it reads is written in the loop
    only by lines that go too, every write of its register in the loop goes, and that register is read nowhere outside the loop
    nor in it before its first write: then each pass would write it the same values, and no one sees it before or after."""
    depth=0; info={}; wr={}; rd={}
    for k in range(lo+1,hi):
        ln=lines[k]
        if ln.endswith(':'): continue
        toks=ln.split(); base=toks[0].split('.')[0]; d=_dst(toks); rs=set(_srcs(toks)); own=d in rs   # own: reads what it writes (d += k)
        if d is not None and '.' in toks[0]: rs.add(d)                  # a masked write keeps the old lanes
        info[k]=(base, d, rs, depth, own)
        if d is not None: wr.setdefault(d,[]).append(k)
        for r in rs: rd.setdefault(r,[]).append(k)
        if base in ('IF','LOOP'): depth+=1
        elif base in ('ENDIF','ENDL'): depth-=1
    outside=set()
    for k,ln in enumerate(lines):
        if lo<=k<=hi or ln.endswith(':'): continue
        toks=ln.split(); outside.update(_srcs(toks))
        d=_dst(toks)
        if d is not None and '.' in toks[0]: outside.add(d)
    cand={k for k,(base,d,rs,dep,own) in info.items() if dep==0 and not own and base in _LICM_OPS and d is not None and d not in outside
          and min(rd.get(d,[hi]))>=min(wr[d])}
    while True:
        bad=set()
        for k in cand:
            base,d,rs,dep,own=info[k]
            if any(w not in cand for w in wr[d]): bad.add(k); continue
            if any(any(w not in cand or w>k for w in wr[s]) for s in rs if s in wr and s!=d): bad.add(k)
        if not bad: return cand
        cand-=bad

def licm(lines):
    """forward=True: loop-invariant code motion out of hardware loops (e.g. a vec3(0.0, 1.0, 0.0)
    built from three scalars every pass). Innermost loop first, a loop at a time to a fixpoint: what leaves an inner loop may leave its
    outer one next. A LOOP 1 (a function's early return) runs once anyway."""
    while True:
        stk=[]; moved=False
        for i,ln in enumerate(lines):
            if ln.endswith(':'): continue
            t=ln.split()
            if t[0]=='LOOP': stk.append(i)
            elif t[0]=='ENDL' and stk:
                lo=stk.pop()
                if lines[lo].split()[1]=='1': continue
                go=_licm_loop(lines, lo, i)
                if go:
                    lines=lines[:lo]+[lines[k] for k in sorted(go)]+[lines[lo]]+[lines[k] for k in range(lo+1,len(lines)) if k not in go]
                    moved=True; break
        if not moved: return lines

def drop_labels(code):
    """forward=True: a label nothing jumps to goes. An inlined function's epilogue label (Lret) stays behind when strip_dead_bra
    drops the BRA to it, and as a block end it would stop block_forward and the other per-block passes for nothing (a
    `return dot(p, n) + h;` called in a loop would keep a copy a pass). No jump lands there, so it joins no paths."""
    refs=set()
    for ln in code:
        if not ln.endswith(':'): refs.update(ln.split()[1:])
    return [ln for ln in code if not (ln.endswith(':') and ln[:-1] not in refs)]

def hoist_consts(code):
    """When the program has real branches (VS BRP/BRA), the constant pool loads (`LDP rN @M`) are CACHED
    (emitted once, reused) — but a cached load emitted inside one branch is stale on the other path. Hoist
    all constant-pool loads to the top so they execute before any branch. (No-op without BRP/BRA: FS
    if/else uses masked IF/ELSE where both paths execute, and loops are unrolled.)
    A hardware LOOP too: a constant first used after a loop's BREAK would be loaded there, and used after the
    loop it would be missing in the lanes that left before, its register taken by the next iteration's first instructions in
    the rest (gles_test "a loop on a local bound")."""
    if not any((not l.endswith(':')) and l.split()[0] in ('BRP','BRA','IF','ELSE','LOOP') for l in code): return code
    def is_const_ld(l):
        t=l.split(); return (not l.endswith(':')) and t[0]=='LDP' and len(t)==3 and t[2].startswith('@')
    consts=[l for l in code if is_const_ld(l)]
    if not consts: return code
    return consts + [l for l in code if not is_const_ld(l)]

def _interleave(front):
    """forward=True: the work every TEX coordinate of the front needs first, then the TEXes, each as soon as what it
    reads is there. A warp issues in order and a varying's LDV is a chain (its lambda2 steps wait for the lambda1 ones to
    land), so `LDV a, TEX, LDV b, TEX` waits a chain per TEX, `LDV a, LDV b, TEX, TEX` one
    (the second TEX ~10 cycles after the first instead of ~21). A list schedule over the front's dependencies (a read of
    what an earlier one writes, a write of what it reads or writes, a masked write reading its old lanes): the earliest
    ready instruction that is not a TEX, else the earliest ready TEX (a TEX of a TEX's result waits for it)."""
    n=len(front); info=[]
    for l in front:
        t=l.split(); base=t[0].split('.')[0]; d=_dst(t); rs=set(_srcs(t))
        if d is not None and '.' in t[0] and base in _ALU_DST: rs.add(d)
        info.append((d, rs, base=='TEX'))
    preds=[set() for _ in range(n)]
    for b in range(n):
        db,rb,_=info[b]
        for a in range(b):
            da,ra,_=info[a]
            if (da is not None and (da in rb or da==db)) or (db is not None and db in ra): preds[b].add(a)
    out=[]; done=set(); left=list(range(n))
    while left:
        pick=next((x for x in left if not info[x][2] and preds[x]<=done), None)
        if pick is None: pick=next(x for x in left if preds[x]<=done)
        out.append(front[pick]); done.add(pick); left.remove(pick)
    return out

def _sched_region(region, interleave=False):
    """Reorder ONE straight-line region so each TEX (with the instructions that produce its uv) moves to
    the front, followed by everything else in original order. The FS core is NON-BLOCKING on TEX, so
    issuing the sample early lets the region's remaining (texel-independent) math overlap the ~21-cyc TMU
    fetch. Pre-allocation virtual regs are SINGLE-ASSIGNMENT (alloc never recycles; masked-write chains are
    handled by pulling ALL of a reg's in-region writers), so the dependency-closed backward slice of a TEX
    is a valid topological prefix — moving it to the front preserves the dataflow exactly."""
    m=len(region)
    if m<2 or not any(l.split()[0].split('.')[0]=='TEX' for l in region): return region
    reads=[]; writers={}
    for k,l in enumerate(region):
        t=l.split(); base=t[0].split('.')[0]
        d=_dst(t); s=set(_srcs(t))
        if d is not None and '.' in t[0] and base in _ALU_DST: s.add(d)   # masked write reads old dest
        reads.append(s)
        if d is not None: writers.setdefault(d,[]).append(k)
    # dependency-closed backward slice of every TEX (BFS over RAW producers earlier in the region)
    stack=[k for k,l in enumerate(region) if l.split()[0].split('.')[0]=='TEX']
    inslice=set(stack)
    while stack:
        k=stack.pop()
        for r in reads[k]:
            for w in writers.get(r,()):
                if w<k and w not in inslice: inslice.add(w); stack.append(w)
    if len(inslice)==m: return _interleave(region) if interleave else region   # the whole region feeds the TEXes
    front=[region[k] for k in range(m) if k in inslice]
    rest =[region[k] for k in range(m) if k not in inslice]
    return (_interleave(front) if interleave else front)+rest

def hoist_tex(lines, interleave=False):
    """Texture-latency-hiding schedule. The FS core is NON-BLOCKING on TEX: a TEX only marks its dest in the
    per-register scoreboard; subsequent INDEPENDENT instructions keep issuing while the ~21-cyc TMU fetch is
    in flight, and only an instruction that READS the texel dest stalls (until writeback). Hoisting each TEX
    (and its uv-producing slice) to the top of its straight-line region maximises the independent-math window
    that overlaps the fetch — hiding the TMU latency with a single outstanding request. Pure performance
    reorder: results are bit-identical, only cycle count drops. Motion is confined to a straight-line region
    (a control op / label / side-effect op ends it), so a TEX inside an if/loop stays put (ES2: implicit-LOD
    sampling requires uniform control flow anyway). Texel-dest live ranges lengthen; allocate_registers (run
    afterwards) accounts for that."""
    def is_boundary(ln):
        if ln.endswith(':'): return True                    # label: a new region begins
        return ln.split()[0].split('.')[0] in _SIDE         # IF/ELSE/ENDIF/LOOP/ENDL/BRA/BRP/SETP/STO/STV/STOZ/DISCARD/END/...
    out=[]; i=0; n=len(lines)
    while i<n:
        if is_boundary(lines[i]): out.append(lines[i]); i+=1; continue
        lo=i
        while i<n and not is_boundary(lines[i]): i+=1
        out.extend(_sched_region(lines[lo:i], interleave))
    return out

def _replace_src(ln, dv, sv):                        # replace reg #dv with #sv in a line, keeping modifiers + swizzle
    toks=ln.split(); out=[toks[0]]
    for t in toks[1:]:
        m=_REGTOK.match(t)
        out.append(f'{m.group(1)}r{sv}{m.group(3) or ""}' if (m and int(m.group(2))==dv) else t)
    return ' '.join(out)

def _rhs(toks):                                      # value-identity key (op + sources, excluding dest)
    base=toks[0].split('.')[0]
    if base=='SET': return (toks[0], toks[1], toks[3], toks[4])
    return (toks[0],) + tuple(toks[2:])

def copy_prop(lines):
    """Copy propagation: for a plain full `MOV d,s` where d is defined only here and s is not redefined over
    d's live range, replace every use of d with s and drop the MOV. (Modifier prefixes on uses preserved.)

    Scalable form: each sweep collects a batch of mutually-independent propagations (disjoint source/dest
    registers, so they commute) and applies them together instead of one-then-restart. Copy propagation is
    confluent for valid moves, so the batched fixpoint is identical to the sequential one."""
    lines=list(lines); changed=True
    while changed:
        changed=False
        defc={}
        for ln in lines:
            if ln.endswith(':'): continue
            d=_dst(ln.split())
            if d is not None: defc[d]=defc.get(d,0)+1
        chosen=[]; used=set()                                  # used = regs already claimed this sweep
        for i,ln in enumerate(lines):
            if ln.endswith(':'): continue
            toks=ln.split()
            if toks[0]!='MOV' or len(toks)!=3 or _hasmod(toks[2]) or _hasswz(toks[2]): continue   # a swizzled copy is a permutation
            dv,sv=_regnum(toks[1]),_regnum(toks[2])
            if dv is None or sv is None or dv==sv or defc.get(dv)!=1: continue
            if dv in used or sv in used: continue              # keep the batch independent
            lastuse=-1
            for k in range(i+1,len(lines)):
                if not lines[k].endswith(':') and dv in _srcs(lines[k].split()): lastuse=k
            if any((not lines[k].endswith(':')) and _dst(lines[k].split())==sv for k in range(i+1,lastuse+1)):
                continue                                       # s redefined within d's live range -> unsafe
            chosen.append((i,dv,sv)); used.add(dv); used.add(sv)
        if chosen:
            delset=set(i for (i,_,_) in chosen)
            for (i,dv,sv) in chosen:
                for k in range(len(lines)):
                    if k in delset or lines[k].endswith(':'): continue
                    lines[k]=_replace_src(lines[k], dv, sv)
            lines=[ln for k,ln in enumerate(lines) if k not in delset]; changed=True
    return lines

# ---- block-local copy forwarding + dead stores (forward=True: the host libraries, tools default to False). A mutable
# local is defined more than once, which copy_prop leaves alone, and a translated Direct3D shader is nothing but those:
# 35 of the 63 instructions of NFS HP2's main vertex shader were MOVs. Both passes work per component inside a basic
# block; inside one every instruction runs under the same lanes, which is what makes either of them legal.
_BLOCK_END = {'IF','ELSE','ENDIF','BRP','BRA','LOOP','ENDL','BREAK','CONT','SETP','DISCARD','END'}
# The readers a copy with a modifier may reach: the float ops, compares and stores, whose operands go through the modifier field
# bit for bit as the MOV's did (isa64._mod: negate flips the sign bit, abs clears it, -abs sets it). Not SEL (the core reads
# its registers raw), the integer ops (no modifiers), a TEX coordinate or an LDPX index.
_NEG_OK = {'ADD','SUB','MUL','MAD','MIN','MAX','DP3','DP4','MOV','RCP','RSQRT','LOG2','EXP2','STO','STV',
           'SET','SETP','STOZ','SIN','COS','FLR','CEIL','TRUNC','RNDE','SQRT','RCPM','RSQM','CLAMP'}
_MODTXT = ('', '-', '!', '-!')

def _modbits(t):
    """a token's modifier text as isa64.parse_src reads it: '-' toggles the negation, '!' sets the absolute value"""
    m=0
    for ch in t: m = m^1 if ch=='-' else m|2
    return m

def _modcomp(outer, inner):
    """the modifier of `outer applied to (inner applied to x)`: an abs drops what was inside, a negation flips the sign"""
    return outer if outer&2 else inner^outer

def _swz4(swz):
    """a token's swizzle as 4 component numbers (isa64.enc_swz: 1 letter splats, of 2-3 the last repeats)"""
    s=swz[1:] if swz else 'xyzw'
    s=s+s[-1]*(4-len(s))
    return ['xyzw'.index(ch) for ch in s]

def _swz_text(q):
    s=''.join('xyzw'[c] for c in q)
    return '' if s=='xyzw' else '.'+s[0] if s==s[0]*4 else '.'+s

def _wmask(toks):
    return sum(1<<'xyzw'.index(ch) for ch in toks[0].split('.')[1]) if '.' in toks[0] else 15

def block_forward(lines, sv):
    """Reads of a copy become reads of its source, component by component, while neither has been written since.
    sv: the scalar registers, every component of which is the one value. A swizzled read of the register being
    written is left alone: the assembler refuses an instruction that reads a component it has already written.
    Only the swizzle positions the instruction reads count (_read_pos): a TEX's .xy reads two of a vec4 a constructor
    built (gl_ff.c's tightened coordinates), and the copy those two came from is enough. A copy with a modifier (`MOV t, !x`,
    abs() and negation) reaches the readers of _NEG_OK with the two modifiers combined (_modcomp), so `MOV t, !x; SET d, t, y`
    is one SET (e.g. `abs(dS) < SURF_DIST`)."""
    out=[]; cp={}                                       # (reg, comp) -> (source reg, source comp, modifier 0..3)
    for ln in lines:
        if ln.endswith(':'): cp={}; out.append(ln); continue
        toks=ln.split(); base=toks[0].split('.')[0]; dp=_dst_pos(toks)
        d=_regnum(toks[dp]) if dp is not None else None
        for i in range(1,len(toks)):
            if i==dp: continue
            m=_REGTOK.match(toks[i])
            if not m: continue
            mod,r,q=m.group(1),int(m.group(2)),_swz4(m.group(3))
            res=[cp.get((r,c),(r,c,0)) for c in q]
            rp=[p for p in range(4) if _read_pos(toks)>>p&1]
            s,cm=res[rp[0]][0],res[rp[0]][2]
            if any(res[p][0]!=s or res[p][2]!=cm for p in rp): continue
            nq=[res[p][1] if res[p][0]==s and res[p][2]==cm else None for p in range(4)]
            ident=all(nq[p]==p for p in rp)
            nq=[c if c is not None else p if ident else nq[rp[-1]] for p,c in enumerate(nq)]   # a position nothing reads: as its own, or the last read one's
            if s==r and all(nq[p]==q[p] for p in rp) and not cm: continue
            if cm and base not in _NEG_OK: continue
            if s==d and nq!=[0,1,2,3]: continue
            nmod=_MODTXT[_modcomp(_modbits(mod), cm)] if cm else mod
            toks[i]=f'{nmod}r{s}{"" if s in sv else _swz_text(nq)}'
        if base=='MOV' and len(toks)==3 and toks[2]==f'r{d}': continue        # the identity, under any mask
        out.append(' '.join(toks))
        if base in _BLOCK_END: cp={}; continue
        if d is None: continue
        wm=15 if d in sv else _wmask(toks)
        for k in [k for k,v in cp.items() if (k[0]==d and wm>>k[1]&1) or (v[0]==d and wm>>v[1]&1)]: del cp[k]
        if base=='MOV' and len(toks)==3:
            m=_REGTOK.match(toks[2])
            if m and int(m.group(2))!=d:
                s,q,cm=int(m.group(2)),_swz4(m.group(3)),_modbits(m.group(1))
                for c in range(4):
                    if wm>>c&1: cp[(d,c)]=(s,q[c],cm)
    return out

def dead_stores(lines, sv):
    """Going backwards through each block: a write every component of which is written again before it is read, or is
    never read again, goes. Unlike dce this sees masked writes, and a local's zero initialisation."""
    loops=_loop_regions(lines); reads={}
    for idx,ln in enumerate(lines):
        if ln.endswith(':'): continue
        for r in _srcs(ln.split()): reads.setdefault(r,[]).append(idx)
    def read_from(r, end):                              # read at or after `end`, or in a loop around it (the back-edge)
        v=reads.get(r,())
        return bool(v) and (v[-1]>=end or any(lo<=end<=hi and any(lo<=x<end for x in v) for lo,hi in loops))
    keep=[True]*len(lines); over={}; end=len(lines)     # over: reg -> the components no later read of this block sees
    def ov(r):
        if r not in over: over[r]=0 if read_from(r,end) else 15
        return over[r]
    for i in range(len(lines)-1,-1,-1):
        ln=lines[i]
        if ln.endswith(':'): continue                    # a jump here cannot reach what precedes: every way out of that code ends its block
        toks=ln.split(); base=toks[0].split('.')[0]
        if base in _BLOCK_END: over={}; end=i
        dp=_dst_pos(toks)
        if dp is not None:
            d=_regnum(toks[dp]); wm=15 if d in sv else _wmask(toks)
            if not wm&~ov(d): keep[i]=False; continue
            over[d]=ov(d)|wm
        for k in range(1,len(toks)):
            if k==dp: continue
            m=_REGTOK.match(toks[k])
            if m:
                r=int(m.group(2)); o=ov(r)
                over[r]=o&~_read_mask(toks,k)
    return [ln for k,ln in enumerate(lines) if keep[k]]

def forward_opt(lines, svregs):
    sv=set(svregs or ())
    while True:
        new=dce(dead_stores(block_forward(lines, sv), sv))
        if new==lines: return new
        lines=new

# ---- shape_opt (forward=True, isa64; after hoist_tex, whose slices need every register written once): what the code generator's
# shapes cost. Each rule takes a batch of register-disjoint changes per sweep (they commute), the rules in turn to a fixpoint.
_FOLD_OPS = {'ADD','SUB','MUL','MAD','MIN','MAX'}

def _tab(lines):
    """per register: the lines that write it, the lines that read it (one entry a token), first and last mention"""
    wr={}; rd={}; first={}; last={}
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split(); d=_dst(toks)
        if d is not None:
            wr.setdefault(d,[]).append(i); first.setdefault(d,i); last[d]=i
        for r in _srcs(toks):
            rd.setdefault(r,[]).append(i); first.setdefault(r,i); last[r]=i
    return wr, rd, first, last

def _plain(tok):
    m=_REGTOK.match(tok); return m is not None and not m.group(1) and not m.group(3)

def _splat(tok):
    m=_REGTOK.match(tok); return m is not None and m.group(3) is not None and len(m.group(3))==2

def _ends_block(ln):
    return ln.endswith(':') or ln.split()[0].split('.')[0] in _BLOCK_END

def _coalesce(lines, sv):
    """`MOV[.m] d, s`: d merges into s when d is first mentioned there, s is never written after, and d is not written
    before s's last read: until then the two hold the same value, after it only d is read. Not across a loop (the next
    iteration could read s after d's write). A masked copy leaves d's other lanes s's, where they were nobody's."""
    wr,rd,first,last=_tab(lines); loops=_loop_regions(lines)
    remap={}; dele=set(); used=set()
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split()
        if toks[0].split('.')[0]!='MOV' or len(toks)!=3 or not _plain(toks[1]) or not _plain(toks[2]): continue
        d,s=_regnum(toks[1]),_regnum(toks[2])
        if d==s or d in used or s in used or (d in sv)!=(s in sv) or first.get(d)!=i: continue
        if any(k>i for k in wr.get(s,())): continue
        L=max(rd[s])
        if any(i<k<=L for k in wr.get(d,())): continue
        if any(lo<=last[d] and hi>=i for lo,hi in loops): continue
        remap[d]=s; dele.add(i); used.add(d); used.add(s)
    if not remap: return lines, False
    out=[]
    for i,ln in enumerate(lines):
        if i in dele: continue
        if ln.endswith(':'): out.append(ln); continue
        toks=ln.split()
        for k in range(1,len(toks)):
            m=_REGTOK.match(toks[k])
            if m and int(m.group(2)) in remap: toks[k]=f'{m.group(1)}r{remap[int(m.group(2))]}{m.group(3) or ""}'
        out.append(' '.join(toks))
    return out, True

def _mask_fold(lines, sv):
    """`OP t, a, b` then `MOV.m d, t`, t read only there: `OP.m d, a, b` where the OP was, d not mentioned in between (same
    block). A source that is d is read unswizzled: each micro-op reads its own component before writing it (isa64.check_overlap)."""
    wr,rd,first,last=_tab(lines)
    new={}; dele=set(); used=set()
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split()
        if toks[0].split('.')[0]!='MOV' or '.' not in toks[0] or len(toks)!=3 or not _plain(toks[1]) or not _plain(toks[2]): continue
        d,t=_regnum(toks[1]),_regnum(toks[2])
        if d==t or d in sv or d in used or t in used or len(wr.get(t,()))!=1 or len(rd.get(t,()))!=1: continue
        j=wr[t][0]
        if j>=i or j in new: continue
        tj=lines[j].split()
        if tj[0] not in _FOLD_OPS or not _plain(tj[1]): continue
        srcs=tj[2:]; regs=[_regnum(x) for x in srcs]
        if any(r is None or r==t or r in used for r in regs): continue
        if any(r==d and _hasswz(x) for r,x in zip(regs,srcs)): continue
        if t in sv and any(r not in sv and not _splat(x) for r,x in zip(regs,srcs)): continue   # a scalar from splats only
        if any(_ends_block(lines[k]) or d in _srcs(lines[k].split()) or _dst(lines[k].split())==d for k in range(j+1,i)): continue
        new[j]=f'{tj[0]}.{toks[0].split(".")[1]} r{d} '+' '.join(srcs); dele.add(i)
        used.update([d,t]+regs)
    if not new: return lines, False
    return [new.get(i,ln) for i,ln in enumerate(lines) if i not in dele], True

def _copy_back(lines, sv):
    """`OP t, a, b` then `MOV d, t`, t written only there: `OP d, a, b` and t's reads read d (so `dO += dS` is not
    `ADD t, dO, dS; MOV dO, t` each pass of a loop). Both whole writes in one block, d not mentioned between them; t read
    only after the OP (not round a loop), and d written nowhere from the MOV to t's last read nor in a loop that reads t after
    the OP. A source that is d is read unswizzled (as _mask_fold)."""
    wr,rd,first,last=_tab(lines); loops=_loop_regions(lines)
    new={}; dele=set(); used=set(); remap={}
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split()
        if toks[0]!='MOV' or len(toks)!=3 or not _plain(toks[1]) or not _plain(toks[2]): continue
        d,t=_regnum(toks[1]),_regnum(toks[2])
        if d==t or (d in sv)!=(t in sv) or d in used or t in used or len(wr.get(t,()))!=1: continue
        j=wr[t][0]
        if j>=i or j in new: continue
        tj=lines[j].split()
        if tj[0] not in _FOLD_OPS or not _plain(tj[1]): continue
        srcs=tj[2:]; regs=[_regnum(x) for x in srcs]
        if any(r is None or r==t or r in used for r in regs): continue
        if any(r==d and _hasswz(x) for r,x in zip(regs,srcs)): continue
        if any(_ends_block(lines[k]) or d in _srcs(lines[k].split()) or _dst(lines[k].split())==d for k in range(j+1,i)): continue
        tr=rd.get(t,())
        if min(tr)<=j: continue
        lr=max(tr)
        if any(i<k<=lr for k in wr.get(d,())): continue
        if any(lo>j and any(lo<=x<=hi for x in tr) and any(lo<=k<=hi for k in wr.get(d,())) for lo,hi in loops): continue
        new[j]=f'{tj[0]} r{d} '+' '.join(srcs); dele.add(i); remap[t]=d
        used.update([d,t]+regs)
    if not new: return lines, False
    out=[]
    for i,ln in enumerate(lines):
        if i in dele: continue
        ln=new.get(i,ln)
        if not ln.endswith(':'):
            toks=ln.split()
            for k in range(1,len(toks)):
                m=_REGTOK.match(toks[k])
                if m and int(m.group(2)) in remap: toks[k]=f'{m.group(1)}r{remap[int(m.group(2))]}{m.group(3) or ""}'
            ln=' '.join(toks)
        out.append(ln)
    return out, True

def _const_of(lines, wr, r, pool):
    """the constant register r holds (written once, by an LDP of the pool), or None"""
    w=wr.get(r,())
    if len(w)!=1: return None
    t=lines[w[0]].split()
    if t[0]!='LDP' or len(t)!=3 or not t[2].startswith('@') or not t[2][1:].isdigit(): return None
    return tuple(pool[int(t[2][1:])])

def _setp_fold(lines, sv, pool):
    """SETP SNE of a SET's 0 / 1 against 0 is the SET's comparison: `SETP cc, a, b` (the same compare, NaN included).
    SETP SNE of 1 - x against 0, x a SET's 0 / 1, is SETP SEQ x, 0; and SETP SEQ of a SET SEQ / SNE against 0 the other
    one (equality has an exact complement, ordering does not: a NaN fails both). All in one block, nothing written between."""
    wr,rd,first,last=_tab(lines); changed=False; out=list(lines)
    def zero(tok):
        m=_REGTOK.match(tok)
        return m is not None and not m.group(1) and _const_of(lines, wr, int(m.group(2)), pool)==(0.0,0.0,0.0,0.0)
    def between_ok(j, i, regs):
        return not any(_ends_block(lines[k]) or _dst(lines[k].split()) in regs for k in range(j+1,i))
    def set_line(r):
        w=wr.get(r,())
        if len(w)!=1: return None
        t=lines[w[0]].split()
        if t[0]!='SET' or not _plain(t[2]): return None
        return w[0], t
    def splat(tok, comp):
        m=_REGTOK.match(tok); r=int(m.group(2))
        return f'{m.group(1)}r{r}' if r in sv else f'{m.group(1)}r{r}.{"xyzw"[_swz4(m.group(3))[comp]]}'
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split()
        if toks[0]!='SETP' or len(toks)!=4 or not zero(toks[3]): continue
        m=_REGTOK.match(toks[2])
        if not m or m.group(1): continue
        a=int(m.group(2)); s0=_swz4(m.group(3))[0]
        w=wr.get(a,())
        if len(w)!=1 or w[0]>=i: continue
        j=w[0]; tj=lines[j].split()
        if toks[1]=='SNE' and tj[0]=='SUB' and len(tj)==4 and _plain(tj[1]):
            mo=_REGTOK.match(tj[2]); mx=_REGTOK.match(tj[3])
            if not mo or mo.group(1) or _const_of(lines, wr, int(mo.group(2)), pool)!=(1.0,1.0,1.0,1.0) or not mx or mx.group(1): continue
            x=int(mx.group(2))
            s=set_line(x)
            if s is None or s[0]>=j or not between_ok(s[0], i, (x,)): continue
            out[i]=f'SETP SEQ {splat(tj[3], s0)} {toks[3]}'; changed=True
        elif toks[1] in ('SNE','SEQ') and tj[0]=='SET' and _plain(tj[2]):
            if toks[1]=='SEQ' and tj[1] not in ('SEQ','SNE'): continue
            regs=[_regnum(tj[3]), _regnum(tj[4])]
            if None in regs or not between_ok(j, i, set(regs)): continue
            cc=tj[1] if toks[1]=='SNE' else ('SNE' if tj[1]=='SEQ' else 'SEQ')
            out[i]=f'SETP {cc} {splat(tj[3], s0)} {splat(tj[4], s0)}'; changed=True
    return out, changed

def _else_drop(lines):
    """`ELSE L` straight to `L:` and ENDIF, L jumped to by nothing else: the ELSE runs nothing (ENDIF restores the lanes the
    ELSE would have set, and its jump lands on the ENDIF)"""
    refs={}
    for ln in lines:
        if not ln.endswith(':'):
            for t in ln.split()[1:]: refs[t]=refs.get(t,0)+1
    dele=set()
    for i,ln in enumerate(lines):
        t=ln.split()
        if t[0]=='ELSE' and len(t)==2 and i+2<len(lines) and lines[i+1]==t[1]+':' and lines[i+2]=='ENDIF' and refs.get(t[1])==1:
            dele.update((i,i+1))
    if not dele: return lines, False
    return [ln for i,ln in enumerate(lines) if i not in dele], True

_NARROW = {'ADD','SUB','MUL','MAD','MIN','MAX','MOV','RCP','RSQRT','LOG2','EXP2','SIN','COS','FLR','CEIL','TRUNC','RNDE','SQRT','RCPM','RSQM','CLAMP'}

def _exposed(lines, r, lo, hi):
    """is register r read at the top of the loop lo..hi before it is written there: a read (or a masked write, which keeps
    the other lanes) before a whole write at the loop's own level (not under an IF or an inner loop, so every lane still in
    the loop makes it). Not exposed, no value of r goes round the loop: the next pass writes it before it reads it. A `LOOP 1`
    (a function's early return, _flow_open) has no next pass."""
    if lines[lo].split()[1:2]==['1']: return False
    depth=0
    for k in range(lo+1,hi):
        ln=lines[k]
        if ln.endswith(':'): continue
        toks=ln.split(); base=toks[0].split('.')[0]
        if r in _srcs(toks): return True
        if _dst(toks)==r:
            if '.' in toks[0] or depth: return True
            return False
        if base in ('IF','LOOP'): depth+=1
        elif base in ('ENDIF','ENDL'): depth-=1
    return False

def narrow_masks(lines, sv):
    """after _mad_fuse (a masked MUL fuses no more): a component-wise write loses the components nothing reads before they
    are written again (vec3 and vec2 work is done in vec4 registers: a micro-op a component). dead_stores' accounting going
    backwards through each block, a later read anywhere keeping a register whole; a read counts the components its micro-ops
    read (_read_mask), so a narrowed reader frees what its sources computed for it. A read earlier in a loop counts as a read
    after the block only when the register goes round the loop (_exposed: a vec3 worked out afresh every pass is not
    written whole)."""
    loops=_loop_regions(lines); reads={}
    for idx,ln in enumerate(lines):
        if ln.endswith(':'): continue
        for r in _srcs(ln.split()): reads.setdefault(r,[]).append(idx)
    def read_from(r, end):
        v=reads.get(r,())
        return bool(v) and (v[-1]>=end or any(lo<=end<=hi and any(lo<=x<end for x in v) and _exposed(lines, r, lo, hi) for lo,hi in loops))
    out=list(lines); over={}; end=len(lines)
    def ov(r):
        if r not in over: over[r]=0 if read_from(r,end) else 15
        return over[r]
    for i in range(len(lines)-1,-1,-1):
        ln=lines[i]
        if ln.endswith(':'): continue
        toks=ln.split(); base=toks[0].split('.')[0]
        if base in _BLOCK_END: over={}; end=i
        dp=_dst_pos(toks)
        if dp is not None:
            d=_regnum(toks[dp]); wm=15 if d in sv else _wmask(toks); o=ov(d); nm=wm&~o
            if nm and nm!=wm and d not in sv and base in _NARROW and not _hasswz(toks[dp]):
                toks[0]=base+'.'+''.join('xyzw'[c] for c in range(4) if nm>>c&1); out[i]=' '.join(toks)
            over[d]=o|wm
        for k in range(1,len(toks)):
            if k==dp: continue
            m=_REGTOK.match(toks[k])
            if m:
                r=int(m.group(2)); over[r]=ov(r)&~_read_mask(toks,k)
    return out

def shape_opt(lines, svregs, pool):
    sv=set(svregs or ())
    while True:
        lines,a=_coalesce(lines, sv)
        lines,b=_mask_fold(lines, sv)
        lines,c=_setp_fold(lines, sv, pool)
        lines,d=_else_drop(lines)
        lines,e=_copy_back(lines, sv)
        if not (a or b or c or d or e): return dce(lines)

def _read_pos(toks):
    """The positions of a source's swizzle this instruction reads. Micro-op c of an instruction reads its sources at
    swizzle[c], for each c of the write mask; a DP reads the first 3 or 4 whatever it writes, a TEX its coordinate set
    (the trailing .xy). Anything else (STO, SET, SETP ...) counts as the whole swizzle."""
    base=toks[0].split('.')[0]
    if base=='DP3': return 7
    if base=='TEX': return sum(1<<'xyzw'.index(ch) for ch in toks[-1][1:]) if toks[-1].startswith('.') else 15
    if base!='DP4' and base in _ALU_DST: return _wmask(toks)
    return 15

def _read_mask(toks, k):
    """The components of the register in token k that this instruction reads (_read_pos through its swizzle)."""
    q=_swz4(_REGTOK.match(toks[k]).group(3)); pos=_read_pos(toks)
    return sum(1<<c for c in {q[p] for p in range(4) if pos>>p&1})

def ldv_masks(lines, svregs):
    """A fragment shader's LDV writes only the components something reads: a user varying costs two chain micro-ops per
    component, and gl_ff.c's `vec4 vTc` is read as .xy. The reads of the whole program count, so control
    flow and registers defined more than once need no care: a component no instruction reads is not observable."""
    sv=set(svregs or ()); read={}
    for ln in lines:
        if ln.endswith(':'): continue
        toks=ln.split(); dp=_dst_pos(toks)
        for k in range(1,len(toks)):
            if k==dp: continue
            m=_REGTOK.match(toks[k])
            if m: read[int(m.group(2))]=read.get(int(m.group(2)),0)|_read_mask(toks,k)
    out=[]
    for ln in lines:
        toks=ln.split()
        if toks[0]=='LDV' and _regnum(toks[1]) not in sv and read.get(_regnum(toks[1]),15)!=15:
            m=read.get(_regnum(toks[1]))
            ln=' '.join(['LDV.'+''.join('xyzw'[c] for c in range(4) if m>>c&1)]+toks[1:])
        out.append(ln)
    return out

def cse(lines, skip_ldp=False, ldv_tex=False):
    """Common-subexpression elimination within a basic block: if two full-write instructions have identical
    (op, sources) and none of those sources (nor the first dest) is redefined between, redirect the second's
    result to the first and delete it. Skips masked writes (they merge old lanes) and side-effect ops.
    skip_ldp=True leaves constant loads (LDP) un-merged — under constant rematerialization, merging them back
    into one load per basic block would re-create the long-lived constant registers remat exists to avoid.
    ldv_tex=True (forward, a fragment program): an LDV looks past a TEX for its twin. Two varyings of one row
    (Appendix A.7, the link packs two coordinates so) are two LDVs of it, and a TEX usually stands between them."""
    lines=list(lines); changed=True
    while changed:
        changed=False
        defc={}                                                # definition count per reg (full+masked writes)
        for ln in lines:
            if ln.endswith(':'): continue
            d=_dst(ln.split())
            if d is not None: defc[d]=defc.get(d,0)+1
        chosen=[]; used=set()                                  # batch of register-disjoint redirects (CSE is confluent)
        for i in range(len(lines)):
            li=lines[i]
            if li.endswith(':'): continue
            ti=li.split(); bi=ti[0].split('.')[0]
            if skip_ldp and bi=='LDP': continue                # keep rematerialized constants short-lived
            if bi in _SIDE or bi in _CTRL or '.' in ti[0]: continue
            di=_dst(ti)
            if di is None or defc.get(di)!=1: continue         # reused value must be SSA-stable (never reassigned)
            if di in used: continue
            key=_rhs(ti); srcs=set(_srcs(ti)); hit=None
            for k in range(i+1,len(lines)):
                lk=lines[k]
                if lk.endswith(':'): continue
                tk=lk.split(); bk=tk[0].split('.')[0]
                if bk in _CTRL and not (ldv_tex and bk=='TEX' and bi=='LDV'): break   # basic-block boundary
                dk=_dst(tk)
                if dk in srcs or dk==di: break                 # a source (or di) got redefined -> stop
                if bk not in _SIDE and '.' not in tk[0] and dk is not None and defc.get(dk)==1 and _rhs(tk)==key:
                    hit=(k,dk); break                          # dk single-def -> safe to delete + redirect
            if hit and hit[1] not in used and hit[1]!=di:
                k,dk=hit; chosen.append((k,dk,di)); used.add(di); used.add(dk)
        if chosen:
            delset=set(k for (k,_,_) in chosen)
            for (k,dk,di) in chosen:
                for m in range(len(lines)):
                    if m in delset or lines[m].endswith(':'): continue
                    lines[m]=_replace_src(lines[m], dk, di)
            lines=[ln for m,ln in enumerate(lines) if m not in delset]; changed=True
    return lines

def _mad_signs(mn, temp_is_first, temp_mod):
    """MUL t,a,b then (ADD/SUB) d,X,Y  ->  MAD d,a,b',c. The MAD computes a*b + c but ONLY src1 (b) is
    modifiable (its rs2 addend occupies the src0-modifier bits) and the addend c must be strictly positive.
    Given the combining op, whether the temp t is the FIRST operand, and t's modifier prefix, return the
    NEGATE flag for the product (applied as a '-' on src1 b), or None if the pattern can't become a MAD."""
    if '!' in temp_mod:            # abs on the temp: a src1 |.| yields a*|b|, NOT |a*b| -> not representable
        return None
    neg = ('-' in temp_mod)        # an explicit negate already on the temp operand
    if mn == 'ADD':                # d = (+/-)a*b + c   (caller guarantees the OTHER operand c is +/unmodified)
        return neg
    if temp_is_first:              # SUB d,X,Y with temp = MINUEND: d = (+/-)a*b - c -> addend negative,
        return None                #   which the MAD addend can't encode -> not fusable
    return not neg                 # SUB with temp = SUBTRAHEND: d = c - (+/-)a*b -> flip the product sign

def mad_fuse(lines):
    """Fuse `MUL t,a,b` + `ADD d,t,c` -> `MAD d,a,b,c` (native fused multiply-add). Safe conditions: t is a
    plain temp used exactly twice (its MUL def + this ADD), the MUL is a full unmodified write, a/b/c carry
    no source modifiers, and a/b/c are not redefined between the MUL and the ADD (same basic block)."""
    lines=list(lines); changed=True
    while changed:
        changed=False; occ=_occ(lines)
        for i,ln in enumerate(lines):
            if ln.endswith(':'): continue
            toks=ln.split()
            if toks[0].split('.')[0]!='ADD' or len(toks)!=4: continue
            d,p,q=toks[1],toks[2],toks[3]; fused=False
            for tmp,other in ((p,q),(q,p)):
                if _hasmod(tmp) or _hasmod(other): continue
                tv=_regnum(tmp)
                if tv is None or occ.get(tv)!=2: continue
                j=i-1; a=b=None                                    # find the MUL that defines tv, in-block
                while j>=0:
                    lj=lines[j]
                    if lj.endswith(':'): break
                    tj=lj.split()
                    if tj[0].split('.')[0] in _CTRL: break
                    if _dst(tj)==tv:
                        if tj[0]=='MUL' and len(tj)==4 and not _hasmod(tj[2]) and not _hasmod(tj[3]): a,b=tj[2],tj[3]
                        break
                    j-=1
                if a is None: continue
                an,bn,on=_regnum(a),_regnum(b),_regnum(other)      # operands must be stable j+1..i-1
                if any((not lines[k].endswith(':')) and _dst(lines[k].split()) in (an,bn,on) for k in range(j+1,i)):
                    continue
                mask=('.'+toks[0].split('.',1)[1]) if '.' in toks[0] else ''
                lines[i]=f'MAD{mask} {d} {a} {b} {other}'; del lines[j]
                changed=True; fused=True; break
            if fused: break
    return lines

def allocate_registers(lines, nphys=31, svregs=None):
    """Liveness-based linear-scan allocation of virtual registers onto r1..r{nphys}. Each virtual reg's
    live interval = [first mention, last mention] on the linear instruction stream (loops are unrolled
    and IF/ELSE execute both masked paths, so linear liveness is exact here). Non-overlapping intervals
    share a physical register; a masked-write accumulator keeps one physical reg across its whole range.
    svregs (isa64): the SCALAR virtual registers — each gets one component of a physical register (four
    scalars with overlapping lifetimes share a register), and `_rewrite_packed` turns their mentions into
    single-component destinations / splat sources (a scalar micro-op costs 1/4 of a vec4 one)."""
    first={}; last={}; labelpos={}
    for i,ln in enumerate(lines):
        if ln.endswith(':'): labelpos[ln[:-1]]=i; continue
        for t in ln.split():
            m=_REGTOK.match(t)
            if m:
                v=int(m.group(2)); first.setdefault(v,i); last[v]=i
    # loop regions: a value live anywhere inside a loop must stay live to the loop end (back-edge / ENDL),
    # else its physical reg could be reused mid-loop and clobbered across iterations. VS back-edge = a BRA
    # to an earlier label; FS hardware loop = LOOP..ENDL.
    loops=[]; loopstk=[]
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        t=ln.split()
        if t[0]=='BRA' and t[1] in labelpos and labelpos[t[1]]<i: loops.append((labelpos[t[1]], i))
        elif t[0]=='LOOP': loopstk.append(i)
        elif t[0]=='ENDL' and loopstk: loops.append((loopstk.pop(), i))
    for lo,hi in loops:
        for v in first:
            # loop-carried values (defined before the loop, used inside) must survive every back-edge;
            # per-iteration temps (defined inside) are re-created each pass, so they don't need extending.
            if first[v]<=lo and last[v]>=lo: last[v]=max(last[v], hi)
    order=sorted(first, key=lambda v:(first[v], last[v]))
    packed=set(svregs) if svregs else set()
    active=[]                                  # (end, phys, comp | None, vreg)
    freep=list(range(nphys,0,-1))              # r{nphys}..r1 ; pop() picks the lowest
    partial={}                                 # phys -> mask of its FREE components (scalar-packed registers)
    mapping={}; comp={}
    def release(ph, cp):
        if cp is None: freep.append(ph); return
        partial[ph]|=1<<cp
        if partial[ph]==0xF: del partial[ph]; freep.append(ph)
    for v in order:
        s=first[v]
        keep=[]
        for e,ph,cp,av in active:
            if e < s: release(ph, cp)          # interval ended -> physical (or its component) returns to pool
            else: keep.append((e,ph,cp,av))
        active=keep
        cp=None
        if v in packed:
            ph=next((q for q in sorted(partial) if partial[q]), None)   # a register with a free component first
            if ph is None:
                if not freep:
                    raise CompileError(f"register spill needed at instruction {s} ({len(active)+1} live > {nphys}) "
                                       f"— shader exceeds {nphys} live registers (split it or reduce temporaries)")
                freep.sort(reverse=True); ph=freep.pop(); partial[ph]=0xF
            cp=next(c for c in range(4) if (partial[ph]>>c)&1); partial[ph]&=~(1<<cp); comp[v]=cp
        else:
            if not freep:
                raise CompileError(f"register spill needed at instruction {s} ({len(active)+1} live > {nphys}) "
                                   f"— shader exceeds {nphys} live registers (split it or reduce temporaries)")
            freep.sort(reverse=True)
            ph=freep.pop()
        mapping[v]=ph; active.append((last[v], ph, cp, v))
    if packed: return _rewrite_packed(lines, mapping, comp)
    def rw(t):
        m=_REGTOK.match(t)
        return f'{m.group(1)}r{mapping[int(m.group(2))]}{m.group(3) or ""}' if m else t
    return [ln if ln.endswith(':') else ' '.join(rw(t) for t in ln.split()) for ln in lines]

def _dst_pos(toks):
    base=toks[0].split('.')[0]
    if base=='SET': return 2
    if base in _ALU_DST: return 1
    return None

def _rewrite_packed(lines, mapping, comp):
    """Physical names with the scalar packing applied: a packed scalar `rV` becomes `rP.c` as a destination (one
    component written = one micro-op; an op-level mask on it is dropped) and `rP.c` (the splat) as a source; its
    lanes are equal by construction, so any splat / single-letter swizzle on it reads the same value. A scalar
    load (LDP / LDV) reads `.x` of its slot, the component every host-side packer fills. Anything else (a
    multi-component mask or a mixed swizzle on a scalar) is a code-generator bug and raises."""
    out=[]
    for ln in lines:
        if ln.endswith(':'): out.append(ln); continue
        toks=ln.split(); dp=_dst_pos(toks); opmask=toks[0].split('.')[1] if '.' in toks[0] else ''
        new=list(toks); dst_packed=False
        for i,t in enumerate(toks):
            m=_REGTOK.match(t)
            if not m: continue
            v=int(m.group(2)); mods=m.group(1); suf=(m.group(3) or '')[1:]; p=mapping[v]
            if v not in comp: new[i]=f'{mods}r{p}' + (f'.{suf}' if suf else ''); continue
            c='xyzw'[comp[v]]
            if i==dp:
                if len(suf)>1 or (opmask and len(opmask)>1):
                    raise CompileError(f"packed scalar r{v} written with a multi-component mask: {ln}")
                new[i]=f'r{p}.{c}'; dst_packed=True
            else:
                new[i]=f'{mods}r{p}.{c}'                    # any swizzle of a scalar: v.gba = vec3(0.0) after copy_prop
        if dst_packed and opmask: new[0]=toks[0].split('.')[0]      # the destination carries its component
        if dst_packed and toks[0] in ('LDP','LDV','LDPX') and '.' not in new[2]: new[2]=new[2]+'.x'
        out.append(' '.join(new))
    return out

def sfu_fuse(lines, sv):
    """xops bit 0: `RCP t, x` (or RSQRT) then `MUL d, y, t`, t a scalar read only there and d a scalar: RCPM d, x, y (RSQM), one
    SFU micro-op for two, bit for bit (isa64 RCPM / RSQM). Only a scalar result: N components of an RCPM are N SFU micro-ops, and the
    SFU runs about a tenth of the issue rate (two in flight), where one RCP feeding N MULs is one; an RCP that feeds several MULs, or
    that LICM took out of a loop, stays. Same block, x not written between, neither register read swizzled where it is written."""
    wr,rd,first,last=_tab(lines)
    new={}; dele=set(); used=set()
    for i,ln in enumerate(lines):
        if ln.endswith(':'): continue
        toks=ln.split()
        if toks[0]!='MUL' or len(toks)!=4 or not _plain(toks[1]): continue
        d=_regnum(toks[1])
        if d not in sv: continue
        for k,ok in ((3,2),(2,3)):
            if not _plain(toks[k]): continue
            t=_regnum(toks[k]); y=toks[ok]
            if t not in sv or t==d or t in used or d in used or len(wr.get(t,()))!=1 or len(rd.get(t,()))!=1 or _regnum(y)==t: continue
            j=wr[t][0]
            if j>=i or j in new: continue
            tj=lines[j].split()
            if tj[0] not in ('RCP','RSQRT') or len(tj)!=3 or not _plain(tj[1]): continue
            x=tj[2]; xr=_regnum(x)
            if xr is None or xr in used or _regnum(y) in used: continue
            if (xr==d and _hasswz(x)) or (_regnum(y)==d and _hasswz(y)): continue
            if any(_ends_block(lines[q]) or _dst(lines[q].split())==xr for q in range(j+1,i)): continue
            new[i]=f'{"RCPM" if tj[0]=="RCP" else "RSQM"} r{d} {x} {y}'; dele.add(j)
            used.update([d,t,xr,_regnum(y)]); break
    if not new: return lines
    return [new.get(i,ln) for i,ln in enumerate(lines) if i not in dele]

def mad_fuse_alloc(lines, fuse=True, isa64=False, svregs=None, narrow=False, xsfu=False):
    """Allocate physical registers, THEN fuse a `MUL t,a,b` feeding an `ADD`/`SUB` into a native fused
    multiply-add. Done post-alloc so the addend can be constrained to r0-r7 (MAD's rs2 field is only 3 bits,
    ins[10:8]; the 64-bit ISA has a full addend field and every source swizzled / modifiable, so isa64=True
    drops that limit and only requires the MUL's result to be read unswizzled). Patterns fused (like the Mesa NIR
    ffma peephole, but at the ISA level):
        MUL t,a,b ; ADD d,t,c      -> MAD d, a,  b, c
        MUL t,a,b ; ADD d,-t,c     -> MAD d, a, -b, c     (negated temp)
        MUL t,a,b ; SUB d,c,t      -> MAD d, a, -b, c     (temp is the subtrahend: c - a*b)
    Only src1 (b) can carry a modifier and the addend must be a positive r0-r7, so `a*b - c` (negative
    addend) and an abs'd temp are left un-fused. Fuses only when: t is a full unmodified MUL consumed solely
    by this op (no read between, dead after) and a/b are not rewritten in between. fuse=False just allocates
    (used to A/B-verify that fusion is semantics-preserving). isa64: the fusion runs BEFORE allocation on the
    virtual registers (no addend constraint there), so that the scalar packing's component names never confuse
    the register matching; allocation (with the packing) follows."""
    if isa64:
        lines=_mad_fuse(lines, isa64) if fuse else lines
        if xsfu: lines=sfu_fuse(lines, set(svregs or ()))           # xops bit 0: RCP / RSQRT + a scalar MUL -> RCPM / RSQM
        if narrow: lines=narrow_masks(lines, set(svregs or ()))      # forward=True: masks to what is read
        return allocate_registers(lines, svregs=svregs)
    lines=allocate_registers(lines)
    if not fuse: return lines
    return _mad_fuse(lines, isa64)

def _mad_fuse(lines, isa64):
    lines=list(lines)
    changed=True
    while changed:                                             # re-pass until no fusion (cascades); each pass is a
        changed=False; i=0                                     # single left-to-right sweep, not a per-fuse restart
        while i < len(lines):
            ln=lines[i]
            mn0 = '' if ln.endswith(':') else ln.split()[0].split('.')[0]
            if mn0 not in ('ADD','SUB') or len(ln.split())!=4: i+=1; continue
            toks=ln.split(); d,p,q=toks[1],toks[2],toks[3]; fused=False
            for tmp,other,temp_first in ((p,q,True),(q,p,False)):
                if _hasmod(other) and not isa64: continue       # 32-bit: addend must be positive/unmodified
                tm=_REGTOK.match(tmp)
                if not tm or tm.group(3): continue               # a swizzled read of the product is not a*b
                pneg=_mad_signs(mn0, temp_first, tm.group(1))    # product-negate flag, or None if not fusable
                if pneg is None: continue
                oc=_regnum(other); tv=int(tm.group(2))
                if oc is None or (oc>7 and not isa64): continue # 32-bit: MAD rs2 (addend) must be r0-r7
                if oc==tv: continue                             # t±t (= 2ab / 0) is not a*b+c
                j=i-1; a=b=None; ok=True                        # find the MUL def of tv within the block
                while j>=0:
                    lj=lines[j]
                    if lj.endswith(':'): ok=False; break
                    tj=lj.split()
                    if tj[0].split('.')[0] in _CTRL: ok=False; break
                    if tv in _srcs(tj): ok=False; break         # tmp read before the combine -> not single-use
                    if _dst(tj)==tv:
                        if tj[0]=='MUL' and len(tj)==4 and not _hasmod(tj[2]) and not _hasmod(tj[3]): a,b=tj[2],tj[3]
                        break
                    j-=1
                if not ok or a is None: continue
                an,bn=_regnum(a),_regnum(b)
                if any((not lines[k].endswith(':')) and _dst(lines[k].split()) in (an,bn) for k in range(j+1,i)):
                    continue
                dn=_regnum(d)                                   # the MAD reads a and b as it writes d: a swizzled d would see what
                if (an==dn and _hasswz(a)) or (bn==dn and _hasswz(b)): continue   # an earlier micro-op wrote (isa64.check_overlap)
                dead=True                                       # tmp must be dead after the combine
                for k in range(i+1,len(lines)):
                    if lines[k].endswith(':'): continue
                    tk=lines[k].split()
                    if tv in _srcs(tk): dead=False; break
                    if _dst(tk)==tv:
                        if isa64 and '.' in tk[0]: dead=False    # virtual registers: a masked write keeps tmp's other lanes
                        break
                if not dead: continue
                bt = f'-{b}' if pneg else b                      # product sign -> src1 (b) negate modifier
                mask=('.'+toks[0].split('.',1)[1]) if '.' in toks[0] else ''
                lines[i]=f'MAD{mask} {d} {a} {bt} {other}'; del lines[j]  # j<i -> our line shifts to i-1;
                changed=True; fused=True; break                          # old i+1 is now at index i -> don't ++i
            if not fused: i+=1
    return lines

# --------------- top-level program build ---------------
# The cores' program memories hold 4096 instructions; board A takes a vertex program of up to 4096 (VS_PROGRAM refuses more), board
# B's draw state gives a fragment program's length in 12 bits, so 4095 at most
PROG_MAX={'vs':4096, 'fs':4095}

def compile_stage(src, stage, optimize=True, remat_consts=False, hw_sincos=True, fuse=True, isa64=False, forward=False, unread=0, strict=False, pack=None, xops=0, noops=0, attrs=None):
    # Auto-fallback: if a shader exceeds the 31-register file with the default (constant-pinning) allocation,
    # retry with constant rematerialization (fresh LDP per use) — trades a few more instructions for far lower
    # register pressure, letting heavy shaders (e.g. the Seascape) compile transparently through g.program().
    # hw_sincos: emit the SIN/COS SFU ops (False -> software Taylor).  fuse: enable MUL+ADD/SUB -> MAD fusion.
    # isa64: the 64-bit swizzling ISA (board B's scalar core; assemble the .sasm with tools/isa64.py).
    # forward: block-local copy forwarding + dead stores + LDV write masks + shape_opt (isa64 only), what host/win/shc always does.
    # unread: a mask of the varying slots the linked fragment stage never reads: their STVs go, and with them what only they needed.
    # strict: every error the language requires (a program from an application); False: the libraries' own programs and the benches.
    # pack: Appendix A.7, for a program that does not fit one row a variable: {'umap': uniform slot -> row * 4 + lane, 'urows': the rows they take, 'vmap': the same of a varying slot, -1 = dropped}.
    # xops: board B's fragment-core operations, as its STATUS b_flags bits 28 / 29 say (>> 28): 1 SQRT (length, distance, sqrt) and RCPM / RSQM (an RCP or
    # RSQRT feeding one scalar MUL), 2 CLAMP. Bit for bit the code they replace; a fragment program only (the vertex core has none), isa64 only.
    # noops: the units this stage's core lacks, as its STATUS flags bits 30 / 31 say (>> 30; board A's for the vertex stage, board
    # B's b_flags for the fragment stage): 1 the integer divider (int / becomes _idiv_lowered), 2 SIN / COS (_sincos_lowered); isa64.
    # attrs: a vertex stage's attribute bindings at a link, name -> location (place_attribs).
    # A spill retries first without the loop-invariant hoist (licm: what it takes out of a loop stays live across all of it, so a long
    # program can need more registers with it than without), then with the vectors of known components built a lane at a time
    # (fold_cvec=False: a folded one holds a register of its own to its last use, the lanes of the scalars it is built from are shared),
    # with and without the hoist, then with every constant rematerialized, without the hoist either: never a refusal that the code
    # before the hoist or the folding did not have.
    if not remat_consts:
        for fold, hoist in ((True, True), (True, False), (False, True), (False, False)):
            try:
                return _compile_stage(src, stage, optimize, remat_consts=False, hw_sincos=hw_sincos, fuse=fuse, isa64=isa64, forward=forward, unread=unread, strict=strict, pack=pack, xops=xops, hoist=hoist, noops=noops, attrs=attrs, fold_cvec=fold)
            except CompileError as e:
                if 'register spill' not in str(e): raise
    return _compile_stage(src, stage, optimize, remat_consts=True, hw_sincos=hw_sincos, fuse=fuse, isa64=isa64, forward=forward, unread=unread, strict=strict, pack=pack, xops=xops, hoist=False, noops=noops, attrs=attrs)

# Attribute locations (§2.10.4): an attribute the program reads is active and takes a location, a matrix one a column, in a run; the
# others leave the table. attrs (a link's glBindAttribLocation table, name -> location) puts a bound one at its location, two of them
# also at one (aliasing: no path reads both); the rest, in declaration order, take the lowest run no active one holds. The code reads
# them there; the table keeps the active ones in declaration order. Past 16 locations the assembler refuses the LDV, a link the program.
def place_attribs(code, attribs, attrs):
    read=set()
    for ln in code:
        t=ln.split(' ')
        if t[0].split('.')[0]=='LDV': read.add(int(t[2].split('.')[0]))
    act=[(n,i,ty,ncomp2(ty) if is_mat(ty) else 1) for n,(i,ty) in attribs.items()]
    act=[a for a in act if any(a[1]+k in read for k in range(a[3]))]
    loc={}; held=set()
    for n,i,ty,w in act:
        if attrs and n in attrs: loc[n]=attrs[n]; held.update(range(attrs[n], attrs[n]+w))
    for n,i,ty,w in act:
        if n in loc: continue
        at=0
        while any(at+k in held for k in range(w)): at+=1
        loc[n]=at; held.update(range(at, at+w))
    new={i+k:loc[n]+k for n,i,ty,w in act for k in range(w)}
    def moved(ln):
        t=ln.split(' ')
        if t[0].split('.')[0]!='LDV': return ln
        m=re.match(r'(\d+)(.*)$', t[2]); t[2]=f'{new[int(m.group(1))]}{m.group(2)}'
        return ' '.join(t)
    return [moved(ln) for ln in code], {n:(loc[n],ty) for n,i,ty,w in act}

def _compile_stage(src, stage, optimize=True, remat_consts=False, hw_sincos=True, fuse=True, isa64=False, forward=False, unread=0, strict=False, pack=None, xops=0, hoist=True, noops=0, attrs=None, fold_cvec=True):
    exts=set()
    src=preprocess(src, stage, exts, strict)     # #define / #ifdef / #if ... + comment stripping
    toks=lex(src, exts); parser=Parser(toks, strict); ast=parser.parse()
    sem=Sema(stage, parser, exts, strict); sem.run(ast)
    c=Compiler(stage); c.remat_consts=remat_consts; c.fold_cvec=fold_cvec; c.hw_sincos=hw_sincos; c.isa64=isa64; c.callmap=sem.callmap; c.ints=sem.ints
    c.xops=xops&3 if isa64 and stage=='fs' else 0
    if isa64 and noops&1: c.hw_idiv=False
    if isa64 and noops&2: c.hw_sincos=False
    if pack: c.umap=pack.get('umap'); c.vmap=pack.get('vmap'); c.depth_units=pack.get('depth',0)
    # pass 0: register struct type definitions (needed to size struct uniforms below)
    for d in parser.hoisted+ast:
        if d[0]=='struct': c.structs[d[1]]=d[2]
    for k,v in GL_MAX.items(): c.const_env[k]=IntLit(v)
    # pass 1: globals (attributes/uniforms/varyings)
    for d in ast:
        if d[0]=='gdecl':
            _,qual,ty,name,arr,init=d
            if qual=='attribute': c.attribs[name]=(c.next_attr,ty); c.next_attr+=ncomp2(ty) if is_mat(ty) else 1
            elif qual in ('uniform','global'):                                # 'global' = bare struct instance
                if ty in SAMPLERS:
                    c.samplers[name]=c.next_sampler                           # texture unit (no uniform slot)
                    if arr is not None: c.sampler_sizes[name]=int(c.fold_const(arr))
                    c.next_sampler+=c.sampler_sizes.get(name,1)
                elif ty in c.structs:                                         # uniform struct (+ array of structs)
                    asz=c._arrsize(arr); n=c._type_nslots(ty, asz)
                    c.uniforms[name]=(c.next_uniform,ty,asz) if asz is not None else (c.next_uniform,ty)
                    c._struct_samplers(name, ty, asz, c.next_uniform)
                    c.next_uniform+=n
                else:
                    per=ncomp2(ty) if is_mat(ty) else 1
                    if arr is not None:                                       # uniform array: N contiguous slots
                        n=int(c.fold_const(arr)); c.uniforms[name]=(c.next_uniform,ty,n); c.next_uniform+=per*n
                    else:
                        c.uniforms[name]=(c.next_uniform,ty); c.next_uniform+=per
            elif qual=='varying':
                asz=c._arrsize(arr)
                c.varyings[name]=(c.next_vary,ty,asz) if asz is not None else (c.next_vary,ty)
                c.next_vary+=(ncomp2(ty) if is_mat(ty) else 1)*(asz or 1)
            elif qual=='const':                                           # global constant (ES2: initializer required)
                if init is None: raise CompileError(f"global const {name!r} needs an initializer")
                cv=c.fold_const(init)
                if cv is not None and ty in SCALAR: c.const_env[name]=IntLit(int(cv)) if ty=='int' else cv   # scalar -> compile-time (array sizes, loop bounds)
                else: c.global_consts[name]=(ty, init)                      # vec/mat/struct const -> materialize on reference
    if sem.depthrange:                                                    # the built-in uniform (§7.5): the library fills it
        c.structs[DEPTHRANGE[0]]=DEPTHRANGE[1]
        c.uniforms['gl_DepthRange']=(c.next_uniform,DEPTHRANGE[0]); c.next_uniform+=3
    # collect user-defined functions (inlined at call sites) + find main
    main=None
    for d in ast:
        if d[0]=='func':
            if d[2]=='main': main=d
            else: c.funcs[sem.fnames.get(id(d),d[2])]=c.funcs[d[2]]=(d[1], d[3], d[4])    # name -> (return_type, params, body)
    if main is None: raise CompileError("no main()")
    c.locals.append(Scope())
    c.scan_body(main[4])                # pre-pass: vars assigned under runtime if/else can't be const-tracked
    for _fn in c.funcs.values(): c.scan_body(_fn[2])   # scan inlined function bodies too
    c.shadow_varyings(main[4])
    for d in ast:                       # a function called under an if writes a global under the if: never a tracked constant
        if d[0]=='gdecl' and d[1]=='var':
            g=('ldecl', False, d[2], d[3], d[5], d[4])
            c.cond_decls.add(id(g)); c.run_stmt(g)
    c.run_main(main[4])
    c.flush_varyings()
    c.emit('END')
    if unread: c.code=[ln for ln in c.code if not (ln.startswith('STV ') and unread>>int(ln.split()[1])&1)]
    if optimize:
        code = strip_dead_bra(c.code)                   # 0. drop no-op tail-return branches (inlining)
        if forward and isa64: code = drop_labels(code)  # 0b. and the labels nothing jumps to any more
        code = dce(code)                                # 1. dead-code elimination (drops unused temps)
        code = copy_prop(code)                          # 2. copy propagation (remove identity MOVs)
        if forward and isa64: code = forward_opt(code, c.svregs)   # 2b. the same for registers defined more than once
        code = cse(code, skip_ldp=remat_consts, ldv_tex=forward and isa64 and stage=='fs')   # 3. common-subexpression elimination (keep remat LDPs split)
        code = dce(code)                                # 4. final DCE (clean up freed temps)
        if forward and isa64 and stage=='fs': code = ldv_masks(code, c.svregs)   # 4b. a varying's unread components
        if not remat_consts: code = hoist_consts(code)  # 5. hoist cached const loads above branches and loops [correctness];
        if forward and isa64 and hoist: code = licm(code)   # 5a. what a hardware loop works out the same every pass, before it
        code = hoist_tex(code, interleave=forward and isa64)   # remat re-loads at each use (already convergent) -> no hoist; forward: TEX slices interleaved
        if forward and isa64: code = shape_opt(code, c.svregs, c.const_order)   # 5c. merged copies, masked results, direct SETPs, no empty ELSE
        code = mad_fuse_alloc(code, fuse=fuse, isa64=isa64, svregs=c.svregs if isa64 else None, narrow=forward and isa64, xsfu=bool(c.xops&1))   # 5b/6. hoist TEX; fuse MUL+ADD/SUB -> MAD (+ RCPM / RSQM, narrow) + allocate (packed scalars on isa64)
    else:
        code = strip_dead_bra(list(c.code))
        if not remat_consts: code = hoist_consts(code)      # correctness passes (needed even unoptimized)
        code = allocate_registers(code, svregs=c.svregs if isa64 else None)
    base=pack['urows'] if pack and c.umap is not None else c.next_uniform   # 3. resolve @N constant markers -> uniform slots
    # Only the constants the final code loads take a parameter vector, renumbered in their order: a constant whose load
    # the optimizer removed, or an unrolled loop's index nobody reads, would otherwise stay in the pool (a 60-pass loop would
    # keep 60 vectors for one load — of the fragment core's 64, the uniforms included)
    used=sorted({int(n) for ln in code for n in re.findall(r'@(\d+)', ln)})
    # A constant whose four lanes are one value, loaded by LDP only, takes one lane of a row it shares with three others: LDP's
    # swizzle picks it (a scalar reads .x of its slot, a vector its splat). The rows left over go to the uniforms
    other={int(n) for ln in code if not ln.startswith('LDP ') for n in re.findall(r'@(\d+)', ln)}
    lane=[o for o in used if o not in other and c.const_order[o][0]==c.const_order[o][1]==c.const_order[o][2]==c.const_order[o][3]]
    whole=[o for o in used if o not in set(lane)]
    pool=[list(c.const_order[o]) for o in whole]
    slot={o:(i,None) for i,o in enumerate(whole)}
    for i,o in enumerate(lane):
        if i%4==0: pool.append([0.0]*4)
        pool[-1][i%4]=c.const_order[o][0]; slot[o]=(len(pool)-1, i%4)
    def at(m):
        r,k=slot[int(m.group(1))]
        if k is None: return f'{base+r}{m.group(2) or ""}'
        return f'{base+r}.' + 'xyzw'[k]*(len(m.group(2))-1 if m.group(2) else 4)
    out=[re.sub(r'@(\d+)(\.[xyzw]+)?', at, ln) for ln in code]
    attribs=c.attribs
    if stage=='vs': out, attribs = place_attribs(out, c.attribs, attrs)
    n=sum(1 for ln in out if ln.split('//')[0].split(';')[0].strip() and not ln.strip().endswith(':'))
    if isa64 and n>PROG_MAX[stage]:
        raise CompileError(f"the program has {n} instructions: the {'vertex' if stage=='vs' else 'fragment'} core runs at most {PROG_MAX[stage]}")
    return {'sasm':'\n'.join(out), 'attribs':attribs, 'uniforms':c.uniforms, 'samplers':c.samplers, 'sampler_sizes':c.sampler_sizes,
            'varyings':c.varyings, 'const_base':base, 'const_pool':pool, 'isa64':isa64, 'vs_tex':c.vs_tex}

if __name__=='__main__':
    files=[a for a in sys.argv[1:] if not a.startswith('--')]
    path=files[0]; src=open(path).read()
    stage='fs' if (path.endswith('.frag') or path.endswith('.fs') or '--fs' in sys.argv) else 'vs'
    r=compile_stage(src, stage, remat_consts=('--remat' in sys.argv), isa64=('--isa64' in sys.argv))
    print(f"// stage={stage}  attribs={r['attribs']}  uniforms={r['uniforms']}  varyings={r['varyings']}")
    print(f"// const pool @ uniform[{r['const_base']}..]: {r['const_pool']}")
    print(r['sasm'])
