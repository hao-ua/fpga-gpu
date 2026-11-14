# build.ps1 — the Windows GPU libraries and their tests, x86 and x64.
# Toolchain: the MinGW-w64 GCC 8.3.0 that ships inside Vivado (Model_Composer\tps\mingw\8.3.0): it has multilib (-m32) and the
# ddraw / d3d9 / GL headers, so nothing is installed. No make in that tree: this script is the build.
#   powershell -File host/win/build.ps1 [-Arch x64,x86] [-Target core,tests,...] [-Clean]
param([string[]]$Arch = @('x64', 'x86'), [string[]]$Target = @('all'), [switch]$Clean)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$mingw = 'C:\AMDDesignTools\2026.1\Model_Composer\tps\mingw\8.3.0\win64.o\nt'
if ($env:GPU2_MINGW) { $mingw = $env:GPU2_MINGW }
if (-not (Test-Path "$mingw\bin\gcc.exe")) { throw "no gcc at $mingw (set GPU2_MINGW)" }
$env:Path = "$mingw\bin;$env:Path"
$Arch = @($Arch | ForEach-Object { $_ -split ',' })
$Target = @($Target | ForEach-Object { $_ -split ',' })
$cflags = @('-std=gnu11', '-O2', '-g', '-Wall', '-Wno-unused-function', '-Wno-maybe-uninitialized', '-D__USE_MINGW_ANSI_STDIO=1', "-I$root\core")
$cxxflags = @('-std=gnu++17', '-O2', '-g', '-Wall', '-Wno-unused-function', '-fno-exceptions', '-fno-rtti', "-I$root\core")
$syslibs = @('-lws2_32', '-lwinmm')

function Want($t) { ($Target -contains 'all') -or ($Target -contains $t) }

function Compile($arch, $src, $out, $flags, $cxx) {
    New-Item -ItemType Directory -Force (Split-Path $out) | Out-Null
    if ((Test-Path $out) -and ((Get-Item $out).LastWriteTime -gt (Get-Item $src).LastWriteTime) -and ((Get-Item $out).LastWriteTime -gt $script:hdr)) { return }
    $m = if ($arch -eq 'x86') { '-m32' } else { '-m64' }
    $tool = if ($cxx) { 'g++.exe' } else { 'gcc.exe' }
    & "$mingw\bin\$tool" $m @flags -c $src -o $out
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $src" }
}

function Link($arch, $objs, $out, $flags) {
    $m = if ($arch -eq 'x86') { '-m32' } else { '-m64' }
    & "$mingw\bin\g++.exe" $m -static-libgcc -static-libstdc++ @objs -o $out @flags
    if ($LASTEXITCODE -ne 0) { throw "link failed: $out" }
}

foreach ($a in $Arch) {
    $b = "$root\build\$a"
    if ($Clean -and (Test-Path $b)) { Remove-Item -Recurse -Force $b }
    New-Item -ItemType Directory -Force $b | Out-Null
    # an object older than the newest header is compiled again (the tree is small). Per object, not one stamp written at the end: a
    # stamp would leave objects of the old header behind when a header changes while a build runs, or after a build of other targets.
    # 10 s more for a compile that had read the header before it changed
    $script:hdr = (Get-ChildItem "$root\core", "$root\tests", "$root\d3d9", "$root\d3d8", "$root\gl", "$root\gles", "$root\shc", "$root\ddraw" -Recurse -Filter *.h -ErrorAction SilentlyContinue | Sort-Object LastWriteTime -Descending | Select-Object -First 1).LastWriteTime.AddSeconds(10)
    $core = @()
    foreach ($f in 'g2_core', 'g2_util', 'g2_env', 'g2_render', 'g2_dxt') {
        Compile $a "$root\core\$f.c" "$b\core\$f.o" $cflags $false
        $core += "$b\core\$f.o"
    }
    if (Want 'ddraw') {
        New-Item -ItemType Directory -Force "$b\ddraw" | Out-Null
        Compile $a "$root\ddraw\ddraw_main.c" "$b\ddraw\ddraw_main.o" ($cflags + "-I$root\ddraw") $false
        Link $a (@("$b\ddraw\ddraw_main.o") + $core + "$root\ddraw\ddraw.def") "$b\ddraw\ddraw.dll" (@('-shared', '-static', '-Wl,--enable-stdcall-fixup') + $syslibs + @('-lgdi32', '-luser32', '-lole32', '-luuid'))
        Compile $a "$root\tests\dd_test.c" "$b\ddraw\dd_test.o" ($cflags + "-I$root\tests") $false
        # linked against our own ddraw.dll, found beside the .exe at run time: the way a game gets it
        Link $a @("$b\ddraw\dd_test.o", "$b\ddraw\ddraw.dll") "$b\ddraw\dd_test.exe" @('-Wl,--enable-stdcall-fixup', '-ldxguid', '-lgdi32', '-luser32')
    }
    if ((Want 'gl') -and (Test-Path "$root\shc\g2_shc.h")) {
        $inc = @("-I$root\gl", "-I$root\shc")
        $o = @()
        foreach ($f in 'gl_api', 'gl_dev', 'gl_draw', 'gl_eval', 'gl_ext', 'gl_ff', 'gl_list', 'gl_pixel', 'gl_select', 'gl_state', 'gl_stubs_gen', 'gl_tex', 'gl_tex3d', 'gl_glsl', 'gl_vsrun') {
            Compile $a "$root\gl\$f.c" "$b\gl\$f.o" ($cflags + $inc + '-Wno-implicit-function-declaration') $false
            $o += "$b\gl\$f.o"
        }
        foreach ($f in (Get-ChildItem "$root\shc" -Filter *.c -File)) {
            Compile $a $f.FullName "$b\shc\$($f.BaseName).o" ($cflags + '-Wno-unused-variable' + $(if ($a -eq 'x86') { @('-msse2', '-mfpmath=sse') } else { @() })) $false
            $o += "$b\shc\$($f.BaseName).o"
        }
        Link $a ($o + $core + "$root\gl\opengl32.def") "$b\gl\opengl32.dll" (@('-shared', '-static', '-Wl,--enable-stdcall-fixup') + $syslibs + @('-lgdi32', '-luser32'))
        # linked against the system's import library, our opengl32.dll found beside the .exe at run time: the way a game gets it
        foreach ($t in 'gl_test', 'gl_scissor_test', 'gl_far_test', 'gl_tex_cow_test', 'gl_many_tex_test') {
            Compile $a "$root\tests\$t.c" "$b\gl\$t.o" $cflags $false
            Link $a @("$b\gl\$t.o") "$b\gl\$t.exe" @('-lopengl32', '-lgdi32', '-luser32')
        }
        # the fixed-function programs of chosen states, with no board: the library's objects linked in
        Compile $a "$root\tests\ff_dump.c" "$b\gl\ff_dump.o" ($cflags + $inc) $false
        Link $a (@("$b\gl\ff_dump.o") + $o + $core) "$b\gl\ff_dump.exe" (@('-static') + $syslibs + @('-lgdi32', '-luser32'))
    }
    if ((Want 'gles') -and (Test-Path "$root\shc\g2_shc.h")) {
        $inc = @("-I$root\shc")
        $o = @()
        foreach ($f in 'gles_main', 'gles_stubs_gen') {
            Compile $a "$root\gles\$f.c" "$b\gles\$f.o" ($cflags + $inc) $false
            $o += "$b\gles\$f.o"
        }
        foreach ($f in (Get-ChildItem "$root\shc" -Filter *.c -File)) {
            Compile $a $f.FullName "$b\shc\$($f.BaseName).o" ($cflags + '-Wno-unused-variable' + $(if ($a -eq 'x86') { @('-msse2', '-mfpmath=sse') } else { @() })) $false
            $o += "$b\shc\$($f.BaseName).o"
        }
        Link $a ($o + $core + "$root\gles\libGLESv2.def") "$b\gles\libGLESv2.dll" (@('-shared', '-static', '-Wl,--enable-stdcall-fixup') + $syslibs + @('-lgdi32', '-luser32'))
        # libEGL.dll holds no code: its exports forward to libGLESv2.dll, ordinals as ANGLE's
        Compile $a "$root\gles\egl_empty.c" "$b\gles\egl_empty.o" $cflags $false
        Link $a @("$b\gles\egl_empty.o", "$root\gles\libEGL.def") "$b\gles\libEGL.dll" @('-shared', '-static', '-Wl,--enable-stdcall-fixup')
        # linked against both DLLs, found beside the .exe at run time: the way a game gets them
        foreach ($t in 'gles_test', 'gles_shader') {      # gles_shader: a fragment shader file over the whole window
            Compile $a "$root\tests\$t.c" "$b\gles\$t.o" $cflags $false
            Link $a @("$b\gles\$t.o", "$b\gles\libEGL.dll", "$b\gles\libGLESv2.dll") "$b\gles\$t.exe" @('-Wl,--enable-stdcall-fixup,--allow-multiple-definition', '-lgdi32', '-luser32')     # egl*: libEGL.dll comes first, so its forwarders are the ones called
        }
    }
    if ((Want 'd3d9') -and (Test-Path "$root\shc\g2_shc.h")) {
        $o = @()
        foreach ($f in 'd3d9_main', 'd3d9_dev', 'd3d9_res', 'd3d9_ff', 'd3d9_sm', 'd3d9_cpu', 'd3d9_cursor', 'd3d9_rec') {
            Compile $a "$root\d3d9\$f.c" "$b\d3d9\$f.o" ($cflags + "-I$root\shc" + "-I$root\d3d9") $false
            $o += "$b\d3d9\$f.o"
        }
        foreach ($f in (Get-ChildItem "$root\shc" -Filter *.c -File)) {
            Compile $a $f.FullName "$b\shc\$($f.BaseName).o" ($cflags + '-Wno-unused-variable' + $(if ($a -eq 'x86') { @('-msse2', '-mfpmath=sse') } else { @() })) $false
            $o += "$b\shc\$($f.BaseName).o"
        }
        Link $a ($o + $core + "$root\d3d9\d3d9.def") "$b\d3d9\d3d9.dll" (@('-shared', '-static', '-Wl,--enable-stdcall-fixup') + $syslibs + @('-lgdi32', '-luser32', '-lole32', '-luuid'))
        # linked against our d3d9.dll, found beside the .exe at run time: the way a game gets it
        Compile $a "$root\tests\d3d_test.c" "$b\d3d9\d3d_test.o" ($cflags + "-I$root\tests") $false
        Link $a @("$b\d3d9\d3d_test.o", "$b\d3d9\d3d9.dll") "$b\d3d9\d3d_test.exe" @('-Wl,--enable-stdcall-fixup', '-ldxguid', '-lgdi32', '-luser32')
        # the depth resolution of far geometry; a plain Direct3D 9 program, so away from our d3d9.dll it measures the PC's adapter
        Compile $a "$root\tests\d3d_far_test.c" "$b\d3d9\d3d_far_test.o" ($cflags + "-I$root\tests") $false
        Link $a @("$b\d3d9\d3d_far_test.o", "$b\d3d9\d3d9.dll") "$b\d3d9\d3d_far_test.exe" @('-Wl,--enable-stdcall-fixup', '-ldxguid', '-lgdi32', '-luser32')
        # textures changed after draws that sampled them, in the same scene; also a plain Direct3D 9 program
        Compile $a "$root\tests\d3d_tex_cow_test.c" "$b\d3d9\d3d_tex_cow_test.o" ($cflags + "-I$root\tests") $false
        Link $a @("$b\d3d9\d3d_tex_cow_test.o", "$b\d3d9\d3d9.dll") "$b\d3d9\d3d_tex_cow_test.exe" @('-Wl,--enable-stdcall-fixup', '-ldxguid', '-lgdi32', '-luser32')
        if ($a -eq 'x86') {     # plays what d3d9_rec.c recorded; the file holds d3state as x86 lays it out
            Compile $a "$root\tests\d3_replay.c" "$b\d3d9\d3_replay.o" ($cflags + "-I$root\shc" + "-I$root\d3d9") $false
            Link $a @("$b\d3d9\d3_replay.o", "$b\d3d9\d3d9.dll") "$b\d3d9\d3_replay.exe" @('-Wl,--enable-stdcall-fixup', '-ldxguid', '-lgdi32', '-luser32')
        }
    }
    # Direct3D 8 is 32-bit only: d3d9.dll's objects behind Direct3D 8's tables, d3d8_check.c compiled to prove the tables against d3d8.h
    if ((Want 'd3d8') -and ($a -eq 'x86') -and (Test-Path "$root\shc\g2_shc.h")) {
        $o = @()
        foreach ($f in 'd3d9_main', 'd3d9_dev', 'd3d9_res', 'd3d9_ff', 'd3d9_sm', 'd3d9_cpu', 'd3d9_cursor', 'd3d9_rec') {
            Compile $a "$root\d3d9\$f.c" "$b\d3d9\$f.o" ($cflags + "-I$root\shc" + "-I$root\d3d9") $false
            $o += "$b\d3d9\$f.o"
        }
        foreach ($f in (Get-ChildItem "$root\shc" -Filter *.c -File)) {
            Compile $a $f.FullName "$b\shc\$($f.BaseName).o" ($cflags + '-Wno-unused-variable' + @('-msse2', '-mfpmath=sse')) $false
            $o += "$b\shc\$($f.BaseName).o"
        }
        Compile $a "$root\d3d8\d3d8_check.c" "$b\d3d8\d3d8_check.o" ($cflags + "-I$root\d3d8") $false
        Compile $a "$root\d3d8\d3d8_main.c" "$b\d3d8\d3d8_main.o" ($cflags + "-I$root\shc" + "-I$root\d3d9" + "-I$root\d3d8") $false
        Link $a (@("$b\d3d8\d3d8_main.o") + $o + $core + "$root\d3d8\d3d8.def") "$b\d3d8\d3d8.dll" (@('-shared', '-static', '-Wl,--enable-stdcall-fixup') + $syslibs + @('-lgdi32', '-luser32', '-lole32', '-luuid'))
        # linked against our d3d8.dll, found beside the .exe at run time: the way a game gets it
        Compile $a "$root\tests\d3d8_test.c" "$b\d3d8\d3d8_test.o" $cflags $false
        Link $a @("$b\d3d8\d3d8_test.o", "$b\d3d8\d3d8.dll") "$b\d3d8\d3d8_test.exe" @('-Wl,--enable-stdcall-fixup', '-lgdi32', '-luser32')
    }
    if (Want 'tests') {
        foreach ($t in 'g2_parity', 'g2_cube', 'g2_soak') {
            Compile $a "$root\tests\$t.c" "$b\tests\$t.o" ($cflags + "-I$root\tests") $false
            Link $a (@("$b\tests\$t.o") + $core) "$b\$t.exe" $syslibs
        }
        Compile $a "$root\tests\fake_board.c" "$b\tests\fake_board.o" ($cflags + "-I$root\tests") $false
        Compile $a "$root\tests\g2_loss_test.c" "$b\tests\g2_loss_test.o" ($cflags + "-I$root\tests") $false
        Link $a (@("$b\tests\g2_loss_test.o", "$b\tests\fake_board.o") + $core) "$b\g2_loss_test.exe" $syslibs
        Compile $a "$root\tests\g2_mode_test.c" "$b\tests\g2_mode_test.o" ($cflags + "-I$root\tests") $false
        Link $a (@("$b\tests\g2_mode_test.o", "$b\tests\fake_board.o") + $core) "$b\g2_mode_test.exe" $syslibs
        if (($a -eq 'x86') -and (Test-Path "$b\d3d8\d3d8.dll")) {     # Release's counts as Direct3D 8 games rely on them, against fake_board; loads the d3d8.dll beside it once GPU2_IP points there
            Compile $a "$root\tests\d3d8_ref_test.c" "$b\tests\d3d8_ref_test.o" ($cflags + "-I$root\tests") $false
            Link $a @("$b\tests\d3d8_ref_test.o", "$b\tests\fake_board.o") "$b\d3d8\d3d8_ref_test.exe" (@($syslibs) + @('-lgdi32', '-luser32'))
        }
    }
    Write-Output "built $a"
}
