# Copies the current dev build into .\play so it can be run while the code keeps changing.
# Usage: pwsh tools\snapshot.ps1   then run .\play\joyreach.exe
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$play = Join-Path $root 'play'
New-Item -ItemType Directory -Force $play | Out-Null
Copy-Item (Join-Path $root 'target\debug\joyreach.exe'), (Join-Path $root 'target\debug\bevy_dylib.dll') $play -Force
# dynamic_linking: the exe also needs Rust's std DLL.
Copy-Item (Join-Path (rustc --print target-libdir) 'std-*.dll') $play -Force
Copy-Item (Join-Path $root 'assets') $play -Recurse -Force
"snapshot ready: $play\joyreach.exe"
