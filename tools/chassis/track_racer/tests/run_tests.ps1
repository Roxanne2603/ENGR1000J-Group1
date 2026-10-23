param([string]$OutputRoot = (Join-Path ([IO.Path]::GetTempPath()) ('engr100_policy_' + [guid]::NewGuid().ToString('N'))))
$ErrorActionPreference = 'Stop'
$taskSource = Split-Path $PSScriptRoot
$taskVswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
$taskVs = & $taskVswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (!$taskVs) { throw 'MSVC Build Tools with C++ is required for host tests.' }
$taskVc = Get-ChildItem (Join-Path $taskVs 'VC\Tools\MSVC') -Directory | Sort-Object Name -Descending | Select-Object -First 1
$taskSdk = Join-Path ${env:ProgramFiles(x86)} 'Windows Kits\10'
$taskSdkVer = Get-ChildItem (Join-Path $taskSdk 'Include') -Directory | Sort-Object Name -Descending | Select-Object -First 1
$taskCompiler = Join-Path $taskVc.FullName 'bin\Hostx64\x64\cl.exe'
foreach ($case in @('policy_tests','default_gate')) {
  $taskCase = Join-Path $OutputRoot $case
  New-Item -ItemType Directory -Path $taskCase -Force | Out-Null
  Copy-Item -LiteralPath (Join-Path $taskSource 'controller.h') -Destination $taskCase
  $taskConfig = if ($case -eq 'policy_tests') { Join-Path $PSScriptRoot 'fixture_config.h' } else { Join-Path $taskSource 'config.h' }
  Copy-Item -LiteralPath $taskConfig -Destination (Join-Path $taskCase 'config.h')
  Copy-Item -LiteralPath (Join-Path $PSScriptRoot "$case.cpp") -Destination $taskCase
  & $taskCompiler /nologo /EHsc /std:c++17 /W4 "/I$($taskVc.FullName)\include" "/I$($taskSdkVer.FullName)\ucrt" "/Fe:$taskCase\$case.exe" "/Fo:$taskCase\$case.obj" "$taskCase\$case.cpp" /link "/LIBPATH:$($taskVc.FullName)\lib\x64" "/LIBPATH:$taskSdk\Lib\$($taskSdkVer.Name)\ucrt\x64" "/LIBPATH:$taskSdk\Lib\$($taskSdkVer.Name)\um\x64"
  if ($LASTEXITCODE -ne 0) { throw "$case compilation failed" }
  & "$taskCase\$case.exe"
  if ($LASTEXITCODE -ne 0) { throw "$case assertions failed" }
}
