$ErrorActionPreference='Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class DesktopWindowCapture {
 [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left,Top,Right,Bottom; }
 [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
 [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
 [DllImport("user32.dll")] public static extern bool IsIconic(IntPtr h);
 [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h,int n);
 [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h,IntPtr dc,uint flags);
 [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
}
'@
[void][DesktopWindowCapture]::SetProcessDPIAware()
$app=Get-Process -Name GitHubDesktop | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $app) { throw 'No GitHub Desktop window' }
$handle=$app.MainWindowHandle
if ([DesktopWindowCapture]::IsIconic($handle)) { [void][DesktopWindowCapture]::ShowWindow($handle,9) }
$rect=New-Object DesktopWindowCapture+RECT
[void][DesktopWindowCapture]::GetWindowRect($handle,[ref]$rect)
$width=$rect.Right-$rect.Left;$height=$rect.Bottom-$rect.Top
$bitmap=New-Object System.Drawing.Bitmap($width,$height)
$graphics=[System.Drawing.Graphics]::FromImage($bitmap)
$dc=$graphics.GetHdc()
try { $captured=[DesktopWindowCapture]::PrintWindow($handle,$dc,2) }
finally { $graphics.ReleaseHdc($dc) }
if (-not $captured) { throw 'Window-only capture failed' }
$output=Join-Path $PSScriptRoot 'github_desktop\desktop_connection.png'
$bitmap.Save($output,[System.Drawing.Imaging.ImageFormat]::Png)
$graphics.Dispose();$bitmap.Dispose()
Write-Output 'DESKTOP_WINDOW_CAPTURED'
