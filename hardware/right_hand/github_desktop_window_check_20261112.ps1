$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
$apps = @(Get-Process -Name GitHubDesktop -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 })
foreach ($app in $apps) {
    Write-Output ('WINDOW ' + $app.Id + ' ' + $app.MainWindowTitle)
    $root = [System.Windows.Automation.AutomationElement]::FromHandle($app.MainWindowHandle)
    $queue = New-Object System.Collections.Queue
    $queue.Enqueue(@($root,0))
    $seen=0
    while ($queue.Count -gt 0 -and $seen -lt 160) {
        $item=$queue.Dequeue();$element=$item[0];$depth=$item[1];$seen++
        try {
            $name=$element.Current.Name;$kind=$element.Current.ControlType.ProgrammaticName
            if ($name -and $name.Length -lt 350) { Write-Output ($kind + ' ' + $name) }
            if ($depth -lt 6 -and $kind -notin @('ControlType.List','ControlType.Table','ControlType.DataGrid')) {
                $children=$element.FindAll([System.Windows.Automation.TreeScope]::Children,[System.Windows.Automation.Condition]::TrueCondition)
                for ($i=0;$i -lt [Math]::Min($children.Count,35);$i++) {$queue.Enqueue(@($children[$i],$depth+1))}
            }
        } catch {}
    }
}
if ($apps.Count -eq 0) { Write-Output 'NO_DESKTOP_WINDOW' }
$preparing=@(Get-CimInstance Win32_Process -Filter "Name = 'python.exe'" | Where-Object { $_.CommandLine -like '*prepare_github_desktop_workspace_20261112.py*' })
foreach ($process in $preparing) { Write-Output ('PREPARATION_RUNNING_PID ' + $process.ProcessId) }
