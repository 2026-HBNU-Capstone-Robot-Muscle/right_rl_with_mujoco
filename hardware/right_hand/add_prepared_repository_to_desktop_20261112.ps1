$ErrorActionPreference='Stop'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
$app=Get-Process -Name GitHubDesktop | Where-Object { $_.MainWindowHandle -ne 0 } | Select-Object -First 1
if (-not $app) { throw 'No GitHub Desktop window' }
$root=[System.Windows.Automation.AutomationElement]::FromHandle($app.MainWindowHandle)
$scope=[System.Windows.Automation.TreeScope]::Descendants
$dialog=$root.FindFirst($scope,(New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty,'Add local repository')))
if (-not $dialog) { throw 'Expected Add local repository dialog' }
$edit=$dialog.FindFirst($scope,(New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::ControlTypeProperty,[System.Windows.Automation.ControlType]::Edit)))
if (-not $edit) { throw 'Expected Local path field' }
$path=Join-Path $PSScriptRoot 'github_desktop\right_rl_with_mujoco'
$value=$edit.GetCurrentPattern([System.Windows.Automation.ValuePattern]::Pattern)
if ($value.Current.Value -ne $path) { $value.SetValue($path) }
if ($value.Current.Value -ne $path) { throw 'Repository path was not set' }
$button=$dialog.FindFirst($scope,(New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty,'Add repository')))
if (-not $button -or -not $button.Current.IsEnabled) { throw 'Add repository button unavailable' }
$button.GetCurrentPattern([System.Windows.Automation.InvokePattern]::Pattern).Invoke()
Write-Output 'LOCAL_REPOSITORY_ADDED'
