//go:build windows

package filepicker

import (
	"fmt"
	"path/filepath"
	"strings"
)

func pickPlatformSplineFilePath(save bool, defaultPath string) (string, error) {
	shell, err := findPowerShell()
	if err != nil {
		return "", fmt.Errorf("no supported Windows file dialog host found; install PowerShell")
	}
	return runDialogCommand(shell, "-NoProfile", "-STA", "-Command", buildPowerShellDialogScript(save, defaultPath))
}

func findPowerShell() (string, error) {
	candidates := []string{"powershell", "powershell.exe", "pwsh", "pwsh.exe"}
	for _, name := range candidates {
		if _, err := lookPath(name); err == nil {
			return name, nil
		}
	}
	return "", fmt.Errorf("powershell not found")
}

func buildPowerShellDialogScript(save bool, defaultPath string) string {
	initialDir := filepath.Dir(defaultPath)
	defaultName := filepath.Base(defaultPath)
	dialogType := "OpenFileDialog"
	resultValue := "[System.Windows.Forms.DialogResult]::OK"
	config := []string{
		"$dialog.Filter = 'JSON files (*.json)|*.json'",
		"$dialog.InitialDirectory = '" + quotePowerShellString(initialDir) + "'",
		"$dialog.FileName = '" + quotePowerShellString(defaultName) + "'",
	}
	if save {
		dialogType = "SaveFileDialog"
		config = append(config,
			"$dialog.DefaultExt = 'json'",
			"$dialog.AddExtension = $true",
			"$dialog.OverwritePrompt = $true",
		)
	}

	lines := []string{
		"Add-Type -AssemblyName System.Windows.Forms",
		"$dialog = New-Object System.Windows.Forms." + dialogType,
	}
	lines = append(lines, config...)
	lines = append(lines,
		"if ($dialog.ShowDialog() -eq "+resultValue+") {",
		"  [Console]::Out.Write($dialog.FileName)",
		"}",
	)
	return strings.Join(lines, "\n")
}

func quotePowerShellString(value string) string {
	return strings.ReplaceAll(value, "'", "''")
}
