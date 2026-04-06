package filepicker

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
)

func PickSplineFilePath(save bool) (string, error) {
	home, _ := os.UserHomeDir()
	startDir := home
	if startDir == "" {
		startDir = "."
	}
	defaultPath := filepath.Join(startDir, "splines.json")

	path, err := pickPlatformSplineFilePath(save, defaultPath)
	if err != nil {
		return "", err
	}
	return normalizePickedPath(path, save), nil
}

func runDialogCommand(name string, args ...string) (string, error) {
	cmd := exec.Command(name, args...)
	output, err := cmd.Output()
	if err != nil {
		if exitErr, ok := err.(*exec.ExitError); ok {
			code := exitErr.ExitCode()
			if code == 1 {
				return "", nil
			}
			stderr := strings.TrimSpace(string(exitErr.Stderr))
			if stderr != "" {
				return "", wrapDialogError(name, code, stderr)
			}
			return "", wrapDialogError(name, code, "")
		}
		return "", err
	}
	return strings.TrimSpace(string(output)), nil
}

func lookPath(name string) (string, error) {
	return exec.LookPath(name)
}

func normalizePickedPath(path string, save bool) string {
	path = strings.TrimSpace(path)
	path = strings.TrimPrefix(path, "file://")
	if path == "" {
		return ""
	}
	if save && filepath.Ext(path) == "" {
		path += ".json"
	}
	return path
}

func wrapDialogError(name string, code int, stderr string) error {
	if stderr != "" {
		return fmt.Errorf("%s: %s", name, stderr)
	}
	return fmt.Errorf("%s exited with code %d", name, code)
}
