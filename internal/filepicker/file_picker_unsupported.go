//go:build !linux && !windows

package filepicker

import (
	"fmt"
	"runtime"
)

func pickPlatformSplineFilePath(save bool, defaultPath string) (string, error) {
	return "", fmt.Errorf("built-in file picker is unsupported on %s", runtime.GOOS)
}
