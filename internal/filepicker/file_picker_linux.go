//go:build linux

package filepicker

import "fmt"

func pickPlatformSplineFilePath(save bool, defaultPath string) (string, error) {
	type picker struct {
		name string
		run  func(bool, string) (string, error)
	}

	pickers := []picker{
		{name: "zenity", run: pickFileWithZenity},
		{name: "yad", run: pickFileWithYad},
		{name: "kdialog", run: pickFileWithKDialog},
	}

	for _, picker := range pickers {
		if _, err := lookPath(picker.name); err != nil {
			continue
		}
		return picker.run(save, defaultPath)
	}

	return "", fmt.Errorf("no supported Linux file dialog found; install zenity, yad, or kdialog")
}

func pickFileWithZenity(save bool, defaultPath string) (string, error) {
	args := []string{"--file-selection", "--title=Select spline file"}
	if save {
		args = append(args, "--save", "--confirm-overwrite")
	}
	args = append(args, "--filename="+defaultPath, "--file-filter=Supported files | *.json *.png *.jpg *.jpeg")
	return runDialogCommand("zenity", args...)
}

func pickFileWithYad(save bool, defaultPath string) (string, error) {
	args := []string{"--file", "--title=Select spline file"}
	if save {
		args = append(args, "--save", "--confirm-overwrite")
	}
	args = append(args, "--filename="+defaultPath, "--file-filter=Supported files | *.json *.png *.jpg *.jpeg")
	return runDialogCommand("yad", args...)
}

func pickFileWithKDialog(save bool, defaultPath string) (string, error) {
	filter := "Supported files (*.json *.png *.jpg *.jpeg)"
	if save {
		return runDialogCommand("kdialog", "--getsavefilename", defaultPath, filter)
	}
	return runDialogCommand("kdialog", "--getopenfilename", defaultPath, filter)
}
