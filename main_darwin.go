//go:build darwin

package main

import (
	"fmt"
	"os"
)

func main() {
	fmt.Fprintln(os.Stderr, "simple-traffic-simulation does not support macOS")
	os.Exit(1)
}
