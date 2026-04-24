//go:build !darwin

package main

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"

	rl "github.com/gen2brain/raylib-go/raylib"
)

// MapTile is an immutable background tile loaded from a .tif/.asc pair.
// Position and size are expressed in simulation (raylib) coordinates.
type MapTile struct {
	DemPath   string
	OrthoPath string
	Texture   rl.Texture2D
	LeftX     float32
	TopY      float32
	WidthM    float32
	HeightM   float32
}

type ascHeader struct {
	NCols     int
	NRows     int
	XLLCenter float64
	YLLCenter float64
	CellSize  float64
	LLIsCorner bool
}

func parseAscHeader(path string) (*ascHeader, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 1024*1024), 64*1024*1024)
	h := &ascHeader{}
	seen := 0
	for seen < 6 && sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" {
			continue
		}
		parts := strings.Fields(line)
		if len(parts) < 2 {
			break
		}
		key := strings.ToLower(parts[0])
		val := parts[1]
		switch key {
		case "ncols":
			h.NCols, _ = strconv.Atoi(val)
		case "nrows":
			h.NRows, _ = strconv.Atoi(val)
		case "xllcenter":
			h.XLLCenter, _ = strconv.ParseFloat(val, 64)
		case "xllcorner":
			h.XLLCenter, _ = strconv.ParseFloat(val, 64)
			h.LLIsCorner = true
		case "yllcenter":
			h.YLLCenter, _ = strconv.ParseFloat(val, 64)
		case "yllcorner":
			h.YLLCenter, _ = strconv.ParseFloat(val, 64)
			h.LLIsCorner = true
		case "cellsize":
			h.CellSize, _ = strconv.ParseFloat(val, 64)
		case "nodata_value":
			// ignored
		default:
			// header ended
			return h, nil
		}
		seen++
	}
	return h, nil
}

// findSmallestIFDIndex walks a classic TIFF's IFD chain and returns the
// zero-based index of the smallest sub-image, plus its width and height.
func findSmallestIFDIndex(path string) (int, uint64, uint64, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return 0, 0, 0, err
	}
	if len(data) < 8 {
		return 0, 0, 0, fmt.Errorf("tiff too small")
	}
	var order binary.ByteOrder
	switch string(data[0:2]) {
	case "II":
		order = binary.LittleEndian
	case "MM":
		order = binary.BigEndian
	default:
		return 0, 0, 0, fmt.Errorf("not a tiff file")
	}
	if order.Uint16(data[2:4]) != 42 {
		return 0, 0, 0, fmt.Errorf("unsupported tiff (BigTIFF not supported)")
	}
	off := order.Uint32(data[4:8])
	bestIdx := -1
	var bestW, bestH uint64
	idx := 0
	visited := map[uint32]bool{}
	for off != 0 && !visited[off] {
		visited[off] = true
		if int(off)+2 > len(data) {
			break
		}
		count := order.Uint16(data[off:])
		entriesStart := int(off) + 2
		if entriesStart+int(count)*12+4 > len(data) {
			break
		}
		var w, h uint64
		for i := 0; i < int(count); i++ {
			e := entriesStart + i*12
			tag := order.Uint16(data[e:])
			typ := order.Uint16(data[e+2:])
			val := data[e+8 : e+12]
			readNum := func() uint64 {
				switch typ {
				case 1:
					return uint64(val[0])
				case 3:
					return uint64(order.Uint16(val))
				case 4:
					return uint64(order.Uint32(val))
				}
				return 0
			}
			if tag == 256 {
				w = readNum()
			} else if tag == 257 {
				h = readNum()
			}
		}
		if w > 0 && h > 0 {
			// Prefer the smallest IFD whose longer side is at least minLong
			// pixels, so we pick a usable-resolution overview rather than a
			// tiny thumbnail, while avoiding the full-resolution image that
			// can easily exceed GPU texture limits.
			const minLong uint64 = 2048
			longW := w
			if h > longW {
				longW = h
			}
			longBest := bestW
			if bestH > longBest {
				longBest = bestH
			}
			candidate := longW >= minLong
			bestOk := longBest >= minLong
			if bestIdx == -1 {
				bestIdx = idx
				bestW, bestH = w, h
			} else if candidate && !bestOk {
				bestIdx = idx
				bestW, bestH = w, h
			} else if candidate && bestOk && w*h < bestW*bestH {
				bestIdx = idx
				bestW, bestH = w, h
			} else if !candidate && !bestOk && w*h > bestW*bestH {
				bestIdx = idx
				bestW, bestH = w, h
			}
		}
		off = order.Uint32(data[entriesStart+int(count)*12:])
		idx++
	}
	if bestIdx == -1 {
		return 0, 0, 0, fmt.Errorf("no IFDs found")
	}
	return bestIdx, bestW, bestH, nil
}

// convertTiffSubimageToPng uses ImageMagick's `convert` to extract the sub-image
// at `index` from a classic TIFF into `outPath` as PNG. We shell out because
// Go's x/image/tiff does not support YCbCr+JPEG compressed tiles commonly used
// in aerial orthophoto TIFFs.
func convertTiffSubimageToPng(tifPath string, index int, outPath string) error {
	src := fmt.Sprintf("%s[%d]", tifPath, index)
	cmd := exec.Command("convert", src, outPath)
	cmd.Stderr = os.Stderr
	return cmd.Run()
}

type mapJson struct {
	Version      int                    `json:"version"`
	Name         string                 `json:"name"`
	Simulation   string                 `json:"simulation,omitempty"`
	RaylibCenter struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	} `json:"raylib_center"`
	Tiles []struct {
		Dem   string `json:"dem"`
		Ortho string `json:"ortho"`
	} `json:"tiles"`
}

// loadMapFormat reads a map.json and its referenced tiles. Returns the loaded
// tiles, the raylib centre (in EPSG metric coords), the name of the
// referenced simulation file (empty if none), and any error.
func loadMapFormat(mapJsonPath string) (tiles []MapTile, centerX, centerY float64, simName string, mapName string, err error) {
	data, err := os.ReadFile(mapJsonPath)
	if err != nil {
		return nil, 0, 0, "", "", err
	}
	var m mapJson
	if err := json.Unmarshal(data, &m); err != nil {
		return nil, 0, 0, "", "", err
	}
	dir := filepath.Dir(mapJsonPath)
	centerX = m.RaylibCenter.X
	centerY = m.RaylibCenter.Y
	simName = m.Simulation
	mapName = m.Name

	for _, t := range m.Tiles {
		ascPath := filepath.Join(dir, t.Dem)
		tifPath := filepath.Join(dir, t.Ortho)
		h, aerr := parseAscHeader(ascPath)
		if aerr != nil {
			err = fmt.Errorf("parse %s: %w", t.Dem, aerr)
			return
		}
		idx, _, _, ferr := findSmallestIFDIndex(tifPath)
		if ferr != nil {
			err = fmt.Errorf("inspect %s: %w", t.Ortho, ferr)
			return
		}
		cachePath := filepath.Join(dir, ".cache_"+strings.TrimSuffix(t.Ortho, filepath.Ext(t.Ortho))+".png")
		if _, statErr := os.Stat(cachePath); statErr != nil {
			if cerr := convertTiffSubimageToPng(tifPath, idx, cachePath); cerr != nil {
				err = fmt.Errorf("convert %s: %w (is ImageMagick's `convert` installed?)", t.Ortho, cerr)
				return
			}
		}
		tex := rl.LoadTexture(cachePath)
		if tex.ID == 0 {
			err = fmt.Errorf("failed to load cached texture %s", cachePath)
			return
		}

		widthM := float64(h.NCols) * h.CellSize
		heightM := float64(h.NRows) * h.CellSize
		var leftEpsg, topEpsg float64
		if h.LLIsCorner {
			leftEpsg = h.XLLCenter
			topEpsg = h.YLLCenter + heightM
		} else {
			leftEpsg = h.XLLCenter - h.CellSize/2
			topEpsg = h.YLLCenter - h.CellSize/2 + heightM
		}
		leftR := float32(leftEpsg - centerX)
		topR := float32(-(topEpsg - centerY))

		tiles = append(tiles, MapTile{
			DemPath:   ascPath,
			OrthoPath: tifPath,
			Texture:   tex,
			LeftX:     leftR,
			TopY:      topR,
			WidthM:    float32(widthM),
			HeightM:   float32(heightM),
		})
	}
	return
}

// updateMapJsonSimulationField rewrites map.json preserving all fields but
// setting the "simulation" field to simName.
func updateMapJsonSimulationField(mapJsonPath, simName string) error {
	data, err := os.ReadFile(mapJsonPath)
	if err != nil {
		return err
	}
	var raw map[string]interface{}
	if err := json.Unmarshal(data, &raw); err != nil {
		return err
	}
	raw["simulation"] = simName
	out, err := json.MarshalIndent(raw, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(mapJsonPath, out, 0644)
}

func drawMapTiles(tiles []MapTile) {
	for _, t := range tiles {
		if t.Texture.ID == 0 {
			continue
		}
		srcRec := rl.NewRectangle(0, 0, float32(t.Texture.Width), float32(t.Texture.Height))
		dstRec := rl.NewRectangle(t.LeftX, t.TopY, t.WidthM, t.HeightM)
		rl.DrawTexturePro(t.Texture, srcRec, dstRec, rl.NewVector2(0, 0), 0, rl.White)
	}
}
