package geometry

import (
	"math"

	"github.com/golang/freetype/truetype"
	"golang.org/x/image/math/fixed"
)

// Point2D represents a flattened 2D vector coordinate.
type Point2D struct {
	X, Y float64
}

// Point3D represents a 3D vertex in the mesh.
type Point3D struct {
	X, Y, Z float64
}

// Triangle represents a single 3D facet in STL coordinate space.
type Triangle struct {
	Normal     Point3D
	V1, V2, V3 Point3D
}

// Generate3DText converts a string into a slice of extruded 3D triangles.
func Generate3DText(text string, font *truetype.Font, fontSize float64, depth float64) []Triangle {
	var allTriangles []Triangle
	var cursorX float64

	scale := fixed.Int26_6(fontSize * 64)
	glyphBuf := &truetype.GlyphBuf{}

	for _, ch := range text {
		index := font.Index(ch)
		if err := glyphBuf.Load(font, scale, index, nil); err != nil {
			continue
		}

		// 1. Convert FreeType Bézier contours into 2D polygon loops
		contours := extractContours(glyphBuf, cursorX)

		// 2. Extrude each contour loop into 3D prisms (front cap, back cap, walls)
		for _, contour := range contours {
			if len(contour) < 3 {
				continue
			}
			triangles := extrudeContour(contour, depth)
			allTriangles = append(allTriangles, triangles...)
		}

		// Advance cursor for next character
		advance := float64(font.HMetric(scale, index).AdvanceWidth) / 64.0
		cursorX += advance
	}

	return allTriangles
}

// extractContours samples quadratic Bézier curves into linear 2D segment loops.
func extractContours(gb *truetype.GlyphBuf, offsetX float64) [][]Point2D {
	var contours [][]Point2D
	start := 0

	for _, end := range gb.Ends {
		var loop []Point2D
		points := gb.Points[start:end]
		n := len(points)
		if n == 0 {
			start = end
			continue
		}

		for i := 0; i < n; i++ {
			curr := points[i]
			next := points[(i+1)%n]

			currX := float64(curr.X)/64.0 + offsetX
			currY := float64(curr.Y) / 64.0

			// On-curve point: append straight line
			if (curr.Flags & 1) != 0 {
				loop = append(loop, Point2D{X: currX, Y: currY})
			} else {
				// Off-curve control point: sample quadratic Bézier B(t) = (1-t)^2 P0 + 2(1-t)t P1 + t^2 P2
				prev := points[(i-1+n)%n]
				p0 := Point2D{X: float64(prev.X)/64.0 + offsetX, Y: float64(prev.Y) / 64.0}
				p1 := Point2D{X: currX, Y: currY}
				p2 := Point2D{X: float64(next.X)/64.0 + offsetX, Y: float64(next.Y) / 64.0}

				steps := 5
				for s := 1; s <= steps; s++ {
					t := float64(s) / float64(steps)
					it := 1.0 - t
					bx := it*it*p0.X + 2*it*t*p1.X + t*t*p2.X
					by := it*it*p0.Y + 2*it*t*p1.Y + t*t*p2.Y
					loop = append(loop, Point2D{X: bx, Y: by})
				}
			}
		}

		contours = append(contours, loop)
		start = end
	}

	return contours
}

// extrudeContour builds front/back caps via ear-clipping and connects side walls.
func extrudeContour(poly []Point2D, depth float64) []Triangle {
	var triangles []Triangle
	n := len(poly)

	// --- 1. Side Walls (Perimeter Quads -> 2 Triangles each) ---
	for i := 0; i < n; i++ {
		next := (i + 1) % n

		p1Back := Point3D{X: poly[i].X, Y: poly[i].Y, Z: 0}
		p2Back := Point3D{X: poly[next].X, Y: poly[next].Y, Z: 0}
		p1Front := Point3D{X: poly[i].X, Y: poly[i].Y, Z: depth}
		p2Front := Point3D{X: poly[next].X, Y: poly[next].Y, Z: depth}

		triangles = append(triangles,
			makeFacet(p1Back, p2Back, p2Front),
			makeFacet(p1Back, p2Front, p1Front),
		)
	}

	// --- 2. Front & Back Caps (Ear Clipping Triangulation) ---
	indices := triangulate2D(poly)
	for i := 0; i < len(indices); i += 3 {
		i1, i2, i3 := indices[i], indices[i+1], indices[i+2]

		// Front cap (Z = depth, counter-clockwise outward normal)
		triangles = append(triangles, makeFacet(
			Point3D{X: poly[i1].X, Y: poly[i1].Y, Z: depth},
			Point3D{X: poly[i2].X, Y: poly[i2].Y, Z: depth},
			Point3D{X: poly[i3].X, Y: poly[i3].Y, Z: depth},
		))

		// Back cap (Z = 0, clockwise reversed for downward outward normal)
		triangles = append(triangles, makeFacet(
			Point3D{X: poly[i1].X, Y: poly[i1].Y, Z: 0},
			Point3D{X: poly[i3].X, Y: poly[i3].Y, Z: 0},
			Point3D{X: poly[i2].X, Y: poly[i2].Y, Z: 0},
		))
	}

	return triangles
}

// triangulate2D implements basic ear-clipping for simple planar polygons.
func triangulate2D(pts []Point2D) []int {
	n := len(pts)
	idx := make([]int, n)
	for i := range idx {
		idx[i] = i
	}

	var result []int
	for len(idx) > 2 {
		found := false
		count := len(idx)
		for i := 0; i < count; i++ {
			prev := idx[(i-1+count)%count]
			curr := idx[i]
			next := idx[(i+1)%count]

			if isConvex(pts[prev], pts[curr], pts[next]) {
				result = append(result, prev, curr, next)
				idx = append(idx[:i], idx[i+1:]...)
				found = true
				break
			}
		}
		if !found {
			// Fallback fan if ear test terminates early on degenerate loop
			for j := 1; j < len(idx)-1; j++ {
				result = append(result, idx[0], idx[j], idx[j+1])
			}
			break
		}
	}
	return result
}

func isConvex(a, b, c Point2D) bool {
	return (b.X-a.X)*(c.Y-b.Y)-(b.Y-a.Y)*(c.X-b.X) > 0
}

// makeFacet creates a triangle facet and computes its unit normal vector via cross product.
func makeFacet(v1, v2, v3 Point3D) Triangle {
	ax, ay, az := v2.X-v1.X, v2.Y-v1.Y, v2.Z-v1.Z
	bx, by, bz := v3.X-v1.X, v3.Y-v1.Y, v3.Z-v1.Z

	nx := ay*bz - az*by
	ny := az*bx - ax*bz
	nz := ax*by - ay*bx

	lenVal := math.Sqrt(nx*nx + ny*ny + nz*nz)
	if lenVal > 0 {
		nx, ny, nz = nx/lenVal, ny/lenVal, nz/lenVal
	}

	return Triangle{
		Normal: Point3D{X: nx, Y: ny, Z: nz},
		V1:     v1,
		V2:     v2,
		V3:     v3,
	}
}
