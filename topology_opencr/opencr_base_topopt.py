#!/usr/bin/env python3
"""2D topology optimization for an OpenCR mounting base.

The model is a plane-stress bracket: enclosure mounting holes are fixed and
loads are applied at the OpenCR mounting pads. The optimized mask can be
extruded to an STL for an initial physical prototype.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla


def parse_xy_list(value: str) -> list[tuple[float, float]]:
    """Parse "x,y;x,y" into a list of coordinates."""
    try:
        points = []
        for pair in value.split(";"):
            x, y = pair.split(",")
            points.append((float(x), float(y)))
        return points
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "coordinates must use the format 'x,y;x,y'"
        ) from exc


def lk(nu: float) -> np.ndarray:
    """Return the standard 4-node plane-stress element stiffness matrix."""
    k = np.array(
        [
            1 / 2 - nu / 6,
            1 / 8 + nu / 8,
            -1 / 4 - nu / 12,
            -1 / 8 + 3 * nu / 8,
            -1 / 4 + nu / 12,
            -1 / 8 - nu / 8,
            nu / 6,
            1 / 8 - 3 * nu / 8,
        ]
    )
    return 1 / (1 - nu**2) * np.array(
        [
            [k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7]],
            [k[1], k[0], k[7], k[6], k[5], k[4], k[3], k[2]],
            [k[2], k[7], k[0], k[5], k[6], k[3], k[4], k[1]],
            [k[3], k[6], k[5], k[0], k[7], k[2], k[1], k[4]],
            [k[4], k[5], k[6], k[7], k[0], k[1], k[2], k[3]],
            [k[5], k[4], k[3], k[2], k[1], k[0], k[7], k[6]],
            [k[6], k[3], k[4], k[1], k[2], k[7], k[0], k[5]],
            [k[7], k[2], k[1], k[4], k[3], k[6], k[5], k[0]],
        ]
    )


def build_edof(nelx: int, nely: int) -> np.ndarray:
    nodenrs = np.arange((nelx + 1) * (nely + 1)).reshape(
        (nely + 1, nelx + 1), order="F"
    )
    edof_vec = (2 * nodenrs[:-1, :-1] + 2).reshape(nelx * nely, order="F")
    offsets = np.array([0, 1, 2 * nely + 2, 2 * nely + 3, 2 * nely, 2 * nely + 1, -2, -1])
    return edof_vec[:, None] + offsets[None, :]


def build_filter(nelx: int, nely: int, rmin: float) -> tuple[sp.csr_matrix, np.ndarray]:
    rows: list[int] = []
    cols: list[int] = []
    vals: list[float] = []
    radius = int(math.ceil(rmin)) - 1
    for i in range(nelx):
        for j in range(nely):
            row = i * nely + j
            for k in range(max(i - radius, 0), min(i + radius + 1, nelx)):
                for l in range(max(j - radius, 0), min(j + radius + 1, nely)):
                    weight = rmin - math.sqrt((i - k) ** 2 + (j - l) ** 2)
                    if weight > 0:
                        rows.append(row)
                        cols.append(k * nely + l)
                        vals.append(weight)
    h = sp.coo_matrix((vals, (rows, cols)), shape=(nelx * nely, nelx * nely)).tocsr()
    return h, np.asarray(h.sum(axis=1)).ravel()


def element_centers(nelx: int, nely: int, cell_mm: float) -> tuple[np.ndarray, np.ndarray]:
    x = (np.arange(nelx) + 0.5) * cell_mm
    y = (np.arange(nely) + 0.5) * cell_mm
    return np.meshgrid(x, y, indexing="xy")


def circular_mask(
    xx: np.ndarray, yy: np.ndarray, points: list[tuple[float, float]], radius: float
) -> np.ndarray:
    mask = np.zeros_like(xx, dtype=bool)
    for x, y in points:
        mask |= (xx - x) ** 2 + (yy - y) ** 2 <= radius**2
    return mask


def nearest_nodes(
    points: list[tuple[float, float]], nelx: int, nely: int, cell_mm: float
) -> list[int]:
    nodes = []
    for x, y in points:
        ix = int(np.clip(round(x / cell_mm), 0, nelx))
        iy = int(np.clip(round(y / cell_mm), 0, nely))
        nodes.append(ix * (nely + 1) + iy)
    return nodes


def optimize(args: argparse.Namespace) -> tuple[np.ndarray, list[float]]:
    nelx = round(args.width / args.cell)
    nely = round(args.height / args.cell)
    if nelx < 4 or nely < 4:
        raise ValueError("domain must contain at least 4 x 4 elements")

    ndof = 2 * (nelx + 1) * (nely + 1)
    ke = lk(args.poisson)
    edof = build_edof(nelx, nely)
    ik = np.kron(edof, np.ones((8, 1))).ravel()
    jk = np.kron(edof, np.ones((1, 8))).ravel()
    h, hs = build_filter(nelx, nely, args.rmin)

    xx, yy = element_centers(nelx, nely, args.cell)
    board_holes = circular_mask(xx, yy, args.board_holes, args.hole_radius)
    frame_holes = circular_mask(xx, yy, args.frame_holes, args.hole_radius)
    holes = board_holes | frame_holes
    board_pads = circular_mask(xx, yy, args.board_holes, args.pad_radius)
    frame_pads = circular_mask(xx, yy, args.frame_holes, args.pad_radius)
    # Keep pads solid in the FEA model so center-node bolt loads and supports
    # transfer into the bracket. Hole bores are subtracted from exported geometry.
    solid = board_pads | frame_pads

    x = np.full(nelx * nely, args.volfrac)
    solid_flat = solid.reshape(-1, order="F")
    void_flat = np.zeros_like(solid_flat)
    x[solid_flat] = 1.0
    x[void_flat] = 0.0
    passive = solid_flat | void_flat
    design = ~passive

    f = np.zeros(ndof)
    load_nodes = nearest_nodes(args.board_holes, nelx, nely, args.cell)
    direction = np.array([args.load_x, args.load_y], dtype=float)
    norm = np.linalg.norm(direction)
    if norm == 0:
        raise ValueError("load direction cannot be zero")
    direction /= norm
    for node in load_nodes:
        f[2 * node : 2 * node + 2] += args.load * direction / len(load_nodes)

    fixed_nodes = nearest_nodes(args.frame_holes, nelx, nely, args.cell)
    fixed = np.array([dof for node in fixed_nodes for dof in (2 * node, 2 * node + 1)])
    free = np.setdiff1d(np.arange(ndof), fixed)

    history: list[float] = []
    for iteration in range(1, args.max_iter + 1):
        stiffness = args.emin + x**args.penal * (args.e0 - args.emin)
        sk = (ke.ravel()[:, None] * stiffness[None, :]).ravel(order="F")
        k = sp.coo_matrix((sk, (ik, jk)), shape=(ndof, ndof)).tocsc()
        k = (k + k.T) * 0.5

        u = np.zeros(ndof)
        u[free] = spla.spsolve(k[free, :][:, free], f[free])
        ue = u[edof]
        ce = np.einsum("ij,ij->i", ue @ ke, ue)
        compliance = float(np.sum(stiffness * ce))
        dc = -args.penal * (args.e0 - args.emin) * x ** (args.penal - 1) * ce
        dc = np.asarray(h @ (x * dc) / hs / np.maximum(1e-3, x))
        dc[passive] = 0.0

        x_old = x.copy()
        l1, l2 = 0.0, 1e9
        target = args.volfrac * design.sum()
        while (l2 - l1) / (l1 + l2 + 1e-12) > 1e-4:
            lmid = 0.5 * (l1 + l2)
            candidate = np.clip(
                x * np.sqrt(np.maximum(0.0, -dc / lmid)),
                x - args.move,
                x + args.move,
            )
            candidate = np.clip(candidate, 0.0, 1.0)
            candidate[solid_flat] = 1.0
            candidate[void_flat] = 0.0
            if candidate[design].sum() > target:
                l1 = lmid
            else:
                l2 = lmid
        x = candidate
        change = float(np.max(np.abs(x - x_old)))
        history.append(compliance)
        print(
            f"iter={iteration:03d} compliance={compliance:12.4f} "
            f"volume={x[design].mean():.3f} change={change:.4f}"
        )
        if change < args.tol:
            break

    return x.reshape((nely, nelx), order="F"), history


def write_svg(path: Path, mask: np.ndarray, cell_mm: float) -> None:
    nely, nelx = mask.shape
    rects = []
    for row, col in np.argwhere(mask):
        x = col * cell_mm
        y = (nely - row - 1) * cell_mm
        rects.append(
            f'<rect x="{x:.3f}" y="{y:.3f}" width="{cell_mm:.3f}" '
            f'height="{cell_mm:.3f}"/>'
        )
    width = nelx * cell_mm
    height = nely * cell_mm
    path.write_text(
        "\n".join(
            [
                f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
                '<g fill="black" stroke="none">',
                *rects,
                "</g>",
                "</svg>",
            ]
        )
        + "\n",
        encoding="ascii",
    )


def triangle(normal: tuple[float, float, float], vertices: list[tuple[float, float, float]]) -> str:
    lines = [f"  facet normal {normal[0]} {normal[1]} {normal[2]}", "    outer loop"]
    lines.extend(f"      vertex {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices)
    lines.extend(["    endloop", "  endfacet"])
    return "\n".join(lines)


def write_stl(path: Path, mask: np.ndarray, cell_mm: float, thickness: float) -> None:
    """Write a watertight voxel extrusion as ASCII STL."""
    nely, nelx = mask.shape
    facets: list[str] = []
    for row, col in np.argwhere(mask):
        x0, x1 = col * cell_mm, (col + 1) * cell_mm
        y0, y1 = (nely - row - 1) * cell_mm, (nely - row) * cell_mm
        z0, z1 = 0.0, thickness
        facets.append(triangle((0, 0, -1), [(x0, y0, z0), (x1, y1, z0), (x1, y0, z0)]))
        facets.append(triangle((0, 0, -1), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0)]))
        facets.append(triangle((0, 0, 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1)]))
        facets.append(triangle((0, 0, 1), [(x0, y0, z1), (x1, y1, z1), (x0, y1, z1)]))
        neighbors = [
            (row, col - 1, (-1, 0, 0), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)]),
            (row, col + 1, (1, 0, 0), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
            (row + 1, col, (0, -1, 0), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
            (row - 1, col, (0, 1, 0), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]),
        ]
        for nr, nc, normal, quad in neighbors:
            if nr < 0 or nr >= nely or nc < 0 or nc >= nelx or not mask[nr, nc]:
                facets.append(triangle(normal, [quad[0], quad[1], quad[2]]))
                facets.append(triangle(normal, [quad[0], quad[2], quad[3]]))
    path.write_text("solid opencr_base\n" + "\n".join(facets) + "\nendsolid opencr_base\n", encoding="ascii")


def save_outputs(args: argparse.Namespace, density: np.ndarray, history: list[float]) -> None:
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    nely, nelx = density.shape
    xx, yy = element_centers(nelx, nely, args.cell)
    holes = circular_mask(xx, yy, args.board_holes + args.frame_holes, args.hole_radius)
    mask = (density >= args.threshold) & ~holes
    np.save(output / "density.npy", density)
    np.save(output / "mask.npy", mask)

    extent = [0, args.width, 0, args.height]
    plt.figure(figsize=(10, 7))
    plt.imshow(density, cmap="gray_r", origin="lower", extent=extent, vmin=0, vmax=1)
    plt.colorbar(label="material density")
    plt.xlabel("x [mm]")
    plt.ylabel("y [mm]")
    plt.title("OpenCR base topology optimization")
    plt.tight_layout()
    plt.savefig(output / "density.png", dpi=180)
    plt.close()

    plt.figure(figsize=(7, 4))
    plt.plot(np.arange(1, len(history) + 1), history)
    plt.xlabel("iteration")
    plt.ylabel("compliance")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output / "convergence.png", dpi=180)
    plt.close()

    write_svg(output / "opencr_base.svg", mask, args.cell)
    write_stl(output / "opencr_base.stl", mask, args.cell, args.thickness)
    print(f"outputs written to: {output.resolve()}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, default=125.0, help="domain width [mm]")
    parser.add_argument("--height", type=float, default=95.0, help="domain height [mm]")
    parser.add_argument("--cell", type=float, default=1.0, help="element size [mm]")
    parser.add_argument("--thickness", type=float, default=4.0, help="STL extrusion thickness [mm]")
    parser.add_argument("--volfrac", type=float, default=0.35, help="design-domain material fraction")
    parser.add_argument("--penal", type=float, default=3.0, help="SIMP penalization")
    parser.add_argument("--rmin", type=float, default=2.5, help="filter radius [elements]")
    parser.add_argument("--move", type=float, default=0.2, help="maximum density change per iteration")
    parser.add_argument("--tol", type=float, default=0.01, help="convergence tolerance")
    parser.add_argument("--max-iter", type=int, default=120, help="maximum iterations")
    parser.add_argument("--threshold", type=float, default=0.5, help="density threshold for SVG/STL")
    parser.add_argument("--e0", type=float, default=1.0, help="solid Young's modulus scaling")
    parser.add_argument("--emin", type=float, default=1e-9, help="void Young's modulus scaling")
    parser.add_argument("--poisson", type=float, default=0.3, help="Poisson ratio")
    parser.add_argument("--hole-radius", type=float, default=1.7, help="mounting hole radius [mm]")
    parser.add_argument("--pad-radius", type=float, default=6.0, help="forced-solid pad radius [mm]")
    parser.add_argument("--load", type=float, default=1.0, help="total in-plane load [arbitrary units]")
    parser.add_argument("--load-x", type=float, default=0.0, help="load direction x component")
    parser.add_argument("--load-y", type=float, default=-1.0, help="load direction y component")
    parser.add_argument(
        "--board-holes",
        type=parse_xy_list,
        default=parse_xy_list("15,15;110,15;15,80;110,80"),
        help="OpenCR mounting holes 'x,y;x,y' [mm]",
    )
    parser.add_argument(
        "--frame-holes",
        type=parse_xy_list,
        default=parse_xy_list("5,5;120,5;5,90;120,90"),
        help="fixed enclosure holes 'x,y;x,y' [mm]",
    )
    parser.add_argument("--output", default="topology_opencr/results", help="output directory")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    density, history = optimize(args)
    save_outputs(args, density, history)


if __name__ == "__main__":
    main()
