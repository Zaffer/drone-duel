"""The first-person rifle's textures: the rifle's maps and the bare arms' and sleeves' out of the
listing's pbr_textures.zip (tools.models.operatortextures has the listing), shrunk and committed
as assets/rifle_textures/ for tools.models.riflemodel.

    uv run python -m tools.models.rifletextures ~/Downloads/pbr_textures.zip \\
        assets/rifle_textures

The rifle and the hands are a hand's breadth from the eye, so their maps are larger than the
operator's (SIDE_PX), each a JPEG named <material>_<map>.jpg, made as operatortextures makes
them: base colour, normal (OpenGL's way up) and packed occlusion, roughness and metal. The rifle
comes in the listing's olive; the base colour's alpha (the sight's glass) is dropped, the glass
being a material of its own.
"""

import sys
import zipfile
from pathlib import Path

from tools.models.operatortextures import ORM, shrink

SIDE_PX: dict[str, dict[str, int]] = {
    # material: map: side
    "Rifle": {"base": 2048, "normal": 2048, "orm": 1024},
    "M_Quantum_Arms": {"base": 1024, "normal": 1024, "orm": 1024},
    "M_Shirt_RolledUp": {"base": 1024, "normal": 512, "orm": 512},
}
SOURCES: dict[str, dict[str, str]] = {
    # material: map: path in the zip ("!" before a normal map: Unreal's way up, flip its green)
    "Rifle": {
        "base": "Weapon/Rifle_Olive_BaseColor.png",
        "normal": "Weapon/Rifle_Olive_Unity_Normal.png",
        "orm": f"Weapon/Rifle_Olive_{ORM}.png",
    },
    "M_Quantum_Arms": {
        "base": "Arms/T_Quantum_Basemesh_Arms_BaseColor.1003.png",
        "normal": "!Arms/T_Quantum_Basemesh_Arms_Normal.1003.png",
        "orm": f"Arms/T_Quantum_Basemesh_Arms_{ORM}.1003.png",
    },
    "M_Shirt_RolledUp": {
        "base": "Shirt_RolledUp/T_Shirt_RolledUp_Blue_BaseColor.png",
        "normal": "Shirt_RolledUp/T_Shirt_RolledUp_Unity_Normal.png",
        "orm": f"Shirt_RolledUp/T_Shirt_RolledUp_{ORM}.png",
    },
}


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: python -m tools.models.rifletextures <pbr_textures.zip> <dir>")
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(sys.argv[1]) as z:
        for material, maps in SOURCES.items():
            for kind, path in maps.items():
                flip = path.startswith("!")
                data = z.read(f"PBR_Textures/{path.lstrip('!')}")
                side = SIDE_PX[material][kind]
                (out / f"{material}_{kind}.jpg").write_bytes(shrink(data, kind, flip, side))
    total = sum(f.stat().st_size for f in out.glob("*.jpg"))
    print(f"wrote {len(SOURCES) * 3} maps to {out} ({total / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
