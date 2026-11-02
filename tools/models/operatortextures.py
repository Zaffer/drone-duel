"""The operator's textures: the few maps tools.models.operatormodel uses, out of the listing's
pbr_textures.zip (1.2 GB of 4096 px PNGs), shrunk and committed as assets/quantum_textures/.

    uv run python -m tools.models.operatortextures ~/Downloads/pbr_textures.zip \\
        assets/quantum_textures

Each material gets its base colour (BASE_PX), its normal map and its packed occlusion, roughness
and metal map (DETAIL_PX; Unreal packs them red, green, blue, as glTF reads them), as JPEGs named
<material>_<map>.jpg. glTF's normal maps are OpenGL's way up (green up): the listing's "Unity"
normals are, its Unreal ones are not, so where only those exist their green is flipped. Most
parts come only in the listing's beige ("Bege"); the shirt is blue.
"""

import io
import sys
import zipfile
from pathlib import Path

from PIL import Image, ImageChops

BASE_PX = 1024
"""The base colour's side. The operator is a few hundred pixels tall at most."""
DETAIL_PX = 512
"""The normal and occlusion-roughness-metal maps' side."""
QUALITY = {"base": 88, "normal": 92, "orm": 88}

ORM = "OcclusionRoughnessMetallic"
SOURCES: dict[str, dict[str, str]] = {
    # material: map: path in the zip ("!" before a normal map: Unreal's way up, flip its green)
    "M_Head": {
        "base": "Body/T_Head_BaseColor.png",
        "normal": "Body/T_Head_Unity_Normal.png",
        "orm": f"Body/T_Head_{ORM}.png",
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
    "M_Jeans": {
        "base": "Jeans/T_Jeans_Bege_BaseColor.png",
        "normal": "Jeans/T_Jeans_Unity_Normal.png",
        "orm": f"Jeans/T_Jeans_{ORM}.png",
    },
    "M_Bulletproof_Light": {
        "base": "Bulletproof/T_Bulletproof_Bege_BaseColor.png",
        "normal": "Bulletproof/T_Bulletproof_Light_Unity_Normal.png",
        "orm": f"Bulletproof/T_Bulletproof_Light_{ORM}.png",
    },
    "M_Drops_Tactical": {
        "base": "Drops/T_Drops_Tactical_Bege_BaseColor.png",
        "normal": "Drops/T_Drops_Tactical_Unity_Normal.png",
        "orm": f"Drops/T_Drops_Tactical_{ORM}.png",
    },
    "M_Cap": {
        "base": "Cap/T_Cap_Bege_BaseColor.png",
        "normal": "Cap/T_Cap_Unity_Normal.png",
        "orm": f"Cap/T_Cap_{ORM}.png",
    },
    "M_Patches": {
        "base": "Patch/T_Patches_BaseColor.jpg",
        "normal": "Patch/T_Patches_Unity_Normal.png",
        "orm": f"Patch/T_Patches_{ORM}.png",
    },
    "M_Holster_Hard": {
        "base": "Holdster_Hard/M_Holster_Hard_Bege_BaseColor.png",
        "normal": "Holdster_Hard/M_Holster_Hard_Unity_Normal.png",
        "orm": f"Holdster_Hard/M_Holster_Hard_{ORM}.png",
    },
    "M_Pistol": {
        "base": "Weapon/Pistol_BaseColor.png",
        "normal": "Weapon/Pistol_Unity_Normal.png",
        "orm": f"Weapon/Pistol_{ORM}.png",
    },
}


def shrink(data: bytes, kind: str, flip_green: bool = False) -> bytes:
    """One map as a JPEG at its kind's size."""
    image = Image.open(io.BytesIO(data)).convert("RGB")
    side = BASE_PX if kind == "base" else DETAIL_PX
    image = image.resize((side, side), Image.Resampling.LANCZOS)
    if flip_green:
        r, g, b = image.split()
        image = Image.merge("RGB", (r, ImageChops.invert(g), b))
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=QUALITY[kind], optimize=True)
    return out.getvalue()


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit("usage: python -m tools.models.operatortextures <pbr_textures.zip> <dir>")
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(sys.argv[1]) as z:
        for material, maps in SOURCES.items():
            for kind, path in maps.items():
                flip = path.startswith("!")
                data = shrink(z.read(f"PBR_Textures/{path.lstrip('!')}"), kind, flip)
                (out / f"{material}_{kind}.jpg").write_bytes(data)
    total = sum(f.stat().st_size for f in out.glob("*.jpg"))
    print(f"wrote {len(SOURCES) * 3} maps to {out} ({total / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
