"""Green buildings for both windows, from the Veins Erlangen polygon file.

- erlangen.poly.xml:       SUMO polygons, type=building recoloured green
                           (from erlangen.poly.xml.orig)
- erlangen.buildings.xml:  the same buildings as INET physical-environment
                           objects in OMNeT++ coordinates, for drawing only
                           (no obstacle loss is configured, so they do not
                           affect the radio)

usage (from simulations/): python3 tools/make_buildings.py
"""
import xml.etree.ElementTree as ET

GREEN_SUMO = "0.13,0.55,0.13"   # forest green
GREEN_INET = "34 139 34"
HEIGHT = 10.0                   # m, only used by 3D views
MARGIN = 25.0                   # Veins TraCI coordinate margin

loc = None
for ev, el in ET.iterparse("erlangen.net.xml", events=("start",)):
    if el.tag == "location":
        loc = el
        break
xmin, ymin, xmax, ymax = map(float, loc.get("convBoundary").split(","))

tree = ET.parse("erlangen.poly.xml.orig")
env = ET.Element("environment")
n = 0
for poly in tree.getroot().iter("poly"):
    if poly.get("type") != "building":
        continue
    poly.set("color", GREEN_SUMO)
    # SUMO -> OMNeT++ coordinates, as Veins does: x - xmin + margin, ymax - y + margin
    pts = [tuple(map(float, p.split(","))) for p in poly.get("shape").split()]
    pts = [(x - xmin + MARGIN, ymax - y + MARGIN) for x, y in pts]
    if pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        continue
    minx = min(x for x, _ in pts); miny = min(y for _, y in pts)
    rel = " ".join(f"{x - minx:.2f} {y - miny:.2f}" for x, y in pts)
    ET.SubElement(env, "object", {
        "id": poly.get("id"),
        "position": f"min {minx:.2f} {miny:.2f} 0",
        "orientation": "0 0 0",
        "shape": f"prism {HEIGHT} {rel}",
        "material": "concrete",   # required by INET; unused (no obstacle loss)
        "fill-color": GREEN_INET,
        "line-color": "0 90 0",
        "opacity": "1",
    })
    n += 1

tree.write("erlangen.poly.xml", encoding="utf-8", xml_declaration=True)
ET.indent(env)
ET.ElementTree(env).write("erlangen.buildings.xml", encoding="utf-8", xml_declaration=True)
print(f"{n} buildings -> erlangen.poly.xml (SUMO) and erlangen.buildings.xml (OMNeT++)")
