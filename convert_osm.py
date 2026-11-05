#!/usr/bin/env python3
# convert_osm.py
# Usage: python convert_osm.py chandigarh-roads.osm nodes.csv edges.csv

import sys
import xml.etree.ElementTree as ET
from math import radians, sin, cos, asin, sqrt

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1))*cos(radians(lat2))*sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    return R * c

if len(sys.argv) < 4:
    print("Usage: python convert_osm.py input.osm nodes.csv edges.csv")
    sys.exit(1)

infile = sys.argv[1]
nodes_out = sys.argv[2]
edges_out = sys.argv[3]

# First pass: collect nodes (id -> (lat,lon))
nodes = {}
print("Pass1: reading nodes...")
context = ET.iterparse(infile, events=('start','end'))
_, root = next(context)
for event, elem in context:
    if event == 'end' and elem.tag == 'node':
        nid = int(elem.get('id'))
        lat = float(elem.get('lat'))
        lon = float(elem.get('lon'))
        nodes[nid] = (lat, lon)
        root.clear()

print("Writing nodes.csv...")
with open(nodes_out, 'w', encoding='utf8') as f:
    for nid, (lat, lon) in nodes.items():
        f.write(f"{nid},{lat:.8f},{lon:.8f}\n")

# Second pass: ways -> edges (only ways with highway tag)
print("Pass2: scanning ways for highway tags and creating edges...")
context = ET.iterparse(infile, events=('start','end'))
_, root = next(context)
with open(edges_out, 'w', encoding='utf8') as fe:
    for event, elem in context:
        if event == 'end' and elem.tag == 'way':
            # check tags for highway
            is_highway = False
            node_refs = []
            for child in elem:
                if child.tag == 'nd':
                    ref = child.get('ref')
                    if ref:
                        node_refs.append(int(ref))
                elif child.tag == 'tag':
                    if child.get('k') == 'highway':
                        is_highway = True
            if is_highway:
                # output consecutive pairs (bidirectional for now)
                for i in range(1, len(node_refs)):
                    u = node_refs[i-1]; v = node_refs[i]
                    if u in nodes and v in nodes:
                        lat1, lon1 = nodes[u]; lat2, lon2 = nodes[v]
                        w = haversine(lat1, lon1, lat2, lon2)
                        fe.write(f"{u},{v},{w:.3f},0\n")
                        fe.write(f"{v},{u},{w:.3f},0\n")
            root.clear()

print("Done. nodes.csv and edges.csv created.")
