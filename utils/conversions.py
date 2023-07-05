#!/usr/bin/env python3
#encoding: utf-8

import argparse



parser = argparse.ArgumentParser(description="Conversions d'échelle.")

parser.add_argument('--scale', help='Échelle, exprimée en entier. Exemple: 400 signifie 1:400. Défaut: 400.', type=int, default=400)
parser.add_argument('--resolution', help='Résolution en DPI (point par pouce). Défaut: 300.', type=int, default=300)
parser.add_argument('--unit', help='Unité de la valeur d\'entrée. Valeurs possibles: "m_world" pour mètres dans le monde réel, "cm_map" pour centimètre sur la carte, "px" pour nombre de pixels. Défaut: "m_world"', type=str, choices=['m_world', 'cm_map', 'px'], default='m_world')
parser.add_argument('--value', help="Valeur d'entrée.", type=float, required=True)

args = parser.parse_args()


resolution=args.resolution
scale=args.scale

unit=args.unit
value=args.value

if unit == "m_world":
    value_world_m = value
    value_map_cm = value / scale * 100
    value_px = value_map_cm / 2.54 * resolution
elif unit == "cm_map":
    value_map_cm = value
    value_world_m = value_map_cm * scale / 100
    value_px = value_map_cm / 2.54 * resolution
elif unit == "px":
    value_px = value
    value_map_cm = value_px * 2.54 / resolution
    value_world_m = value_map_cm * scale / 100
else:
    print("unkown unit")
    exit(1)

print("Résolution:", resolution, "points par pouce")
print("Échelle:", "1:" + str(scale))
print()

print("Dimension réelle:", value_world_m, "m")
print("Dimension sur la carte:", value_map_cm, "cm, ", value_px, "px")

