#!/usr/bin/env python3
"""Create vector A4 sheets from the existing 20 mm SVGs; standard library only."""
from pathlib import Path
import xml.etree.ElementTree as ET

folder = Path(__file__).resolve().parents[1] / 'docs/aruco_markers'
output = folder / 'DICT_4X4_50_20mm_A4_tiled.pdf'
mm = 72/25.4
width, height = 210*mm, 297*mm
objects = [b'<< /Type /Catalog /Pages 2 0 R >>',
           b'<< /Type /Pages /Kids [4 0 R 6 0 R] /Count 2 >>',
           b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>']
for marker_id in (0, 1):
    svg = ET.parse(folder / f'DICT_4X4_50_id{marker_id}_20mm.svg').getroot()
    rectangles = svg.findall('.//{http://www.w3.org/2000/svg}g/{http://www.w3.org/2000/svg}rect')
    commands = ['0 g']
    def text(x, y, size, value):
        value = value.replace('\\', '\\\\').replace('(', '\\(').replace(')', '\\)')
        commands.append(f'BT /F1 {size} Tf {x*mm:.6f} {y*mm:.6f} Td ({value}) Tj ET')
    text(15, 283, 12, f'ArUco DICT_4X4_50 | ID {marker_id} | 48 copies')
    text(15, 277, 9, 'Black square: 20 x 20 mm. Print A4 at Actual size / 100%. Disable Fit to page.')
    for row in range(8):
        for col in range(6):
            left, top = 15+col*30, 297-26-row*30
            for rect in rectangles:
                x,y,w,h = [float(rect.get(k)) for k in ('x','y','width','height')]
                commands.append(f'{(left+x)*mm:.6f} {(top-y-h)*mm:.6f} {w*mm:.6f} {h*mm:.6f} re f')
    text(15, 22, 9, 'Center spacing: 30 mm. White margin: 5 mm per side. Verify size with a ruler.')
    commands.append(f'0.4 w {15*mm} {15*mm} m {35*mm} {15*mm} l S')
    for x in (15,35):commands.append(f'{x*mm} {14*mm} m {x*mm} {16*mm} l S')
    text(38, 14, 8, '20 mm reference')
    content = ('\n'.join(commands)+'\n').encode('ascii')
    stream_number = len(objects)+2
    objects.append(f'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {width:.6f} {height:.6f}] /Resources << /Font << /F1 3 0 R >> >> /Contents {stream_number} 0 R >>'.encode())
    objects.append(f'<< /Length {len(content)} >>\nstream\n'.encode()+content+b'endstream')
pdf = bytearray(b'%PDF-1.4\n%\xe2\xe3\xcf\xd3\n');offsets = [0]
for index,obj in enumerate(objects,1):
    offsets.append(len(pdf));pdf.extend(f'{index} 0 obj\n'.encode()+obj+b'\nendobj\n')
xref=len(pdf);pdf.extend(f'xref\n0 {len(offsets)}\n0000000000 65535 f \n'.encode())
for offset in offsets[1:]:pdf.extend(f'{offset:010d} 00000 n \n'.encode())
pdf.extend(f'trailer\n<< /Size {len(offsets)} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n'.encode())
output.write_bytes(pdf)
print(output)
