"""Create a printable ID41 reference marker for the one-camera LEAP setup.

The marker belongs on the rigid hand mount, outside the cube's occlusion area.
Print the PDF at actual size and measure the black square after printing.
"""
from pathlib import Path

import cv2
import numpy as np
import zlib

ROOT = Path(__file__).resolve().parent
MARKER_ID = 41
BLACK_SQUARE_MM = 30.0


def main():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, MARKER_ID, 600)
    image = cv2.copyMakeBorder(marker, 150, 150, 150, 150,
                               cv2.BORDER_CONSTANT, value=255)
    cv2.imencode('.png', image)[1].tofile(str(ROOT / 'leap_anchor_id41.png'))
    # Small PDF writer avoids depending on optional plotting packages.
    page_w, page_h = 595.276, 841.890  # A4 points
    image_w = BLACK_SQUARE_MM * 1.5 * 72 / 25.4  # 7.5 mm white border each side
    x, y = (page_w - image_w) / 2, 500.0
    compressed = zlib.compress(np.ascontiguousarray(image).tobytes())
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        b'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595.276 841.890] '
        b'/Resources << /XObject << /Im0 4 0 R >> /Font << /F1 6 0 R >> >> /Contents 5 0 R >>',
        (f'<< /Type /XObject /Subtype /Image /Width {image.shape[1]} /Height {image.shape[0]} '
         f'/ColorSpace /DeviceGray /BitsPerComponent 8 /Filter /FlateDecode /Length {len(compressed)} >>\n'
         ).encode() + b'stream\n' + compressed + b'\nendstream',
        (f'<< /Length 130 >>\nstream\nq\n{image_w:.5f} 0 0 {image_w:.5f} {x:.5f} {y:.5f} cm\n/Im0 Do\nQ\n'
         'BT /F1 11 Tf 160 470 Td (LEAP rigid-mount reference: ArUco 4x4_50 ID 41) Tj ET\n'
         'BT /F1 10 Tf 160 452 Td (Print 100% actual size; black square 30 mm) Tj ET\n'
         'endstream').encode(),
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
    ]
    # Set the content-stream length from the actual bytes.
    content = objects[4]
    stream_start = content.index(b'stream\n') + len(b'stream\n')
    stream_end = content.index(b'\nendstream', stream_start)
    objects[4] = content.replace(b'/Length 130', f'/Length {stream_end-stream_start}'.encode(), 1)
    pdf = bytearray(b'%PDF-1.4\n')
    offsets = [0]
    for number, obj in enumerate(objects, 1):
        offsets.append(len(pdf))
        pdf += f'{number} 0 obj\n'.encode() + obj + b'\nendobj\n'
    xref = len(pdf)
    pdf += f'xref\n0 {len(objects)+1}\n0000000000 65535 f \n'.encode()
    for offset in offsets[1:]:
        pdf += f'{offset:010d} 00000 n \n'.encode()
    pdf += (f'trailer\n<< /Size {len(objects)+1} /Root 1 0 R >>\n'
            f'startxref\n{xref}\n%%EOF\n').encode()
    (ROOT / 'leap_anchor_id41_A4.pdf').write_bytes(pdf)
    print(ROOT / 'leap_anchor_id41_A4.pdf')


if __name__ == '__main__':
    main()
