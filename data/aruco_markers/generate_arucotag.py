import argparse

import cv2
import cv2.aruco as aruco
import numpy as np


def mm_to_pixels(mm, dpi):
    inches = mm / 25.4
    pixels = int(inches * dpi)
    return pixels


def generate_aruco_marker(marker_size, marker_id, aruco_family):
    aruco_dict = aruco.Dictionary_get(aruco_family)

    # Create a blank white image
    marker_image = 255 * np.ones((marker_size, marker_size), dtype=np.uint8)

    # Generate the marker
    marker = aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

    return marker


if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate ArUco marker")
    parser.add_argument("--marker_id", type=int, required=True, help="ArUco marker ID")
    parser.add_argument(
        "--aruco_family",
        type=int,
        default=aruco.DICT_4X4_250,
        help="ArUco dictionary/family (default: 4x4 dictionary)",
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default="",
        help="Output file name (default: aruco_marker_<marker_id>.png)",
    )
    args = parser.parse_args()

    # A4 paper size in millimeters
    a4_width_mm = 210
    a4_height_mm = 297

    # Desired resolution in dots per inch (dpi)
    dpi = 300

    # Convert A4 paper dimensions to pixels
    a4_width_pixels = mm_to_pixels(a4_width_mm, dpi)
    a4_height_pixels = mm_to_pixels(a4_height_mm, dpi)

    # Choose a marker size relative to the A4 paper dimensions
    marker_size = min(a4_width_pixels, a4_height_pixels) // 4

    # Generate the ArUco marker
    marker = generate_aruco_marker(marker_size, args.marker_id, args.aruco_family)

    # Save the marker image
    if args.output_file:
        cv2.imwrite(f"{args.output_file}.png", marker)
    else:
        cv2.imwrite(f"aruco_marker_{args.marker_id}.png", marker)
