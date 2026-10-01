#!/usr/bin/env python3
"""Auto-label single-target blue armor pairs for empirical camera alignment.

This is not a physical stereo calibration. Ambiguous or missing light pairs are skipped.
The generated labels must be checked before being used for multi-target operation.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def blue_components(image, difference, minimum_blue, minimum_area):
    blue, _, red = cv2.split(image)
    mask = np.uint8(
        (blue.astype(np.int16) - red.astype(np.int16) > difference)
        & (blue > minimum_blue)) * 255
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(map(int, stats[i])) for i in range(1, count)
            if stats[i, cv2.CC_STAT_AREA] >= minimum_area]


def center(box):
    x, y, width, height, _ = box
    return (x + width / 2.0, y + height / 2.0)


def find_main_pair(image):
    parts = blue_components(image, 75, 120, 15)
    parts = [box for box in parts if box[3] >= 7 and box[3] >= 1.5 * box[2]
             and box[1] > image.shape[0] * 0.28]
    pairs = []
    for i, first in enumerate(parts):
        for second in parts[i + 1:]:
            left, right = first, second
            if center(left)[0] > center(right)[0]:
                left, right = right, left
            length = (left[3] + right[3]) / 2.0
            gap = center(right)[0] - center(left)[0]
            if not (1.5 * length <= gap <= 6.0 * length):
                continue
            if abs(center(left)[1] - center(right)[1]) > 0.5 * length:
                continue
            if min(left[3], right[3]) / max(left[3], right[3]) < 0.55:
                continue
            score = min(left[4], right[4]) * min(left[3], right[3])
            pairs.append((score, left, right))
    pairs.sort(key=lambda item: item[0], reverse=True)
    if not pairs or (len(pairs) > 1 and pairs[1][0] > 0.65 * pairs[0][0]):
        return None
    _, left, right = pairs[0]
    return [(center(left)[0] + center(right)[0]) / 2.0,
            (center(left)[1] + center(right)[1]) / 2.0]


def find_number_box(image):
    # Close targets may bloom into one blob; increasing the blue/red contrast
    # threshold separates the two cores without guessing a position.
    for difference in (100, 110, 120, 130, 140, 150, 170, 190):
        parts = blue_components(image, difference, 180, 35)
        parts = [box for box in parts if box[1] > image.shape[0] * 0.35
                 and box[3] >= 10 and box[2] <= image.shape[1] * 0.15]
        parts.sort(key=lambda box: box[4], reverse=True)
        if len(parts) < 2 or parts[1][4] < 0.2 * parts[0][4]:
            continue
        left, right = sorted(parts[:2], key=lambda box: center(box)[0])
        gap = center(right)[0] - center(left)[0]
        height = (left[3] + right[3]) / 2.0
        if not (0.7 * height <= gap <= 3.0 * height):
            continue
        if abs(center(left)[1] - center(right)[1]) > 0.5 * height:
            continue
        if min(left[3], right[3]) / max(left[3], right[3]) < 0.5:
            continue
        x = (center(left)[0] + center(right)[0]) / 2.0
        y = (center(left)[1] + center(right)[1]) / 2.0 + 0.15 * height
        width = max(8.0, 0.7 * gap)
        box_height = max(8.0, 1.25 * height)
        box = [round(x - width / 2), round(y - box_height / 2),
               round(width), round(box_height)]
        if box[0] >= 0 and box[1] >= 0 and box[0] + box[2] <= image.shape[1] \
                and box[1] + box[3] <= image.shape[0]:
            return box
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or args.directory / 'auto_annotations.jsonl'
    if output.exists():
        parser.error(f'{output} already exists; choose a new --output to preserve labels')
    accepted, rejected = [], []
    for line in (args.directory / 'pairs.jsonl').read_text().splitlines():
        pair = json.loads(line)
        main_image = cv2.imread(str(args.directory / f"{pair['name']}_main.png"))
        number_image = cv2.imread(str(args.directory / f"{pair['name']}_number.png"))
        if main_image is None or number_image is None:
            rejected.append((pair['name'], 'image_missing'))
            continue
        main_center = find_main_pair(main_image)
        number_box = find_number_box(number_image)
        if main_center is None or number_box is None:
            rejected.append((pair['name'], 'main_or_number_pair_ambiguous'))
            continue
        pair['annotations'] = [{'main_center': main_center, 'number_box': number_box}]
        accepted.append(pair)
    with output.open('w', encoding='utf-8') as stream:
        for record in accepted:
            stream.write(json.dumps(record) + '\n')
    print(f'Auto-labeled {len(accepted)}/{len(accepted) + len(rejected)} pairs to {output}')
    for name, reason in rejected:
        print(f'  skipped {name}: {reason}')


if __name__ == '__main__':
    main()
