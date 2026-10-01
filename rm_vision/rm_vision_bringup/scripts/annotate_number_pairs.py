#!/usr/bin/env python3
"""Click a primary armor center, then draw its matching secondary digit box."""

import argparse
import json
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    output = args.directory / 'annotations.jsonl'
    existing = set()
    if output.exists():
        existing = {json.loads(line)['name'] for line in output.read_text().splitlines()}
    for line in (args.directory / 'pairs.jsonl').read_text().splitlines():
        pair = json.loads(line)
        if pair['name'] in existing:
            continue
        main_img = cv2.imread(str(args.directory / f"{pair['name']}_main.png"))
        number_img = cv2.imread(str(args.directory / f"{pair['name']}_number.png"))
        if main_img is None or number_img is None:
            continue
        annotations = []
        points = []

        def click(event, x, y, flags, userdata):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append([x, y])

        cv2.namedWindow('main: click armor center; q=next')
        cv2.setMouseCallback('main: click armor center; q=next', click)
        while True:
            shown = main_img.copy()
            for item in annotations:
                cv2.circle(shown, tuple(item['main_center']), 5, (0, 255, 0), 2)
            cv2.imshow('main: click armor center; q=next', shown)
            key = cv2.waitKey(50) & 0xff
            if key == ord('q'):
                break
            if not points:
                continue
            point = points.pop(0)
            box = cv2.selectROI('number: draw corresponding digit', number_img,
                                showCrosshair=True, fromCenter=False)
            cv2.destroyWindow('number: draw corresponding digit')
            if box[2] > 0 and box[3] > 0:
                annotations.append({'main_center': point, 'number_box': list(box)})
        cv2.destroyWindow('main: click armor center; q=next')
        pair['annotations'] = annotations
        with output.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(pair) + '\n')


if __name__ == '__main__':
    main()
