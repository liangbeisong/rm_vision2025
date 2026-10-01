#!/usr/bin/env python3
"""Fit normalized image-to-image mapping with a 1/depth term."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def robust_fit(features, centers, width, height, threshold_px=10.0):
    """RANSAC fit; return coefficients and a conservative inlier mask."""
    pixels = centers * [width, height]
    random = np.random.default_rng(42)
    best_mask = None
    best_score = (-1, float('-inf'))
    for _ in range(4000):
        chosen = random.choice(len(features), 4, replace=False)
        sample = features[chosen]
        if np.linalg.matrix_rank(sample) < 4 or np.linalg.cond(sample) > 10000:
            continue
        coefficients = np.linalg.lstsq(sample, pixels[chosen], rcond=None)[0]
        error = np.linalg.norm(features @ coefficients - pixels, axis=1)
        mask = error < threshold_px
        score = (int(mask.sum()), -float(np.median(error[mask])))
        if score > best_score:
            best_score, best_mask = score, mask
    if best_mask is None or best_mask.sum() < 8:
        raise ValueError('Too few consistent correspondences for robust fitting')
    for _ in range(5):
        coefficients = np.linalg.lstsq(features[best_mask], pixels[best_mask], rcond=None)[0]
        error = np.linalg.norm(features @ coefficients - pixels, axis=1)
        next_mask = error < threshold_px
        if np.array_equal(next_mask, best_mask):
            break
        best_mask = next_mask
    coefficients = np.linalg.lstsq(features[best_mask], pixels[best_mask], rcond=None)[0]
    return coefficients / [width, height], best_mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('annotations', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--robust', action='store_true',
                        help='reject inconsistent automatic labels before fitting')
    args = parser.parse_args()
    records = [json.loads(line) for line in args.annotations.read_text().splitlines()]
    records = [r for r in records if r['annotations']]
    if not records:
        parser.error('No annotations found')
    sizes = {(tuple(r['main_size']), tuple(r['number_size'])) for r in records}
    if len(sizes) != 1:
        parser.error('All frames must have identical image sizes')
    (mw, mh), (nw, nh) = sizes.pop()
    features, centers, sizes_out, positions, depths, names = [], [], [], [], [], []
    for record in records:
        depth = float(record['distance_m'])
        for annotation in record['annotations']:
            u, v = annotation['main_center'][0] / mw, annotation['main_center'][1] / mh
            x, y, w, h = annotation['number_box']
            features.append([u, v, 1.0, 1.0 / depth])
            centers.append([(x + w / 2) / nw, (y + h / 2) / nh])
            sizes_out.append([w / nw, h / nh])
            positions.append([u, v])
            depths.append(depth)
            names.append(record['name'])
    features = np.asarray(features, dtype=np.float64)
    centers = np.asarray(centers, dtype=np.float64)
    size_features = features[:, 2:]
    if len(features) < 8 or np.linalg.matrix_rank(features) < 4:
        parser.error('Need at least 8 geometrically diverse matches across distances')
    if args.robust:
        fitted, inliers = robust_fit(features, centers, nw, nh)
        if inliers.sum() < 0.6 * len(inliers):
            parser.error('Fewer than 60% of labels agree; mapping not written')
        for distance in sorted(set(depths)):
            count = sum(inliers & (np.asarray(depths) == distance))
            if count < 3:
                parser.error(f'Only {count} inliers at {distance:g} m; mapping not written')
        print(f'Robust fit kept {inliers.sum()}/{len(inliers)} labels')
        print('Rejected:', ', '.join(name for name, keep in zip(names, inliers) if not keep))
        center_coeff = fitted.T
    else:
        inliers = np.ones(len(features), dtype=bool)
        center_coeff = np.linalg.lstsq(features, centers, rcond=None)[0].T
    size_coeff = np.linalg.lstsq(size_features[inliers],
                                 np.asarray(sizes_out)[inliers], rcond=None)[0].T
    prediction = features @ center_coeff.T
    errors = np.linalg.norm((prediction - centers) * [nw, nh], axis=1)
    print(f'{inliers.sum()} fitted samples; training center error median/max: '
          f'{np.median(errors[inliers]):.1f}/{np.max(errors[inliers]):.1f} px')
    positions = np.asarray(positions)[inliers]
    fs = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    for key, value in {
        'main_width': mw, 'main_height': mh, 'number_width': nw, 'number_height': nh,
        'min_depth': min(depths), 'max_depth': max(depths),
        'min_u': float(positions[:, 0].min()), 'max_u': float(positions[:, 0].max()),
        'min_v': float(positions[:, 1].min()), 'max_v': float(positions[:, 1].max()),
    }.items():
        fs.write(key, value)
    fs.write('center_coeff', center_coeff)
    fs.write('size_coeff', size_coeff)
    fs.release()


if __name__ == '__main__':
    main()
