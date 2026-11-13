"""AprilTag detector adapter shared by live and offline tools."""

import cv2
import numpy as np


class Detector:
    def __init__(self, config):
        from apriltag import apriltag
        self.family = config['family']
        self.allowed_ids = set(config['tag_ids'])
        self.native = apriltag(self.family, threads=int(config['threads']),
                               maxhamming=int(config['maxhamming']),
                               decimate=float(config['decimate']), refine_edges=True)

    def detect(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detections = []
        for tag in self.native.detect(np.ascontiguousarray(gray)):
            if int(tag['id']) not in self.allowed_ids:
                continue
            detections.append({'family': self.family, 'id': int(tag['id']),
                               'center_px': np.asarray(tag['center']).tolist(),
                               'corners_lb_rb_rt_lt_px': np.asarray(tag['lb-rb-rt-lt']).tolist(),
                               'hamming': int(tag['hamming']), 'decision_margin': float(tag['margin'])})
        return detections
