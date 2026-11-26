#!/usr/bin/env python3
"""Jetson CUDA inference service; local loopback only, no ROS 2 dependencies.

Protocol: request = uint32 BE length + JPEG BGR image bytes.
          reply   = uint32 BE length + JSON array of raw detections.
"""
import json
import os
import socket
import struct
import time

import cv2
import numpy as np
import torch
from ultralytics import YOLO

MAX_IMAGE_BYTES = 12 * 1024 * 1024


def read_exact(sock, size):
    buf = bytearray()
    while len(buf) < size:
        piece = sock.recv(size - len(buf))
        if not piece:
            raise EOFError('Connection closed')
        buf.extend(piece)
    return bytes(buf)


def serve_client(conn, model, model_names, conf, iou, imgsz):
    while True:
        try:
            raw_length = read_exact(conn, 4)
        except EOFError:
            break
        length, = struct.unpack('!I', raw_length)
        if not 0 < length <= MAX_IMAGE_BYTES:
            raise ValueError(f'Unexpected image length: {length}')
        jpeg = read_exact(conn, length)
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError('JPEG decoding failed')
        results = model.predict(source=image, device=0, conf=conf, iou=iou,
                                imgsz=imgsz, verbose=False)
        records = []
        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            records.append({
                'xyxy': [float(x) for x in box.xyxy[0].cpu().tolist()],
                'confidence': float(box.conf[0].item()),
                'class_name': str(model_names.get(class_id, class_id)),
            })
        reply = json.dumps(records, separators=(',', ':')).encode('utf8')
        conn.sendall(struct.pack('!I', len(reply)) + reply)


def main():
    if not torch.cuda.is_available():
        raise SystemExit('CUDA is unavailable; refusing to fall back to CPU')
    model_path = os.environ.get('YOLO_MODEL', '/models/best.pt')
    imgsz = int(os.environ.get('YOLO_IMGSZ', '640'))
    conf = float(os.environ.get('YOLO_CONF', '0.30'))
    iou = float(os.environ.get('YOLO_IOU', '0.45'))
    port = int(os.environ.get('YOLO_PORT', '58751'))
    print(f'[YOLO] Loading {model_path} (CUDA, imgsz={imgsz})', flush=True)
    model = YOLO(model_path)
    names = model.names
    # Warm up before accepting frames. Synthetic image has no semantic meaning.
    model.predict(source=np.zeros((640, 640, 3), dtype=np.uint8), device=0,
                  imgsz=imgsz, conf=conf, iou=iou, verbose=False)
    print(f'[YOLO] Ready; classes={names}, port={port}', flush=True)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', port))
        server.listen(1)
        while True:
            conn, addr = server.accept()
            with conn:
                conn.settimeout(10.0)
                try:
                    serve_client(conn, model, names, conf, iou, imgsz)
                except (OSError, ValueError, EOFError) as e:
                    print(f'[YOLO] Client disconnected: {e}', flush=True)


if __name__ == '__main__':
    main()
