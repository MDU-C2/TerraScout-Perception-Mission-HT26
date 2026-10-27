#!/usr/bin/env python3
"""
OAK-D Interactive Image Capture Utility.
Captures full 1080p RGB frames directly from Luxonis OAK-D camera.

Controls:
  - Press 's': Save current frame as PNG
  - Press 'q': Quit application
"""

import os
from pathlib import Path
import cv2
import depthai as dai


def main():
    # Destination folder for captured images
    output_dir = Path("dataset_captured")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("Initializing OAK-D camera pipeline...")
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera)
        cam.build()

        video_out = cam.requestOutput(
            size=(1920, 1080),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.CROP,
            fps=30,
        )

        q_video = video_out.createOutputQueue(maxSize=4, blocking=False)
        pipeline.start()

        print("\n=======================================================")
        print(" OAK Camera Capture App Started!")
        print(f" Saving images to: {output_dir.resolve()}/")
        print(" Controls:")
        print("   [s] -> Save current frame")
        print("   [q] -> Quit")
        print("=======================================================\n")

        # Determine next image counter from existing files
        existing = list(output_dir.glob("oak_capture_*.png"))
        img_counter = len(existing)

        cv2.namedWindow("OAK Camera Capture App", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            "OAK Camera Capture App", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )

        try:
            while True:
                in_video = q_video.get()
                if in_video is not None:
                    frame = in_video.getCvFrame()
                    cv2.imshow("OAK Camera Capture App", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    filename = output_dir / f"oak_capture_{img_counter}.png"
                    cv2.imwrite(str(filename), frame)
                    print(f"✓ تم حفظ الصورة بنجاح: {filename}")
                    img_counter += 1
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
