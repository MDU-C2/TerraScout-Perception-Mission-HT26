#!/usr/bin/env python3
import cv2
import depthai as dai

# 1. Initialize Pipeline
with dai.Pipeline() as pipeline:

  # 2. Create the unified Camera Node
  cam = pipeline.create(dai.node.Camera)
  cam.build()

  # 3. Request the stream (1080p بدقة كاملة)
  video_out = cam.requestOutput(
    size=(1920, 1080),
    type=dai.ImgFrame.Type.BGR888p,
    resizeMode=dai.ImgResizeMode.CROP,
    fps=30,
  )

  # 4. Create the output queue BEFORE starting the pipeline
  q_video = video_out.createOutputQueue(maxSize=4, blocking=False)

  # 5. Start the pipeline
  pipeline.start()

  print("App Started. Press 's' to save an image, 'q' to quit.")
  img_counter = 0

  cv2.namedWindow("OAK Camera Capture App", cv2.WINDOW_NORMAL)
  cv2.setWindowProperty(
    "OAK Camera Capture App", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
  )

  while True:
    in_video = q_video.get()
    if in_video is not None:
      frame = in_video.getCvFrame()
      cv2.imshow("OAK Camera Capture App", frame)

    key = cv2.waitKey(1)
    if key == ord("q"):
      break
    elif key == ord("s"):
      filename = f"oak_capture_{img_counter}.png"
      cv2.imwrite(filename, frame)
      print(f"image saved: {filename}")
      img_counter += 1
