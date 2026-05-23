import sys

try:
    import numpy as np
    import cv2
    import depthai as dai
    print(f"Versión de NumPy: {np.__version__}")
    print(f"Versión de OpenCV: {cv2.__version__}")
except ImportError as e:
    print(f"Error al importar librerías: {e}")
    sys.exit(1)

# Configuración del Pipeline de la OAK-D
pipeline = dai.Pipeline()

cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(640, 480)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("video")
cam_rgb.preview.link(xout_rgb.input)

# Ejecución
try:
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="video", maxSize=4, blocking=False)
        print("Cámara iniciada. Pulsa 'q' en la ventana de vídeo para salir.")
        
        while True:
            in_rgb = q_rgb.get()
            frame = in_rgb.getCvFrame()
            cv2.imshow("FWMini - OAK-D S2", frame)

            if cv2.waitKey(1) == ord('q'):
                break
except Exception as e:
    print(f"Error de hardware o conexión: {e}")
finally:
    cv2.destroyAllWindows()
