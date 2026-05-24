import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type ImageMessage = {
  width: number;
  height: number;
  encoding?: string;
  data: ArrayLike<number>;
};

function CameraPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [imageMsg, setImageMsg] = useState<ImageMessage | null>(null);

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      for (const msgEvent of renderState.currentFrame ?? []) {
        if (msgEvent.topic === "/oak/rgb/image_raw") {
          setImageMsg(msgEvent.message as ImageMessage);
        }
      }
    };

    context.watch("currentFrame");

    context.subscribe([{ topic: "/oak/rgb/image_raw" }]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !imageMsg) {
      return;
    }

    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    const { width, height, encoding = "rgb8", data } = imageMsg;
    canvas.width = width;
    canvas.height = height;

    const imageData = ctx.createImageData(width, height);
    const source = data instanceof Uint8Array ? data : Uint8Array.from(data);

    if (encoding === "mono8") {
      for (let i = 0; i < width * height; i += 1) {
        const value = source[i] ?? 0;
        const pixelIndex = i * 4;
        imageData.data[pixelIndex] = value;
        imageData.data[pixelIndex + 1] = value;
        imageData.data[pixelIndex + 2] = value;
        imageData.data[pixelIndex + 3] = 255;
      }
    } else {
      const rgbOrder = encoding !== "bgr8" && encoding !== "bgra8";
      const bytesPerPixel = encoding === "rgba8" || encoding === "bgra8" ? 4 : 3;

      for (let i = 0; i < width * height; i += 1) {
        const sourceIndex = i * bytesPerPixel;
        const pixelIndex = i * 4;
        const first = source[sourceIndex] ?? 0;
        const second = source[sourceIndex + 1] ?? 0;
        const third = source[sourceIndex + 2] ?? 0;
        const alpha = bytesPerPixel === 4 ? source[sourceIndex + 3] ?? 255 : 255;

        imageData.data[pixelIndex] = rgbOrder ? first : third;
        imageData.data[pixelIndex + 1] = second;
        imageData.data[pixelIndex + 2] = rgbOrder ? third : first;
        imageData.data[pixelIndex + 3] = alpha;
      }
    }

    ctx.putImageData(imageData, 0, 0);
  }, [imageMsg]);

  return (
    <div
      style={{
        display: "flex",
        flexDirection: "column",
        gap: "0.75rem",
        height: "100%",
        padding: "0.75rem",
        background: "#101418",
        color: "#f5f7fa",
        boxSizing: "border-box",
      }}
    >
      <div>
        <h2 style={{ margin: 0, fontSize: "1.1rem" }}>Cámara OAK</h2>
        <div style={{ color: "#9aa4b2", fontSize: "0.9rem" }}>/oak/rgb/image_raw</div>
      </div>

      <div
        style={{
          flex: 1,
          minHeight: 0,
          border: "1px solid #263241",
          borderRadius: "12px",
          overflow: "hidden",
          background: "#000",
          display: "flex",
          alignItems: "center",
          justifyContent: "center",
        }}
      >
        {imageMsg ? (
          <canvas
            ref={canvasRef}
            style={{
              width: "100%",
              height: "100%",
              display: "block",
              objectFit: "contain",
            }}
          />
        ) : (
          <div style={{ color: "#9aa4b2", fontSize: "0.95rem", textAlign: "center" }}>
            Esperando imágenes de la cámara...
          </div>
        )}
      </div>
    </div>
  );
}

export function initCameraPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CameraPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
