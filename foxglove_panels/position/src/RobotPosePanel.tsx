import {PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

const POSE_TOPICS = ["/amcl_pose", "/localizer_node/costmap/pose", "/odom", "/odometry/filtered"] as const;

// Función auxiliar para convertir cuaterniones a ángulos de Euler (yaw, pitch, roll)
function quaternionToEuler(x: number, y: number, z: number, w: number) {
  const ysqr = y * y;

  // roll (x-axis rotation)
  const t0 = +2.0 * (w * x + y * z);
  const t1 = +1.0 - 2.0 * (x * x + ysqr);
  const roll = Math.atan2(t0, t1);

  // pitch (y-axis rotation)
  let t2 = +2.0 * (w * y - z * x);
  t2 = t2 > +1.0 ? +1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  const pitch = Math.asin(t2);

  // yaw (z-axis rotation)
  const t3 = +2.0 * (w * z + x * y);
  const t4 = +1.0 - 2.0 * (ysqr + z * z);
  const yaw = Math.atan2(t3, t4);

  return { roll, pitch, yaw };
}

function RobotPosePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [poseMsg, setPoseMsg] = useState<any | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const msgs = renderState.currentFrame ?? [];
      for (const msgEvent of msgs) {
        if (POSE_TOPICS.includes(msgEvent.topic as typeof POSE_TOPICS[number])) {
          setPoseMsg(msgEvent.message);
        }
      }
    };

    context.watch("currentFrame");
    context.subscribe(POSE_TOPICS.map((topic) => ({ topic })));
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const position = poseMsg?.pose?.pose?.position;
  const orientation = poseMsg?.pose?.pose?.orientation;
  const { roll, pitch, yaw } = orientation
    ? quaternionToEuler(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      )
    : { roll: 0, pitch: 0, yaw: 0 };

  return (
    <div
      style={{
        padding: "1rem",
        fontFamily: "monospace",
        color: "#eee",
        background: "#1a1a1a",
        height: "100%",
      }}
    >
      <h2>📍 Pose del Robot</h2>
      {poseMsg ? (
        <div style={{ marginTop: "1rem" }}>
          <p>
            <b>Posición:</b>
            <br />
            x: {position?.x.toFixed(3)}<br />
            y: {position?.y.toFixed(3)}<br />
            z: {position?.z.toFixed(3)}
          </p>
          <p>
            <b>Orientación (Cuaternión):</b>
            <br />
            x: {orientation?.x.toFixed(3)}<br />
            y: {orientation?.y.toFixed(3)}<br />
            z: {orientation?.z.toFixed(3)}<br />
            w: {orientation?.w.toFixed(3)}
          </p>
          <p>
            <b>Orientación (Euler):</b>
            <br />
            Roll: {(roll * 180 / Math.PI).toFixed(1)}°<br />
            Pitch: {(pitch * 180 / Math.PI).toFixed(1)}°<br />
            Yaw: {(yaw * 180 / Math.PI).toFixed(1)}°
          </p>
        </div>
      ) : (
        <p style={{ color: "#888" }}>Esperando mensajes de pose…</p>
      )}
    </div>
  );
}

export function initRobotPosePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RobotPosePanel context={context} />);
  return () => root.unmount();
}
