import { PanelExtensionContext} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

function BatteryPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [batteryMsg, setBatteryMsg] = useState<any | null>(null);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const msgs = renderState.currentFrame ?? [];
      for (const msgEvent of msgs) {
        if (msgEvent.topic === "/chassis_info_fb") {
          setBatteryMsg(msgEvent.message);
        }
      }
    };

    context.watch("currentFrame");
    context.subscribe([{ topic: "/chassis_info_fb" }]);
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Datos de batería
  const voltage = batteryMsg?.bms_fb?.bms_fb_voltage ?? 0;
  const current = batteryMsg?.bms_fb?.bms_fb_current ?? 0;
  const soc = batteryMsg?.bms_flag_fb?.bms_flag_fb_soc ?? 0; // porcentaje
  const remaining = batteryMsg?.bms_fb?.bms_fb_remaining_capacity ?? 0;

  return (
    <div style={{ padding: "1rem", fontFamily: "sans-serif", color: "#eee", background: "#1a1a1a", height: "100%" }}>
      <h2>🔋 Estado de la batería</h2>
      {batteryMsg ? (
        <div style={{ marginTop: "1rem" }}>
          <p><b>Voltaje:</b> {voltage.toFixed(2)} V</p>
          <p><b>Corriente:</b> {current.toFixed(2)} A</p>
          <p><b>Capacidad restante:</b> {remaining.toFixed(2)}</p>
          <p><b>Estado de carga (SOC):</b> {soc}%</p>

          <div
            style={{
              width: "100%",
              height: "25px",
              background: "#333",
              borderRadius: "10px",
              overflow: "hidden",
              marginTop: "0.5rem",
              border: "1px solid #555",
            }}
          >
            <div
              style={{
                width: `${soc}%`,
                height: "100%",
                background:
                  soc < 20 ? "red" : soc < 50 ? "orange" : "limegreen",
                transition: "width 0.3s",
              }}
            />
          </div>
        </div>
      ) : (
        <p style={{ color: "#888" }}>Esperando mensajes de /chassis_info_fb…</p>
      )}
    </div>
  );
}

export function initBatteryPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<BatteryPanel context={context} />);
  return () => root.unmount();
}
