import { PanelExtensionContext } from "@foxglove/extension";
import { useRef, useEffect, ReactElement } from "react";
import { createRoot } from "react-dom/client";

import L from "leaflet";
import "leaflet/dist/leaflet.css";

function ENavOutdoorPanel({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const mapRef = useRef<L.Map | null>(null);
  const gpsMarkerRef = useRef<L.CircleMarker | null>(null);
  const goalMarkerRef = useRef<L.CircleMarker | null>(null);
  const routeLineRef = useRef<L.Polyline | null>(null);

  useEffect(() => {
    context.watch("currentFrame");
    context.watch("topics");

    const container = context.panelElement;

    // Contenedor del mapa
    const mapContainer = document.createElement("div");
    mapContainer.style.width = "100%";
    mapContainer.style.height = "100vh";

    container.appendChild(mapContainer);

    // =========================
    // CREAR MAPA
    // =========================
    const map = L.map(mapContainer, {
      center: [39.48268, -0.34583],
      zoom: 19,
      maxZoom: 25,
      minZoom: 1,
      zoomControl: true,
    });

    mapRef.current = map;

    // Cursor tipo puntero preciso
    map.getContainer().style.cursor = "default";

    // =========================
    // MAPBOX
    // =========================
    const MAPBOX_ACCESS_TOKEN = 'pk.eyJ1IjoiYWkybGFiIiwiYSI6ImNtYjZmdmp5MTAwZzcybXF5cXFhajMxazAifQ.mNSVi5QsLPJSnFG2FU3LWA';

    L.tileLayer(
      `https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token=${MAPBOX_ACCESS_TOKEN}`,
      {
        maxZoom: 25,
        tileSize: 512,
        zoomOffset: -1,
        id: "mapbox/streets-v11",
        attribution:
          '© <a href="https://www.mapbox.com/about/maps/">Mapbox</a> © OpenStreetMap',
      }
    ).addTo(map);

    // =========================
    // MARCADOR GPS ACTUAL
    // =========================
    gpsMarkerRef.current = L.circleMarker(
      [39.48268, -0.34583],
      {
        radius: 5,
        color: "grey",
        fillColor: "grey",
        fillOpacity: 0.9,
      }
    ).addTo(map);
    // =========================
    // RECORRIDO ROJO
    // =========================
    routeLineRef.current = L.polyline([], {
      color: "red",
      weight: 2,
      opacity: 1.0,
    }).addTo(map);


    // =========================
    // ADVERTISE TOPIC
    // =========================
    context.advertise?.("/goal_latlong", "sensor_msgs/NavSatFix");

    // =========================
    // CLICK EN MAPA
    // =========================
    map.on("click", (e: L.LeafletMouseEvent) => {
      const lat = e.latlng.lat;
      const lng = e.latlng.lng;

      const popupContent = `
        <div style="min-width:200px">
          <h3>Goal</h3>
          <p><b>Lat:</b> ${lat.toFixed(6)}</p>
          <p><b>Lng:</b> ${lng.toFixed(6)}</p>

          <button id="btn-yes">Sí</button>
          <button id="btn-no">No</button>
        </div>
      `;

      L.popup()
        .setLatLng(e.latlng)
        .setContent(popupContent)
        .openOn(map);

      setTimeout(() => {
        const yesBtn = document.getElementById("btn-yes");
        const noBtn = document.getElementById("btn-no");

        yesBtn?.addEventListener("click", () => {
          // Crear círculo verde
          if (goalMarkerRef.current) {
            map.removeLayer(goalMarkerRef.current);
          }

          goalMarkerRef.current = L.circleMarker(e.latlng, {
            radius: 3,
            color: "green",
            fillColor: "green",
            fillOpacity: 1.0,
          }).addTo(map);

          // Publicar NavSatFix
          context.publish?.("/goal_latlong", {
            header: {
              seq: 0,
              stamp: {
                sec: Math.floor(Date.now() / 1000),
                nsec: 0,
              },
              frame_id: "gps",
            },

            status: {
              status: 0,
              service: 1,
            },

            latitude: lat,
            longitude: lng,
            altitude: 0,

            position_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],

            position_covariance_type: 0,
          });

          map.closePopup();
        });

        noBtn?.addEventListener("click", () => {
          map.closePopup();
        });
      }, 0);
    });

    // =========================
    // SUBSCRIPCIÓN GPS
    // =========================
    context.subscribe([{ topic: "/fixposition/odometry_llh" }]);

    // =========================
    // RENDER
    // =========================
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msgEvent of renderState.currentFrame) {
          const topic = msgEvent.topic;
          const msg = msgEvent.message as any;

          if (topic === "/fixposition/odometry_llh" && gpsMarkerRef.current) {
            const { latitude, longitude } = msg;

            if (
              typeof latitude === "number" &&
              typeof longitude === "number"
            ) {
              gpsMarkerRef.current.setLatLng([
                latitude,
                longitude,
              ]);
              // Añadir punto al recorrido rojo
              routeLineRef.current?.addLatLng([
                latitude,
                longitude,
              ]);
            }
          }
        }
      }

      done();
    };

    // =========================
    // CLEANUP
    // =========================
    return () => {
      map.remove();

      if (container.contains(mapContainer)) {
        container.removeChild(mapContainer);
      }
    };
  }, [context]);

  return <></>;
}

export function initENavOutdoorPanel(
  context: PanelExtensionContext
): () => void {
  const root = createRoot(context.panelElement);

  root.render(<ENavOutdoorPanel context={context} />);

  return () => {
    root.unmount();
  };
}
