import { PanelExtensionContext } from "@foxglove/extension";
import { useRef, useEffect, ReactElement } from "react";
import { createRoot } from "react-dom/client";

import L from "leaflet";
import "leaflet/dist/leaflet.css";

type Waypoint = {
  latitude: number;
  longitude: number;
  yaw: number;
};

function Nav2OutdoorPanel({
  context,
}: {
  context: PanelExtensionContext;
}): ReactElement {
  const mapRef = useRef<L.Map | null>(null);

  const waypointsRef = useRef<Waypoint[]>([]);
  const markerLayerRef = useRef<L.LayerGroup | null>(null);
  const gpsMarkerRef = useRef<L.CircleMarker | null>(null);

  useEffect(() => {
    context.watch("currentFrame");
    context.watch("topics");

    const container = context.panelElement;

    const mapContainer = document.createElement("div");
    mapContainer.style.width = "100%";
    mapContainer.style.height = "100vh";
    container.appendChild(mapContainer);

    // =========================
    // MAPA
    // =========================
    const map = L.map(mapContainer, {
      center: [39.48268, -0.34583],
      zoom: 19,
    });

    mapRef.current = map;
    // Cursor tipo puntero preciso
    map.getContainer().style.cursor = "default";
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

    gpsMarkerRef.current = L.circleMarker(
      [39.48268, -0.34583],
      {
        radius: 5,
        color: "grey",
        fillColor: "grey",
        fillOpacity: 0.9,
      }
    ).addTo(map);

    markerLayerRef.current = L.layerGroup().addTo(map);

    // =========================
    // ADVERTISE TOPIC
    // =========================
    context.advertise?.("/goals", "std_msgs/String");

    // =========================
    // CLICK MAP
    // =========================
    map.on("click", (e: L.LeafletMouseEvent) => {
      const lat = e.latlng.lat;
      const lng = e.latlng.lng;

      const popupContent = `
        <div style="min-width:220px">
          <h3>Waypoint</h3>
          <p>Lat: ${lat.toFixed(6)}</p>
          <p>Lng: ${lng.toFixed(6)}</p>

          <button id="add_wp">Añadir waypoint</button>
          <button id="finish_wp">Publicar ruta</button>
        </div>
      `;

      L.popup()
        .setLatLng(e.latlng)
        .setContent(popupContent)
        .openOn(map);

      setTimeout(() => {
        const addBtn = document.getElementById("add_wp");
        const finishBtn = document.getElementById("finish_wp");

        // =========================
        // ADD WAYPOINT
        // =========================
        addBtn?.addEventListener("click", () => {
          waypointsRef.current.push({
            latitude: lat,
            longitude: lng,
            yaw: 0.0,
          });

          L.circleMarker(e.latlng, {
            radius: 5,
            color: "green",
            fillColor: "green",
            fillOpacity: 1,
          }).addTo(markerLayerRef.current!);

          map.closePopup();
        });

        // =========================
        // PUBLISH WAYPOINTS
        // =========================
        finishBtn?.addEventListener("click", () => {
          waypointsRef.current.push({
            latitude: lat,
            longitude: lng,
            yaw: 0.0,
          });

          L.circleMarker(e.latlng, {
            radius: 5,
            color: "green",
            fillColor: "green",
            fillOpacity: 1,
          }).addTo(markerLayerRef.current!);

          if (waypointsRef.current.length === 0) {
            map.closePopup();
            return;
          }

          const json = JSON.stringify(waypointsRef.current);

          context.publish?.("/goals", {
            data: json,
          });

          // reset
          waypointsRef.current = [];
          // markerLayerRef.current?.clearLayers();

          map.closePopup();
        });
      }, 0);
    });

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

export function initNav2OutdoorPanel(
  context: PanelExtensionContext
): () => void {
  const root = createRoot(context.panelElement);

  root.render(<Nav2OutdoorPanel context={context} />);

  return () => root.unmount();
}
