const elements = {
  cachedCount: document.getElementById("cached-count"),
  boundsSouth: document.getElementById("bounds-south"),
  boundsWest: document.getElementById("bounds-west"),
  boundsNorth: document.getElementById("bounds-north"),
  boundsEast: document.getElementById("bounds-east"),
  minZoom: document.getElementById("min-zoom"),
  maxZoom: document.getElementById("max-zoom"),
  tileCount: document.getElementById("tile-count"),
  tileSize: document.getElementById("tile-size"),
  useView: document.getElementById("use-view"),
  clearSelection: document.getElementById("clear-selection"),
  startDownload: document.getElementById("start-download"),
  cancelDownload: document.getElementById("cancel-download"),
  progressFill: document.getElementById("progress-fill"),
  progressText: document.getElementById("progress-text"),
  progressCount: document.getElementById("progress-count"),
  progressErrors: document.getElementById("progress-errors"),
  statusMessage: document.getElementById("status-message"),
  instructions: document.getElementById("map-instructions"),
};

const map = L.map("setup-map", { zoomControl: true }).setView([39.0, -105.0], 12);

L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: "&copy; OpenStreetMap contributors",
}).addTo(map);

const selection = {
  cornerA: null,
  cornerB: null,
  rectangle: null,
  markers: [],
  bounds: null,
};

const MAX_LAT = 85.0511;

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function formatCoord(value) {
  return value == null ? "" : value.toFixed(6);
}

function formatCount(value) {
  if (value == null) {
    return "--";
  }
  return value.toLocaleString();
}

function formatBytes(bytes) {
  if (!bytes || bytes <= 0) {
    return "--";
  }
  const units = ["B", "KB", "MB", "GB"];
  let idx = 0;
  let size = bytes;
  while (size >= 1024 && idx < units.length - 1) {
    size /= 1024;
    idx += 1;
  }
  return `${size.toFixed(1)} ${units[idx]}`;
}

function clearSelection() {
  selection.cornerA = null;
  selection.cornerB = null;
  selection.bounds = null;

  selection.markers.forEach((marker) => marker.remove());
  selection.markers = [];

  if (selection.rectangle) {
    selection.rectangle.remove();
    selection.rectangle = null;
  }

  elements.instructions.textContent = "Click to set the first corner";
  updateBoundsInputs();
  updateEstimate();
}

function updateBoundsInputs() {
  const bounds = selection.bounds;
  elements.boundsSouth.value = bounds ? formatCoord(bounds.getSouth()) : "";
  elements.boundsWest.value = bounds ? formatCoord(bounds.getWest()) : "";
  elements.boundsNorth.value = bounds ? formatCoord(bounds.getNorth()) : "";
  elements.boundsEast.value = bounds ? formatCoord(bounds.getEast()) : "";
}

function setSelectionFromBounds(bounds) {
  selection.bounds = bounds;
  selection.cornerA = bounds.getSouthWest();
  selection.cornerB = bounds.getNorthEast();

  selection.markers.forEach((marker) => marker.remove());
  selection.markers = [
    L.circleMarker(selection.cornerA, { radius: 6, color: "#2b7a65" }).addTo(map),
    L.circleMarker(selection.cornerB, { radius: 6, color: "#2b7a65" }).addTo(map),
  ];

  if (!selection.rectangle) {
    selection.rectangle = L.rectangle(bounds, { color: "#2b7a65", weight: 2, fillOpacity: 0.15 }).addTo(map);
  } else {
    selection.rectangle.setBounds(bounds);
  }

  elements.instructions.textContent = "Selection ready";
  updateBoundsInputs();
  updateEstimate();
}

function handleMapClick(event) {
  if (!selection.cornerA || (selection.cornerA && selection.cornerB)) {
    clearSelection();
    selection.cornerA = event.latlng;
    selection.markers.push(
      L.circleMarker(selection.cornerA, { radius: 6, color: "#2b7a65" }).addTo(map)
    );
    elements.instructions.textContent = "Click to set the opposite corner";
    return;
  }

  selection.cornerB = event.latlng;
  selection.markers.push(
    L.circleMarker(selection.cornerB, { radius: 6, color: "#2b7a65" }).addTo(map)
  );

  const bounds = L.latLngBounds(selection.cornerA, selection.cornerB);
  setSelectionFromBounds(bounds);
}

function latLonToTile(lat, lon, zoom) {
  const latClamped = clamp(lat, -MAX_LAT, MAX_LAT);
  const lonClamped = clamp(lon, -180, 180);
  const n = 2 ** zoom;
  const x = Math.floor(((lonClamped + 180) / 360) * n);
  const latRad = (latClamped * Math.PI) / 180;
  const y = Math.floor(
    ((1 - Math.log(Math.tan(latRad) + 1 / Math.cos(latRad)) / Math.PI) / 2) * n
  );
  return {
    x: clamp(x, 0, n - 1),
    y: clamp(y, 0, n - 1),
  };
}

function estimateTiles(bounds, minZoom, maxZoom) {
  if (!bounds) {
    return null;
  }
  if (Number.isNaN(minZoom) || Number.isNaN(maxZoom) || minZoom > maxZoom) {
    return null;
  }

  let total = 0;
  for (let zoom = minZoom; zoom <= maxZoom; zoom += 1) {
    const nw = latLonToTile(bounds.getNorth(), bounds.getWest(), zoom);
    const se = latLonToTile(bounds.getSouth(), bounds.getEast(), zoom);
    const xMin = Math.min(nw.x, se.x);
    const xMax = Math.max(nw.x, se.x);
    const yMin = Math.min(nw.y, se.y);
    const yMax = Math.max(nw.y, se.y);
    total += (xMax - xMin + 1) * (yMax - yMin + 1);
  }

  return total;
}

function updateEstimate() {
  const bounds = selection.bounds;
  const minZoom = parseInt(elements.minZoom.value, 10);
  const maxZoom = parseInt(elements.maxZoom.value, 10);
  const tileCount = estimateTiles(bounds, minZoom, maxZoom);

  if (tileCount == null) {
    elements.tileCount.textContent = "--";
    elements.tileSize.textContent = "--";
    return;
  }

  const estimatedBytes = tileCount * 35000;
  elements.tileCount.textContent = formatCount(tileCount);
  elements.tileSize.textContent = formatBytes(estimatedBytes);
}

async function startDownload() {
  if (!selection.bounds) {
    elements.statusMessage.textContent = "Select a map region before downloading.";
    return;
  }

  const minZoom = parseInt(elements.minZoom.value, 10);
  const maxZoom = parseInt(elements.maxZoom.value, 10);

  if (Number.isNaN(minZoom) || Number.isNaN(maxZoom) || minZoom > maxZoom) {
    elements.statusMessage.textContent = "Check the zoom range.";
    return;
  }

  const payload = {
    bounds: {
      south: selection.bounds.getSouth(),
      west: selection.bounds.getWest(),
      north: selection.bounds.getNorth(),
      east: selection.bounds.getEast(),
    },
    min_zoom: minZoom,
    max_zoom: maxZoom,
  };

  try {
    const response = await fetch("/api/map/download", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await response.json();
    if (!response.ok || !data.ok) {
      elements.statusMessage.textContent = data.error || "Download request failed.";
      return;
    }
    elements.statusMessage.textContent = "Download started.";
  } catch (error) {
    elements.statusMessage.textContent = "Download request failed.";
  }
}

async function cancelDownload() {
  try {
    await fetch("/api/map/cancel", { method: "POST" });
    elements.statusMessage.textContent = "Cancel requested.";
  } catch (error) {
    elements.statusMessage.textContent = "Cancel request failed.";
  }
}

function updateProgress(status) {
  const total = status.total || 0;
  const completed = status.completed || 0;
  const failed = status.failed || 0;
  const percent = total > 0 ? (completed / total) * 100 : 0;

  elements.progressFill.style.width = `${percent.toFixed(1)}%`;
  elements.progressCount.textContent = `${completed} / ${total}`;

  if (status.running) {
    elements.progressText.textContent = "Downloading";
  } else if (status.finished_at) {
    elements.progressText.textContent = status.canceled ? "Canceled" : "Complete";
  } else {
    elements.progressText.textContent = "Idle";
  }

  const errorText = failed > 0 ? `${failed} failed` : "";
  elements.progressErrors.textContent = status.last_error || errorText;
  elements.cachedCount.textContent = formatCount(status.cached_tiles);

  elements.startDownload.disabled = status.running;
  elements.cancelDownload.disabled = !status.running;
}

async function pollStatus() {
  try {
    const response = await fetch("/api/map/status");
    if (!response.ok) {
      return;
    }
    const data = await response.json();
    if (!data || !data.status) {
      return;
    }
    updateProgress(data.status);
  } catch (error) {
    // ignore transient network errors
  }
}

map.on("click", handleMapClick);

if (elements.useView) {
  elements.useView.addEventListener("click", () => {
    setSelectionFromBounds(map.getBounds());
  });
}

if (elements.clearSelection) {
  elements.clearSelection.addEventListener("click", clearSelection);
}

if (elements.startDownload) {
  elements.startDownload.addEventListener("click", startDownload);
}

if (elements.cancelDownload) {
  elements.cancelDownload.addEventListener("click", cancelDownload);
}

[elements.minZoom, elements.maxZoom].forEach((input) => {
  if (input) {
    input.addEventListener("input", updateEstimate);
  }
});

clearSelection();
updateEstimate();

pollStatus();
setInterval(pollStatus, 1500);
