const elements = {
  connDot: document.getElementById("conn-dot"),
  connStatus: document.getElementById("conn-status"),
  packetCount: document.getElementById("packet-count"),
  lastUpdate: document.getElementById("last-update"),
  callsign: document.getElementById("callsign"),
  crcStatus: document.getElementById("crc-status"),
  radioRssi: document.getElementById("radio-rssi"),
  radioSnr: document.getElementById("radio-snr"),
  radioPayload: document.getElementById("radio-payload"),
  radioAscii: document.getElementById("radio-ascii"),
  radioHex: document.getElementById("radio-hex"),
  gpsLat: document.getElementById("gps-lat"),
  gpsLon: document.getElementById("gps-lon"),
  gpsAlt: document.getElementById("gps-alt"),
  gpsTime: document.getElementById("gps-time"),
  gpsLatDeg: document.getElementById("gps-lat-deg"),
  gpsLonDeg: document.getElementById("gps-lon-deg"),
  gpsLatE7: document.getElementById("gps-lat-e7"),
  gpsLonE7: document.getElementById("gps-lon-e7"),
  gpsAltM: document.getElementById("gps-alt-m"),
  gpsAltMm: document.getElementById("gps-alt-mm"),
  altTime: document.getElementById("alt-time"),
  altPressPa: document.getElementById("alt-press-pa"),
  altPressKpa: document.getElementById("alt-press-kpa"),
  altTempC: document.getElementById("alt-temp-c"),
  altPressX10: document.getElementById("alt-press-x10"),
  altTempX100: document.getElementById("alt-temp-x100"),
  imuTime: document.getElementById("imu-time"),
  imuTimeRaw: document.getElementById("imu-time-raw"),
  imuGyro: document.getElementById("imu-gyro"),
  imuAccel: document.getElementById("imu-accel"),
  imuGyroRaw: document.getElementById("imu-gyro-raw"),
  imuAccelRaw: document.getElementById("imu-accel-raw"),
  batTime: document.getElementById("bat-time"),
  batVoltage: document.getElementById("bat-voltage"),
  batVoltageMv: document.getElementById("bat-voltage-mv"),
  batState: document.getElementById("bat-state"),
  attRoll: document.getElementById("att-roll"),
  attPitch: document.getElementById("att-pitch"),
  attYaw: document.getElementById("att-yaw"),
  mapOverlay: document.getElementById("map-overlay"),
};

const rocketCanvas = document.getElementById("rocket-canvas");
const mapCanvas = document.getElementById("map-canvas");
const rocketCtx = rocketCanvas.getContext("2d");
const mapCtx = mapCanvas.getContext("2d");

const state = {
  lastUpdateMs: 0,
  connected: false,
  orientation: { roll: 0, pitch: 0, yaw: 0 },
  lastImuTime: null,
  mapOrigin: null,
  mapPath: [],
};

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

const rocketModel = (() => {
  const points = [];
  const edges = [];
  const ring = (z, r, count) => {
    const start = points.length;
    for (let i = 0; i < count; i += 1) {
      const angle = (i / count) * Math.PI * 2;
      points.push([Math.cos(angle) * r, Math.sin(angle) * r, z]);
    }
    for (let i = 0; i < count; i += 1) {
      edges.push([start + i, start + ((i + 1) % count)]);
    }
    return start;
  };

  const base = ring(-1.0, 0.25, 10);
  const top = ring(0.6, 0.2, 10);
  const fin = ring(-0.7, 0.32, 4);
  const noseIndex = points.length;
  points.push([0, 0, 1.2]);

  for (let i = 0; i < 10; i += 1) {
    edges.push([base + i, top + i]);
    edges.push([top + i, noseIndex]);
  }

  for (let i = 0; i < 4; i += 1) {
    edges.push([fin + i, base + i * 2]);
  }

  edges.push([base, base + 5]);
  edges.push([base + 2, base + 7]);

  return { points, edges };
})();

function formatNumber(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "--";
  }
  return Number(value).toFixed(digits);
}

function formatInt(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "--";
  }
  return Math.round(value).toString();
}

function updateConnection(connected) {
  state.connected = connected;
  elements.connDot.classList.toggle("connected", connected);
  elements.connDot.classList.toggle("disconnected", !connected);
  elements.connStatus.textContent = connected ? "Live" : "Waiting";
}

function updateFromTelemetry(snapshot) {
  state.lastUpdateMs = Date.now();
  updateConnection(true);

  elements.packetCount.textContent = formatInt(snapshot.packet_count || 0);
  elements.callsign.textContent = snapshot.callsign || "--";

  const radio = snapshot.radio || {};
  elements.crcStatus.textContent = radio.crc_ok === null ? "--" : (radio.crc_ok ? "OK" : "BAD");
  elements.radioRssi.textContent = radio.rssi_dbm !== null ? `${radio.rssi_dbm} dBm` : "--";
  elements.radioSnr.textContent = radio.snr_db !== null ? `${formatNumber(radio.snr_db, 1)} dB` : "--";
  elements.radioPayload.textContent = radio.payload_len !== null ? `${radio.payload_len} bytes` : "--";
  elements.radioAscii.textContent = radio.raw_ascii || "--";
  elements.radioHex.textContent = radio.raw_hex || "--";

  const gps = snapshot.gps || {};
  elements.gpsTime.textContent = gps.t_ms !== null && gps.t_ms !== undefined ? `${gps.t_ms} ms` : "--";
  elements.gpsLat.textContent = gps.lat_deg !== null && gps.lat_deg !== undefined ? formatNumber(gps.lat_deg, 6) : "--";
  elements.gpsLon.textContent = gps.lon_deg !== null && gps.lon_deg !== undefined ? formatNumber(gps.lon_deg, 6) : "--";
  elements.gpsAlt.textContent = gps.alt_m !== null && gps.alt_m !== undefined ? `${formatNumber(gps.alt_m, 1)} m` : "--";
  elements.gpsLatDeg.textContent = elements.gpsLat.textContent;
  elements.gpsLonDeg.textContent = elements.gpsLon.textContent;
  elements.gpsLatE7.textContent = gps.lat_e7 ?? "--";
  elements.gpsLonE7.textContent = gps.lon_e7 ?? "--";
  elements.gpsAltM.textContent = gps.alt_m !== null && gps.alt_m !== undefined ? formatNumber(gps.alt_m, 2) : "--";
  elements.gpsAltMm.textContent = gps.height_mm ?? "--";

  const alt = snapshot.alt || {};
  elements.altTime.textContent = alt.t_ms !== null && alt.t_ms !== undefined ? `${alt.t_ms} ms` : "--";
  elements.altPressPa.textContent = alt.press_pa !== null && alt.press_pa !== undefined ? formatNumber(alt.press_pa, 1) : "--";
  elements.altPressKpa.textContent = alt.press_kpa !== null && alt.press_kpa !== undefined ? formatNumber(alt.press_kpa, 3) : "--";
  elements.altTempC.textContent = alt.temp_c !== null && alt.temp_c !== undefined ? formatNumber(alt.temp_c, 2) : "--";
  elements.altPressX10.textContent = alt.press_pa_x10 ?? "--";
  elements.altTempX100.textContent = alt.temp_c_x100 ?? "--";

  const imu = snapshot.imu || {};
  elements.imuTime.textContent = imu.t_ms !== null && imu.t_ms !== undefined ? `${imu.t_ms} ms` : "--";
  elements.imuTimeRaw.textContent = elements.imuTime.textContent;
  elements.imuGyro.textContent = imu.gx_dps !== null && imu.gx_dps !== undefined
    ? `${formatNumber(imu.gx_dps, 1)}, ${formatNumber(imu.gy_dps, 1)}, ${formatNumber(imu.gz_dps, 1)}`
    : "--";
  elements.imuAccel.textContent = imu.ax_g !== null && imu.ax_g !== undefined
    ? `${formatNumber(imu.ax_g, 2)}, ${formatNumber(imu.ay_g, 2)}, ${formatNumber(imu.az_g, 2)}`
    : "--";
  elements.imuGyroRaw.textContent = imu.gx !== null && imu.gx !== undefined
    ? `${imu.gx}, ${imu.gy}, ${imu.gz}`
    : "--";
  elements.imuAccelRaw.textContent = imu.ax !== null && imu.ax !== undefined
    ? `${imu.ax}, ${imu.ay}, ${imu.az}`
    : "--";

  const battery = snapshot.battery || {};
  elements.batTime.textContent = battery.t_ms !== null && battery.t_ms !== undefined ? `${battery.t_ms} ms` : "--";
  elements.batVoltage.textContent = battery.vbat_v !== null && battery.vbat_v !== undefined
    ? formatNumber(battery.vbat_v, 2)
    : "--";
  elements.batVoltageMv.textContent = battery.vbat_mv ?? "--";
  elements.batState.textContent = battery.bat_state_label || "--";

  updateOrientation(imu);
  updateMap(gps);
}

function updateOrientation(imu) {
  if (!imu || imu.gx === null || imu.ax === null) {
    return;
  }

  const nowMs = imu.t_ms !== null && imu.t_ms !== undefined ? imu.t_ms : Date.now();
  let dt = 0.02;
  if (state.lastImuTime !== null) {
    dt = (nowMs - state.lastImuTime) / 1000;
    if (!Number.isFinite(dt) || dt <= 0 || dt > 1) {
      dt = 0.02;
    }
  }
  state.lastImuTime = nowMs;

  const gx = imu.gx_dps ?? (imu.gx / 10);
  const gy = imu.gy_dps ?? (imu.gy / 10);
  const gz = imu.gz_dps ?? (imu.gz / 10);

  state.orientation.roll += gx * dt;
  state.orientation.pitch += gy * dt;
  state.orientation.yaw += gz * dt;

  const ax = imu.ax_g ?? (imu.ax / 1000);
  const ay = imu.ay_g ?? (imu.ay / 1000);
  const az = imu.az_g ?? (imu.az / 1000);

  if (ax !== null && ay !== null && az !== null) {
    const rollAcc = Math.atan2(ay, az) * RAD_TO_DEG;
    const pitchAcc = Math.atan2(-ax, Math.sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    const alpha = 0.96;
    state.orientation.roll = alpha * state.orientation.roll + (1 - alpha) * rollAcc;
    state.orientation.pitch = alpha * state.orientation.pitch + (1 - alpha) * pitchAcc;
  }

  elements.attRoll.textContent = `${formatNumber(state.orientation.roll, 1)}°`;
  elements.attPitch.textContent = `${formatNumber(state.orientation.pitch, 1)}°`;
  elements.attYaw.textContent = `${formatNumber(state.orientation.yaw, 1)}°`;
}

function resizeCanvas(canvas, ctx) {
  const { width, height } = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
}

function rotatePoint(point, roll, pitch, yaw) {
  const [x, y, z] = point;
  const r = roll * DEG_TO_RAD;
  const p = pitch * DEG_TO_RAD;
  const yw = yaw * DEG_TO_RAD;

  const cosR = Math.cos(r);
  const sinR = Math.sin(r);
  const cosP = Math.cos(p);
  const sinP = Math.sin(p);
  const cosY = Math.cos(yw);
  const sinY = Math.sin(yw);

  const x1 = x * cosR - y * sinR;
  const y1 = x * sinR + y * cosR;
  const z1 = z;

  const y2 = y1 * cosP - z1 * sinP;
  const z2 = y1 * sinP + z1 * cosP;
  const x2 = x1;

  const z3 = z2 * cosY - x2 * sinY;
  const x3 = z2 * sinY + x2 * cosY;
  const y3 = y2;

  return [x3, y3, z3];
}

function renderRocket() {
  resizeCanvas(rocketCanvas, rocketCtx);
  const { width, height } = rocketCanvas.getBoundingClientRect();
  rocketCtx.clearRect(0, 0, width, height);

  const centerX = width / 2;
  const centerY = height / 2 + 20;
  const scale = Math.min(width, height) * 0.32;
  const camera = 3.5;

  const { roll, pitch, yaw } = state.orientation;
  const transformed = rocketModel.points.map((point) => rotatePoint(point, roll, pitch, yaw));

  rocketCtx.lineWidth = 2;
  rocketCtx.strokeStyle = "rgba(255, 255, 255, 0.85)";
  rocketCtx.beginPath();
  rocketModel.edges.forEach(([a, b]) => {
    const pa = transformed[a];
    const pb = transformed[b];
    const paScale = camera / (camera - pa[2]);
    const pbScale = camera / (camera - pb[2]);
    const ax = centerX + pa[0] * scale * paScale;
    const ay = centerY - pa[1] * scale * paScale;
    const bx = centerX + pb[0] * scale * pbScale;
    const by = centerY - pb[1] * scale * pbScale;
    rocketCtx.moveTo(ax, ay);
    rocketCtx.lineTo(bx, by);
  });
  rocketCtx.stroke();

  rocketCtx.strokeStyle = "rgba(46, 210, 165, 0.8)";
  rocketCtx.lineWidth = 3;
  rocketCtx.beginPath();
  rocketCtx.moveTo(centerX, centerY);
  rocketCtx.lineTo(centerX, centerY - scale * 0.9);
  rocketCtx.stroke();

  requestAnimationFrame(renderRocket);
}

function updateMap(gps) {
  if (!gps || gps.lat_deg === null || gps.lon_deg === null) {
    elements.mapOverlay.textContent = "Awaiting GPS fix";
    return;
  }

  elements.mapOverlay.textContent = "";

  if (!state.mapOrigin) {
    state.mapOrigin = { lat: gps.lat_deg, lon: gps.lon_deg };
  }

  const originLat = state.mapOrigin.lat;
  const originLon = state.mapOrigin.lon;
  const dLat = (gps.lat_deg - originLat) * DEG_TO_RAD;
  const dLon = (gps.lon_deg - originLon) * DEG_TO_RAD;
  const earthRadius = 6371000;
  const x = dLon * Math.cos(originLat * DEG_TO_RAD) * earthRadius;
  const y = dLat * earthRadius;

  state.mapPath.push({ x, y });
  if (state.mapPath.length > 200) {
    state.mapPath.shift();
  }

  renderMap();
}

function renderMap() {
  resizeCanvas(mapCanvas, mapCtx);
  const { width, height } = mapCanvas.getBoundingClientRect();
  mapCtx.clearRect(0, 0, width, height);

  mapCtx.fillStyle = "rgba(255, 255, 255, 0.04)";
  mapCtx.fillRect(0, 0, width, height);

  if (state.mapPath.length === 0) {
    return;
  }

  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  state.mapPath.forEach((pt) => {
    minX = Math.min(minX, pt.x);
    maxX = Math.max(maxX, pt.x);
    minY = Math.min(minY, pt.y);
    maxY = Math.max(maxY, pt.y);
  });

  const padding = 40;
  const spanX = Math.max(10, maxX - minX);
  const spanY = Math.max(10, maxY - minY);
  const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);

  const centerX = width / 2;
  const centerY = height / 2;

  mapCtx.strokeStyle = "rgba(255, 255, 255, 0.15)";
  mapCtx.lineWidth = 1;
  const gridStep = 50;
  for (let dx = -500; dx <= 500; dx += gridStep) {
    mapCtx.beginPath();
    mapCtx.moveTo(centerX + dx * scale, 0);
    mapCtx.lineTo(centerX + dx * scale, height);
    mapCtx.stroke();
  }
  for (let dy = -500; dy <= 500; dy += gridStep) {
    mapCtx.beginPath();
    mapCtx.moveTo(0, centerY + dy * scale);
    mapCtx.lineTo(width, centerY + dy * scale);
    mapCtx.stroke();
  }

  mapCtx.strokeStyle = "rgba(46, 210, 165, 0.9)";
  mapCtx.lineWidth = 2;
  mapCtx.beginPath();
  state.mapPath.forEach((pt, index) => {
    const x = centerX + (pt.x - (minX + maxX) / 2) * scale;
    const y = centerY - (pt.y - (minY + maxY) / 2) * scale;
    if (index === 0) {
      mapCtx.moveTo(x, y);
    } else {
      mapCtx.lineTo(x, y);
    }
  });
  mapCtx.stroke();

  const latest = state.mapPath[state.mapPath.length - 1];
  const markerX = centerX + (latest.x - (minX + maxX) / 2) * scale;
  const markerY = centerY - (latest.y - (minY + maxY) / 2) * scale;
  mapCtx.fillStyle = "#2ed2a5";
  mapCtx.beginPath();
  mapCtx.arc(markerX, markerY, 6, 0, Math.PI * 2);
  mapCtx.fill();

  mapCtx.fillStyle = "rgba(255, 255, 255, 0.8)";
  mapCtx.font = "12px 'Space Mono', monospace";
  mapCtx.fillText("N", centerX + 8, centerY - 8);
}

function updateClock() {
  if (state.lastUpdateMs === 0) {
    elements.lastUpdate.textContent = "--";
    updateConnection(false);
    return;
  }

  const elapsed = Math.max(0, Date.now() - state.lastUpdateMs);
  if (elapsed > 4000) {
    updateConnection(false);
  }
  elements.lastUpdate.textContent = `${(elapsed / 1000).toFixed(1)}s ago`;
}

function initEventSource() {
  fetch("/api/state")
    .then((res) => res.json())
    .then((payload) => updateFromTelemetry(payload.state))
    .catch(() => {});

  const source = new EventSource("/events");
  source.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      updateFromTelemetry(payload.state);
    } catch (err) {
      console.warn("Telemetry parse error", err);
    }
  };

  source.onerror = () => {
    updateConnection(false);
  };
}

window.addEventListener("resize", () => {
  renderMap();
});

setInterval(updateClock, 500);

renderRocket();
initEventSource();
