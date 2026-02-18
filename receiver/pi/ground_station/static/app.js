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
  radioLastPacket: document.getElementById("radio-last-packet"),
  gpsLat: document.getElementById("gps-lat"),
  gpsLon: document.getElementById("gps-lon"),
  gpsAlt: document.getElementById("gps-alt"),
  gpsTime: document.getElementById("gps-time"),
  gpsLatDeg: document.getElementById("gps-lat-deg"),
  gpsLonDeg: document.getElementById("gps-lon-deg"),
  gpsAltM: document.getElementById("gps-alt-m"),
  navsatTime: document.getElementById("navsat-time"),
  navsatSvsUsed: document.getElementById("navsat-svs-used"),
  navsatSvsTotal: document.getElementById("navsat-svs-total"),
  navsatCnoMax: document.getElementById("navsat-cno-max"),
  navsatCnoAvg: document.getElementById("navsat-cno-avg"),
  altTime: document.getElementById("alt-time"),
  altPressPa: document.getElementById("alt-press-pa"),
  altPressKpa: document.getElementById("alt-press-kpa"),
  altTempC: document.getElementById("alt-temp-c"),
  recoveryMode: document.getElementById("recovery-mode"),
  recoveryPhase: document.getElementById("recovery-phase"),
  recoveryAgl: document.getElementById("recovery-agl"),
  recoveryVspeed: document.getElementById("recovery-vspeed"),
  recoveryDrogue: document.getElementById("recovery-drogue"),
  recoveryMain: document.getElementById("recovery-main"),
  imuTime: document.getElementById("imu-time"),
  imuTimeRaw: document.getElementById("imu-time-raw"),
  imuGyro: document.getElementById("imu-gyro"),
  imuAccel: document.getElementById("imu-accel"),
  batTime: document.getElementById("bat-time"),
  batVoltage: document.getElementById("bat-voltage"),
  batState: document.getElementById("bat-state"),
  vmStatus: document.getElementById("vm-status"),
  vmUpdated: document.getElementById("vm-updated"),
  vmVin: document.getElementById("vm-vin"),
  vmVout: document.getElementById("vm-vout"),
  vmVbatt: document.getElementById("vm-vbatt"),
  vmTempC: document.getElementById("vm-temp-c"),
  vmTempF: document.getElementById("vm-temp-f"),
  vmWarning: document.getElementById("vm-warning"),
  attRoll: document.getElementById("att-roll"),
  attPitch: document.getElementById("att-pitch"),
  attYaw: document.getElementById("att-yaw"),
  attitudeFilter: document.getElementById("attitude-filter"),
  attitudeThreshold: document.getElementById("attitude-threshold"),
  mapOverlay: document.getElementById("map-overlay"),
  mapJump: document.getElementById("map-jump"),
  mapSetHome: document.getElementById("map-set-home"),
  sdStart: document.getElementById("sd-start"),
  sdStop: document.getElementById("sd-stop"),
  sdStatus: document.getElementById("sd-status"),
  telemEnable: document.getElementById("telem-enable"),
  telemDisable: document.getElementById("telem-disable"),
  telemStatus: document.getElementById("telem-status"),
  buzzerDuration: document.getElementById("buzzer-duration"),
  buzzerActivate: document.getElementById("buzzer-activate"),
  buzzerStatus: document.getElementById("buzzer-status"),
  telemetryWaiting: document.getElementById("telemetry-waiting"),
};

const rocketCanvas = document.getElementById("rocket-canvas");
const mapContainer = document.getElementById("map");
const rocketCtx = rocketCanvas.getContext("2d");

const state = {
  lastUpdateMs: 0,
  connected: false,
  orientation: { roll: 0, pitch: 0, yaw: 0 },
  lastImuTime: null,
  attitudeFilter: null,
  attitudeThreshold: null,
  map: null,
  mapMarker: null,
  mapPathLine: null,
  mapPath: [],
  mapAutoFollow: true,
  mapHomeMarker: null,
  mapHomePending: false,
  mapHome: null,
  mapLastFix: null,
  sdLoggingAckTs: null,
  sdLoggingEnabled: null,
  sdCommandPending: false,
  telemetryTxAckTs: null,
  telemetryEnabled: null,
  telemetryCommandPending: false,
  buzzerCommandPending: false,
};

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;
const FILTER_LABELS = {
  madgwick: "Madgwick AHRS",
  mahony: "Mahony AHRS",
  complementary: "Complementary",
  "accel-threshold": "Accel Threshold",
};
const MAP_PATH_LIMIT = 200;

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

function formatTimestamp(timestampSec) {
  if (timestampSec === null || timestampSec === undefined || Number.isNaN(timestampSec)) {
    return "--";
  }
  const date = new Date(timestampSec * 1000);
  if (Number.isNaN(date.getTime())) {
    return "--";
  }
  return date.toLocaleString();
}

function formatInt(value) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "--";
  }
  return Math.round(value).toString();
}

function setDeployIndicator(element, stage) {
  if (!element) {
    return;
  }
  element.classList.remove("pending", "deployed");
  if (!stage || stage.deployed !== true) {
    element.textContent = "Pending";
    element.classList.add("pending");
    return;
  }

  const altitude = stage.deploy_alt_agl_m;
  const reason = stage.reason;
  const parts = ["DEPLOYED"];
  if (altitude !== null && altitude !== undefined) {
    parts.push(`${formatNumber(altitude, 1)} m AGL`);
  }
  if (reason) {
    parts.push(reason);
  }
  element.textContent = parts.join(" · ");
  element.classList.add("deployed");
}

function labelForFilter(value) {
  if (!value) {
    return "Unknown";
  }
  return FILTER_LABELS[value] || value;
}

function setMonitorTone(element, tone) {
  if (!element) {
    return;
  }
  element.classList.remove("tone-ok", "tone-warn", "tone-error");
  if (tone) {
    element.classList.add(tone);
  }
}

function setFilterOptions(filters, active) {
  if (!elements.attitudeFilter) {
    return;
  }
  elements.attitudeFilter.innerHTML = "";
  filters.forEach((filter) => {
    const option = document.createElement("option");
    option.value = filter;
    option.textContent = labelForFilter(filter);
    elements.attitudeFilter.appendChild(option);
  });
  if (active) {
    elements.attitudeFilter.value = active;
    state.attitudeFilter = active;
  }
}

function setThresholdValue(value) {
  if (!elements.attitudeThreshold) {
    return;
  }
  if (value === null || value === undefined || Number.isNaN(value)) {
    return;
  }
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) {
    return;
  }
  if (document.activeElement !== elements.attitudeThreshold) {
    elements.attitudeThreshold.value = numeric.toFixed(2);
  }
  state.attitudeThreshold = numeric;
}

function syncFilterSelection(attitude) {
  if (!elements.attitudeFilter || !attitude) {
    return;
  }
  const active = attitude.filter;
  if (!active) {
    return;
  }
  if (document.activeElement !== elements.attitudeFilter && elements.attitudeFilter.value !== active) {
    elements.attitudeFilter.value = active;
  }
  state.attitudeFilter = active;
}

function syncThreshold(attitude) {
  if (!elements.attitudeThreshold || !attitude) {
    return;
  }
  setThresholdValue(attitude.threshold_g);
}

function postFilterSelection(value) {
  const previous = state.attitudeFilter;
  fetch("/api/attitude/filter", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ filter: value }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Filter update failed");
      }
      state.attitudeFilter = payload.filter;
      if (elements.attitudeFilter) {
        elements.attitudeFilter.value = payload.filter;
      }
    })
    .catch(() => {
      if (elements.attitudeFilter && previous) {
        elements.attitudeFilter.value = previous;
      }
    });
}

if (elements.sdStart) {
  elements.sdStart.addEventListener("click", () => {
    postSdCommand("sd_start", "start logging");
  });
}

if (elements.sdStop) {
  elements.sdStop.addEventListener("click", () => {
    postSdCommand("sd_stop", "stop logging");
  });
}

if (elements.telemEnable) {
  elements.telemEnable.addEventListener("click", () => {
    postTelemetryCommand("telemetry_enable", "enable telemetry");
  });
}

if (elements.telemDisable) {
  elements.telemDisable.addEventListener("click", () => {
    postTelemetryCommand("telemetry_disable", "disable telemetry");
  });
}

if (elements.buzzerActivate) {
  elements.buzzerActivate.addEventListener("click", () => {
    const duration = elements.buzzerDuration ? Number(elements.buzzerDuration.value) : 0;
    postBuzzerCommand(duration);
  });
}

function setControlStatus(message, statusClass) {
  if (!elements.sdStatus) {
    return;
  }
  elements.sdStatus.textContent = message;
  elements.sdStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.sdStatus.classList.add(statusClass);
  }
}

function toggleControlButtons(disabled) {
  if (elements.sdStart) {
    elements.sdStart.disabled = disabled;
  }
  if (elements.sdStop) {
    elements.sdStop.disabled = disabled;
  }
}

function updateSdControlState() {
  if (state.sdCommandPending) {
    toggleControlButtons(true);
    return;
  }
  if (state.sdLoggingEnabled === true) {
    if (elements.sdStart) {
      elements.sdStart.disabled = true;
    }
    if (elements.sdStop) {
      elements.sdStop.disabled = false;
    }
  } else if (state.sdLoggingEnabled === false) {
    if (elements.sdStart) {
      elements.sdStart.disabled = false;
    }
    if (elements.sdStop) {
      elements.sdStop.disabled = true;
    }
  } else {
    toggleControlButtons(false);
  }
}

function postSdCommand(action, label) {
  setControlStatus(`Sending ${label}…`, "pending");
  state.sdCommandPending = true;
  toggleControlButtons(true);
  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setControlStatus(`Command sent: ${label}. Awaiting ack…`, "pending");
    })
    .catch((err) => {
      setControlStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.sdCommandPending = false;
      updateSdControlState();
    });
}

function setTelemetryStatus(message, statusClass) {
  if (!elements.telemStatus) {
    return;
  }
  elements.telemStatus.textContent = message;
  elements.telemStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.telemStatus.classList.add(statusClass);
  }
}

function toggleTelemetryButtons(disabled) {
  if (elements.telemEnable) {
    elements.telemEnable.disabled = disabled;
  }
  if (elements.telemDisable) {
    elements.telemDisable.disabled = disabled;
  }
}

function updateTelemetryControlState() {
  if (state.telemetryCommandPending) {
    toggleTelemetryButtons(true);
    return;
  }
  if (state.telemetryEnabled === true) {
    if (elements.telemEnable) {
      elements.telemEnable.disabled = true;
    }
    if (elements.telemDisable) {
      elements.telemDisable.disabled = false;
    }
  } else if (state.telemetryEnabled === false) {
    if (elements.telemEnable) {
      elements.telemEnable.disabled = false;
    }
    if (elements.telemDisable) {
      elements.telemDisable.disabled = true;
    }
  } else {
    toggleTelemetryButtons(false);
  }
}

function postTelemetryCommand(action, label) {
  setTelemetryStatus(`Sending ${label}…`, "pending");
  state.telemetryCommandPending = true;
  toggleTelemetryButtons(true);
  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setTelemetryStatus(`Command sent: ${label}. Awaiting ack…`, "pending");
    })
    .catch((err) => {
      setTelemetryStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.telemetryCommandPending = false;
      updateTelemetryControlState();
    });
}

function setBuzzerStatus(message, statusClass) {
  if (!elements.buzzerStatus) {
    return;
  }
  elements.buzzerStatus.textContent = message;
  elements.buzzerStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.buzzerStatus.classList.add(statusClass);
  }
}

function toggleBuzzerControls(disabled) {
  if (elements.buzzerActivate) {
    elements.buzzerActivate.disabled = disabled;
  }
  if (elements.buzzerDuration) {
    elements.buzzerDuration.disabled = disabled;
  }
}

function postBuzzerCommand(durationSec) {
  if (!Number.isFinite(durationSec) || durationSec <= 0) {
    setBuzzerStatus("Invalid duration", "error");
    return;
  }
  setBuzzerStatus(`Sending buzzer (${durationSec}s)…`, "pending");
  state.buzzerCommandPending = true;
  toggleBuzzerControls(true);
  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "buzzer", duration_s: durationSec }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setBuzzerStatus(`Buzzer command sent (${durationSec}s).`, "ok");
    })
    .catch((err) => {
      setBuzzerStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.buzzerCommandPending = false;
      toggleBuzzerControls(false);
    });
}

function postThresholdUpdate(value) {
  const previous = state.attitudeThreshold;
  fetch("/api/attitude/threshold", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ threshold_g: value }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Threshold update failed");
      }
      setThresholdValue(payload.threshold_g);
    })
    .catch(() => {
      if (elements.attitudeThreshold && previous !== null) {
        elements.attitudeThreshold.value = Number(previous).toFixed(2);
      }
    });
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

  const packetCount = snapshot.packet_count || 0;
  if (elements.telemetryWaiting) {
    const waiting = packetCount === 0;
    elements.telemetryWaiting.classList.toggle("visible", waiting);
  }

  const radio = snapshot.radio || {};
  elements.crcStatus.textContent = radio.crc_ok === null ? "--" : (radio.crc_ok ? "OK" : "BAD");
  elements.radioRssi.textContent = radio.rssi_dbm !== null ? `${radio.rssi_dbm} dBm` : "--";
  elements.radioSnr.textContent = radio.snr_db !== null ? `${formatNumber(radio.snr_db, 1)} dB` : "--";
  elements.radioPayload.textContent = radio.payload_len !== null ? `${radio.payload_len} bytes` : "--";
  elements.radioLastPacket.textContent = formatTimestamp(snapshot.timestamp);

  const gps = snapshot.gps || {};
  elements.gpsTime.textContent = gps.t_ms !== null && gps.t_ms !== undefined ? `${gps.t_ms} ms` : "--";
  elements.gpsLat.textContent = gps.lat_deg !== null && gps.lat_deg !== undefined ? formatNumber(gps.lat_deg, 6) : "--";
  elements.gpsLon.textContent = gps.lon_deg !== null && gps.lon_deg !== undefined ? formatNumber(gps.lon_deg, 6) : "--";
  elements.gpsAlt.textContent = gps.alt_m !== null && gps.alt_m !== undefined ? `${formatNumber(gps.alt_m, 1)} m` : "--";
  elements.gpsLatDeg.textContent = elements.gpsLat.textContent;
  elements.gpsLonDeg.textContent = elements.gpsLon.textContent;
  elements.gpsAltM.textContent = gps.alt_m !== null && gps.alt_m !== undefined ? formatNumber(gps.alt_m, 2) : "--";

  const navsat = snapshot.navsat || {};
  elements.navsatTime.textContent = navsat.t_ms !== null && navsat.t_ms !== undefined
    ? `${navsat.t_ms} ms`
    : "--";
  elements.navsatSvsUsed.textContent = navsat.svs_used !== null && navsat.svs_used !== undefined
    ? navsat.svs_used
    : "--";
  elements.navsatSvsTotal.textContent = navsat.svs_total !== null && navsat.svs_total !== undefined
    ? navsat.svs_total
    : "--";
  elements.navsatCnoMax.textContent = navsat.cno_max !== null && navsat.cno_max !== undefined
    ? `${navsat.cno_max} dB-Hz`
    : "--";
  elements.navsatCnoAvg.textContent = navsat.cno_avg !== null && navsat.cno_avg !== undefined
    ? `${navsat.cno_avg} dB-Hz`
    : "--";

  const alt = snapshot.alt || {};
  elements.altTime.textContent = alt.t_ms !== null && alt.t_ms !== undefined ? `${alt.t_ms} ms` : "--";
  elements.altPressPa.textContent = alt.press_pa !== null && alt.press_pa !== undefined ? formatNumber(alt.press_pa, 1) : "--";
  elements.altPressKpa.textContent = alt.press_kpa !== null && alt.press_kpa !== undefined ? formatNumber(alt.press_kpa, 3) : "--";
  elements.altTempC.textContent = alt.temp_c !== null && alt.temp_c !== undefined ? formatNumber(alt.temp_c, 2) : "--";

  const recovery = snapshot.recovery || {};
  elements.recoveryMode.textContent = recovery.mode || "--";
  elements.recoveryPhase.textContent = recovery.phase || "--";
  elements.recoveryAgl.textContent = recovery.altitude_agl_m !== null && recovery.altitude_agl_m !== undefined
    ? `${formatNumber(recovery.altitude_agl_m, 1)} m`
    : "--";
  elements.recoveryVspeed.textContent = recovery.vertical_speed_mps !== null && recovery.vertical_speed_mps !== undefined
    ? `${formatNumber(recovery.vertical_speed_mps, 1)} m/s`
    : "--";
  setDeployIndicator(elements.recoveryDrogue, recovery.drogue);
  setDeployIndicator(elements.recoveryMain, recovery.main);

  const imu = snapshot.imu || {};
  elements.imuTime.textContent = imu.t_ms !== null && imu.t_ms !== undefined ? `${imu.t_ms} ms` : "--";
  elements.imuTimeRaw.textContent = elements.imuTime.textContent;
  elements.imuGyro.textContent = imu.gx_dps !== null && imu.gx_dps !== undefined
    ? `${formatNumber(imu.gx_dps, 1)}, ${formatNumber(imu.gy_dps, 1)}, ${formatNumber(imu.gz_dps, 1)}`
    : "--";
  elements.imuAccel.textContent = imu.ax_g !== null && imu.ax_g !== undefined
    ? `${formatNumber(imu.ax_g, 2)}, ${formatNumber(imu.ay_g, 2)}, ${formatNumber(imu.az_g, 2)}`
    : "--";

  const battery = snapshot.battery || {};
  elements.batTime.textContent = battery.t_ms !== null && battery.t_ms !== undefined ? `${battery.t_ms} ms` : "--";
  elements.batVoltage.textContent = battery.vbat_v !== null && battery.vbat_v !== undefined
    ? formatNumber(battery.vbat_v, 2)
    : "--";
  elements.batState.textContent = battery.bat_state_label || "--";

  const monitor = snapshot.voltage_monitor || {};
  const monitorStatus = monitor.status || "--";
  const monitorDisabled = monitor.enabled === false && monitor.disabled_reason;
  let statusTone = null;
  elements.vmStatus.textContent = monitorDisabled
    ? `disabled (${monitor.disabled_reason})`
    : monitorStatus;
  if (monitor.shutdown_triggered || monitor.status === "error") {
    statusTone = "tone-error";
  } else if (monitor.warning) {
    statusTone = "tone-warn";
  } else if (monitor.status === "running") {
    statusTone = "tone-ok";
  } else if (monitorDisabled || monitor.status === "disabled") {
    statusTone = "tone-warn";
  }
  setMonitorTone(elements.vmStatus, statusTone);

  elements.vmUpdated.textContent = formatTimestamp(monitor.timestamp);
  elements.vmVin.textContent = monitor.vin_v !== null && monitor.vin_v !== undefined
    ? `${formatNumber(monitor.vin_v, 2)} V`
    : "--";
  elements.vmVout.textContent = monitor.vout_v !== null && monitor.vout_v !== undefined
    ? `${formatNumber(monitor.vout_v, 2)} V`
    : "--";
  elements.vmVbatt.textContent = monitor.vbatt_v !== null && monitor.vbatt_v !== undefined
    ? `${formatNumber(monitor.vbatt_v, 2)} V`
    : "--";
  elements.vmTempC.textContent = monitor.temp_c !== null && monitor.temp_c !== undefined
    ? `${formatNumber(monitor.temp_c, 2)} °C`
    : "--";
  elements.vmTempF.textContent = monitor.temp_f !== null && monitor.temp_f !== undefined
    ? `${formatNumber(monitor.temp_f, 2)} °F`
    : "--";

  if (monitor.shutdown_triggered) {
    elements.vmWarning.textContent = "Shutdown triggered: low input and low battery";
    setMonitorTone(elements.vmWarning, "tone-error");
  } else if (monitor.last_error) {
    elements.vmWarning.textContent = `Error: ${monitor.last_error}`;
    setMonitorTone(elements.vmWarning, "tone-error");
  } else {
    elements.vmWarning.textContent = monitor.warning || "--";
    setMonitorTone(elements.vmWarning, monitor.warning ? "tone-warn" : null);
  }

  const sdLogging = snapshot.sd_logging || {};
  if (sdLogging.ack_timestamp && sdLogging.ack_timestamp !== state.sdLoggingAckTs) {
    state.sdLoggingAckTs = sdLogging.ack_timestamp;
    state.sdLoggingEnabled = sdLogging.enabled;
    const enabled = sdLogging.enabled;
    const statusText = enabled === null || enabled === undefined
      ? "Logging ack received"
      : (enabled ? "Logging enabled" : "Logging stopped");
    const timeLabel = formatTimestamp(sdLogging.ack_timestamp);
    setControlStatus(`${statusText} (${timeLabel})`, "ok");
    updateSdControlState();
    if (elements.telemetryWaiting && enabled === true) {
      elements.telemetryWaiting.classList.remove("visible");
    }
  }

  const telemetryTx = snapshot.telemetry_tx || {};
  if (telemetryTx.ack_timestamp && telemetryTx.ack_timestamp !== state.telemetryTxAckTs) {
    state.telemetryTxAckTs = telemetryTx.ack_timestamp;
    state.telemetryEnabled = telemetryTx.enabled;
    const enabled = telemetryTx.enabled;
    const statusText = enabled === null || enabled === undefined
      ? "Telemetry TX ack received"
      : (enabled ? "Telemetry TX enabled" : "Telemetry TX disabled");
    const timeLabel = formatTimestamp(telemetryTx.ack_timestamp);
    setTelemetryStatus(`${statusText} (${timeLabel})`, "ok");
    updateTelemetryControlState();
  }

  const attitude = snapshot.attitude || {};
  updateOrientation(attitude, imu);
  syncFilterSelection(attitude);
  syncThreshold(attitude);
  updateMap(gps);
}

function updateOrientation(attitude, imu) {
  const hasAttitude = attitude
    && attitude.roll !== null
    && attitude.pitch !== null
    && attitude.yaw !== null
    && attitude.roll !== undefined
    && attitude.pitch !== undefined
    && attitude.yaw !== undefined;

  if (hasAttitude) {
    state.orientation.roll = attitude.roll;
    state.orientation.pitch = attitude.pitch;
    state.orientation.yaw = attitude.yaw;
    state.lastImuTime = null;
  } else {
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

function clearMapTrack() {
  state.mapPath = [];
  if (state.mapPathLine) {
    state.mapPathLine.setLatLngs([]);
  }
}

function setHomeMarker(lat, lon) {
  if (!state.map || !window.L) {
    return;
  }
  const homeLatLng = [lat, lon];
  state.mapHome = homeLatLng;
  if (state.mapHomeMarker) {
    state.mapHomeMarker.setLatLng(homeLatLng);
    return;
  }
  state.mapHomeMarker = L.circleMarker(homeLatLng, {
    radius: 7,
    color: "#b98118",
    fillColor: "#b98118",
    fillOpacity: 0.9,
    weight: 2,
  }).addTo(state.map);
}

function jumpToCurrentFix() {
  if (!state.map || !state.mapLastFix) {
    return;
  }
  state.mapAutoFollow = true;
  state.map.panTo(state.mapLastFix, { animate: true });
}

function updateMap(gps) {
  if (!gps || gps.lat_deg === null || gps.lon_deg === null) {
    elements.mapOverlay.textContent = "Awaiting GPS fix";
    return;
  }

  elements.mapOverlay.textContent = "";

  const lat = gps.lat_deg;
  const lon = gps.lon_deg;
  state.mapLastFix = [lat, lon];

  if (!state.map && mapContainer && window.L) {
    state.map = L.map(mapContainer, { zoomControl: true });
    L.tileLayer("/tiles/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: "&copy; OpenStreetMap contributors",
    }).addTo(state.map);
    state.map.setView([lat, lon], 13);
    state.mapMarker = L.circleMarker([lat, lon], {
      radius: 6,
      color: "#2ed2a5",
      fillColor: "#2ed2a5",
      fillOpacity: 0.9,
    }).addTo(state.map);
    state.mapPathLine = L.polyline([[lat, lon]], {
      color: "#2ed2a5",
      weight: 3,
      opacity: 0.85,
    }).addTo(state.map);
    state.mapPath = [[lat, lon]];
    state.mapAutoFollow = true;
    state.map.on("dragstart", () => {
      state.mapAutoFollow = false;
    });
    state.map.on("zoomstart", () => {
      state.mapAutoFollow = false;
    });
  }

  if (!state.map) {
    elements.mapOverlay.textContent = "Map unavailable";
    return;
  }

  if (state.mapHomePending) {
    setHomeMarker(lat, lon);
    state.mapHomePending = false;
  }

  state.mapPath.push([lat, lon]);
  if (state.mapPath.length > MAP_PATH_LIMIT) {
    state.mapPath.shift();
  }

  if (state.mapMarker) {
    state.mapMarker.setLatLng([lat, lon]);
  }
  if (state.mapPathLine) {
    state.mapPathLine.setLatLngs(state.mapPath);
  }
  if (state.mapAutoFollow) {
    state.map.panTo([lat, lon], { animate: false });
  }
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

  fetch("/api/attitude/filters")
    .then((res) => res.json())
    .then((payload) => {
      if (payload && Array.isArray(payload.filters)) {
        setFilterOptions(payload.filters, payload.active);
      }
      if (payload && payload.threshold_g !== undefined) {
        setThresholdValue(payload.threshold_g);
      }
    })
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
  if (state.map) {
    state.map.invalidateSize();
  }
});

if (elements.attitudeFilter) {
  elements.attitudeFilter.addEventListener("change", (event) => {
    const value = event.target.value;
    if (value) {
      postFilterSelection(value);
    }
  });
}

if (elements.attitudeThreshold) {
  elements.attitudeThreshold.addEventListener("change", (event) => {
    const value = Number(event.target.value);
    if (Number.isFinite(value) && value > 0) {
      postThresholdUpdate(value);
    }
  });
}

if (elements.mapJump) {
  elements.mapJump.addEventListener("click", () => {
    jumpToCurrentFix();
  });
}

if (elements.mapSetHome) {
  elements.mapSetHome.addEventListener("click", () => {
    const shouldReset = window.confirm(
      "Set a new home position? This will clear the current GPS track."
    );
    if (!shouldReset) {
      return;
    }
    clearMapTrack();
    state.mapHomePending = true;
    state.mapAutoFollow = true;
    state.mapHome = null;
    if (state.mapHomeMarker) {
      state.mapHomeMarker.remove();
      state.mapHomeMarker = null;
    }
  });
}

setInterval(updateClock, 500);

renderRocket();
initEventSource();
