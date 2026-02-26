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
  recoveryArmed: document.getElementById("recovery-armed"),
  recoveryGpsFix: document.getElementById("recovery-gps-fix"),
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
  batGsVoltage: document.getElementById("bat-gs-voltage"),
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
  telemTxPower: document.getElementById("telem-tx-power"),
  telemTxPowerValue: document.getElementById("telem-tx-power-value"),
  telemTxPowerApply: document.getElementById("telem-tx-power-apply"),
  telemTxPowerStatus: document.getElementById("telem-tx-power-status"),
  telemTxPowerActive: document.getElementById("telem-tx-power-active"),
  buzzerDuration: document.getElementById("buzzer-duration"),
  buzzerActivate: document.getElementById("buzzer-activate"),
  buzzerStatus: document.getElementById("buzzer-status"),
  altCalibrate: document.getElementById("alt-calibrate"),
  altCalStatus: document.getElementById("alt-cal-status"),
  imuCalibrate: document.getElementById("imu-calibrate"),
  imuCalStatus: document.getElementById("imu-cal-status"),
  launchArm: document.getElementById("launch-arm"),
  launchArmStatus: document.getElementById("launch-arm-status"),
  soundEnabled: document.getElementById("sound-enabled"),
  soundVolume: document.getElementById("sound-volume"),
  soundVolumeValue: document.getElementById("sound-volume-value"),
  soundTest: document.getElementById("sound-test"),
  soundStatus: document.getElementById("sound-status"),
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
  telemetryTxPowerCommandPending: false,
  telemetryTxPowerDbm: 17,
  commandLockoutActive: false,
  buzzerCommandPending: false,
  altCalCommandPending: false,
  imuCalCommandPending: false,
  launchArmPending: false,
  launchArmAwaitingAck: false,
  launchArmAwaitingSinceMs: 0,
  launchArmAckTs: null,
  recoveryLaunchArmed: false,
  recoveryGpsFix3d: false,
  soundEnabled: true,
  soundVolumePct: 70,
  soundSettingsLoaded: false,
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
const SOUND_ENABLED_STORAGE_KEY = "gs_sound_enabled";
const SOUND_VOLUME_STORAGE_KEY = "gs_sound_volume";
const ROCKET_COLORS = {
  body: [198, 222, 235],
  nose: [244, 252, 255],
  fin: [58, 129, 121],
  nozzle: [46, 62, 78],
  wire: [120, 236, 204],
};

let soundAudioContext = null;

const rocketScene = {
  stars: [],
  width: 0,
  height: 0,
};

const rocketModel = (() => {
  const points = [];
  const faces = [];
  const wireEdges = [];

  const addPoint = (x, y, z) => {
    points.push([x, y, z]);
    return points.length - 1;
  };

  const addRing = (z, radius, count, twist = 0) => {
    const start = points.length;
    for (let i = 0; i < count; i += 1) {
      const angle = ((i / count) * Math.PI * 2) + twist;
      addPoint(Math.cos(angle) * radius, Math.sin(angle) * radius, z);
    }
    return start;
  };

  const connectRings = (ringA, ringB, count, color) => {
    for (let i = 0; i < count; i += 1) {
      const a = ringA + i;
      const b = ringA + ((i + 1) % count);
      const c = ringB + ((i + 1) % count);
      const d = ringB + i;
      faces.push({ indices: [a, b, c, d], color });
      wireEdges.push([a, b], [a, d]);
    }
  };

  const segments = 18;
  const baseRing = addRing(-1.0, 0.25, segments);
  const midRing = addRing(0.55, 0.22, segments, Math.PI / segments);
  const shoulderRing = addRing(0.82, 0.15, segments, Math.PI / segments);
  const noseTipIndex = addPoint(0, 0, 1.28);

  const nozzleSegments = 10;
  const nozzleRing = addRing(-1.16, 0.12, nozzleSegments);
  const tailIndex = addPoint(0, 0, -1.27);

  connectRings(baseRing, midRing, segments, ROCKET_COLORS.body);
  connectRings(midRing, shoulderRing, segments, [170, 198, 220]);

  for (let i = 0; i < segments; i += 1) {
    const a = shoulderRing + i;
    const b = shoulderRing + ((i + 1) % segments);
    faces.push({ indices: [a, b, noseTipIndex], color: ROCKET_COLORS.nose });
    wireEdges.push([a, b], [a, noseTipIndex]);
  }

  for (let i = 0; i < nozzleSegments; i += 1) {
    const a = nozzleRing + i;
    const b = nozzleRing + ((i + 1) % nozzleSegments);
    faces.push({ indices: [a, b, tailIndex], color: ROCKET_COLORS.nozzle });
    wireEdges.push([a, b], [a, tailIndex]);
  }

  for (let i = 0; i < segments; i += 3) {
    wireEdges.push([baseRing + i, shoulderRing + ((i + 9) % segments)]);
  }

  for (let i = 0; i < 4; i += 1) {
    const angle = i * (Math.PI / 2);
    const radialX = Math.cos(angle);
    const radialY = Math.sin(angle);
    const tangentX = -radialY;
    const tangentY = radialX;

    const rootTop = addPoint(
      (radialX * 0.24) + (tangentX * 0.035),
      (radialY * 0.24) + (tangentY * 0.035),
      -0.52
    );
    const rootBottom = addPoint(
      (radialX * 0.24) - (tangentX * 0.035),
      (radialY * 0.24) - (tangentY * 0.035),
      -0.84
    );
    const rootInset = addPoint(radialX * 0.29, radialY * 0.29, -0.64);
    const tip = addPoint(radialX * 0.52, radialY * 0.52, -1.04);

    faces.push({ indices: [rootTop, rootBottom, tip], color: ROCKET_COLORS.fin });
    faces.push({ indices: [rootTop, rootInset, rootBottom], color: [43, 100, 95] });
    wireEdges.push(
      [rootTop, rootBottom],
      [rootBottom, tip],
      [tip, rootTop],
      [rootTop, rootInset],
      [rootBottom, rootInset]
    );
  }

  return {
    points,
    faces,
    wireEdges,
    noseTipIndex,
    tailIndex,
  };
})();

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function subVec3(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function crossVec3(a, b) {
  return [
    (a[1] * b[2]) - (a[2] * b[1]),
    (a[2] * b[0]) - (a[0] * b[2]),
    (a[0] * b[1]) - (a[1] * b[0]),
  ];
}

function dotVec3(a, b) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

function normalizeVec3(v) {
  const mag = Math.hypot(v[0], v[1], v[2]);
  if (mag === 0) {
    return [0, 0, 0];
  }
  return [v[0] / mag, v[1] / mag, v[2] / mag];
}

function shadeColor(baseRgb, light, alpha = 1) {
  const tone = clamp(0.28 + (light * 0.82), 0, 1.25);
  const r = Math.round(clamp(baseRgb[0] * tone, 0, 255));
  const g = Math.round(clamp(baseRgb[1] * tone, 0, 255));
  const b = Math.round(clamp(baseRgb[2] * tone, 0, 255));
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function hashFract(seed) {
  const value = Math.sin(seed) * 43758.5453123;
  return value - Math.floor(value);
}

function ensureRocketScene(width, height) {
  if (rocketScene.width === width && rocketScene.height === height) {
    return;
  }

  rocketScene.width = width;
  rocketScene.height = height;
  rocketScene.stars = [];

  const starCount = Math.max(28, Math.floor((width * height) / 5200));
  for (let i = 0; i < starCount; i += 1) {
    const index = i + 1;
    rocketScene.stars.push({
      x: hashFract(index * 12.9898) * width,
      y: hashFract(index * 78.233) * (height * 0.74),
      radius: 0.4 + (hashFract(index * 35.425) * 1.6),
      alpha: 0.16 + (hashFract(index * 91.719) * 0.54),
    });
  }
}

function setTelemetryTxPowerInlineValue(powerDbm) {
  if (!elements.telemTxPowerValue) {
    return;
  }
  if (!Number.isFinite(powerDbm)) {
    elements.telemTxPowerValue.textContent = "-- dBm";
    return;
  }
  elements.telemTxPowerValue.textContent = `${Math.round(powerDbm)} dBm`;
}

function setTelemetryTxPowerStatus(message, statusClass) {
  if (!elements.telemTxPowerStatus) {
    return;
  }
  elements.telemTxPowerStatus.textContent = message;
  elements.telemTxPowerStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.telemTxPowerStatus.classList.add(statusClass);
  }
}

function setSoundVolumeInlineValue(volumePct) {
  if (!elements.soundVolumeValue) {
    return;
  }
  if (!Number.isFinite(volumePct)) {
    elements.soundVolumeValue.textContent = "--%";
    return;
  }
  elements.soundVolumeValue.textContent = `${Math.round(volumePct)}%`;
}

function setSoundStatus(message, statusClass) {
  if (!elements.soundStatus) {
    return;
  }
  elements.soundStatus.textContent = message;
  elements.soundStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.soundStatus.classList.add(statusClass);
  }
}

function updateSoundControlState() {
  const enabled = state.soundEnabled;
  if (elements.soundVolume) {
    elements.soundVolume.disabled = !enabled;
  }
  if (elements.soundTest) {
    elements.soundTest.disabled = !enabled;
  }
}

function persistSoundSettings() {
  if (!state.soundSettingsLoaded) {
    return;
  }
  try {
    localStorage.setItem(SOUND_ENABLED_STORAGE_KEY, state.soundEnabled ? "1" : "0");
    localStorage.setItem(SOUND_VOLUME_STORAGE_KEY, String(Math.round(state.soundVolumePct)));
  } catch (_err) {
    // Ignore storage failures (private mode / restricted browser policies).
  }
}

function loadSoundSettings() {
  let enabled = true;
  let volumePct = 70;

  try {
    const enabledRaw = localStorage.getItem(SOUND_ENABLED_STORAGE_KEY);
    if (enabledRaw === "0" || enabledRaw === "false") {
      enabled = false;
    }

    const volumeRaw = localStorage.getItem(SOUND_VOLUME_STORAGE_KEY);
    const parsedVolume = Number(volumeRaw);
    if (Number.isFinite(parsedVolume)) {
      volumePct = clamp(Math.round(parsedVolume), 0, 100);
    }
  } catch (_err) {
    // Keep defaults if local storage is unavailable.
  }

  state.soundEnabled = enabled;
  state.soundVolumePct = volumePct;
  state.soundSettingsLoaded = true;

  if (elements.soundEnabled) {
    elements.soundEnabled.checked = enabled;
  }
  if (elements.soundVolume) {
    elements.soundVolume.value = String(volumePct);
  }
  setSoundVolumeInlineValue(volumePct);
  setSoundStatus(enabled ? "Sound cues enabled" : "Sound cues muted", enabled ? "ok" : "pending");
  updateSoundControlState();
}

function ensureSoundAudioContext() {
  const AudioCtor = window.AudioContext || window.webkitAudioContext;
  if (!AudioCtor) {
    return null;
  }
  if (!soundAudioContext) {
    soundAudioContext = new AudioCtor();
  }
  if (soundAudioContext.state === "suspended") {
    soundAudioContext.resume().catch(() => {});
  }
  return soundAudioContext;
}

function playSoundPattern(pattern) {
  if (!state.soundEnabled) {
    return false;
  }
  const ctx = ensureSoundAudioContext();
  if (!ctx) {
    setSoundStatus("Browser audio unavailable", "error");
    return false;
  }

  const gain = ctx.createGain();
  gain.gain.value = clamp((state.soundVolumePct / 100) * 0.16, 0, 0.25);
  gain.connect(ctx.destination);

  let t = ctx.currentTime + 0.01;
  pattern.forEach((step) => {
    const freqHz = step[0];
    const durationS = step[1] / 1000;
    if (freqHz > 0) {
      const osc = ctx.createOscillator();
      osc.type = "sine";
      osc.frequency.setValueAtTime(freqHz, t);
      osc.connect(gain);
      osc.start(t);
      osc.stop(t + durationS);
    }
    t += durationS;
  });

  return true;
}

function playSoundTestTone() {
  const played = playSoundPattern([
    [784, 110],
    [0, 40],
    [1047, 130],
  ]);
  if (played) {
    setSoundStatus(`Test tone played (${Math.round(state.soundVolumePct)}%)`, "ok");
  }
}

function playLaunchArmCue(accepted) {
  if (accepted) {
    playSoundPattern([
      [740, 100],
      [988, 120],
      [1319, 140],
    ]);
  } else {
    playSoundPattern([
      [392, 150],
      [262, 190],
    ]);
  }
}

function playHomePointSetCue() {
  const played = playSoundPattern([
    [523, 90],
    [659, 100],
    [784, 120],
  ]);
  if (played) {
    setSoundStatus("Home point set cue played", "ok");
  }
}

function toggleTelemetryTxPowerControls(disabled) {
  if (elements.telemTxPowerApply) {
    elements.telemTxPowerApply.disabled = disabled;
  }
  if (elements.telemTxPower) {
    elements.telemTxPower.disabled = disabled;
  }
}

function drawRocketBackdrop(width, height, centerX, centerY, scale) {
  ensureRocketScene(width, height);
  const now = performance.now() * 0.001;

  const backdrop = rocketCtx.createLinearGradient(0, 0, 0, height);
  backdrop.addColorStop(0, "#091214");
  backdrop.addColorStop(0.45, "#102225");
  backdrop.addColorStop(1, "#070e12");
  rocketCtx.fillStyle = backdrop;
  rocketCtx.fillRect(0, 0, width, height);

  const halo = rocketCtx.createRadialGradient(
    centerX,
    centerY - (scale * 0.68),
    scale * 0.12,
    centerX,
    centerY,
    scale * 1.8
  );
  halo.addColorStop(0, "rgba(97, 242, 212, 0.22)");
  halo.addColorStop(0.5, "rgba(36, 102, 111, 0.13)");
  halo.addColorStop(1, "rgba(6, 14, 18, 0)");
  rocketCtx.fillStyle = halo;
  rocketCtx.fillRect(0, 0, width, height);

  for (const star of rocketScene.stars) {
    const shimmer = 0.72 + (Math.sin(now + (star.x * 0.047) + (star.y * 0.021)) * 0.28);
    rocketCtx.fillStyle = `rgba(220, 249, 255, ${star.alpha * shimmer})`;
    rocketCtx.beginPath();
    rocketCtx.arc(star.x, star.y, star.radius, 0, Math.PI * 2);
    rocketCtx.fill();
  }

  rocketCtx.strokeStyle = "rgba(112, 187, 175, 0.16)";
  rocketCtx.lineWidth = 1;
  rocketCtx.setLineDash([5, 8]);
  rocketCtx.beginPath();
  rocketCtx.moveTo(centerX - (scale * 1.18), centerY + (scale * 0.62));
  rocketCtx.lineTo(centerX + (scale * 1.18), centerY + (scale * 0.62));
  rocketCtx.stroke();
  rocketCtx.setLineDash([]);

  rocketCtx.save();
  rocketCtx.translate(centerX, centerY + (scale * 0.93));
  rocketCtx.scale(1.22, 0.34);
  rocketCtx.strokeStyle = "rgba(82, 172, 160, 0.2)";
  rocketCtx.lineWidth = 2;
  rocketCtx.beginPath();
  rocketCtx.arc(0, 0, scale * 1.03, 0, Math.PI * 2);
  rocketCtx.stroke();
  rocketCtx.restore();
}

function projectPoint(point, centerX, centerY, scale, camera) {
  const perspective = camera / (camera - point[2]);
  return {
    x: centerX + (point[0] * scale * perspective),
    y: centerY - (point[1] * scale * perspective),
    z: point[2],
  };
}

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

if (elements.telemTxPower) {
  elements.telemTxPower.addEventListener("input", () => {
    const value = Number(elements.telemTxPower.value);
    if (Number.isFinite(value)) {
      state.telemetryTxPowerDbm = value;
      setTelemetryTxPowerInlineValue(value);
    }
  });
}

if (elements.telemTxPowerApply) {
  elements.telemTxPowerApply.addEventListener("click", () => {
    const value = elements.telemTxPower ? Number(elements.telemTxPower.value) : NaN;
    postTelemetryTxPowerCommand(value);
  });
}

if (elements.buzzerActivate) {
  elements.buzzerActivate.addEventListener("click", () => {
    const duration = elements.buzzerDuration ? Number(elements.buzzerDuration.value) : 0;
    postBuzzerCommand(duration);
  });
}

if (elements.altCalibrate) {
  elements.altCalibrate.addEventListener("click", () => {
    postAltCalibrateCommand();
  });
}

if (elements.imuCalibrate) {
  elements.imuCalibrate.addEventListener("click", () => {
    postImuCalibrateCommand();
  });
}

if (elements.launchArm) {
  elements.launchArm.addEventListener("click", () => {
    postLaunchArmCommand();
  });
}

if (elements.soundEnabled) {
  elements.soundEnabled.addEventListener("change", () => {
    state.soundEnabled = Boolean(elements.soundEnabled.checked);
    persistSoundSettings();
    updateSoundControlState();
    if (state.soundEnabled) {
      setSoundStatus("Sound cues enabled", "ok");
      playSoundPattern([[988, 90]]);
    } else {
      setSoundStatus("Sound cues muted", "pending");
    }
  });
}

if (elements.soundVolume) {
  elements.soundVolume.addEventListener("input", () => {
    const value = Number(elements.soundVolume.value);
    if (!Number.isFinite(value)) {
      return;
    }
    state.soundVolumePct = clamp(Math.round(value), 0, 100);
    setSoundVolumeInlineValue(state.soundVolumePct);
    persistSoundSettings();
    if (state.soundEnabled) {
      setSoundStatus(`Volume ${state.soundVolumePct}%`, "ok");
    }
  });
}

if (elements.soundTest) {
  elements.soundTest.addEventListener("click", () => {
    playSoundTestTone();
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

function phaseIndicatesInFlight(phase) {
  const normalized = typeof phase === "string" ? phase.toLowerCase() : "";
  return normalized === "ascent" || normalized === "descent" || normalized === "boost" || normalized === "coast";
}

function updateSdControlState() {
  if (state.sdCommandPending) {
    toggleControlButtons(true);
    return;
  }

  if (state.commandLockoutActive) {
    if (elements.sdStart) {
      elements.sdStart.disabled = state.sdLoggingEnabled === true;
    }
    if (elements.sdStop) {
      elements.sdStop.disabled = true;
    }
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

  if (state.commandLockoutActive) {
    if (elements.telemEnable) {
      elements.telemEnable.disabled = state.telemetryEnabled === true;
    }
    if (elements.telemDisable) {
      elements.telemDisable.disabled = true;
    }
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

function postTelemetryTxPowerCommand(powerDbm) {
  const value = Math.round(Number(powerDbm));
  if (!Number.isFinite(value) || value < 2 || value > 17) {
    setTelemetryTxPowerStatus("Power must be between 2 and 17 dBm", "error");
    return;
  }

  setTelemetryTxPowerInlineValue(value);
  state.telemetryTxPowerDbm = value;
  state.telemetryTxPowerCommandPending = true;
  setTelemetryTxPowerStatus(`Sending TX power ${value} dBm…`, "pending");
  toggleTelemetryTxPowerControls(true);

  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "telemetry_tx_power", tx_power_dbm: value }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setTelemetryTxPowerStatus(`Command sent: ${value} dBm. Awaiting ack…`, "pending");
    })
    .catch((err) => {
      setTelemetryTxPowerStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.telemetryTxPowerCommandPending = false;
      toggleTelemetryTxPowerControls(false);
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

function setAltCalStatus(message, statusClass) {
  if (!elements.altCalStatus) {
    return;
  }
  elements.altCalStatus.textContent = message;
  elements.altCalStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.altCalStatus.classList.add(statusClass);
  }
}

function toggleAltCalControls(disabled) {
  if (elements.altCalibrate) {
    elements.altCalibrate.disabled = disabled;
  }
}

function postAltCalibrateCommand() {
  setAltCalStatus("Sending altitude zero + launch reset...", "pending");
  state.altCalCommandPending = true;
  toggleAltCalControls(true);
  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "alt_calibrate" }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setAltCalStatus("Altitude zero + launch reset command sent.", "ok");
    })
    .catch((err) => {
      setAltCalStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.altCalCommandPending = false;
      toggleAltCalControls(false);
    });
}

function setImuCalStatus(message, statusClass) {
  if (!elements.imuCalStatus) {
    return;
  }
  elements.imuCalStatus.textContent = message;
  elements.imuCalStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.imuCalStatus.classList.add(statusClass);
  }
}

function toggleImuCalControls(disabled) {
  if (elements.imuCalibrate) {
    elements.imuCalibrate.disabled = disabled;
  }
}

function postImuCalibrateCommand() {
  setImuCalStatus("Sending IMU calibration... keep rocket still.", "pending");
  state.imuCalCommandPending = true;
  toggleImuCalControls(true);
  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "imu_calibrate" }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      setImuCalStatus("IMU calibration command sent.", "ok");
    })
    .catch((err) => {
      setImuCalStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.imuCalCommandPending = false;
      toggleImuCalControls(false);
    });
}

function setLaunchArmStatus(message, statusClass) {
  if (!elements.launchArmStatus) {
    return;
  }
  elements.launchArmStatus.textContent = message;
  elements.launchArmStatus.classList.remove("ok", "pending", "error");
  if (statusClass) {
    elements.launchArmStatus.classList.add(statusClass);
  }
}

function updateLaunchArmControlState() {
  if (!elements.launchArm) {
    return;
  }

  let preserveStatus = false;

  if (state.launchArmPending) {
    elements.launchArm.disabled = true;
    return;
  }

  if (state.launchArmAwaitingAck) {
    const waitingAgeMs = Date.now() - state.launchArmAwaitingSinceMs;
    if (waitingAgeMs <= 10000) {
      elements.launchArm.disabled = true;
      setLaunchArmStatus("Launch-arm command sent. Awaiting ack...", "pending");
      return;
    }
    state.launchArmAwaitingAck = false;
    state.launchArmAwaitingSinceMs = 0;
    setLaunchArmStatus("Launch-arm ack timeout; you can retry", "error");
    preserveStatus = true;
  }

  if (state.commandLockoutActive) {
    elements.launchArm.disabled = true;
    if (!preserveStatus) {
      setLaunchArmStatus("Arm locked: flight in progress", "error");
    }
    return;
  }

  if (state.recoveryLaunchArmed) {
    elements.launchArm.disabled = true;
    if (!preserveStatus) {
      setLaunchArmStatus("Launch detect armed", "ok");
    }
    return;
  }

  if (!state.recoveryGpsFix3d) {
    elements.launchArm.disabled = true;
    if (!preserveStatus) {
      setLaunchArmStatus("Waiting for GPS 3D fix", "pending");
    }
    return;
  }

  elements.launchArm.disabled = false;
  if (!preserveStatus) {
    setLaunchArmStatus("Ready to arm launch detect", "ok");
  }
}

function postLaunchArmCommand() {
  if (state.commandLockoutActive) {
    setLaunchArmStatus("Arm locked: flight in progress", "error");
    return;
  }
  if (state.recoveryLaunchArmed) {
    setLaunchArmStatus("Launch detect already armed", "ok");
    return;
  }
  if (!state.recoveryGpsFix3d) {
    setLaunchArmStatus("Arm blocked: waiting for GPS 3D fix", "error");
    return;
  }

  state.launchArmPending = true;
  updateLaunchArmControlState();
  setLaunchArmStatus("Sending launch-arm command...", "pending");

  fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ action: "launch_arm" }),
  })
    .then((res) => res.json())
    .then((payload) => {
      if (!payload.ok) {
        throw new Error(payload.error || "Command failed");
      }
      state.launchArmAwaitingAck = true;
      state.launchArmAwaitingSinceMs = Date.now();
      setLaunchArmStatus("Launch-arm command sent. Awaiting ack...", "pending");
    })
    .catch((err) => {
      state.launchArmAwaitingAck = false;
      state.launchArmAwaitingSinceMs = 0;
      setLaunchArmStatus(err.message || "Command failed", "error");
    })
    .finally(() => {
      state.launchArmPending = false;
      updateLaunchArmControlState();
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
  const commandLockout = snapshot.command_lockout || {};
  if (commandLockout.active === null || commandLockout.active === undefined) {
    state.commandLockoutActive = phaseIndicatesInFlight(recovery.phase);
  } else {
    state.commandLockoutActive = Boolean(commandLockout.active);
  }
  state.recoveryLaunchArmed = recovery.launch_armed === true;
  state.recoveryGpsFix3d = recovery.gps_fix_3d === true;
  if (state.recoveryLaunchArmed) {
    state.launchArmAwaitingAck = false;
    state.launchArmAwaitingSinceMs = 0;
  }
  const launchArmAckTs = recovery.launch_arm_ack_timestamp;
  const launchArmAckEnabled = recovery.launch_arm_ack_enabled === true;
  elements.recoveryMode.textContent = recovery.mode || "--";
  elements.recoveryPhase.textContent = recovery.phase || "--";
  if (elements.recoveryArmed) {
    elements.recoveryArmed.textContent = state.recoveryLaunchArmed ? "ARMED" : "NO";
  }
  if (elements.recoveryGpsFix) {
    elements.recoveryGpsFix.textContent = state.recoveryGpsFix3d ? "3D FIX" : "NO FIX";
  }
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
  elements.batGsVoltage.textContent = battery.ground_vbat_v !== null && battery.ground_vbat_v !== undefined
    ? formatNumber(battery.ground_vbat_v, 2)
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
  const activePowerDbm = telemetryTx.active_power_dbm;
  if (elements.telemTxPowerActive) {
    elements.telemTxPowerActive.textContent = activePowerDbm === null || activePowerDbm === undefined
      ? "Active: -- dBm"
      : `Active: ${formatInt(activePowerDbm)} dBm`;
  }

  if (elements.telemTxPower && activePowerDbm !== null && activePowerDbm !== undefined) {
    const clampedPower = Math.max(2, Math.min(17, Number(activePowerDbm)));
    if (Number.isFinite(clampedPower) && !state.telemetryTxPowerCommandPending) {
      if (document.activeElement !== elements.telemTxPower) {
        elements.telemTxPower.value = String(clampedPower);
      }
      setTelemetryTxPowerInlineValue(clampedPower);
      state.telemetryTxPowerDbm = clampedPower;
    }
  }

  if (telemetryTx.ack_timestamp && telemetryTx.ack_timestamp !== state.telemetryTxAckTs) {
    state.telemetryTxAckTs = telemetryTx.ack_timestamp;
    state.telemetryEnabled = telemetryTx.enabled;
    const lastCommand = telemetryTx.last_command;
    const timeLabel = formatTimestamp(telemetryTx.ack_timestamp);
    if (lastCommand === "telemetry_tx_power") {
      const txPowerText = activePowerDbm === null || activePowerDbm === undefined
        ? "Telemetry TX power ack received"
        : `Telemetry TX power set: ${formatInt(activePowerDbm)} dBm`;
      setTelemetryTxPowerStatus(`${txPowerText} (${timeLabel})`, "ok");
    } else {
      const enabled = telemetryTx.enabled;
      const statusText = enabled === null || enabled === undefined
        ? "Telemetry TX ack received"
        : (enabled ? "Telemetry TX enabled" : "Telemetry TX disabled");
      setTelemetryStatus(`${statusText} (${timeLabel})`, "ok");
    }
    updateTelemetryControlState();
  }

  const attitude = snapshot.attitude || {};
  updateOrientation(attitude, imu);
  syncFilterSelection(attitude);
  syncThreshold(attitude);
  updateSdControlState();
  updateTelemetryControlState();
  updateLaunchArmControlState();
  if (launchArmAckTs && launchArmAckTs !== state.launchArmAckTs) {
    state.launchArmAckTs = launchArmAckTs;
    state.launchArmAwaitingAck = false;
    state.launchArmAwaitingSinceMs = 0;
    const timeLabel = formatTimestamp(launchArmAckTs);
    const ackAgeSec = (Date.now() / 1000) - Number(launchArmAckTs);
    const ackRecent = Number.isFinite(ackAgeSec) && ackAgeSec >= 0 && ackAgeSec <= 8;
    if (launchArmAckEnabled) {
      setLaunchArmStatus(`Launch detect armed (${timeLabel})`, "ok");
      if (ackRecent) {
        playLaunchArmCue(true);
      }
    } else if (state.commandLockoutActive) {
      setLaunchArmStatus(`Launch-arm rejected: flight in progress (${timeLabel})`, "error");
      if (ackRecent) {
        playLaunchArmCue(false);
      }
    } else if (!state.recoveryGpsFix3d) {
      setLaunchArmStatus(`Launch-arm rejected: GPS 3D fix required (${timeLabel})`, "error");
      if (ackRecent) {
        playLaunchArmCue(false);
      }
    } else {
      setLaunchArmStatus(`Launch-arm rejected (${timeLabel})`, "error");
      if (ackRecent) {
        playLaunchArmCue(false);
      }
    }
    if (elements.launchArm) {
      elements.launchArm.disabled = state.launchArmPending
        || state.launchArmAwaitingAck
        || state.commandLockoutActive
        || state.recoveryLaunchArmed
        || !state.recoveryGpsFix3d;
    }
  }
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
  if (width <= 0 || height <= 0) {
    requestAnimationFrame(renderRocket);
    return;
  }

  const centerX = width / 2;
  const centerY = (height * 0.5) + 20;
  const scale = Math.min(width, height) * 0.3;
  const camera = 4.2;

  drawRocketBackdrop(width, height, centerX, centerY, scale);

  const { roll, pitch, yaw } = state.orientation;
  const transformed = rocketModel.points.map((point) => rotatePoint(point, roll, pitch, yaw));
  const projected = transformed.map((point) => projectPoint(point, centerX, centerY, scale, camera));

  const lightDir = normalizeVec3([0.42, -0.36, 1]);
  const viewDir = [0, 0, 1];
  const drawableFaces = [];

  for (const face of rocketModel.faces) {
    if (!face || !face.indices || face.indices.length < 3) {
      continue;
    }

    const worldA = transformed[face.indices[0]];
    const worldB = transformed[face.indices[1]];
    const worldC = transformed[face.indices[2]];
    const edgeAB = subVec3(worldB, worldA);
    const edgeAC = subVec3(worldC, worldA);
    const normal = normalizeVec3(crossVec3(edgeAB, edgeAC));
    const facing = dotVec3(normal, viewDir);
    if (facing <= -0.06) {
      continue;
    }

    const light = clamp((dotVec3(normal, lightDir) + 1) * 0.5, 0, 1);
    let depth = 0;
    const screenPoints = [];
    for (const idx of face.indices) {
      depth += transformed[idx][2];
      screenPoints.push(projected[idx]);
    }
    depth /= face.indices.length;

    drawableFaces.push({
      screenPoints,
      depth,
      fill: shadeColor(face.color, light, clamp(0.7 + (facing * 0.26), 0.4, 0.96)),
      stroke: shadeColor(face.color, light * 0.65, 0.42),
    });
  }

  drawableFaces.sort((a, b) => a.depth - b.depth);
  for (const face of drawableFaces) {
    rocketCtx.beginPath();
    rocketCtx.moveTo(face.screenPoints[0].x, face.screenPoints[0].y);
    for (let i = 1; i < face.screenPoints.length; i += 1) {
      rocketCtx.lineTo(face.screenPoints[i].x, face.screenPoints[i].y);
    }
    rocketCtx.closePath();
    rocketCtx.fillStyle = face.fill;
    rocketCtx.fill();
    rocketCtx.strokeStyle = face.stroke;
    rocketCtx.lineWidth = 0.9;
    rocketCtx.stroke();
  }

  rocketCtx.lineWidth = 1.25;
  rocketCtx.strokeStyle = shadeColor(ROCKET_COLORS.wire, 1.0, 0.68);
  rocketCtx.beginPath();
  rocketModel.wireEdges.forEach(([a, b]) => {
    const pa = projected[a];
    const pb = projected[b];
    rocketCtx.moveTo(pa.x, pa.y);
    rocketCtx.lineTo(pb.x, pb.y);
  });
  rocketCtx.stroke();

  const nose = projected[rocketModel.noseTipIndex];
  const tail = projected[rocketModel.tailIndex];

  const noseGlow = rocketCtx.createRadialGradient(nose.x, nose.y, 1, nose.x, nose.y, scale * 0.26);
  noseGlow.addColorStop(0, "rgba(191, 255, 244, 0.48)");
  noseGlow.addColorStop(1, "rgba(191, 255, 244, 0)");
  rocketCtx.fillStyle = noseGlow;
  rocketCtx.beginPath();
  rocketCtx.arc(nose.x, nose.y, scale * 0.26, 0, Math.PI * 2);
  rocketCtx.fill();

  rocketCtx.strokeStyle = "rgba(124, 248, 219, 0.86)";
  rocketCtx.lineWidth = 2.6;
  rocketCtx.beginPath();
  rocketCtx.moveTo(tail.x, tail.y);
  rocketCtx.lineTo(nose.x, nose.y);
  rocketCtx.stroke();

  rocketCtx.fillStyle = "rgba(124, 248, 219, 0.94)";
  rocketCtx.beginPath();
  rocketCtx.arc(nose.x, nose.y, 3.4, 0, Math.PI * 2);
  rocketCtx.fill();

  rocketCtx.strokeStyle = "rgba(104, 194, 177, 0.32)";
  rocketCtx.lineWidth = 1;
  rocketCtx.setLineDash([3, 6]);
  rocketCtx.beginPath();
  rocketCtx.moveTo(nose.x, nose.y);
  rocketCtx.lineTo(nose.x, 18);
  rocketCtx.stroke();
  rocketCtx.setLineDash([]);

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
      maxZoom: 18,
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
    playHomePointSetCue();
  });
}

if (elements.telemTxPower) {
  const initial = Number(elements.telemTxPower.value);
  if (Number.isFinite(initial)) {
    state.telemetryTxPowerDbm = initial;
    setTelemetryTxPowerInlineValue(initial);
  }
}

loadSoundSettings();

setInterval(updateClock, 500);

renderRocket();
initEventSource();
