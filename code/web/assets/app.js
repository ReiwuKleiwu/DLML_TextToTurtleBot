const chatLog = document.getElementById("chat-log");
const chatForm = document.getElementById("chat-form");
const chatInput = document.getElementById("chat-input");
const sendButton = document.getElementById("send-button");
const snapshotButton = document.getElementById("snapshot-button");
const snapshotOutput = document.getElementById("snapshot-output");
const statusText = document.getElementById("status-text");
const clearMapButton = document.getElementById("clear-map-button");
const resetViewButton = document.getElementById("reset-view-button");
const mapMeta = document.getElementById("map-meta");
const cameraFeed = document.getElementById("camera-feed");
const cameraStatus = document.getElementById("camera-status");
const stateStackList = document.getElementById("state-stack");

const mapCanvas = document.getElementById("map-canvas");
const ctx = mapCanvas.getContext("2d");
const dpr = window.devicePixelRatio || 1;

const theme = {
  background: "#181825",
  gridMinor: "rgba(108, 112, 134, 0.2)",
  gridMajor: "rgba(137, 140, 161, 0.35)",
  lidar: "rgba(148, 226, 213, 0.55)",
  obstacle: "rgba(243, 139, 168, 0.55)",
  robotFill: "rgba(137, 180, 250, 0.85)",
  robotStroke: "rgba(205, 214, 244, 0.9)",
  trajectory: "rgba(250, 179, 135, 0.85)",
  object: "rgba(166, 227, 161, 0.85)",
  target: "rgba(243, 139, 168, 0.9)",
  navLine: "rgba(249, 226, 175, 0.6)",
  navGoal: "rgba(249, 226, 175, 0.9)",
  label: "#cdd6f4",
};

const mapState = {
  data: null,
  initialized: false,
  displayPosition: null,
  lastUpdateTime: performance.now(),
};

const viewport = {
  centerX: 0,
  centerY: 0,
  scale: 55,
  minScale: 8,
  maxScale: 280,
};

let isPanning = false;
let panStart = { x: 0, y: 0 };
let panOrigin = { x: 0, y: 0 };

function resizeCanvas() {
  const bounds = mapCanvas.getBoundingClientRect();
  const width = Math.max(bounds.width, 200);
  const height = Math.max(bounds.height, 200);
  mapCanvas.width = width * dpr;
  mapCanvas.height = height * dpr;
  mapCanvas.style.width = `${width}px`;
  mapCanvas.style.height = `${height}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

resizeCanvas();

window.addEventListener("resize", resizeCanvas);

function worldToScreen(x, y) {
  const width = mapCanvas.clientWidth;
  const height = mapCanvas.clientHeight;
  const sx = width / 2 + (x - viewport.centerX) * viewport.scale;
  const sy = height / 2 - (y - viewport.centerY) * viewport.scale;
  return [sx, sy];
}

function screenToWorld(sx, sy) {
  const width = mapCanvas.clientWidth;
  const height = mapCanvas.clientHeight;
  const x = (sx - width / 2) / viewport.scale + viewport.centerX;
  const y = (height / 2 - sy) / viewport.scale + viewport.centerY;
  return [x, y];
}

function quaternionToYaw(orientation) {
  if (!Array.isArray(orientation) || orientation.length < 4) {
    return 0;
  }
  const [x, y, z, w] = orientation.map(Number);
  const siny = 2 * (w * z + x * y);
  const cosy = 1 - 2 * (y * y + z * z);
  return Math.atan2(siny, cosy);
}

function addMessage(role, text) {
  const entry = document.createElement("article");
  entry.classList.add("message", role);

  const heading = document.createElement("h3");
  heading.textContent = role === "agent" ? "LLM" : role === "user" ? "You" : "System";

  const body = document.createElement("p");
  body.textContent = text;

  entry.append(heading, body);
  chatLog.append(entry);
  chatLog.scrollTop = chatLog.scrollHeight;
}

function addSystemMessage(text) {
  addMessage("system", text);
}

function setStatus(message) {
  statusText.textContent = message;
}

function setBusy(isBusy) {
  sendButton.disabled = isBusy;
  chatInput.disabled = isBusy;
  snapshotButton.disabled = isBusy;
  if (!isBusy) {
    chatInput.focus();
  }
}

async function fetchJson(url, options) {
  const response = await fetch(url, options);
  if (!response.ok) {
    let detail = response.statusText;
    try {
      const payload = await response.json();
      if (payload.detail) {
        detail = payload.detail;
      }
    } catch (error) {
      // ignore parsing errors
    }
    throw new Error(detail || `Request failed with status ${response.status}`);
  }
  return response.json();
}

async function refreshSnapshot(updateStatus = true) {
  try {
    const snapshot = await fetchJson("/api/snapshot");
    snapshotOutput.textContent = JSON.stringify(snapshot, null, 2);
    if (updateStatus) {
      setStatus("Snapshot refreshed.");
    }
  } catch (error) {
    addSystemMessage(`Snapshot error: ${error.message}`);
    if (updateStatus) {
      setStatus("Snapshot failed.");
    }
  }
}

async function refreshMapData() {
  try {
    const data = await fetchJson("/api/map");
    mapState.data = data;
    mapState.lastUpdateTime = performance.now();

    const robotPos = data?.robot?.position;
    if (!mapState.initialized && Array.isArray(robotPos) && robotPos.length >= 2) {
      viewport.centerX = Number(robotPos[0]);
      viewport.centerY = Number(robotPos[1]);
      mapState.displayPosition = [Number(robotPos[0]), Number(robotPos[1])];
      mapState.initialized = true;
    }

    updateMapMeta(data);
    renderStateStack(data.state_stack);
  } catch (error) {
    addSystemMessage(`Map fetch error: ${error.message}`);
  }
}

function updateMapMeta(data) {
  if (!data || !data.additional_info) {
    mapMeta.textContent = "No telemetry available.";
    return;
  }
  const info = data.additional_info;
  const nav = data.navigation || {};
  const robot = data.robot || {};
  const lines = [
    `State: ${robot.state || "?"}`,
    `Target: ${robot.target_object || "—"}`,
    `Nav: ${nav.status || "IDLE"}`,
    `Objects tracked: ${info.total_objects ?? 0}`,
    `Lidar points: ${info.lidar_points ?? 0}`,
  ];
  mapMeta.textContent = lines.join("\n");
}

function renderStateStack(stack) {
  if (!stateStackList) {
    return;
  }
  stateStackList.innerHTML = "";
  if (!Array.isArray(stack) || !stack.length) {
    const item = document.createElement("li");
    item.textContent = "No state data.";
    stateStackList.append(item);
    return;
  }
  stack.forEach((entry, idx) => {
    const li = document.createElement("li");
    const state = entry?.state ?? "?";
    const source = entry?.source ?? "?";
    const data = entry?.data && Object.keys(entry.data).length ? JSON.stringify(entry.data) : "{}";
    li.textContent = `${state} (${source}) ${data}`;
    if (idx === stack.length - 1) {
      li.style.borderColor = "rgba(249, 226, 175, 0.65)";
      li.style.boxShadow = "0 0 12px rgba(249, 226, 175, 0.25)";
    }
    stateStackList.append(li);
  });
}

let cameraFetchInFlight = false;
async function refreshCameraFrame() {
  if (cameraFetchInFlight) {
    return;
  }
  cameraFetchInFlight = true;
  try {
    const response = await fetch(`/api/camera/latest?ts=${Date.now()}`);
    if (response.status === 204) {
      cameraStatus.textContent = "No frame";
      cameraFetchInFlight = false;
      return;
    }
    const blob = await response.blob();
    const url = URL.createObjectURL(blob);
    cameraFeed.src = url;
    cameraStatus.textContent = "Live";
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  } catch (error) {
    cameraStatus.textContent = "Error";
  } finally {
    cameraFetchInFlight = false;
  }
}

async function sendMessage(message) {
  addMessage("user", message);
  setBusy(true);
  setStatus("Waiting for LLM response...");

  try {
    const data = await fetchJson("/api/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ message }),
    });

    addMessage("agent", data.output ?? "(No response)");
    if (data.snapshot) {
      snapshotOutput.textContent = JSON.stringify(data.snapshot, null, 2);
    }
    const elapsed = typeof data.elapsed === "number" ? data.elapsed.toFixed(1) : "?";
    setStatus(`Response received in ${elapsed}s.`);
  } catch (error) {
    addSystemMessage(`Agent error: ${error.message}`);
    setStatus("Agent request failed.");
  } finally {
    setBusy(false);
  }
}

chatForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const message = chatInput.value.trim();
  if (!message) {
    return;
  }
  chatInput.value = "";
  await sendMessage(message);
});

snapshotButton.addEventListener("click", () => {
  setStatus("Fetching snapshot...");
  refreshSnapshot();
});

clearMapButton.addEventListener("click", async () => {
  clearMapButton.disabled = true;
  try {
    await fetchJson("/api/clear-map", { method: "POST" });
    addSystemMessage("Persistent map cleared.");
    await refreshMapData();
  } catch (error) {
    addSystemMessage(`Clear map failed: ${error.message}`);
  } finally {
    clearMapButton.disabled = false;
  }
});

resetViewButton.addEventListener("click", () => {
  const robotPos = mapState.data?.robot?.position;
  if (Array.isArray(robotPos) && robotPos.length >= 2) {
    viewport.centerX = Number(robotPos[0]);
    viewport.centerY = Number(robotPos[1]);
  } else {
    viewport.centerX = 0;
    viewport.centerY = 0;
  }
  viewport.scale = 55;
});

mapCanvas.addEventListener("pointerdown", (event) => {
  isPanning = true;
  panStart = { x: event.clientX, y: event.clientY };
  panOrigin = { x: viewport.centerX, y: viewport.centerY };
  mapCanvas.setPointerCapture(event.pointerId);
});

mapCanvas.addEventListener("pointermove", (event) => {
  if (!isPanning) {
    return;
  }
  const dx = (event.clientX - panStart.x) / viewport.scale;
  const dy = (event.clientY - panStart.y) / viewport.scale;
  viewport.centerX = panOrigin.x - dx;
  viewport.centerY = panOrigin.y + dy;
});

mapCanvas.addEventListener("pointerup", (event) => {
  isPanning = false;
  mapCanvas.releasePointerCapture(event.pointerId);
});

mapCanvas.addEventListener("pointerleave", () => {
  isPanning = false;
});

mapCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const zoomFactor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
  const [worldX, worldY] = screenToWorld(event.offsetX, event.offsetY);
  const newScale = Math.min(Math.max(viewport.scale * zoomFactor, viewport.minScale), viewport.maxScale);
  viewport.scale = newScale;
  const [afterX, afterY] = screenToWorld(event.offsetX, event.offsetY);
  viewport.centerX += worldX - afterX;
  viewport.centerY += worldY - afterY;
}, { passive: false });

function drawGrid(width, height) {
  ctx.fillStyle = theme.background;
  ctx.fillRect(0, 0, width, height);

  const stepMeters = chooseGridStep(viewport.scale);
  const step = stepMeters * viewport.scale;
  const [originX, originY] = worldToScreen(0, 0);

  ctx.save();
  ctx.strokeStyle = theme.gridMinor;
  ctx.lineWidth = 1;
  ctx.beginPath();

  const startX = originX % step;
  for (let x = startX; x < width; x += step) {
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
  }

  const startY = originY % step;
  for (let y = startY; y < height; y += step) {
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
  }

  ctx.stroke();

  ctx.strokeStyle = theme.gridMajor;
  ctx.beginPath();
  ctx.moveTo(originX, 0);
  ctx.lineTo(originX, height);
  ctx.moveTo(0, originY);
  ctx.lineTo(width, originY);
  ctx.stroke();
  ctx.restore();
}

function chooseGridStep(scale) {
  const steps = [0.5, 1, 2, 5, 10];
  for (const step of steps) {
    if (step * scale >= 60) {
      return step;
    }
  }
  return steps[steps.length - 1];
}

function drawLidar(lidar) {
  if (!lidar || !Array.isArray(lidar.scan_points)) {
    return;
  }
  ctx.save();
  ctx.fillStyle = theme.lidar;
  lidar.scan_points.forEach((pt) => {
    if (!Array.isArray(pt) || pt.length < 2) {
      return;
    }
    const [sx, sy] = worldToScreen(pt[0], pt[1]);
    ctx.beginPath();
    ctx.arc(sx, sy, 2, 0, Math.PI * 2);
    ctx.fill();
  });

  if (Array.isArray(lidar.obstacle_points)) {
    ctx.fillStyle = theme.obstacle;
    lidar.obstacle_points.forEach((pt) => {
      if (!Array.isArray(pt) || pt.length < 2) {
        return;
      }
      const [sx, sy] = worldToScreen(pt[0], pt[1]);
      ctx.beginPath();
      ctx.arc(sx, sy, 2.5, 0, Math.PI * 2);
      ctx.fill();
    });
  }
  ctx.restore();
}

function drawObjects(objects, targetClass, robotWorld) {
  if (!objects) {
    return;
  }
  Object.entries(objects).forEach(([cls, entries]) => {
    if (!Array.isArray(entries)) {
      return;
    }
    ctx.save();
    const highlightClass = cls === targetClass;
    entries.forEach((entry) => {
      const coords = entry.world_coords;
      if (!Array.isArray(coords) || coords.length < 2) {
        return;
      }
      const [sx, sy] = worldToScreen(coords[0], coords[1]);
      const isPrimaryTarget = Boolean(entry.is_selected_target);
      const isSelected = isPrimaryTarget || highlightClass;
      const radius = isPrimaryTarget ? 9 : 6;
      ctx.fillStyle = isPrimaryTarget ? theme.target : theme.object;
      ctx.beginPath();
      ctx.arc(sx, sy, radius, 0, Math.PI * 2);
      ctx.fill();

      if (isPrimaryTarget && Array.isArray(robotWorld) && robotWorld.length >= 2) {
        const [rx, ry] = worldToScreen(robotWorld[0], robotWorld[1]);
        ctx.strokeStyle = "rgba(243, 139, 168, 0.45)";
        ctx.lineWidth = 1.6;
        ctx.setLineDash([8, 6]);
        ctx.beginPath();
        ctx.moveTo(rx, ry);
        ctx.lineTo(sx, sy);
        ctx.stroke();
        ctx.setLineDash([]);

        const dx = coords[0] - robotWorld[0];
        const dy = coords[1] - robotWorld[1];
        const distance = Math.hypot(dx, dy);
        const midX = (rx + sx) / 2;
        const midY = (ry + sy) / 2;
        ctx.font = "12px Inter, sans-serif";
        ctx.fillStyle = theme.label;
        ctx.textBaseline = "bottom";
        ctx.textAlign = "center";
        ctx.fillText(`${distance.toFixed(1)} m`, midX, midY - 6);
      }

      ctx.font = "12px Inter, sans-serif";
      ctx.fillStyle = theme.label;
      ctx.textBaseline = "bottom";
      ctx.textAlign = "left";
      ctx.fillText(cls, sx + radius + 4, sy - 2);
    });
    ctx.restore();
  });
}

function drawTrajectory(points) {
  if (!Array.isArray(points) || points.length < 2) {
    return;
  }
  ctx.save();
  ctx.strokeStyle = theme.trajectory;
  ctx.lineWidth = 2;
  ctx.beginPath();
  points.forEach((pt, index) => {
    if (!Array.isArray(pt) || pt.length < 2) {
      return;
    }
    const [sx, sy] = worldToScreen(pt[0], pt[1]);
    if (index === 0) {
      ctx.moveTo(sx, sy);
    } else {
      ctx.lineTo(sx, sy);
    }
  });
  ctx.stroke();
  ctx.restore();
}

function drawNavGoal(nav, robotWorld) {
  const goal = nav?.goal;
  if (!Array.isArray(goal) || goal.length < 2) {
    return;
  }
  const [sx, sy] = worldToScreen(goal[0], goal[1]);
  ctx.save();
  if (Array.isArray(robotWorld) && robotWorld.length >= 2) {
    const [rx, ry] = worldToScreen(robotWorld[0], robotWorld[1]);
    ctx.strokeStyle = theme.navLine;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(sx, sy);
    ctx.stroke();
  }

  ctx.strokeStyle = theme.navGoal;
  ctx.lineWidth = 3;
  ctx.beginPath();
  const size = 10;
  ctx.moveTo(sx - size, sy - size);
  ctx.lineTo(sx + size, sy + size);
  ctx.moveTo(sx - size, sy + size);
  ctx.lineTo(sx + size, sy - size);
  ctx.stroke();
  ctx.restore();
}

function updateRobotPose(robot) {
  const pos = robot?.position;
  if (!Array.isArray(pos) || pos.length < 2) {
    return null;
  }

  const smoothing = 0.18;
  let display = mapState.displayPosition;
  if (!Array.isArray(display) || display.length < 2) {
    display = [Number(pos[0]), Number(pos[1])];
  } else {
    display = [
      display[0] + (pos[0] - display[0]) * smoothing,
      display[1] + (pos[1] - display[1]) * smoothing,
    ];
  }
  mapState.displayPosition = display;
  const yaw = quaternionToYaw(robot.orientation || []);
  return { position: display, yaw };
}

function renderRobot(pose) {
  if (!pose || !Array.isArray(pose.position) || pose.position.length < 2) {
    return;
  }
  const [sx, sy] = worldToScreen(pose.position[0], pose.position[1]);
  const size = 14;
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = theme.robotFill;
  ctx.strokeStyle = theme.robotStroke;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.6);
  ctx.lineTo(-size * 0.6, -size * 0.6);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function renderMap() {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const width = mapCanvas.clientWidth;
  const height = mapCanvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  drawGrid(width, height);

  const data = mapState.data;
  if (!data) {
    return;
  }

  const robotPose = updateRobotPose(data.robot);
  drawLidar(data.lidar);
  drawTrajectory(data.robot?.trajectory);
  const robotWorld = robotPose?.position || null;
  drawObjects(data.objects, data.robot?.target_object, robotWorld);
  drawNavGoal(data.navigation, robotWorld);
  renderRobot(robotPose);
}

function animationLoop() {
  requestAnimationFrame(animationLoop);
  renderMap();
}

animationLoop();

setInterval(refreshMapData, 1000 / 24);
setInterval(refreshCameraFrame, 1000 / 60);
refreshMapData();
refreshCameraFrame();
refreshSnapshot(false);
addSystemMessage("Connected. Type a command to begin.");
chatInput.focus();
