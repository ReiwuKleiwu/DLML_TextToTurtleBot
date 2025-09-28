import { formatNumber } from './formatters.js';

export function createMapController({ canvas, statusCursorEl, resetButton }) {
  if (!canvas) {
    return {
      render() {},
    };
  }

  const TRAIL_TTL_MS = 45_000;
  const TRAIL_MIN_DISTANCE = 0.05;
  const TRAIL_MIN_TIME_MS = 250;

  const ctx = canvas.getContext('2d');
  const viewState = {
    center: null,
    zoom: 1,
    baseScale: 1,
    autoCenter: true,
    autoFit: true,
    hasFit: false,
  };
  const dragState = {
    active: false,
    pointerId: null,
    startX: 0,
    startY: 0,
    startCenter: null,
  };

  let devicePixelRatioCache = window.devicePixelRatio || 1;
  let latestState = null;
  let cursorWorld = null;
  let trailSamples = [];

  function render(state) {
    latestState = state || null;
    updateTrail(latestState);
    drawMap(latestState);
  }

  function drawMap(state) {
    ensureCanvasSize();

    const dpr = devicePixelRatioCache;
    const width = canvas.width / dpr;
    const height = canvas.height / dpr;

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#232126';
    ctx.fillRect(0, 0, width, height);

    if (!state) {
      updateCursorLabel(cursorWorld);
      return;
    }

    const robot = state.robot || {};
    const robotPosRaw = robot.position ? normalizePoint(robot.position) : null;
    const robotPos = isValidPoint(robotPosRaw) ? robotPosRaw : null;
    const trail = trailSamples.map((sample) => ({ x: sample.x, y: sample.y }));
    const persistentObjects = state.persistent_map && Array.isArray(state.persistent_map.objects)
      ? state.persistent_map.objects.filter((obj) => obj && isValidPoint(obj.world))
      : [];
    const navGoal = (state.navigation && state.navigation.goal && state.navigation.goal.position && isValidPoint(state.navigation.goal.position))
      ? state.navigation.goal
      : null;
    const target = state.target && state.target.world && isValidPoint(state.target.world) ? state.target : null;

    const points = [];
    if (robotPos) points.push(robotPos);
    trail.forEach((point) => points.push(point));
    persistentObjects.forEach((object) => points.push(object.world));
    if (navGoal) points.push(navGoal.position);
    if (target) points.push(target.world);

    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;

    if (points.length === 0) {
      minX = -4;
      maxX = 4;
      minY = -3;
      maxY = 3;
    } else {
      for (const point of points) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
        if (point.y < minY) minY = point.y;
        if (point.y > maxY) maxY = point.y;
      }
    }

    const spanXRaw = maxX - minX;
    const spanYRaw = maxY - minY;
    const margin = Math.max(1, Math.max(spanXRaw, spanYRaw) * 0.2);

    const bounds = {
      minX: minX - margin,
      maxX: maxX + margin,
      minY: minY - margin,
      maxY: maxY + margin,
    };

    const spanX = Math.max(bounds.maxX - bounds.minX, 1);
    const spanY = Math.max(bounds.maxY - bounds.minY, 1);
    const fitScale = Math.min(width / spanX, height / spanY);
    const resolvedScale = Number.isFinite(fitScale) && fitScale > 0
      ? fitScale
      : Math.min(width, height) / 8;

    if (viewState.autoFit || !viewState.hasFit || !Number.isFinite(viewState.baseScale) || viewState.baseScale <= 0) {
      viewState.baseScale = resolvedScale;
      viewState.hasFit = true;
    }

    const centerX = (bounds.minX + bounds.maxX) / 2;
    const centerY = (bounds.minY + bounds.maxY) / 2;

    if (!viewState.center || viewState.autoCenter) {
      viewState.center = { x: centerX, y: centerY };
    }

    const scale = viewState.baseScale * viewState.zoom;
    const navigationStatus = state.navigation ? state.navigation.status : undefined;

    drawGrid(viewState.center, scale, width, height);
    drawTrail([...trail, robotPos].filter(isValidPoint), scale);
    drawPersistentObjects(persistentObjects, scale);

    if (robotPos && target) {
      drawTargetLink(robotPos, target, scale);
    }

    if (navGoal) {
      drawNavigationGoal(navGoal, navigationStatus, scale);
    }

    if (robotPos) {
      drawRobot(robotPos, robot.yaw, scale);
    }

    updateCursorLabel(cursorWorld);
  }

  function updateTrail(state) {
    const nowMs = getStateTimestampMs(state);
    pruneTrail(nowMs);

    if (!state || !state.robot || !state.robot.position) {
      return;
    }

    const point = normalizePoint(state.robot.position);
    if (!isValidPoint(point)) {
      return;
    }

    const last = trailSamples[trailSamples.length - 1];
    const timeSinceLast = last ? nowMs - last.time : Infinity;
    const distanceSinceLast = last ? Math.hypot(point.x - last.x, point.y - last.y) : Infinity;

    if (distanceSinceLast < TRAIL_MIN_DISTANCE && timeSinceLast < TRAIL_MIN_TIME_MS) {
      return;
    }

    trailSamples = [...trailSamples, { x: point.x, y: point.y, time: nowMs }];
    if (trailSamples.length > 600) {
      trailSamples = trailSamples.slice(-600);
    }
  }

  function drawGrid(center, scale, width, height) {
    const worldPerPixel = 1 / scale;
    const left = center.x - (width / 2) * worldPerPixel;
    const right = center.x + (width / 2) * worldPerPixel;
    const bottom = center.y - (height / 2) * worldPerPixel;
    const top = center.y + (height / 2) * worldPerPixel;

    const span = Math.max(right - left, top - bottom);
    const step = chooseGridStep(span);

    const startX = Math.floor(left / step) * step;
    const endX = Math.ceil(right / step) * step;
    const startY = Math.floor(bottom / step) * step;
    const endY = Math.ceil(top / step) * step;

    ctx.save();
    ctx.lineWidth = 1;

    for (let x = startX; x <= endX; x += step) {
      const alpha = Math.abs(x) < 1e-6 ? 0.25 : 0.12;
      ctx.strokeStyle = `rgba(252,252,250,${alpha})`;
      ctx.beginPath();
      const screen = worldToScreen({ x, y: bottom }, center, scale, width, height);
      ctx.moveTo(screen.x, screen.y);
      const screenTop = worldToScreen({ x, y: top }, center, scale, width, height);
      ctx.lineTo(screenTop.x, screenTop.y);
      ctx.stroke();
    }

    for (let y = startY; y <= endY; y += step) {
      const alpha = Math.abs(y) < 1e-6 ? 0.25 : 0.12;
      ctx.strokeStyle = `rgba(252,252,250,${alpha})`;
      ctx.beginPath();
      const screen = worldToScreen({ x: left, y }, center, scale, width, height);
      ctx.moveTo(screen.x, screen.y);
      const screenRight = worldToScreen({ x: right, y }, center, scale, width, height);
      ctx.lineTo(screenRight.x, screenRight.y);
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawTrail(trail, scale) {
    if (!Array.isArray(trail) || trail.length === 0) {
      return;
    }

    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const center = viewState.center;

    if (!center) {
      return;
    }

    const projected = trail
      .map((point) => worldToScreen(point, center, scale, width, height))
      .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));

    if (projected.length === 0) {
      return;
    }

    ctx.save();
    if (projected.length >= 2) {
      ctx.beginPath();
      ctx.moveTo(projected[0].x, projected[0].y);
      for (let i = 1; i < projected.length; i += 1) {
        ctx.lineTo(projected[i].x, projected[i].y);
      }
      ctx.strokeStyle = 'rgba(252, 152, 103, 0.9)';
      ctx.lineWidth = 3.5;
      ctx.lineCap = 'round';
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawPersistentObjects(objects, scale) {
    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const center = viewState.center;

    ctx.save();
    ctx.font = '12px "Inter", sans-serif';
    ctx.textBaseline = 'bottom';

    for (const object of objects) {
      const point = worldToScreen(object.world, center, scale, width, height);
      const radius = object.is_target ? 14 : 11;
      ctx.beginPath();
      ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
      ctx.fillStyle = object.is_target ? 'rgba(255, 97, 136, 0.92)' : 'rgba(171, 157, 242, 0.92)';
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = 'rgba(35, 33, 38, 0.9)';
      ctx.stroke();

      if (object.class_name) {
        ctx.fillStyle = 'rgba(252, 252, 250, 0.88)';
        ctx.fillText(object.class_name, point.x + radius + 4, point.y - radius - 2);
      }
    }

    ctx.restore();
  }

  function drawTargetLink(robotPos, target, scale) {
    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const center = viewState.center;
    const robotPoint = worldToScreen(robotPos, center, scale, width, height);
    const targetPoint = worldToScreen(target.world, center, scale, width, height);

    ctx.save();
    ctx.strokeStyle = 'rgba(255, 97, 136, 0.75)';
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 6]);
    ctx.beginPath();
    ctx.moveTo(robotPoint.x, robotPoint.y);
    ctx.lineTo(targetPoint.x, targetPoint.y);
    ctx.stroke();
    ctx.restore();

    if (typeof target.distance_to_robot === 'number') {
      const labelX = (robotPoint.x + targetPoint.x) / 2;
      const labelY = (robotPoint.y + targetPoint.y) / 2;
      ctx.save();
      ctx.font = '12px "Inter", sans-serif';
      ctx.fillStyle = 'rgba(255, 97, 136, 0.9)';
      ctx.fillText(`${target.distance_to_robot.toFixed(2)} m`, labelX + 6, labelY - 6);
      ctx.restore();
    }

    ctx.save();
    ctx.beginPath();
    ctx.arc(targetPoint.x, targetPoint.y, 10, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255, 97, 136, 0.7)';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
  }

  function drawNavigationGoal(navGoal, status, scale) {
    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const center = viewState.center;
    const point = worldToScreen(navGoal.position, center, scale, width, height);

    ctx.save();
    ctx.strokeStyle = 'rgba(120, 220, 232, 0.95)';
    ctx.lineWidth = 2.5;
    ctx.strokeRect(point.x - 14, point.y - 14, 28, 28);
    ctx.beginPath();
    ctx.moveTo(point.x - 14, point.y);
    ctx.lineTo(point.x + 14, point.y);
    ctx.moveTo(point.x, point.y - 14);
    ctx.lineTo(point.x, point.y + 14);
    ctx.stroke();

    if (typeof navGoal.yaw === 'number') {
      const length = 32;
      const head = 9;
      const dirX = Math.cos(navGoal.yaw);
      const dirY = Math.sin(navGoal.yaw);
      const tipX = point.x + dirX * length;
      const tipY = point.y - dirY * length;
      ctx.beginPath();
      ctx.moveTo(point.x, point.y);
      ctx.lineTo(tipX, tipY);
      ctx.stroke();

      const norm = Math.hypot(dirX, dirY) || 1;
      const nX = dirX / norm;
      const nY = -dirY / norm;
      const pX = -nY;
      const pY = nX;
      const leftX = tipX - nX * head + pX * head * 0.8;
      const leftY = tipY - nY * head + pY * head * 0.8;
      const rightX = tipX - nX * head - pX * head * 0.8;
      const rightY = tipY - nY * head - pY * head * 0.8;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(leftX, leftY);
      ctx.lineTo(rightX, rightY);
      ctx.closePath();
      ctx.fillStyle = 'rgba(120, 220, 232, 0.95)';
      ctx.fill();
    }

    ctx.restore();

    if (status) {
      ctx.save();
      ctx.font = '12px "Inter", sans-serif';
      ctx.fillStyle = 'rgba(120, 220, 232, 0.9)';
      ctx.fillText(String(status), point.x + 14, point.y - 12);
      ctx.restore();
    }
  }

  function drawRobot(robotPos, yaw, scale) {
    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const center = viewState.center;
    const point = worldToScreen(robotPos, center, scale, width, height);

    ctx.save();
    ctx.beginPath();
    ctx.arc(point.x, point.y, 14, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(169, 220, 118, 0.95)';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(35, 33, 38, 0.9)';
    ctx.stroke();

    if (typeof yaw === 'number') {
      const dirX = Math.cos(yaw);
      const dirY = Math.sin(yaw);
      const length = 38;
      const tipX = point.x + dirX * length;
      const tipY = point.y - dirY * length;
      ctx.beginPath();
      ctx.moveTo(point.x, point.y);
      ctx.lineTo(tipX, tipY);
      ctx.strokeStyle = 'rgba(169, 220, 118, 0.95)';
      ctx.lineWidth = 3;
      ctx.lineCap = 'round';
      ctx.stroke();

      const norm = Math.hypot(dirX, dirY) || 1;
      const nX = dirX / norm;
      const nY = -dirY / norm;
      const pX = -nY;
      const pY = nX;
      const head = 9;
      const leftX = tipX - nX * head + pX * head * 0.7;
      const leftY = tipY - nY * head + pY * head * 0.7;
      const rightX = tipX - nX * head - pX * head * 0.7;
      const rightY = tipY - nY * head - pY * head * 0.7;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(leftX, leftY);
      ctx.lineTo(rightX, rightY);
      ctx.closePath();
      ctx.fillStyle = 'rgba(169, 220, 118, 0.95)';
      ctx.fill();
    }

    ctx.restore();
  }

  function worldToScreen(point, center, scale, width, height) {
    return {
      x: width / 2 + (point.x - center.x) * scale,
      y: height / 2 - (point.y - center.y) * scale,
    };
  }

  function screenToWorld(x, y) {
    const width = canvas.width / devicePixelRatioCache;
    const height = canvas.height / devicePixelRatioCache;
    const scale = viewState.baseScale * viewState.zoom;
    if (!viewState.center) {
      return { x: 0, y: 0 };
    }
    return {
      x: viewState.center.x + (x - width / 2) / scale,
      y: viewState.center.y - (y - height / 2) / scale,
    };
  }

  function chooseGridStep(span) {
    if (span > 80) return 20;
    if (span > 40) return 10;
    if (span > 20) return 5;
    if (span > 12) return 2;
    if (span > 6) return 1;
    if (span > 3) return 0.5;
    return 0.25;
  }

  function normalizePoint(point) {
    if (!point) {
      return null;
    }

    const x = typeof point.x === 'number' ? point.x : parseFloat(point.x);
    const y = typeof point.y === 'number' ? point.y : parseFloat(point.y);

    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      return null;
    }

    return { x, y };
  }

  function isValidPoint(point) {
    return (
      point &&
      Number.isFinite(point.x) &&
      Number.isFinite(point.y)
    );
  }

  function ensureCanvasSize() {
    const parent = canvas.parentElement;
    if (!parent) {
      return;
    }
    const rect = parent.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    devicePixelRatioCache = dpr;
    const width = Math.max(rect.width, 320);
    const height = Math.max(rect.height, 240);
    const displayWidth = Math.round(width * dpr);
    const displayHeight = Math.round(height * dpr);

    if (canvas.width !== displayWidth || canvas.height !== displayHeight) {
      canvas.width = displayWidth;
      canvas.height = displayHeight;
      canvas.style.width = `${width}px`;
      canvas.style.height = `${height}px`;
    }
  }

  function updateCursorLabel(worldPoint) {
    if (!statusCursorEl) {
      return;
    }
    if (!worldPoint || !Number.isFinite(worldPoint.x) || !Number.isFinite(worldPoint.y)) {
      statusCursorEl.textContent = 'Cursor: —';
      return;
    }
    statusCursorEl.textContent = `Cursor: (${formatNumber(worldPoint.x)}, ${formatNumber(worldPoint.y)})`;
  }

  function pruneTrail(_nowMs) {
    // Trail pruning disabled to preserve the full history on the canvas.
    // const cutoff = _nowMs - TRAIL_TTL_MS;
    // if (!Number.isFinite(cutoff)) {
    //   return;
    // }
    // trailSamples = trailSamples.filter((sample) => sample.time >= cutoff);
  }

  function getStateTimestampMs(state) {
    if (state && typeof state.timestamp === 'number') {
      return state.timestamp * 1000;
    }
    return Date.now();
  }

  function handlePointerDown(event) {
    updateCursorFromEvent(event);
    if (typeof canvas.setPointerCapture === 'function') {
      canvas.setPointerCapture(event.pointerId);
    }
    dragState.active = true;
    dragState.pointerId = event.pointerId;
    dragState.startX = event.clientX;
    dragState.startY = event.clientY;
    dragState.startCenter = viewState.center ? { ...viewState.center } : null;
    viewState.autoFit = false;
  }

  function handlePointerMove(event) {
    updateCursorFromEvent(event);
    if (!dragState.active || dragState.pointerId !== event.pointerId || !dragState.startCenter) {
      return;
    }

    const dx = event.clientX - dragState.startX;
    const dy = event.clientY - dragState.startY;
    const scale = viewState.baseScale * viewState.zoom;
    if (!Number.isFinite(scale) || scale <= 0) {
      return;
    }

    const worldPerPixel = 1 / scale;
    viewState.center = {
      x: dragState.startCenter.x - dx * worldPerPixel,
      y: dragState.startCenter.y + dy * worldPerPixel,
    };
    viewState.autoCenter = false;
    viewState.autoFit = false;
    requestRender();
  }

  function handlePointerUp(event) {
    if (dragState.pointerId === event.pointerId) {
      dragState.active = false;
      dragState.pointerId = null;
    }
  }

  function handlePointerLeave() {
    cursorWorld = null;
    updateCursorLabel(null);
  }

  function handleWheel(event) {
    event.preventDefault();
    const delta = -event.deltaY;
    const zoomFactor = Math.exp(delta * 0.0015);
    const prevZoom = viewState.zoom;
    const nextZoom = clamp(prevZoom * zoomFactor, 0.25, 20);

    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const worldBefore = screenToWorld(x, y);

    viewState.zoom = nextZoom;
    viewState.autoCenter = false;
    viewState.autoFit = false;

    const worldAfter = screenToWorld(x, y);
    if (viewState.center) {
      viewState.center.x += worldBefore.x - worldAfter.x;
      viewState.center.y += worldBefore.y - worldAfter.y;
    }

    requestRender();
  }

  function handleDoubleClick() {
    resetView();
  }

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  function resetView() {
    viewState.zoom = 1;
    viewState.autoCenter = true;
    viewState.autoFit = true;
    viewState.baseScale = 1;
    viewState.hasFit = false;
    requestRender();
  }

  function updateCursorFromEvent(event) {
    if (!viewState.center) {
      cursorWorld = null;
      updateCursorLabel(null);
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    cursorWorld = screenToWorld(x, y);
    updateCursorLabel(cursorWorld);
  }

  function requestRender() {
    if (latestState) {
      drawMap(latestState);
    } else {
      drawMap(null);
    }
  }

  function observeCanvasSize() {
    const parent = canvas.parentElement;
    if (!parent) {
      return;
    }
    if (window.ResizeObserver) {
      const observer = new ResizeObserver(() => {
        ensureCanvasSize();
        requestRender();
      });
      observer.observe(parent);
    } else {
      window.addEventListener('resize', () => {
        ensureCanvasSize();
        requestRender();
      });
    }
  }

  canvas.addEventListener('pointerdown', handlePointerDown);
  canvas.addEventListener('pointermove', handlePointerMove);
  canvas.addEventListener('pointerup', handlePointerUp);
  canvas.addEventListener('pointercancel', handlePointerUp);
  canvas.addEventListener('pointerleave', handlePointerLeave);
  canvas.addEventListener('wheel', handleWheel, { passive: false });
  canvas.addEventListener('dblclick', handleDoubleClick);

  if (resetButton) {
    resetButton.addEventListener('click', resetView);
  }

  observeCanvasSize();
  ensureCanvasSize();

  return {
    render,
  };
}
