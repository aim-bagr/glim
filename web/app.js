// State
let ws = null;
let autoScroll = true;
let isJobRunning = false;

// 3D Visualizer State
let threeScene = null;
let threeCamera = null;
let threeRenderer = null;
let threeControls = null;
let threeGrid = null;
let currentRunSubmaps = [];
let currentRunName = null;
let activeSubmapId = 0;
let submapCache = new Map(); // id -> THREE.Points
let submapMarkers = null; // THREE.Points for origin spheres
let trajLine = null; // THREE.Line
let pointSize = 2.0;
let colorMode = 'rainbow';
let isIsolated = true;

// DOM Elements
const statusBadge = document.getElementById('service-status-badge');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const wsIndicator = document.getElementById('ws-indicator');
const wsText = document.getElementById('ws-text');

const datasetSelect = document.getElementById('dataset-select');
const runNameInput = document.getElementById('run-name-input');
const startBtn = document.getElementById('start-btn');
const stopBtn = document.getElementById('stop-btn');
const launcherForm = document.getElementById('launcher-form');
const runsList = document.getElementById('runs-list');
const logConsole = document.getElementById('log-console');
const autoScrollToggle = document.getElementById('autoscroll-toggle');

const metricBagTime = document.getElementById('metric-bag-time');
const metricSpeed = document.getElementById('metric-speed');
const metricScans = document.getElementById('metric-scans');
const metricElapsed = document.getElementById('metric-elapsed');
const metricQueueOdom = document.getElementById('metric-queue-odom');
const metricQueueSub = document.getElementById('metric-queue-sub');
const metricQueueGlob = document.getElementById('metric-queue-glob');

// Tab Elements
const tabBtn3d = document.getElementById('tab-btn-3d');
const tabBtnConsole = document.getElementById('tab-btn-console');
const tabContent3d = document.getElementById('tab-content-3d');
const tabContentConsole = document.getElementById('tab-content-console');

// 3D Controls Elements
const viewerRunSelect = document.getElementById('viewer-run-select');
const viewerLoadBtn = document.getElementById('viewer-load-btn');
const viewerIsolateToggle = document.getElementById('viewer-isolate-toggle');
const viewerPrevBtn = document.getElementById('viewer-prev-btn');
const viewerNextBtn = document.getElementById('viewer-next-btn');
const viewerSubmapSlider = document.getElementById('viewer-submap-slider');
const viewerFocusBtn = document.getElementById('viewer-focus-btn');
const viewerColorMode = document.getElementById('viewer-color-mode');
const viewerPointSize = document.getElementById('viewer-point-size');
const viewportContainer = document.getElementById('viewport-3d');
const viewportPlaceholder = document.getElementById('viewport-placeholder');
const viewerHud = document.getElementById('viewer-hud');
const hudSubmapText = document.getElementById('hud-submap-text');

// Initialize
document.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) lucide.createIcons();
  fetchDatasets();
  fetchRuns();
  connectWebSocket();
  setupTabs();
  setup3DViewerControls();

  // Launcher & Console Listeners
  launcherForm.addEventListener('submit', handleStartJob);
  stopBtn.addEventListener('click', handleStopJob);
  document.getElementById('refresh-datasets-btn').addEventListener('click', fetchDatasets);
  document.getElementById('refresh-runs-btn').addEventListener('click', fetchRuns);
  document.getElementById('clear-logs-btn').addEventListener('click', () => {
    logConsole.innerHTML = '';
  });
  autoScrollToggle.addEventListener('change', (e) => {
    autoScroll = e.target.checked;
  });
});

// Tab Setup
function setupTabs() {
  tabBtn3d.addEventListener('click', () => switchTab('3d'));
  tabBtnConsole.addEventListener('click', () => switchTab('console'));
}

function switchTab(tab) {
  if (tab === '3d') {
    tabBtn3d.className = 'px-4 py-2 text-xs font-semibold rounded-lg bg-blue-600/20 text-blue-400 border border-blue-500/30 flex items-center gap-2 transition';
    tabBtnConsole.className = 'px-4 py-2 text-xs font-medium rounded-lg text-slate-400 hover:text-slate-200 border border-transparent hover:bg-slate-800/50 flex items-center gap-2 transition';
    tabContent3d.classList.remove('hidden');
    tabContentConsole.classList.add('hidden');
    onViewportResize();
  } else {
    tabBtnConsole.className = 'px-4 py-2 text-xs font-semibold rounded-lg bg-blue-600/20 text-blue-400 border border-blue-500/30 flex items-center gap-2 transition';
    tabBtn3d.className = 'px-4 py-2 text-xs font-medium rounded-lg text-slate-400 hover:text-slate-200 border border-transparent hover:bg-slate-800/50 flex items-center gap-2 transition';
    tabContentConsole.classList.remove('hidden');
    tabContent3d.classList.add('hidden');
  }
  if (window.lucide) lucide.createIcons();
}

// WebSocket Connection
function connectWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/api/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    wsIndicator.className = 'flex items-center gap-1.5 text-xs text-emerald-400';
    wsText.textContent = 'Live';
  };

  ws.onclose = () => {
    wsIndicator.className = 'flex items-center gap-1.5 text-xs text-slate-500';
    wsText.textContent = 'Reconnecting...';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'status') {
        updateJobState(msg.data);
      } else if (msg.type === 'log') {
        appendLog(msg.line);
        if (msg.progress) {
          updateMetrics(msg.progress);
        }
      }
    } catch (e) {
      console.error('WS Parse Error', e);
    }
  };
}

// Fetch Datasets
async function fetchDatasets() {
  try {
    const res = await fetch('/api/datasets');
    const datasets = await res.json();
    datasetSelect.innerHTML = '';

    if (!datasets || datasets.length === 0) {
      datasetSelect.innerHTML = '<option value="">No .mcap datasets found in /data</option>';
      return;
    }

    datasets.forEach(ds => {
      const opt = document.createElement('option');
      opt.value = ds.path;
      opt.textContent = `${ds.name} (${ds.size_human})`;
      datasetSelect.appendChild(opt);
    });

    if (datasets.length > 0 && !runNameInput.value) {
      const baseName = datasets[0].name.replace(/\.[^/.]+$/, '').replace(/[^a-zA-Z0-9_-]/g, '_');
      runNameInput.placeholder = `run_${baseName.substring(0, 16)}`;
    }
  } catch (err) {
    datasetSelect.innerHTML = '<option value="">Failed to fetch datasets</option>';
  }
}

// Fetch Historical Runs
async function fetchRuns() {
  try {
    const res = await fetch('/api/runs');
    const runs = await res.json();
    runsList.innerHTML = '';
    viewerRunSelect.innerHTML = '<option value="">Select a run...</option>';

    if (!runs || runs.length === 0) {
      runsList.innerHTML = '<div class="text-xs text-slate-500 py-4 text-center">No completed runs found yet.</div>';
      return;
    }

    runs.forEach(run => {
      // Add to viewer dropdown
      const opt = document.createElement('option');
      opt.value = run.name;
      opt.textContent = `${run.name} (${run.submaps_count} submaps)`;
      viewerRunSelect.appendChild(opt);

      // Add to list
      const item = document.createElement('div');
      item.className = 'bg-slate-950/70 border border-slate-800/80 rounded-lg p-3 hover:border-slate-700 transition flex items-center justify-between gap-3';
      item.innerHTML = `
        <div class="min-w-0">
          <div class="text-xs font-semibold text-slate-200 truncate">${run.name}</div>
          <div class="text-[11px] text-slate-400 flex items-center gap-2 mt-0.5">
            <span>${run.modified_at}</span>
            <span>•</span>
            <span class="text-blue-400 font-medium">${run.submaps_count} submaps</span>
            <span>•</span>
            <span>${run.size_human}</span>
          </div>
        </div>
        <div class="flex items-center gap-1.5 shrink-0">
          <button onclick="openRunIn3D('${run.name}')" class="p-1.5 bg-blue-600/20 hover:bg-blue-600/30 text-blue-400 rounded text-xs flex items-center gap-1 transition" title="View 3D Submaps & Trajectory">
            <i data-lucide="box" class="w-3.5 h-3.5"></i>
            <span class="text-[11px] font-medium hidden sm:inline">3D</span>
          </button>
          ${run.has_traj_lidar ? `
            <a href="/api/runs/${encodeURIComponent(run.name)}/trajectory?opt=true" download="${run.name}_traj_lidar.txt" class="p-1.5 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded text-xs flex items-center gap-1 transition" title="Download Optimized LiDAR Trajectory (TUM)">
              <i data-lucide="download" class="w-3.5 h-3.5"></i>
            </a>` : ''}
        </div>
      `;
      runsList.appendChild(item);
    });

    if (window.lucide) lucide.createIcons();

    // Auto-select first run in viewer dropdown
    if (runs.length > 0 && !currentRunName) {
      viewerRunSelect.value = runs[0].name;
    }
  } catch (err) {
    runsList.innerHTML = '<div class="text-xs text-rose-500 py-4 text-center">Failed to load run history</div>';
  }
}

// Open run directly in 3D
window.openRunIn3D = function(runName) {
  viewerRunSelect.value = runName;
  switchTab('3d');
  loadMapRun(runName);
};

// Start SLAM Job
async function handleStartJob(e) {
  e.preventDefault();
  const datasetPath = datasetSelect.value;
  if (!datasetPath) {
    alert('Please select a dataset.');
    return;
  }

  const payload = {
    dataset_path: datasetPath,
    run_name: runNameInput.value.trim() || null,
    max_num_keyframes: parseInt(document.getElementById('param-keyframes').value, 10),
    submap_target_num_points: parseInt(document.getElementById('param-points').value, 10),
    min_travel_dist: parseFloat(document.getElementById('param-min-travel').value),
    start_offset_sec: parseFloat(document.getElementById('param-start-offset').value)
  };

  startBtn.disabled = true;
  startBtn.textContent = 'Launching...';

  try {
    const res = await fetch('/api/jobs/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    if (!res.ok) {
      const err = await res.json();
      throw new Error(err.detail || 'Failed to start job');
    }
    const job = await res.json();
    updateJobState(job);
    switchTab('console');
  } catch (err) {
    alert(err.message);
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    if (window.lucide) lucide.createIcons();
  }
}

// Stop SLAM Job
async function handleStopJob() {
  if (!confirm('Stop current SLAM run? Final submaps and trajectory will be flushed to disk.')) {
    return;
  }
  stopBtn.disabled = true;
  stopBtn.textContent = 'Finalizing...';
  try {
    await fetch('/api/jobs/stop', { method: 'POST' });
  } catch (err) {
    alert('Error stopping job: ' + err.message);
    stopBtn.disabled = false;
  }
}

// Update Job State UI
function updateJobState(job) {
  isJobRunning = job.state === 'running' || job.state === 'finalizing';

  if (job.state === 'running') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-blue-900/30 border-blue-700/50 text-blue-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-blue-400 status-running';
    statusText.textContent = `Running: ${job.run_name || ''}`;
    startBtn.classList.add('hidden');
    stopBtn.classList.remove('hidden');
    stopBtn.disabled = false;
    stopBtn.innerHTML = '<i data-lucide="square" class="w-3.5 h-3.5 fill-current"></i> Stop & Finalize';
  } else if (job.state === 'finalizing') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-amber-900/30 border-amber-700/50 text-amber-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-amber-400 status-running';
    statusText.textContent = 'Finalizing & Saving Map...';
    stopBtn.disabled = true;
    stopBtn.textContent = 'Saving...';
  } else if (job.state === 'completed' || job.state === 'stopped') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-emerald-900/30 border-emerald-700/50 text-emerald-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-emerald-400';
    statusText.textContent = job.state === 'completed' ? 'Completed' : 'Stopped & Saved';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
    fetchRuns();
  } else if (job.state === 'failed') {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-rose-900/30 border-rose-700/50 text-rose-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-rose-500';
    statusText.textContent = 'Failed';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
    fetchRuns();
  } else {
    statusBadge.className = 'flex items-center gap-2 px-3 py-1 rounded-full text-xs font-medium border bg-slate-800/80 border-slate-700 text-slate-300';
    statusDot.className = 'h-2 w-2 rounded-full bg-slate-500';
    statusText.textContent = 'Idle';
    startBtn.classList.remove('hidden');
    startBtn.disabled = false;
    startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
    stopBtn.classList.add('hidden');
  }

  updateMetrics(job);
  if (window.lucide) lucide.createIcons();
}

// Update Metrics Display
function updateMetrics(job) {
  if (!job) return;
  metricBagTime.textContent = (job.bag_time || 0).toFixed(1) + 's';
  metricSpeed.textContent = (job.speed || 0).toFixed(2) + 'x';
  metricScans.textContent = (job.scans || 0).toLocaleString();
  metricElapsed.textContent = (job.elapsed_sec || 0).toFixed(1) + 's';

  metricQueueOdom.textContent = job.odom_queue || 0;
  metricQueueSub.textContent = job.sub_queue || 0;
  metricQueueGlob.textContent = job.glob_queue || 0;
}

// Append Log Line
function appendLog(line) {
  const div = document.createElement('div');
  div.className = 'log-line';

  if (line.includes('[error]') || line.includes('[critical]')) {
    div.classList.add('log-err');
  } else if (line.includes('[warning]') || line.includes('[warn]')) {
    div.classList.add('log-warn');
  } else if (line.includes('[progress]')) {
    div.classList.add('log-progress');
  } else {
    div.classList.add('log-info');
  }

  div.textContent = line;
  logConsole.appendChild(div);

  if (autoScroll) {
    logConsole.scrollTop = logConsole.scrollHeight;
  }
}

// =============================================================================
// Three.js 3D WebGL Viewer & Submap Isolation
// =============================================================================

function initThreeScene() {
  if (threeRenderer) return;

  const width = viewportContainer.clientWidth;
  const height = viewportContainer.clientHeight;

  threeScene = new THREE.Scene();
  threeScene.background = new THREE.Color(0x020617); // Slate 950

  threeCamera = new THREE.PerspectiveCamera(55, width / height, 0.1, 10000);
  threeCamera.position.set(0, -30, 40);
  threeCamera.up.set(0, 0, 1); // Z-up for LiDAR/SLAM coordinates!

  threeRenderer = new THREE.WebGLRenderer({ antialias: true });
  threeRenderer.setSize(width, height);
  threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

  viewportContainer.innerHTML = '';
  viewportContainer.appendChild(threeRenderer.domElement);
  viewportContainer.appendChild(viewerHud);

  threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
  threeControls.enableDamping = true;
  threeControls.dampingFactor = 0.08;
  threeControls.screenSpacePanning = true;

  // Grid
  threeGrid = new THREE.GridHelper(200, 40, 0x334155, 0x1e293b);
  threeGrid.rotation.x = Math.PI / 2; // Orient grid to XY ground plane with Z-up
  threeScene.add(threeGrid);

  // Lights
  const ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
  threeScene.add(ambientLight);

  window.addEventListener('resize', onViewportResize);

  function animate() {
    requestAnimationFrame(animate);
    threeControls.update();
    threeRenderer.render(threeScene, threeCamera);
  }
  animate();
}

function onViewportResize() {
  if (!threeRenderer || !viewportContainer) return;
  const width = viewportContainer.clientWidth;
  const height = viewportContainer.clientHeight;
  if (width === 0 || height === 0) return;
  threeCamera.aspect = width / height;
  threeCamera.updateProjectionMatrix();
  threeRenderer.setSize(width, height);
}

function setup3DViewerControls() {
  viewerIsolateToggle.checked = true;
  viewerLoadBtn.addEventListener('click', () => {
    const run = viewerRunSelect.value;
    if (run) loadMapRun(run);
  });

  viewerIsolateToggle.addEventListener('change', (e) => {
    isIsolated = e.target.checked;
    updateSubmapVisibility();
  });

  viewerPrevBtn.addEventListener('click', () => {
    if (activeSubmapId > 0) {
      setSubmap(activeSubmapId - 1);
    }
  });

  viewerNextBtn.addEventListener('click', () => {
    if (activeSubmapId < currentRunSubmaps.length - 1) {
      setSubmap(activeSubmapId + 1);
    }
  });

  viewerSubmapSlider.addEventListener('input', (e) => {
    setSubmap(parseInt(e.target.value, 10));
  });

  viewerFocusBtn.addEventListener('click', () => {
    focusCameraOnSubmap(activeSubmapId);
  });

  viewerPointSize.addEventListener('input', (e) => {
    pointSize = parseFloat(e.target.value);
    submapCache.forEach(points => {
      points.material.size = pointSize;
    });
  });

  viewerColorMode.addEventListener('change', (e) => {
    colorMode = e.target.value;
    // Re-apply colors to cached submaps
    submapCache.forEach((points, id) => {
      applySubmapColors(points.geometry, id);
    });
  });
}

// Load Full Map Run
async function loadMapRun(runName) {
  initThreeScene();
  currentRunName = runName;
  viewerHud.classList.remove('hidden');
  hudSubmapText.textContent = `Loading ${runName} metadata...`;

  // 1. Clear previous submaps & trajectory
  submapCache.forEach(p => threeScene.remove(p));
  submapCache.clear();
  if (trajLine) {
    threeScene.remove(trajLine);
    trajLine = null;
  }
  if (submapMarkers) {
    threeScene.remove(submapMarkers);
    submapMarkers = null;
  }

  // 2. Load Trajectory (Binary)
  try {
    const trajRes = await fetch(`/api/runs/${encodeURIComponent(runName)}/trajectory_binary?opt=true`);
    if (trajRes.ok) {
      const buffer = await trajRes.arrayBuffer();
      const coords = new Float32Array(buffer);
      const trajGeom = new THREE.BufferGeometry();
      trajGeom.setAttribute('position', new THREE.BufferAttribute(coords, 3));
      const trajMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2 });
      trajLine = new THREE.Line(trajGeom, trajMat);
      threeScene.add(trajLine);
    }
  } catch (err) {
    console.warn('Could not load trajectory binary', err);
  }

  // 3. Load Submap Metadata
  try {
    const subRes = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
    if (!subRes.ok) throw new Error('Failed to load submaps');
    currentRunSubmaps = await subRes.json();

    if (currentRunSubmaps.length > 0) {
      viewerSubmapSlider.min = 0;
      viewerSubmapSlider.max = currentRunSubmaps.length - 1;
      viewerSubmapSlider.value = 0;

      // Render Submap Centers as markers
      const markerCoords = new Float32Array(currentRunSubmaps.length * 3);
      currentRunSubmaps.forEach((sm, i) => {
        markerCoords[i * 3] = sm.pos[0];
        markerCoords[i * 3 + 1] = sm.pos[1];
        markerCoords[i * 3 + 2] = sm.pos[2];
      });
      const markerGeom = new THREE.BufferGeometry();
      markerGeom.setAttribute('position', new THREE.BufferAttribute(markerCoords, 3));
      const markerMat = new THREE.PointsMaterial({ color: 0xf43f5e, size: 5.0, sizeAttenuation: false });
      submapMarkers = new THREE.Points(markerGeom, markerMat);
      threeScene.add(submapMarkers);

      // Load initial submap & focus camera
      setSubmap(0, /*autoFocus=*/true);
    }
  } catch (err) {
    hudSubmapText.textContent = `Error loading submaps: ${err.message}`;
  }
}

// Select Active Submap
async function setSubmap(submapId, autoFocus = false) {
  if (submapId < 0 || submapId >= currentRunSubmaps.length) return;
  activeSubmapId = submapId;
  viewerSubmapSlider.value = submapId;

  const sm = currentRunSubmaps[submapId];
  hudSubmapText.textContent = `Submap #${sm.id} / ${currentRunSubmaps.length - 1} | Points: ${(sm.num_points || 0).toLocaleString()} | Pos: (${sm.pos[0].toFixed(1)}, ${sm.pos[1].toFixed(1)}, ${sm.pos[2].toFixed(1)})`;

  // Load points for this submap if not yet cached
  await loadSubmapPoints(submapId);
  updateSubmapVisibility();

  if (autoFocus) {
    focusCameraOnSubmap(submapId);
  }
}

// Load Submap Points (Binary Float32Array)
async function loadSubmapPoints(submapId) {
  if (submapCache.has(submapId)) return submapCache.get(submapId);

  const sm = currentRunSubmaps[submapId];
  if (!sm) return null;

  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(currentRunName)}/submaps/${submapId}/points`);
    if (!res.ok) return null;

    const buffer = await res.arrayBuffer();
    const positions = new Float32Array(buffer);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Apply 4x4 matrix from GLIM
    const matrix = new THREE.Matrix4();
    matrix.fromArray(sm.matrix);
    geom.applyMatrix4(matrix);

    // Compute Altitude Rainbow Colors
    applySubmapColors(geom, submapId);

    const mat = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true
    });

    const pointsObj = new THREE.Points(geom, mat);
    threeScene.add(pointsObj);
    submapCache.set(submapId, pointsObj);
    return pointsObj;
  } catch (err) {
    console.error(`Failed to load submap ${submapId} points:`, err);
    return null;
  }
}

// Altitude Rainbow Color Calculation (Turbo Colormap)
function applySubmapColors(geom, submapId) {
  const pos = geom.attributes.position;
  const count = pos.count;
  const colors = new Float32Array(count * 3);

  let minZ = Infinity, maxZ = -Infinity;
  for (let i = 0; i < count; i++) {
    const z = pos.getZ(i);
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  const rangeZ = Math.max(maxZ - minZ, 0.1);

  for (let i = 0; i < count; i++) {
    if (colorMode === 'flat') {
      colors[i * 3] = 0.9;
      colors[i * 3 + 1] = 0.9;
      colors[i * 3 + 2] = 0.9;
    } else {
      // Rainbow based on normalized Z
      const t = (pos.getZ(i) - minZ) / rangeZ;
      const rgb = turboColormap(t);
      colors[i * 3] = rgb[0];
      colors[i * 3 + 1] = rgb[1];
      colors[i * 3 + 2] = rgb[2];
    }
  }

  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geom.attributes.color.needsUpdate = true;
}

// Simple Turbo / Rainbow colormap
function turboColormap(t) {
  const r = Math.sin(t * Math.PI * 1.5);
  const g = Math.sin(t * Math.PI);
  const b = Math.cos(t * Math.PI * 1.5);
  return [Math.max(0, Math.min(1, r * 0.8 + 0.2)), Math.max(0, Math.min(1, g)), Math.max(0, Math.min(1, b * 0.9 + 0.1))];
}

// Update Submap Visibility based on Isolation Toggle
function updateSubmapVisibility() {
  submapCache.forEach((points, id) => {
    if (isIsolated) {
      points.visible = (id === activeSubmapId);
    } else {
      points.visible = true;
    }
  });
}

// Focus Camera smoothly on a Submap
function focusCameraOnSubmap(submapId) {
  const sm = currentRunSubmaps[submapId];
  if (!sm || !threeControls) return;

  const targetPos = new THREE.Vector3(sm.pos[0], sm.pos[1], sm.pos[2]);
  threeControls.target.copy(targetPos);
  threeCamera.position.set(sm.pos[0] - 15, sm.pos[1] - 15, sm.pos[2] + 12);
  threeControls.update();
}
