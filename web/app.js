// GLIM Headless SLAM Dashboard & Real-Time Three.js Visualizer
let ws = null;
let currentJobState = 'idle';
let activeLiveRunName = null;
let currentRunName = null;
let viewerMode = 'idle'; // 'idle' | 'live' | 'loaded'
let loadSessionId = 0;
let autoScroll = true;
let totalLogLines = 0;
let liveStreamTimer = null;

// Three.js State
let threeScene, threeCamera, threeRenderer, threeControls;
let currentRunSubmaps = [];
let submapCache = new Map(); // submap_id -> THREE.Points
let threeTrajectoryLine = null;
let threeLoopLines = null;
let currentRunLoops = [];
let isTrajVisible = true;
let isLoopsVisible = true;
let activeSubmapMarker = null;
let activeSubmapId = null;
let isIsolated = false;
let colorMode = 'rainbow';
let pointSize = 2.0;

// DOM Elements
const statusBadge = document.getElementById('service-status-badge');
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const wsIndicator = document.getElementById('ws-indicator');
const wsText = document.getElementById('ws-text');

// Floating Modals & Drawers
const launcherModal = document.getElementById('launcher-modal');
const runsModal = document.getElementById('runs-modal');
const consoleDrawer = document.getElementById('console-drawer');
const minConsoleBtn = document.getElementById('min-console-btn');

const toggleLauncherBtn = document.getElementById('toggle-launcher-btn');
const closeLauncherBtn = document.getElementById('close-launcher-btn');
const toggleRunsBtn = document.getElementById('toggle-runs-btn');
const closeRunsBtn = document.getElementById('close-runs-btn');
const toggleConsoleBtn = document.getElementById('toggle-console-btn');
const closeConsoleBtn = document.getElementById('close-console-btn');
const expandConsoleBtn = document.getElementById('expand-console-btn');
const switchToLiveBtn = document.getElementById('switch-to-live-btn');
let isConsoleWide = false;

// Launcher Form
const launcherForm = document.getElementById('launcher-form');
const datasetSelect = document.getElementById('dataset-select');
const runNameInput = document.getElementById('run-name-input');
const startBtn = document.getElementById('start-btn');
const headerStopBtn = document.getElementById('header-stop-btn');
const runsList = document.getElementById('runs-list');

// Log Console
const logConsole = document.getElementById('log-console');
const logCount = document.getElementById('log-count');
const autoScrollToggle = document.getElementById('autoscroll-toggle');
const clearLogsBtn = document.getElementById('clear-logs-btn');

// Top KPIs
const metricBagTime = document.getElementById('metric-bag-time');
const metricSpeed = document.getElementById('metric-speed');
const metricScans = document.getElementById('metric-scans');
const metricQueueOdom = document.getElementById('metric-queue-odom');
const metricQueueSub = document.getElementById('metric-queue-sub');
const metricQueueGlob = document.getElementById('metric-queue-glob');

// 3D Controls Elements
const viewportContainer = document.getElementById('viewport-3d');
const viewportPlaceholder = document.getElementById('viewport-placeholder');
const viewerModeBadge = document.getElementById('viewer-mode-badge');
const viewerRunSelect = document.getElementById('viewer-run-select');
const viewerLoadBtn = document.getElementById('viewer-load-btn');
const viewerClearBtn = document.getElementById('viewer-clear-btn');
const viewerIsolateToggle = document.getElementById('viewer-isolate-toggle');
const viewerPlayBtn = document.getElementById('viewer-play-btn');
const viewerPlayBtnLabel = document.getElementById('viewer-play-btn-label');
const viewerPlaySpeed = document.getElementById('viewer-play-speed');
const viewerPrevBtn = document.getElementById('viewer-prev-btn');
const viewerNextBtn = document.getElementById('viewer-next-btn');
const viewerSubmapSlider = document.getElementById('viewer-submap-slider');
const viewerFocusBtn = document.getElementById('viewer-focus-btn');
const viewerFitBtn = document.getElementById('viewer-fit-btn');
const viewerTopdownBtn = document.getElementById('viewer-topdown-btn');
const viewerTrajToggle = document.getElementById('viewer-traj-toggle');
const viewerLoopsToggle = document.getElementById('viewer-loops-toggle');
const hudLoopsCount = document.getElementById('hud-loops-count');
const viewerColorMode = document.getElementById('viewer-color-mode');
const viewerPointSize = document.getElementById('viewer-point-size');
const viewerHud = document.getElementById('viewer-hud');
const hudSubmapText = document.getElementById('hud-submap-text');

// Submap Playback State
let isPlayingSubmaps = false;
let playSubmapTimer = null;

// Toast System
let toastTimer = null;
function showToast(msg, type = 'info', duration = 2500) {
  const toast = document.getElementById('toast');
  const toastMsg = document.getElementById('toast-msg');
  const toastIcon = document.getElementById('toast-icon');
  if (!toast || !toastMsg) return;

  toastMsg.textContent = msg;

  if (toastIcon) {
    if (type === 'success') {
      toastIcon.setAttribute('data-lucide', 'check-circle');
      toastIcon.className = 'w-4 h-4 text-emerald-400 shrink-0';
    } else if (type === 'error') {
      toastIcon.setAttribute('data-lucide', 'alert-circle');
      toastIcon.className = 'w-4 h-4 text-rose-400 shrink-0';
    } else if (type === 'warn') {
      toastIcon.setAttribute('data-lucide', 'alert-triangle');
      toastIcon.className = 'w-4 h-4 text-amber-400 shrink-0';
    } else {
      toastIcon.setAttribute('data-lucide', 'info');
      toastIcon.className = 'w-4 h-4 text-blue-400 shrink-0';
    }
    if (window.lucide) lucide.createIcons();
  }

  toast.classList.remove('hidden');
  requestAnimationFrame(() => {
    toast.classList.remove('opacity-0');
    toast.classList.add('opacity-100');
  });

  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    toast.classList.remove('opacity-100');
    toast.classList.add('opacity-0');
    setTimeout(() => {
      toast.classList.add('hidden');
    }, 300);
  }, duration);
}

// Initialize Application
document.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) lucide.createIcons();

  initThreeJS();
  fetchDatasets();
  fetchRuns();
  connectWebSocket();
  setupModalToggles();
  setup3DViewerControls();
  updateViewerControlsState();

  // Launcher & Actions
  launcherForm.addEventListener('submit', handleStartJob);
  headerStopBtn.addEventListener('click', handleStopJob);
  document.getElementById('refresh-datasets-btn').addEventListener('click', () => {
    fetchDatasets();
    showToast('Refreshing datasets...', 'info');
  });
  document.getElementById('refresh-runs-btn').addEventListener('click', () => {
    fetchRuns();
    showToast('Refreshing past runs...', 'info');
  });

  clearLogsBtn.addEventListener('click', () => {
    logConsole.innerHTML = '';
    totalLogLines = 0;
    updateLogCount();
    showToast('Console cleared', 'info');
  });

  autoScrollToggle.addEventListener('change', (e) => {
    autoScroll = e.target.checked;
  });
});

// Update Mode Badge & Visual Status
function setViewerMode(mode, runName = null) {
  viewerMode = mode;
  currentRunName = runName;

  if (mode === 'live') {
    viewerModeBadge.textContent = '🔴 LIVE SLAM';
    viewerModeBadge.className = 'px-2 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider bg-rose-500/20 text-rose-400 border border-rose-500/40 select-none';
    viewportPlaceholder.classList.add('hidden');
    switchToLiveBtn.classList.add('hidden');
  } else if (mode === 'loaded') {
    viewerModeBadge.textContent = '📁 LOADED RUN';
    viewerModeBadge.className = 'px-2 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider bg-blue-500/20 text-blue-400 border border-blue-500/40 select-none';
    viewportPlaceholder.classList.add('hidden');
    if (currentJobState === 'running' || currentJobState === 'finalizing') {
      switchToLiveBtn.classList.remove('hidden');
    } else {
      switchToLiveBtn.classList.add('hidden');
    }
  } else { // idle
    viewerModeBadge.textContent = 'IDLE';
    viewerModeBadge.className = 'px-2 py-0.5 rounded text-[10px] font-bold uppercase tracking-wider bg-slate-800 text-slate-400 border border-slate-700 select-none';
    viewportPlaceholder.classList.remove('hidden');
    if (currentJobState === 'running' || currentJobState === 'finalizing') {
      switchToLiveBtn.classList.remove('hidden');
    } else {
      switchToLiveBtn.classList.add('hidden');
    }
  }

  updateViewerControlsState();
}

// Grey out and disable controls when not relevant
function updateViewerControlsState() {
  const hasSubmaps = currentRunSubmaps && currentRunSubmaps.length > 0;
  const count = hasSubmaps ? currentRunSubmaps.length : 0;
  const isLoaded = Boolean(currentRunName);

  // 1. Isolate Toggle
  viewerIsolateToggle.disabled = !hasSubmaps;
  const isolateLabel = viewerIsolateToggle.closest('label');
  if (isolateLabel) {
    isolateLabel.classList.toggle('opacity-40', !hasSubmaps);
    isolateLabel.classList.toggle('cursor-not-allowed', !hasSubmaps);
    isolateLabel.classList.toggle('pointer-events-none', !hasSubmaps);
  }

  // 1b. Play & Speed: enabled only if multiple submaps
  const canPlay = hasSubmaps && count > 1;
  viewerPlayBtn.disabled = !canPlay;
  viewerPlayBtn.classList.toggle('opacity-40', !canPlay);
  viewerPlayBtn.classList.toggle('cursor-not-allowed', !canPlay);
  viewerPlaySpeed.disabled = !canPlay;
  viewerPlaySpeed.classList.toggle('opacity-40', !canPlay);
  viewerPlaySpeed.classList.toggle('cursor-not-allowed', !canPlay);
  if (!canPlay && isPlayingSubmaps) {
    stopSubmapPlayback();
  }

  // 2. Previous (<) button: enabled only if activeSubmapId > 0
  const canPrev = hasSubmaps && activeSubmapId !== null && activeSubmapId > 0;
  viewerPrevBtn.disabled = !canPrev;
  viewerPrevBtn.classList.toggle('opacity-40', !canPrev);
  viewerPrevBtn.classList.toggle('cursor-not-allowed', !canPrev);

  // 3. Next (>) button: enabled only if activeSubmapId < count - 1
  const canNext = hasSubmaps && activeSubmapId !== null && activeSubmapId < count - 1;
  viewerNextBtn.disabled = !canNext;
  viewerNextBtn.classList.toggle('opacity-40', !canNext);
  viewerNextBtn.classList.toggle('cursor-not-allowed', !canNext);

  // 4. Slider: enabled only if multiple submaps
  const canSlide = hasSubmaps && count > 1;
  viewerSubmapSlider.disabled = !canSlide;
  viewerSubmapSlider.classList.toggle('opacity-40', !canSlide);
  viewerSubmapSlider.classList.toggle('cursor-not-allowed', !canSlide);
  if (hasSubmaps) {
    viewerSubmapSlider.max = Math.max(0, count - 1);
    viewerSubmapSlider.value = activeSubmapId !== null ? activeSubmapId : 0;
  } else {
    viewerSubmapSlider.max = 0;
    viewerSubmapSlider.value = 0;
  }

  // 5. Focus Button
  const canFocus = hasSubmaps && activeSubmapId !== null;
  viewerFocusBtn.disabled = !canFocus;
  viewerFocusBtn.classList.toggle('opacity-40', !canFocus);
  viewerFocusBtn.classList.toggle('cursor-not-allowed', !canFocus);

  // 6. Fit & Top-Down Camera buttons
  const canFit = isLoaded && (hasSubmaps || threeTrajectoryLine !== null);
  viewerFitBtn.disabled = !canFit;
  viewerFitBtn.classList.toggle('opacity-40', !canFit);
  viewerFitBtn.classList.toggle('cursor-not-allowed', !canFit);

  viewerTopdownBtn.disabled = !canFit;
  viewerTopdownBtn.classList.toggle('opacity-40', !canFit);
  viewerTopdownBtn.classList.toggle('cursor-not-allowed', !canFit);

  // 7. Color Mode & Point Size
  viewerColorMode.disabled = !hasSubmaps;
  viewerColorMode.classList.toggle('opacity-40', !hasSubmaps);
  viewerColorMode.classList.toggle('cursor-not-allowed', !hasSubmaps);

  viewerPointSize.disabled = !hasSubmaps;
  viewerPointSize.classList.toggle('opacity-40', !hasSubmaps);
  viewerPointSize.classList.toggle('cursor-not-allowed', !hasSubmaps);

  // 8. Trajectory Toggle
  const hasTraj = threeTrajectoryLine !== null;
  viewerTrajToggle.disabled = !hasTraj;
  const trajLabel = viewerTrajToggle.closest('label');
  if (trajLabel) {
    trajLabel.classList.toggle('opacity-40', !hasTraj);
    trajLabel.classList.toggle('cursor-not-allowed', !hasTraj);
    trajLabel.classList.toggle('pointer-events-none', !hasTraj);
  }

  // 9. Loops Toggle
  const hasLoops = currentRunLoops && currentRunLoops.length > 0;
  viewerLoopsToggle.disabled = !hasLoops;
  const loopsLabel = viewerLoopsToggle.closest('label');
  if (loopsLabel) {
    loopsLabel.classList.toggle('opacity-40', !hasLoops);
    loopsLabel.classList.toggle('cursor-not-allowed', !hasLoops);
    loopsLabel.classList.toggle('pointer-events-none', !hasLoops);
  }

  // 10. Clear Button
  viewerClearBtn.disabled = (viewerMode === 'idle');
  viewerClearBtn.classList.toggle('opacity-40', viewerMode === 'idle');
  viewerClearBtn.classList.toggle('cursor-not-allowed', viewerMode === 'idle');
}

// Update Active Button Visual Styles
function updateModalButtonStates() {
  const launcherOpen = !launcherModal.classList.contains('hidden');
  const runsOpen = !runsModal.classList.contains('hidden');
  const consoleOpen = !consoleDrawer.classList.contains('hidden');

  if (launcherOpen) {
    toggleLauncherBtn.classList.add('ring-2', 'ring-blue-400', 'bg-blue-700');
  } else {
    toggleLauncherBtn.classList.remove('ring-2', 'ring-blue-400', 'bg-blue-700');
  }

  if (runsOpen) {
    toggleRunsBtn.classList.add('ring-2', 'ring-purple-400', 'bg-slate-700');
  } else {
    toggleRunsBtn.classList.remove('ring-2', 'ring-purple-400', 'bg-slate-700');
  }

  if (consoleOpen) {
    toggleConsoleBtn.classList.add('ring-2', 'ring-amber-400', 'bg-slate-700');
  } else {
    toggleConsoleBtn.classList.remove('ring-2', 'ring-amber-400', 'bg-slate-700');
  }
}

// Setup Floating Modal Visibility & Toggles
function setupModalToggles() {
  toggleLauncherBtn.addEventListener('click', () => {
    const isHidden = launcherModal.classList.contains('hidden');
    if (isHidden) {
      runsModal.classList.add('hidden');
      launcherModal.classList.remove('hidden');
    } else {
      launcherModal.classList.add('hidden');
    }
    updateModalButtonStates();
  });

  closeLauncherBtn.addEventListener('click', () => {
    launcherModal.classList.add('hidden');
    updateModalButtonStates();
  });

  toggleRunsBtn.addEventListener('click', () => {
    const isHidden = runsModal.classList.contains('hidden');
    if (isHidden) {
      launcherModal.classList.add('hidden');
      runsModal.classList.remove('hidden');
      fetchRuns();
    } else {
      runsModal.classList.add('hidden');
    }
    updateModalButtonStates();
  });

  closeRunsBtn.addEventListener('click', () => {
    runsModal.classList.add('hidden');
    updateModalButtonStates();
  });

  toggleConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.toggle('hidden');
    minConsoleBtn.classList.toggle('hidden', !consoleDrawer.classList.contains('hidden'));
    updateModalButtonStates();
  });

  if (expandConsoleBtn) {
    expandConsoleBtn.addEventListener('click', () => {
      isConsoleWide = !isConsoleWide;
      consoleDrawer.classList.toggle('console-wide', isConsoleWide);
      consoleDrawer.classList.toggle('console-standard', !isConsoleWide);
      expandConsoleBtn.innerHTML = isConsoleWide
        ? '<i data-lucide="minimize-2" class="w-3.5 h-3.5"></i>'
        : '<i data-lucide="maximize-2" class="w-3.5 h-3.5"></i>';
      expandConsoleBtn.title = isConsoleWide ? 'Switch to standard (80 cols) width' : 'Switch to wide (120 cols) width';
      if (window.lucide) lucide.createIcons();
      showToast(isConsoleWide ? 'Console: Wide (120 cols)' : 'Console: Standard (80 cols)', 'info');
    });
  }

  closeConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.add('hidden');
    minConsoleBtn.classList.remove('hidden');
    updateModalButtonStates();
  });

  minConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.remove('hidden');
    minConsoleBtn.classList.add('hidden');
    updateModalButtonStates();
  });

  // Close modals when clicking on background 3D viewport canvas
  viewportContainer.addEventListener('click', (e) => {
    if (e.target === viewportContainer || e.target.tagName === 'CANVAS') {
      let changed = false;
      if (!launcherModal.classList.contains('hidden')) {
        launcherModal.classList.add('hidden');
        changed = true;
      }
      if (!runsModal.classList.contains('hidden')) {
        runsModal.classList.add('hidden');
        changed = true;
      }
      if (changed) updateModalButtonStates();
    }
  });

  updateModalButtonStates();
}

// WebSocket Telemetry Connection
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
    wsText.textContent = 'Offline';
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = () => {
    ws.close();
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'status') {
        updateJobStatus(msg.data);
      } else if (msg.type === 'log') {
        appendLog(msg.line);
        if (msg.progress) {
          updateJobStatus(msg.progress);
        }
      }
    } catch (e) {
      console.error('WS message error:', e);
    }
  };
}

// Update Job Status & Top KPI Strip
function updateJobStatus(job) {
  currentJobState = job.state;
  statusText.textContent = job.state.toUpperCase();

  if (job.run_name) {
    activeLiveRunName = job.run_name;
  }

  statusDot.className = 'h-2 w-2 rounded-full';

  switch (job.state) {
    case 'running':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-amber-500/10 border-amber-500/30 text-amber-400';
      statusDot.classList.add('bg-amber-400', 'status-running');
      headerStopBtn.classList.remove('hidden');
      startBtn.disabled = true;

      // If we are not currently viewing a loaded past run, track the live run automatically!
      if (viewerMode !== 'loaded' && job.run_name && (viewerMode !== 'live' || currentRunName !== job.run_name)) {
        startLiveTracking(job.run_name);
      } else if (viewerMode === 'loaded') {
        // Show the switch-to-live pill so the user can easily hop back to live tracking
        switchToLiveBtn.classList.remove('hidden');
      }
      break;

    case 'finalizing':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-purple-500/10 border-purple-500/30 text-purple-400';
      statusDot.classList.add('bg-purple-400', 'status-running');
      headerStopBtn.classList.remove('hidden');
      break;

    case 'completed':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-emerald-500/10 border-emerald-500/30 text-emerald-400';
      statusDot.classList.add('bg-emerald-400');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      switchToLiveBtn.classList.add('hidden');
      if (viewerMode === 'live') {
        stopLiveTracking(true);
      }
      fetchRuns();
      showToast('SLAM run completed successfully!', 'success');
      break;

    case 'stopped':
    case 'failed':
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-rose-500/10 border-rose-500/30 text-rose-400';
      statusDot.classList.add('bg-rose-400');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      switchToLiveBtn.classList.add('hidden');
      if (viewerMode === 'live') {
        stopLiveTracking(false);
      }
      fetchRuns();
      showToast(`SLAM job ended: ${job.state}`, 'warn');
      break;

    default: // idle
      statusBadge.className = 'flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-medium border bg-slate-900 border-slate-800 text-slate-300';
      statusDot.classList.add('bg-slate-500');
      headerStopBtn.classList.add('hidden');
      startBtn.disabled = false;
      startBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i> Start SLAM Run';
      if (window.lucide) lucide.createIcons();
      switchToLiveBtn.classList.add('hidden');
      break;
  }

  metricSpeed.textContent = job.speed ? `${job.speed.toFixed(1)}x` : '0.0x';
  metricBagTime.textContent = job.bag_time ? `${job.bag_time.toFixed(1)}s` : '0.0s';
  metricScans.textContent = job.scans || '0';
  metricQueueOdom.textContent = job.queue_odom || '0';
  metricQueueSub.textContent = job.queue_sub || '0';
  metricQueueGlob.textContent = job.queue_glob || '0';
}

// Append Terminal Log
function appendLog(line) {
  const div = document.createElement('div');
  div.className = 'log-line';

  if (line.includes('[error]') || line.includes('critical') || line.includes('failed')) {
    div.classList.add('log-err');
  } else if (line.includes('[warning]')) {
    div.classList.add('log-warn');
  } else if (line.includes('[progress]')) {
    div.classList.add('log-progress');
  } else {
    div.classList.add('log-info');
  }

  div.textContent = line;
  logConsole.appendChild(div);
  totalLogLines++;
  updateLogCount();

  if (autoScroll) {
    logConsole.scrollTop = logConsole.scrollHeight;
  }
}

function updateLogCount() {
  logCount.textContent = `${totalLogLines} lines`;
}

// Fetch Available Datasets
async function fetchDatasets() {
  try {
    const res = await fetch('/api/datasets');
    const datasets = await res.json();
    datasetSelect.innerHTML = '';

    if (datasets.length === 0) {
      datasetSelect.innerHTML = '<option value="">No MCAP datasets found in /data</option>';
      return;
    }

    datasets.forEach(d => {
      const opt = document.createElement('option');
      opt.value = d.path;
      opt.textContent = `${d.name} (${d.size_human})`;
      datasetSelect.appendChild(opt);
    });
  } catch (err) {
    datasetSelect.innerHTML = '<option value="">Error scanning datasets</option>';
  }
}

// Fetch Past Runs History
async function fetchRuns() {
  try {
    const res = await fetch('/api/runs');
    const runs = await res.json();
    runsList.innerHTML = '';

    const prevSelectVal = viewerRunSelect.value;
    viewerRunSelect.innerHTML = '<option value="">Select a run...</option>';

    if (runs.length === 0) {
      runsList.innerHTML = '<div class="text-xs text-slate-500 py-4 text-center">No past runs found in /data/glim_results</div>';
      return;
    }

    runs.forEach(run => {
      const opt = document.createElement('option');
      opt.value = run.name;
      opt.textContent = `${run.name} (${run.submaps_count} submaps)`;
      viewerRunSelect.appendChild(opt);

      const item = document.createElement('div');
      item.className = 'bg-slate-950/70 border border-slate-800/80 rounded-lg p-2.5 hover:border-slate-700 transition flex items-center justify-between gap-3 text-xs';
      item.innerHTML = `
        <div class="min-w-0">
          <div class="font-semibold text-slate-200 truncate">${run.name}</div>
          <div class="text-[11px] text-slate-400 flex items-center gap-1.5 mt-0.5">
            <span>${run.modified_at}</span>
            <span>•</span>
            <span class="text-blue-400 font-medium">${run.submaps_count} submaps</span>
            <span>•</span>
            <span>${run.size_human}</span>
          </div>
        </div>
        <div class="flex items-center gap-1.5 shrink-0">
          <button onclick="openRunIn3D('${run.name}')" class="px-2.5 py-1 bg-blue-600 hover:bg-blue-500 active:scale-95 text-white rounded flex items-center gap-1 transition text-[11px] font-medium shadow-sm" title="View in 3D Viewport">
            <i data-lucide="box" class="w-3.5 h-3.5"></i>
            <span>3D</span>
          </button>
          ${run.has_traj_lidar ? `
            <a href="/api/runs/${encodeURIComponent(run.name)}/trajectory?opt=true" download="${run.name}_traj_lidar.txt" class="p-1.5 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded flex items-center transition" title="Download Optimized LiDAR Trajectory (TUM)">
              <i data-lucide="download" class="w-3.5 h-3.5"></i>
            </a>` : ''}
        </div>
      `;
      runsList.appendChild(item);
    });

    if (window.lucide) lucide.createIcons();

    if (prevSelectVal && Array.from(viewerRunSelect.options).some(o => o.value === prevSelectVal)) {
      viewerRunSelect.value = prevSelectVal;
    }
  } catch (err) {
    runsList.innerHTML = '<div class="text-xs text-rose-500 py-4 text-center">Failed to load run history</div>';
  }
}

// Open run directly in 3D
window.openRunIn3D = function(runName) {
  runsModal.classList.add('hidden');
  updateModalButtonStates();
  viewerRunSelect.value = runName;
  loadMapRun(runName);
  showToast(`Loading 3D map: ${runName}...`, 'info');
};

// Start SLAM Job
async function handleStartJob(e) {
  e.preventDefault();
  const datasetPath = datasetSelect.value;
  if (!datasetPath) {
    alert('Please select an input MCAP dataset.');
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
  showToast('Starting SLAM run...', 'info');

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

    launcherModal.classList.add('hidden');
    consoleDrawer.classList.remove('hidden');
    minConsoleBtn.classList.add('hidden');
    updateModalButtonStates();

    // Clear interface completely before starting live tracking of new run
    clear3DScene();
    updateJobStatus(job);
    startLiveTracking(job.run_name);
    showToast(`SLAM Job launched: ${job.run_name}`, 'success');
  } catch (err) {
    alert(`Could not start job: ${err.message}`);
    startBtn.disabled = false;
    startBtn.textContent = 'Start SLAM Run';
    showToast(`Failed: ${err.message}`, 'error');
  }
}

// Stop SLAM Job
async function handleStopJob() {
  if (!confirm('Are you sure you want to stop and finalize the current SLAM run?')) return;
  headerStopBtn.disabled = true;
  headerStopBtn.textContent = 'Finalizing...';
  showToast('Finalizing run and generating loop-closed trajectory...', 'info');

  try {
    const res = await fetch('/api/jobs/stop', { method: 'POST' });
    const job = await res.json();
    updateJobStatus(job);
  } catch (err) {
    alert(`Error stopping job: ${err.message}`);
  } finally {
    headerStopBtn.disabled = false;
  }
}

// ============================================================================
// REAL-TIME 3D VIEWPORT & SUBMAP STREAMING (Three.js)
// ============================================================================

function initThreeJS() {
  const container = viewportContainer;
  const width = window.innerWidth;
  const height = window.innerHeight;

  // Scene
  threeScene = new THREE.Scene();
  threeScene.background = new THREE.Color(0x020617);

  // Camera (Z-Up for SLAM / Robotics convention)
  threeCamera = new THREE.PerspectiveCamera(55, width / height, 0.1, 5000);
  threeCamera.up.set(0, 0, 1);
  threeCamera.position.set(-25, -25, 20);

  // Renderer
  threeRenderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  threeRenderer.setSize(width, height);
  threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(threeRenderer.domElement);

  // OrbitControls
  threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
  threeControls.enableDamping = true;
  threeControls.dampingFactor = 0.08;
  threeControls.screenSpacePanning = true;
  threeControls.target.set(0, 0, 0);

  // Grid Helper (aligned with Z-up)
  const grid = new THREE.GridHelper(200, 40, 0x1e293b, 0x0f172a);
  grid.rotation.x = Math.PI / 2;
  threeScene.add(grid);

  // Axes Helper (X: Red, Y: Green, Z: Blue)
  const axes = new THREE.AxesHelper(3.0);
  threeScene.add(axes);

  // Active Submap 3D Marker
  createActiveSubmapMarker();

  // Window Resize Listener
  window.addEventListener('resize', onWindowResize);

  // Animation Loop
  function animate() {
    requestAnimationFrame(animate);
    threeControls.update();
    threeRenderer.render(threeScene, threeCamera);
  }
  animate();
}

function onWindowResize() {
  if (!threeCamera || !threeRenderer) return;
  const w = window.innerWidth;
  const h = window.innerHeight;
  threeCamera.aspect = w / h;
  threeCamera.updateProjectionMatrix();
  threeRenderer.setSize(w, h);
}

// Create 3D Active Submap Visual Indicator (Box + Axes + Ring)
function createActiveSubmapMarker() {
  if (activeSubmapMarker) return;
  const group = new THREE.Group();

  // Wireframe bounding cube
  const boxGeom = new THREE.BoxGeometry(5.0, 5.0, 3.5);
  const boxMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2, transparent: true, opacity: 0.9 });
  const wireframe = new THREE.LineSegments(new THREE.WireframeGeometry(boxGeom), boxMat);
  group.add(wireframe);

  // Submap local coordinate frame axes (X: Red, Y: Green, Z: Blue)
  const axes = new THREE.AxesHelper(3.5);
  group.add(axes);

  // Origin point dot
  const dotGeom = new THREE.SphereGeometry(0.35, 16, 16);
  const dotMat = new THREE.MeshBasicMaterial({ color: 0x60a5fa });
  const dot = new THREE.Mesh(dotGeom, dotMat);
  group.add(dot);

  group.visible = false;
  threeScene.add(group);
  activeSubmapMarker = group;
}

function updateActiveSubmapMarker(submapId) {
  if (!activeSubmapMarker) createActiveSubmapMarker();
  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (!sm) {
    activeSubmapMarker.visible = false;
    return;
  }

  activeSubmapMarker.position.set(sm.pos[0], sm.pos[1], sm.pos[2]);
  if (sm.matrix) {
    const mat4 = new THREE.Matrix4();
    mat4.fromArray(sm.matrix);
    activeSubmapMarker.setRotationFromMatrix(mat4);
  }
  activeSubmapMarker.visible = true;
}

// 3D HUD Controls Setup
function setup3DViewerControls() {
  // Auto-load immediately when a run is selected from dropdown
  viewerRunSelect.addEventListener('change', (e) => {
    const runName = e.target.value;
    if (runName) {
      loadMapRun(runName);
    } else {
      clear3DScene();
      setViewerMode('idle');
    }
  });

  viewerLoadBtn.addEventListener('click', () => {
    const runName = viewerRunSelect.value;
    if (runName) {
      loadMapRun(runName);
    } else {
      showToast('Select a run from the dropdown first', 'warn');
    }
  });

  viewerClearBtn.addEventListener('click', () => {
    stopLiveTracking(false);
    clear3DScene();
    setViewerMode('idle');
    viewerRunSelect.value = '';
    showToast('Viewer cleared', 'info');
  });

  switchToLiveBtn.addEventListener('click', () => {
    if (activeLiveRunName) {
      clear3DScene();
      startLiveTracking(activeLiveRunName);
      showToast(`Viewing active live SLAM run: ${activeLiveRunName}`, 'info');
    }
  });

  viewerIsolateToggle.addEventListener('change', (e) => {
    isIsolated = e.target.checked;
    updateSubmapVisibility();
    showToast(isIsolated ? 'Isolate mode ON (Single submap)' : 'Showing all submaps', 'info');
  });

  viewerPlayBtn.addEventListener('click', toggleSubmapPlayback);

  viewerPlaySpeed.addEventListener('change', () => {
    if (isPlayingSubmaps) {
      const interval = parseInt(viewerPlaySpeed.value, 10) || 200;
      scheduleNextSubmapPlay(interval);
      showToast(`Playback speed: ${viewerPlaySpeed.options[viewerPlaySpeed.selectedIndex].text}`, 'info');
    }
  });

  viewerSubmapSlider.addEventListener('input', (e) => {
    stopSubmapPlayback();
    activeSubmapId = parseInt(e.target.value, 10);
    onActiveSubmapChanged(activeSubmapId);
  });

  viewerPrevBtn.addEventListener('click', () => {
    stopSubmapPlayback();
    if (currentRunSubmaps.length === 0) return;
    if (activeSubmapId === null) activeSubmapId = 0;
    if (activeSubmapId > 0) {
      activeSubmapId--;
      viewerSubmapSlider.value = activeSubmapId;
      onActiveSubmapChanged(activeSubmapId);
    }
  });

  viewerNextBtn.addEventListener('click', () => {
    stopSubmapPlayback();
    if (currentRunSubmaps.length === 0) return;
    if (activeSubmapId === null) activeSubmapId = -1;
    if (activeSubmapId < currentRunSubmaps.length - 1) {
      activeSubmapId++;
      viewerSubmapSlider.value = activeSubmapId;
      onActiveSubmapChanged(activeSubmapId);
    }
  });

  viewerFocusBtn.addEventListener('click', () => {
    if (activeSubmapId !== null) {
      focusCameraOnSubmap(activeSubmapId);
      showToast(`Focused on Submap #${String(activeSubmapId).padStart(4, '0')}`, 'info');
    }
  });

  viewerFitBtn.addEventListener('click', fitAllInView);
  viewerTopdownBtn.addEventListener('click', setTopDownView);

  viewerColorMode.addEventListener('change', (e) => {
    colorMode = e.target.value;
    submapCache.forEach((points, id) => {
      applySubmapColors(points.geometry, id);
    });
    showToast(`Color mode: ${colorMode === 'rainbow' ? 'Rainbow (Z-altitude)' : 'Flat White'}`, 'info');
  });

  viewerPointSize.addEventListener('input', (e) => {
    pointSize = parseFloat(e.target.value);
    submapCache.forEach((points, id) => {
      points.material.size = (id === activeSubmapId && !isIsolated) ? pointSize * 1.5 : pointSize;
      points.material.needsUpdate = true;
    });
  });

  viewerTrajToggle.addEventListener('change', (e) => {
    isTrajVisible = e.target.checked;
    if (threeTrajectoryLine) {
      threeTrajectoryLine.visible = isTrajVisible;
    }
    showToast(`Trajectory line: ${isTrajVisible ? 'Visible' : 'Hidden'}`, 'info');
  });

  viewerLoopsToggle.addEventListener('change', (e) => {
    isLoopsVisible = e.target.checked;
    updateLoopLines();
    showToast(`Loop closures: ${isLoopsVisible ? 'Visible' : 'Hidden'}`, 'info');
  });

  // Global Keyboard Shortcuts
  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
      if (e.key === 'Escape') document.activeElement.blur();
      return;
    }

    if (e.key === 'Escape') {
      launcherModal.classList.add('hidden');
      runsModal.classList.add('hidden');
      updateModalButtonStates();
    } else if (e.key === ' ' || e.key === 'p' || e.key === 'P') {
      e.preventDefault();
      if (!viewerPlayBtn.disabled) {
        toggleSubmapPlayback();
      }
    } else if (e.key === 'ArrowLeft') {
      e.preventDefault();
      if (!viewerPrevBtn.disabled) viewerPrevBtn.click();
    } else if (e.key === 'ArrowRight') {
      e.preventDefault();
      if (!viewerNextBtn.disabled) viewerNextBtn.click();
    } else if (e.key === 'f' || e.key === 'F') {
      if (!viewerFocusBtn.disabled) viewerFocusBtn.click();
    } else if (e.key === 'r' || e.key === 'R') {
      if (!viewerFitBtn.disabled) viewerFitBtn.click();
    } else if (e.key === 't' || e.key === 'T') {
      if (!viewerTopdownBtn.disabled) viewerTopdownBtn.click();
    } else if (e.key === 'i' || e.key === 'I') {
      if (!viewerIsolateToggle.disabled) {
        viewerIsolateToggle.checked = !viewerIsolateToggle.checked;
        viewerIsolateToggle.dispatchEvent(new Event('change'));
      }
    } else if (e.key === 'o' || e.key === 'O') {
      if (!viewerTrajToggle.disabled) {
        viewerTrajToggle.checked = !viewerTrajToggle.checked;
        viewerTrajToggle.dispatchEvent(new Event('change'));
      }
    } else if (e.key === 'l' || e.key === 'L') {
      if (!viewerLoopsToggle.disabled) {
        viewerLoopsToggle.checked = !viewerLoopsToggle.checked;
        viewerLoopsToggle.dispatchEvent(new Event('change'));
      }
    }
  });
}

// Fit Entire Point Cloud & Trajectory in Frustum
function fitAllInView() {
  const box = new THREE.Box3();
  if (threeTrajectoryLine && threeTrajectoryLine.geometry && threeTrajectoryLine.geometry.attributes.position) {
    threeTrajectoryLine.geometry.computeBoundingBox();
    if (threeTrajectoryLine.geometry.boundingBox) {
      box.union(threeTrajectoryLine.geometry.boundingBox);
    }
  }
  submapCache.forEach((points) => {
    if (points.geometry && points.geometry.attributes.position) {
      points.geometry.computeBoundingBox();
      if (points.geometry.boundingBox) {
        box.union(points.geometry.boundingBox);
      }
    }
  });

  if (box.isEmpty()) return;

  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z, 25.0);
  const fov = threeCamera.fov * (Math.PI / 180);
  const cameraDist = Math.abs(maxDim / 2 / Math.tan(fov / 2)) * 1.25;

  threeControls.target.copy(center);
  threeCamera.position.set(center.x - cameraDist * 0.6, center.y - cameraDist * 0.6, center.z + cameraDist * 0.7);
  threeCamera.up.set(0, 0, 1);
  threeControls.update();
  showToast('Fit view to full map', 'info');
}

// Top-Down Bird's Eye View (Snaps to 2D Plan View)
function setTopDownView() {
  const box = new THREE.Box3();
  if (threeTrajectoryLine && threeTrajectoryLine.geometry && threeTrajectoryLine.geometry.attributes.position) {
    threeTrajectoryLine.geometry.computeBoundingBox();
    if (threeTrajectoryLine.geometry.boundingBox) {
      box.union(threeTrajectoryLine.geometry.boundingBox);
    }
  }
  submapCache.forEach((points) => {
    if (points.geometry && points.geometry.attributes.position) {
      points.geometry.computeBoundingBox();
      if (points.geometry.boundingBox) {
        box.union(points.geometry.boundingBox);
      }
    }
  });

  const center = box.isEmpty() ? new THREE.Vector3(0, 0, 0) : box.getCenter(new THREE.Vector3());
  const size = box.isEmpty() ? new THREE.Vector3(50, 50, 20) : box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, 35.0);
  const fov = threeCamera.fov * (Math.PI / 180);
  const cameraDist = Math.abs(maxDim / 2 / Math.tan(fov / 2)) * 1.25;

  threeControls.target.copy(center);
  threeCamera.position.set(center.x, center.y - 0.01, center.z + cameraDist);
  threeCamera.up.set(0, 1, 0); // Y-forward
  threeControls.update();
  showToast("Top-down bird's-eye view", 'info');
}

// Clear Scene Completely Between Runs
function clear3DScene() {
  stopSubmapPlayback();
  loadSessionId++; // Invalidate any running background submap streams or polls
  if (liveStreamTimer) {
    clearInterval(liveStreamTimer);
    liveStreamTimer = null;
  }

  submapCache.forEach((points) => {
    threeScene.remove(points);
    if (points.geometry) points.geometry.dispose();
    if (points.material) points.material.dispose();
  });
  submapCache.clear();

  if (threeTrajectoryLine) {
    threeScene.remove(threeTrajectoryLine);
    if (threeTrajectoryLine.geometry) threeTrajectoryLine.geometry.dispose();
    if (threeTrajectoryLine.material) threeTrajectoryLine.material.dispose();
    threeTrajectoryLine = null;
  }

  if (threeLoopLines) {
    threeScene.remove(threeLoopLines);
    if (threeLoopLines.geometry) threeLoopLines.geometry.dispose();
    if (threeLoopLines.material) threeLoopLines.material.dispose();
    threeLoopLines = null;
  }
  currentRunLoops = [];
  if (hudLoopsCount) {
    hudLoopsCount.classList.add('hidden');
    hudLoopsCount.textContent = '0';
  }

  if (activeSubmapMarker) {
    activeSubmapMarker.visible = false;
  }

  currentRunSubmaps = [];
  activeSubmapId = null;
  viewerSubmapSlider.max = 0;
  viewerSubmapSlider.value = 0;
  viewerHud.classList.add('hidden');
  updateViewerControlsState();
}

// Load a Completed or Past Run
async function loadMapRun(runName) {
  if (!runName) return;

  // Stop any active live tracking before loading past run!
  stopLiveTracking(false);
  clear3DScene();
  setViewerMode('loaded', runName);

  const thisSessionId = loadSessionId;
  viewerLoadBtn.disabled = true;
  viewerLoadBtn.innerHTML = '<i data-lucide="rotate-cw" class="w-3 h-3 animate-spin"></i><span>Loading...</span>';
  if (window.lucide) lucide.createIcons();

  showToast(`Loading 3D Map: ${runName}...`, 'info');

  try {
    // 1. Fetch Trajectory Binary
    await loadTrajectory(runName, true);
    if (thisSessionId !== loadSessionId) return;

    // 2. Fetch Submaps Metadata
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
    if (!res.ok) throw new Error('Failed to load submap list');
    const submaps = await res.json();
    if (thisSessionId !== loadSessionId) return;

    currentRunSubmaps = submaps;

    // Load Loop Closures
    await loadLoopClosures(runName);
    if (thisSessionId !== loadSessionId) return;

    if (currentRunSubmaps.length === 0) {
      hudSubmapText.textContent = 'No submaps recorded';
      viewerHud.classList.remove('hidden');
      updateViewerControlsState();
      showToast(`Run ${runName} has no submaps recorded`, 'warn');
      return;
    }

    viewerHud.classList.remove('hidden');

    // 3. Load First Batch of Submaps (first 35 for instantaneous initial view)
    const initialBatch = currentRunSubmaps.slice(0, 35);
    await Promise.all(initialBatch.map(sm => loadSubmapPoints(runName, sm.id)));
    if (thisSessionId !== loadSessionId) return;

    onActiveSubmapChanged(0);
    fitAllInView();

    showToast(`Loaded ${currentRunSubmaps.length} submaps (${runName})`, 'success');

    // Stream remaining submaps in background smoothly
    if (currentRunSubmaps.length > 35) {
      streamRemainingSubmaps(thisSessionId, runName, 35);
    }
  } catch (err) {
    console.error('Failed to load map run:', err);
    showToast(`Could not load map: ${err.message}`, 'error');
  } finally {
    viewerLoadBtn.disabled = false;
    viewerLoadBtn.innerHTML = '<i data-lucide="rotate-cw" class="w-3 h-3"></i><span>Load</span>';
    if (window.lucide) lucide.createIcons();
    updateViewerControlsState();
  }
}

// Background Submap Streaming
async function streamRemainingSubmaps(sessionId, runName, startIndex) {
  const chunkSize = 15;
  for (let i = startIndex; i < currentRunSubmaps.length; i += chunkSize) {
    if (sessionId !== loadSessionId) return; // Abort if scene was cleared or switched!
    const chunk = currentRunSubmaps.slice(i, i + chunkSize);
    await Promise.all(chunk.map(sm => loadSubmapPoints(runName, sm.id)));
    await new Promise(r => setTimeout(r, 40));
  }
}

// Load Binary Submap Point Cloud (Zero deserialization overhead)
async function loadSubmapPoints(runName, submapId) {
  if (submapCache.has(submapId)) {
    return submapCache.get(submapId);
  }

  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (!sm) return null;

  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps/${submapId}/points`);
    if (!res.ok) return null;
    const buffer = await res.arrayBuffer();

    const positions = new Float32Array(buffer);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));

    // Apply submap origin transformation matrix
    const matrix = new THREE.Matrix4();
    matrix.fromArray(sm.matrix);
    geom.applyMatrix4(matrix);

    // Apply Altitude Rainbow Colors
    applySubmapColors(geom, submapId);

    const mat = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: false, // Crisp screen-space points at any distance
      transparent: true,
      opacity: (submapId === activeSubmapId || isIsolated) ? 1.0 : 0.6
    });

    const pointsObj = new THREE.Points(geom, mat);
    threeScene.add(pointsObj);
    submapCache.set(submapId, pointsObj);

    if (isIsolated) {
      pointsObj.visible = (submapId === activeSubmapId);
    }

    return pointsObj;
  } catch (err) {
    return null;
  }
}

// Load Binary Trajectory
async function loadTrajectory(runName, opt = true) {
  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/trajectory_binary?opt=${opt}`);
    if (!res.ok) return;
    const buffer = await res.arrayBuffer();
    const positions = new Float32Array(buffer);

    if (threeTrajectoryLine) {
      threeTrajectoryLine.geometry.dispose();
      threeTrajectoryLine.geometry = new THREE.BufferGeometry();
      threeTrajectoryLine.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      threeTrajectoryLine.visible = isTrajVisible;
    } else {
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      const mat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2 });
      threeTrajectoryLine = new THREE.Line(geom, mat);
      threeTrajectoryLine.visible = isTrajVisible;
      threeScene.add(threeTrajectoryLine);
    }
  } catch (err) {
    console.warn('Trajectory fetch failed:', err);
  }
}

// Load Loop Closures from GLIM factor graph
async function loadLoopClosures(runName) {
  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/loops`);
    if (!res.ok) return;
    const data = await res.json();
    currentRunLoops = data.loops || [];

    if (hudLoopsCount) {
      if (currentRunLoops.length > 0) {
        hudLoopsCount.textContent = currentRunLoops.length.toLocaleString();
        hudLoopsCount.classList.remove('hidden');
      } else {
        hudLoopsCount.classList.add('hidden');
      }
    }

    updateLoopLines();
    updateViewerControlsState();
  } catch (err) {
    console.warn('Loop closures fetch failed:', err);
  }
}

// Render or update loop closure lines in 3D Scene
function updateLoopLines() {
  if (!threeScene) return;

  if (!isLoopsVisible || !currentRunLoops || currentRunLoops.length === 0 || !currentRunSubmaps || currentRunSubmaps.length === 0) {
    if (threeLoopLines) {
      threeLoopLines.visible = false;
    }
    return;
  }

  // Fast mapping of submap ID -> origin position [x, y, z]
  const submapPosMap = new Map();
  for (let i = 0; i < currentRunSubmaps.length; i++) {
    const sm = currentRunSubmaps[i];
    if (sm && sm.pos && sm.pos.length === 3) {
      submapPosMap.set(sm.id, sm.pos);
    }
  }

  // Filter loops: if in isolation mode, show only loops connecting to the active submap
  let activeLoops = currentRunLoops;
  if (isIsolated && activeSubmapId !== null) {
    activeLoops = currentRunLoops.filter(([id1, id2]) => id1 === activeSubmapId || id2 === activeSubmapId);
  }

  if (activeLoops.length === 0) {
    if (threeLoopLines) {
      threeLoopLines.visible = false;
    }
    return;
  }

  // Build Float32Array positions: 2 vertices per line (6 floats)
  const positions = new Float32Array(activeLoops.length * 6);
  let pIdx = 0;
  let validLines = 0;

  for (let i = 0; i < activeLoops.length; i++) {
    const [id1, id2] = activeLoops[i];
    const p1 = submapPosMap.get(id1);
    const p2 = submapPosMap.get(id2);
    if (!p1 || !p2) continue;

    positions[pIdx++] = p1[0];
    positions[pIdx++] = p1[1];
    positions[pIdx++] = p1[2];

    positions[pIdx++] = p2[0];
    positions[pIdx++] = p2[1];
    positions[pIdx++] = p2[2];
    validLines++;
  }

  if (validLines === 0) {
    if (threeLoopLines) {
      threeLoopLines.visible = false;
    }
    return;
  }

  const finalPositions = positions.subarray(0, validLines * 6);

  if (threeLoopLines) {
    threeLoopLines.geometry.dispose();
    threeLoopLines.geometry = new THREE.BufferGeometry();
    threeLoopLines.geometry.setAttribute('position', new THREE.BufferAttribute(finalPositions, 3));
    threeLoopLines.visible = isLoopsVisible;
  } else {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(finalPositions, 3));
    // Emerald green (0x10b981) for loop closure edges
    const mat = new THREE.LineBasicMaterial({
      color: 0x10b981,
      linewidth: 2,
      transparent: true,
      opacity: 0.8
    });
    threeLoopLines = new THREE.LineSegments(geom, mat);
    threeLoopLines.visible = isLoopsVisible;
    threeScene.add(threeLoopLines);
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
      colors[i * 3] = 0.92;
      colors[i * 3 + 1] = 0.92;
      colors[i * 3 + 2] = 0.92;
    } else {
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

function turboColormap(t) {
  const r = Math.sin(t * Math.PI * 1.5);
  const g = Math.sin(t * Math.PI);
  const b = Math.cos(t * Math.PI * 1.5);
  return [
    Math.max(0, Math.min(1, r * 0.8 + 0.2)),
    Math.max(0, Math.min(1, g)),
    Math.max(0, Math.min(1, b * 0.9 + 0.1))
  ];
}

// ============================================================================
// SUBMAP PLAYBACK ENGINE
// ============================================================================

function startSubmapPlayback() {
  if (!currentRunSubmaps || currentRunSubmaps.length <= 1) return;
  if (isPlayingSubmaps) return;

  // Auto-enable isolate mode if not active so user immediately sees isolated submaps
  if (!isIsolated) {
    viewerIsolateToggle.checked = true;
    isIsolated = true;
    updateSubmapVisibility();
  }

  // If at the last submap, rewind to beginning to play the full sequence
  if (activeSubmapId === null || activeSubmapId >= currentRunSubmaps.length - 1) {
    onActiveSubmapChanged(0);
  }

  isPlayingSubmaps = true;
  updatePlayButtonUI();

  const interval = parseInt(viewerPlaySpeed.value, 10) || 200;
  scheduleNextSubmapPlay(interval);
}

function stopSubmapPlayback() {
  if (!isPlayingSubmaps && !playSubmapTimer) return;
  isPlayingSubmaps = false;
  if (playSubmapTimer) {
    clearTimeout(playSubmapTimer);
    playSubmapTimer = null;
  }
  updatePlayButtonUI();
}

function toggleSubmapPlayback() {
  if (isPlayingSubmaps) {
    stopSubmapPlayback();
    showToast('Playback stopped', 'info');
  } else {
    startSubmapPlayback();
    showToast('Playing submaps forward', 'info');
  }
}

function updatePlayButtonUI() {
  if (!viewerPlayBtn) return;
  if (isPlayingSubmaps) {
    viewerPlayBtn.classList.remove('bg-slate-800', 'text-emerald-400', 'hover:bg-slate-700');
    viewerPlayBtn.classList.add('bg-amber-600', 'text-white', 'hover:bg-amber-500');
    viewerPlayBtn.title = 'Stop Submap Forwarding (Space / P)';
    viewerPlayBtn.innerHTML = '<i data-lucide="square" class="w-3 h-3 fill-current"></i><span id="viewer-play-btn-label">Stop</span>';
  } else {
    viewerPlayBtn.classList.remove('bg-amber-600', 'text-white', 'hover:bg-amber-500');
    viewerPlayBtn.classList.add('bg-slate-800', 'text-emerald-400', 'hover:bg-slate-700');
    viewerPlayBtn.title = 'Play/Stop Submap Forwarding (Space / P)';
    viewerPlayBtn.innerHTML = '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i><span id="viewer-play-btn-label">Play</span>';
  }
  if (window.lucide) lucide.createIcons();
}

function scheduleNextSubmapPlay(interval) {
  if (playSubmapTimer) clearTimeout(playSubmapTimer);
  playSubmapTimer = setTimeout(() => {
    if (!isPlayingSubmaps) return;
    if (!currentRunSubmaps || currentRunSubmaps.length === 0) {
      stopSubmapPlayback();
      return;
    }

    let nextId = (activeSubmapId === null) ? 0 : activeSubmapId + 1;
    if (nextId >= currentRunSubmaps.length) {
      stopSubmapPlayback();
      showToast('Finished submap playback', 'info');
      return;
    }

    onActiveSubmapChanged(nextId);

    // Pre-fetch upcoming submap point clouds ahead to ensure zero stutter
    for (let ahead = 1; ahead <= 4; ahead++) {
      const preId = nextId + ahead;
      if (preId < currentRunSubmaps.length && !submapCache.has(preId)) {
        loadSubmapPoints(currentRunName, preId);
      }
    }

    const currentInterval = parseInt(viewerPlaySpeed.value, 10) || 200;
    scheduleNextSubmapPlay(currentInterval);
  }, interval);
}

// Active Submap Selection Changed
function onActiveSubmapChanged(submapId) {
  activeSubmapId = submapId;
  if (viewerSubmapSlider) viewerSubmapSlider.value = submapId;
  const sm = currentRunSubmaps.find(s => s.id === submapId);
  const smLoops = currentRunLoops.filter(([id1, id2]) => id1 === submapId || id2 === submapId).length;
  const loopInfo = smLoops > 0 ? ` • ${smLoops} loops` : '';
  if (sm) {
    hudSubmapText.textContent = `Submap #${String(submapId).padStart(4, '0')} (${sm.num_points.toLocaleString()} pts)${loopInfo}`;
  } else {
    hudSubmapText.textContent = `Submap #${submapId}${loopInfo}`;
  }
  updateSubmapVisibility();
  loadSubmapPoints(currentRunName, submapId);
  updateViewerControlsState();
}

// Update Submap Visibility for Isolation Mode & Visual Pop
function updateSubmapVisibility() {
  submapCache.forEach((points, id) => {
    if (isIsolated) {
      points.visible = (id === activeSubmapId);
      points.material.opacity = 1.0;
      points.material.size = pointSize * 1.2;
    } else {
      points.visible = true;
      if (activeSubmapId !== null && id === activeSubmapId) {
        points.material.opacity = 1.0;
        points.material.size = pointSize * 1.5;
      } else {
        points.material.opacity = 0.55;
        points.material.size = pointSize;
      }
    }
    points.material.needsUpdate = true;
  });

  updateActiveSubmapMarker(activeSubmapId);
  updateLoopLines();
}

// Focus Camera smoothly on Submap Position
function focusCameraOnSubmap(submapId) {
  const sm = currentRunSubmaps.find(s => s.id === submapId);
  if (!sm || !threeControls) return;

  const targetPos = new THREE.Vector3(sm.pos[0], sm.pos[1], sm.pos[2]);
  threeControls.target.copy(targetPos);
  threeCamera.position.set(sm.pos[0] - 18, sm.pos[1] - 18, sm.pos[2] + 14);
  threeControls.update();
}

// ============================================================================
// STRETCH GOAL: LIVE STREAMING SUBMAPS & TRAJECTORY DURING SLAM
// ============================================================================

function startLiveTracking(runName) {
  stopLiveTracking(false);
  clear3DScene();
  setViewerMode('live', runName);

  viewerHud.classList.remove('hidden');
  let knownSubmapCount = 0;
  const thisSessionId = loadSessionId;

  async function pollLiveRun() {
    if (thisSessionId !== loadSessionId || (currentJobState !== 'running' && currentJobState !== 'finalizing')) {
      stopLiveTracking(false);
      return;
    }

    try {
      // 1. Poll Trajectory live
      await loadTrajectory(runName, false);
      if (thisSessionId !== loadSessionId) return;

      // 2. Poll Submaps list live
      const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
      if (res.ok && thisSessionId === loadSessionId) {
        const submaps = await res.json();
        currentRunSubmaps = submaps;

        if (submaps.length > knownSubmapCount) {
          // Ingest new submaps
          for (let i = knownSubmapCount; i < submaps.length; i++) {
            if (thisSessionId !== loadSessionId) return;
            await loadSubmapPoints(runName, submaps[i].id);
          }

          // Follow latest submap if user hasn't manually scrubbed
          const latestId = submaps[submaps.length - 1].id;
          if (activeSubmapId === null || knownSubmapCount === 0) {
            activeSubmapId = latestId;
            focusCameraOnSubmap(latestId);
          }
          hudSubmapText.textContent = `Live Submap #${String(latestId).padStart(4, '0')} (${submaps.length} total)`;
          knownSubmapCount = submaps.length;
          updateSubmapVisibility();
        }
        updateViewerControlsState();
      }
    } catch (e) {
      console.warn('Live tracking poll error:', e);
    }
  }

  pollLiveRun();
  liveStreamTimer = setInterval(pollLiveRun, 1500);
}

function stopLiveTracking(loadFinalOptimized = true) {
  if (liveStreamTimer) {
    clearInterval(liveStreamTimer);
    liveStreamTimer = null;
  }
  if (loadFinalOptimized && currentRunName && viewerMode === 'live') {
    setTimeout(() => {
      if (viewerMode === 'live') {
        loadMapRun(currentRunName);
      }
    }, 1000);
  }
}
