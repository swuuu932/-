/**
 * 2D Linkage Mechanism Simulator (2차원 링크 메커니즘 시뮬레이터)
 * Position-Based Dynamics (PBD) & Serial Kinematics Engine
 */

(function () {
  'use strict';

  // --- DOM Elements ---
  const canvas = document.getElementById('simCanvas');
  const ctx = canvas.getContext('2d');

  // Input Fields
  const inputN = document.getElementById('inputN');
  const inputR = document.getElementById('inputR');
  const inputA = document.getElementById('inputA');
  const inputB = document.getElementById('inputB');
  const inputSpeed = document.getElementById('inputSpeed');
  const btnApply = document.getElementById('btnApply');

  // Controls
  const btnAutoRotate = document.getElementById('btnAutoRotate');
  const autoRotateIcon = document.getElementById('autoRotateIcon');
  const autoRotateText = document.getElementById('autoRotateText');
  const btnResetPose = document.getElementById('btnResetPose');
  const btnCenterAxes = document.getElementById('btnCenterAxes');
  const btnResetView = document.getElementById('btnResetView');
  const btnToggleHelp = document.getElementById('btnToggleHelp');

  // Visualization Toggles
  const checkTrace = document.getElementById('checkTrace');
  const btnClearTrace = document.getElementById('btnClearTrace');
  const checkLimitGuides = document.getElementById('checkLimitGuides');
  const checkLabels = document.getElementById('checkLabels');

  // Dashboard & Alerts
  const dispN = document.getElementById('dispN');
  const dispR = document.getElementById('dispR');
  const dispA = document.getElementById('dispA');
  const dispB = document.getElementById('dispB');
  const statusPill = document.getElementById('statusPill');
  const statusText = document.getElementById('statusText');
  const hudAutoPauseAlert = document.getElementById('hudAutoPauseAlert');

  // --- Simulation State ---
  const state = {
    N: 4,                  // 관절/축 개수
    r: 120,                // 고정축 간격
    a: 80,                 // 굵은 막대 a 길이
    b: 130,                // 가는 막대 b 길이
    speedDeg: 45,          // 자동 회전 속도 (deg/sec)
    
    isAutoRotating: false,
    autoRotateAngle: 0,    // 1번 크랭크 각도 (rad)
    
    // Nodes
    axes: [],              // O_1 .. O_N
    cranks: [],            // P_1 .. P_N
    
    // Traces
    traces: {},            // id -> array of {x, y}
    maxTraceLength: 800,
    
    // Interaction
    draggedNode: null,     // node currently dragged
    hoveredNode: null,     // node currently hovered
    dragOffset: { x: 0, y: 0 },
    
    // Viewport transform (Pan & Zoom)
    view: {
      panX: 0,
      panY: 0,
      zoom: 1.0,
      isPanning: false,
      panStartX: 0,
      panStartY: 0
    }
  };

  // Joint Colors for distinct trace visualization
  const TRACE_COLORS = [
    '#38bdf8', '#f43f5e', '#10b981', '#f59e0b', 
    '#ec4899', '#14b8a6', '#6366f1', '#8b5cf6'
  ];

  // --- Resize & HiDPI Setup ---
  function resizeCanvas() {
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  window.addEventListener('resize', () => {
    resizeCanvas();
  });

  // --- Mechanism Initialization ---
  function initMechanism(keepBasePositions = false) {
    const { N, r, a } = state;

    // Canvas size
    const rect = canvas.parentElement.getBoundingClientRect();
    const startX = (rect.width - (N - 1) * r) / 2;
    const startY = rect.height / 2 + 50;

    // 1. Initialize Fixed Axes (O_1 .. O_N)
    const oldAxes = state.axes;
    state.axes = [];
    for (let i = 0; i < N; i++) {
      let x = startX + i * r;
      let y = startY;
      if (keepBasePositions && oldAxes[i]) {
        x = oldAxes[i].x;
        y = oldAxes[i].y;
      }
      state.axes.push({
        id: `O${i + 1}`,
        index: i,
        type: 'axis',
        x: x,
        y: y,
        radius: 10
      });
    }

    // 2. Initialize Crank Joints (P_1 .. P_N)
    const oldCranks = state.cranks;
    state.cranks = [];
    for (let i = 0; i < N; i++) {
      let angle = -Math.PI / 2 + (i * 0.25);
      if (keepBasePositions && oldCranks[i]) {
        angle = oldCranks[i].angle;
      }
      const axis = state.axes[i];
      const px = axis.x + a * Math.cos(angle);
      const py = axis.y + a * Math.sin(angle);

      state.cranks.push({
        id: `P${i + 1}`,
        index: i,
        type: 'crank',
        angle: angle,
        x: px,
        y: py,
        radius: 8,
        axisIndex: i
      });
    }

    // Solve initial constraints once to ensure exact geometry
    solveKinematics(null, 40);

    // Synchronize autoRotateAngle with crank 0
    state.autoRotateAngle = state.cranks[0].angle;

    // Update Dashboard Display
    updateDashboard();
  }

  // --- Kinematic Constraint Solver (PBD / Chain Kinematics) ---
  /**
   * Solves all distance and circular constraints iteratively.
   * If reach limits are encountered, neighbor joints are smoothly pulled along.
   * @param {Object} drivenNode - The node being dragged by mouse or rotated by motor
   * @param {number} iterations - Number of relaxation passes
   */
  function solveKinematics(drivenNode = null, iterations = 35) {
    const { N, a, b } = state;

    for (let iter = 0; iter < iterations; iter++) {
      // 1. Maintain Circular Constraints for Cranks (P_i constrained to radius a around O_i)
      for (let i = 0; i < N; i++) {
        const crank = state.cranks[i];
        const axis = state.axes[i];

        if (crank === drivenNode) {
          // Driven crank: angle is determined by drag or auto-rotation
          crank.x = axis.x + a * Math.cos(crank.angle);
          crank.y = axis.y + a * Math.sin(crank.angle);
        } else {
          // Free crank: project onto radius a from axis
          const dx = crank.x - axis.x;
          const dy = crank.y - axis.y;
          const dist = Math.hypot(dx, dy) || 1e-5;
          crank.x = axis.x + (dx / dist) * a;
          crank.y = axis.y + (dy / dist) * a;
          crank.angle = Math.atan2(crank.y - axis.y, crank.x - axis.x);
        }
      }

      // 2. Bar b Constraints: serial coupler connecting P_i and P_{i+1}
      for (let i = 0; i < N - 1; i++) {
        const p1 = state.cranks[i];
        const p2 = state.cranks[i + 1];
        relaxDistance(p1, p2, b, drivenNode, 0.5, 0.5);
      }
    }

    // Final pass: ensure strict radius constraint on cranks
    for (let i = 0; i < N; i++) {
      const crank = state.cranks[i];
      const axis = state.axes[i];
      const dx = crank.x - axis.x;
      const dy = crank.y - axis.y;
      const dist = Math.hypot(dx, dy) || 1e-5;
      crank.x = axis.x + (dx / dist) * a;
      crank.y = axis.y + (dy / dist) * a;
      crank.angle = Math.atan2(crank.y - axis.y, crank.x - axis.x);
    }
  }

  /**
   * Relaxes distance between two nodes n1 and n2 to target distance L
   */
  function relaxDistance(n1, n2, targetL, drivenNode, weight1 = 0.5, weight2 = 0.5) {
    const dx = n2.x - n1.x;
    const dy = n2.y - n1.y;
    const currentDist = Math.hypot(dx, dy) || 1e-5;
    const delta = currentDist - targetL;

    if (Math.abs(delta) < 1e-4) return;

    const nx = dx / currentDist;
    const ny = dy / currentDist;

    // Weight allocation based on whether a node is being dragged
    let w1 = weight1;
    let w2 = weight2;

    if (n1 === drivenNode) {
      w1 = 0;
      w2 = 1.0;
    } else if (n2 === drivenNode) {
      w1 = 1.0;
      w2 = 0;
    }

    n1.x += nx * (delta * w1);
    n1.y += ny * (delta * w1);
    n2.x -= nx * (delta * w2);
    n2.y -= ny * (delta * w2);
  }

  // --- Trace Management ---
  function recordTraces() {
    if (!checkTrace.checked) return;

    // Record cranks (P_1 .. P_N)
    state.cranks.forEach((crank) => {
      if (!state.traces[crank.id]) state.traces[crank.id] = [];
      const list = state.traces[crank.id];
      list.push({ x: crank.x, y: crank.y });
      if (list.length > state.maxTraceLength) list.shift();
    });
  }

  function clearTraces() {
    state.traces = {};
  }

  // --- Safety & Auto-Rotation Control ---
  function setAutoRotation(active) {
    state.isAutoRotating = active;
    if (active) {
      btnAutoRotate.classList.add('stop');
      autoRotateIcon.textContent = '■';
      autoRotateText.textContent = '자동 회전 중지';
      statusPill.classList.remove('paused');
      statusText.textContent = '1번 막대 회전 중';
    } else {
      btnAutoRotate.classList.remove('stop');
      autoRotateIcon.textContent = '▶';
      autoRotateText.textContent = '자동 회전 시작';
      statusPill.classList.add('paused');
      statusText.textContent = '정지됨';
    }
  }

  function triggerSafetyAutoPause() {
    if (state.isAutoRotating) {
      setAutoRotation(false);
      hudAutoPauseAlert.style.display = 'block';
      // Restart animation
      hudAutoPauseAlert.style.animation = 'none';
      hudAutoPauseAlert.offsetHeight; // trigger reflow
      hudAutoPauseAlert.style.animation = 'fadeInOut 2.2s ease forwards';
      setTimeout(() => {
        hudAutoPauseAlert.style.display = 'none';
      }, 2300);
    }
  }

  // --- Rendering ---
  function draw() {
    const { width, height } = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, width, height);

    ctx.save();
    // Apply View Transform (Pan & Zoom)
    ctx.translate(state.view.panX, state.view.panY);
    ctx.scale(state.view.zoom, state.view.zoom);

    // 1. Draw Subtle Axis Baseline / Grid
    drawBaseSupportLines();

    // 2. Draw Geometric Reach Limit Circles (Optional guide)
    if (checkLimitGuides.checked) {
      drawLimitGuides();
    }

    // 3. Draw Joint Traces
    if (checkTrace.checked) {
      drawTraces();
    }

    // 4. Draw Bars
    drawBars();

    // 5. Draw Fixed Axes Ground Brackets & Pivots
    drawAxes();

    // 6. Draw Crank Joint Nodes (P_1 .. P_N)
    drawJoints();

    ctx.restore();
  }

  function drawBaseSupportLines() {
    if (state.axes.length < 2) return;
    const first = state.axes[0];
    const last = state.axes[state.axes.length - 1];

    ctx.save();
    ctx.strokeStyle = 'rgba(43, 56, 78, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(first.x - 60, first.y);
    ctx.lineTo(last.x + 60, last.y);
    ctx.stroke();
    ctx.restore();
  }

  function drawLimitGuides() {
    const { a, b } = state;
    ctx.save();
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);

    // Circle of radius a around each fixed axis
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
    for (const axis of state.axes) {
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, a, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Reach circles of radius b from cranks
    ctx.strokeStyle = 'rgba(244, 63, 94, 0.15)';
    for (const crank of state.cranks) {
      ctx.beginPath();
      ctx.arc(crank.x, crank.y, b, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.restore();
  }

  function drawTraces() {
    ctx.save();
    ctx.lineWidth = 2;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    let colorIdx = 0;
    for (const [id, points] of Object.entries(state.traces)) {
      if (points.length < 2) continue;
      const color = TRACE_COLORS[colorIdx % TRACE_COLORS.length];
      colorIdx++;

      ctx.strokeStyle = color;
      ctx.beginPath();
      ctx.moveTo(points[0].x, points[0].y);
      for (let i = 1; i < points.length; i++) {
        ctx.lineTo(points[i].x, points[i].y);
      }
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawBars() {
    // --- Draw Thin Bars (b) First (behind thick bars): P_i to P_{i+1} ---
    ctx.save();
    ctx.lineWidth = 3.5;
    ctx.strokeStyle = '#f43f5e';
    ctx.lineCap = 'round';

    for (let i = 0; i < state.cranks.length - 1; i++) {
      const p1 = state.cranks[i];
      const p2 = state.cranks[i + 1];
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }
    ctx.restore();

    // --- Draw Thick Bars (a): O_i to P_i ---
    ctx.save();
    ctx.lineWidth = 9;
    ctx.lineCap = 'round';

    for (let i = 0; i < state.cranks.length; i++) {
      const axis = state.axes[i];
      const crank = state.cranks[i];

      // Outer sleek stroke
      ctx.strokeStyle = '#0284c7';
      ctx.beginPath();
      ctx.moveTo(axis.x, axis.y);
      ctx.lineTo(crank.x, crank.y);
      ctx.stroke();

      // Inner highlight line for metallic depth
      ctx.lineWidth = 4;
      ctx.strokeStyle = '#38bdf8';
      ctx.beginPath();
      ctx.moveTo(axis.x, axis.y);
      ctx.lineTo(crank.x, crank.y);
      ctx.stroke();
      ctx.lineWidth = 9;
    }
    ctx.restore();
  }

  function drawAxes() {
    for (const axis of state.axes) {
      const isHovered = state.hoveredNode === axis;
      const isDragged = state.draggedNode === axis;

      ctx.save();

      // 1. Fixed Base Support Bracket (Standard engineering hatch triangle)
      ctx.fillStyle = 'rgba(23, 31, 45, 0.9)';
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(axis.x, axis.y);
      ctx.lineTo(axis.x - 14, axis.y + 20);
      ctx.lineTo(axis.x + 14, axis.y + 20);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Ground hatching lines
      ctx.beginPath();
      for (let ox = -14; ox <= 14; ox += 7) {
        ctx.moveTo(axis.x + ox, axis.y + 20);
        ctx.lineTo(axis.x + ox - 5, axis.y + 26);
      }
      ctx.stroke();

      // 2. Central Pivot Pin
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, axis.radius, 0, Math.PI * 2);
      ctx.fill();

      // Inner anchor ring
      ctx.fillStyle = '#0f141c';
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, 4, 0, Math.PI * 2);
      ctx.fill();

      // Hover / Drag Glow
      if (isHovered || isDragged) {
        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.arc(axis.x, axis.y, axis.radius + 5, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Axis Label
      if (checkLabels.checked) {
        ctx.fillStyle = '#f59e0b';
        ctx.font = 'bold 11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(axis.id, axis.x, axis.y + 36);
      }

      ctx.restore();
    }
  }

  function drawJoints() {
    // Draw Crank Joints (P_1 .. P_N)
    for (const crank of state.cranks) {
      const isHovered = state.hoveredNode === crank;
      const isDragged = state.draggedNode === crank;

      ctx.save();
      // Outer Halo
      if (isHovered || isDragged) {
        ctx.fillStyle = 'rgba(16, 185, 129, 0.25)';
        ctx.beginPath();
        ctx.arc(crank.x, crank.y, crank.radius + 6, 0, Math.PI * 2);
        ctx.fill();
      }

      // Joint Circle
      ctx.fillStyle = '#10b981';
      ctx.strokeStyle = '#064e3b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(crank.x, crank.y, crank.radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Center Pin
      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(crank.x, crank.y, 2.5, 0, Math.PI * 2);
      ctx.fill();

      // Label
      if (checkLabels.checked) {
        ctx.fillStyle = '#6ee7b7';
        ctx.font = 'bold 11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(crank.id, crank.x, crank.y - 12);
      }

      ctx.restore();
    }
  }

  // --- Animation Loop ---
  let lastTimestamp = 0;
  function animate(timestamp) {
    if (!lastTimestamp) lastTimestamp = timestamp;
    const dt = Math.min((timestamp - lastTimestamp) / 1000, 0.1);
    lastTimestamp = timestamp;

    // Handle Auto-Rotation
    if (state.isAutoRotating && !state.draggedNode && state.cranks.length > 0) {
      const speedRad = (state.speedDeg * Math.PI) / 180;
      state.autoRotateAngle += speedRad * dt;
      // Keep in range [-PI, PI]
      state.autoRotateAngle = Math.atan2(Math.sin(state.autoRotateAngle), Math.cos(state.autoRotateAngle));

      const firstCrank = state.cranks[0];
      firstCrank.angle = state.autoRotateAngle;
      firstCrank.x = state.axes[0].x + state.a * Math.cos(state.autoRotateAngle);
      firstCrank.y = state.axes[0].y + state.a * Math.sin(state.autoRotateAngle);

      // Solve kinematics forward through the chain
      solveKinematics(firstCrank, 25);
      recordTraces();
    }

    draw();
    requestAnimationFrame(animate);
  }

  // --- Coordinate Transformation Helpers ---
  function getMouseWorldPos(e) {
    const rect = canvas.getBoundingClientRect();
    const clientX = e.clientX - rect.left;
    const clientY = e.clientY - rect.top;

    // Transform by pan & zoom
    return {
      x: (clientX - state.view.panX) / state.view.zoom,
      y: (clientY - state.view.panY) / state.view.zoom
    };
  }

  function findNodeUnderMouse(worldX, worldY) {
    const hitRadiusPadding = 8 / state.view.zoom;

    // 1. Check Cranks (P_i)
    for (const crank of state.cranks) {
      if (Math.hypot(crank.x - worldX, crank.y - worldY) <= crank.radius + hitRadiusPadding) {
        return crank;
      }
    }

    // 2. Check Fixed Axes (O_i)
    for (const axis of state.axes) {
      if (Math.hypot(axis.x - worldX, axis.y - worldY) <= axis.radius + hitRadiusPadding) {
        return axis;
      }
    }

    return null;
  }

  // --- Mouse / Touch Interaction ---
  canvas.addEventListener('mousedown', (e) => {
    const worldPos = getMouseWorldPos(e);
    const node = findNodeUnderMouse(worldPos.x, worldPos.y);

    if (e.button === 0 && node) {
      // Clicked on a joint or axis
      state.draggedNode = node;
      state.dragOffset.x = node.x - worldPos.x;
      state.dragOffset.y = node.y - worldPos.y;

      // 요구사항 4: 사용자가 마우스로 관절을 직접 잡고 움직이면 자동 회전 즉시 일시 정지
      triggerSafetyAutoPause();
      canvas.style.cursor = 'grabbing';
    } else if (e.button === 1 || e.button === 2 || (!node && e.button === 0)) {
      // Start Canvas Panning
      state.view.isPanning = true;
      state.view.panStartX = e.clientX - state.view.panX;
      state.view.panStartY = e.clientY - state.view.panY;
      canvas.style.cursor = 'move';
    }
  });

  window.addEventListener('mousemove', (e) => {
    const worldPos = getMouseWorldPos(e);

    if (state.draggedNode) {
      const node = state.draggedNode;
      const targetX = worldPos.x + state.dragOffset.x;
      const targetY = worldPos.y + state.dragOffset.y;

      if (node.type === 'axis') {
        // Dragging fixed axis: translates axis & its crank
        const dx = targetX - node.x;
        const dy = targetY - node.y;
        node.x = targetX;
        node.y = targetY;
        const crank = state.cranks[node.index];
        if (crank) {
          crank.x += dx;
          crank.y += dy;
        }
        solveKinematics(node, 20);
      } else if (node.type === 'crank') {
        // Dragging crank joint P_i: rotates on circle of radius a
        const axis = state.axes[node.axisIndex];
        node.angle = Math.atan2(targetY - axis.y, targetX - axis.x);
        node.x = axis.x + state.a * Math.cos(node.angle);
        node.y = axis.y + state.a * Math.sin(node.angle);

        // If dragging crank 1, synchronize state.autoRotateAngle
        if (node.axisIndex === 0) {
          state.autoRotateAngle = node.angle;
        }

        // Run kinematic chain relaxation!
        solveKinematics(node, 30);
      }

      recordTraces();
    } else if (state.view.isPanning) {
      state.view.panX = e.clientX - state.view.panStartX;
      state.view.panY = e.clientY - state.view.panStartY;
    } else {
      // Hover detection
      const hovered = findNodeUnderMouse(worldPos.x, worldPos.y);
      state.hoveredNode = hovered;
      if (hovered) {
        canvas.style.cursor = 'grab';
      } else {
        canvas.style.cursor = 'default';
      }
    }
  });

  window.addEventListener('mouseup', () => {
    if (state.draggedNode) {
      state.draggedNode = null;
      canvas.style.cursor = state.hoveredNode ? 'grab' : 'default';
    }
    if (state.view.isPanning) {
      state.view.isPanning = false;
      canvas.style.cursor = 'default';
    }
  });

  canvas.addEventListener('contextmenu', (e) => e.preventDefault());

  // Mouse Wheel Zoom
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
    const newZoom = Math.min(Math.max(0.3, state.view.zoom * zoomFactor), 4.0);

    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    // Zoom centered at mouse cursor
    state.view.panX = mouseX - (mouseX - state.view.panX) * (newZoom / state.view.zoom);
    state.view.panY = mouseY - (mouseY - state.view.panY) * (newZoom / state.view.zoom);
    state.view.zoom = newZoom;
  }, { passive: false });

  // --- Reset View (Center Fit) ---
  function resetView() {
    if (state.axes.length === 0) return;
    const rect = canvas.parentElement.getBoundingClientRect();

    // Compute bounding box
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    const allNodes = [...state.axes, ...state.cranks];
    for (const node of allNodes) {
      minX = Math.min(minX, node.x);
      maxX = Math.max(maxX, node.x);
      minY = Math.min(minY, node.y);
      maxY = Math.max(maxY, node.y);
    }

    const padding = 80;
    const w = (maxX - minX) + padding * 2;
    const h = (maxY - minY) + padding * 2;
    const zoom = Math.min(rect.width / w, rect.height / h, 1.2);

    state.view.zoom = zoom;
    state.view.panX = (rect.width - (minX + maxX) * zoom) / 2;
    state.view.panY = (rect.height - (minY + maxY) * zoom) / 2;
  }

  // --- Top Dashboard Values Update ---
  function updateDashboard() {
    dispN.textContent = state.N;
    dispR.textContent = `${state.r} px`;
    dispA.textContent = `${state.a} px`;
    dispB.textContent = `${state.b} px`;
  }

  // --- Button & Input Event Handlers ---
  btnApply.addEventListener('click', () => {
    // Read and validate values
    const newN = Math.max(2, Math.min(10, parseInt(inputN.value, 10) || 4));
    const newR = Math.max(30, Math.min(350, parseFloat(inputR.value) || 120));
    const newA = Math.max(20, Math.min(250, parseFloat(inputA.value) || 80));
    const newB = Math.max(30, Math.min(400, parseFloat(inputB.value) || 130));
    const newSpeed = parseFloat(inputSpeed.value) || 45;

    state.N = newN;
    state.r = newR;
    state.a = newA;
    state.b = newB;
    state.speedDeg = newSpeed;

    initMechanism(false);
    clearTraces();
    resetView();
  });

  btnAutoRotate.addEventListener('click', () => {
    setAutoRotation(!state.isAutoRotating);
  });

  btnResetPose.addEventListener('click', () => {
    initMechanism(true);
    clearTraces();
  });

  btnCenterAxes.addEventListener('click', () => {
    initMechanism(false);
    clearTraces();
    resetView();
  });

  btnResetView.addEventListener('click', resetView);

  btnClearTrace.addEventListener('click', clearTraces);

  btnToggleHelp.addEventListener('click', () => {
    const guideSection = document.querySelector('.guide-content');
    if (guideSection) {
      guideSection.parentElement.scrollIntoView({ behavior: 'smooth' });
    }
  });

  // --- Initialization & Boot ---
  resizeCanvas();
  initMechanism(false);
  resetView();
  requestAnimationFrame(animate);

})();
