/**
 * 2D Linkage Mechanism Simulator (2차원 링크 메커니즘 시뮬레이터)
 * Exact Analytical Rigid-Body Kinematics Engine with Uniform Initial Alignment
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
    a: 80,                 // 굵은 막대 a 길이 (완전 고정 강체)
    b: 120,                // 가는 막대 b 길이 (완전 고정 강체, 기본 r과 일치하여 완벽한 평행 구성)
    speedDeg: 45,          // 자동 회전 속도 (deg/sec)
    defaultAngle: -Math.PI / 3, // 상방 +60도 (수평면 기준 통일된 초기 기준 각도)
    
    isAutoRotating: false,
    autoRotateAngle: -Math.PI / 3,
    
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

  // Trace Colors
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

  function normalizeAngleDiff(angle1, angle2) {
    let diff = angle1 - angle2;
    while (diff < -Math.PI) diff += 2 * Math.PI;
    while (diff > Math.PI) diff -= 2 * Math.PI;
    return diff;
  }

  // --- Exact Circle-Circle Intersection Solver ---
  /**
   * Solves analytically for Point P such that:
   * 1. |P - O| = r1 (Radius r1 around fixed axis O, length of bar a)
   * 2. |P - targetP| = r2 (Radius r2 around neighbor joint, length of bar b)
   * 
   * Guarantees 100% exact mathematical rigidity with ZERO stretching or deformation.
   * refAngleOrPrevP:
   *  - if number: selects branch minimizing angular difference to this reference angle (parallel uncrossed branch)
   *  - if object {x, y}: selects branch closest to this previous point (smooth trajectory continuity)
   */
  function solveCircleIntersection(O, r1, targetP, r2, refAngleOrPrevP = null) {
    const dx = targetP.x - O.x;
    const dy = targetP.y - O.y;
    const d = Math.hypot(dx, dy);

    // Physical Reach Limit Check
    const eps = 1e-5;
    if (d > r1 + r2 + eps || d < Math.abs(r1 - r2) - eps || d < eps) {
      return null; // Out of geometric reach limit
    }

    const clampedD = Math.max(Math.abs(r1 - r2), Math.min(r1 + r2, d));

    // Distance from O to the chord along O -> targetP line
    const x0 = (r1 * r1 - r2 * r2 + clampedD * clampedD) / (2 * clampedD);
    const hSq = r1 * r1 - x0 * x0;
    const h = Math.sqrt(Math.max(0, hSq));

    const ux = dx / clampedD;
    const uy = dy / clampedD;
    const vx = -uy;
    const vy = ux;

    const sol1 = {
      x: O.x + x0 * ux + h * vx,
      y: O.y + x0 * uy + h * vy
    };
    const sol2 = {
      x: O.x + x0 * ux - h * vx,
      y: O.y + x0 * uy - h * vy
    };

    if (refAngleOrPrevP === null) {
      // Default to upper branch in canvas coordinates (smaller y)
      return sol1.y <= sol2.y ? sol1 : sol2;
    }

    if (typeof refAngleOrPrevP === 'number') {
      // Choose solution whose angle is closest to refAngle (uncrossed parallel branch)
      const a1 = Math.atan2(sol1.y - O.y, sol1.x - O.x);
      const a2 = Math.atan2(sol2.y - O.y, sol2.x - O.x);
      const diff1 = Math.abs(normalizeAngleDiff(a1, refAngleOrPrevP));
      const diff2 = Math.abs(normalizeAngleDiff(a2, refAngleOrPrevP));
      return diff1 <= diff2 ? sol1 : sol2;
    }

    // Branch continuity: select solution closest to previous position
    const d1 = Math.hypot(sol1.x - refAngleOrPrevP.x, sol1.y - refAngleOrPrevP.y);
    const d2 = Math.hypot(sol2.x - refAngleOrPrevP.x, sol2.y - refAngleOrPrevP.y);
    return d1 <= d2 ? sol1 : sol2;
  }

  // --- Cascading Chain Kinematics Solver ---
  /**
   * Solves the whole linkage chain with crank drivenIndex at targetAngle.
   * Maintains uncrossed, parallel-aligned continuous motion.
   */
  function solveChainFrom(drivenIndex, targetAngle) {
    const { N, a, b, axes, cranks } = state;
    const cand = new Array(N);

    // 1. Driven crank position
    const drivenAxis = axes[drivenIndex];
    cand[drivenIndex] = {
      x: drivenAxis.x + a * Math.cos(targetAngle),
      y: drivenAxis.y + a * Math.sin(targetAngle),
      angle: targetAngle
    };

    // 2. Backward propagation: from drivenIndex - 1 down to 0
    for (let i = drivenIndex - 1; i >= 0; i--) {
      const axis = axes[i];
      const nextP = cand[i + 1];
      const prevCrank = cranks[i] && cranks[i].x !== undefined ? cranks[i] : cand[i + 1].angle;
      const sol = solveCircleIntersection(axis, a, nextP, b, prevCrank);
      if (!sol) return null; // Backward reach limit hit

      cand[i] = {
        x: sol.x,
        y: sol.y,
        angle: Math.atan2(sol.y - axis.y, sol.x - axis.x)
      };
    }

    // 3. Forward propagation: from drivenIndex + 1 up to N - 1
    for (let i = drivenIndex + 1; i < N; i++) {
      const axis = axes[i];
      const prevP = cand[i - 1];
      const prevCrank = cranks[i] && cranks[i].x !== undefined ? cranks[i] : cand[i - 1].angle;
      const sol = solveCircleIntersection(axis, a, prevP, b, prevCrank);
      if (!sol) return null; // Forward reach limit hit

      cand[i] = {
        x: sol.x,
        y: sol.y,
        angle: Math.atan2(sol.y - axis.y, sol.x - axis.x)
      };
    }

    return cand;
  }

  /**
   * Applies validated candidate chain positions to state.cranks.
   */
  function applyChain(cand) {
    for (let i = 0; i < state.N; i++) {
      state.cranks[i].x = cand[i].x;
      state.cranks[i].y = cand[i].y;
      state.cranks[i].angle = cand[i].angle;
    }
  }

  // --- Uniform Initial Pose Alignment ---
  /**
   * 요구사항 1 & 2: 초기 상태 및 초기화 시 모든 막대 a의 각도를 동일하게 정렬 (예: 상방 +60도).
   * 꼬임 없이 순차적으로 자연스럽게 정렬된 uncrossed parallel chain을 구성.
   */
  function setUniformInitialPose(targetAngle = state.defaultAngle) {
    const { N, a, b, axes } = state;
    const cand = new Array(N);

    // Crank 0 at uniform target angle (상방 60도)
    cand[0] = {
      x: axes[0].x + a * Math.cos(targetAngle),
      y: axes[0].y + a * Math.sin(targetAngle),
      angle: targetAngle
    };

    // Sequentially align all other cranks to the exact same uniform direction
    let isPossible = true;
    for (let i = 1; i < N; i++) {
      const axis = axes[i];
      const prevP = cand[i - 1];
      // Target the uniform targetAngle to strictly align parallel branches
      const sol = solveCircleIntersection(axis, a, prevP, b, targetAngle);
      if (!sol) {
        isPossible = false;
        break;
      }
      cand[i] = {
        x: sol.x,
        y: sol.y,
        angle: Math.atan2(sol.y - axis.y, sol.x - axis.x)
      };
    }

    if (isPossible) {
      applyChain(cand);
      state.autoRotateAngle = state.cranks[0].angle;
      return true;
    }

    // Fallback: If exact targetAngle cannot close due to custom r, a, b values,
    // search closest uniform upper angle
    const candidates = [-Math.PI / 2, -Math.PI / 3, -2 * Math.PI / 3, -Math.PI / 4, -0.2, 0.2];
    for (const ang of candidates) {
      const testCand = solveChainFrom(0, ang);
      if (testCand) {
        applyChain(testCand);
        state.autoRotateAngle = state.cranks[0].angle;
        return true;
      }
    }

    showAlert('⚠️ 현재 입력된 수치로는 링크를 연결할 수 없습니다. 축 간격(r)이나 막대 길이를 확인해 주세요.');
    return false;
  }

  // --- Mechanism Initialization ---
  function initMechanism(keepBasePositions = false) {
    const { N, r, a } = state;

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

    // 2. Initialize Cranks (P_1 .. P_N)
    state.cranks = [];
    for (let i = 0; i < N; i++) {
      const axis = state.axes[i];
      state.cranks.push({
        id: `P${i + 1}`,
        index: i,
        type: 'crank',
        angle: state.defaultAngle,
        x: axis.x + a * Math.cos(state.defaultAngle),
        y: axis.y + a * Math.sin(state.defaultAngle),
        radius: 8,
        axisIndex: i
      });
    }

    // 3. Apply Uniform Initial Alignment
    setUniformInitialPose(state.defaultAngle);

    updateDashboard();
  }

  // --- Trace Management ---
  function recordTraces() {
    if (!checkTrace.checked) return;

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

  function showAlert(msg) {
    hudAutoPauseAlert.textContent = msg;
    hudAutoPauseAlert.style.display = 'block';
    hudAutoPauseAlert.style.animation = 'none';
    hudAutoPauseAlert.offsetHeight; // reflow
    hudAutoPauseAlert.style.animation = 'fadeInOut 2.5s ease forwards';
    setTimeout(() => {
      hudAutoPauseAlert.style.display = 'none';
    }, 2600);
  }

  function triggerSafetyAutoPause() {
    if (state.isAutoRotating) {
      setAutoRotation(false);
      showAlert('⚠️ 마우스 조작 감지: 안전을 위해 자동 회전이 즉시 정지되었습니다.');
    }
  }

  // --- Animation Loop ---
  let lastTimestamp = 0;
  function animate(timestamp) {
    if (!lastTimestamp) lastTimestamp = timestamp;
    const dt = Math.min((timestamp - lastTimestamp) / 1000, 0.05);
    lastTimestamp = timestamp;

    if (state.isAutoRotating && !state.draggedNode && state.cranks.length > 0) {
      const speedRad = (state.speedDeg * Math.PI) / 180;
      const nextAngle = state.autoRotateAngle + speedRad * dt;

      const cand = solveChainFrom(0, nextAngle);
      if (cand) {
        applyChain(cand);
        state.autoRotateAngle = nextAngle;
        recordTraces();
      } else {
        // Physical limit reached: stop auto-rotation without deforming bars
        setAutoRotation(false);
        showAlert('🛑 기하학적 한계(Reach Limit) 도달: 막대 길이를 유지하기 위해 자동 회전이 정지되었습니다.');
      }
    }

    draw();
    requestAnimationFrame(animate);
  }

  // --- Rendering ---
  function draw() {
    const { width, height } = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, width, height);

    ctx.save();
    ctx.translate(state.view.panX, state.view.panY);
    ctx.scale(state.view.zoom, state.view.zoom);

    // 1. Ground baseline
    drawBaseSupportLines();

    // 2. Reach Limit Guides
    if (checkLimitGuides.checked) {
      drawLimitGuides();
    }

    // 3. Traces
    if (checkTrace.checked) {
      drawTraces();
    }

    // 4. Bars (Rigid 100% fixed lengths a and b)
    drawBars();

    // 5. Fixed Axes (O_1 .. O_N)
    drawAxes();

    // 6. Crank Joints (P_1 .. P_N)
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

    ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
    for (const axis of state.axes) {
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, a, 0, Math.PI * 2);
      ctx.stroke();
    }

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
    // --- Draw Thin Bars (b): P_i to P_{i+1} ---
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

      ctx.strokeStyle = '#0284c7';
      ctx.beginPath();
      ctx.moveTo(axis.x, axis.y);
      ctx.lineTo(crank.x, crank.y);
      ctx.stroke();

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

      // Ground bracket
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

      ctx.beginPath();
      for (let ox = -14; ox <= 14; ox += 7) {
        ctx.moveTo(axis.x + ox, axis.y + 20);
        ctx.lineTo(axis.x + ox - 5, axis.y + 26);
      }
      ctx.stroke();

      // Pivot pin
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, axis.radius, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#0f141c';
      ctx.beginPath();
      ctx.arc(axis.x, axis.y, 4, 0, Math.PI * 2);
      ctx.fill();

      if (isHovered || isDragged) {
        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.arc(axis.x, axis.y, axis.radius + 5, 0, Math.PI * 2);
        ctx.stroke();
      }

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
    for (const crank of state.cranks) {
      const isHovered = state.hoveredNode === crank;
      const isDragged = state.draggedNode === crank;

      ctx.save();
      if (isHovered || isDragged) {
        ctx.fillStyle = 'rgba(16, 185, 129, 0.25)';
        ctx.beginPath();
        ctx.arc(crank.x, crank.y, crank.radius + 6, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = '#10b981';
      ctx.strokeStyle = '#064e3b';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(crank.x, crank.y, crank.radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.beginPath();
      ctx.arc(crank.x, crank.y, 2.5, 0, Math.PI * 2);
      ctx.fill();

      if (checkLabels.checked) {
        ctx.fillStyle = '#6ee7b7';
        ctx.font = 'bold 11px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(crank.id, crank.x, crank.y - 12);
      }

      ctx.restore();
    }
  }

  // --- Coordinate Transformation Helpers ---
  function getMouseWorldPos(e) {
    const rect = canvas.getBoundingClientRect();
    const clientX = e.clientX - rect.left;
    const clientY = e.clientY - rect.top;

    return {
      x: (clientX - state.view.panX) / state.view.zoom,
      y: (clientY - state.view.panY) / state.view.zoom
    };
  }

  function findNodeUnderMouse(worldX, worldY) {
    const hitRadiusPadding = 8 / state.view.zoom;

    for (const crank of state.cranks) {
      if (Math.hypot(crank.x - worldX, crank.y - worldY) <= crank.radius + hitRadiusPadding) {
        return crank;
      }
    }

    for (const axis of state.axes) {
      if (Math.hypot(axis.x - worldX, axis.y - worldY) <= axis.radius + hitRadiusPadding) {
        return axis;
      }
    }

    return null;
  }

  // --- Mouse Interaction with Substepped Cascading Kinematics ---
  canvas.addEventListener('mousedown', (e) => {
    const worldPos = getMouseWorldPos(e);
    const node = findNodeUnderMouse(worldPos.x, worldPos.y);

    if (e.button === 0 && node) {
      state.draggedNode = node;
      state.dragOffset.x = node.x - worldPos.x;
      state.dragOffset.y = node.y - worldPos.y;

      triggerSafetyAutoPause();
      canvas.style.cursor = 'grabbing';
    } else if (e.button === 1 || e.button === 2 || (!node && e.button === 0)) {
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
        const oldX = node.x;
        const oldY = node.y;
        node.x = targetX;
        node.y = targetY;

        const testPose = solveChainFrom(node.index, state.cranks[node.index].angle);
        if (testPose) {
          applyChain(testPose);
        } else {
          node.x = oldX;
          node.y = oldY;
        }
      } else if (node.type === 'crank') {
        const k = node.index;
        const axis = state.axes[k];
        const targetAngle = Math.atan2(targetY - axis.y, targetX - axis.x);
        const currentAngle = node.angle;
        const delta = normalizeAngleDiff(targetAngle, currentAngle);

        if (Math.abs(delta) > 1e-4) {
          const steps = Math.min(25, Math.max(5, Math.ceil(Math.abs(delta) / 0.04)));
          let bestValidCand = null;

          for (let s = 1; s <= steps; s++) {
            const testAng = currentAngle + (delta * s) / steps;
            const cand = solveChainFrom(k, testAng);
            if (cand) {
              bestValidCand = cand;
            } else {
              break;
            }
          }

          if (bestValidCand) {
            applyChain(bestValidCand);
            if (k === 0) {
              state.autoRotateAngle = state.cranks[0].angle;
            }
          }
        }
      }

      recordTraces();
    } else if (state.view.isPanning) {
      state.view.panX = e.clientX - state.view.panStartX;
      state.view.panY = e.clientY - state.view.panStartY;
    } else {
      const hovered = findNodeUnderMouse(worldPos.x, worldPos.y);
      state.hoveredNode = hovered;
      canvas.style.cursor = hovered ? 'grab' : 'default';
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

    state.view.panX = mouseX - (mouseX - state.view.panX) * (newZoom / state.view.zoom);
    state.view.panY = mouseY - (mouseY - state.view.panY) * (newZoom / state.view.zoom);
    state.view.zoom = newZoom;
  }, { passive: false });

  // --- Reset View (Center Fit) ---
  function resetView() {
    if (state.axes.length === 0) return;
    const rect = canvas.parentElement.getBoundingClientRect();

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
    const newN = Math.max(2, Math.min(10, parseInt(inputN.value, 10) || 4));
    const newR = Math.max(30, Math.min(350, parseFloat(inputR.value) || 120));
    const newA = Math.max(20, Math.min(250, parseFloat(inputA.value) || 80));
    const newB = Math.max(30, Math.min(400, parseFloat(inputB.value) || 120));
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

  // 요구사항 1: '자세 초기화' 버튼 시 모든 관절의 시작 각도를 상방 +60도로 완전 통일
  btnResetPose.addEventListener('click', () => {
    setUniformInitialPose(state.defaultAngle);
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

  // --- Boot ---
  resizeCanvas();
  initMechanism(false);
  resetView();
  requestAnimationFrame(animate);

})();
