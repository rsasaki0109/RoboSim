(() => {
/* The native host owns simulation state. Browser animation only requests bounded fixed steps. */
"use strict";
const $ = id => document.getElementById(id);
let state = null;
let playing = false;
let playEpoch = 0;
let queue = Promise.resolve();
let modelKey = "";
let objectKey = "";
let poseKey = "";
const jointControls = new Map();

function status(message, error = false) {
  $("status").textContent = message;
  $("status").classList.toggle("error", error);
}
function pause() {
  playing = false;
  playEpoch++;
  $("play").textContent = "▶ 再生";
  $("mode").textContent = "一時停止";
}
function failed(error) {
  pause();
  if (state) display();
  status(error.message || String(error), true);
}
function run(action) { Promise.resolve().then(action).catch(failed); }
function request(command) {
  const task = queue.then(async () => {
    if (typeof command === "function") command = command();
    const response = await fetch(command ? "/api/command" : "/api/state", command ? {
      method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(command),
    } : {cache: "no-store"});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    state = data;
    display();
    status("接続済み · ネイティブ物理と同じシーンからセンサーを取得しています");
    return data;
  });
  queue = task.catch(() => {});
  return task;
}
async function tick(epoch) {
  if (!playing || epoch !== playEpoch) return;
  try {
    await request({command: "step", steps: 4});
    if (playing && epoch === playEpoch) setTimeout(() => tick(epoch), 80);
  } catch (error) { failed(error); }
}
$("play").onclick = () => {
  if (playing) { pause(); return; }
  playing = true;
  const epoch = ++playEpoch;
  $("play").textContent = "⏸ 一時停止";
  $("mode").textContent = "固定刻みで実行中";
  tick(epoch);
};
$("step").onclick = () => run(async () => { pause(); await request({command: "step", steps: 1}); });
$("reset").onclick = () => run(async () => { pause(); await request({command: "reset"}); });
$("preset").onchange = () => run(async () => { pause(); await request({command: "preset", name: $("preset").value}); });

function position(value, linear) {
  if (!Number.isFinite(value)) throw new Error("関節目標は有限の数値で指定してください");
  return linear ? {kind: "prismatic", position_m: value} : {kind: "revolute", position_rad: value};
}
function displayJoints() {
  const key = JSON.stringify([state.project.robot, state.joints.map(j => j.info)]);
  if (key !== modelKey) {
    modelKey = key;
    jointControls.clear();
    $("joints").replaceChildren();
    for (const joint of state.joints) {
      const info = joint.info;
      const row = document.createElement("div"); row.className = "joint";
      const label = document.createElement("label"); label.className = "joint-name";
      label.textContent = `${info.name} · ${info.linear ? "m" : "rad"}`;
      const range = document.createElement("input"); range.type = "range";
      const number = document.createElement("input"); number.type = "number";
      for (const input of [range, number]) {
        input.min = info.lower; input.max = info.upper; input.step = "any";
        input.setAttribute("aria-label", `${info.name} ${input === range ? "目標スライダー" : "目標値"}`);
      }
      const readout = document.createElement("div"); readout.className = "joint-values";
      range.oninput = () => { number.value = range.value; };
      const commit = value => run(async () => {
        await request(() => {
          const targets = structuredClone(state.project.targets);
          targets[info.name] = position(Number(value), info.linear);
          return {command: "targets", targets};
        });
      });
      range.onchange = () => commit(range.value);
      number.onchange = () => commit(number.value);
      label.append(range);
      row.append(label, number, readout); $("joints").append(row);
      jointControls.set(info.name, {range, number, readout});
    }
  }
  for (const joint of state.joints) {
    const controls = jointControls.get(joint.info.name);
    const target = joint.info.linear ? joint.target.position_m : joint.target.position_rad;
    for (const input of [controls.range, controls.number]) {
      if (document.activeElement !== input) input.value = target;
    }
    controls.readout.textContent = `実測 ${joint.measured.toFixed(4)} / 目標 ${target.toFixed(4)}`;
  }
  $("servo-note").textContent = state.joints.some(j => !j.info.limits_from_model)
    ? "駆動仕様は未定義です。プレビュー用上限：1 rad/s・10 N·m（直動は1 m/s・10 N）。"
    : "関節の位置・目標速度・力/トルク上限はURDFの指定を使用します。";
}
function populate(select, names, previous) {
  select.replaceChildren(...names.map(name => { const option = document.createElement("option"); option.value = name; option.textContent = name; return option; }));
  if (names.includes(previous)) select.value = previous;
  else if (names.length) select.selectedIndex = 0;
}
function displayLists() {
  const names = Object.keys(state.project.poses);
  const nextPoseKey = JSON.stringify(names);
  if (nextPoseKey !== poseKey) { poseKey = nextPoseKey; populate($("poses"), names, $("poses").value); }
  $("apply-pose").disabled = names.length === 0;
  const nextObjectKey = JSON.stringify(state.project.objects);
  if (nextObjectKey !== objectKey) {
    objectKey = nextObjectKey;
    populate($("objects"), state.project.objects.map(o => o.name), $("objects").value);
    selectObject();
  }
}
function selectObject() {
  if (!state) return;
  const object = state.project.objects.find(o => o.name === $("objects").value);
  $("apply-object").disabled = !object;
  $("remove-object").disabled = !object;
  if (!object) return;
  $("object-name").value = object.name;
  ["x", "y", "z"].forEach((axis, i) => {
    $(`pos-${axis}`).value = object.position_m[i];
    $(`size-${axis}`).value = object.size_m[i];
  });
  $("object-color").value = "#" + object.color_rgba.slice(0, 3).map(v => Math.round(v * 255).toString(16).padStart(2, "0")).join("");
}
$("objects").onchange = selectObject;
$("save-pose").onclick = () => run(() => request({command: "save_pose", name: $("pose-name").value.trim()}));
$("apply-pose").onclick = () => run(() => request({command: "apply_pose", name: $("poses").value}));
async function replaceProject(project) { pause(); await request({command: "replace_project", project}); }
$("apply-object").onclick = () => run(async () => {
  const project = structuredClone(state.project);
  const index = project.objects.findIndex(o => o.name === $("objects").value);
  if (index < 0) throw new Error("編集するオブジェクトを選んでください");
  const color = $("object-color").value;
  project.objects[index] = {
    name: $("object-name").value.trim(),
    position_m: ["x", "y", "z"].map(a => Number($(`pos-${a}`).value)),
    size_m: ["x", "y", "z"].map(a => Number($(`size-${a}`).value)),
    color_rgba: [1, 3, 5].map(i => parseInt(color.slice(i, i + 2), 16) / 255).concat(1),
  };
  await replaceProject(project);
});
$("add-object").onclick = () => run(async () => {
  const project = structuredClone(state.project);
  let n = 1;
  while (project.objects.some(o => o.name === `box-${n}`)) n++;
  project.objects.push({name: `box-${n}`, position_m: [1, 0.3, 0], size_m: [0.4, 0.6, 0.4], color_rgba: [0.85, 0.55, 0.26, 1]});
  await replaceProject(project);
  $("objects").value = `box-${n}`; selectObject();
});
$("remove-object").onclick = () => run(async () => {
  const project = structuredClone(state.project);
  project.objects = project.objects.filter(o => o.name !== $("objects").value);
  await replaceProject(project);
});
for (const id of ["yaw", "pitch", "distance"]) $(id).onchange = () => run(() => request({
  command: "view", yaw_rad: Number($("yaw").value), pitch_rad: Number($("pitch").value), distance_m: Number($("distance").value),
}));

function rgbImage(canvas, width, height, data) {
  canvas.getContext("2d").putImageData(new ImageData(data, width, height), 0, 0);
}
function displaySensors() {
  const camera = state.camera;
  rgbImage($("rgb"), camera.width, camera.height, Uint8ClampedArray.from(atob(camera.rgba8_base64), c => c.charCodeAt(0)));
  const colors = new Uint8ClampedArray(camera.width * camera.height * 4);
  camera.depth_m.forEach((v, i) => {
    if (Number.isFinite(v)) {
      const t = Math.max(0, Math.min(1, v / camera.far_m));
      colors[i * 4] = 255 * t; colors[i * 4 + 1] = 230 * (1 - t); colors[i * 4 + 2] = 255 * (1 - t);
    }
    colors[i * 4 + 3] = 255;
  });
  rgbImage($("depth"), camera.width, camera.height, colors);
  const canvas = $("lidar"), ctx = canvas.getContext("2d"), scan = state.lidar;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const scale = 25, cx = canvas.width / 2, cy = canvas.height / 2;
  ctx.strokeStyle = "#2b4055";
  for (let r = 1; r <= 3; r++) { ctx.beginPath(); ctx.arc(cx, cy, r * scale, 0, Math.PI * 2); ctx.stroke(); }
  ctx.fillStyle = "#62e4c3";
  for (const point of scan.points_m) ctx.fillRect(cx + (point[0] - scan.origin_m[0]) * scale - 1, cy - (point[2] - scan.origin_m[2]) * scale - 1, 3, 3);
  ctx.fillStyle = "#fff"; ctx.fillRect(cx - 2, cy - 2, 4, 4);
  $("lidar-info").textContent = `${scan.points_m.length} / ${scan.ranges_m.length} rays · 同心円 1 m · 高さ ${scan.origin_m[1]} m`;
  $("sensor-time").textContent = `センサー取得時刻 ${(Number(camera.sim_time_ticks) / 1e9).toFixed(4)} s · 深度の黒は範囲外`;
}
$("depth").onmousemove = event => {
  if (!state) return;
  const bounds = event.currentTarget.getBoundingClientRect();
  const x = Math.min(159, Math.max(0, Math.floor((event.clientX - bounds.left) / bounds.width * 160)));
  const y = Math.min(119, Math.max(0, Math.floor((event.clientY - bounds.top) / bounds.height * 120)));
  const value = state.camera.depth_m[y * 160 + x];
  $("depth-value").textContent = Number.isFinite(value) ? `(${x}, ${y}) ${value.toFixed(4)} m` : `(${x}, ${y}) 範囲外`;
};
function display() {
  $("clock").textContent = `${(Number(state.sim_time_ticks) / 1e9).toFixed(3)} s`;
  $("hash").textContent = state.state_hash;
  $("inertias").value = state.project.use_declared_inertias ? "declared" : "preview";
  $("up-axis").value = Math.abs(state.project.robot_rotation_rpy_rad[0] + Math.PI / 2) < 1e-6 ? "z" : "y";
  for (const [id, key] of [["yaw", "yaw_rad"], ["pitch", "pitch_rad"], ["distance", "distance_m"]]) {
    if (document.activeElement !== $(id)) $(id).value = state.orbit[key];
  }
  $("view").src = `data:image/png;base64,${state.view_png_base64}`;
  displayJoints(); displayLists(); displaySensors();
}
function download(name, data) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(data) + "\n"], {type: "application/json"}));
  const a = document.createElement("a"); a.href = url; a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
$("download-project").onclick = () => run(() => download("robot.rne.workbench.json", state.project));
async function readFile(input, maxBytes) {
  const file = input.files[0];
  if (!file) return null;
  if (file.size > maxBytes) throw new Error("ファイルがサイズ上限を超えています");
  return {name: file.name, text: await file.text()};
}
$("project-file").onchange = () => run(async () => {
  const file = await readFile($("project-file"), 4 * 1024 * 1024);
  if (file) await replaceProject(JSON.parse(file.text));
  $("project-file").value = "";
});
$("robot-file").onchange = () => run(async () => {
  const file = await readFile($("robot-file"), 2 * 1024 * 1024);
  if (!file) return;
  pause();
  const format = /<mujoco(?:\s|>)/.test(file.text) ? "mjcf" : "urdf";
  await request({command: "import", format, xml: file.text});
  $("robot-file").value = "";
});
run(() => request());

$("inertias").onchange = () => run(async () => {
  const project = structuredClone(state.project);
  project.use_declared_inertias = $("inertias").value === "declared";
  await replaceProject(project);
});
$("up-axis").onchange = () => run(async () => {
  const project = structuredClone(state.project);
  project.robot_rotation_rpy_rad = $("up-axis").value === "z" ? [-Math.PI / 2, 0, 0] : [0, 0, 0];
  await replaceProject(project);
});

})();
