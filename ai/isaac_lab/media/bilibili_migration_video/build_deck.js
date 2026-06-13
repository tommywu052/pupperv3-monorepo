/**
 * Build the NVIDIA-branded Pupper MJX -> Isaac Lab deck.
 *
 * Reads out/scenes.json (written by make_video.py) and produces out/deck.pptx
 * using the shared nvidia-pptx template. Each scene becomes one slide:
 *   - cover scene  -> green-swirl cover slide
 *   - screenshot   -> real VS Code screenshot embedded on the right
 *   - otherwise    -> a dark "VS Code style" code/config panel rendered from
 *                     the scene's code text
 * The left column always shows the chapter key points.
 */

const fs = require("fs");
const path = require("path");

const { createPresentation } = require(path.join(__dirname, "deck_template"));

const OUT = path.join(__dirname, "out");
const scenes = JSON.parse(fs.readFileSync(path.join(OUT, "scenes.json"), "utf-8"));

const EDITOR = {
  bg: "1E1E1E",
  bar: "323233",
  gutter: "858585",
  text: "D4D4D4",
  comment: "6A9955",
  green: "76B900",
};

const KW = ["target", "delay_prob", "PERM", "INV", "KP", "KD", "normalizer",
  "stiffness", "damping", "ImplicitActuator", "DelayedJointPositionAction",
  "randomize_actuator_gains", "export_policy_as_onnx", "forward_position",
  "forward_kp", "forward_kd", "decimation", "episode_length_s"];

function codeColor(line, accent) {
  const t = line.trim();
  if (t.startsWith("#")) return EDITOR.comment;
  if (KW.some((k) => line.includes(k))) return accent;
  return EDITOR.text;
}

function addEditorPanel(s, p, ctx, scene, box) {
  const { C, FONT } = ctx;
  const { x, y, w, h } = box;
  const accent = scene.accent || "76B900";
  // window
  s.addShape(p.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.06, fill: { color: EDITOR.bg }, line: { color: "3A3A3A", width: 1 } });
  // title bar
  s.addShape(p.shapes.RECTANGLE, { x, y, w, h: 0.34, fill: { color: EDITOR.bar }, line: { type: "none" } });
  ["FF5F56", "FFBD2E", "27C93F"].forEach((c, i) => {
    s.addShape(p.shapes.OVAL, { x: x + 0.12 + i * 0.18, y: y + 0.11, w: 0.11, h: 0.11, fill: { color: c }, line: { type: "none" } });
  });
  s.addText(scene.code_title || "config", { x: x + 0.7, y: y + 0.04, w: w - 0.8, h: 0.26, fontSize: 10, fontFace: "Consolas", color: "CCCCCC", valign: "middle" });

  // code body
  const lines = String(scene.code || "").replace(/\t/g, "    ").split("\n").slice(0, 16);
  const para = [];
  lines.forEach((ln, i) => {
    const num = String(i + 1).padStart(2, " ");
    para.push({ text: num + "  ", options: { color: EDITOR.gutter, fontFace: "Consolas", fontSize: 11.5 } });
    para.push({ text: ln === "" ? " " : ln, options: { color: codeColor(ln, accent), fontFace: "Consolas", fontSize: 11.5, breakLine: true } });
  });
  s.addText(para, { x: x + 0.18, y: y + 0.46, w: w - 0.36, h: h - 0.6, valign: "top", lineSpacingMultiple: 1.06, margin: 0 });
}

function addScreenshotPanel(s, p, ctx, scene, box) {
  const { C } = ctx;
  const { x, y, w, h } = box;
  s.addShape(p.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.04, fill: { color: "1E1E1E" }, line: { color: C.greyLine, width: 1 } });
  s.addImage({ path: scene.screenshot, x: x + 0.06, y: y + 0.06, w: w - 0.12, h: h - 0.12, sizing: { type: "contain", w: w - 0.12, h: h - 0.12 } });
}

function addExtraImage(s, p, ctx, scene, box) {
  const { C, FONT } = ctx;
  const { x, y, w, h } = box;
  s.addText("URDF -> USD in Isaac Sim", { x, y: y - 0.28, w, h: 0.26, fontSize: 10, fontFace: FONT, color: C.textDim });
  s.addShape(p.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.04, fill: { color: "1E1E1E" }, line: { color: C.greyLine, width: 1 } });
  s.addImage({ path: scene.extra_image, x: x + 0.06, y: y + 0.06, w: w - 0.12, h: h - 0.12, sizing: { type: "contain", w: w - 0.12, h: h - 0.12 } });
}

function addLinkBlock(s, ctx, scene, box) {
  const { C, FONT } = ctx;
  const { x, y, w } = box;
  s.addText("詳細說明 / Full write-up:", { x, y, w, h: 0.3, fontSize: 13, fontFace: FONT, color: C.text, bold: true });
  s.addText([
    { text: scene.link, options: { color: C.green, fontFace: FONT, fontSize: 13, underline: true, hyperlink: { url: scene.link } } },
  ], { x, y: y + 0.34, w, h: 0.6, valign: "top" });
}

function addBullets(s, ctx, scene, box) {
  const { C, FONT } = ctx;
  const { x, y, w } = box;
  const accent = scene.accent || "76B900";
  s.addText("Key points", { x, y, w, h: 0.32, fontSize: 14, fontFace: FONT, color: accent, bold: true });
  const para = (scene.bullets || []).map((b, i) => ({
    text: b,
    options: {
      bullet: { code: "2022", indent: 14 },
      color: C.text, fontFace: FONT, fontSize: 12.5,
      breakLine: true, paraSpaceAfter: 9,
    },
  }));
  s.addText(para, { x, y: y + 0.42, w, h: 3.4, valign: "top", margin: 0 });
}

async function main() {
  const pres = createPresentation({ title: "Pupper MJX -> Isaac Lab", author: "Pupper v3" });

  for (const scene of scenes) {
    if (scene.cover) {
      pres.addCoverSlide(scene.cover_lines || [scene.title], scene.subtitle || "");
      continue;
    }
    pres.addContentSlide(scene.title, scene.subtitle || "", (s, ctx) => {
      const p = ctx.pres;
      addBullets(s, ctx, scene, { x: 0.45, y: 1.05, w: 3.25 });
      const right = { x: 3.95, y: 1.0, w: 5.65, h: 4.15 };
      if (scene.screenshot) addScreenshotPanel(s, p, ctx, scene, right);
      else addEditorPanel(s, p, ctx, scene, right);
      if (scene.extra_image) addExtraImage(s, p, ctx, scene, { x: 0.45, y: 3.35, w: 3.05, h: 1.8 });
      if (scene.link) addLinkBlock(s, ctx, scene, { x: 0.45, y: 3.55, w: 3.3 });
    });
  }

  await pres.save(path.join(OUT, "deck.pptx"));
}

main().catch((e) => { console.error(e); process.exit(1); });
