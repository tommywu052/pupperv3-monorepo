/**
 * Local, un-branded slide template (derived from the nvidia-pptx template but
 * with the NVIDIA logo / wording removed per request). White content slides,
 * centered title, green file-path subtitle, page number. No corner logo.
 */

const pptxgen = require("pptxgenjs");

const C = {
  white: "FFFFFF", black: "000000", text: "1A1A1A", textDim: "555555",
  textLight: "888888", green: "76B900", greenDk: "5A8F00", teal: "008564",
  purple: "5D1682", magenta: "890C58", blue: "0071C5", greyBg: "F5F5F5",
  greyCard: "F0F0F0", greyLine: "DDDDDD", greyDark: "333333",
};

const FONT = "Segoe UI";

function createPresentation(options = {}) {
  const pres = new pptxgen();
  pres.layout = "LAYOUT_16x9";
  pres.author = options.author || "Pupper v3";
  pres.title = options.title || "Presentation";

  let pageCounter = 0;

  function _addPageNum(s, n) {
    s.addText(`${n}`, { x: 0.4, y: 5.15, w: 0.5, h: 0.3, fontSize: 9, fontFace: FONT, color: C.textLight });
  }

  return {
    pres, C, FONT,

    /** Cover slide: clean white title page (no logo / branding) */
    addCoverSlide(title, subtitle) {
      const s = pres.addSlide();
      s.background = { color: C.white };
      s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 0.18, h: 5.625, fill: { color: C.green }, line: { type: "none" } });
      const titleParts = Array.isArray(title)
        ? title.map((t, i) => ({ text: t, options: { fontSize: 40, fontFace: FONT, color: C.text, bold: true, breakLine: i < title.length - 1 } }))
        : [{ text: title, options: { fontSize: 40, fontFace: FONT, color: C.text, bold: true } }];
      s.addText(titleParts, { x: 0.6, y: 1.6, w: 8.8, h: 2.0, margin: 0 });
      if (subtitle) s.addText(subtitle, { x: 0.62, y: 3.7, w: 8.5, h: 0.4, fontSize: 16, fontFace: FONT, color: C.green });
      return s;
    },

    /** Content slide: white bg + centered title/subtitle + page number */
    addContentSlide(title, subtitle, builder) {
      pageCounter++;
      const s = pres.addSlide();
      s.background = { color: C.white };
      s.addText(title, { x: 0.5, y: 0.18, w: 9, h: 0.45, fontSize: 22, fontFace: FONT, color: C.text, bold: true, align: "center", margin: 0 });
      if (subtitle) s.addText(subtitle, { x: 0.5, y: 0.6, w: 9, h: 0.3, fontSize: 11, fontFace: FONT, color: C.green, align: "center" });
      _addPageNum(s, pageCounter);
      if (builder) builder(s, { C, FONT, pres });
      return s;
    },

    async save(outputPath) {
      await pres.writeFile({ fileName: outputPath });
      console.log("Created: " + outputPath);
      return outputPath;
    },
  };
}

module.exports = { createPresentation, C, FONT };
