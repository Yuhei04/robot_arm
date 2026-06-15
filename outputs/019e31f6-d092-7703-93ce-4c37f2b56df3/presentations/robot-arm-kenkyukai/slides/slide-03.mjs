import { style, rect, text, circle, header, footer, rule } from "./theme.mjs";

export async function slide03(presentation, ctx) {
  const slide = presentation.slides.add();
  rect(slide, ctx, 0, 0, ctx.W, ctx.H, style.bg);
  header(slide, ctx, 3, "手先: 軽量な2指グリッパから始める", "可搬重量よりも、先端重量を下げて肩・肘の負担を抑える");

  rect(slide, ctx, 80, 160, 520, 390, "#FFFFFF", { lineColor: style.mute, lineWidth: 1 });
  text(slide, ctx, "手先写真 / CAD図を配置", 210, 324, 260, 28, { size: 22, bold: true, color: style.soft, align: "center" });
  text(slide, ctx, "差し替え枠", 288, 358, 100, 24, { size: 14, color: style.soft, align: "center" });

  rect(slide, ctx, 742, 238, 210, 38, style.dark);
  circle(slide, ctx, 734, 258, 30, style.blue);
  rect(slide, ctx, 930, 198, 42, 150, style.dark);
  rect(slide, ctx, 962, 182, 120, 20, style.dark);
  rect(slide, ctx, 962, 342, 120, 20, style.dark);
  rule(slide, ctx, 952, 258, 126, style.orange, 4);
  text(slide, ctx, "wrist roll", 704, 304, 110, 22, { size: 13, color: style.blue, face: style.mono });
  text(slide, ctx, "gripper", 986, 376, 110, 22, { size: 13, color: style.orange, face: style.mono });

  const notes = [
    ["軽さ優先", "XC330を使う場合でも、指とフレームの質量を抑える。"],
    ["対象物", "最初は軽いブロック、スポンジ、空箱などに限定する。"],
    ["設計余地", "指先材質、開閉幅、滑り止めは交換式にして試す。"],
  ];
  notes.forEach((n, i) => {
    const y = 448 + i * 56;
    text(slide, ctx, n[0], 684, y, 120, 24, { size: 17, bold: true });
    text(slide, ctx, n[1], 808, y, 340, 36, { size: 14, color: style.soft });
  });

  footer(slide, ctx);
  return slide;
}
