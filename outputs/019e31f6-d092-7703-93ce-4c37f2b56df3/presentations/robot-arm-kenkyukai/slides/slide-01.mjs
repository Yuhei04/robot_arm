import { style, rect, text, circle, rule, footer } from "./theme.mjs";

export async function slide01(presentation, ctx) {
  const slide = presentation.slides.add();
  rect(slide, ctx, 0, 0, ctx.W, ctx.H, style.bg);

  text(slide, ctx, "ロボットアームを作る動機", 58, 54, 760, 60, {
    size: 38,
    bold: true,
  });
  text(slide, ctx, "小型・低コストな実機で、設計から制御、模倣学習まで一貫して試せる環境を作る。", 60, 120, 780, 48, {
    size: 19,
    color: style.soft,
  });

  const items = [
    ["01", "実機で試す", "シミュレーションだけでは見えにくい摩擦、ガタ、配線、電源の問題を含めて学習する。"],
    ["02", "自分で直せる", "3Dプリント部品とDYNAMIXEL構成にして、破損や改造を短いサイクルで回す。"],
    ["03", "データを取る", "leader/follower や遠隔操作へ拡張し、動作データ収集の土台にする。"],
  ];
  items.forEach((item, i) => {
    const y = 214 + i * 122;
    circle(slide, ctx, 84, y + 20, 21, i === 0 ? style.green : i === 1 ? style.blue : style.orange);
    text(slide, ctx, item[0], 68, y + 8, 32, 22, { size: 12, bold: true, color: "#FFFFFF", align: "center", face: style.mono });
    text(slide, ctx, item[1], 124, y, 260, 30, { size: 24, bold: true });
    text(slide, ctx, item[2], 124, y + 38, 520, 48, { size: 16, color: style.soft });
  });

  rect(slide, ctx, 790, 166, 386, 400, "#E8EDE7");
  circle(slide, ctx, 954, 476, 42, style.dark);
  rule(slide, ctx, 954, 270, 8, style.dark, 210);
  circle(slide, ctx, 958, 268, 28, style.green);
  rect(slide, ctx, 954, 253, 170, 14, style.dark);
  circle(slide, ctx, 1126, 260, 24, style.blue);
  rect(slide, ctx, 1118, 260, 12, 112, style.dark);
  circle(slide, ctx, 1124, 376, 20, style.orange);
  rect(slide, ctx, 1094, 374, 82, 10, style.dark);
  rect(slide, ctx, 1174, 354, 12, 52, style.dark);
  text(slide, ctx, "5軸 + グリッパ", 816, 196, 210, 32, { size: 22, bold: true });
  text(slide, ctx, "XC430 / 2XC430 を中心に構成", 816, 230, 260, 28, { size: 15, color: style.soft });

  footer(slide, ctx);
  return slide;
}
