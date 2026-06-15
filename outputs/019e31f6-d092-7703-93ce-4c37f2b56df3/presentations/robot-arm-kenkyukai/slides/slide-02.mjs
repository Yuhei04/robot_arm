import { style, rect, text, circle, header, footer } from "./theme.mjs";

export async function slide02(presentation, ctx) {
  const slide = presentation.slides.add();
  rect(slide, ctx, 0, 0, ctx.W, ctx.H, style.bg);
  header(slide, ctx, 2, "動作確認: OpenCRから各軸を制御", "動画を中央の枠に差し替える想定");

  rect(slide, ctx, 64, 148, 790, 446, "#111820");
  rect(slide, ctx, 82, 166, 754, 410, "#1E2832", { lineColor: "#435160", lineWidth: 1 });
  circle(slide, ctx, 460, 372, 54, "#FFFFFF22", { lineColor: "#FFFFFF66", lineWidth: 2 });
  text(slide, ctx, "▶", 444, 338, 60, 64, { size: 54, color: "#FFFFFF", align: "center" });
  text(slide, ctx, "ここにロボットアームの動画を配置", 254, 450, 410, 30, { size: 22, bold: true, color: "#FFFFFF", align: "center" });
  text(slide, ctx, "ID 1〜5 の認識、現在位置読み取り、小角度動作を確認", 224, 488, 470, 26, { size: 15, color: "#C9D3DD", align: "center" });

  text(slide, ctx, "確認済み", 906, 152, 210, 26, { size: 18, bold: true, color: style.green });
  const checks = [
    "OpenCR から ping / scan",
    "present position の読み取り",
    "torque on / off",
    "小角度の往復動作",
    "ID1 の微振動を調査中",
  ];
  checks.forEach((label, i) => {
    const y = 198 + i * 58;
    circle(slide, ctx, 922, y + 8, 7, i < 4 ? style.green : style.orange);
    text(slide, ctx, label, 944, y, 250, 26, { size: 16, color: i < 4 ? style.ink : style.orange, bold: i === 4 });
  });

  rect(slide, ctx, 904, 514, 262, 70, "#FFFFFF", { lineColor: style.mute, lineWidth: 1 });
  text(slide, ctx, "次の確認", 928, 530, 110, 22, { size: 14, bold: true, color: style.soft });
  text(slide, ctx, "組み立て前に、関節ごとのゼロ位置と可動範囲を固定する。", 928, 554, 204, 38, { size: 14, color: style.ink });

  footer(slide, ctx);
  return slide;
}
