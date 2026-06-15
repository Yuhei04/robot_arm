import { style, rect, text, circle, header, footer } from "./theme.mjs";

export async function slide04(presentation, ctx) {
  const slide = presentation.slides.add();
  rect(slide, ctx, 0, 0, ctx.W, ctx.H, style.bg);
  header(slide, ctx, 4, "問題点と今後の課題", "機構・電装・制御を分けて、組み立て前に潰せるリスクを整理する");

  const cols = [
    ["機構", style.blue, ["J1の荷重をサーボ軸だけで受けない", "スラストベアリングと中心ガイドの設計", "3Dプリント部品のガタ・剛性"]],
    ["電装", style.orange, ["XC330-Mは12V系と混在不可", "配線の張りが根本軸の振動要因になる", "電源容量と電圧降下の確認"]],
    ["制御", style.green, ["ID管理とゼロ位置の固定", "ID1のハンチング対策", "関節ごとの可動範囲制限"]],
  ];

  cols.forEach((col, i) => {
    const x = 78 + i * 388;
    rect(slide, ctx, x, 166, 322, 370, "#FFFFFF", { lineColor: style.mute, lineWidth: 1 });
    circle(slide, ctx, x + 34, 206, 15, col[1]);
    text(slide, ctx, col[0], x + 62, 190, 180, 30, { size: 24, bold: true });
    col[2].forEach((item, j) => {
      const y = 258 + j * 78;
      rect(slide, ctx, x + 28, y + 8, 8, 8, col[1]);
      text(slide, ctx, item, x + 52, y, 232, 48, { size: 15, color: style.ink });
    });
  });

  rect(slide, ctx, 78, 574, 1090, 52, style.dark);
  text(slide, ctx, "次のマイルストーン: J1ベース単体試作 → 肩・肘を載せる → 手先を軽量化しながら動作範囲を広げる", 108, 590, 1030, 22, {
    size: 17,
    bold: true,
    color: "#FFFFFF",
  });

  footer(slide, ctx);
  return slide;
}
