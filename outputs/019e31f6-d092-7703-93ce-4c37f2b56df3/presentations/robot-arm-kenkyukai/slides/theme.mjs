export const style = {
  bg: "#F6F7F4",
  ink: "#17202A",
  soft: "#5E6A71",
  mute: "#D9DED8",
  panel: "#FFFFFF",
  dark: "#17202A",
  green: "#26A269",
  blue: "#2F6F9F",
  orange: "#D48B31",
  red: "#B64B4B",
  sans: "Hiragino Sans",
  mono: "Menlo",
};

export function rect(slide, ctx, x, y, w, h, fill, opts = {}) {
  return ctx.addShape(slide, {
    left: x,
    top: y,
    width: w,
    height: h,
    geometry: opts.geometry ?? "rect",
    fill,
    line: opts.line ?? ctx.line(opts.lineColor ?? fill, opts.lineWidth ?? 0),
    name: opts.name,
  });
}

export function text(slide, ctx, value, x, y, w, h, opts = {}) {
  return ctx.addText(slide, {
    text: String(value ?? ""),
    left: x,
    top: y,
    width: w,
    height: h,
    fontSize: opts.size ?? 22,
    color: opts.color ?? style.ink,
    bold: Boolean(opts.bold),
    typeface: opts.face ?? style.sans,
    align: opts.align ?? "left",
    valign: opts.valign ?? "top",
    fill: "#00000000",
    line: ctx.line(),
    insets: opts.insets ?? { left: 0, right: 0, top: 0, bottom: 0 },
    name: opts.name,
  });
}

export function circle(slide, ctx, cx, cy, r, fill, opts = {}) {
  return rect(slide, ctx, cx - r, cy - r, r * 2, r * 2, fill, {
    geometry: "ellipse",
    lineColor: opts.lineColor ?? fill,
    lineWidth: opts.lineWidth ?? 0,
  });
}

export function rule(slide, ctx, x, y, w, color = style.ink, h = 2) {
  return rect(slide, ctx, x, y, w, h, color);
}

export function header(slide, ctx, number, title, subtitle) {
  text(slide, ctx, `0${number}`, 54, 38, 54, 28, {
    size: 15,
    bold: true,
    color: style.green,
    face: style.mono,
  });
  text(slide, ctx, title, 112, 32, 760, 44, { size: 30, bold: true });
  if (subtitle) {
    text(slide, ctx, subtitle, 114, 77, 760, 28, { size: 14, color: style.soft });
  }
  rule(slide, ctx, 54, 110, 1172, style.mute, 1);
}

export function footer(slide, ctx) {
  text(slide, ctx, "OpenCR + DYNAMIXEL / 3D printed robot arm", 54, 672, 520, 20, {
    size: 12,
    color: style.soft,
  });
}
