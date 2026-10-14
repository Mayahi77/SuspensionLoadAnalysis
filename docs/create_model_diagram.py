from PIL import Image, ImageDraw, ImageFont
from pathlib import Path

root = Path(__file__).resolve().parents[1]
out = root / "docs" / "model_diagram.png"
w, h = 1400, 760
img = Image.new("RGB", (w, h), "white")
d = ImageDraw.Draw(img)
def load_font(size, bold=False):
    names = ["arialbd.ttf", "Arial Bold.ttf"] if bold else ["arial.ttf", "Arial.ttf"]
    for name in names:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()

font = load_font(30)
small = load_font(24)
bold = load_font(34, bold=True)
navy = (28, 59, 87)
blue = (76, 132, 184)
gray = (75, 82, 88)
orange = (211, 125, 54)

d.text((50, 30), "Passive 2-DOF quarter-car model", fill=navy, font=bold)
d.text((50, 80), "Coordinates are upward-positive perturbations about static equilibrium", fill=gray, font=small)
d.rounded_rectangle((470, 160, 930, 270), radius=15, fill=(218, 232, 244), outline=navy, width=4)
d.text((590, 193), "sprung mass ms", fill=navy, font=bold)
d.rounded_rectangle((520, 450, 880, 535), radius=12, fill=(232, 232, 232), outline=gray, width=4)
d.text((610, 475), "unsprung mass mu", fill=gray, font=bold)
pts = [(575,270),(575,292),(545,310),(605,338),(545,366),(605,394),(575,420),(575,450)]
d.line(pts, fill=blue, width=7, joint="curve")
d.text((385, 320), "spring ks", fill=blue, font=font)
d.line((820,270,820,305), fill=orange, width=7)
d.rectangle((792,305,848,370), fill=(250,230,210), outline=orange, width=4)
d.line((820,370,820,450), fill=orange, width=7)
d.text((895, 335), "damper cs", fill=orange, font=font)
d.line((700,535,700,595), fill=gray, width=8)
d.rounded_rectangle((625,595,775,655), radius=10, fill=(224,224,224), outline=gray, width=4)
d.text((815, 603), "tire kt", fill=gray, font=font)
d.line((180,655,1220,655), fill=orange, width=8)
d.text((1030, 685), "road displacement r(x)", fill=orange, font=font)
d.line((1160,600,1160,180), fill=navy, width=5)
d.polygon([(1160,145),(1145,185),(1175,185)], fill=navy)
d.text((1180, 205), "+z", fill=navy, font=font)
d.line((950,220,950,150), fill=blue, width=4)
d.polygon([(950,140),(940,165),(960,165)], fill=blue)
d.text((970,160), "zs", fill=blue, font=small)
d.line((900,500,900,430), fill=gray, width=4)
d.polygon([(900,420),(890,445),(910,445)], fill=gray)
d.text((920,440), "zu", fill=gray, font=small)
d.text((50, 710), "Fs = ks(zs-zu) + cs(vs-vu)       ms*zs_ddot = -Fs       mu*zu_ddot = Fs + kt(r-zu)", fill=navy, font=small)
img.save(out, optimize=True)
print(out)
